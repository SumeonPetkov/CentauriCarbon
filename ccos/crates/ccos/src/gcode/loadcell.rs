//! Load-cell probing as the stock firmware does it (extras/strain_gauge.cpp,
//! extras/hx711s.cpp, extras/dirzctl.cpp): `STRAINGAUGE_Z_HOME`,
//! `STRAINGAUGE_TEST`, `DIRZCTL`, `READ_HX711` and Z homing with
//! `[strain_gauge] enable_z_home`.
//!
//! A probe (`probe_by_step`): the load-cell board is tared
//! (`calibration_sample`) and armed (`sg_probe_check … cmd=1`). The DSP
//! steps Z down on its own (`run_dirzctl`), outside the motion queue, so the
//! host's idea of Z does not change. `result_dirzctl` reports the first
//! step and the stop with their ticks; `sg_resp` reports the press with the
//! tick of its knee. Z at that tick is interpolated between the two
//! reports. Then the same number of steps go back up.
//!
//! Kept stock bugs:
//! * the step counts in `result_dirzctl` are 16 bits, so a first touch
//!   more than ~94 mm (37664 steps) away wraps the arithmetic. It then looks
//!   like a 3 mm+ miss, which is reported as "No response from MCU" and
//!   shuts down.
//! * `STRAINGAUGE_Z_HOME` / G28 with cool-down end with `M106 S0.00`, so the
//!   part fan is turned off.
//!
//! Deviations (undefined behaviour or hangs in the C++):
//! * The busy-wait loops that throw `MCUException` ("No response from MCU")
//!   shut the printer down here with that reason.
//! * `READ_HX711` waits forever for samples nothing collects. Here it sends
//!   the query and returns.
//! * `cal_min_z` spins without checking for a shutdown. Here a shutdown ends
//!   the wait.
//! * `run_G28_Z` with `use_probe_by_step` has no return value. Here it gives
//!   whether Z homed (G28 ignores it, as the stock one does).
//! * `use_probe_by_step: 0` (homing on the endstop via `homing_z_move`) is
//!   not available.

use cc_proto::dict::Arg;

use super::parse::{cpp_double, Abort, CmdResult, GCmd};
use super::Exec;
use crate::loadcell::StrainGaugeCfg;
use crate::machine::{dsp, strain_gauge};
use crate::Target;

/// `StrainGaugeVal`: what the last probe found.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct SgVals {
    pub out_index: i32,
    pub out_val_mm: f64,
    pub end_z_mm: f64,
    /// Bed-mesh taps since the last load-cell Z homing (a shake every 12).
    pub g29_cnt: i32,
    /// `_probe_times` repeats (diagnostics).
    pub re_probe_cnt: i32,
    /// `CHECK_BED_MESH` makes the next `PRTOUCH_READY` do nothing.
    pub jump_probe_ready: bool,
}

/// `ck_sys_sta`: neither the load-cell board nor the DSP is shut down or
/// gone.
fn sys_ok(c: &crate::core::Core) -> bool {
    c.shutdown.is_none() && !c.exit && c.mcus[Target::StrainGauge as usize].is_some() && c.mcus[Target::Dsp as usize].is_some()
}

/// One `usleep(1000)` round of the stock wait loops, and how many rounds
/// pass between resends.
const LOOP: f64 = 0.001;
const RESEND_LOOPS: u32 = 2345;
const RESENDS: u32 = 50;

/// `(index, z, ok)` from `probe_by_step`.
type Probe = (i32, f64, bool);

impl Exec {
    pub(super) fn sg_cfg(&self) -> Option<StrainGaugeCfg> {
        self.core().m.strain_gauge
    }

    /// The `[strain_gauge]` commands only exist with that section (and
    /// `[hx711s]`, `[dirzctl]`), like their `register_command`.
    pub(super) fn loadcell_handles(&self, key: &str) -> bool {
        let c = self.core();
        match key {
            "STRAINGAUGE_Z_HOME" | "STRAINGAUGE_TEST" => c.m.strain_gauge.is_some(),
            "DIRZCTL" => c.m.dirzctl_step_base.is_some(),
            "READ_HX711" => c.m.hx711s.is_some(),
            _ => false,
        }
    }

    pub(super) fn cmd_loadcell(&mut self, key: &str, g: &GCmd) -> CmdResult {
        if let Some(l) = self.core().hw.motion_lock() {
            return Err(Abort(l.into()));
        }
        match key {
            "STRAINGAUGE_Z_HOME" => self.run_probe(false, false, false),
            "STRAINGAUGE_TEST" => self.cmd_straingauge_test(g),
            "DIRZCTL" => self.cmd_dirzctl(g),
            _ => self.cmd_read_hx711(g),
        }
    }

    // ---- the MCUs ----

    /// `ck_sys_sta`: neither the load-cell board nor the DSP is shut down
    /// or gone.
    pub(super) fn ck_sys_sta(&self) -> bool {
        sys_ok(&self.core())
    }

    pub(super) fn sg_send(&self, target: Target, name: &str, args: &[(&str, Arg<'_>)]) -> CmdResult {
        let mut c = self.core();
        let m = c.mcu(target)?;
        let msg = m.encode(name, args);
        m.queue_msg(msg);
        c.wake();
        Ok(())
    }

    /// `MCUException(name, "No response from MCU")`, which the stock G-code
    /// loop does not catch: the printer shuts down.
    fn sg_no_response(&self) -> Abort {
        let why = "strain_gauge_mcu: No response from MCU".to_string();
        self.core().emergency(why.clone());
        Abort(why)
    }

    /// Sleep like `usleep`, ending early on a shutdown.
    pub(super) fn sg_sleep(&self, secs: f64) -> CmdResult {
        match self.p.wait(secs, "load cell", |c| (c.shutdown.is_some() || c.exit).then_some(())) {
            Ok(()) => Err(Abort(self.core().shutdown.clone().unwrap_or_else(|| "exit".into()))),
            Err(_) => Ok(()),
        }
    }

    /// The stock "send, poll every 1 ms, resend every 2345 polls, give up
    /// after 50 resends" loop. `first` = the sleep before polling.
    fn sg_send_until(&self, name: &str, args: &[(&str, Arg<'_>)], first: f64, done: impl Fn(&crate::core::Core) -> bool) -> CmdResult {
        self.sg_send(Target::StrainGauge, name, args)?;
        self.sg_sleep(first)?;
        for resend in 0..=RESENDS {
            if resend > 0 {
                self.sg_send(Target::StrainGauge, name, args)?;
            }
            let r = self.p.wait(LOOP * RESEND_LOOPS as f64, name, |c| done(c).then_some(()));
            if r.is_ok() {
                return Ok(());
            }
            if let Some(s) = self.core().shutdown.clone() {
                return Err(Abort(s));
            }
        }
        Err(self.sg_no_response())
    }

    /// `HX711S::ProbeCheckTriggerStart` (always with the ack).
    pub(super) fn trigger_start(&self, x: i32, y: i32, z: i32) -> CmdResult {
        self.sg_sleep(0.0005)?;
        {
            let c = self.core();
            if !sys_ok(&c) || c.sg.is_calibration & 0x80 == 0 {
                let line = format!("ProbeCheckTriggerStart Error: m_is_calibration({})", c.sg.is_calibration);
                drop(c);
                self.log(line);
                return Ok(());
            }
        }
        {
            let mut c = self.core();
            c.sg.is_trigger = 0;
            c.sg.trigger_timestamp = 0.0;
            c.sg.probe_cmd_response = 0;
        }
        let args = [("oid", Arg::Int(strain_gauge::HX711S as i64)), ("x", Arg::Int(x as i64)), ("y", Arg::Int(y as i64)), ("z", Arg::Int(z as i64)), ("cmd", Arg::Int(1))];
        // 5 ms so an older reply does not count
        self.sg_send_until("sg_probe_check", &args, 0.005, |c| c.sg.probe_cmd_response == 1)
    }

    /// `HX711S::ProbeCheckTriggerStop`.
    pub(super) fn trigger_stop(&self, ack: bool) -> CmdResult {
        if !self.ck_sys_sta() {
            return Ok(());
        }
        {
            let mut c = self.core();
            c.sg.probe_cmd_response = 0;
            c.sg.is_trigger = 0;
        }
        let args = [("oid", Arg::Int(strain_gauge::HX711S as i64)), ("x", Arg::Int(0)), ("y", Arg::Int(0)), ("z", Arg::Int(0)), ("cmd", Arg::Int(2))];
        if !ack {
            return self.sg_send(Target::StrainGauge, "sg_probe_check", &args);
        }
        self.sg_send_until("sg_probe_check", &args, LOOP, |c| c.sg.probe_cmd_response == 2)
    }

    /// `HX711S::CalibrationStart(times_read, false)`: tare the cells and wait
    /// for the result. Ok(false) when it could not start.
    pub(super) fn calibration_start(&self, times_read: i64) -> Result<bool, Abort> {
        if !self.ck_sys_sta() {
            return Ok(false);
        }
        {
            let mut c = self.core();
            c.sg.probe_calibration_response = 0;
            c.sg.is_calibration = 0;
            c.sg.is_trigger = 0;
        }
        let args = [("oid", Arg::Int(strain_gauge::HX711S as i64)), ("times_read", Arg::Int(times_read as u16 as i64))];
        self.sg_send_until("calibration_sample", &args, 0.1, |c| c.sg.probe_calibration_response != 0)?;
        // up to 100000 polls for the tare
        match self.p.wait(100_000.0 * LOOP, "load-cell tare", |c| (c.sg.is_calibration & 0x80 != 0).then_some(())) {
            Ok(()) => Ok(true),
            Err(_) if self.ck_sys_sta() => Err(self.sg_no_response()),
            // the stock loop ends without a return value (UB): not tared
            Err(_) => Ok(false),
        }
    }

    /// `DirZCtl::check_and_run`.
    pub(super) fn dirz_run(&mut self, direct: i64, step_us: u32, step_cnt: u32, wait_finish: bool) -> CmdResult {
        if !self.ck_sys_sta() {
            return Ok(());
        }
        let enabled = {
            let mut c = self.core();
            if step_cnt != 0 {
                c.sg.dirzctl.clear();
            }
            c.enabled[2]
        };
        if !enabled {
            // motor_debug_enable
            self.core().stepper_enable(2, true)?;
        }
        let args = [("oid", Arg::Int(dsp::DIRZCTL as i64)), ("direct", Arg::Int(direct)), ("step_us", Arg::Int(step_us as i64)), ("step_cnt", Arg::Int(step_cnt as i64))];
        self.sg_send(Target::Dsp, "run_dirzctl", &args)?;
        if wait_finish {
            let limit = 1.5 / 1000.0 / 1000.0 * step_us as f64 * step_cnt as f64;
            let _ = self.p.wait(limit, "direct Z move", |c| (c.sg.dirzctl.len() == 2).then_some(()));
        }
        Ok(())
    }

    /// `(Z step distance × step_base)`: one dirzctl step in mm.
    pub(super) fn dirz_step_mm(&self) -> f64 {
        let c = self.core();
        c.m.steppers[2].step_dist * c.m.dirzctl_step_base.unwrap_or(2.0)
    }

    // ---- probing ----

    /// `cal_min_z`: where Z was when the load cells were pressed, from the
    /// two `result_dirzctl` reports. Returns (steps up to that point, steps
    /// moved, ok).
    fn cal_min_z(&mut self, start_z: f64) -> Result<(i32, i32, bool), Abort> {
        // stock: spins until both reports are in
        let (r0, r1) = self.p.wait(f64::INFINITY, "Z stop report", |c| (c.sg.dirzctl.len() == 2).then(|| (c.sg.dirzctl[0], c.sg.dirzctl[1])))?;
        // uint32 arithmetic on the 16-bit counts
        let moved = r0.step.wrapping_sub(r1.step).wrapping_add(1);
        let (trig, trigger_time) = {
            let c = self.core();
            (c.sg.is_trigger, c.sg.trigger_timestamp)
        };
        if trig == 0 {
            return Ok((moved as i32, moved as i32, false));
        }
        let step_mm = self.dirz_step_mm();
        let end_z = start_z - moved as f64 * step_mm;
        self.sg.end_z_mm = end_z;
        // _get_linear2 on (time, z)
        self.sg.out_val_mm = if (r0.time - r1.time).abs() < 0.001 {
            0.0
        } else {
            let a = (end_z - start_z) / (r1.time - r0.time);
            let b = start_z - r0.time * a;
            a * trigger_time + b
        };
        let delta = self.sg.out_val_mm - end_z;
        super::console(&format!("call_min_z, re_probe_cnt=0, out_val_mm={} end_z={} delta: {}", cpp_double(self.sg.out_val_mm), cpp_double(end_z), cpp_double(delta)));
        let up_min = ((delta / step_mm) as i32).max(0);
        if delta.abs() > 3.0 {
            self.log(format!("cal_min_z m_out_val_mm - end_z:{delta:.6}  > 3"));
            self.trigger_stop(true)?;
            return Err(self.sg_no_response());
        }
        Ok((up_min, moved as i32, true))
    }

    /// `probe_by_step`: probe down at most `min_dis` mm at `speed` from
    /// `rdy` (where the host thinks Z is); `up_after` goes back up after.
    pub(super) fn probe_by_step(&mut self, rdy: [f64; 4], speed: f64, min_dis: f64, up_after: bool) -> Result<Probe, Abort> {
        let step_mm = self.dirz_step_mm();
        let step_cnt = (min_dis / step_mm) as i32;
        let step_us = (((min_dis / speed) * 1000.0 * 1000.0) / step_cnt as f64) as i32;
        self.trigger_start(rdy[0] as i32, rdy[1] as i32, rdy[2] as i32)?;
        self.dirz_run(0, step_us as u32, step_cnt as u32, false)?;
        super::console(&format!(
            "PROBE_BY_STEP x= {} y= {} z= {} speed_mm= {} step_us= {step_us} step_cnt= {step_cnt}",
            cpp_double(rdy[0]),
            cpp_double(rdy[1]),
            cpp_double(rdy[2]),
            cpp_double(speed)
        ));
        enum Ev {
            Done,
            Pressed,
            Down,
        }
        loop {
            let ev = self.p.wait(f64::INFINITY, "load-cell press", |c| {
                if !sys_ok(c) {
                    Some(Ev::Down)
                } else if c.sg.dirzctl.len() == 2 {
                    Some(Ev::Done)
                } else if c.sg.is_trigger > 0 {
                    Some(Ev::Pressed)
                } else {
                    None
                }
            });
            match ev {
                // the loop ends: the stock returns the last values as a success
                Ok(Ev::Down) | Err(_) => return Ok((self.sg.out_index, self.sg.out_val_mm, true)),
                Ok(Ev::Done) => {
                    self.log("axis z wait home timeout");
                    if up_after {
                        self.dirz_run(1, (step_us / 3) as u32, step_cnt as u32, true)?;
                    }
                    self.trigger_stop(true)?;
                    return Ok((0, 0.0, false));
                }
                Ok(Ev::Pressed) => break,
            }
        }
        super::console("probe_by_step trigger!!!");
        self.dirz_run(0, 0, 0, false)?;
        self.sg_sleep(0.003)?;
        let (_, up_all, ok) = self.cal_min_z(rdy[2])?;
        if up_after {
            super::console(&format!("up_after : {}", cpp_double(up_all as f64 * self.core().m.steppers[2].step_dist)));
            self.dirz_run(1, (step_us / 3) as u32, up_all as u32, true)?;
        }
        if !ok {
            self.log("cal_min_z result error!!!!!!!!!!!!!!");
            self.trigger_stop(true)?;
            return Ok((0, 0.0, false));
        }
        let test = self.core().m.hx711s.as_ref().map_or(0, |h| h.enable_test);
        if test == 1 || test == 2 {
            self.sg_sleep(1.0)?;
            for _ in 0..10 {
                self.trigger_stop(true)?;
                self.sg_sleep(1.0)?;
            }
        } else {
            self.trigger_stop(true)?;
        }
        Ok((self.sg.out_index, self.sg.out_val_mm, ok))
    }

    /// `_move`: manual_move then wait_moves.
    pub(super) fn sg_move(&mut self, pos: [Option<f64>; 4], speed: f64) -> CmdResult {
        self.manual_move(pos, speed)?;
        self.wait_moves()
    }

    /// `_set_hot_temps(temp, fan, wait)`.
    fn set_hot_temps(&mut self, temp: f64, fan: f64, wait: bool) {
        self.single(&format!("M106 S{fan:.2}"));
        self.single(&format!("{} S{temp:.2}", if wait { "M109" } else { "M104" }));
    }

    /// `shake_motor`: `cnt` small Z wiggles at the toolhead's max velocity.
    pub(super) fn shake_motor(&mut self, cnt: i32, cfg: &StrainGaugeCfg) -> CmdResult {
        let (pos, v) = {
            let c = self.core();
            (c.commanded, c.planner.lim.max_velocity)
        };
        for _ in 0..cnt {
            self.manual_move([Some(pos[0]), Some(pos[1]), Some(pos[2] + cfg.shake_range), None], v)?;
            self.manual_move([Some(pos[0]), Some(pos[1]), Some(pos[2]), None], v)?;
        }
        self.wait_moves()?;
        self.sg_move([Some(pos[0]), Some(pos[1]), Some(pos[2]), Some(pos[3])], cfg.g29_xy_speed)
    }

    /// The common start of `run_probe` / `run_G28_Z`: cool the nozzle, shake,
    /// tare. Returns the extruder target to restore.
    fn probe_prepare(&mut self, cfg: &StrainGaugeCfg, shake: bool, cool: bool) -> Result<f64, Abort> {
        self.wait_moves()?;
        let (target, smoothed) = {
            let c = self.core();
            (c.heaters[0].target, c.heaters[0].smoothed)
        };
        if cfg.g28_wait_cool_down && smoothed > cfg.hot_min_temp + 5.0 && cool {
            super::console(&format!("G28_Z: Wait for Nozzle to cool down[{target:.2} -> {:.2}]...", cfg.hot_min_temp));
            self.set_hot_temps(cfg.hot_min_temp, 255.0, true);
            self.set_hot_temps(cfg.hot_min_temp, 0.0, false);
        }
        if shake {
            self.shake_motor(20, cfg)?;
        }
        self.calibration_start(30)?;
        Ok(target)
    }

    /// The first, fast touch from wherever Z is, taken as Z = 0, then 2 mm up.
    /// Returns the position at Z = 2.
    fn probe_first_touch(&mut self, cfg: &StrainGaugeCfg, start: [f64; 4]) -> Result<[f64; 4], Abort> {
        self.probe_by_step(start, cfg.g28_sta0_speed, cfg.max_z, false)?;
        self.core().set_position_homed([start[0], start[1], 0.0, start[3]], &[2]);
        self.reset_last_position();
        self.sg_move([Some(start[0]), Some(start[1]), Some(2.0), Some(start[3])], 2.0 * cfg.g28_sta1_speed)?;
        Ok(self.core().commanded)
    }

    /// `StrainGaugeWrapper::run_probe` (`STRAINGAUGE_Z_HOME`).
    fn run_probe(&mut self, shake: bool, force_retract: bool, cool: bool) -> CmdResult {
        let Some(cfg) = self.sg_cfg() else { return Ok(()) };
        let target = self.probe_prepare(&cfg, shake, cool)?;
        let mut now_pos = self.core().commanded;
        let sta0 = self.probe_first_touch(&cfg, now_pos)?;
        if !force_retract {
            super::console("start wipe nozzle !!! ");
            for _ in 0..2 {
                self.probe_by_step(sta0, cfg.g28_sta1_speed, cfg.max_z, true)?;
            }
            self.probe_by_step(sta0, cfg.g28_sta1_speed, cfg.max_z, false)?;
            // the host still thinks Z is at 2: this makes it 0
            now_pos[2] = 0.0;
            self.core().set_position_homed(now_pos, &[2]);
            self.reset_last_position();
            super::console("end wipe nozzle !!! ");
        }
        if cfg.g28_wait_cool_down && cool {
            super::console(&format!("G28_Z: Wait for Nozzle to recovery[{:.2} -> {target:.2}]...", cfg.hot_min_temp));
            self.set_hot_temps(target, 0.0, target > cfg.hot_min_temp);
        }
        Ok(())
    }

    /// `StrainGaugeWrapper::run_G28_Z` with `use_probe_by_step`, as G28 calls
    /// it (shake, force retract, cool down). Ok(is_succ).
    pub(super) fn run_g28_z(&mut self) -> Result<bool, Abort> {
        let Some(cfg) = self.sg_cfg() else { return Ok(false) };
        if !cfg.use_probe_by_step {
            return Err(Abort("G28: Z homing with the load cells on the endstop ([strain_gauge] use_probe_by_step: 0) is not available in ccos".into()));
        }
        let (shake, force_retract, cool) = (true, true, true);
        super::console(&format!("--------------run_G28_z use probe_by_step--------------- need_force_retract:{}", force_retract as i32));
        self.sg.g29_cnt = 0;
        let position_endstop = self.core().m.axes[2].position_endstop;
        let target = self.probe_prepare(&cfg, shake, cool)?;
        let now_pos = self.core().commanded;
        let sta0 = self.probe_first_touch(&cfg, now_pos)?;
        let mut is_succ = false;
        if !force_retract {
            super::console("start wipe nozzle !!! ");
            for _ in 0..2 {
                self.calibration_start(30)?;
                self.probe_by_step(sta0, cfg.g28_sta1_speed, cfg.max_z, true)?;
            }
            self.calibration_start(30)?;
            self.probe_by_step(sta0, cfg.g28_sta1_speed, cfg.max_z, false)?;
            super::console("end wipe nozzle !!! ");
            is_succ = true;
        } else {
            const G28_TIMES: usize = 10;
            let mut error_times = 0;
            while error_times < 3 && !is_succ {
                if error_times > 0 {
                    self.calibration_start(30)?;
                    self.probe_by_step(sta0, cfg.g28_sta0_speed, cfg.max_z, false)?;
                    self.core().set_position_homed([sta0[0], sta0[1], 0.0, sta0[3]], &[2]);
                    self.reset_last_position();
                    self.sg_move([Some(sta0[0]), Some(sta0[1]), Some(2.0), Some(sta0[3])], 2.0 * cfg.g28_sta1_speed)?;
                }
                let mut out_mms: Vec<f64> = Vec::new();
                let mut speed = cfg.g28_sta1_speed;
                for i in 0..G28_TIMES {
                    self.calibration_start(30)?;
                    let (_, o_mm, ok) = self.probe_by_step(sta0, speed, cfg.g28_max_try, true)?;
                    if !ok {
                        error_times += 1;
                        break;
                    }
                    super::console(&format!("o_mm0 : {} i : {i}", cpp_double(o_mm)));
                    out_mms.push(o_mm);
                    let n = out_mms.len();
                    if n >= 2 {
                        if (out_mms[n - 1] - out_mms[n - 2]).abs() < cfg.g28_max_err && out_mms[n - 1].abs() < 1.0 {
                            is_succ = true;
                            let z = (out_mms[n - 1] + out_mms[n - 2]) / 2.0;
                            let new_pos = [sta0[0], sta0[1], sta0[2] - z - cfg.self_z_offset + position_endstop, sta0[3]];
                            self.core().set_position_homed(new_pos, &[2]);
                            self.reset_last_position();
                            break;
                        }
                        speed = (speed * 0.5).max(1.0);
                    }
                }
                if out_mms.len() >= G28_TIMES {
                    self.log("homing z failure ! The number of attempts has been exhausted");
                    break;
                }
            }
            if error_times >= 3 {
                is_succ = false;
                self.log("homing z failure !");
                super::console("error ! Homing Z failure, During zeroing, please place the machine on a stable platform and do not touch the hot bed. ");
            }
            self.sg_move([Some(now_pos[0]), Some(now_pos[1]), Some(10.0), Some(now_pos[3])], 2.0 * cfg.g28_sta1_speed)?;
        }
        if cfg.g28_wait_cool_down && cool {
            super::console(&format!("G28_Z: Wait for Nozzle to recovery[{:.2} -> {target:.2}]...", cfg.hot_min_temp));
            self.set_hot_temps(target, 0.0, target > cfg.hot_min_temp);
        }
        Ok(is_succ)
    }

    // ---- commands ----

    /// `STRAINGAUGE_TEST [X] [Y] [Z] [SPEED] [MIN_HOLD] [MAX_HOLD]`: move to
    /// the point and probe up to 50 mm down, then back up.
    fn cmd_straingauge_test(&mut self, g: &GCmd) -> CmdResult {
        let Some(cfg) = self.sg_cfg() else { return Ok(()) };
        let pos = self.core().commanded;
        let x = g.f("X", pos[0])?;
        let y = g.f("Y", pos[1])?;
        let z = g.f("Z", pos[2])?;
        let speed = g.f("SPEED", 1.0)?;
        // read but unused (the stock probe ignores the holds)
        g.i("MIN_HOLD", cfg.g28_min_hold)?;
        g.i("MAX_HOLD", cfg.g28_max_hold)?;
        self.sg_move([Some(x), Some(y), Some(z), None], cfg.g29_xy_speed)?;
        let e = self.core().commanded[3];
        self.probe_by_step([x, y, z, e], speed, 50.0, true)?;
        Ok(())
    }

    /// `DIRZCTL [INDEX] [DIRECT] [STEP_US] [STEP_CNT]`: step Z directly (no
    /// waiting; the host position does not change).
    fn cmd_dirzctl(&mut self, g: &GCmd) -> CmdResult {
        let n = self.core().m.steppers.iter().filter(|s| s.kin[2] != 0.0).count() as i64;
        g.get_int("INDEX", n, Some(0), Some(n))?;
        let direct = g.get_int("DIRECT", 1, Some(0), Some(1))?;
        let step_us = g.get_int("STEP_US", 1500, Some(4), Some(1000 * 100))?;
        let step_cnt = g.get_int("STEP_CNT", 256, Some(0), Some(10000))?;
        self.dirz_run(direct, step_us as u32, step_cnt as u32, false)
    }

    /// `READ_HX711 [C]`: starts the board's sampling. The stock command then
    /// waits for samples nothing collects (it never returns); here it returns.
    fn cmd_read_hx711(&mut self, g: &GCmd) -> CmdResult {
        let cnt = g.get_int("C", 1, Some(1), Some(9999))?;
        if !self.ck_sys_sta() {
            return Ok(());
        }
        self.sg_send(Target::StrainGauge, "query_hx711s", &[("oid", Arg::Int(strain_gauge::HX711S as i64)), ("times_read", Arg::Int(cnt as u16 as i64))])?;
        self.respond("READ_HX711: the load-cell board does not send samples back (the stock command waits here forever)");
        Ok(())
    }
}

