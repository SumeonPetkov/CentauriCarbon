//! Homing as the stock firmware does it (extras/safe_z_home.cpp,
//! extras/homing.cpp, kinematics/corexy.cpp):
//!
//! `G28`: heat the nozzle to 140 °C if it is colder than 138 °C; lift Z by
//! 10 mm if the Z endstop is already triggered; home X and Y **twice each**
//! (X, Y, X, Y) at 80 mm/s with 5000 mm/s² and a 20 mm back-off; move to
//! the Z home point (130, 130) at 300 mm/s; home Z at 8 mm/s, retract
//! 10 mm, home again at 3 mm/s, back off 10 mm. Final position X18 Y17.5,
//! Z = position_endstop + 10.
//!
//! `CUSTOM_ZERO` (replaces the first `G28` of a printed file): the same,
//! without the Z lift and with Z homed at the purge position (202, 264.5).

use cc_proto::dict::Arg;

use super::parse::{cpp_to_string, Abort, CmdResult, GCmd};
use super::Exec;
use crate::core::TrDispatch;
use crate::machine::{self, dsp};
use crate::Target;

const STEPPER_NAMES: [&str; 3] = ["stepper_x", "stepper_y", "stepper_z"];

// Define.h / mcu.h / mcu_io.h / toolhead.cpp
const HOMING_START_DELAY: f64 = 0.001;
const ENDSTOP_SAMPLE_TIME: f64 = 0.000015;
const ENDSTOP_SAMPLE_COUNT: i64 = 4;
const TRSYNC_SINGLE_MCU_TIMEOUT: f64 = 0.250;
/// `kin_flush_delay` = SDS_CHECK_TIME.
const KIN_FLUSH_DELAY: f64 = 0.008576076596340965;
const REASON_ENDSTOP_HIT: i64 = 1;
const REASON_COMMS_TIMEOUT: i64 = 2;
const REASON_HOST_REQUEST: i64 = 3;

impl Exec {
    /// `ToolHead::manual_move`: absent coordinates keep their value.
    pub(super) fn manual_move(&mut self, coord: [Option<f64>; 4], speed: f64) -> CmdResult {
        let mut pos = self.core().commanded;
        for i in 0..4 {
            if let Some(v) = coord[i] {
                pos[i] = v;
            }
        }
        self.wait_buffer()?;
        self.core().add_move(pos, speed, true)?;
        self.reset_last_position();
        Ok(())
    }

    /// `G28` / `CUSTOM_ZERO`.
    pub(super) fn cmd_g28(&mut self, g: &GCmd, custom: bool) -> CmdResult {
        if let Some(l) = self.core().hw.motion_lock() {
            return Err(Abort(l.into()));
        }
        // PrinterHoming::cmd_G28: with [strain_gauge] enable_z_home the load
        // cells home Z (the stock printer.cfg has it off: PG8 is used)
        let z_wanted = !["X", "Y"].iter().any(|a| !g.get_string(a, "").is_empty()) || !g.get_string("Z", "").is_empty();
        let sg_home = {
            let c = self.core();
            c.cfgs.printer.has_section("strain_gauge") && c.cfgs.printer.get_bool("strain_gauge", "enable_z_home", true)
        };
        if z_wanted && sg_home {
            return Err(Abort("G28: Z homing with the load cells ([strain_gauge] enable_z_home) is not available in ccos yet".into()));
        }
        let (temp, smoothed, prev_target, sz) = {
            let c = self.core();
            (c.m.safe_z.temp, c.heaters[0].smoothed, c.heaters[0].target, c.m.safe_z)
        };
        // 1. nozzle pre-heat (softens the filament for the Z touch)
        let preheat = smoothed < temp - 2.0;
        if preheat {
            self.single(&format!("M109 S{}", super::parse::cpp_double(temp)));
        }
        // 2. Z endstop already triggered: lift first (G28 only)
        if !custom && sz.z_hop == 0.0 {
            self.z_off_limit_action()?;
        }
        let present = |a: &str| !g.get_string(a, "").is_empty();
        let (mut x, mut y, mut z) = (present("X"), present("Y"), present("Z"));
        if !x && !y && !z {
            (x, y, z) = (true, true, true);
        }
        let r = (|| -> CmdResult {
            if x || y {
                self.home_axes(x, y)?;
            }
            if z {
                let homed = self.core().homed;
                if !homed[0] || !homed[1] {
                    self.home_axes(true, true)?;
                }
                if custom {
                    let cf = self.core().m.change_filament;
                    self.manual_move([Some(cf.pos_x), None, None, None], 80.0)?;
                    self.manual_move([None, Some(cf.pos_y), None, None], 80.0)?;
                } else {
                    self.manual_move([Some(sz.x), Some(sz.y), None, None], sz.speed)?;
                }
                self.home_rails(2)?;
            }
            Ok(())
        })();
        // 3. restore the nozzle target. The firmware passes a double to "%d"
        // here (undefined behaviour); ccos restores the previous target.
        if preheat {
            self.single(&format!("M109 S{}", prev_target as i64));
        }
        r
    }

    /// `PrinterHoming::cmd_G28` for X/Y: each axis homed twice, interleaved.
    fn home_axes(&mut self, x: bool, y: bool) -> CmdResult {
        self.wait_moves()?;
        let axes: Vec<usize> = [(x, 0), (y, 1)].iter().filter(|(on, _)| *on).map(|(_, a)| *a).collect();
        for _ in 0..2 {
            for &a in &axes {
                self.home_rails(a)?;
            }
        }
        Ok(())
    }

    /// `Z_AXIS_OFF_LIMIT_ACTION`: if the Z endstop reads triggered, take the
    /// current Z as homed and raise it by `z_lift` at `z_hop_speed`.
    pub(super) fn z_off_limit_action(&mut self) -> CmdResult {
        if self.core().hw.motion_lock().is_some() {
            return Ok(());
        }
        {
            let mut c = self.core();
            c.responses.retain(|(_, m)| m.name() != "endstop_state");
            let m = c.mcu(Target::Dsp)?;
            let msg = m.encode("endstop_query_state", &[("oid", Arg::Int(dsp::ES_Z as i64))]);
            m.queue_msg(msg);
            c.wake();
        }
        let pin = self.p.wait(2.0, "Z endstop state", |c| {
            let i = c.responses.iter().position(|(_, m)| m.name() == "endstop_state" && m.int("oid") == Some(dsp::ES_Z as i64))?;
            let (_, m) = c.responses.remove(i).unwrap();
            m.int("pin_value")
        })?;
        if pin != 0 {
            let (pos, sz) = {
                let c = self.core();
                (c.commanded, c.m.safe_z)
            };
            self.core().set_position_homed(pos, &[2]);
            self.manual_move([None, None, Some(sz.z_lift + pos[2]), None], sz.z_hop_speed)?;
        }
        Ok(())
    }

    /// `Homing::home_rails` for one axis (homing.cpp:88). A failed homing is
    /// not an error — the stock kinematics ignore the result and G28 goes on
    /// with the next axis — except on Z, which shuts the printer down.
    fn home_rails(&mut self, a: usize) -> CmdResult {
        let ax = self.core().m.axes[a].clone();
        self.wait_moves()?;
        // homing direction is negative on all axes: start "1.5 × range" away
        let (forcepos, movepos) = {
            let mut c = self.core();
            let (mut f, mut m) = (c.commanded, c.commanded);
            f[a] = ax.position_endstop + 1.5 * (ax.max - ax.position_endstop);
            m[a] = ax.position_endstop;
            c.set_position_homed(f, &[a]);
            (f, m)
        };
        let prev_accel = self.core().planner.lim.max_accel;
        if let Some(acc) = ax.homing_accel {
            self.single(&format!("M204 S{acc:.2}"));
        }
        if let Some(hc) = ax.homing_current {
            self.single(&format!("SET_TMC_CURRENT_{} CURRENT={}", STEPPER_NAMES[a], cpp_to_string(hc)));
        }
        let mut ok = self.homing_move(a, movepos, ax.homing_speed)?;
        let move_d = (forcepos[a] - movepos[a]).abs();
        let retract_to = |dist: f64| {
            let mut p = movepos;
            p[a] = movepos[a] + (forcepos[a] - movepos[a]) * (dist / move_d).min(1.0);
            p
        };
        if ax.retract_dist.abs() > 1e-15 {
            // retract, pretend to be twice as far, home again slower. After a
            // failed first pass the axis is unhomed and both moves are dropped.
            let retractpos = retract_to(ax.retract_dist);
            self.core().add_move(retractpos, ax.retract_speed, true)?;
            self.wait_moves()?;
            let mut force2 = retractpos;
            force2[a] += retractpos[a] - movepos[a];
            self.core().set_position(force2);
            ok = self.homing_move(a, movepos, ax.second_homing_speed)?;
        }
        if ax.force_retract.abs() > 1e-15 {
            let retractpos = retract_to(ax.force_retract);
            self.core().add_move(retractpos, ax.retract_speed, true)?;
            self.wait_moves()?;
            self.core().set_position(retractpos);
        }
        // homing:home_rails_end → gcode_move
        self.reset_last_position();
        self.base[a] = self.homing_position[a];
        if ax.homing_accel.is_some() {
            self.single(&format!("M204 S{prev_accel:.2}"));
        }
        if ax.homing_current.is_some() {
            self.single(&format!("SET_TMC_CURRENT_{}", STEPPER_NAMES[a]));
        }
        if !ok && a == 2 {
            self.invoke_shutdown("Z-axis zeroing failed");
        }
        Ok(())
    }

    /// `HomingMove::homing_move` (homing.cpp:522) with the stock trsync
    /// protocol: `trsync_start` reporting every 0.1 s with a 0.25 s timeout
    /// that trdispatch keeps extending, `endstop_home` sampling once per
    /// motor step of the move (`calc_endstop_rate`), the move dripped out,
    /// then `endstop_home` off and `trsync_trigger` to read the outcome.
    /// Ok(false) (`is_succ`) when the endstop did not trigger: the axes are
    /// then unhomed (`CoreXYKinematics::motor_off`, motors stay enabled).
    fn homing_move(&mut self, a: usize, movepos: [f64; 4], speed: f64) -> Result<bool, Abort> {
        let ax = self.core().m.axes[a].clone();
        let ts = dsp::TRSYNC as i64;
        self.tmc_homing(a, true); // homing:homing_move_begin
        let end_t = {
            let mut c = self.core();
            c.ui.set_status(format!("Homing {}…", ax.name));
            c.flush_moves();
            let start = c.commanded;
            let print_time = c.last_move_time();
            // calc_endstop_rate: one sample burst per motor step
            let d: Vec<f64> = (0..3).map(|i| movepos[i] - start[i]).collect();
            let move_t = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() / speed;
            let mut max_steps = 0.0f64;
            for &s in ax.steppers {
                let def = &c.m.steppers[s];
                let pos = |p: &[f64; 4]| (0..4).map(|k| def.kin[k] * p[k]).sum::<f64>();
                max_steps = max_steps.max((pos(&start) - pos(&movepos)) / def.step_dist);
            }
            let rest_time = if max_steps <= 0.0 { 0.001 } else { move_t / max_steps };
            c.responses.retain(|(_, m)| m.name() != "trsync_state");
            let m = c.mcu(Target::Dsp)?;
            let clock = m.clock.clock(print_time);
            let rest_ticks = m.clock.clock(print_time + rest_time) - clock;
            let sample_ticks = m.clock.ticks(ENDSTOP_SAMPLE_TIME);
            // MCU_trsync::start(print_time, TRSYNC_SINGLE_MCU_TIMEOUT)
            let expire_ticks = m.clock.ticks(TRSYNC_SINGLE_MCU_TIMEOUT) as u64;
            let expire_clock = clock + expire_ticks;
            let report_ticks = m.clock.ticks(TRSYNC_SINGLE_MCU_TIMEOUT * 0.4);
            let min_extend_ticks = m.clock.ticks(TRSYNC_SINGLE_MCU_TIMEOUT * 0.4 * 0.8) as u64;
            let mut msgs = vec![m.encode(
                "trsync_start",
                &[("oid", Arg::Int(ts)), ("report_clock", Arg::Int(clock as u32 as i64)), ("report_ticks", Arg::Int(report_ticks as i64)), ("expire_reason", Arg::Int(REASON_COMMS_TIMEOUT))],
            )];
            for &s in ax.steppers {
                let oid = machine::dsp::STEP_A as i64 + s as i64;
                msgs.push(m.encode("stepper_stop_on_trigger", &[("oid", Arg::Int(oid)), ("trsync_oid", Arg::Int(ts))]));
            }
            msgs.push(m.encode("trsync_set_timeout", &[("oid", Arg::Int(ts)), ("clock", Arg::Int(expire_clock as u32 as i64))]));
            msgs.push(m.encode(
                "endstop_home",
                &[
                    ("oid", Arg::Int(ax.endstop_oid as i64)),
                    ("clock", Arg::Int(clock as u32 as i64)),
                    ("sample_ticks", Arg::Int(sample_ticks as i64)),
                    ("sample_count", Arg::Int(ENDSTOP_SAMPLE_COUNT)),
                    ("rest_ticks", Arg::Int(rest_ticks as u32 as i64)),
                    // triggered ^ invert ("!PG4" triggers low)
                    ("pin_value", Arg::Int((!ax.endstop.invert) as i64)),
                    ("trsync_oid", Arg::Int(ts)),
                    ("trigger_reason", Arg::Int(REASON_ENDSTOP_HIT)),
                ],
            ));
            for msg in msgs {
                m.queue_msg(msg);
            }
            c.trdispatch = Some(TrDispatch {
                target: Target::Dsp,
                oid: dsp::TRSYNC,
                can_trigger: true,
                reason: REASON_HOST_REQUEST as u8,
                expire_clock,
                expire_ticks,
                min_extend_ticks,
                triggered: None,
            });
            c.homing = true;
            c.dwell(HOMING_START_DELAY);
            // drip_move
            c.dwell(KIN_FLUSH_DELAY);
            let r = c.add_move(movepos, speed, true);
            c.flush_moves();
            if let Err(e) = r {
                c.homing = false;
                c.trdispatch = None;
                return Err(e.into());
            }
            c.wake();
            c.planner.print_time
        };
        // drip_move returns on a trigger, or once the last segment of the
        // move has been generated (its end is less than the drip margin away)
        let margin = machine::GEN_AHEAD_HOMING.1;
        let wait = (end_t - crate::clock::now()).max(0.0) + 5.0;
        self.p.wait(wait, "homing move", |c| (c.trdispatch.as_ref().is_some_and(|t| t.triggered.is_some()) || crate::clock::now() >= end_t - margin).then_some(()))?;
        // home_wait: endstop off, trdispatch_stop, trsync stop
        {
            let mut c = self.core();
            c.stop_motion();
            c.homing = false;
            c.trdispatch = None;
            let m = c.mcu(Target::Dsp)?;
            for &s in ax.steppers {
                m.withdraw(machine::dsp::STEP_A + s as u8);
            }
            let msg = m.encode(
                "endstop_home",
                &[
                    ("oid", Arg::Int(ax.endstop_oid as i64)),
                    ("clock", Arg::Int(0)),
                    ("sample_ticks", Arg::Int(0)),
                    ("sample_count", Arg::Int(0)),
                    ("rest_ticks", Arg::Int(0)),
                    ("pin_value", Arg::Int(0)),
                    ("trsync_oid", Arg::Int(0)),
                    ("trigger_reason", Arg::Int(0)),
                ],
            );
            m.queue_msg(msg);
            c.wake();
        }
        let st = super::send_with_response(&self.p, Target::Dsp, "trsync_trigger", &[("oid", Arg::Int(ts)), ("reason", Arg::Int(REASON_HOST_REQUEST))], "trsync_state", ts)?;
        let ok = st.int("trigger_reason") == Some(REASON_ENDSTOP_HIT);
        // note_homing_end: where the MCU stopped each stepper
        for &s in ax.steppers {
            let oid = machine::dsp::STEP_A as i64 + s as i64;
            let r = super::send_with_response(&self.p, Target::Dsp, "stepper_get_position", &[("oid", Arg::Int(oid))], "stepper_position", oid)?;
            self.core().steppers[s].set_mcu_position(r.int("pos").unwrap_or(0));
        }
        self.core().set_position(movepos);
        self.tmc_homing(a, false); // homing:homing_move_end
        if !ok {
            super::console(&format!("Failed to home {} (trigger reason {})", ax.name.to_ascii_lowercase(), st.int("trigger_reason").unwrap_or(0)));
            self.core().homed = [false; 3];
        }
        Ok(ok)
    }

    /// `M8233 S<zo> P<apply>` core: the user Z offset and the Z endstop
    /// coordinate (E = position_endstop_extra − zo + fix_z_offset).
    pub(super) fn apply_z_offset(&mut self, zo: f64, apply: bool, save: bool) {
        let mut c = self.core();
        let extra = c.m.axes[2].position_endstop_extra;
        let material = c.cfgs.printer.get_string("bed_mesh", "platform_material", "standard");
        let fix_key = if material == "standard" { "standard_fix_z_offset" } else { "enhancement_fix_z_offset" };
        let fix = -c.cfgs.unmodifiable.get_double("strain_gauge", fix_key, 0.0);
        c.cfgs.printer.set_double("probe", "z_offset", zo);
        c.cfgs.printer.set_double("stepper_z", "position_endstop", extra - zo);
        let key = if material == "standard" { "standard_z_offset" } else { "enhancement_z_offset" };
        c.cfgs.printer.set_double("bed_mesh", key, zo);
        if apply {
            c.m.axes[2].position_endstop = -zo + extra + fix;
        }
        if save {
            c.cfgs.save_user("bed_mesh", &["platform_material", "standard_z_offset", "enhancement_z_offset"]);
            c.cfgs.save_printer();
            c.cfgs.save_user("probe", &["z_offset"]);
            c.cfgs.save_user("stepper_z", &["position_endstop", "position_endstop_extra"]);
        }
        drop(c);
        super::console(&format!("cmd_M8233 probe z_offset : {}", super::parse::cpp_double(zo)));
    }
}
