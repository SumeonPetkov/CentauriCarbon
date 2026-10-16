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
use crate::machine::{self, dsp};
use crate::Target;

const STEPPER_NAMES: [&str; 3] = ["stepper_x", "stepper_y", "stepper_z"];

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

    /// `Homing::home_rails` for one axis (homing.cpp:87).
    fn home_rails(&mut self, a: usize) -> CmdResult {
        let ax = self.core().m.axes[a].clone();
        // homing direction is negative on all axes: start "1.5 × range" away
        let forcepos = ax.position_endstop + 1.5 * (ax.max - ax.position_endstop);
        self.wait_moves()?;
        {
            let mut c = self.core();
            let mut pos = c.commanded;
            pos[a] = forcepos;
            c.set_position_homed(pos, &[a]);
        }
        let prev_accel = self.core().planner.lim.max_accel;
        if let Some(acc) = ax.homing_accel {
            self.single(&format!("M204 S{acc:.2}"));
        }
        if let Some(hc) = ax.homing_current {
            self.single(&format!("SET_TMC_CURRENT_{} CURRENT={}", STEPPER_NAMES[a], cpp_to_string(hc)));
        }
        let mut ok = self.homing_move(a, ax.position_endstop, ax.homing_speed)?;
        if ok && ax.retract_dist != 0.0 {
            // retract, pretend we are twice as far, home again slower
            let mut pos = self.core().commanded;
            pos[a] = ax.position_endstop + ax.retract_dist;
            self.core().add_move(pos, ax.retract_speed, false)?;
            self.wait_moves()?;
            pos[a] += ax.retract_dist;
            self.core().set_position(pos);
            ok = self.homing_move(a, ax.position_endstop, ax.second_homing_speed)?;
        }
        if ok && ax.force_retract != 0.0 {
            let mut pos = self.core().commanded;
            pos[a] = ax.position_endstop + ax.force_retract;
            self.core().add_move(pos, ax.retract_speed, false)?;
            self.wait_moves()?;
            self.core().set_position(pos);
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
        if !ok {
            super::console(&format!("Failed to home {}", ax.name.to_ascii_lowercase()));
            let _ = self.core().motor_off();
            if a == 2 {
                self.invoke_shutdown("Z-axis zeroing failed");
            }
            return Err(Abort(format!("No trigger on {} after full movement", ax.name.to_ascii_lowercase())));
        }
        Ok(())
    }

    /// One homing move towards the endstop, stopped by the MCU through
    /// trsync. Ok(false) when the endstop never triggered.
    fn homing_move(&mut self, a: usize, endstop: f64, speed: f64) -> Result<bool, Abort> {
        let ax = self.core().m.axes[a].clone();
        let ts = dsp::TRSYNC as i64;
        let start = {
            let mut c = self.core();
            c.ui.set_status(format!("Homing {}…", ax.name));
            c.responses.clear();
            c.homing = true;
            // like homing.py: start at get_last_move_time(), which on an idle
            // toolhead is now + START_DELAY, so a resent frame is still in time
            let t = c.last_move_time();
            let m = c.mcu(Target::Dsp)?;
            let c0 = m.clock.clock(t) as u32 as i64;
            let (sample, rest) = (m.clock.ticks(15e-6) as i64, m.clock.ticks(1e-4) as i64);
            let msgs = [m.encode("trsync_start", &[("oid", Arg::Int(ts)), ("report_clock", Arg::Int(0)), ("report_ticks", Arg::Int(0)), ("expire_reason", Arg::Int(4))])];
            for msg in msgs {
                m.queue_msg(msg);
            }
            for &s in ax.steppers {
                let oid = machine::dsp::STEP_A as i64 + s as i64;
                let msg = m.encode("stepper_stop_on_trigger", &[("oid", Arg::Int(oid)), ("trsync_oid", Arg::Int(ts))]);
                m.queue_msg(msg);
            }
            let msg = m.encode(
                "endstop_home",
                &[
                    ("oid", Arg::Int(ax.endstop_oid as i64)),
                    ("clock", Arg::Int(c0)),
                    ("sample_ticks", Arg::Int(sample)),
                    ("sample_count", Arg::Int(4)),
                    ("rest_ticks", Arg::Int(rest)),
                    // triggered level: the pin's active level ("!PG4" = low)
                    ("pin_value", Arg::Int((!ax.endstop.invert) as i64)),
                    ("trsync_oid", Arg::Int(ts)),
                    ("trigger_reason", Arg::Int(1)),
                ],
            );
            m.queue_msg(msg);
            let mut end = c.commanded;
            let start = end[a];
            end[a] = endstop;
            if let Err(e) = c.add_move(end, speed, false) {
                c.homing = false;
                return Err(e.into());
            }
            c.flush_moves();
            let end_t = c.planner.print_time + 0.1;
            let m = c.mcu(Target::Dsp)?;
            let msg = m.encode("trsync_set_timeout", &[("oid", Arg::Int(ts)), ("clock", Arg::Int(m.clock.clock(end_t) as u32 as i64))]);
            m.queue_msg(msg);
            c.wake();
            start
        };
        let travel_t = (start - endstop).abs() / speed + 3.0;
        let reason = self.p.wait(travel_t, "homing trigger", |c| {
            let i = c.responses.iter().position(|(t, m)| *t == Target::Dsp && m.name() == "trsync_state" && m.int("can_trigger") == Some(0))?;
            let (_, m) = c.responses.remove(i).unwrap();
            Some(m.int("trigger_reason").unwrap_or(0))
        });
        {
            let mut c = self.core();
            c.homing = false;
            c.stop_motion();
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
            for &s in ax.steppers {
                let msg = m.encode("stepper_get_position", &[("oid", Arg::Int(machine::dsp::STEP_A as i64 + s as i64))]);
                m.queue_msg(msg);
            }
            c.wake();
        }
        let mut positions = Vec::new();
        for &s in ax.steppers {
            let oid = machine::dsp::STEP_A as i64 + s as i64;
            let pos = self.p.wait(2.0, "stepper position", |c| {
                let i = c.responses.iter().position(|(_, m)| m.name() == "stepper_position" && m.int("oid") == Some(oid))?;
                let (_, m) = c.responses.remove(i).unwrap();
                m.int("pos")
            })?;
            positions.push((s, pos));
        }
        let reason = reason?;
        let mut c = self.core();
        for (s, pos) in positions {
            c.steppers[s].set_mcu_position(pos);
        }
        let mut p = c.stepper_position();
        p[3] = c.commanded[3];
        if reason != 1 {
            c.set_position(p);
            return Ok(false);
        }
        p[a] = endstop;
        c.set_position(p);
        Ok(true)
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
