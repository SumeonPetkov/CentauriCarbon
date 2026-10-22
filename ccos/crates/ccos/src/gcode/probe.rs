//! `[probe]` (extras/probe.cpp `PrinterProbe`): `PROBE`, `QUERY_PROBE`,
//! `PROBE_ACCURACY`, `PROBE_CALIBRATE`; `HOME_ACCURACY` from the
//! `[bed_mesh_probe]` one; and auto_leveling.cpp's `CALIBRATE_Z_OFFSET` and
//! `LIFT_HOT_BED`. With the load cells every probe is a `run_G29_Z` tap
//! (`probing_move`), as on the printer.
//!
//! Kept stock behaviour:
//! * `LIFT_SPEED` is read and ignored: the configured `lift_speed` is used.
//! * `QUERY_PROBE` reads nothing and prints nothing.
//! * `PROBE_ACCURACY` first moves Z to 0, then taps `SAMPLES` times without
//!   lifting in between (`run_G29_Z` comes back up by itself). Its standard
//!   deviation sums into an `int` and divides as integers, so it is almost
//!   always 0.
//! * `PROBE_CALIBRATE` has no manual step: it takes the probe result as the
//!   new Z offset at once and writes printer.cfg.
//! * `CALIBRATE_Z_OFFSET` sets `position_endstop` to 0 first and leaves it
//!   there if the probe fails.
//! * `LIFT_HOT_BED` sets Z to 0 (homed) before it runs and marks Z unhomed
//!   after.

use super::mesh::ProbeCfg;
use super::parse::{cpp_double, Abort, CmdResult, GCmd, Lim};
use super::Exec;

/// `PrinterProbe` state (`get_status`, `m_probe_calibrate_z`).
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct ProbeState {
    pub last_state: bool,
    pub last_z_result: f64,
    pub probe_calibrate_z: f64,
}

impl Exec {
    pub(super) fn probe_handles(&self, key: &str) -> bool {
        let c = self.core();
        let cfg = &c.cfgs.printer;
        match key {
            "PROBE" | "QUERY_PROBE" | "PROBE_CALIBRATE" | "PROBE_ACCURACY" => cfg.has_section("probe"),
            "HOME_ACCURACY" => cfg.has_section("bed_mesh_probe"),
            "CALIBRATE_Z_OFFSET" | "LIFT_HOT_BED" => cfg.has_section("auto_leveling"),
            _ => false,
        }
    }

    pub(super) fn cmd_probe(&mut self, key: &str, g: &GCmd) -> CmdResult {
        if key == "QUERY_PROBE" {
            return Ok(());
        }
        if let Some(l) = self.core().hw.motion_lock() {
            return Err(Abort(l.into()));
        }
        let section = if key == "HOME_ACCURACY" { "bed_mesh_probe" } else { "probe" };
        let p = ProbeCfg::from_config(&self.core().cfgs.printer, section);
        match key {
            "PROBE" => self.cmd_probe_probe(g, &p),
            "PROBE_ACCURACY" | "HOME_ACCURACY" => self.cmd_probe_accuracy(g, &p),
            "PROBE_CALIBRATE" => self.cmd_probe_calibrate(g, &p),
            "CALIBRATE_Z_OFFSET" => self.cmd_calibrate_z_offset(),
            _ => self.cmd_lift_hot_bed(g),
        }
    }

    /// `[probe] z_offset`, negated (`m_z_offset`), and `z_offset_adjust`.
    fn probe_offsets(&self) -> (f64, f64) {
        let c = self.core();
        let cfg = &c.cfgs.printer;
        (-cfg.get_double("probe", "z_offset", 1.0), cfg.get_double("probe", "z_offset_adjust", 0.2))
    }

    /// `PROBE`: the sampled Z here.
    fn cmd_probe_probe(&mut self, g: &GCmd, p: &ProbeCfg) -> CmdResult {
        self.probe.last_state = false;
        match self.probe_run(g, p) {
            Ok(pos) => {
                self.probe.last_z_result = pos[2];
                self.probe.last_state = true;
                self.log(format!("Probe Result is {:.6}", pos[2]));
                Ok(())
            }
            Err(e) => {
                self.probe.last_state = false;
                self.probe.last_z_result = 0.0;
                self.log(format!("Probe error:{}", e.0));
                Err(e)
            }
        }
    }

    /// `_probe`: one `run_G29_Z` tap.
    fn probe_once(&mut self) -> Result<[f64; 4], Abort> {
        let r = self.run_g29_z()?;
        self.log(format!("probe at {:.3},{:.3} is z={:.6}", r[0], r[1], r[2]));
        Ok(r)
    }

    /// `PROBE_ACCURACY` / `HOME_ACCURACY [SAMPLES] [PROBE_SPEED]`.
    fn cmd_probe_accuracy(&mut self, g: &GCmd, p: &ProbeCfg) -> CmdResult {
        let speed = g.get_double("PROBE_SPEED", p.speed, Lim::above(0.0))?;
        g.get_double("LIFT_SPEED", p.lift_speed, Lim::above(0.0))?;
        let lift_speed = p.lift_speed;
        let sample_count = g.get_int("SAMPLES", 10, Some(1), None)?;
        let retract = g.get_double("SAMPLE_RETRACT_DIST", p.sample_retract_dist, Lim::above(0.0))?;
        let mut pos = self.core().commanded;
        pos[2] = 0.0;
        self.manual_move([Some(pos[0]), Some(pos[1]), Some(pos[2]), Some(pos[3])], lift_speed)?;
        super::console(&format!(
            "PROBE_ACCURACY at X:{:.3} Y:{:.3} Z:{:.3} (samples={sample_count} retract={retract:.3} speed={speed:.1} lift_speed={lift_speed:.1})",
            pos[0], pos[1], pos[2]
        ));
        let mut zs = Vec::new();
        while (zs.len() as i64) < sample_count {
            zs.push(self.probe_once()?[2]);
        }
        let max = zs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min = zs.iter().copied().fold(f64::INFINITY, f64::min);
        let avg = zs.iter().sum::<f64>() / zs.len() as f64;
        let mut s = zs.clone();
        s.sort_by(f64::total_cmp);
        let mid = s.len() / 2;
        let median = if s.len() % 2 == 1 { s[mid] } else { (s[mid - 1] + s[mid]) / 2.0 };
        // `int deviation_sum += pow(…)`, then integer division
        let mut deviation_sum: i32 = 0;
        for z in &zs {
            deviation_sum = (deviation_sum as f64 + (z - avg).powi(2)) as i32;
        }
        let sigma = ((deviation_sum as u64 / zs.len() as u64) as f64).sqrt();
        super::console(&format!(
            "probe accuracy results: maximum {max:.6}, minimum {min:.6}, range {:.6}, average {avg:.6}, median {median:.6}, standard deviation {sigma:.6}",
            max - min
        ));
        Ok(())
    }

    /// `PROBE_CALIBRATE`: take the probed Z as the new Z offset.
    fn cmd_probe_calibrate(&mut self, g: &GCmd, p: &ProbeCfg) -> CmdResult {
        g.get_double("LIFT_SPEED", p.lift_speed, Lim::above(0.0))?;
        let lift_speed = p.lift_speed;
        let (z_offset, adjust) = self.probe_offsets();
        let r = (|| -> CmdResult {
            let cur = self.probe_run(g, p)?;
            super::console(&format!("PrinterProbe cmd_PROBE_CALIBRATE Result is z={:.6}", cur[2]));
            // the new offset applies now, without homing again
            {
                let mut c = self.core();
                let mut pos = c.commanded;
                pos[2] -= cur[2] + adjust;
                c.set_position(pos);
            }
            self.reset_last_position();
            self.probe.probe_calibrate_z = cur[2];
            let up = cur[2] + p.sample_retract_dist;
            self.manual_move([Some(cur[0]), Some(cur[1]), Some(up), None], lift_speed)?;
            self.manual_move([Some(cur[0] + p.x_offset), Some(cur[1] + p.y_offset), Some(up), None], lift_speed)?;
            // probe_calibrate_finalize
            let z = z_offset - self.probe.probe_calibrate_z - adjust;
            let mut c = self.core();
            c.cfgs.printer.set_double("probe", "z_offset", -z);
            c.cfgs.printer.set_double("stepper_z", "position_endstop", z);
            c.m.axes[2].position_endstop = z;
            drop(c);
            if let Some(bm) = self.bm.as_mut() {
                bm.probe_z_offset = z;
            }
            super::console(&format!("probe_calibrate_finalize: z_offset: {z:.3}"));
            super::console("The SAVE_CONFIG command will update the printer config file");
            super::console("with the above and restart the printer.");
            let c = self.core();
            if let Some(path) = c.cfgs.path("printer.cfg") {
                let _ = c.cfgs.printer.write(&path);
            }
            Ok(())
        })();
        if let Err(e) = &r {
            self.log(format!("Probe error:{}", e.0));
        }
        r
    }

    /// `CALIBRATE_Z_OFFSET`: home Z on the endstop with `position_endstop`
    /// 0, probe, and save the difference (+1 mm) as the new endstop position.
    fn cmd_calibrate_z_offset(&mut self) -> CmdResult {
        self.core().m.axes[2].position_endstop = 0.0;
        self.single("G28 Z");
        self.single("G1 Z5 F360");
        self.single("PROBE");
        if !self.probe.last_state {
            self.log("CALIBRATE_Z_OFFSET ERROR");
            return Ok(());
        }
        let mut z = self.probe.last_z_result;
        self.log(format!("z_offset = {z:.3}"));
        z += 1.0;
        let mut c = self.core();
        c.m.axes[2].position_endstop = -z;
        c.m.axes[2].position_endstop_extra = -z;
        c.cfgs.printer.set_double("stepper_z", "position_endstop", -z);
        c.cfgs.printer.set_double("stepper_z", "position_endstop_extra", -z);
        if let Some(path) = c.cfgs.path("printer.cfg") {
            let _ = c.cfgs.printer.write(&path);
        }
        if let Some(path) = c.cfgs.path("user_printer.cfg") {
            let _ = c.cfgs.printer.write_specified(&path, "stepper_z", &["position_endstop", "position_endstop_extra"]);
        }
        Ok(())
    }

    /// `LIFT_HOT_BED [S=<mm>]`: raise the bed (Z down) by up to S mm with
    /// the direct Z control, stopping 5 mm back if the load cells are
    /// pressed.
    fn cmd_lift_hot_bed(&mut self, g: &GCmd) -> CmdResult {
        let mut distance = g.get_double("S", 50.0, Lim { above: Some(0.0), below: Some(100.0), ..Lim::NONE })?;
        self.log(format!("lift hot bed:{distance:.6}"));
        {
            let mut c = self.core();
            let mut pos = c.commanded;
            pos[2] = 0.0;
            c.set_position_homed(pos, &[2]);
        }
        self.reset_last_position();
        let hop_speed = self.core().m.safe_z.z_hop_speed;
        let step_mm = self.dirz_step_mm();
        let steps = |d: f64| {
            let cnt = (d / step_mm) as i32;
            (cnt, (((d / hop_speed) * 1000.0 * 1000.0) / cnt as f64) as i32)
        };
        let (step_cnt, step_us) = steps(distance);
        self.calibration_start(50)?;
        self.trigger_start(0, 0, 0)?;
        self.dirz_run(0, step_us as u32, step_cnt as u32, false)?;
        super::console(&format!("PROBE_BY_STEP x= 0 y= 0 z= 0 speed_mm= {} step_us= {step_us} step_cnt= {step_cnt}", cpp_double(hop_speed)));
        super::console("******************************probe_by_step****************************");
        let mut lp: i32 = 0;
        let mut is_trigger = false;
        while self.ck_sys_sta() {
            lp += 1;
            let (done, pressed) = {
                let c = self.core();
                (c.sg.dirzctl.len() == 2, c.sg.is_trigger > 0)
            };
            if done {
                self.log("axis z wait lift timeout");
                break;
            }
            if lp % 100 == 0 {
                super::console(&format!("running_dis: {}", cpp_double(hop_speed * lp as f64 / 100.0)));
            }
            if pressed && !is_trigger {
                super::console("抬升中 probe_by_step trigger!!!");
                is_trigger = true;
                lp = 0;
                // back 5 mm, so the lift does not end pressed
                distance = 5.0;
                self.dirz_run(0, 0, 0, false)?;
                self.sg_sleep(0.010)?;
                let (cnt, us) = steps(distance);
                self.dirz_run(1, us as u32, cnt as u32, true)?;
            }
            self.sg_sleep(0.010)?;
        }
        self.trigger_stop(true)?;
        self.dirz_run(0, 0, 0, false)?;
        self.core().homed[2] = false;
        Ok(())
    }
}
