//! `[resonance_tester]` commands (resonance_tester.cpp): `TEST_RESONANCES`,
//! `SHAPER_CALIBRATE`, `SHAPER_CALIBRATE_SCRIPT`, `MEASURE_AXES_NOISE`.
//! The object is loaded even without its section, so the commands always
//! exist.
//!
//! Kept stock behaviour:
//! * `HZ_PER_SEC` stays in effect for later tests (it overwrites the
//!   configured value).
//! * The measurement stops as soon as the last pulse is queued, so the
//!   moves still in the buffer (up to 2 s, the highest frequencies) are
//!   not sampled.
//! * `SET_VELOCITY_LIMIT … FORCE=1` raises the acceleration limit for the
//!   test and then sets it to the acceleration in use before it, not to
//!   the old limit.
//! * `TEST_RESONANCES` computes the spectrum and writes nothing (the CSV
//!   output is commented out); only the last `OUTPUT` entry counts.
//!   `AXIS` is not lower-cased: `AXIS=X` is read as a direction vector.
//! * `SHAPER_CALIBRATE` with an `AXIS` other than x/y logs "unsupported
//!   axis", moves to the probe point and does nothing else.
//! * The result goes to printer.cfg and user_printer.cfg and to
//!   `SET_INPUT_SHAPER`, which ignores `SHAPER_TYPE_X/_Y`: the new
//!   frequency applies at once, the type after a restart.
//! * `MEASURE_AXES_NOISE` dwells `MEAS_TIME` but ends the measurement once
//!   the planner has stalled to 2 s of buffered time: about a second of
//!   data for the default 3 s.
//!
//! Undefined behaviour in stock, done the intended way here:
//! * `TEST_RESONANCES` without `AXIS`, an axis that is not `x`, `y` or two
//!   numbers, an accelerometer that is not configured, or a test that got
//!   too little data for one window: an error instead of a null pointer.
//! * `save_params` passes a `std::string` to `%s`: ccos prints the name.

use super::parse::{cpp_double, parse, stod, Abort, CmdResult, GCmd, Lim};
use super::Exec;
use crate::accel::Results;
use crate::app::CalibEvent;
use crate::clock::now;
use crate::resonance::{self, CalibrationData};

/// `std::to_string(double)`.
fn to_string(v: f64) -> String {
    format!("{v:.6}")
}

/// `TestAxis`.
#[derive(Debug, Clone)]
struct TestAxis {
    name: String,
    vib_dir: (f64, f64),
}

impl TestAxis {
    fn named(axis: &str) -> Self {
        let vib_dir = if axis == "x" { (1., 0.) } else { (0., 1.) };
        Self { name: axis.into(), vib_dir }
    }

    fn dir(x: f64, y: f64) -> Self {
        let s = (x * x + y * y).sqrt();
        Self { name: format!("axis={},{}", to_string(x), to_string(y)), vib_dir: (x / s, y / s) }
    }

    fn matches(&self, chip_axis: &str) -> bool {
        (self.vib_dir.0 != 0.0 && chip_axis.contains('x')) || (self.vib_dir.1 != 0.0 && chip_axis.contains('y'))
    }
}

/// `parse_axis`.
fn parse_axis(raw: &str) -> Result<TestAxis, Abort> {
    if raw.is_empty() {
        return Err(Abort("missing AXIS".into()));
    }
    if raw == "x" || raw == "y" {
        return Ok(TestAxis::named(raw));
    }
    let dirs: Vec<&str> = raw.split(',').collect();
    if dirs.len() != 2 {
        super::console("Invalid format of axis");
        if dirs.len() < 2 {
            return Err(Abort("Invalid format of axis".into()));
        }
    }
    let num = |s: &str| stod(s.trim()).map_err(|e| Abort(format!("AXIS: {e}")));
    Ok(TestAxis::dir(num(dirs[0])?, num(dirs[1])?))
}

impl Exec {
    pub(super) fn cmd_test_resonances(&mut self, g: &GCmd) -> CmdResult {
        let axis = parse_axis(&g.get_string("AXIS", ""))?;
        let outputs: Vec<String> = g.get_string("OUTPUT", "resonances").split(',').map(str::to_string).collect();
        for o in &outputs {
            if o != "resonances" && o != "raw_data" {
                self.log("Unsupported output , only 'resonances' and 'raw_data' are supported");
            }
        }
        let _name = g.get_string("NAME", "time");
        let csv_output = outputs.last().is_some_and(|o| o == "resonances");
        self.resonance_run(g, &[axis], csv_output)?;
        Ok(())
    }

    pub(super) fn cmd_shaper_calibrate(&mut self, g: &GCmd) -> CmdResult {
        let axis = g.get_string("AXIS", "");
        let axes = match axis.as_str() {
            "" => vec![TestAxis::named("x"), TestAxis::named("y")],
            "x" | "y" => vec![TestAxis::named(&axis)],
            _ => {
                self.log(format!("unsupported axis {axis}"));
                Vec::new()
            }
        };
        let cfg_smoothing = self.core().m.resonance.max_smoothing;
        let max_smoothing = g.get_double("MAX_SMOOTHING", cfg_smoothing, Lim::min(0.05))?;
        let mut data = self.resonance_run(g, &axes, true)?;
        for axis in &axes {
            self.log(format!("Calculating the best input shaper parameters for {} axis", axis.name));
            let Some(d) = data.iter_mut().find(|(n, _)| *n == axis.name).and_then(|(_, d)| d.take()) else {
                return Err(Abort("axis acceleromter measured no data".into()));
            };
            let (best, _) = resonance::find_best_shaper(&d, max_smoothing);
            self.save_shaper_params(&axis.name, &best.name, best.freq);
        }
        Ok(())
    }

    pub(super) fn cmd_shaper_calibrate_script(&mut self) -> CmdResult {
        self.cmd_measure_axes_noise(&parse("MEASURE_AXES_NOISE").expect("command"))?;
        self.calib(CalibEvent::VibStart);
        self.single("G28");
        if self.core().homed != [true; 3] {
            self.log("G28 failed");
            return Ok(());
        }
        self.calib(CalibEvent::VibStartX);
        self.cmd_shaper_calibrate(&parse("SHAPER_CALIBRATE AXIS=x").expect("command"))?;
        self.calib(CalibEvent::VibFinishX);
        self.calib(CalibEvent::VibStartY);
        self.cmd_shaper_calibrate(&parse("SHAPER_CALIBRATE AXIS=y").expect("command"))?;
        self.calib(CalibEvent::VibFinishY);
        self.calib(CalibEvent::VibFinish);
        Ok(())
    }

    pub(super) fn cmd_measure_axes_noise(&mut self, g: &GCmd) -> CmdResult {
        let meas_time = g.f("MEAS_TIME", 3.)?;
        let chips = self.resonance_chips()?;
        for (_, a) in &chips {
            if !matches!(self.adxl_start(a, None, meas_time as i64)?, Ok(true)) {
                self.calib(CalibEvent::VibErrorX);
                return Err(Abort("resonance_tester error".into()));
            }
        }
        // ToolHead::dwell: check_stall waits for the buffer to drop to 2 s
        self.core().dwell(meas_time);
        self.wait_buffer()?;
        let mut raw = Vec::new();
        for (axes, a) in &chips {
            raw.push((axes.clone(), self.adxl_finish(a)?));
        }
        for (axes, r) in raw {
            let d = resonance::calc_freq_response(&r.x, &r.y, &r.z, r.samp_time).ok_or_else(|| Abort("axis acceleromter measured no data".into()))?;
            let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
            self.respond(&format!(
                "Axes noise for {axes}-axis accelerometer: {} (x), {} (y), {} (z)",
                cpp_double(mean(&d.psd_x)),
                cpp_double(mean(&d.psd_y)),
                cpp_double(mean(&d.psd_z))
            ));
        }
        Ok(())
    }

    /// `m_accel_chips`: each configured chip with the axes it measures.
    fn resonance_chips(&self) -> Result<Vec<(String, crate::accel::AdxlCfg)>, Abort> {
        let c = self.core();
        c.m.resonance
            .chips
            .iter()
            .map(|(axes, name)| match &c.m.adxl {
                Some(a) if a.section == *name => Ok((axes.clone(), a.clone())),
                _ => Err(Abort(format!("resonance_tester: no accelerometer '{name}'"))),
            })
            .collect()
    }

    /// `ResonanceTester::_run_test`: per probe point and axis, measure the
    /// pulse test; with `helper`, each axis's spectrum (None when too
    /// short). `add_data` does nothing in stock, so a second point's data
    /// is dropped.
    fn resonance_run(&mut self, g: &GCmd, axes: &[TestAxis], helper: bool) -> Result<Vec<(String, Option<CalibrationData>)>, Abort> {
        let cfg = self.core().m.resonance.clone();
        // VibrationPulseTest::prepare_test
        let freq_start = g.get_double("FREQ_START", cfg.min_freq, Lim::min(1.))?;
        let freq_end = g.get_double("FREQ_END", cfg.max_freq, Lim::min_max(freq_start, 200.))?;
        let hz = self.res_hz_per_sec.unwrap_or(cfg.hz_per_sec);
        let hz = g.get_double("HZ_PER_SEC", hz, Lim { above: Some(0.), below: Some(2.), ..Lim::NONE })?;
        self.res_hz_per_sec = Some(hz);
        let chips = self.resonance_chips()?;
        let mut data: Vec<(String, Option<CalibrationData>)> = Vec::new();
        for point in &cfg.probe_points {
            let coord = [point.first().copied(), point.get(1).copied(), point.get(2).copied(), None];
            self.manual_move(coord, cfg.move_speed)?;
            for axis in axes {
                self.wait_moves()?;
                self.core().dwell(0.5);
                self.wait_buffer()?;
                if axes.len() > 1 {
                    self.respond("Testing axis ");
                }
                let measure_time = resonance::run_test_time(freq_start, freq_end, hz) as i64;
                for (chip_axes, a) in &chips {
                    if axis.matches(chip_axes) && !matches!(self.adxl_start(a, None, measure_time + 20)?, Ok(true)) {
                        match axis.name.as_str() {
                            "x" => self.calib(CalibEvent::VibErrorX),
                            "y" => self.calib(CalibEvent::VibErrorY),
                            _ => {}
                        }
                        return Err(Abort("resonance_tester error".into()));
                    }
                }
                self.pulse_test(g, axis, freq_start, freq_end, cfg.accel_per_hz, hz)?;
                let mut raw: Vec<(String, Results)> = Vec::new();
                for (chip_axes, a) in &chips {
                    if axis.matches(chip_axes) {
                        raw.push((chip_axes.clone(), self.adxl_finish(a)?));
                        self.respond("-axis accelerometer stats: ");
                    }
                }
                if !helper {
                    continue;
                }
                for (chip_axes, r) in raw {
                    let d = resonance::calc_freq_response(&r.x, &r.y, &r.z, r.samp_time);
                    self.log(format!("axis :{chip_axes}{}", axis.name));
                    if !data.iter().any(|(n, _)| *n == axis.name) {
                        data.push((axis.name.clone(), d));
                    }
                }
            }
        }
        Ok(data)
    }

    /// `VibrationPulseTest::run_test`.
    fn pulse_test(&mut self, g: &GCmd, axis: &TestAxis, freq_start: f64, freq_end: f64, accel_per_hz: f64, hz_per_sec: f64) -> CmdResult {
        let (pos, old_max_accel, has_shaper) = {
            let c = self.core();
            (c.commanded, c.planner.lim.max_accel, c.cfgs.printer.has_section("input_shaper"))
        };
        let old_atd = self.limits.requested_accel_to_decel;
        let max_accel = freq_end * accel_per_hz;
        self.run_script(&[&format!("SET_VELOCITY_LIMIT ACCEL={0} ACCEL_TO_DECEL={0} FORCE=1", to_string(max_accel))]);
        if has_shaper && g.i("INPUT_SHAPING", 0)? == 0 {
            self.core().disable_shaping();
            self.respond("Disabled [input_shaper] for resonance testing");
        }
        let mut freq = freq_start;
        let mut old_freq = freq;
        self.respond(&format!("Testing frequency {} Hz to {} Hz start:{}", to_string(freq), to_string(freq_end), to_string(now())));
        let mut sign = 1.0;
        while freq <= freq_end + 0.000001 {
            let t_seg = 0.25 / freq;
            let accel = accel_per_hz * freq;
            let max_v = accel * t_seg;
            self.single(&format!("M204 S{}", to_string(accel)));
            let l = 0.5 * accel * t_seg * t_seg;
            let (dx, dy) = (axis.vib_dir.0 * l, axis.vib_dir.1 * l);
            self.wait_buffer()?;
            self.core().add_move([pos[0] + sign * dx, pos[1] + sign * dy, pos[2], pos[3]], max_v, true)?;
            self.wait_buffer()?;
            self.core().add_move(pos, max_v, true)?;
            sign = -sign;
            freq += 2. * t_seg * hz_per_sec;
            if freq.floor() > (old_freq + 10.).floor() {
                old_freq = freq;
                self.respond(&format!("frequency {} Hz", to_string(freq)));
            }
        }
        self.respond(&format!("Testing frequency {} Hz end:{}", to_string(freq), to_string(now())));
        self.log("finish move ");
        self.run_script(&[&format!("SET_VELOCITY_LIMIT ACCEL={} ACCEL_TO_DECEL={} FORCE=1", to_string(old_max_accel), to_string(old_atd))]);
        if has_shaper {
            self.core().enable_shaping();
            self.respond("Re-enabled [input_shaper]");
        }
        Ok(())
    }

    /// `ShaperCalibrate::save_params`.
    fn save_shaper_params(&mut self, axis: &str, name: &str, freq: f64) {
        if axis == "xy" {
            self.save_shaper_params("x", name, freq);
            self.save_shaper_params("y", name, freq);
            return;
        }
        {
            let mut c = self.core();
            c.cfgs.printer.set_value("input_shaper", &format!("shaper_type_{axis}"), name);
            c.cfgs.printer.set_double("input_shaper", &format!("shaper_freq_{axis}"), freq);
            c.cfgs.save_printer();
            c.cfgs.save_user("input_shaper", &[]);
        }
        match axis {
            "x" => self.single(&format!("SET_INPUT_SHAPER SHAPER_TYPE_X={name} SHAPER_FREQ_X={freq:.2}")),
            "y" => self.single(&format!("SET_INPUT_SHAPER SHAPER_TYPE_Y={name} SHAPER_FREQ_Y={freq:.2}")),
            _ => {}
        }
        {
            let ui = self.core().ui.clone();
            ui.lock().calib.shaper[(axis == "y") as usize] = (name.to_string(), freq);
        }
        self.respond(&format!("shaper name = {name}"));
        self.respond(&format!("shaper freq = {}", to_string(freq)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axes() {
        let x = parse_axis("x").unwrap();
        assert!(x.matches("xy") && x.matches("x") && !x.matches("y"));
        let d = parse_axis("1,1").unwrap();
        assert_eq!(d.name, "axis=1.000000,1.000000");
        assert!((d.vib_dir.0 - 0.5f64.sqrt()).abs() < 1e-12);
        // not lower-cased: "X" is one number short of a direction
        assert!(parse_axis("X").is_err());
        assert!(parse_axis("").is_err());
    }
}
