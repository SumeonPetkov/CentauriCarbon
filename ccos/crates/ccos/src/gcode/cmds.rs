//! Motion (gcode_move, toolhead, arcs), extruder, heater, fan, LED and
//! stepper commands, as Elegoo's firmware implements them.

use super::parse::{cpp_double, cpp_to_string, Abort, CmdResult, GCmd, Lim};
use super::{Exec, GState, Plane};
use crate::core::{FAN_BOX, FAN_MODEL};

impl Exec {
    // ---- gcode_move ----

    /// `move_with_transform`: bed mesh → `ToolHead::move1`.
    pub(super) fn move_to(&mut self, pos: [f64; 4], speed: f64) -> Result<bool, Abort> {
        self.bs_publish();
        if self.bm.is_some() {
            return self.mesh_move(pos, speed);
        }
        self.wait_buffer()?;
        Ok(self.core().add_move(pos, speed, true)?)
    }

    /// `G0`/`G1` (gcode_move.cpp:149): a bad number aborts the command
    /// after the axes before it were applied, as in the firmware.
    pub(super) fn cmd_g1(&mut self, g: &GCmd) -> CmdResult {
        let saved = self.last;
        for (i, a) in ["X", "Y", "Z"].iter().enumerate() {
            if g.has(a) {
                let v = g.f(a, 0.0)?;
                self.last[i] = if self.absolute_coord { v + self.base[i] } else { self.last[i] + v };
            }
        }
        if g.has("E") {
            let v = g.f("E", 0.0)? * self.extrude_factor;
            self.last[3] = if !self.absolute_coord || !self.absolute_extrude { self.last[3] + v } else { v + self.base[3] };
        }
        if g.has("F") {
            self.speed = g.f("F", 0.0)? * self.speed_factor;
            if self.speed <= 0.001 {
                self.speed = 40.0;
            }
        }
        self.change_filament_check(self.last);
        if !self.move_to(self.last, self.speed)? {
            self.last = saved;
        }
        Ok(())
    }

    pub(super) fn cmd_g92(&mut self, g: &GCmd) -> CmdResult {
        for (i, a) in ["X", "Y", "Z", "E"].iter().enumerate() {
            if g.has(a) {
                // Elegoo: E is not scaled by the extrude factor; no axes = no-op
                self.base[i] = self.last[i] - g.f(a, 0.0)?;
            }
        }
        Ok(())
    }

    pub(super) fn cmd_m220(&mut self, g: &GCmd) -> CmdResult {
        let v = g.get_double("S", 100.0, Lim::above(0.0))? / (60.0 * 100.0);
        let gcode_speed = self.speed / self.speed_factor;
        self.speed = gcode_speed * v;
        self.speed_factor = v;
        Ok(())
    }

    pub(super) fn cmd_m221(&mut self, g: &GCmd) -> CmdResult {
        let new = g.get_double("S", 100.0, Lim::above(0.0))? / 100.0;
        let e_value = (self.last[3] - self.base[3]) / self.extrude_factor;
        self.base[3] = self.last[3] - e_value * new;
        self.extrude_factor = new;
        Ok(())
    }

    pub(super) fn cmd_set_gcode_offset(&mut self, g: &GCmd) -> CmdResult {
        let mut delta = [0.0; 4];
        for (i, a) in ["X", "Y", "Z", "E"].iter().enumerate() {
            let off = if g.has(a) {
                g.f(a, 0.0)?
            } else if g.has(&format!("{a}_ADJUST")) {
                g.f(&format!("{a}_ADJUST"), 0.0)? + self.homing_position[i]
            } else {
                continue;
            };
            delta[i] = off - self.homing_position[i];
            self.base[i] += delta[i];
            self.homing_position[i] = off;
        }
        if g.i("MOVE", 0)? != 0 {
            let speed = g.get_double("MOVE_SPEED", self.speed, Lim::above(0.0))?;
            for i in 0..4 {
                self.last[i] += delta[i];
            }
            let _ = self.move_to(self.last, speed)?;
        }
        Ok(())
    }

    pub(super) fn cmd_save_state(&mut self, g: &GCmd) -> CmdResult {
        let name = g.get_string("STATE", "default");
        let s = GState {
            absolute_coord: self.absolute_coord,
            absolute_extrude: self.absolute_extrude,
            speed: self.speed,
            speed_factor: self.speed_factor,
            extrude_factor: self.extrude_factor,
            last: self.last,
            homing: self.homing_position,
            base: self.base,
        };
        self.saved.insert(name, s);
        Ok(())
    }

    pub(super) fn cmd_restore_state(&mut self, g: &GCmd) -> CmdResult {
        let name = g.get_string("STATE", "default");
        let Some(s) = self.saved.get(&name).cloned() else { return Ok(()) };
        self.absolute_coord = s.absolute_coord;
        self.absolute_extrude = s.absolute_extrude;
        self.base = s.base;
        self.homing_position = s.homing;
        self.speed = s.speed;
        self.speed_factor = s.speed_factor;
        self.extrude_factor = s.extrude_factor;
        self.base[3] += self.last[3] - s.last[3];
        if g.i("MOVE", 0)? != 0 {
            let speed = g.get_double("MOVE_SPEED", self.speed, Lim::above(0.0))?;
            // Elegoo: XY first, then Z
            self.last[0] = s.last[0];
            self.last[1] = s.last[1];
            let _ = self.move_to(self.last, speed)?;
            self.last[2] = s.last[2];
            let _ = self.move_to(self.last, speed)?;
        }
        Ok(())
    }

    /// `MANUAL_MOVE X= Y= Z= E= F=` (the stock UI's jog): G1 semantics, then
    /// the target is clamped to X 0..256, Y 0..265, Z 0..256.
    pub(super) fn cmd_manual_move(&mut self, g: &GCmd) -> CmdResult {
        let saved = self.last;
        for (i, a) in ["X", "Y", "Z"].iter().enumerate() {
            if g.has(a) {
                let v = g.f(a, 0.0)?;
                self.last[i] = if self.absolute_coord { v + self.base[i] } else { self.last[i] + v };
            }
        }
        if g.has("E") {
            let v = g.f("E", 0.0)? * self.extrude_factor;
            self.last[3] = if !self.absolute_coord || !self.absolute_extrude { self.last[3] + v } else { v + self.base[3] };
        }
        if g.has("F") {
            self.speed = g.f("F", 0.0)? * self.speed_factor;
            if self.speed <= 0.001 {
                self.speed = 40.0;
            }
        }
        self.last[0] = self.last[0].clamp(0.0, 256.0);
        self.last[1] = self.last[1].clamp(0.0, 265.0);
        self.last[2] = self.last[2].clamp(0.0, 256.0);
        self.change_filament_check(self.last);
        if !self.move_to(self.last, self.speed)? {
            self.last = saved;
        }
        Ok(())
    }

    /// G-code coordinates (`get_gcode_position`).
    fn gcode_position(&self) -> [f64; 4] {
        [self.last[0] - self.base[0], self.last[1] - self.base[1], self.last[2] - self.base[2], (self.last[3] - self.base[3]) / self.extrude_factor]
    }

    // ---- gcode_arcs ----

    pub(super) fn cmd_arc(&mut self, g: &GCmd) -> CmdResult {
        let cur = self.gcode_position();
        if !self.absolute_coord {
            super::console("G2/G3 does not support relative move mode ");
        }
        let target = [g.f("X", cur[0])?, g.f("Y", cur[1])?, g.f("Z", cur[2])?];
        if g.has("R") {
            super::console("G2/G3 does not support R moves");
        }
        let (i, j, k) = (g.f("I", 0.0)?, g.f("J", 0.0)?, g.f("K", 0.0)?);
        let (offsets, axes) = match self.plane {
            Plane::Xy => ((i, j), (0, 1, 2)),
            Plane::Xz => ((i, k), (0, 2, 1)),
            Plane::Yz => ((j, k), (1, 2, 0)),
        };
        if offsets.0 == 0.0 && offsets.1 == 0.0 {
            super::console("G2/G3 requires IJ, IK or JK parameters");
        }
        let e = if g.has("E") { Some(g.f("E", 0.0)?) } else { None };
        let f = if g.has("F") { Some(g.f("F", 0.0)?) } else { None };
        let clockwise = g.command == "G2";
        let resolution = self.core().m.arc_resolution;
        let coords = plan_arc(&cur, &target, offsets, clockwise, axes, resolution);
        let mut e_base = 0.0;
        let mut e_per_move = 0.0;
        if let Some(e) = e {
            if self.absolute_extrude {
                e_base = cur[3];
            }
            e_per_move = (e - e_base) / coords.len() as f64;
        }
        for c in coords {
            let mut p = std::collections::BTreeMap::new();
            p.insert("X".to_string(), cpp_to_string(c[0]));
            p.insert("Y".to_string(), cpp_to_string(c[1]));
            p.insert("Z".to_string(), cpp_to_string(c[2]));
            if e_per_move != 0.0 {
                p.insert("E".to_string(), cpp_to_string(e_base + e_per_move));
                if self.absolute_extrude {
                    e_base += e_per_move;
                }
            }
            if let Some(f) = f {
                p.insert("F".to_string(), cpp_to_string(f));
            }
            let g1 = GCmd { command: "G1".into(), commandline: g.commandline.clone(), params: p };
            self.cmd_g1(&g1)?;
        }
        Ok(())
    }

    // ---- toolhead ----

    pub(super) fn cmd_g4(&mut self, g: &GCmd) -> CmdResult {
        let delay = g.get_double("P", 0.0, Lim::min(0.0))? / 1000.0;
        super::console(&format!(">>>>G4 delay = {}", cpp_double(delay)));
        self.core().dwell(delay);
        Ok(())
    }

    fn apply_limits(&mut self, velocity: Option<f64>, accel: Option<f64>, scv: Option<f64>) {
        let mut c = self.core();
        let lim = &mut c.planner.lim;
        if let Some(v) = velocity {
            lim.max_velocity = v;
        }
        if let Some(a) = accel {
            lim.max_accel = a;
        }
        if let Some(s) = scv {
            lim.square_corner_velocity = s;
        }
        // calc_junction_deviation
        lim.max_accel_to_decel = self.limits.requested_accel_to_decel.min(lim.max_accel);
    }

    pub(super) fn cmd_set_velocity_limit(&mut self, g: &GCmd) -> CmdResult {
        let cur = self.core().planner.lim;
        let v = g.get_double("VELOCITY", cur.max_velocity, Lim::above(0.0))?;
        let a = g.get_double("ACCEL", cur.max_accel, Lim::above(0.0))?;
        let scv = g.get_double("SQUARE_CORNER_VELOCITY", cur.square_corner_velocity, Lim::min(0.0))?;
        let atd = g.get_double("ACCEL_TO_DECEL", self.limits.requested_accel_to_decel, Lim::above(0.0))?;
        let force = g.i("FORCE", 0)?;
        let velocity = g.has("VELOCITY").then(|| v.min(self.limits.max_velocity_limit));
        let accel = if g.has("ACCEL") {
            if force != 0 {
                self.limits.max_accel_limit = a;
            }
            Some(a.min(self.limits.max_accel_limit))
        } else {
            None
        };
        if g.has("ACCEL_TO_DECEL") {
            self.limits.requested_accel_to_decel = atd;
        }
        self.apply_limits(velocity, accel, g.has("SQUARE_CORNER_VELOCITY").then_some(scv));
        Ok(())
    }

    pub(super) fn cmd_reset_printer_param(&mut self) -> CmdResult {
        let (pa, scv, atd) = {
            let c = self.core();
            (c.m.extruder.pressure_advance, c.m.limits.square_corner_velocity, c.cfgs.printer.get_double("printer", "max_accel_to_decel", c.m.limits.max_accel_to_decel))
        };
        self.single(&format!("M900 P{pa:.6}"));
        self.single(&format!(
            "SET_VELOCITY_LIMIT VELOCITY={:.2} ACCEL={:.2} SQUARE_CORNER_VELOCITY={:.2} ACCEL_TO_DECEL={:.2}",
            self.limits.max_velocity_limit, self.limits.max_accel_limit, scv, atd
        ));
        Ok(())
    }

    pub(super) fn cmd_m204(&mut self, g: &GCmd) -> CmdResult {
        let limit = self.limits.max_accel_limit;
        let lim = Lim { min: Some(0.0), max: Some(limit), above: Some(0.0), below: None };
        let s = g.get_double("S", f64::MAX, lim)?;
        let accel = if s != f64::MAX {
            s
        } else {
            let p = g.get_double("P", f64::MAX, lim)?;
            let t = g.get_double("T", f64::MAX, lim)?;
            if p == f64::MAX && t == f64::MAX {
                self.respond(&format!("Invalid M204 command{}", g.commandline));
                return Ok(());
            }
            p.min(t)
        };
        self.apply_limits(None, Some(limit.min(accel)), None);
        Ok(())
    }

    /// `M222` (max velocity), `M223` (max Z velocity), `M211` (soft limits):
    /// UI only, ignored from a print file.
    pub(super) fn cmd_toolhead_ui_only(&mut self, key: &str, g: &GCmd) -> CmdResult {
        if self.from_file {
            return Ok(());
        }
        match key {
            "M222" => {
                let v = g.f("S", self.core().m.limits.max_velocity)?;
                self.core().planner.lim.max_velocity = v;
            }
            "M223" => {
                let v = g.f("S", self.core().m.limits.max_z_velocity)?;
                self.core().planner.lim.max_z_velocity = v;
            }
            _ => match g.i("S", -1)? {
                0 => self.core().soft_limits = false,
                1 => self.core().soft_limits = true,
                -1 => {}
                _ => super::console("soft endstop limits config error"),
            },
        }
        Ok(())
    }

    // ---- extruder ----

    pub(super) fn cmd_m104(&mut self, g: &GCmd, wait: bool) -> CmdResult {
        let s = g.f("S", 0.0)?;
        if g.has("T") {
            let t = g.get_int("T", -1, Some(0), None)?;
            if t > 0 {
                // no extruder<T> on this machine
                if s <= 0.0 {
                    return Ok(());
                }
                super::console("Extruder not configured");
                return Ok(());
            }
        }
        self.set_temperature(0, s, wait)
    }

    /// `SET_PRESSURE_ADVANCE [ADVANCE=] [SMOOTH_TIME=]` (no reply). Changing
    /// it flushes step generation, like any change of `kin_flush_delay`.
    pub(super) fn cmd_set_pressure_advance(&mut self, g: &GCmd) -> CmdResult {
        let (pa, st) = self.core().pressure_advance;
        let adv = g.get_double("ADVANCE", pa, Lim::min(0.0))?;
        let st = g.get_double("SMOOTH_TIME", st, Lim::min_max(0.0, 0.2))?;
        self.core().set_pressure_advance(adv, st);
        Ok(())
    }

    /// `M900 K<advance>`: K defaults to printer.cfg's value (0.03 without one).
    pub(super) fn cmd_m900(&mut self, g: &GCmd) -> CmdResult {
        let (def, st) = {
            let c = self.core();
            (c.cfgs.printer.get_double("extruder", "pressure_advance", 0.03), c.pressure_advance.1)
        };
        let k = g.get_double("K", def, Lim::min(0.0))?;
        self.core().set_pressure_advance(k, st);
        Ok(())
    }

    /// `SET_INPUT_SHAPER` / `M593` (input_shaper.cpp). The firmware tests
    /// `if (shaper_type_x = -1)` — an assignment — so SHAPER_TYPE_X/_Y never
    /// take effect; only SHAPER_TYPE changes the type. No reply.
    pub(super) fn cmd_set_input_shaper(&mut self, g: &GCmd) -> CmdResult {
        let s = self.core().shaper;
        let dx = g.get_double("DAMPING_RATIO_X", s.damping_x, Lim::min_max(0.0, 1.0))?;
        let dy = g.get_double("DAMPING_RATIO_Y", s.damping_y, Lim::min_max(0.0, 1.0))?;
        let fx = g.get_double("SHAPER_FREQ_X", s.freq_x, Lim::min(0.0))?;
        let fy = g.get_double("SHAPER_FREQ_Y", s.freq_y, Lim::min(0.0))?;
        let (tx, ty) = match crate::motion::shaper::parse(&g.get_string("SHAPER_TYPE", "")) {
            Some(t) => (t, t),
            None => (s.type_x, s.type_y),
        };
        self.core().set_input_shaper(tx, ty, fx, fy, dx, dy);
        Ok(())
    }

    pub(super) fn cmd_set_e_step_distance(&mut self, g: &GCmd) -> CmdResult {
        if !g.has("DISTANCE") {
            return Ok(());
        }
        let d = g.get_double("DISTANCE", 0.0, Lim::above(0.0))?;
        if d <= 0.0 {
            return Ok(());
        }
        self.wait_moves()?;
        let mut c = self.core();
        let mm = c.steppers[3].position_mm();
        c.steppers[3].step_dist = d;
        c.steppers[3].set_position_mm(mm);
        Ok(())
    }

    pub(super) fn cmd_set_min_extrude_temp(&mut self, g: &GCmd) -> CmdResult {
        let mut c = self.core();
        if g.get_string("RESET", "") == "RESET" {
            let t = c.m.heaters[0].min_extrude_temp;
            c.heaters[0].min_extrude_temp = t;
            c.heaters[0].can_extrude_switch = true;
        } else {
            drop(c);
            let s = g.f("S", 0.0)?;
            let mut c = self.core();
            c.heaters[0].min_extrude_temp = s;
            c.heaters[0].can_extrude_switch = false;
        }
        Ok(())
    }

    /// `FLOW_CALIBRATION` (extruder.cpp:469): eight lines with increasing
    /// pressure advance, each labelled with a digit.
    pub(super) fn cmd_flow_calibration(&mut self, g: &GCmd) -> CmdResult {
        // cfg stepper_z.z_offset (absent on the CC → DBL_MIN) + 0.2
        let z = self.core().cfgs.printer.get_double("stepper_z", "z_offset", 0.0) + 0.2;
        let mut start_pa = g.f("start_pa", 0.0)?;
        let start_x = g.f("start_x", 20.0)?;
        let start_y = g.f("start_y", 20.0)?;
        for l in ["G28", "M140 S60", "M104 S210", "M190 S60", "M109 S210", "G91", "G1 E10 F300", "G90", "M83"] {
            self.single(l);
        }
        let (line_length, line_space) = (150.0, 5.0);
        let e_calib = 0.111416 / 2.40528;
        let mut positive = true;
        for i in 0..8 {
            if start_pa < 0.0000001 {
                self.single("SET_PRESSURE_ADVANCE ADVANCE=0");
            } else {
                self.single(&format!("SET_PRESSURE_ADVANCE ADVANCE={start_pa:.2}"));
            }
            start_pa += 0.01;
            let y = line_space * i as f64 + start_y;
            let (from, to) = if positive { (start_x, start_x + line_length) } else { (start_x + line_length, start_x) };
            self.single(&format!("G0 X{from:.2} Y{y:.2} Z{z:.2} F3000"));
            self.single(&format!("G1 X{to:.2} Y{y:.2} E{:.2} Z{z:.2} F3000", e_calib * line_length));
            positive = !positive;
        }
        self.single("G0 Z10 F180");
        self.single(&format!("G0 X{:.2} Y{start_y:.2} F3000", start_x + line_length + 10.0));
        self.single(&format!("G0 Z{z:.2} F180"));
        self.single("SET_PRESSURE_ADVANCE ADVANCE=0");
        for (i, d) in "12345678".chars().enumerate() {
            let mut script = vec!["G1 E5 F2400".to_string()];
            draw_digit(&mut script, start_x + line_length + 10.0, start_y + line_space * i as f64 + 0.5 * line_space, d);
            script.push("G1 E-5 F3600".into());
            let lines: Vec<&str> = script.iter().map(String::as_str).collect();
            self.run_script(&lines);
        }
        self.single("G0 X110 Y110 Z50");
        Ok(())
    }

    // ---- heaters ----

    /// `PrinterHeaters::set_temperature`: target applied now; with `wait`,
    /// block until |smoothed − target| ≤ 2 °C (silently; the screen's
    /// cancel ends the wait).
    pub(super) fn set_temperature(&mut self, h: usize, t: f64, wait: bool) -> CmdResult {
        self.core().set_heater(h, t)?;
        if wait && t != 0.0 {
            self.wait_until("temperature", |c| c.heaters[h].temp.is_some() && !c.heaters[h].busy())?;
        }
        Ok(())
    }

    pub(super) fn cmd_m140(&mut self, g: &GCmd, wait: bool) -> CmdResult {
        let s = g.f("S", 0.0)?;
        self.set_temperature(1, s, wait)
    }

    /// `M105`: `B:<t> /<target> T0:<t> /<target> ` (log only).
    pub(super) fn cmd_m105(&mut self) -> CmdResult {
        let msg = {
            let c = self.core();
            let f = |i: usize| {
                let h = &c.heaters[i];
                let t = if h.temp.is_some() { h.smoothed } else { 0.0 };
                format!("{}:{} /{} ", if i == 1 { "B" } else { "T0" }, cpp_double(t), cpp_double(h.target))
            };
            format!("{}{}", f(1), f(0))
        };
        self.respond(&msg);
        Ok(())
    }

    pub(super) fn cmd_temperature_wait(&mut self, g: &GCmd) -> CmdResult {
        let sensor = g.get_string("SENSOR", "");
        let min = g.f("MINIMUM", f64::MIN_POSITIVE)?;
        let max = g.get_double("MAXIMUM", f64::MAX, Lim { above: Some(min), ..Lim::NONE })?;
        let h = match sensor.as_str() {
            "extruder" => 0,
            "heater_bed" => 1,
            _ => return Ok(()),
        };
        // busy-waits in the firmware; not cancellable from the screen
        self.p.wait(f64::INFINITY, "temperature", |c| {
            let t = c.heaters[h].smoothed;
            (min <= t && t <= max).then_some(())
        })?;
        Ok(())
    }

    /// Reload the extruder's PID gains from the in-memory `[extruder]`.
    fn reset_extruder_pid(&mut self) {
        let mut c = self.core();
        let (kp, ki, kd) = (c.cfgs.printer.get_double("extruder", "pid_Kp", 0.0), c.cfgs.printer.get_double("extruder", "pid_Ki", 0.0), c.cfgs.printer.get_double("extruder", "pid_Kd", 0.0));
        c.heaters[0].set_gains(kp, ki, kd);
    }

    /// `M301`: writes the misspelled `[exturder]` section, then reloads the
    /// extruder gains from `[extruder]` (so P/I/D have no effect).
    pub(super) fn cmd_m301(&mut self, g: &GCmd) -> CmdResult {
        let _h = g.i("H", 1)?;
        let (p, i, d) = (g.f("P", f64::MIN_POSITIVE)?, g.f("I", f64::MIN_POSITIVE)?, g.f("D", f64::MIN_POSITIVE)?);
        {
            let mut c = self.core();
            c.cfgs.printer.set_double("exturder", "pid_Kp", p);
            c.cfgs.printer.set_double("exturder", "pid_Ki", i);
            c.cfgs.printer.set_double("exturder", "pid_Kd", d);
        }
        self.reset_extruder_pid();
        Ok(())
    }

    /// `M130`/`M131`/`M132` (Kp/Ki/Kd): P0 → `[exturder]` = 0, P1 →
    /// `[heater_bed]` = 1.0; then the extruder gains are reloaded.
    pub(super) fn cmd_m13x(&mut self, key: &str, g: &GCmd) -> CmdResult {
        let field = match key {
            "M130" => "pid_Kp",
            "M131" => "pid_Ki",
            _ => "pid_Kd",
        };
        match g.i("P", -1)? {
            0 => self.core().cfgs.printer.set_double("exturder", field, 0.0),
            1 => self.core().cfgs.printer.set_double("heater_bed", field, 1.0),
            _ => return Ok(()),
        }
        self.reset_extruder_pid();
        Ok(())
    }

    /// `PID_CALIBRATE HEATER= TARGET=`: relay autotune; the result is
    /// written to printer.cfg (extruder also user_printer.cfg) but the
    /// running controller keeps its gains, and the heater is left off.
    pub(super) fn cmd_pid_calibrate(&mut self, g: &GCmd) -> CmdResult {
        let heater = g.get_string("HEATER", "");
        let target = g.f("TARGET", f64::MIN_POSITIVE)?;
        let _ = g.i("WRITE_FILE", 0)?;
        let h = match heater.as_str() {
            "extruder" => 0,
            "heater_bed" => 1,
            _ => return Ok(()),
        };
        {
            let mut c = self.core();
            c.last_move_time();
            c.heaters[h].start_autotune(target);
        }
        let r = self.set_temperature(h, target, true);
        let at = self.core().heaters[h].stop_autotune();
        let Some(at) = at else { return r };
        if r.is_err() || at.busy() {
            super::console("pid_calibrate interrupted");
            self.calib(crate::app::CalibEvent::PidError);
            return r;
        }
        let max_power = self.core().heaters[h].cfg.max_power;
        let (kp, ki, kd) = at.calc_final_pid(max_power);
        super::console(&format!("Autotune: final: Kp={} Ki={} Kd={}", cpp_double(kp), cpp_double(ki), cpp_double(kd)));
        let mut c = self.core();
        c.cfgs.printer.set_value(&heater, "control", "pid");
        c.cfgs.printer.set_double(&heater, "pid_Kp", kp);
        c.cfgs.printer.set_double(&heater, "pid_Ki", ki);
        c.cfgs.printer.set_double(&heater, "pid_Kd", kd);
        c.cfgs.save_printer();
        if h == 0 {
            c.cfgs.save_user("extruder", &["pid_Kp", "pid_Ki", "pid_Kd"]);
        }
        Ok(())
    }

    fn g28_or_fail(&mut self) -> bool {
        self.single("G28");
        let ok = self.core().homed.iter().all(|h| *h);
        if !ok {
            super::console("G28 failed");
        }
        ok
    }

    fn pid_move_line(&self) -> String {
        let p = self.core().m.pid_test_point;
        format!("G1 X{} Y{} Z{} F1800", cpp_to_string(p[0]), cpp_to_string(p[1]), cpp_to_string(p[2]))
    }

    pub(super) fn cmd_pid_calibrate_all(&mut self) -> CmdResult {
        if !self.g28_or_fail() {
            return Ok(());
        }
        let (te, tb) = {
            let c = self.core();
            (c.m.pid_extruder_temp, c.m.pid_bed_temp)
        };
        let mv = self.pid_move_line();
        let script = [
            "G90".to_string(),
            mv,
            "M106 S255".into(),
            format!("PID_CALIBRATE HEATER=extruder TARGET={}", cpp_to_string(te)),
            "M106 S255".into(),
            format!("PID_CALIBRATE HEATER=heater_bed TARGET={}", cpp_to_string(tb)),
            "M104 S0".into(),
            "M140 S0".into(),
            "M107".into(),
            "M18".into(),
        ];
        let lines: Vec<&str> = script.iter().map(String::as_str).collect();
        self.run_script(&lines);
        Ok(())
    }

    pub(super) fn cmd_pid_calibrate_hotbed(&mut self) -> CmdResult {
        if !self.g28_or_fail() {
            return Ok(());
        }
        let tb = self.core().m.pid_bed_temp;
        let mv = self.pid_move_line();
        for l in ["G90".to_string(), mv, "M106 S255".into(), format!("PID_CALIBRATE HEATER=heater_bed TARGET={}", cpp_to_string(tb)), "M104 S0".into(), "M140 S0".into(), "M107".into(), "M18".into()] {
            self.single(&l);
        }
        Ok(())
    }

    pub(super) fn cmd_pid_calibrate_nozzle(&mut self, g: &GCmd) -> CmdResult {
        let def = self.core().m.pid_extruder_temp;
        let t = g.get_double("TARGET_TEMP", def, Lim::min_max(180.0, 335.0))?;
        self.calib(crate::app::CalibEvent::PidStart);
        if !self.g28_or_fail() {
            return Ok(());
        }
        let mv = self.pid_move_line();
        for l in ["G90".to_string(), mv, "M106 S255".into(), "M109 S100".into()] {
            self.single(&l);
        }
        self.calib(crate::app::CalibEvent::PidPreheatDone);
        for l in [format!("PID_CALIBRATE HEATER=extruder TARGET={}", cpp_to_string(t)), "M104 S0".into(), "M140 S0".into(), "M107".into(), "M18".into()] {
            self.single(&l);
        }
        if self.core().shutdown.is_none() {
            self.calib(crate::app::CalibEvent::PidFinish);
        }
        Ok(())
    }

    // ---- fans and LEDs ----

    /// `M106 [P1..3] [S0..255]`: P1/none = part fan, P2 = aux, P3 = chamber.
    pub(super) fn cmd_m106(&mut self, g: &GCmd) -> CmdResult {
        let p = g.get_int("P", 1, Some(0), Some(100))?;
        let s = g.get_double("S", 255.0, Lim::min_max(0.0, 255.0))?;
        if !(1..=3).contains(&p) {
            return Ok(()); // out-of-bounds fan index in the firmware
        }
        self.core().fan_from_command(p as usize - 1, s / 255.0);
        Ok(())
    }

    /// `SET_FAN_SPEED I=<0..2> S=<0..1>` (Elegoo's form).
    pub(super) fn cmd_set_fan_speed(&mut self, g: &GCmd) -> CmdResult {
        let i = g.get_int("I", 0, Some(0), Some(100))?;
        let s = g.get_double("S", 0.0, Lim::min_max(0.0, 1.0))?;
        if !(0..=FAN_BOX as i64).contains(&i) {
            return Ok(());
        }
        self.core().fan_from_command(i as usize, s);
        Ok(())
    }

    /// `SET_LED_led1` / `SET_LED_led2`: only RED has a pin; INDEX is
    /// 0-based; TRANSMIT defaults to 0 (colour stored, sent later).
    pub(super) fn cmd_set_led(&mut self, n: usize, g: &GCmd) -> CmdResult {
        let red = g.get_double("RED", 0.0, Lim::min_max(0.0, 1.0))?;
        for c in ["GREEN", "BLUE", "WHITE"] {
            g.get_double(c, 0.0, Lim::min_max(0.0, 1.0))?;
        }
        let index = g.get_int("INDEX", 0, Some(0), None)?;
        let transmit = g.i("TRANSMIT", 0)?;
        if index >= 1 {
            super::console(&format!("Invalid LED index {index}"));
            return Ok(());
        }
        self.print.led_color[n] = red;
        if transmit != 0 {
            self.core().set_led(n, red);
        }
        Ok(())
    }

    // ---- steppers ----

    pub(super) fn cmd_set_stepper_enable(&mut self, g: &GCmd) -> CmdResult {
        let name = g.get_string("STEPPER", "");
        let enable = g.i("ENABLE", 1)? != 0;
        let idx = self.core().m.steppers.iter().position(|s| s.name == name);
        let Some(i) = idx else {
            super::console(&format!("SET_STEPPER_ENABLE: Invalid stepper {name}"));
            return Ok(());
        };
        self.core().stepper_enable(i, enable)?;
        super::console(&format!("{name} has been manually {}", if enable { "enabled" } else { "disabled" }));
        Ok(())
    }

    /// `STEPPER_BUZZ STEPPER=stepper_x`: ten 1 mm back-and-forth moves of
    /// that motor (the other steppers are undefined behaviour in the firmware).
    pub(super) fn cmd_stepper_buzz(&mut self, g: &GCmd) -> CmdResult {
        if g.get_string("STEPPER", "") != "stepper_x" {
            return Ok(());
        }
        self.wait_moves()?;
        let start = self.core().commanded;
        // motor A alone: a = x + y moves, b = x − y stays
        let fwd = [start[0] + 0.5, start[1] + 0.5, start[2], start[3]];
        for _ in 0..10 {
            {
                let mut c = self.core();
                c.add_move(fwd, 4.0 / 2f64.sqrt(), false)?;
                c.dwell(0.05);
                c.add_move(start, 4.0 / 2f64.sqrt(), false)?;
                c.dwell(0.45);
            }
        }
        self.wait_moves()?;
        self.core().stepper_enable(0, false)?;
        Ok(())
    }

    /// Part fan currently commanded (pause/resume save it).
    pub(super) fn fan_speed(&self, fan: usize) -> f64 {
        self.core().fans[fan].current_speed
    }

    pub(super) fn set_fan(&self, fan: usize, v: f64) {
        self.core().fan_from_command(fan, v);
    }

    #[allow(dead_code)]
    fn model_fan(&self) -> f64 {
        self.fan_speed(FAN_MODEL)
    }
}

/// `planArc` (gcode_arcs.cpp), including the firmware's emission of the
/// intermediate points in plane order (alpha, beta, helical) as X, Y, Z.
fn plan_arc(cur: &[f64; 4], tgt: &[f64; 3], off: (f64, f64), clockwise: bool, axes: (usize, usize, usize), resolution: f64) -> Vec<[f64; 3]> {
    let (a, b, h) = axes;
    let cur3 = [cur[0], cur[1], cur[2]];
    let r_p = -off.0;
    let r_q = -off.1;
    let center_p = cur3[a] - r_p;
    let center_q = cur3[b] - r_q;
    let rt_a = tgt[a] - center_p;
    let rt_b = tgt[b] - center_q;
    let mut ang = (r_p * rt_b - r_q * rt_a).atan2(r_p * rt_a + r_q * rt_b);
    if ang < 0.0 {
        ang += std::f64::consts::TAU;
    }
    if clockwise {
        ang -= std::f64::consts::TAU;
    }
    if ang == 0.0 && cur3[a] == tgt[a] && cur3[b] == tgt[b] {
        ang = std::f64::consts::TAU;
    }
    let lin = tgt[h] - cur3[h];
    let radius = r_p.hypot(r_q);
    let flat = radius * ang;
    let mm = if lin != 0.0 { flat.hypot(lin) } else { flat.abs() };
    let segments = (mm / resolution).floor().max(1.0) as usize;
    let mut coords = Vec::with_capacity(segments);
    for i in 1..segments {
        let theta = i as f64 * ang / segments as f64;
        let (s, c) = theta.sin_cos();
        let rp = -off.0 * c + off.1 * s;
        let rq = -off.0 * s - off.1 * c;
        coords.push([center_p + rp, center_q + rq, cur3[h] + i as f64 * lin / segments as f64]);
    }
    coords.push(*tgt);
    coords
}

/// `draw_digital` (helper/my_math.cpp): a 2 × 4 mm seven-segment digit.
fn draw_digit(script: &mut Vec<String>, x: f64, y: f64, c: char) {
    let len = 2.0;
    let e = 0.111416 / 2.40528;
    let p = [(x, y), (x + len, y), (x + len, y - len), (x, y - len), (x, y - len * 2.0), (x + len, y - len * 2.0)];
    let mv = |i: usize| format!("G0 X{} Y{}", cpp_double(p[i].0), cpp_double(p[i].1));
    let ex = |i: usize, k: f64| format!("G0 X{} Y{} E{}", cpp_double(p[i].0), cpp_double(p[i].1), cpp_double(e * len * k));
    let seq: Vec<String> = match c {
        '0' => vec![mv(0), ex(1, 1.0), ex(5, 2.0), ex(4, 1.0), ex(0, 2.0)],
        '1' => vec![mv(1), ex(5, 2.0)],
        '2' => vec![mv(0), ex(1, 1.0), ex(2, 1.0), ex(3, 1.0), ex(4, 1.0), ex(5, 1.0)],
        '3' => vec![mv(0), ex(1, 1.0), ex(5, 2.0), ex(4, 1.0), mv(2), ex(3, 1.0)],
        '4' => vec![mv(0), ex(3, 1.0), ex(2, 1.0), mv(1), ex(5, 2.0)],
        '5' => vec![mv(1), ex(0, 1.0), ex(3, 1.0), ex(2, 1.0), ex(5, 1.0), ex(4, 1.0)],
        '6' => vec![mv(1), ex(0, 1.0), ex(4, 2.0), ex(5, 1.0), ex(2, 1.0), ex(3, 1.0)],
        '7' => vec![mv(0), ex(1, 1.0), ex(5, 2.0)],
        '8' => vec![mv(2), ex(3, 1.0), ex(4, 1.0), ex(5, 1.0), ex(1, 2.0), ex(0, 1.0), ex(3, 1.0)],
        '9' => vec![mv(5), ex(1, 2.0), ex(0, 1.0), ex(3, 1.0), ex(2, 1.0)],
        _ => vec![],
    };
    script.extend(seq);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arc_quarter_circle() {
        // from (10,0) around (0,0) counter-clockwise to (0,10), r = 10
        let pts = plan_arc(&[10.0, 0.0, 0.0, 0.0], &[0.0, 10.0, 0.0], (-10.0, 0.0), false, (0, 1, 2), 0.1);
        assert_eq!(pts.len(), 157); // floor(15.708 / 0.1)
        for p in &pts {
            assert!((p[0].hypot(p[1]) - 10.0).abs() < 1e-9);
        }
        assert_eq!(*pts.last().unwrap(), [0.0, 10.0, 0.0]);
    }
}
