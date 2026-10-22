//! Power-loss resume on the G-code side:
//! * `load_variables` (break_save.cpp) into [`Saved`] (`s_saved_print_para`);
//! * `cmd_SDCARD_PRINT_FILE_from_break` (virtual_sdcard.cpp): printing the
//!   file a power-loss save names restores its state and goes on from the
//!   saved line;
//! * `STOP_RESUME` (pause_resume.cpp): lift Z, home X/Y, purge and wipe at
//!   the pause position, go back down to the saved height;
//! * the boot prompt (app_top.cpp `break_resume_msgbox_callback`) and the
//!   print screen's 3 minutes at half speed after a resume (app_print.cpp).
//!
//! The snapshot the watcher saves ([`crate::breaksave::Snap`]) is published
//! here, before each command and before each move waits for buffer space.

use std::io::{Seek, SeekFrom};

use super::parse::{cpp_to_string, CmdResult, GCmd};
use super::{console, Exec};
use crate::app::Phase;
use crate::clock::now;
use crate::config::Config;

/// What `load_variables` keeps (`s_saved_print_para`, with the stock types:
/// positions, speeds and fan speeds are `float`, temperatures `short`).
#[derive(Debug, Clone, Default)]
pub struct Saved {
    /// `m_virtual_sd_stats_Variables["file_path"]`.
    pub vsd_file_path: Option<String>,
    /// `m_virtual_sd_stats_Variables` has `file_position`.
    pub has_file_position: bool,
    pub offset_size: i64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub e: f32,
    pub save_x_stop: f32,
    pub save_y_stop: f32,
    pub save_z_stop: f32,
    pub speed: f32,
    pub speed_factor: f32,
    pub absolute_coord: bool,
    pub absolute_extrude: bool,
    pub last_position: [f64; 4],
    pub homing_position: [f64; 4],
    pub curr_layer_num: u16,
    pub total_layer_num: u16,
    pub last_print_time: f64,
    pub filament_used: f64,
    pub bed_temp: i16,
    pub end_temp: i16,
    pub printer_fan_speed: f32,
    pub model_helper_fan_speed: f32,
    pub box_fan_speed: f32,
    pub is_z_homing: bool,
    pub power_off_save_breakpoint: i32,
}

/// `std::to_string(float)`.
fn fs(v: f32) -> String {
    cpp_to_string(v as f64)
}

impl Exec {
    /// A print file was opened: the snapshot names it.
    pub(super) fn bs_start(&mut self, filename: &str) {
        let mut c = self.core();
        c.bs.snap.file_path.clear();
        c.bs.snap.file_path.push_str(filename);
        c.bs.power_pause = false;
    }

    /// Publish what the watcher would read now (only while a file is loaded).
    pub(super) fn bs_publish(&self) {
        let p = &self.print;
        if p.path.is_none() {
            return;
        }
        let paused = matches!(p.phase, Phase::Paused);
        let gpos = [self.last[0] - self.base[0], self.last[1] - self.base[1], self.last[2] - self.base[2], (self.last[3] - self.base[3]) / self.extrude_factor];
        let base = match self.saved.get("PAUSE_STATE") {
            Some(s) if paused => s.last,
            _ => gpos,
        };
        let elapsed = p.print_start.map_or(0.0, |t| t.elapsed().as_secs_f64()) + p.last_print_time;
        let mut c = self.p.lock();
        let s = &mut c.bs.snap;
        s.loaded = true;
        s.file_position = p.line_start;
        s.file_size = p.size;
        s.is_active = p.active;
        s.print_duration = elapsed;
        s.current_layer = p.current_layer;
        s.total_layers = p.total_layers;
        s.absolute_coord = self.absolute_coord;
        s.absolute_extrude = self.absolute_extrude;
        s.speed = self.speed / self.speed_factor;
        s.speed_factor = self.speed_factor * 60.0;
        s.extrude_factor = self.extrude_factor;
        s.last = self.last;
        s.homing = self.homing_position;
        s.base = base;
        s.save_extruder_temp = p.save_extruder_temp;
    }

    /// `load_variables`.
    pub(super) fn load_variables(&mut self) {
        let sel = self.core().bs.load_selected();
        self.bsp.vsd_file_path = None;
        self.bsp.has_file_position = false;
        let Some((_, c)) = sel else { return };
        let s = &mut self.bsp;
        let d = |sec: &str, key: &str, def: f64| c.get_double(sec, key, def);
        if c.has_section("generic") {
            s.power_off_save_breakpoint = (c.get_string("generic", "save_reason", " ") == "work_handler") as i32;
        }
        if c.has_section("virtual_sd_stats") {
            s.vsd_file_path = c.get("virtual_sd_stats", "file_path").map(str::to_string);
            s.has_file_position = c.get("virtual_sd_stats", "file_position").is_some();
            s.offset_size = c.get_int_clamped("virtual_sd_stats", "file_position", 0, 0, i32::MAX as i64);
        }
        if c.has_section("printstats") {
            s.curr_layer_num = c.get_int_clamped("printstats", "current_layer", 0, 0, i32::MAX as i64) as u16;
            s.total_layer_num = c.get_int_clamped("printstats", "total_layers", 0, 0, i32::MAX as i64) as u16;
            s.last_print_time = d("printstats", "print_duration", 0.0);
            s.filament_used = d("printstats", "filament_used", 0.0);
        }
        if c.has_section("gcode_status") {
            let g = "gcode_status";
            s.x = d(g, "base_position_x", 0.0) as f32;
            s.y = d(g, "base_position_y", 0.0) as f32;
            s.z = d(g, "base_position_z", 0.0) as f32;
            s.e = d(g, "base_position_e", 0.0) as f32;
            for (i, a) in ["x", "y", "z", "e"].iter().enumerate() {
                s.last_position[i] = d(g, &format!("last_position_{a}"), 0.0);
                s.homing_position[i] = d(g, &format!("homing_position_{a}"), 0.0);
            }
            s.speed = d(g, "speed", 0.0) as f32;
            s.speed_factor = d(g, "speed_factor", 0.0) as f32;
            s.absolute_coord = c.get_bool(g, "absolute_coord", true);
            s.absolute_extrude = c.get_bool(g, "absolute_extrude", true);
            s.save_x_stop = c.get_double_min(g, "base_position_x_stop", s.x as f64, 0.0) as f32;
            s.save_y_stop = c.get_double_min(g, "base_position_y_stop", s.y as f64, 0.0) as f32;
            s.save_z_stop = c.get_double_min(g, "base_position_z_stop", s.z as f64, 0.0) as f32;
        }
        let fan = |sec: &str| (255.0 * c.get_double_range(sec, "speed", 1.0, 0.0, 1.0)) as f32;
        if c.has_section("printer_fan_status") {
            s.printer_fan_speed = fan("printer_fan_status");
        }
        if c.has_section("model_helper_fan_status") {
            s.model_helper_fan_speed = fan("model_helper_fan_status");
        }
        if c.has_section("box_fan_status") {
            s.box_fan_speed = fan("box_fan_status");
        }
        if c.has_section("z_homing_stats") {
            s.is_z_homing = c.get_bool("z_homing_stats", "is_z_homing", false);
        }
        s.bed_temp = c.get_double_min("heater_bed", "target_temp", 0.0, 0.0) as i16;
        s.end_temp = c.get_double_min("extruder", "target_temp", 0.0, 0.0) as i16;
    }

    /// `cmd_SDCARD_PRINT_FILE_from_break` (also `SDCARD_PRINT_FILE_BREAK_SAVE`).
    pub(super) fn from_break(&mut self) {
        if self.print.reader.is_none() || !self.bsp.has_file_position || self.bsp.offset_size == 0 {
            return;
        }
        let s = self.bsp.clone();
        self.single(&format!("SET_PRINT_STATS_INFO CURRENT_LAYER={} TOTAL_LAYERS={}", s.curr_layer_num, s.total_layer_num));
        self.single(&format!("SET_PRINT_STATS_INFO LAST_PRINT_TIME={}", cpp_to_string(s.last_print_time)));
        // (the stock code also prints `alread_print_time`, a field nothing sets)
        let off = s.offset_size as u64;
        if let Some(r) = self.print.reader.as_mut() {
            let _ = r.seek(SeekFrom::Start(off));
        }
        self.print.next_line = None;
        self.print.position = off;
        self.print.line_start = off;
        // set_position({x, y, z}, {2}): the stock vector has no E, which the
        // toolhead then reads past the end of; ccos keeps the current E
        {
            let mut c = self.core();
            let e = c.commanded[3];
            c.set_position_homed([s.save_x_stop as f64, s.save_y_stop as f64, s.save_z_stop as f64, e], &[2]);
        }
        self.reset_last_position();
        let lines = [
            format!("M140 S{}", s.bed_temp),
            format!("M104 S{}", s.end_temp),
            format!("M190 S{}", s.bed_temp),
            format!("M109 S{}", s.end_temp),
            "M106 P1 S255".into(),
            format!("M106 P2 S{}", fs(s.model_helper_fan_speed)),
            format!("M106 P3 S{}", fs(s.box_fan_speed)),
            format!("STOP_RESUME X={} Y={} Z={} E={} F={} save_z_stop={}", fs(s.x), fs(s.y), fs(s.z), fs(s.e), fs(s.speed), fs(s.save_z_stop)),
            format!("M106 S{}", fs(s.printer_fan_speed)),
            if s.absolute_coord { "G90".into() } else { "G91".into() },
            if s.absolute_extrude { "M82".into() } else { "M83".into() },
            format!("SET_PRINT_STATS_INFO CURRENT_LAYER={} TOTAL_LAYERS={}", s.curr_layer_num, s.total_layer_num),
        ];
        for l in &lines {
            self.single(l);
        }
        self.print.last_print_time = s.last_print_time;
        self.print.files_status = true;
        self.print.slow_since = None;
        self.set_print_speed(50);
        self.publish_resume();
    }

    fn publish_resume(&self) {
        let p = &self.print;
        let c = self.core();
        let mut u = c.ui.lock();
        u.print.layer = p.current_layer;
        u.print.total_layers = p.total_layers;
    }

    /// `STOP_RESUME`.
    pub(super) fn cmd_stop_resume(&mut self, g: &GCmd) -> CmdResult {
        console("STOP_RESUME");
        let mesh = self.take_mesh();
        let pc = self.core().m.pause;
        let _velocity = g.f("VELOCITY", pc.recover_velocity)?;
        self.single("G90");
        let mut position = self.core().commanded;
        let path = self.core().bs.path(0);
        let mark = |on: bool| {
            if let Some(p) = &path {
                let mut f = Config::parse("");
                f.set_value("z_homing_stats", "is_z_homing", if on { "1" } else { "0" });
                let _ = f.write_specified(p, "z_homing_stats", &["is_z_homing"]);
            }
        };
        if !self.bsp.is_z_homing {
            self.single(&format!("G1 Z{} F360", cpp_to_string(position[2] + 10.0)));
            mark(true);
        } else {
            position[2] += 10.0;
            self.core().set_position(position);
            self.reset_last_position();
        }
        self.single("G28 X Y");
        self.wait_moves()?;
        console("G28 X Y");
        self.run_script(&[&format!("G1 X{} F4500", cpp_to_string(pc.x)), &format!("G1 Y{} F4500", cpp_to_string(pc.y))]);
        self.wait_moves()?;
        let mut s: Vec<String> = vec!["M400".into(), "M106 S255".into(), "M83".into()];
        for _ in 0..8 {
            s.extend(["G1 E13 F523".into(), "G1 E2 F150".into(), "M400".into()]);
        }
        s.extend(["G4 S4".into(), "M204 S15000".into(), "M729".into(), "M106 S0".into(), "M82".into()]);
        let lines: Vec<&str> = s.iter().map(String::as_str).collect();
        self.run_script(&lines);
        self.wait_moves()?;
        // back over the part, then down to the saved height
        let mut s = vec![format!("G0 X{} Y{}  F12000", cpp_to_string(g.f("X", 0.0)?), cpp_to_string(g.f("Y", 0.0)?))];
        let save_z_stop = g.f("save_z_stop", 0.0)?;
        if self.core().cfgs.printer.has_section("safe_z_home") {
            s.push("G90".into());
            s.push(format!("G0 Z{} F360", cpp_to_string(save_z_stop)));
            console(&format!("cmd_SDCARD_PRINT_FILE_from_break {}", s[2]));
        } else {
            s.push("G91".into());
            s.push("G1 Z-10 F360".into());
            console("m_safe_z_homing no res");
        }
        mark(false);
        let lines: Vec<&str> = s.iter().map(String::as_str).collect();
        self.run_script(&lines);
        self.wait_moves()?;
        let f = g.f("F", 0.0)?;
        self.run_script(&[&format!("G92 E{}", cpp_to_string(g.f("E", 0.0)?)), &format!("G1 F{}", cpp_to_string(f))]);
        let cur = self.core().commanded;
        self.core().set_position_homed([cur[0], cur[1], save_z_stop, cur[3]], &[0, 1, 2]);
        self.reset_last_position();
        self.set_mesh(mesh);
        let z = g.f("Z", 0.0)?;
        self.single("G90");
        self.single(&format!("G0 X{} Y{} Z{} F{}", cpp_to_string(cur[0]), cpp_to_string(cur[1]), cpp_to_string(z), cpp_to_string(f)));
        self.wait_moves()?;
        self.send_resume_command();
        self.print.is_paused = false;
        Ok(())
    }

    /// `api_set_print_speed` (the print screen's speed setting, in %).
    pub(super) fn set_print_speed(&mut self, value: i32) {
        let val = value as f64 / (60.0 * 100.0);
        self.speed = self.speed / self.speed_factor * val;
        self.speed_factor = val;
        if self.print.phase == Phase::Paused {
            if let Some(s) = self.saved.get_mut("PAUSE_STATE") {
                s.speed = s.speed / s.speed_factor * val;
                s.speed_factor = val;
            }
        }
    }

    /// The print screen's timer: full speed 3 minutes after a resume.
    pub(super) fn slow_start_check(&mut self) {
        if !self.print.files_status {
            return;
        }
        match self.print.slow_since {
            None => {
                self.print.slow_since = Some(now());
                console("speed slow");
            }
            Some(t) if now() - t > 180.0 => self.slow_start_end(),
            Some(_) => {}
        }
    }

    pub(super) fn slow_start_end(&mut self) {
        self.set_print_speed(100);
        self.print.files_status = false;
        self.print.slow_since = None;
        console("speed fast");
    }

    /// At boot (`break_resume_msgbox_callback`): offer to resume.
    pub(super) fn offer_resume(&mut self) {
        let offer = {
            let c = self.core();
            (c.bs.enable && c.bs.select_load_variables() != -1).then(|| c.bs.get_save_file_name())
        };
        if let Some(path) = offer {
            self.core().ui.lock().resume_offer = Some(path);
        }
    }

    /// The prompt's Cancel: count the time, forget the save.
    pub(super) fn discard_break_save(&mut self) {
        self.load_variables();
        let mut c = self.core();
        let t = c.cfgs.sysconf.get_int("system", "cumulative_time", 0) + self.bsp.last_print_time as u64 as i64;
        c.cfgs.sysconf.set_int("system", "cumulative_time", t);
        if let Some(p) = c.cfgs.path("sysconf.cfg") {
            let _ = c.cfgs.sysconf.write(&p);
        }
        c.bs.delete_save_files();
    }
}
