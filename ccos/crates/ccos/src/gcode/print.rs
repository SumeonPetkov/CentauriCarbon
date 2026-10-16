//! Printing a file and everything around it, as the stock firmware does it:
//! virtual_sdcard.cpp (the per-line pipeline), print_stats.cpp,
//! pause_resume.cpp, change_filament.cpp and auto_leveling.cpp (M729, wipe).
//!
//! The file pipeline: each line is cut at the first `;`; `;LAYER:<n>`
//! (Elegoo/Cura slicers) or `;AFTER_LAYER_CHANGE` (others) update the layer
//! count; the first line containing `G28` is replaced by `CUSTOM_ZERO`;
//! commands from the screen run between lines.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::time::Instant;

use super::parse::{cpp_double, cpp_to_string, stoi, CmdResult, GCmd};
use super::Exec;
use crate::app::Phase;
use crate::clock::now;
use crate::core::{FAN_HELPER, FAN_MODEL};

#[derive(Default)]
pub struct ChangeFilament {
    pub active: bool,
    pub busy: bool,
    pub check_move_ignore: bool,
}

#[derive(Default)]
pub struct PrintState {
    // virtual_sdcard
    reader: Option<BufReader<File>>,
    next_line: Option<String>,
    path: Option<PathBuf>,
    size: u64,
    position: u64,
    fileslice: i64,
    /// The work loop should run (`is_active`).
    pub active: bool,
    must_pause: bool,
    custom_zero: bool,
    cancelled: bool,
    pub resume_requested: bool,
    // pause_resume
    is_paused: bool,
    pause_command_sent: bool,
    sd_paused: bool,
    save_extruder_temp: f64,
    save_bed_temp: f64,
    save_fans: [f64; 2],
    last_pause: f64,
    last_pause_tick: f64,
    ext_timeout_done: bool,
    fan_timeout_done: bool,
    // print_stats
    phase: Phase,
    print_start: Option<Instant>,
    last_print_time: f64,
    pub current_layer: u32,
    pub total_layers: u32,
    // LEDs (colour set without TRANSMIT)
    pub led_color: [f64; 2],
}

impl PrintState {
    /// `M117`: restart the print timer.
    pub fn stats_reset_timer(&mut self) {
        self.print_start = Some(Instant::now());
        self.last_print_time = 0.0;
    }

    pub fn clear_pause(&mut self) {
        self.is_paused = false;
        self.pause_command_sent = false;
        self.last_pause = 0.0;
        self.ext_timeout_done = false;
        self.fan_timeout_done = false;
    }

    fn close(&mut self) {
        self.reader = None;
        self.next_line = None;
    }
}

impl Exec {
    fn set_phase(&mut self, phase: Phase) {
        self.print.phase = phase;
        self.publish_print();
    }

    fn publish_print(&self) {
        let p = &self.print;
        let c = self.core();
        let mut s = c.ui.lock();
        s.print.phase = p.phase;
        s.print.file = p.path.as_ref().and_then(|x| x.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        s.print.progress = if p.size > 0 { (p.position as f64 / p.size as f64) as f32 } else { 0.0 };
        if let Some(t) = p.print_start {
            s.print.started = Some(t);
            s.print.elapsed = t.elapsed().as_secs_f64() + p.last_print_time;
        }
        s.print.layer = p.current_layer;
        s.print.total_layers = p.total_layers;
    }

    pub(super) fn cmd_set_print_stats_info(&mut self, g: &GCmd) -> CmdResult {
        let cur = g.i("CURRENT_LAYER", 0)? as u32;
        let last = g.f("LAST_PRINT_TIME", 0.0)?;
        let total = g.i("TOTAL_LAYERS", 0)? as u32;
        if cur > 0 {
            self.print.current_layer = cur;
        }
        if total > 0 {
            if cur != 0 {
                self.print.current_layer = cur.min(total);
            }
            self.print.total_layers = total;
        }
        if last > 0.0 {
            self.print.last_print_time = last;
        }
        self.publish_print();
        Ok(())
    }

    /// `_reset_file`.
    pub(super) fn reset_file(&mut self) {
        if self.print.reader.is_some() {
            self.do_pause();
            self.print.close();
        }
        self.print.path = None;
        self.print.position = 0;
        self.print.size = 0;
        self.print.current_layer = 0;
        self.print.total_layers = 0;
        self.print.last_print_time = 0.0;
        self.print.print_start = None;
        self.print.cancelled = false;
        self.print.phase = Phase::Idle;
        self.publish_print();
    }

    fn do_pause(&mut self) {
        self.print.must_pause = true;
        self.print.active = false;
    }

    /// `do_cancel`.
    fn do_cancel(&mut self) {
        self.do_pause();
        self.print.close();
        self.print.cancelled = true;
        self.print.custom_zero = false;
        self.from_file = false;
    }

    /// `SDCARD_PRINT_FILE FILENAME=<absolute path> FILESLICE=<slicer>`.
    pub(super) fn cmd_sdcard_print_file(&mut self, g: &GCmd) -> CmdResult {
        if self.print.active {
            super::console("SD busy");
            return Ok(());
        }
        let filename = g.get_string("FILENAME", "");
        let fileslice = g.i("FILESLICE", 0)?;
        self.reset_file();
        let file = match File::open(&filename) {
            Ok(f) => f,
            Err(_) => {
                super::console(&format!("Failed to open file {filename}"));
                return Ok(());
            }
        };
        let size = file.metadata().map(|m| m.len()).unwrap_or(0);
        super::console(&format!("File opened:{filename} Size:{size}"));
        super::console("File selected");
        self.print.reader = Some(BufReader::with_capacity(64 * 1024, file));
        self.print.path = Some(PathBuf::from(&filename));
        self.print.size = size;
        self.print.fileslice = fileslice;
        self.print.print_start = Some(Instant::now());
        self.core().busy_printing = true;
        self.core().ui.set_status(format!("Printing {}", self.print.path.as_ref().unwrap().file_name().unwrap_or_default().to_string_lossy()));
        // do_resume
        self.print.must_pause = false;
        self.print.active = true;
        self.sd_work();
        Ok(())
    }

    fn read_line(&mut self) -> Option<String> {
        if let Some(l) = self.print.next_line.take() {
            return Some(l);
        }
        let r = self.print.reader.as_mut()?;
        let mut s = String::new();
        match r.read_line(&mut s) {
            Ok(0) | Err(_) => None,
            Ok(_) => Some(s),
        }
    }

    fn peek_line(&mut self) -> String {
        if self.print.next_line.is_none() {
            self.print.next_line = self.read_line();
        }
        self.print.next_line.clone().unwrap_or_default()
    }

    /// `work_handler`: run file lines until paused, cancelled or done.
    pub(super) fn sd_work(&mut self) {
        if self.print.phase != Phase::Printing {
            self.set_phase(Phase::Printing);
        }
        let mut published = now();
        while !self.print.must_pause && self.print.active {
            // commands from the screen run between file lines
            while let Some(job) = self.next_job(false) {
                self.handle_job(job);
            }
            if !self.print.active || self.print.must_pause {
                break;
            }
            if self.p.lock().shutdown.is_some() {
                self.do_pause();
                break;
            }
            let Some(raw) = self.read_line() else {
                self.print.close();
                self.respond("Done printing file");
                break;
            };
            self.print.position += raw.len() as u64;
            let raw = raw.strip_suffix('\n').unwrap_or(&raw).to_string();
            let next = self.peek_line();
            let mut line = self.load_current_layer(&raw, &next);
            if !self.print.custom_zero && line.contains("G28") {
                self.print.custom_zero = true;
                line = "CUSTOM_ZERO".into();
            }
            self.from_file = true;
            if !line.is_empty() {
                self.single(&line);
            }
            self.from_file = false;
            if now() - published > 0.5 {
                published = now();
                self.publish_print();
            }
        }
        self.is_root = false;
        if self.print.reader.is_none() {
            self.print.active = false;
            self.print.custom_zero = false;
            self.core().busy_printing = false;
            if !self.print.cancelled {
                let _ = self.wait_moves();
                self.print.position = self.print.size;
                self.set_phase(Phase::Done);
                let name = self.print.path.as_ref().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                let secs = self.print.print_start.map_or(0.0, |t| t.elapsed().as_secs_f64());
                self.core().ui.set_status(format!("Finished {name} in {secs:.0} s"));
            }
            self.print.cancelled = false;
        }
        self.publish_print();
    }

    /// `load_current_layer`: cut at `;`, track the layer number.
    fn load_current_layer(&mut self, line: &str, next_line: &str) -> String {
        let Some(p) = line.find(';') else { return line.to_string() };
        let after = &line[p + 1..];
        let keyword = if matches!(self.print.fileslice, 2 | 4) { "LAYER:" } else { "AFTER_LAYER_CHANGE" };
        if let Some(rest) = after.strip_prefix(keyword) {
            let layer = if keyword == "LAYER:" {
                let digits: String = rest.chars().skip_while(|c| !c.is_ascii_digit()).take_while(|c| c.is_ascii_digit()).collect();
                digits.parse::<u32>().ok()
            } else {
                next_line.split(' ').next().and_then(|t| stoi(t).ok()).map(|v| (v + 1) as u32)
            };
            let layer = layer.unwrap_or(self.print.current_layer);
            self.single(&format!("SET_PRINT_STATS_INFO CURRENT_LAYER={layer}"));
        }
        line[..p].to_string()
    }

    // ---- pause_resume ----

    fn send_pause_command(&mut self) {
        if !self.print.pause_command_sent {
            if self.print.active {
                self.print.sd_paused = true;
                self.do_pause();
            } else {
                self.respond("action:paused");
            }
            self.print.pause_command_sent = true;
        }
    }

    pub(super) fn cmd_pause(&mut self, g: &GCmd) -> CmdResult {
        if self.print.is_paused {
            self.respond("Print already paused");
            return Ok(());
        }
        let extrude = g.i("EXTRUDE", 0)? != 0;
        let absolute_extrude = self.absolute_extrude;
        self.send_pause_command();
        {
            let c = self.p.lock();
            self.print.save_extruder_temp = c.heaters[0].target;
            self.print.save_bed_temp = c.heaters[1].target;
            self.print.save_fans = [c.fans[FAN_MODEL].current_speed, c.fans[FAN_HELPER].current_speed];
        }
        let (pc, homed) = {
            let c = self.core();
            (c.m.pause, c.homed)
        };
        let mut s: Vec<String> = vec!["SAVE_GCODE_STATE STATE=PAUSE_STATE".into(), "M83".into(), "G1 E-2 F1800".into()];
        if absolute_extrude {
            s.push("M82".into());
        }
        s.push("M211 S0".into());
        if !homed.iter().all(|h| *h) {
            s.push("G28".into());
        }
        s.push("G91".into());
        s.push(format!("G1 Z{} F600", cpp_double(pc.rel_z)));
        s.push("G90".into());
        s.push(format!("G1 X{} F{}", cpp_double(pc.x), cpp_double(pc.move_speed)));
        s.push(format!("G1 Y{} F{}", cpp_double(pc.y), cpp_double(pc.move_speed)));
        if extrude {
            s.push("M83".into());
            for _ in 0..7 {
                s.extend(["G1 E13 F523".into(), "G1 E2 F150".into(), "M400".into()]);
            }
            s.push("G1 E-5 F1800".into());
            for _ in 0..8 {
                s.extend(["G1 E13 F523".into(), "G1 E2 F150".into(), "M400".into()]);
            }
            if absolute_extrude {
                s.push("M82".into());
            }
            s.push("M729".into());
        }
        s.push("M211 S1".into());
        let lines: Vec<&str> = s.iter().map(String::as_str).collect();
        self.run_script(&lines);
        let _ = self.wait_moves();
        self.print.is_paused = true;
        self.print.last_pause = 0.0;
        self.print.last_pause_tick = now();
        self.print.ext_timeout_done = false;
        self.print.fan_timeout_done = false;
        self.core().busy_printing = true;
        self.set_phase(Phase::Paused);
        self.core().ui.set_status("Paused");
        Ok(())
    }

    /// The 1 Hz `pause_timer`: cool the nozzle and stop the fan after a while.
    pub(super) fn pause_timer(&mut self) {
        if !self.print.is_paused {
            return;
        }
        while now() - self.print.last_pause_tick >= 1.0 {
            self.print.last_pause_tick += 1.0;
            let pc = self.core().m.pause;
            if self.print.last_pause > pc.extruder_timeout && !self.print.ext_timeout_done {
                self.print.ext_timeout_done = true;
                let t = self.print.save_extruder_temp.min(pc.extruder_temp);
                self.single(&format!("M104 S{t:.2}"));
            }
            if self.print.last_pause > pc.fan_timeout && !self.print.fan_timeout_done {
                self.print.fan_timeout_done = true;
                self.single(&format!("M106 S{}", (pc.fan_speed + 0.5) as i64));
            }
            self.print.last_pause += 1.0;
        }
    }

    pub(super) fn cmd_resume(&mut self, g: &GCmd) -> CmdResult {
        if !self.print.is_paused {
            self.respond("Print is not paused, resume aborted");
            return Ok(());
        }
        self.print.last_pause = 0.0;
        let absolute_extrude = self.absolute_extrude;
        let pc = self.core().m.pause;
        let _velocity = g.f("VELOCITY", pc.recover_velocity)?;
        let mut s: Vec<String> = vec![
            format!("M190 S{}", cpp_double(self.print.save_bed_temp)),
            format!("M109 S{}", cpp_double(self.print.save_extruder_temp)),
            "M211 S0".into(),
            "M106 S255".into(),
            "M83".into(),
        ];
        for _ in 0..8 {
            s.extend(["G1 E13 F523".into(), "G1 E2 F150".into(), "M400".into()]);
        }
        s.extend(["G4 S4".into(), "M204 S15000".into(), "M729".into(), "M106 S0".into()]);
        if absolute_extrude {
            s.push("M82".into());
        }
        s.push(format!("M106 S{}", cpp_to_string(pc.resume_fan_speed)));
        s.push("M211 S1".into());
        s.push(format!("RESTORE_GCODE_STATE STATE=PAUSE_STATE MOVE=1 MOVE_SPEED={}", cpp_to_string(pc.recover_velocity)));
        let lines: Vec<&str> = s.iter().map(String::as_str).collect();
        self.run_script(&lines);
        self.print.save_extruder_temp = 0.0;
        let fans = self.print.save_fans;
        self.set_fan(FAN_MODEL, fans[0]);
        self.set_fan(FAN_HELPER, fans[1]);
        let _ = self.wait_moves();
        // send_resume_command
        if self.print.sd_paused {
            self.print.sd_paused = false;
            self.print.must_pause = false;
            self.print.active = true;
            self.print.resume_requested = true;
        } else {
            self.respond("action:resumed");
        }
        self.print.pause_command_sent = false;
        self.print.is_paused = false;
        self.set_phase(Phase::Printing);
        Ok(())
    }

    fn cancel_common(&mut self) {
        self.ctl.cancel.store(false, Ordering::Relaxed);
        if self.print.active || self.print.sd_paused || self.print.reader.is_some() {
            self.do_cancel();
        } else {
            self.respond("action:cancel");
        }
        self.print.sd_paused = false;
        self.print.clear_pause();
        self.core().busy_printing = false;
    }

    pub(super) fn cmd_cancel_print(&mut self) -> CmdResult {
        self.cancel_common();
        let s = ["G91", "G0 Z10 F600", "G90", "G28 X Y", "G1 X202 F3000", "G1 Y264.5 F3000", "M104 S0", "M140 S0", "M107", "M18"];
        self.run_script(&s);
        self.set_phase(Phase::Cancelled);
        self.core().ui.set_status("Print cancelled");
        let _ = self.wait_moves();
        Ok(())
    }

    pub(super) fn cmd_stop_print(&mut self) -> CmdResult {
        self.cancel_common();
        self.run_script(&["M104 S0", "M140 S0", "M107", "M18"]);
        self.set_phase(Phase::Cancelled);
        let _ = self.wait_moves();
        Ok(())
    }

    pub(super) fn cmd_m600(&mut self) -> CmdResult {
        super::console("pause because of M600");
        self.run_script(&["PAUSE", "CUT_OFF_FILAMENT", "EXTRUDE_FILAMENT E=-60 F=240 FAN_ON=0 REPORT=0"]);
        self.wait_moves()
    }

    // ---- change_filament ----

    /// `ChangeFilament::check_move` (from G1 while printing).
    pub(super) fn change_filament_check(&mut self, pos: [f64; 4]) {
        if !self.cf.active || !self.print.active {
            return;
        }
        let cf = self.core().m.change_filament;
        if pos[0] > cf.min_x && pos[1] < cf.max_y && !self.cf.check_move_ignore {
            super::console(&format!("Can't move to {:.6}, {:.6}", pos[0], pos[1]));
            self.cf.check_move_ignore = true;
        }
    }

    fn homed_axes(&self) -> [bool; 3] {
        self.core().homed
    }

    pub(super) fn cmd_move_to_extrude(&mut self, g: &GCmd) -> CmdResult {
        let tar = g.i("TARGET_TEMP", 220)?;
        let mv = g.get_int("MOVE", 1, Some(0), Some(1))?;
        let zero_z = g.i("ZERO_Z", 1)?;
        let h = self.homed_axes();
        let cf = self.core().m.change_filament;
        let mut s: Vec<String> = Vec::new();
        if zero_z != 0 {
            s.push(if h.iter().all(|x| *x) { "Z_AXIS_OFF_LIMIT_ACTION".into() } else { "G28".into() });
        } else {
            s.push(if h[0] && h[1] { "Z_AXIS_OFF_LIMIT_ACTION".into() } else { "G28 X Y".into() });
        }
        if mv != 0 {
            s.push("G90".into());
            s.push(format!("G1 X{} F4500", cpp_to_string(cf.pos_x)));
            s.push(format!("G1 Y{} F4500", cpp_to_string(cf.pos_y)));
        }
        s.push(format!("M109 S{tar}"));
        let lines: Vec<&str> = s.iter().map(String::as_str).collect();
        self.run_script(&lines);
        self.wait_moves()
    }

    pub(super) fn cmd_cut_off_filament(&mut self, g: &GCmd) -> CmdResult {
        let zero_z = g.i("ZERO_Z", 1)?;
        let h = self.homed_axes();
        let (cf, temp, smoothed) = {
            let c = self.core();
            (c.m.change_filament, c.m.safe_z.temp, c.heaters[0].smoothed)
        };
        let mut s: Vec<String> = Vec::new();
        let all = if zero_z != 0 { h.iter().all(|x| *x) } else { h[0] && h[1] };
        if !all {
            s.push(if zero_z != 0 { "G28".into() } else { "G28 X Y".into() });
        } else if smoothed < temp - 2.0 {
            s.push(format!("M109 S{}", cpp_double(temp)));
            s.push("Z_AXIS_OFF_LIMIT_ACTION".into());
        }
        s.extend([
            "G90".into(),
            "G1 Y30 F4500".into(),
            "G1 X255 F4500".into(),
            "G1 Y3 F600".into(),
            "G1 Y30 F4500".into(),
            format!("G1 X{} F4500", cpp_to_string(cf.pos_x)),
            format!("G1 Y{} F4500", cpp_to_string(cf.pos_y)),
            "M104 S0".into(),
        ]);
        let lines: Vec<&str> = s.iter().map(String::as_str).collect();
        self.run_script(&lines);
        self.wait_moves()
    }

    pub(super) fn cmd_extrude_filament(&mut self, g: &GCmd) -> CmdResult {
        let e = g.f("E", 60.0)?;
        let f = g.f("F", 240.0)?;
        let fan = g.get_int("FAN_ON", 0, Some(0), Some(1))? != 0;
        let _report = g.i("REPORT", 1)?;
        if fan {
            let speed = self.core().m.change_filament.extrude_fan_speed;
            self.single(&format!("M106 S{}", speed as i64));
        }
        self.single("M83");
        if e < 1e-15 {
            self.single("SET_MIN_EXTRUDE_TEMP S0");
            self.single(&format!("G1 E{} F{}", e as i64, f as i64));
            self.single("SET_MIN_EXTRUDE_TEMP RESET");
        } else {
            self.single(&format!("G1 E{} F{}", e as i64, f as i64));
            self.single("M729");
        }
        self.single("M82");
        self.wait_moves()?;
        if fan {
            self.single("M106 S0");
        }
        self.print.last_pause = 0.0;
        Ok(())
    }

    // ---- auto_leveling (motion-only parts) ----

    /// `M729`: nozzle wipe on the purge area.
    pub(super) fn cmd_m729(&mut self) -> CmdResult {
        let absolute = self.absolute_coord;
        let mut s: Vec<&str> = vec!["G90", "G1 X202 F6000", "G1 Y264.5 F6000", "G1 X173 F8000"];
        for _ in 0..2 {
            s.extend(["G1 X187", "G1 X173"]);
        }
        s.extend(["G1 Y262.5 F800", "G1 X187 F8000"]);
        for _ in 0..2 {
            s.extend(["G1 X173", "G1 X187"]);
        }
        s.extend(["G1 X173", "G1 Y250", "G1 X202", "G1 Y264.5 F1200", "M400"]);
        if !absolute {
            s.push("G91");
        }
        self.run_script(&s);
        Ok(())
    }

    /// `WIPE_NOZZLE`: G28 + `[auto_leveling] extrude_feed_gcode`, mesh off.
    pub(super) fn cmd_wipe_nozzle(&mut self) -> CmdResult {
        let feed = self.core().cfgs.printer.get_string("auto_leveling", "extrude_feed_gcode", "");
        let mut s: Vec<String> = vec!["G28".into()];
        s.extend(feed.split(',').map(|l| l.trim().to_string()).filter(|l| !l.is_empty()));
        let saved = self.mesh_enabled;
        self.mesh_enabled = false;
        let lines: Vec<&str> = s.iter().map(String::as_str).collect();
        self.run_script(&lines);
        self.mesh_enabled = saved;
        Ok(())
    }

    /// `G29.1 P1` re-applies the configured Z endstop; `P0` zeroes it and
    /// the G-code Z offsets.
    pub(super) fn cmd_g29_1(&mut self, g: &GCmd) -> CmdResult {
        if g.i("P", 1)? == 1 {
            let mut c = self.core();
            let e = c.cfgs.printer.get_double("stepper_z", "position_endstop", 4.5);
            c.m.axes[2].position_endstop = e;
        } else {
            self.base[2] = 0.0;
            self.homing_position[2] = 0.0;
            self.core().m.axes[2].position_endstop = 0.0;
        }
        Ok(())
    }
}
