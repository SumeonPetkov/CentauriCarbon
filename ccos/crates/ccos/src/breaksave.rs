//! Power-loss resume ("break save"): a port of the stock
//! `extras/break_save.cpp` and the `break_save_task` thread `main.cpp` starts.
//!
//! The A7 board sees mains loss on GPIOG(0) and keeps running for a moment
//! from a supercapacitor (charged through GPIOC(1), switched onto the 5 V
//! rail by GPIOG(11)). The watcher polls the pin every millisecond; after a
//! 200 ms re-check it stops the file reader and writes the print's state to
//! `<break_save_path>break_save0.gcode`, an INI file. At the next boot the
//! screen offers to resume (`gcode/breaksave.rs`); printing the same file
//! again resumes it.
//!
//! What the stock thread reads "live" while the G-code thread runs, ccos
//! reads from a snapshot the G-code thread publishes before each command
//! and before each move waits ([`Snap`]): the same values, without racing
//! a half-updated position.

use std::fs::File;
use std::io;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::app::Phase;
use crate::clock::now;
use crate::config::{to_string, Config, Configs};
use crate::core::{Core, Shared, FAN_BOX, FAN_HELPER, FAN_MODEL};

/// `BREAK_DETECTION_GPIO` GPIOG(0): high = mains lost.
pub const DETECT_GPIO: u32 = 192;
/// `BREAK_CONTROL_GPIO` GPIOG(11): capacitor output enable.
pub const CONTROL_GPIO: u32 = 203;
/// `CC_EN_GPIO` GPIOC(1): capacitor charging enable.
pub const CC_EN_GPIO: u32 = 65;

/// `BREAK_SAVE_PATH`.
const STOCK_PATH: &str = "/board-resource/";

/// What the G-code thread last published about the print (`virtual_sd`,
/// `print_stats`, `gcode_move` and `pause_resume` status).
#[derive(Debug, Clone, Default)]
pub struct Snap {
    /// A print file is loaded.
    pub loaded: bool,
    pub file_path: String,
    /// Start of the line being executed (`m_file_position`).
    pub file_position: u64,
    pub file_size: u64,
    pub is_active: bool,
    pub print_duration: f64,
    pub current_layer: u32,
    pub total_layers: u32,
    pub absolute_coord: bool,
    pub absolute_extrude: bool,
    /// mm/min (`get_gcode_speed`).
    pub speed: f64,
    /// ×60 (`get_gcode_speed_override`).
    pub speed_factor: f64,
    pub extrude_factor: f64,
    pub last: [f64; 4],
    pub homing: [f64; 4],
    /// `base_position` as saved: the G-code position, or while paused the
    /// raw position kept in `PAUSE_STATE`.
    pub base: [f64; 4],
    /// `pause_resume` saved extruder target.
    pub save_extruder_temp: f64,
}

/// `BreakSave` state shared by the watcher and the G-code thread.
#[derive(Debug, Default)]
pub struct BreakSave {
    /// `m_break_save_enable`: sysconf `power_off_resume_print`.
    pub enable: bool,
    /// `m_break_save_reserve`: a resume is being prepared; no new save.
    pub reserve: bool,
    /// `save_files_num` (2..30).
    pub files_num: u32,
    /// `break_save_path`, as a folder on this system (None: no config folder).
    pub dir: Option<PathBuf>,
    /// `gcode_cmds_before` / `gcode_cmds_after`: read, unused (as on the printer).
    pub gcode_cmds_before: Vec<String>,
    pub gcode_cmds_after: Vec<String>,
    pub snap: Snap,
    /// The watcher asks the G-code thread to stop reading the file (`do_pause`).
    pub power_pause: bool,
}

impl BreakSave {
    /// `BreakSave::BreakSave` plus the app's `m_break_save_enable` from sysconf.
    pub fn new(cfgs: &Configs) -> Self {
        let c = &cfgs.printer;
        let s = "break_save";
        let num = c.get_int_clamped(s, "save_files_num", 10, 2, 30);
        let path = c.get_string(s, "break_save_path", STOCK_PATH);
        // the stock folder is where ccos keeps the config files
        let dir = match (path.strip_prefix("/board-resource"), &cfgs.dir) {
            (Some(rest), Some(d)) => Some(d.join(rest.trim_start_matches('/'))),
            (Some(_), None) => None,
            (None, _) => Some(PathBuf::from(path)),
        };
        let list = |k: &str| {
            let v = c.get_string(s, k, "");
            if v.is_empty() {
                Vec::new()
            } else {
                v.split(',').map(str::to_string).collect()
            }
        };
        BreakSave {
            enable: cfgs.sysconf.get_bool("system", "power_off_resume_print", false),
            reserve: false,
            files_num: num as u32,
            dir,
            gcode_cmds_before: list("gcode_cmds_before"),
            gcode_cmds_after: list("gcode_cmds_after"),
            snap: Snap::default(),
            power_pause: false,
        }
    }

    pub fn path(&self, i: i64) -> Option<PathBuf> {
        self.dir.as_ref().map(|d| d.join(format!("break_save{i}.gcode")))
    }

    fn read(&self, i: i64) -> Option<Config> {
        Config::read(&self.path(i)?).ok()
    }

    /// `delete_save_files`.
    pub fn delete_save_files(&self) {
        for i in 0..self.files_num as i64 {
            if let Some(p) = self.path(i) {
                let _ = std::fs::remove_file(p);
            }
        }
    }

    /// `get_save_file_name`: the first save that names a file.
    pub fn get_save_file_name(&self) -> String {
        for i in 0..self.files_num as i64 {
            let Some(c) = self.read(i) else { continue };
            if c.has_section("virtual_sd_stats") {
                let f = c.get_string("virtual_sd_stats", "file_path", "");
                if !f.is_empty() {
                    return f;
                }
            }
        }
        String::new()
    }

    /// `select_load_variables`: the newest save of that file, or -1.
    pub fn select_load_variables(&self) -> i64 {
        let file_name = self.get_save_file_name();
        if file_name.is_empty() {
            return -1;
        }
        let (mut best, mut max_eventtime) = (-1, 0.0);
        for i in 0..self.files_num as i64 {
            let Some(c) = self.read(i) else { continue };
            if !c.has_section("virtual_sd_stats") || c.get_string("virtual_sd_stats", "file_path", "") != file_name {
                continue;
            }
            if c.has_section("generic") {
                let t = c.get_double_min("generic", "get_monotonic", 0.0, 0.0);
                if t > max_eventtime {
                    max_eventtime = t;
                    best = i;
                }
            }
        }
        best
    }

    /// The file `load_variables` reads (`break_save<select>.gcode`).
    pub fn load_selected(&self) -> Option<(PathBuf, Config)> {
        let p = self.path(self.select_load_variables())?;
        console(&format!("load_variables from  {}", p.display()));
        let c = Config::read(&p).ok()?;
        Some((p, c))
    }
}

fn console(s: &str) {
    crate::gcode::console(s);
}

fn b(v: bool) -> String {
    (v as i32).to_string()
}

/// `save_current_run_status`: every section the stock firmware writes.
pub fn save_current_run_status(c: &Core, eventtime: f64) -> Config {
    let s = &c.bs.snap;
    let mut f = Config::parse("");
    let mut set = |sec: &str, key: &str, v: String| f.set_value(sec, key, &v);
    // save_current_Z_status: the steppers' commanded positions (A = x+y,
    // B = x−y on CoreXY; the stock code saves them as X and Y)
    set("generic", "get_monotonic", to_string(eventtime));
    set("generic", "break_save_num", "0".into());
    set("gcode_status", "base_position_x_stop", to_string(c.steppers[0].position_mm()));
    set("gcode_status", "base_position_y_stop", to_string(c.steppers[1].position_mm()));
    set("gcode_status", "base_position_z_stop", to_string(c.steppers[2].position_mm()));
    // save_virtual_sd_stats_status
    set("virtual_sd_stats", "file_path", s.file_path.clone());
    let progress = if s.file_size > 0 { s.file_position as f64 / s.file_size as f64 } else { 0.0 };
    set("virtual_sd_stats", "progress", to_string(progress));
    set("virtual_sd_stats", "is_active", b(s.is_active));
    set("virtual_sd_stats", "file_position", s.file_position.to_string());
    set("virtual_sd_stats", "file_size", s.file_size.to_string());
    set("z_homing_stats", "is_z_homing", b(false));
    // save_printstats_status (ccos does not count filament yet)
    set("printstats", "filename", s.file_path.clone());
    set("printstats", "total_duration", to_string(s.print_duration));
    set("printstats", "print_duration", to_string(s.print_duration));
    set("printstats", "filament_used", to_string(0.0));
    set("printstats", "current_layer", s.current_layer.to_string());
    set("printstats", "total_layers", s.total_layers.to_string());
    // save_tool_head_status (kinematics + toolhead get_status)
    let homed: String = ["x", "y", "z"].iter().zip(c.homed).filter(|(_, h)| *h).map(|(a, _)| format!("{a},")).collect();
    set("tool_head_status", "homed_axes", homed);
    let (mut amin, mut amax) = (String::new(), String::new());
    for a in &c.m.axes {
        amin.push_str(&format!("{},", to_string(a.min)));
        amax.push_str(&format!("{},", to_string(a.max)));
    }
    set("tool_head_status", "axis_minimum", amin);
    set("tool_head_status", "axis_maximum", amax);
    let p = c.commanded;
    let lim = &c.planner.lim;
    set("tool_head_status", "print_time", to_string(c.planner.print_time));
    set("tool_head_status", "stalls", "0".into());
    set("tool_head_status", "estimated_print_time", to_string(now()));
    set("tool_head_status", "extruder", "extruder".into());
    set("tool_head_status", "position", format!("{},{},{},{}", to_string(p[0]), to_string(p[1]), to_string(p[2]), to_string(p[3])));
    set("tool_head_status", "position_e", to_string(p[3]));
    set("tool_head_status", "max_velocity", to_string(lim.max_velocity));
    set("tool_head_status", "max_accel", to_string(lim.max_accel));
    set("tool_head_status", "max_accel_to_decel", to_string(lim.max_accel_to_decel));
    set("tool_head_status", "square_corner_velocity", to_string(lim.square_corner_velocity));
    // save_gcode_status
    set("gcode_status", "absolute_coord", b(s.absolute_coord));
    set("gcode_status", "absolute_extrude", b(s.absolute_extrude));
    set("gcode_status", "speed", to_string(s.speed));
    set("gcode_status", "speed_factor", to_string(s.speed_factor));
    set("gcode_status", "extrude_factor", to_string(s.extrude_factor));
    for (i, a) in ["x", "y", "z", "e"].iter().enumerate() {
        set("gcode_status", &format!("last_position_{a}"), to_string(s.last[i]));
        set("gcode_status", &format!("homing_position_{a}"), to_string(s.homing[i]));
        set("gcode_status", &format!("base_position_{a}"), to_string(s.base[i]));
    }
    // save_stepper_status: the rails' position_min/max
    for (i, a) in ["x", "y", "z"].iter().enumerate() {
        set("stepper_status", &format!("m_limits_min_{a}"), to_string(c.m.axes[i].min));
        set("stepper_status", &format!("m_limits_max_{a}"), to_string(c.m.axes[i].max));
    }
    // save_printer_fan_status: model and helper fans without their
    // startup_voltage, the box fan as is (no tachometer in ccos: rpm 0)
    let speed = |i: usize| {
        let f = &c.fans[i];
        let sv = f.cfg.startup_voltage;
        let v = (f.last_fan_value - sv) / (1.0 - sv);
        if v < 1e-9 {
            0.0
        } else {
            v
        }
    };
    set("printer_fan_status", "speed", to_string(speed(FAN_MODEL)));
    set("printer_fan_status", "rpm", to_string(0.0));
    set("model_helper_fan_status", "speed", to_string(speed(FAN_HELPER)));
    set("box_fan_status", "speed", to_string(c.fans[FAN_BOX].last_fan_value));
    // save_heaters_status (the extruder's paused target while paused)
    for (i, name) in [(1, "heater_bed"), (0, "extruder")] {
        let target = if name == "extruder" && s.save_extruder_temp > 1e-15 { s.save_extruder_temp } else { c.heaters[i].target };
        set(name, "name", name.into());
        set(name, "target_temp", to_string(target));
    }
    f
}

/// `Write_File`: the whole file, then `sync`.
pub fn write_file(path: &Path, mut f: Config) -> bool {
    f.set_value("generic", "Write_File_monotonic", &to_string(now()));
    if let Some(d) = path.parent() {
        let _ = std::fs::create_dir_all(d);
    }
    if f.write(path).is_err() {
        console("Unable to save variable");
        return false;
    }
    unsafe { libc::sync() };
    console("break_save write finish");
    std::thread::sleep(Duration::from_millis(10));
    true
}

/// A sysfs GPIO, exported (`gpio_init` + `gpio_set_direction`).
struct Gpio {
    value: PathBuf,
    file: Option<File>,
}

impl Gpio {
    fn open(root: &Path, n: u32, out: bool) -> io::Result<Gpio> {
        let base = root.join(format!("gpio{n}"));
        if !base.exists() {
            std::fs::write(root.join("export"), n.to_string())?;
        }
        std::fs::write(base.join("direction"), if out { "out" } else { "in" })?;
        let value = base.join("value");
        let file = if out { None } else { Some(File::open(&value)?) };
        Ok(Gpio { value, file })
    }

    fn set(&self, high: bool) {
        let _ = std::fs::write(&self.value, if high { "1" } else { "0" });
    }

    fn get(&self) -> bool {
        let mut b = [0u8; 1];
        match &self.file {
            Some(f) => f.read_at(&mut b, 0).is_ok_and(|n| n == 1) && b[0] == b'1',
            None => false,
        }
    }
}

/// `break_save_task`: watch for mains loss until ccos exits.
pub fn run_watch(p: Shared) {
    let root = p.lock().hw.gpio_root.clone();
    let pins = (|| Ok::<_, io::Error>((Gpio::open(&root, DETECT_GPIO, false)?, Gpio::open(&root, CONTROL_GPIO, true)?, Gpio::open(&root, CC_EN_GPIO, true)?)))();
    let (detect, control, cc_en) = match pins {
        Ok(g) => g,
        Err(e) => {
            p.lock().log(format!("power-loss detection off: GPIO: {e}"));
            return;
        }
    };
    control.set(false);
    cc_en.set(true);
    let sleep = |ms: u64| std::thread::sleep(Duration::from_millis(ms));
    // check_power_outages: also stops charging the capacitor
    let outage = || {
        if !detect.get() {
            return false;
        }
        cc_en.set(false);
        p.lock().bs.enable
    };
    let mut polls = 0u32;
    loop {
        polls = polls.wrapping_add(1);
        if polls % 50 == 0 && p.lock().exit {
            return;
        }
        if !outage() {
            sleep(1);
            continue;
        }
        cc_en.set(false); // stop power capacity charging
        control.set(true); // enable capacity power output
        sleep(200);
        if !outage() {
            console("[break_save_task] power outages interference signal detected.");
            control.set(false);
            cc_en.set(true);
            continue;
        }
        console("[break_save_task] power outages detected.");
        let save = {
            let mut c = p.lock();
            let printing = matches!(c.ui.lock().print.phase, Phase::Printing | Phase::Paused);
            // not printing, or a resume being prepared: nothing is saved
            if !printing || c.bs.reserve {
                None
            } else {
                // stop reading G-code, then save
                c.bs.power_pause = true;
                c.wake();
                p.cv.notify_all();
                let f = save_current_run_status(&c, now());
                c.bs.path(0).map(|path| (path, f))
            }
        };
        match save {
            None => {
                control.set(false);
                // the stock thread ends here: no detection until the next boot
                return;
            }
            Some((path, f)) => {
                write_file(&path, f);
                console("braek save finished");
                sleep(1);
                control.set(false);
                // The stock thread then spins forever (`while(1){}`, undefined
                // behaviour in C++); ccos just stops watching.
                return;
            }
        }
    }
}
