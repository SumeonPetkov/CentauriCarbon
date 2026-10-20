//! The G-code thread: a port of Elegoo's G-code handling
//! (firmware/core/klippy: gcode.cpp, gcode_move.cpp, toolhead.cpp,
//! extruder.cpp, heaters.cpp, fan.cpp, led.cpp, safe_z_home.cpp,
//! homing.cpp, pause_resume.cpp, change_filament.cpp, auto_leveling.cpp,
//! virtual_sdcard.cpp, print_stats.cpp, printer_para.cpp …).
//!
//! The command table is the one the stock firmware has **on the Centauri
//! Carbon** (modules the stock config does not load are absent here too, so
//! e.g. `M73`, `G10`, `EXCLUDE_OBJECT` are unknown commands, as on the
//! printer). Parsing, parameter validation, defaults and the scripts that
//! Elegoo's commands run are reproduced; see `docs/re/10-gcode.md` for the
//! command list, what is implemented and the few deliberate deviations
//! (undefined behaviour in the C++).
//!
//! Like the stock firmware, nothing is sent back to the sender: no `ok`,
//! no error replies. Messages go to the log (the Tools screen, stderr).

pub mod parse;

mod cmds;
mod home;
mod para;
mod print;
mod tmc;

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::Arc;

use crate::app::{LinkState, LinkStatus};
use crate::clock::now;
use crate::core::{Core, Shared};
use crate::Target;
use parse::{Abort, CmdResult, GCmd};

/// Text the stock firmware prints to stdout / its log (never to the sender).
pub fn console(s: &str) {
    eprintln!("ccos: {s}");
    console_keep(s);
}

fn console_keep(s: &str) {
    let mut c = CONSOLE.lock().unwrap_or_else(|e| e.into_inner());
    if c.len() == CONSOLE_LINES {
        c.pop_front();
    }
    c.push_back(s.to_string());
}

const CONSOLE_LINES: usize = 256;
static CONSOLE: std::sync::Mutex<std::collections::VecDeque<String>> = std::sync::Mutex::new(std::collections::VecDeque::new());

/// The last 256 console lines (the stock firmware's log), oldest first.
pub fn console_lines() -> Vec<String> {
    CONSOLE.lock().unwrap_or_else(|e| e.into_inner()).iter().cloned().collect()
}

/// Work for the G-code thread (the UI's command queues).
#[derive(Debug, Clone)]
pub enum Job {
    /// One or more lines (`\n`-separated), like the stock UI's command queue.
    Gcode(String),
    /// Print a file (`SDCARD_PRINT_FILE`).
    Print(PathBuf),
    /// Re-attach every MCU (after a shutdown).
    Restart,
    Reset(Target),
    Probe,
    RefreshFiles,
}

/// Flags the screen sets without waiting for the G-code thread.
#[derive(Default)]
pub struct Control {
    /// The "highest priority" signal (stock UI's cancel): ends waits.
    pub cancel: AtomicBool,
}

/// `SAVE_GCODE_STATE` contents.
#[derive(Debug, Clone)]
struct GState {
    absolute_coord: bool,
    absolute_extrude: bool,
    speed: f64,
    speed_factor: f64,
    extrude_factor: f64,
    last: [f64; 4],
    homing: [f64; 4],
    base: [f64; 4],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Plane {
    Xy,
    Xz,
    Yz,
}

/// Toolhead limits that commands change (`SET_VELOCITY_LIMIT`, `M204`…).
#[derive(Debug, Clone, Copy)]
struct Limits {
    max_velocity_limit: f64,
    max_accel_limit: f64,
    requested_accel_to_decel: f64,
}

pub struct Exec {
    p: Shared,
    ctl: Arc<Control>,
    jobs: Option<Receiver<Job>>,
    // gcode_move
    absolute_coord: bool,
    absolute_extrude: bool,
    base: [f64; 4],
    last: [f64; 4],
    homing_position: [f64; 4],
    /// Internal speed in mm/s (already × speed_factor).
    speed: f64,
    /// mm/min → mm/s, times M220.
    speed_factor: f64,
    extrude_factor: f64,
    saved: HashMap<String, GState>,
    plane: Plane,
    limits: Limits,
    /// printer_para ROOT flag.
    is_root: bool,
    /// The command being executed came from the print file.
    from_file: bool,
    print: print::PrintState,
    cf: print::ChangeFilament,
    mesh_enabled: bool,
    /// Not-yet-available commands already reported once.
    reported: std::collections::HashSet<&'static str>,
    /// `[tmc2209 …]` drivers, in config order (owned by the G-code thread).
    tmc: Vec<crate::tmc::Tmc>,
}

/// `G`/`M` list and the coverage table: every command the stock firmware
/// registers on the Centauri Carbon.
pub const COMMANDS: &[&str] = &[
    "ACCELEROMETER_MEASURE_X",
    "ACCELEROMETER_QUERY_X",
    "ACTIVATE_EXTRUDER",
    "ADXL345_DEBUG_READ_X",
    "ADXL345_DEBUG_WRITE_X",
    "BED_MESH_APPLICATIONS",
    "BED_MESH_CALIBRATE",
    "BED_MESH_CLEAR",
    "BED_MESH_MAP",
    "BED_MESH_OFFSET",
    "BED_MESH_OUTPUT",
    "BED_MESH_SET_INDEX",
    "BREAK_SAVE_STATUS",
    "CALIBRATE_Z_OFFSET",
    "CANCEL_PRINT",
    "CHANGE_FILAMENT_SET_ACTIVE",
    "CHANGE_FILAMENT_SET_BUSY",
    "CHANGE_FILAMENT_SET_CHECK_MOVE_IGNORE",
    "CHECK_BED_MESH",
    "CLEAR_PAUSE",
    "CUSTOM_ZERO",
    "CUT_OFF_FILAMENT",
    "DIRZCTL",
    "DUMP_TMC",
    "ECHO",
    "EXTRUDE_FILAMENT",
    "FIRMWARE_RESTART",
    "FLOW_CALIBRATION",
    "G",
    "G0",
    "G1",
    "G17",
    "G18",
    "G19",
    "G2",
    "G20",
    "G21",
    "G28",
    "G29",
    "G29.1",
    "G3",
    "G4",
    "G90",
    "G91",
    "G92",
    "GET_POSITION",
    "HELP",
    "HOME_ACCURACY",
    "INIT_TMC",
    "LIFT_HOT_BED",
    "LOAD_FILAMENT",
    "M",
    "M104",
    "M105",
    "M106",
    "M107",
    "M109",
    "M110",
    "M112",
    "M114",
    "M115",
    "M117",
    "M130",
    "M131",
    "M132",
    "M140",
    "M18",
    "M190",
    "M204",
    "M211",
    "M220",
    "M221",
    "M222",
    "M223",
    "M301",
    "M400",
    "M593",
    "M600",
    "M601",
    "M602",
    "M603",
    "M729",
    "M75",
    "M76",
    "M77",
    "M82",
    "M83",
    "M84",
    "M900",
    "MANUAL_MOVE",
    "MEASURE_AXES_NOISE",
    "MEASURE_GAP_TEST",
    "MOVE_TO_EXTRUDE",
    "NOZZLE_CLEAR",
    "PAUSE",
    "PID_CALIBRATE",
    "PID_CALIBRATE_ALL",
    "PID_CALIBRATE_HOTBED",
    "PID_CALIBRATE_NOZZLE",
    "PROBE",
    "PROBE_ACCURACY",
    "PROBE_CALIBRATE",
    "PRTOUCH_READY",
    "QUERY_PROBE",
    "READ_HX711",
    "RESET_EXTRUDER",
    "RESET_PRINTER_PARAM",
    "RESTART",
    "RESTORE_GCODE_STATE",
    "RESUME",
    "ROOT",
    "SAVE_GCODE_STATE",
    "SDCARD_PRINT_FILE",
    "SDCARD_PRINT_FILE_BREAK_SAVE",
    "SDCARD_RESET_FILE",
    "SET_EXTRUDER_STEP_DISTANCE",
    "SET_FAN_SPEED",
    "SET_GCODE_OFFSET",
    "SET_HEATER_TEMPERATURE",
    "SET_IDLE_TIMEOUT",
    "SET_INPUT_SHAPER",
    "SET_LED_led1",
    "SET_LED_led2",
    "SET_MIN_EXTRUDE_TEMP",
    "SET_PRESSURE_ADVANCE",
    "SET_PRINT_STATS_INFO",
    "SET_STEPPER_ENABLE",
    "SET_TMC_CURRENT_extruder",
    "SET_TMC_CURRENT_stepper_x",
    "SET_TMC_CURRENT_stepper_y",
    "SET_TMC_CURRENT_stepper_z",
    "SET_TMC_FIELD",
    "SET_VELOCITY_LIMIT",
    "SHAPER_CALIBRATE",
    "SHAPER_CALIBRATE_SCRIPT",
    "STATUS",
    "STEPPER_BUZZ",
    "STOP_PRINT",
    "STOP_RESUME",
    "STRAINGAUGE_TEST",
    "STRAINGAUGE_Z_HOME",
    "TEMPERATURE_WAIT",
    "TEST_RESONANCES",
    "TURN_OFF_HEATERS",
    "UNROOT",
    "UPDATE_IDLE_TIMER",
    "WIPE_NOZZLE",
    "Z_AXIS_OFF_LIMIT_ACTION",
];

/// Commands that exist on the printer but whose subsystem ccos does not
/// have yet (load cells, bed mesh, accelerometer,
/// power-loss resume). They are accepted and logged.
pub const NOT_YET: &[(&str, &str)] = &[
    ("ACCELEROMETER_MEASURE_X", "accelerometer"),
    ("ACCELEROMETER_QUERY_X", "accelerometer"),
    ("ADXL345_DEBUG_READ_X", "accelerometer"),
    ("ADXL345_DEBUG_WRITE_X", "accelerometer"),
    ("BED_MESH_CALIBRATE", "load-cell probing"),
    ("BED_MESH_MAP", "bed mesh"),
    ("BED_MESH_OFFSET", "bed mesh"),
    ("BED_MESH_OUTPUT", "bed mesh"),
    ("BED_MESH_SET_INDEX", "bed mesh"),
    ("BREAK_SAVE_STATUS", "power-loss resume"),
    ("CALIBRATE_Z_OFFSET", "load-cell probing"),
    ("CHECK_BED_MESH", "load-cell probing"),
    ("DIRZCTL", "direct Z control"),
    ("G29", "load-cell probing"),
    ("HOME_ACCURACY", "load-cell probing"),
    ("LIFT_HOT_BED", "load-cell probing"),
    ("MEASURE_AXES_NOISE", "accelerometer"),
    ("MEASURE_GAP_TEST", "load-cell probing"),
    ("NOZZLE_CLEAR", "load-cell probing"),
    ("PROBE", "load-cell probing"),
    ("PROBE_ACCURACY", "load-cell probing"),
    ("PROBE_CALIBRATE", "load-cell probing"),
    ("PRTOUCH_READY", "load-cell probing"),
    ("QUERY_PROBE", "load-cell probing"),
    ("READ_HX711", "load-cell probing"),
    ("SDCARD_PRINT_FILE_BREAK_SAVE", "power-loss resume"),
    ("SHAPER_CALIBRATE", "accelerometer"),
    ("SHAPER_CALIBRATE_SCRIPT", "accelerometer"),
    ("STOP_RESUME", "power-loss resume"),
    ("STRAINGAUGE_TEST", "load-cell probing"),
    ("STRAINGAUGE_Z_HOME", "load-cell probing"),
    ("TEST_RESONANCES", "accelerometer"),
];

impl Exec {
    pub fn new(p: Shared, ctl: Arc<Control>, jobs: Option<Receiver<Job>>) -> Self {
        let tmc = Self::tmc_load(&p);
        let limits = {
            let c = p.lock();
            Limits { max_velocity_limit: c.m.limits.max_velocity, max_accel_limit: c.m.limits.max_accel, requested_accel_to_decel: c.m.limits.max_accel_to_decel }
        };
        Self {
            p,
            ctl,
            jobs,
            absolute_coord: true,
            absolute_extrude: true,
            base: [0.0; 4],
            last: [0.0; 4],
            homing_position: [0.0; 4],
            speed: 25.0,
            speed_factor: 1.0 / 60.0,
            extrude_factor: 1.0,
            saved: HashMap::new(),
            plane: Plane::Xy,
            limits,
            is_root: false,
            from_file: false,
            print: print::PrintState::default(),
            cf: print::ChangeFilament::default(),
            mesh_enabled: true,
            reported: Default::default(),
            tmc,
        }
    }

    fn core(&self) -> std::sync::MutexGuard<'_, Core> {
        self.p.lock()
    }

    fn log(&self, s: impl Into<String>) {
        self.p.lock().log(s);
    }

    /// `respond_info` / `respond_raw`: the firmware log.
    fn respond(&self, s: &str) {
        console(s);
    }

    /// `run_script`: each line through the dispatcher; a failing command
    /// is logged and the next line runs.
    pub fn run_script(&mut self, lines: &[&str]) {
        for l in lines {
            self.single(l);
        }
    }

    /// `single_command` / `process_command` for one line.
    pub fn single(&mut self, line: &str) {
        let Some(g) = parse::parse(line) else { return };
        if let Err(e) = self.dispatch(&g) {
            if self.p.lock().shutdown.is_none() && e.0 != "cancelled" {
                let msg = format!("error: {}: {e}", g.commandline);
                console_keep(&msg);
                self.log(msg);
            }
        }
    }

    /// Look the command up like `gcode_handlers.find(cmd)`.
    pub fn dispatch(&mut self, g: &GCmd) -> CmdResult {
        let key = g.key();
        if let Some((name, what)) = NOT_YET.iter().find(|(c, _)| *c == key) {
            if self.reported.insert(name) {
                self.respond(&format!("{key}: not available in ccos yet (needs {what})"));
            }
            return Ok(());
        }
        if key.starts_with("M8") && key.len() == 5 && para::handles(&key) {
            return self.cmd_para(&key, g);
        }
        if self.tmc_handles(&key) {
            return self.cmd_tmc(&key, g);
        }
        match key.as_str() {
            // gcode.cpp built-ins
            "G" | "M" => {
                self.respond(&format!("list of {key} command"));
                for c in COMMANDS.iter().filter(|c| c.starts_with(key.as_str())) {
                    self.respond(c);
                }
                Ok(())
            }
            "M110" | "M115" | "STATUS" | "HELP" | "G20" | "G21" | "GET_POSITION" | "ACTIVATE_EXTRUDER" | "RESET_EXTRUDER" => Ok(()),
            "M112" => {
                self.invoke_shutdown("shutdown due to M112 command");
                Ok(())
            }
            "RESTART" | "FIRMWARE_RESTART" => self.cmd_restart(),
            "ECHO" => {
                self.respond(&g.commandline);
                Ok(())
            }
            // gcode_move
            "G0" | "G1" => self.cmd_g1(g),
            "M82" => {
                self.absolute_extrude = true;
                Ok(())
            }
            "M83" => {
                self.absolute_extrude = false;
                Ok(())
            }
            "G90" => {
                self.absolute_coord = true;
                Ok(())
            }
            "G91" => {
                self.absolute_coord = false;
                Ok(())
            }
            "G92" => self.cmd_g92(g),
            "M220" => self.cmd_m220(g),
            "M221" => self.cmd_m221(g),
            "M114" => Ok(()),
            "SET_GCODE_OFFSET" => self.cmd_set_gcode_offset(g),
            "SAVE_GCODE_STATE" => self.cmd_save_state(g),
            "RESTORE_GCODE_STATE" => self.cmd_restore_state(g),
            "MANUAL_MOVE" => self.cmd_manual_move(g),
            // arcs
            "G17" => {
                self.plane = Plane::Xy;
                Ok(())
            }
            "G18" => {
                self.plane = Plane::Xz;
                Ok(())
            }
            "G19" => {
                self.plane = Plane::Yz;
                Ok(())
            }
            "G2" | "G3" => self.cmd_arc(g),
            // toolhead
            "G4" => self.cmd_g4(g),
            "M400" => self.wait_moves(),
            "SET_VELOCITY_LIMIT" => self.cmd_set_velocity_limit(g),
            "RESET_PRINTER_PARAM" => self.cmd_reset_printer_param(),
            "M204" => self.cmd_m204(g),
            "M222" | "M223" | "M211" => self.cmd_toolhead_ui_only(&key, g),
            // extruder
            "M104" | "M109" => self.cmd_m104(g, key == "M109"),
            "SET_PRESSURE_ADVANCE" => self.cmd_set_pressure_advance(g),
            "M900" => self.cmd_m900(g),
            "SET_EXTRUDER_STEP_DISTANCE" => self.cmd_set_e_step_distance(g),
            "SET_MIN_EXTRUDE_TEMP" => self.cmd_set_min_extrude_temp(g),
            "LOAD_FILAMENT" => {
                let t = g.i("TEMPERATURE", 215)?;
                self.single(&format!("M109 S{t}"));
                Ok(())
            }
            "FLOW_CALIBRATION" => self.cmd_flow_calibration(g),
            // heaters
            "M140" | "M190" => self.cmd_m140(g, key == "M190"),
            "M105" => self.cmd_m105(),
            "TEMPERATURE_WAIT" => self.cmd_temperature_wait(g),
            "SET_HEATER_TEMPERATURE" => {
                let t = g.f("TARGET", 0.0)?;
                self.set_temperature(0, t, false)
            }
            "TURN_OFF_HEATERS" => {
                let mut c = self.core();
                for h in &mut c.heaters {
                    h.set_target(0.0);
                }
                Ok(())
            }
            "M301" => self.cmd_m301(g),
            "M130" | "M131" | "M132" => self.cmd_m13x(&key, g),
            "PID_CALIBRATE" => self.cmd_pid_calibrate(g),
            "PID_CALIBRATE_ALL" => self.cmd_pid_calibrate_all(),
            "PID_CALIBRATE_HOTBED" => self.cmd_pid_calibrate_hotbed(),
            "PID_CALIBRATE_NOZZLE" => self.cmd_pid_calibrate_nozzle(g),
            // fans and LEDs
            "M106" => self.cmd_m106(g),
            "M107" => {
                self.core().fan_from_command(crate::core::FAN_MODEL, 0.0);
                Ok(())
            }
            "SET_FAN_SPEED" => self.cmd_set_fan_speed(g),
            "SET_LED_led1" => self.cmd_set_led(0, g),
            "SET_LED_led2" => self.cmd_set_led(1, g),
            // steppers
            "M18" | "M84" => self.core().motor_off().map_err(Abort::from),
            "SET_STEPPER_ENABLE" => self.cmd_set_stepper_enable(g),
            "STEPPER_BUZZ" => self.cmd_stepper_buzz(g),
            // idle timeout
            "SET_IDLE_TIMEOUT" => {
                let t = g.get_double("TIMEOUT", self.core().m.idle_timeout, parse::Lim::above(0.0))?;
                let mut c = self.core();
                c.m.idle_timeout = t;
                c.last_activity = now();
                Ok(())
            }
            "UPDATE_IDLE_TIMER" => {
                self.core().last_activity = now();
                Ok(())
            }
            // input shaper (values kept; shaping itself is not in ccos yet)
            "SET_INPUT_SHAPER" | "M593" => self.cmd_set_input_shaper(g),
            // homing
            "G28" => self.cmd_g28(g, false),
            "CUSTOM_ZERO" => self.cmd_g28(g, true),
            "Z_AXIS_OFF_LIMIT_ACTION" => self.z_off_limit_action(),
            // printing
            "M117" => {
                self.print.stats_reset_timer();
                Ok(())
            }
            "SET_PRINT_STATS_INFO" => self.cmd_set_print_stats_info(g),
            "SDCARD_PRINT_FILE" => self.cmd_sdcard_print_file(g),
            "SDCARD_RESET_FILE" => {
                self.reset_file();
                Ok(())
            }
            "PAUSE" | "M601" | "M76" => self.cmd_pause(g),
            "RESUME" | "M602" | "M75" => self.cmd_resume(g),
            "CLEAR_PAUSE" => {
                self.print.clear_pause();
                Ok(())
            }
            "CANCEL_PRINT" | "M603" | "M77" => self.cmd_cancel_print(),
            "STOP_PRINT" => self.cmd_stop_print(),
            "M600" => self.cmd_m600(),
            "CHANGE_FILAMENT_SET_ACTIVE" => {
                self.cf.active = g.get_int("ACTIVE", 0, Some(0), Some(1))? != 0;
                self.respond(&format!("ChangeFilament status : {}", self.cf.active as u8));
                Ok(())
            }
            "CHANGE_FILAMENT_SET_BUSY" => {
                self.cf.busy = g.get_int("BUSY", 0, Some(0), Some(1))? != 0;
                Ok(())
            }
            "CHANGE_FILAMENT_SET_CHECK_MOVE_IGNORE" => {
                self.cf.check_move_ignore = g.get_int("CHECK_MOVE_IGNORE", 0, Some(0), Some(1))? != 0;
                Ok(())
            }
            "CUT_OFF_FILAMENT" => self.cmd_cut_off_filament(g),
            "EXTRUDE_FILAMENT" => self.cmd_extrude_filament(g),
            "MOVE_TO_EXTRUDE" => self.cmd_move_to_extrude(g),
            "M729" => self.cmd_m729(),
            "WIPE_NOZZLE" => self.cmd_wipe_nozzle(),
            "G29.1" => self.cmd_g29_1(g),
            "BED_MESH_APPLICATIONS" => {
                self.mesh_enabled = g.i("ENABLE", 1)? != 0;
                Ok(())
            }
            "BED_MESH_CLEAR" => Ok(()),
            // printer_para
            "ROOT" => {
                self.is_root = true;
                Ok(())
            }
            "UNROOT" => {
                self.is_root = false;
                Ok(())
            }
            _ => self.cmd_default(g),
        }
    }

    /// `cmd_default`: unknown commands print a note and are ignored.
    fn cmd_default(&mut self, g: &GCmd) -> CmdResult {
        let cmd = g.command.as_str();
        if cmd == "M105" || cmd == "M21" {
            return Ok(());
        }
        if cmd.is_empty() {
            if !g.commandline.is_empty() {
                self.respond(&format!("note: cmd_default {}", g.commandline));
            }
            return Ok(());
        }
        self.respond(&format!("unknown command: {}", g.key()));
        Ok(())
    }

    /// `Printer::invoke_shutdown`.
    fn invoke_shutdown(&mut self, why: &str) {
        self.single("M104 S0");
        self.single("M140 S0");
        self.single("M107");
        self.core().emergency(why.to_string());
    }

    /// `RESTART` / `FIRMWARE_RESTART` on this firmware: motors and fans off.
    fn cmd_restart(&mut self) -> CmdResult {
        {
            let mut c = self.core();
            let _ = c.motor_off();
            for f in 0..3 {
                c.fan_from_command(f, 0.0);
            }
            c.dwell(0.5);
        }
        self.wait_moves()
    }

    // ---- waiting ----

    /// Wait for a condition, ending early on the highest-priority signal.
    fn wait_until(&self, what: &str, mut f: impl FnMut(&mut Core) -> bool) -> CmdResult {
        let ctl = self.ctl.clone();
        let r = self.p.wait(f64::INFINITY, what, |c| {
            if ctl.cancel.load(Ordering::Relaxed) {
                return Some(false);
            }
            f(c).then_some(true)
        })?;
        if r {
            Ok(())
        } else {
            Err(Abort("cancelled".into()))
        }
    }

    /// `toolhead.wait_moves()`.
    fn wait_moves(&mut self) -> CmdResult {
        self.core().flush_moves();
        self.wait_until("moves to finish", |c| c.idle())
    }

    /// Keep the planner from running too far ahead of the MCUs.
    fn wait_buffer(&self) -> CmdResult {
        self.wait_until("buffer space", |c| c.buffered_time() < crate::machine::BUFFER_TIME_HIGH && c.mcus.iter().flatten().all(|m| m.backlog() < 600))
    }

    /// `gcode_move.reset_last_position`.
    fn reset_last_position(&mut self) {
        let c = self.p.lock().commanded;
        self.last = c;
    }
}

/// `Serialhdl::send_with_response` (`SerialRetryCommand::get_response`):
/// send, look for the reply 7 ms later and send again until it comes;
/// "No response from MCU" after ~1500 tries. With a `print_time` the request
/// waits for that time on a command queue of its own.
pub(crate) fn send_with_response(p: &Shared, target: Target, name: &str, args: &[(&str, cc_proto::dict::Arg<'_>)], resp: &str, oid: i64) -> Result<cc_proto::OwnedMessage, String> {
    send_with_response_at(p, target, name, args, resp, oid, None)
}

pub(crate) fn send_with_response_at(
    p: &Shared,
    target: Target,
    name: &str,
    args: &[(&str, cc_proto::dict::Arg<'_>)],
    resp: &str,
    oid: i64,
    print_time: Option<f64>,
) -> Result<cc_proto::OwnedMessage, String> {
    // the response callback is registered before sending: older replies don't count
    p.lock().responses.retain(|(t, m)| !(*t == target && m.name() == resp && m.int("oid") == Some(oid)));
    for _ in 0..1516 {
        {
            let mut c = p.lock();
            if let Some(s) = &c.shutdown {
                return Err(s.clone());
            }
            let m = c.mcu(target).map_err(|e| e.to_string())?;
            let msg = m.encode(name, args);
            match print_time {
                Some(pt) => {
                    let clock = m.clock.clock(pt);
                    m.queue_held(clock, msg);
                }
                None => m.queue_msg(msg),
            }
            c.wake();
        }
        let r = p.wait(0.007, resp, |c| {
            let i = c.responses.iter().position(|(t, m)| *t == target && m.name() == resp && m.int("oid") == Some(oid))?;
            c.responses.remove(i).map(|(_, m)| m)
        });
        match r {
            Ok(m) => return Ok(m),
            Err(e) => {
                let c = p.lock();
                if c.shutdown.is_some() || c.exit || c.mcus[target as usize].is_none() {
                    return Err(e.to_string());
                }
            }
        }
    }
    Err(format!("{}: No response from MCU", target.title()))
}

// ---- the thread ----

/// List printable files, newest first.
pub fn list_files(dir: &Path) -> Vec<String> {
    let mut v: Vec<(std::time::SystemTime, String)> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            let lower = name.to_ascii_lowercase();
            (lower.ends_with(".gcode") || lower.ends_with(".gco") || lower.ends_with(".g")).then(|| (e.metadata().and_then(|m| m.modified()).unwrap_or(std::time::UNIX_EPOCH), name))
        })
        .collect();
    v.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    v.into_iter().map(|(_, n)| n).collect()
}

pub fn refresh_files(p: &Shared) {
    let c = p.lock();
    let files = list_files(&c.hw.gcode_dir);
    let mut s = c.ui.lock();
    s.files = files;
    s.gcode_dir = c.hw.gcode_dir.display().to_string();
}

#[cfg(target_os = "linux")]
fn connect(p: &Shared, t: Target) {
    let (hw, m) = {
        let c = p.lock();
        (c.hw.clone(), c.m.clone())
    };
    p.lock().links[t as usize] = LinkStatus { state: LinkState::Unknown, detail: "connecting".into() };
    match crate::core::attach(&hw, &m, t) {
        Ok((mcu, detail)) => {
            let mut c = p.lock();
            c.log(format!("{}: connected ({detail})", t.title()));
            c.install(t, mcu, detail);
        }
        Err(e) => {
            let mut c = p.lock();
            c.links[t as usize] = LinkStatus { state: LinkState::Failed, detail: e.to_string() };
            c.log(format!("{}: {e}", t.title()));
        }
    }
}

#[cfg(target_os = "linux")]
fn connect_all(p: &Shared) {
    if crate::sys::stock_app_running() && !p.lock().hw.simulated {
        p.lock().ui.set_status("Stock app is running — stop it to use the MCUs");
        return;
    }
    p.lock().ui.set_status("Connecting to MCUs…");
    for t in Target::ALL {
        connect(p, t);
    }
    let c = p.lock();
    let ok = c.mcus.iter().filter(|m| m.is_some()).count();
    let lock = if c.hw.motion_lock().is_some() { " · motion locked" } else { "" };
    c.ui.set_status(format!("{ok} of 3 MCUs connected{lock}"));
}

/// The G-code thread body.
#[cfg(target_os = "linux")]
pub fn run(p: Shared, ctl: Arc<Control>, jobs: Receiver<Job>) {
    // busy until klippy:connect work (TMC setup) is done, like "not ready"
    p.lock().ui.lock().busy = true;
    refresh_files(&p);
    connect_all(&p);
    let mut ex = Exec::new(p.clone(), ctl.clone(), Some(jobs));
    ex.tmc_connect(None);
    p.lock().ui.lock().busy = false;
    // Like the stock UI at boot: apply the saved Z offset (M8233 … P1).
    ex.apply_saved_z_offset();
    while let Some(job) = ex.next_job(true) {
        ex.handle_job(job);
        if p.lock().exit {
            return;
        }
    }
}

impl Exec {
    /// Next job from the screen; `block` waits for one.
    fn next_job(&mut self, block: bool) -> Option<Job> {
        if !block {
            return self.jobs.as_ref()?.try_recv().ok();
        }
        {
            loop {
                let r = self.jobs.as_ref()?.recv_timeout(std::time::Duration::from_millis(200));
                match r {
                    Ok(j) => return Some(j),
                    Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                        if self.print.resume_requested {
                            self.print.resume_requested = false;
                            self.sd_work();
                        }
                        self.pause_timer();
                    }
                    Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => return None,
                }
            }
        }
    }

    #[cfg(target_os = "linux")]
    fn handle_job(&mut self, job: Job) {
        let p = self.p.clone();
        p.lock().ui.lock().busy = true;
        match job {
            Job::Gcode(text) => {
                if !self.print.active {
                    self.ctl.cancel.store(false, Ordering::Relaxed);
                }
                let lines: Vec<&str> = text.lines().collect();
                self.run_script(&lines);
                if !self.print.active {
                    self.core().flush_moves();
                }
            }
            Job::Print(path) => {
                let line = format!("SDCARD_PRINT_FILE FILESLICE=4 FILENAME=\"{}\"", path.display());
                self.single(&line);
            }
            Job::Restart => {
                self.ctl.cancel.store(false, Ordering::Relaxed);
                {
                    let mut c = p.lock();
                    c.shutdown = None;
                    for t in Target::ALL {
                        c.detach(t, "restarting");
                    }
                }
                connect_all(&p);
                self.tmc_connect(None);
                self.reset_last_position();
                self.apply_saved_z_offset();
            }
            Job::Reset(t) => {
                p.lock().detach(t, "resetting");
                connect(&p, t);
                self.tmc_connect(Some(t));
            }
            Job::Probe => {
                let c = p.lock();
                for t in Target::ALL {
                    let msg = match &c.mcus[t as usize] {
                        Some(m) => format!("{}: {:.3} MHz, {} resends, {} KiB sent", t.title(), m.clock.freq / 1e6, m.retransmits, m.bytes_sent / 1024),
                        None => format!("{}: not connected ({})", t.title(), c.links[t as usize].detail),
                    };
                    c.log(msg);
                }
            }
            Job::RefreshFiles => refresh_files(&p),
        }
        if self.print.resume_requested {
            self.print.resume_requested = false;
            self.sd_work();
        }
        p.lock().ui.lock().busy = self.print.active;
    }

    /// `M8233 S<saved z offset> P1`, as `BED_MESH_SET_INDEX` does at boot.
    fn apply_saved_z_offset(&mut self) {
        let zo = {
            let c = self.core();
            let mat = c.cfgs.printer.get_string("bed_mesh", "platform_material", "standard");
            let key = if mat == "standard" { "standard_z_offset" } else { "enhancement_z_offset" };
            c.cfgs.printer.get_double("bed_mesh", key, c.cfgs.printer.get_double("probe", "z_offset", 0.0))
        };
        self.apply_z_offset(zo, true, false);
    }
}

