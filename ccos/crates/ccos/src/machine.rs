//! The machine as ccos drives it, read from `printer.cfg` (stock values
//! when no file is present) the way Elegoo's firmware reads it: pins,
//! kinematics limits, homing, heaters, fans, pause and filament positions.
//! Fixed parts (oids, pin roles of the CoreXY layout) are constants.

use crate::config::{parse_double, Config};
use crate::heater::HeaterConfig;
use crate::motion::planner::Limits;
use crate::Target;

/// A pin from printer.cfg: `!stm32:PC15` → toolhead, inverted, `PC15`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pin {
    pub target: Target,
    pub invert: bool,
    pub name: String,
}

pub fn parse_pin(s: &str) -> Pin {
    let mut s = s.trim();
    let mut invert = false;
    loop {
        if let Some(r) = s.strip_prefix('!') {
            invert = !invert;
            s = r;
        } else if let Some(r) = s.strip_prefix('^').or_else(|| s.strip_prefix('~')) {
            s = r;
        } else {
            break;
        }
    }
    let (target, name) = match s.split_once(':') {
        Some(("stm32", p)) => (Target::Toolhead, p),
        Some(("strain_gauge_mcu", p)) => (Target::StrainGauge, p),
        Some((_, p)) => (Target::Dsp, p),
        None => (Target::Dsp, s),
    };
    Pin { target, invert, name: name.trim().to_string() }
}

/// One stepper motor.
#[derive(Debug, Clone)]
pub struct StepperDef {
    pub name: &'static str,
    pub target: Target,
    pub oid: u8,
    pub step: Pin,
    pub dir: Pin,
    pub enable_oid: u8,
    pub enable: Pin,
    pub step_dist: f64,
    /// Motor position = kin · [x, y, z, e] (CoreXY: a = x + y, b = x − y).
    pub kin: [f64; 4],
}

/// Homing of one axis (`[stepper_*]`).
#[derive(Debug, Clone)]
pub struct AxisDef {
    pub name: char,
    pub steppers: &'static [usize],
    pub endstop_oid: u8,
    pub endstop: Pin,
    /// Coordinate assigned at the endstop (Z: changed at runtime by M8233).
    pub position_endstop: f64,
    pub position_endstop_extra: f64,
    pub min: f64,
    pub max: f64,
    pub homing_speed: f64,
    pub second_homing_speed: f64,
    pub retract_dist: f64,
    pub retract_speed: f64,
    pub force_retract: f64,
    pub homing_accel: Option<f64>,
    pub homing_current: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct SafeZHome {
    pub x: f64,
    pub y: f64,
    pub speed: f64,
    pub z_hop: f64,
    pub z_lift: f64,
    pub z_hop_speed: f64,
    pub temp: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ExtruderCfg {
    pub min_extrude_temp: f64,
    pub max_extrude_only_velocity: f64,
    pub max_extrude_only_accel: f64,
    pub max_extrude_only_distance: f64,
    pub nozzle_diameter: f64,
    pub filament_area: f64,
    pub max_extrude_ratio: f64,
    pub pressure_advance: f64,
    pub pressure_advance_smooth_time: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct PauseCfg {
    pub x: f64,
    pub y: f64,
    pub rel_z: f64,
    pub move_speed: f64,
    pub extruder_timeout: f64,
    pub extruder_temp: f64,
    pub fan_timeout: f64,
    pub fan_speed: f64,
    pub resume_fan_speed: f64,
    pub recover_velocity: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ChangeFilamentCfg {
    pub min_x: f64,
    pub max_y: f64,
    pub pos_x: f64,
    pub pos_y: f64,
    pub extrude_fan_speed: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct FanCfg {
    pub pin_oid: (Target, u8),
    pub max_power: f64,
    pub kick_start_time: f64,
    pub off_below: f64,
    pub cycle_time: f64,
    pub startup_voltage: f64,
    pub shutdown_speed: f64,
}

#[derive(Debug, Clone)]
pub struct Machine {
    pub steppers: [StepperDef; 4],
    pub axes: [AxisDef; 3],
    pub limits: Limits,
    pub safe_z: SafeZHome,
    pub extruder: ExtruderCfg,
    pub heaters: [HeaterConfig; 2],
    pub pause: PauseCfg,
    pub change_filament: ChangeFilamentCfg,
    pub arc_resolution: f64,
    /// `[fan model]`, `[fan model_helper_fan]`, `[fan box_fan]` in file order (M106 P1..P3).
    pub fans: [FanCfg; 3],
    pub hotend_fan: FanCfg,
    pub hotend_fan_temp: f64,
    pub board_fan: FanCfg,
    pub board_fan_idle_timeout: f64,
    pub pid_test_point: [f64; 3],
    pub pid_extruder_temp: f64,
    pub pid_bed_temp: f64,
    pub idle_timeout: f64,
    pub led_pins: [Pin; 2],
    /// One bit-banged TMC UART per MCU (`MCU_TMC_uart_bitbang`, shared by
    /// the drivers on the same pins).
    pub tmc_uarts: Vec<TmcUart>,
    /// `[hx711s]`, `[strain_gauge]`, `[dirzctl] step_base`: each object
    /// exists only with its section.
    pub hx711s: Option<crate::loadcell::Hx711sCfg>,
    pub strain_gauge: Option<crate::loadcell::StrainGaugeCfg>,
    pub dirzctl_step_base: Option<f64>,
    /// `[adxl345 X]` on the toolhead.
    pub adxl: Option<crate::accel::AdxlCfg>,
    /// `[resonance_tester]` (loaded with or without the section).
    pub resonance: crate::resonance::ResonanceCfg,
}

#[derive(Debug, Clone)]
pub struct TmcUart {
    pub target: Target,
    pub rx: Pin,
    pub tx: Pin,
    pub pull_up: bool,
}

fn tmc_uarts(c: &Config) -> Vec<TmcUart> {
    let mut v: Vec<TmcUart> = Vec::new();
    for sec in crate::tmc::sections(c) {
        let rx_s = c.get_string(&sec, "uart_pin", "");
        let rx = parse_pin(&rx_s);
        let tx_s = c.get_string(&sec, "tx_pin", "");
        let tx = if tx_s.is_empty() { rx.clone() } else { parse_pin(&tx_s) };
        if !v.iter().any(|u| u.target == rx.target) {
            v.push(TmcUart { target: rx.target, pull_up: rx_s.contains('^'), rx, tx });
        }
    }
    v
}

fn axis(c: &Config, sec: &str, name: char, steppers: &'static [usize], oid: u8, endstop_default: &str) -> AxisDef {
    let pe = c.get_double(sec, "position_endstop", 0.0);
    let speed = c.get_double(sec, "homing_speed", 5.0);
    let opt = |k: &str| c.get(sec, k).filter(|v| !v.is_empty()).map(parse_double);
    let endstop = c.get_string(sec, "endstop_pin", endstop_default);
    // `tmc2209_stepper_x:virtual_endstop` → the driver's diag pin
    let endstop = if endstop.contains("virtual_endstop") { c.get_string(&format!("tmc2209 {sec}"), "diag_pin", endstop_default) } else { endstop };
    AxisDef {
        name,
        steppers,
        endstop_oid: oid,
        endstop: parse_pin(&endstop),
        position_endstop: pe,
        position_endstop_extra: c.get_double(sec, "position_endstop_extra", 0.0),
        min: c.get_double(sec, "position_min", 0.0),
        max: c.get_double(sec, "position_max", 0.0),
        homing_speed: speed,
        second_homing_speed: c.get_double(sec, "second_homing_speed", speed / 2.0),
        retract_dist: c.get_double(sec, "homing_retract_dist", 5.0),
        retract_speed: c.get_double(sec, "homing_retract_speed", speed),
        force_retract: c.get_double(sec, "homing_force_retract", 0.0),
        homing_accel: opt("homing_accel"),
        homing_current: opt("homing_current"),
    }
}

fn stepper(c: &Config, sec: &'static str, oid: u8, enable_oid: u8, kin: [f64; 4]) -> StepperDef {
    let step = parse_pin(&c.get_string(sec, "step_pin", ""));
    let step_dist = match c.get(sec, "step_distance") {
        Some(v) => parse_double(v),
        None => {
            // rotation_distance / (full_steps * microsteps * gear ratio)
            let rd = c.get_double(sec, "rotation_distance", 1.0);
            let fs = c.get_double(sec, "full_steps_per_rotation", 200.0);
            let ms = c.get_double(sec, "microsteps", 16.0);
            let gr = c.get_string(sec, "gear_ratio", "1:1");
            let ratio = gr.split(',').map(|p| p.split_once(':').map_or(1.0, |(a, b)| parse_double(a) / parse_double(b))).product::<f64>();
            rd / (fs * ms * ratio)
        }
    };
    StepperDef {
        name: sec,
        target: step.target,
        oid,
        dir: parse_pin(&c.get_string(sec, "dir_pin", "")),
        enable: parse_pin(&c.get_string(sec, "enable_pin", "")),
        step,
        enable_oid,
        step_dist,
        kin,
    }
}

fn fan(c: &Config, sec: &str, place: (Target, u8), shutdown_default: f64) -> FanCfg {
    FanCfg {
        pin_oid: place,
        max_power: c.get_double(sec, "max_power", 1.0),
        kick_start_time: c.get_double(sec, "kick_start_time", 0.1),
        off_below: c.get_double(sec, "off_below", 0.0),
        cycle_time: c.get_double(sec, "cycle_time", 0.010),
        startup_voltage: c.get_double(sec, "startup_voltage", 0.0),
        shutdown_speed: c.get_double(sec, "shutdown_speed", shutdown_default),
    }
}

fn heater(c: &Config, sec: &'static str, check: &str, gain_default: f64, beta_default: f64) -> HeaterConfig {
    let beta = c.get_string(sec, "sensor_type", "").rsplit(' ').next().map(parse_double).filter(|b| *b > 0.0).unwrap_or(beta_default);
    HeaterConfig {
        name: sec,
        beta,
        min_temp: c.get_double(sec, "min_temp", 0.0),
        max_temp: c.get_double(sec, "max_temp", 300.0),
        kp: c.get_double(sec, "pid_Kp", 0.0),
        ki: c.get_double(sec, "pid_Ki", 0.0),
        kd: c.get_double(sec, "pid_Kd", 0.0),
        max_power: c.get_double(sec, "max_power", 1.0),
        smooth_time: c.get_double(sec, "smooth_time", 1.0),
        pwm_cycle_time: c.get_double(sec, "pwm_cycle_time", 0.100),
        min_extrude_temp: c.get_double(sec, "min_extrude_temp", 170.0),
        max_error: c.get_double(check, "max_error", 120.0),
        check_gain_time: c.get_double(check, "check_gain_time", gain_default),
        hysteresis: c.get_double(check, "hysteresis", 5.0),
        heating_gain: c.get_double(check, "heating_gain", 2.0),
    }
}

impl Machine {
    pub fn from_config(c: &Config) -> Machine {
        let steppers = [
            stepper(c, "stepper_x", dsp::STEP_A, dsp::EN_A, [1.0, 1.0, 0.0, 0.0]),
            stepper(c, "stepper_y", dsp::STEP_B, dsp::EN_B, [1.0, -1.0, 0.0, 0.0]),
            stepper(c, "stepper_z", dsp::STEP_Z, dsp::EN_Z, [0.0, 0.0, 1.0, 0.0]),
            stepper(c, "extruder", toolhead::STEP_E, toolhead::EN_E, [0.0, 0.0, 0.0, 1.0]),
        ];
        let max_accel = c.get_double("printer", "max_accel", 3000.0);
        let nozzle = c.get_double("extruder", "nozzle_diameter", 0.4);
        let fil_d = c.get_double("extruder", "filament_diameter", 1.75);
        let filament_area = std::f64::consts::PI * (fil_d * 0.5).powi(2);
        let cross = c.get_double("extruder", "max_extrude_cross_section", 4.0 * nozzle * nozzle);
        Machine {
            axes: [
                axis(c, "stepper_x", 'X', &[0, 1], dsp::ES_X, "!PG4"),
                axis(c, "stepper_y", 'Y', &[0, 1], dsp::ES_Y, "!PF6"),
                axis(c, "stepper_z", 'Z', &[2], dsp::ES_Z, "PG8"),
            ],
            steppers,
            limits: Limits {
                max_velocity: c.get_double("printer", "max_velocity", 500.0),
                max_accel,
                max_accel_to_decel: c.get_double("printer", "max_accel_to_decel", max_accel * 0.5).min(max_accel),
                square_corner_velocity: c.get_double("printer", "square_corner_velocity", 5.0),
                max_z_velocity: c.get_double("printer", "max_z_velocity", 20.0),
                max_z_accel: c.get_double("printer", "max_z_accel", max_accel),
                max_extrude_only_velocity: c.get_double("extruder", "max_extrude_only_velocity", 60.0),
                max_extrude_only_accel: c.get_double("extruder", "max_extrude_only_accel", 5000.0),
                instant_corner_v: c.get_double("extruder", "instantaneous_corner_velocity", 1.0),
            },
            safe_z: SafeZHome {
                x: c.get_double("safe_z_home", "home_x_position", 130.0),
                y: c.get_double("safe_z_home", "home_y_position", 130.0),
                speed: c.get_double("safe_z_home", "speed", 50.0),
                z_hop: c.get_double("safe_z_home", "z_hop", 0.0),
                z_lift: c.get_double("safe_z_home", "z_lift", 10.0),
                z_hop_speed: c.get_double("safe_z_home", "z_hop_speed", 15.0),
                temp: c.get_double("safe_z_home", "temp", 140.0),
            },
            extruder: ExtruderCfg {
                min_extrude_temp: c.get_double("extruder", "min_extrude_temp", 170.0),
                max_extrude_only_velocity: c.get_double("extruder", "max_extrude_only_velocity", 60.0),
                max_extrude_only_accel: c.get_double("extruder", "max_extrude_only_accel", 5000.0),
                max_extrude_only_distance: c.get_double("extruder", "max_extrude_only_distance", 60.0),
                nozzle_diameter: nozzle,
                filament_area,
                max_extrude_ratio: cross / filament_area,
                pressure_advance: c.get_double("extruder", "pressure_advance", 0.0),
                pressure_advance_smooth_time: c.get_double("extruder", "pressure_advance_smooth_time", 0.040),
            },
            heaters: [heater(c, "extruder", "verify_heater extruder", 20.0, 4300.0), heater(c, "heater_bed", "verify_heater heater_bed", 60.0, 3950.0)],
            pause: PauseCfg {
                x: c.get_double("pause_resume", "pause_abs_pos_x", 202.0),
                y: c.get_double("pause_resume", "pause_abs_pos_y", 264.5),
                rel_z: c.get_double("pause_resume", "pause_rel_pos_z", 10.0),
                move_speed: c.get_double("pause_resume", "pause_move_speed", 3000.0),
                extruder_timeout: c.get_double("pause_resume", "pause_extruder_timeout", 300.0),
                extruder_temp: c.get_double("pause_resume", "pause_extruder_temp", 45.0),
                fan_timeout: c.get_double("pause_resume", "pause_fan_timeout", 300.0),
                fan_speed: c.get_double("pause_resume", "pause_fan_speed", 0.0),
                resume_fan_speed: c.get_double("pause_resume", "resume_extrude_fan_speed", 255.0),
                recover_velocity: c.get_double("pause_resume", "recover_velocity", 50.0),
            },
            change_filament: ChangeFilamentCfg {
                min_x: c.get_double("change_filament", "min_x", 246.0),
                max_y: c.get_double("change_filament", "max_y", 20.0),
                pos_x: c.get_double("change_filament", "pos_x", 202.0),
                pos_y: c.get_double("change_filament", "pos_y", 264.5),
                extrude_fan_speed: c.get_double("change_filament", "extrude_fan_speed", 77.0),
            },
            arc_resolution: c.get_double("gcode_arcs", "resolution", 1.0),
            fans: [
                fan(c, "fan model", (Target::Toolhead, toolhead::PART_FAN), 0.0),
                fan(c, "fan model_helper_fan", (Target::Dsp, dsp::AUX_FAN), 0.0),
                fan(c, "fan box_fan", (Target::Dsp, dsp::CHAMBER_FAN), 0.0),
            ],
            hotend_fan: fan(c, "heater_fan hotend_cooling_fan", (Target::Toolhead, toolhead::HOTEND_FAN), 1.0),
            hotend_fan_temp: c.get_double("heater_fan hotend_cooling_fan", "heater_temp", 50.0),
            board_fan: fan(c, "controller_fan board_cooling_fan", (Target::Dsp, dsp::BOARD_FAN), 0.0),
            board_fan_idle_timeout: c.get_double("controller_fan board_cooling_fan", "idle_timeout", 30.0),
            pid_test_point: {
                let v = c.get_doubles("pid_calibrate", "test_points");
                [v.first().copied().unwrap_or(128.0), v.get(1).copied().unwrap_or(128.0), v.get(2).copied().unwrap_or(3.0)]
            },
            pid_extruder_temp: c.get_double("pid_calibrate", "extruder_target_temp", 220.0),
            pid_bed_temp: c.get_double("pid_calibrate", "heater_bed_target_temp", 60.0),
            idle_timeout: c.get_double("idle_timeout", "timeout", 1800.0),
            led_pins: [parse_pin(&c.get_string("led led1", "red_pin", "stm32:PC9")), parse_pin(&c.get_string("led led2", "red_pin", "PG15"))],
            tmc_uarts: tmc_uarts(c),
            hx711s: crate::loadcell::Hx711sCfg::from_config(c),
            strain_gauge: crate::loadcell::StrainGaugeCfg::from_config(c),
            dirzctl_step_base: c.has_section("dirzctl").then(|| c.get_double("dirzctl", "step_base", 2.0)),
            adxl: crate::accel::AdxlCfg::from_config(c).filter(|a| a.cs_pin.target == Target::Toolhead),
            resonance: crate::resonance::ResonanceCfg::from_config(c),
        }
    }

    pub fn stock() -> Machine {
        Machine::from_config(&Config::parse(crate::config::STOCK_PRINTER_CFG))
    }
}

/// Moves start this long after they are committed from idle (Klipper's
/// `buffer_time_start`).
pub const START_DELAY: f64 = 0.25;
/// Steps are generated ahead of their time: refilled when less than the
/// first value is buffered, up to the second (homing: less, so the stop
/// after a trigger is quick).
pub const GEN_AHEAD: (f64, f64) = (0.25, 0.5);
/// Homing: the stock drip mode (`update_drip_move_time`) sends steps once
/// they are less than DRIP_TIME + move_flush_time + kin_flush_delay away,
/// DRIP_SEGMENT_TIME at a time. This is DRIP_TIME + move_flush_time.
pub const DRIP_DELAY: f64 = 0.100 + 0.05;
pub const DRIP_SEGMENT_TIME: f64 = 0.050;
/// Planned motion buffered at most (`buffer_time_high`).
pub const BUFFER_TIME_HIGH: f64 = 2.0;
/// Klipper's default `max_stepper_error`.
pub const MAX_STEP_ERROR: f64 = 25e-6;
pub const STEP_PULSE: f64 = 200e-9;
/// Move-queue slots kept free for heater/fan updates.
pub const RESERVED_SLOTS: usize = 64;
pub const MOVE_COUNT: usize = 1024;

/// oids on the DSP.
pub mod dsp {
    pub const STEP_A: u8 = 0;
    pub const STEP_B: u8 = 1;
    pub const STEP_Z: u8 = 2;
    pub const EN_A: u8 = 3;
    pub const EN_B: u8 = 4;
    pub const EN_Z: u8 = 5;
    pub const ES_X: u8 = 6;
    pub const ES_Y: u8 = 7;
    pub const ES_Z: u8 = 8;
    pub const TRSYNC: u8 = 9;
    pub const BED_HEATER: u8 = 10;
    pub const ADC_BED: u8 = 11;
    pub const ADC_CHAMBER: u8 = 12;
    /// `[led led2]` red_pin PG15 (chamber light).
    pub const LIGHT: u8 = 13;
    pub const AUX_FAN: u8 = 14;
    pub const CHAMBER_FAN: u8 = 15;
    pub const BOARD_FAN: u8 = 16;
    /// TMC2209 UART shared by stepper_x/y/z (tx PE9, rx PE6).
    pub const TMCUART: u8 = 17;
    /// `[dirzctl]`: Z stepped directly for load-cell probing.
    pub const DIRZCTL: u8 = 18;
    pub const COUNT: u8 = 19;
}

/// oids on the load-cell board.
pub mod strain_gauge {
    pub const HX711S: u8 = 0;
    pub const COUNT: u8 = 1;
}

/// oids on the toolhead STM32.
pub mod toolhead {
    pub const STEP_E: u8 = 0;
    pub const EN_E: u8 = 1;
    pub const HOTEND_HEATER: u8 = 2;
    pub const ADC_HOTEND: u8 = 3;
    pub const PART_FAN: u8 = 4;
    pub const HOTEND_FAN: u8 = 5;
    /// `[led led1]` red_pin stm32:PC9.
    pub const LED: u8 = 6;
    /// TMC2209 UART of the extruder (tx PC6, rx PC7).
    pub const TMCUART: u8 = 7;
    /// `[adxl345 X]`: its SPI device (CS PA4 on SPI1) and the LIS2DW12.
    pub const ACCEL_SPI: u8 = 8;
    pub const ACCEL: u8 = 9;
    pub const COUNT: u8 = 10;
}

/// ADC (Define.h): 8 samples 1 ms apart, reported every 0.5 s.
pub const ADC_SAMPLES: u32 = 8;
pub const ADC_SAMPLE_TIME: f64 = 0.001;
pub const ADC_REPORT_TIME: f64 = 0.5;
pub const ADC_RANGE_CHECKS: u32 = 4;
/// LED soft-PWM period (`[led] cycle_time`).
pub const LED_CYCLE_TIME: f64 = 0.016;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stock_machine_matches_printer_cfg() {
        let m = Machine::stock();
        assert_eq!(m.steppers[0].step, Pin { target: Target::Dsp, invert: false, name: "PE8".into() });
        assert!(m.steppers[0].dir.invert);
        assert_eq!(m.steppers[3].target, Target::Toolhead);
        assert!((m.steppers[3].step_dist - 28.8 / (200.0 * 16.0 * 5.2)).abs() < 1e-12);
        assert_eq!(m.axes[0].endstop, Pin { target: Target::Dsp, invert: true, name: "PG4".into() });
        assert_eq!(m.axes[2].position_endstop, 4.5);
        assert_eq!(m.axes[0].force_retract, 20.0);
        assert_eq!(m.axes[2].second_homing_speed, 3.0);
        assert_eq!(m.axes[0].homing_accel, Some(5000.0));
        assert_eq!(m.axes[2].homing_accel, None);
        assert_eq!(m.limits.max_velocity, 500.0);
        assert_eq!(m.arc_resolution, 0.1);
        assert_eq!(m.heaters[0].min_temp, -200.0);
        assert_eq!(m.fans[1].startup_voltage, 0.5);
        assert_eq!(m.safe_z.temp, 140.0);
        assert!((m.extruder.max_extrude_ratio - 0.26608).abs() < 1e-4);
    }
}
