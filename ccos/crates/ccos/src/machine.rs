//! The Centauri Carbon as ccos drives it: pins, oids, kinematics and limits,
//! taken from the stock `printer.cfg` (see docs/re/02-hardware.md).

use crate::motion::planner::Limits;
use crate::Target;

/// One stepper motor.
#[derive(Debug, Clone, Copy)]
pub struct StepperDef {
    pub name: &'static str,
    pub target: Target,
    pub oid: u8,
    pub step_pin: &'static str,
    pub dir_pin: &'static str,
    /// `!` on dir_pin in printer.cfg.
    pub invert_dir: bool,
    pub enable_oid: u8,
    /// Enable pin; all are active-low (`!PE10` …).
    pub enable_pin: &'static str,
    pub step_dist: f64,
    /// Motor position = kin · [x, y, z, e] (CoreXY: a = x + y, b = x − y).
    pub kin: [f64; 4],
}

pub const E_STEP_DIST: f64 = 28.8 / (200.0 * 16.0 * 5.2);

pub const STEPPERS: [StepperDef; 4] = [
    StepperDef { name: "stepper_x", target: Target::Dsp, oid: dsp::STEP_A, step_pin: "PE8", dir_pin: "PE7", invert_dir: true, enable_oid: dsp::EN_A, enable_pin: "PE10", step_dist: 0.012460, kin: [1.0, 1.0, 0.0, 0.0] },
    StepperDef { name: "stepper_y", target: Target::Dsp, oid: dsp::STEP_B, step_pin: "PE14", dir_pin: "PE13", invert_dir: true, enable_oid: dsp::EN_B, enable_pin: "PE15", step_dist: 0.012460, kin: [1.0, -1.0, 0.0, 0.0] },
    StepperDef { name: "stepper_z", target: Target::Dsp, oid: dsp::STEP_Z, step_pin: "PG3", dir_pin: "PG2", invert_dir: true, enable_oid: dsp::EN_Z, enable_pin: "PG1", step_dist: 0.0025, kin: [0.0, 0.0, 1.0, 0.0] },
    StepperDef { name: "extruder", target: Target::Toolhead, oid: toolhead::STEP_E, step_pin: "PC13", dir_pin: "PC14", invert_dir: false, enable_oid: toolhead::EN_E, enable_pin: "PC15", step_dist: E_STEP_DIST, kin: [0.0, 0.0, 0.0, 1.0] },
];

/// Homing of one axis.
#[derive(Debug, Clone, Copy)]
pub struct AxisDef {
    pub name: char,
    /// Steppers that move this axis (both belts for X and Y on CoreXY).
    pub steppers: &'static [usize],
    pub endstop_oid: u8,
    pub endstop_pin: &'static str,
    /// Pin level that means "triggered" (diag pins are active-low).
    pub triggered_level: u8,
    pub position_endstop: f64,
    pub min: f64,
    pub max: f64,
    pub homing_speed: f64,
    pub homing_accel: f64,
    pub retract: f64,
    pub second_speed: f64,
}

pub const AXES: [AxisDef; 3] = [
    // X/Y: TMC2209 StallGuard on the diag pins ("virtual endstop").
    AxisDef { name: 'X', steppers: &[0, 1], endstop_oid: dsp::ES_X, endstop_pin: "PG4", triggered_level: 0, position_endstop: -2.0, min: -2.01, max: 256.5, homing_speed: 80.0, homing_accel: 5000.0, retract: 0.0, second_speed: 0.0 },
    AxisDef { name: 'Y', steppers: &[0, 1], endstop_oid: dsp::ES_Y, endstop_pin: "PF6", triggered_level: 0, position_endstop: -2.5, min: -2.51, max: 270.0, homing_speed: 80.0, homing_accel: 5000.0, retract: 0.0, second_speed: 0.0 },
    // Z: PG8 (bed contact). ccos defines z = 0 at contact; the stock host
    // uses position_endstop 4.5 plus load-cell offsets.
    AxisDef { name: 'Z', steppers: &[2], endstop_oid: dsp::ES_Z, endstop_pin: "PG8", triggered_level: 1, position_endstop: 0.0, min: 0.0, max: 258.0, homing_speed: 8.0, homing_accel: 500.0, retract: 10.0, second_speed: 3.0 },
];

/// Where Z is homed (bed centre).
pub const Z_HOME_XY: [f64; 2] = [128.0, 128.0];

pub const LIMITS: Limits = Limits {
    max_velocity: 500.0,
    max_accel: 20000.0,
    max_accel_to_decel: 20000.0,
    square_corner_velocity: 9.0,
    max_z_velocity: 20.0,
    max_z_accel: 500.0,
    max_extrude_only_velocity: 60.0,
    max_extrude_only_accel: 5000.0,
    instant_corner_v: 1.0,
};

pub const MIN_EXTRUDE_TEMP: f64 = 170.0;
/// Moves start this long after they are committed from idle.
pub const START_DELAY: f64 = 0.25;
/// Steps are generated ahead of their time: refilled when less than the
/// first value is buffered, up to the second (homing: less, so the stop
/// after a trigger is quick).
pub const GEN_AHEAD: (f64, f64) = (0.25, 0.5);
pub const GEN_AHEAD_HOMING: (f64, f64) = (0.05, 0.1);
/// Planned motion buffered at most (G-code reading pauses beyond this).
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
    pub const LIGHT: u8 = 13;
    pub const AUX_FAN: u8 = 14;
    pub const CHAMBER_FAN: u8 = 15;
    pub const BOARD_FAN: u8 = 16;
    pub const COUNT: u8 = 17;
}

/// oids on the toolhead STM32.
pub mod toolhead {
    pub const STEP_E: u8 = 0;
    pub const EN_E: u8 = 1;
    pub const HOTEND_HEATER: u8 = 2;
    pub const ADC_HOTEND: u8 = 3;
    pub const PART_FAN: u8 = 4;
    pub const HOTEND_FAN: u8 = 5;
    pub const COUNT: u8 = 6;
}

/// ADC: 8 samples 1 ms apart, reported every 300 ms (Klipper's defaults).
pub const ADC_SAMPLES: u32 = 8;
pub const ADC_SAMPLE_TIME: f64 = 0.001;
pub const ADC_REPORT_TIME: f64 = 0.3;
/// Out-of-range readings in a row before the MCU shuts down.
pub const ADC_RANGE_CHECKS: u32 = 4;
/// Soft-PWM period for fans and the light.
pub const FAN_CYCLE_TIME: f64 = 0.01;
