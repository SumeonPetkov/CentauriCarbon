//! `ccos`: a lightweight native host for the Elegoo Centauri Carbon.
//!
//! Threads: the UI (`runtime`), the printer core I/O loop (`core`), and the
//! G-code thread (`gcode`). Talks to real hardware or to `ccsim` (`hw`).

pub mod accel;
pub mod app;
pub mod breaksave;
pub mod clock;
pub mod config;
pub mod core;
pub mod gcode;
pub mod heater;
pub mod hw;
pub mod loadcell;
pub mod mesh;
pub mod resonance;
pub mod machine;
pub mod mcu;
pub mod motion;
pub mod runtime;
pub mod sys;
pub mod tmc;

/// Screens, fonts and icons compiled from `ccos/ui/` by `build.rs`.
pub mod ui {
    include!(concat!(env!("OUT_DIR"), "/ui.rs"));
}

/// Where the stock firmware keeps things on the printer.
pub mod paths {
    pub const FRAMEBUFFER: &str = "/dev/fb0";
    pub const TOUCH: &str = "/dev/input/event1";
}

/// Which MCU, with its stock link parameters (printer.cfg `[mcu*]`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target {
    Dsp,
    Toolhead,
    StrainGauge,
}

impl Target {
    pub const ALL: [Target; 3] = [Target::Dsp, Target::Toolhead, Target::StrainGauge];

    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "dsp" | "mcu" | "main" => Target::Dsp,
            "toolhead" | "stm32" | "extruder" => Target::Toolhead,
            "sg" | "strain_gauge" | "loadcell" | "load_cell" | "bed" => Target::StrainGauge,
            _ => return None,
        })
    }

    pub fn label(self) -> &'static str {
        match self {
            Target::Dsp => "dsp",
            Target::Toolhead => "toolhead",
            Target::StrainGauge => "loadcell",
        }
    }

    pub fn title(self) -> &'static str {
        match self {
            Target::Dsp => "DSP",
            Target::Toolhead => "Toolhead",
            Target::StrainGauge => "Load cell",
        }
    }

    /// Pin numbering stride used on the wire.
    pub fn pins_per_bank(self) -> u32 {
        if self == Target::Dsp { 32 } else { 16 }
    }

    pub fn nominal_clock(self) -> f64 {
        if self == Target::Dsp { cc_proto::clock::DSP_CLOCK_FREQ } else { cc_proto::clock::STM32_CLOCK_FREQ }
    }

    /// R528 GPIO (sysfs number = bank * 32 + n) that powers this board
    /// (printer.cfg `power_pin`). The DSP has none.
    pub fn power_gpio(self) -> Option<u32> {
        match self {
            Target::Dsp => None,
            Target::Toolhead => Some(4 * 32 + 12),   // PE12
            Target::StrainGauge => Some(6 * 32 + 9), // PG9
        }
    }
}
