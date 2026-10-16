pub mod dashboard;
pub mod mcu;

/// Where the stock firmware keeps things on the printer.
pub mod paths {
    pub const RESOURCES: &str = "/app/resources";
    pub const FONT: &str = "/app/resources/fonts/font.ttc";
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
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "dsp" | "mcu" | "main" => Target::Dsp,
            "toolhead" | "stm32" | "extruder" => Target::Toolhead,
            "sg" | "strain_gauge" | "loadcell" => Target::StrainGauge,
            _ => return None,
        })
    }
    pub fn label(self) -> &'static str {
        match self {
            Target::Dsp => "dsp",
            Target::Toolhead => "toolhead",
            Target::StrainGauge => "sg",
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
            Target::Toolhead => Some(4 * 32 + 12),    // PE12
            Target::StrainGauge => Some(6 * 32 + 9), // PG9
        }
    }

    /// Power-cycle an STM32 board exactly like `MCU::mcu_identify` (sysfs
    /// GPIO low 1 s, high, then wait for it to boot / re-enumerate). Returns
    /// it to an unconfigured state.
    #[cfg(target_os = "linux")]
    pub fn power_cycle(self) -> std::io::Result<()> {
        use std::fs::write;
        use std::thread::sleep;
        use std::time::Duration;
        let Some(n) = self.power_gpio() else {
            return Err(std::io::Error::other("the DSP is restarted via remoteproc, not a power pin"));
        };
        let base = format!("/sys/class/gpio/gpio{n}");
        if !std::path::Path::new(&base).exists() {
            write("/sys/class/gpio/export", n.to_string())?;
        }
        write(format!("{base}/direction"), "out")?;
        write(format!("{base}/value"), "0")?;
        sleep(Duration::from_secs(1));
        write(format!("{base}/value"), "1")?;
        // The stock host waits 2 s, plus 3 s more for the USB toolhead.
        sleep(Duration::from_secs(if self == Target::Toolhead { 5 } else { 2 }));
        Ok(())
    }

    #[cfg(target_os = "linux")]
    pub fn open(self) -> std::io::Result<Box<dyn cc_transport::Link>> {
        use cc_transport::serial::*;
        Ok(match self {
            Target::Dsp => Box::new(cc_transport::dsp::DspLink::open(std::time::Duration::from_secs(3))?),
            Target::Toolhead => Box::new(SerialLink::open(TOOLHEAD_PORT, TOOLHEAD_BAUD)?),
            Target::StrainGauge => Box::new(SerialLink::open(STRAIN_GAUGE_PORT, STRAIN_GAUGE_BAUD)?),
        })
    }
}
