pub mod app;
pub mod mcu;
pub mod sys;

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

    /// Serial device and baud rate of this board.
    pub fn port(self) -> Option<(&'static str, u32)> {
        use cc_transport::serial::*;
        match self {
            Target::Dsp => None,
            Target::Toolhead => Some((TOOLHEAD_PORT, TOOLHEAD_BAUD)),
            Target::StrainGauge => Some((STRAIN_GAUGE_PORT, STRAIN_GAUGE_BAUD)),
        }
    }

    /// Power-cycle an STM32 board and boot its existing firmware, leaving it
    /// unconfigured.
    ///
    /// Sequence: power pin low 1 s, high (as `MCU::mcu_identify`). Elegoo's
    /// bootloader then waits **indefinitely** for a YMODEM upload; an `a`
    /// byte (ABORT2 in `mcu/bootloader/Utils/Src/ymodem.c`) makes it jump to
    /// the application. The toolhead re-enumerates on USB afterwards.
    #[cfg(target_os = "linux")]
    pub fn power_cycle(self) -> std::io::Result<()> {
        use cc_transport::Link;
        use std::fs::write;
        use std::thread::sleep;
        use std::time::{Duration, Instant};
        let (Some(n), Some((port, baud))) = (self.power_gpio(), self.port()) else {
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

        let wait_port = |t: Duration| -> std::io::Result<()> {
            let deadline = Instant::now() + t;
            while !std::path::Path::new(port).exists() {
                if Instant::now() > deadline {
                    return Err(std::io::Error::new(std::io::ErrorKind::TimedOut, format!("{port} did not appear")));
                }
                sleep(Duration::from_millis(100));
            }
            Ok(())
        };
        // Bootloader up (USB CDC enumeration for the toolhead).
        sleep(Duration::from_millis(500));
        wait_port(Duration::from_secs(10))?;
        let mut link = cc_transport::serial::SerialLink::open(port, baud)?;
        link.send(&[b'a'; 16])?;
        drop(link);
        // Application up; the USB toolhead drops and re-creates ttyACM0.
        sleep(Duration::from_secs(if self == Target::Toolhead { 3 } else { 1 }));
        wait_port(Duration::from_secs(10))
    }

    /// Open the link. For the DSP, `restart_dsp` allows a remoteproc restart
    /// when another host attached to it earlier in this boot.
    #[cfg(target_os = "linux")]
    pub fn open(self, restart_dsp: bool) -> std::io::Result<Box<dyn cc_transport::Link>> {
        use cc_transport::serial::SerialLink;
        Ok(match self {
            Target::Dsp => Box::new(cc_transport::dsp::DspLink::open(restart_dsp, std::time::Duration::from_secs(5))?),
            _ => {
                let (port, baud) = self.port().unwrap();
                Box::new(SerialLink::open(port, baud)?)
            }
        })
    }
}
