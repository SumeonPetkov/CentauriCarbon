//! Where the MCUs, power pins, screen and G-code files are: the real
//! printer, or a simulation (`ccsim`).
//!
//! Defaults are the stock printer's. Environment overrides:
//!
//! | Variable | Meaning |
//! |---|---|
//! | `CCOS_SIM=<dir>` | Use a `ccsim` virtual printer in `<dir>` (sets all below) |
//! | `CCOS_DSP_PORT=<tty>` | DSP as a serial port instead of the shared-memory ring |
//! | `CCOS_TOOLHEAD_PORT`, `CCOS_LOADCELL_PORT` | Serial ports of the STM32 boards |
//! | `CCOS_GPIO_ROOT` | sysfs GPIO directory (power pins) |
//! | `CCOS_GCODE_DIR` | Where printable files are listed from |
//! | `CCOS_CONFIG_DIR` | Folder with printer.cfg / user_printer.cfg / unmodifiable.cfg |
//! | `CCOS_TMP_DIR` | Where stock's `/tmp` files go (accelerometer CSVs) |
//! | `CCOS_ENABLE_MOTION=1` | Allow heaters and motors on real hardware (see below) |
//!
//! **Motion lock.** On the real printer, heaters and motors stay disabled
//! unless explicitly enabled: the printing core is tested against the
//! simulator only, and real X/Y homing also needs TMC2209 StallGuard set up
//! over UART, which ccos does not do yet.

use std::io;
use std::path::{Path, PathBuf};
use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::Target;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Endpoint {
    /// The R528 DSP's shared-memory ring and rpmsg doorbell.
    DspKernel,
    Serial { path: String, baud: u32 },
}

#[derive(Debug, Clone)]
pub struct Hardware {
    pub endpoints: [Endpoint; 3],
    pub gpio_root: PathBuf,
    /// Restart the DSP through remoteproc if another host attached this boot.
    pub restart_dsp: bool,
    /// Heaters and motors may be configured.
    pub motion: bool,
    pub gcode_dir: PathBuf,
    /// `/board-resource`: printer.cfg, user_printer.cfg, unmodifiable.cfg.
    pub config_dir: Option<PathBuf>,
    /// Talking to `ccsim`: shorter bootloader/USB delays, motion allowed.
    pub simulated: bool,
    /// `/tmp`.
    pub tmp_dir: PathBuf,
}

impl Hardware {
    /// The stock printer.
    pub fn printer(restart_dsp: bool) -> Self {
        use cc_transport::serial::*;
        Self {
            endpoints: [
                Endpoint::DspKernel,
                Endpoint::Serial { path: TOOLHEAD_PORT.into(), baud: TOOLHEAD_BAUD },
                Endpoint::Serial { path: STRAIN_GAUGE_PORT.into(), baud: STRAIN_GAUGE_BAUD },
            ],
            gpio_root: "/sys/class/gpio".into(),
            restart_dsp,
            motion: false,
            gcode_dir: "/user-resource".into(),
            config_dir: Some("/board-resource".into()),
            simulated: false,
            tmp_dir: "/tmp".into(),
        }
    }

    /// A `ccsim` virtual printer whose ptys and GPIO tree live in `dir`.
    pub fn sim(dir: &Path) -> Self {
        use cc_transport::serial::*;
        Self {
            endpoints: [
                Endpoint::Serial { path: dir.join("dsp").display().to_string(), baud: 1_000_000 },
                Endpoint::Serial { path: dir.join("ttyACM0").display().to_string(), baud: TOOLHEAD_BAUD },
                Endpoint::Serial { path: dir.join("ttyS4").display().to_string(), baud: STRAIN_GAUGE_BAUD },
            ],
            gpio_root: dir.join("gpio"),
            restart_dsp: false,
            motion: true,
            gcode_dir: dir.join("gcodes"),
            config_dir: Some(dir.join("board-resource")),
            simulated: true,
            tmp_dir: dir.join("tmp"),
        }
    }

    /// Stock printer, or a simulation, adjusted by `CCOS_*` variables.
    pub fn from_env(restart_dsp: bool) -> Self {
        let var = |k: &str| std::env::var(k).ok().filter(|v| !v.is_empty());
        let mut hw = match var("CCOS_SIM") {
            Some(dir) => Self::sim(Path::new(&dir)),
            None => Self::printer(restart_dsp),
        };
        for (i, k) in ["CCOS_DSP_PORT", "CCOS_TOOLHEAD_PORT", "CCOS_LOADCELL_PORT"].iter().enumerate() {
            if let Some(p) = var(k) {
                let baud = match &hw.endpoints[i] {
                    Endpoint::Serial { baud, .. } => *baud,
                    Endpoint::DspKernel => 1_000_000,
                };
                hw.endpoints[i] = Endpoint::Serial { path: p, baud };
            }
        }
        if let Some(g) = var("CCOS_GPIO_ROOT") {
            hw.gpio_root = g.into();
        }
        if let Some(g) = var("CCOS_GCODE_DIR") {
            hw.gcode_dir = g.into();
        }
        if let Some(g) = var("CCOS_CONFIG_DIR") {
            hw.config_dir = Some(g.into());
        }
        if let Some(g) = var("CCOS_TMP_DIR") {
            hw.tmp_dir = g.into();
        }
        if var("CCOS_ENABLE_MOTION").as_deref() == Some("1") {
            hw.motion = true;
        }
        hw
    }

    pub fn endpoint(&self, t: Target) -> &Endpoint {
        &self.endpoints[t as usize]
    }

    /// Why heaters/motors are locked, if they are.
    pub fn motion_lock(&self) -> Option<&'static str> {
        (!self.motion).then_some("Motion is locked on real hardware (set CCOS_ENABLE_MOTION=1 at your own risk)")
    }

    #[cfg(target_os = "linux")]
    pub fn open(&self, t: Target) -> io::Result<Box<dyn cc_transport::Link>> {
        Ok(match self.endpoint(t) {
            Endpoint::DspKernel => Box::new(cc_transport::dsp::DspLink::open(self.restart_dsp, Duration::from_secs(5))?),
            Endpoint::Serial { path, baud } => Box::new(cc_transport::serial::SerialLink::open(path, *baud)?),
        })
    }

    /// Power-cycle an STM32 board and boot its existing firmware, leaving it
    /// unconfigured.
    ///
    /// Sequence: power pin low 1 s, high (as `MCU::mcu_identify`). Elegoo's
    /// bootloader then waits **indefinitely** for a YMODEM upload; an `a`
    /// byte (ABORT2 in `mcu/bootloader/Utils/Src/ymodem.c`) makes it jump to
    /// the application. The toolhead re-enumerates on USB afterwards.
    #[cfg(target_os = "linux")]
    pub fn power_cycle(&self, t: Target) -> io::Result<()> {
        use cc_transport::Link;
        use std::fs::write;
        let (Some(n), Endpoint::Serial { path, baud }) = (t.power_gpio(), self.endpoint(t)) else {
            return Err(io::Error::other("the DSP is restarted via remoteproc, not a power pin"));
        };
        // The simulator reacts within milliseconds; real boards need the full delays.
        let scale = if self.simulated { 0.05 } else { 1.0 };
        let wait = |s: f64| sleep(Duration::from_secs_f64(s * scale));
        let base = self.gpio_root.join(format!("gpio{n}"));
        if !base.exists() {
            write(self.gpio_root.join("export"), n.to_string())?;
        }
        write(base.join("direction"), "out")?;
        write(base.join("value"), "0")?;
        wait(1.0);
        write(base.join("value"), "1")?;

        let wait_port = |timeout: Duration| -> io::Result<()> {
            let deadline = Instant::now() + timeout;
            while !Path::new(path).exists() {
                if Instant::now() > deadline {
                    return Err(io::Error::new(io::ErrorKind::TimedOut, format!("{path} did not appear")));
                }
                sleep(Duration::from_millis(100));
            }
            Ok(())
        };
        // Bootloader up (USB CDC enumeration for the toolhead).
        wait(0.5);
        wait_port(Duration::from_secs(10))?;
        let mut link = cc_transport::serial::SerialLink::open(path, *baud)?;
        link.send(&[b'a'; 16])?;
        drop(link);
        // Application up; the USB toolhead drops and re-creates ttyACM0.
        wait(if t == Target::Toolhead { 3.0 } else { 1.0 });
        wait_port(Duration::from_secs(10))
    }

    /// Bring a board to a fresh, unconfigured firmware: power-cycle the
    /// STM32s; the DSP is (re)attached by [`open`](Self::open). In a
    /// simulation the DSP is reset with the `reset` command instead of
    /// remoteproc.
    #[cfg(target_os = "linux")]
    pub fn restart(&self, t: Target) -> io::Result<()> {
        match (t, self.endpoint(t)) {
            (Target::Dsp, Endpoint::Serial { .. }) => {
                let mut link = self.open(t)?;
                let mut p = cc_proto::MsgBuf::new();
                cc_proto::Dictionary::centauri().encode_str("reset", &mut p).map_err(io::Error::other)?;
                // Any sequence number: send a few so one matches.
                for seq in 0..16 {
                    link.send(&cc_proto::encode_frame(seq, p.as_slice()).map_err(io::Error::other)?)?;
                }
                sleep(Duration::from_millis(50));
                Ok(())
            }
            (Target::Dsp, Endpoint::DspKernel) => Ok(()),
            _ => self.power_cycle(t),
        }
    }
}
