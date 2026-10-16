//! Byte links to the three Centauri Carbon MCUs.
//!
//! | MCU (printer.cfg)    | Hardware                     | Link                                   |
//! |----------------------|------------------------------|----------------------------------------|
//! | `[mcu]`              | R528 HiFi4 DSP (on the SoC)  | [`dsp::DspLink`] shared-memory ring    |
//! | `[mcu stm32]`        | STM32F401 toolhead board     | [`serial::SerialLink`] `/dev/ttyACM0` @ 1 Mbaud (USB CDC) |
//! | `[mcu strain_gauge_mcu]` | STM32F401 load-cell board | [`serial::SerialLink`] `/dev/ttyS4` @ 115200 |
//!
//! All links carry framed Klipper messages (see `cc-proto`).

use std::io;
use std::time::Duration;

pub mod ring;
#[cfg(target_os = "linux")]
pub mod dsp;
#[cfg(unix)]
pub mod serial;

/// A bidirectional byte pipe to one MCU.
pub trait Link: Send {
    /// Send raw bytes (one or more complete frames).
    fn send(&mut self, data: &[u8]) -> io::Result<()>;
    /// Receive whatever is available, waiting at most `timeout`.
    /// Returns 0 on timeout.
    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;
    /// File descriptor that becomes readable when data arrives, for callers
    /// that wait on several links with one `poll()`.
    fn fd(&self) -> Option<i32> {
        None
    }
}

/// In-memory loopback, handy for tests and the simulator.
#[derive(Default)]
pub struct Loopback {
    pub sent: Vec<u8>,
    pub to_receive: std::collections::VecDeque<u8>,
}

impl Link for Loopback {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        self.sent.extend_from_slice(data);
        Ok(())
    }
    fn recv(&mut self, buf: &mut [u8], _timeout: Duration) -> io::Result<usize> {
        let n = buf.len().min(self.to_receive.len());
        for (i, b) in self.to_receive.drain(..n).enumerate() {
            buf[i] = b;
        }
        Ok(n)
    }
}
