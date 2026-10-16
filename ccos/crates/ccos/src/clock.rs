//! Host time: monotonic seconds since the process started. Print times are
//! expressed on this clock and converted to each MCU's clock with its
//! [`ClockSync`](crate::mcu::ClockSync).

use std::sync::OnceLock;
use std::time::Instant;

static START: OnceLock<Instant> = OnceLock::new();

pub fn now() -> f64 {
    START.get_or_init(Instant::now).elapsed().as_secs_f64()
}
