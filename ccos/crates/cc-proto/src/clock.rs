//! MCU clock helpers.
//!
//! The MCUs report 32-bit clocks; the host keeps a 64-bit estimate. Clock
//! frequencies are **hard-coded** in Elegoo's host (`serialhdl.cpp`), not read
//! from the identify dictionary.

/// R528 HiFi4 DSP timer frequency (`mcu` section, `mem_interface_DSP`).
pub const DSP_CLOCK_FREQ: f64 = 200_000_000.0;
/// STM32F401 toolhead / strain-gauge board clock.
pub const STM32_CLOCK_FREQ: f64 = 84_000_000.0;
/// `PWM_MAX` the host assumes for the DSP.
pub const DSP_PWM_MAX: u32 = 255;
/// `PWM_MAX` the host assumes for the STM32 boards.
pub const STM32_PWM_MAX: u32 = 2_000_000;
pub const ADC_MAX: u32 = 4095;

/// Extend a 32-bit MCU clock to 64 bits using the most recent known clock
/// (same as Klipper's `clock_from_clock32`).
pub fn clock_from_clock32(last_clock: u64, clock32: u32) -> u64 {
    let diff = clock32.wrapping_sub(last_clock as u32) as i32 as i64;
    (last_clock as i64 + diff) as u64
}

/// Linear clock estimate: `clock = conv_clock + (time - conv_time) * freq`.
#[derive(Debug, Clone, Copy)]
pub struct ClockEstimate {
    pub conv_time: f64,
    pub conv_clock: u64,
    pub freq: f64,
}

impl ClockEstimate {
    pub fn clock_from_time(&self, t: f64) -> u64 {
        ((t - self.conv_time) * self.freq + self.conv_clock as f64).max(0.0) as u64
    }
    pub fn time_from_clock(&self, clock: u64) -> f64 {
        (clock as f64 - self.conv_clock as f64) / self.freq + self.conv_time
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock32_wraps_forward_and_back() {
        let last = 0x1_ffff_fff0u64;
        assert_eq!(clock_from_clock32(last, 0x0000_0010), 0x2_0000_0010);
        assert_eq!(clock_from_clock32(last, 0xffff_ff00), 0x1_ffff_ff00);
    }

    #[test]
    fn estimate_roundtrip() {
        let e = ClockEstimate { conv_time: 10.0, conv_clock: 1_000, freq: STM32_CLOCK_FREQ };
        let c = e.clock_from_time(11.5);
        assert!((e.time_from_clock(c) - 11.5).abs() < 1e-6);
    }
}
