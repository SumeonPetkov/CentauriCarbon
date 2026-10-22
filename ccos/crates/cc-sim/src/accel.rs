//! The toolhead's LIS2DW12 accelerometer (SPI1, CS PA4) and the motion it
//! feels.
//!
//! The toolhead is a mass on the belts: per axis a spring to the carriage
//! position the steppers set (`p'' = ω²(c − p) − 2ζω p'`), integrated as the
//! steps arrive. That is the resonance `SHAPER_CALIBRATE` looks for. Each
//! chip sample is the mean acceleration over its period (Δv/Δt, standing in
//! for the chip's digital low-pass so step chatter does not alias), plus
//! gravity and a little noise, mapped through `axes_map: x,z,-y`.
//!
//! The chip: WHO_AM_I, CTRL1 (ODR), CTRL6 (full scale), FIFO_CTRL
//! (bypass/stream), a 32-sample FIFO with FIFO_SAMPLES (count and overrun)
//! and OUT_X_L..OUT_Z_H, auto-incremented on multi-byte reads.

use std::collections::VecDeque;

pub const WHO_AM_I: u8 = 0x0F;
pub const CTRL1: u8 = 0x20;
pub const CTRL6: u8 = 0x25;
pub const OUT_X_L: u8 = 0x28;
pub const FIFO_CTRL: u8 = 0x2E;
pub const FIFO_SAMPLES: u8 = 0x2F;
pub const LIS2DW12_ID: u8 = 0x44;
const FIFO_DEPTH: usize = 32;
/// mm/s² per 16-bit LSB at ±2 g (0.061 mg/LSB); ×2 per full-scale step.
const LSB_2G: f64 = 9806.65 * 4.0 / 65536.0;
const GRAVITY: f64 = 9806.65;
/// Integration step of the toolhead model.
const H: f64 = 1.0 / 40_000.0;

/// One axis of the toolhead on its belts.
#[derive(Debug, Clone, Copy)]
struct Axis {
    w: f64,
    zeta: f64,
    p: f64,
    v: f64,
}

impl Axis {
    fn step(&mut self, c: f64, h: f64) {
        let a = self.w * self.w * (c - self.p) - 2.0 * self.zeta * self.w * self.v;
        self.v += a * h;
        self.p += self.v * h;
    }
}

pub struct Lis2dw12 {
    regs: [u8; 64],
    fifo: VecDeque<[i16; 3]>,
    overrun: bool,
    /// Latest sample (what OUT_* read in bypass mode).
    last: [i16; 3],
    axes: [Axis; 2],
    /// Carriage position the steppers last set.
    carriage: [f64; 2],
    t: f64,
    /// Next sample time and the velocity at the previous one.
    next_sample: f64,
    v_prev: [f64; 2],
    rng: u64,
    pub samples: u64,
}

impl Lis2dw12 {
    /// `resonance`: [X Hz, Y Hz, X damping ratio, Y damping ratio].
    pub fn new(resonance: [f64; 4], start: [f64; 2]) -> Self {
        let axis = |f: f64, zeta: f64, p: f64| Axis { w: 2.0 * std::f64::consts::PI * f, zeta, p, v: 0.0 };
        let mut regs = [0u8; 64];
        regs[WHO_AM_I as usize] = LIS2DW12_ID;
        regs[0x21] = 0x04; // CTRL2: IF_ADD_INC
        Self {
            regs,
            fifo: VecDeque::new(),
            overrun: false,
            last: [0; 3],
            axes: [axis(resonance[0], resonance[2], start[0]), axis(resonance[1], resonance[3], start[1])],
            carriage: start,
            t: 0.0,
            next_sample: f64::INFINITY,
            v_prev: [0.0; 2],
            rng: 0x853c_49e6_748f_ea9b,
            samples: 0,
        }
    }

    /// Output data rate (Hz) from CTRL1, 0 = powered down.
    fn odr(&self) -> f64 {
        let c = self.regs[CTRL1 as usize];
        let hp = c & 0x0C == 0x04;
        match c >> 4 {
            0 => 0.0,
            1 => 12.5,
            n @ 2..=5 => 12.5 * f64::from(1u32 << (n - 2)),
            n @ 6..=9 if hp => 200.0 * f64::from(1u32 << (n - 6)),
            6..=9 => 200.0,
            _ => 0.0,
        }
    }

    fn lsb(&self) -> f64 {
        LSB_2G * f64::from(1u32 << ((self.regs[CTRL6 as usize] >> 4) & 3))
    }

    fn noise(&mut self) -> f64 {
        // sum of 4 uniforms: roughly normal, σ ≈ 1
        let mut s = 0.0;
        for _ in 0..4 {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 7;
            self.rng ^= self.rng << 17;
            s += (self.rng >> 11) as f64 / (1u64 << 53) as f64 - 0.5;
        }
        s * 1.732
    }

    /// Run the toolhead model up to `t` with the carriage where it is now,
    /// taking samples on the way.
    fn advance(&mut self, t: f64) {
        while self.t < t {
            let h = H.min(t - self.t);
            let to = self.t + h;
            if self.next_sample <= to {
                // finish exactly on the sample time
                let hs = self.next_sample - self.t;
                for (i, a) in self.axes.iter_mut().enumerate() {
                    a.step(self.carriage[i], hs);
                }
                self.t = self.next_sample;
                self.sample();
                continue;
            }
            for (i, a) in self.axes.iter_mut().enumerate() {
                a.step(self.carriage[i], h);
            }
            self.t = to;
        }
    }

    fn sample(&mut self) {
        let odr = self.odr();
        if odr <= 0.0 {
            self.next_sample = f64::INFINITY;
            return;
        }
        let dt = 1.0 / odr;
        let acc = [0, 1].map(|i| (self.axes[i].v - self.v_prev[i]) / dt);
        self.v_prev = [self.axes[0].v, self.axes[1].v];
        let sigma = 60.0;
        let (ax, ay, az) = (acc[0] + sigma * self.noise(), acc[1] + sigma * self.noise(), GRAVITY + sigma * self.noise());
        // axes_map x,z,-y: printer x = chip x, y = chip z, z = −chip y
        let lsb = self.lsb();
        let raw = |v: f64| ((v / lsb).round().clamp(-32768.0, 32767.0) as i16) & !3;
        self.last = [raw(ax), raw(-az), raw(ay)];
        self.samples += 1;
        if self.regs[FIFO_CTRL as usize] >> 5 == 0b110 {
            if self.fifo.len() == FIFO_DEPTH {
                self.fifo.pop_front();
                self.overrun = true;
            }
            self.fifo.push_back(self.last);
        }
        self.next_sample += dt;
    }

    /// The steppers moved the carriage at `t`.
    pub fn carriage_moved(&mut self, t: f64, x: f64, y: f64) {
        self.advance(t);
        self.carriage = [x, y];
    }

    fn write(&mut self, reg: u8, v: u8) {
        let was = self.odr();
        self.regs[reg as usize & 63] = v;
        match reg {
            CTRL1 if self.odr() != was => {
                self.next_sample = if self.odr() > 0.0 { self.t + 1.0 / self.odr() } else { f64::INFINITY };
                self.v_prev = [self.axes[0].v, self.axes[1].v];
            }
            FIFO_CTRL if v >> 5 == 0 => {
                // bypass mode empties the FIFO
                self.fifo.clear();
                self.overrun = false;
            }
            _ => {}
        }
    }

    fn read(&mut self, reg: u8) -> u8 {
        match reg {
            FIFO_SAMPLES => (self.fifo.len().min(63) as u8) | ((self.overrun as u8) << 6) | (((self.fifo.len() >= FIFO_DEPTH) as u8) << 7),
            0x28..=0x2D => {
                // reading OUT_X_L takes the next sample from the FIFO
                if reg == OUT_X_L {
                    if let Some(s) = self.fifo.pop_front() {
                        self.last = s;
                        self.overrun = false;
                    }
                }
                let s = self.last[((reg - OUT_X_L) / 2) as usize];
                if reg & 1 == 0 {
                    s as u8
                } else {
                    (s >> 8) as u8
                }
            }
            r => self.regs[r as usize & 63],
        }
    }

    /// One SPI transaction at `t` (CS low for all of `data`): byte 0 is the
    /// register (0x80 = read), the rest data. Replies go back into `data`.
    pub fn transfer(&mut self, t: f64, data: &mut [u8]) {
        self.advance(t);
        let Some(&cmd) = data.first() else { return };
        let read = cmd & 0x80 != 0;
        let mut reg = cmd & 0x3F;
        let inc = self.regs[0x21] & 0x04 != 0;
        data[0] = 0;
        for b in data.iter_mut().skip(1) {
            if read {
                *b = self.read(reg);
            } else {
                self.write(reg, *b);
            }
            if inc {
                reg = (reg + 1) & 0x3F;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rd(c: &mut Lis2dw12, t: f64, reg: u8) -> u8 {
        let mut d = [reg | 0x80, 0];
        c.transfer(t, &mut d);
        d[1]
    }

    #[test]
    fn id_fifo_and_gravity() {
        let mut c = Lis2dw12::new([50.0, 40.0, 0.05, 0.05], [100.0, 100.0]);
        assert_eq!(rd(&mut c, 0.0, WHO_AM_I), LIS2DW12_ID);
        c.transfer(0.0, &mut [CTRL6, 0x34]);
        c.transfer(0.0, &mut [FIFO_CTRL, 0xC0]);
        c.transfer(0.0, &mut [CTRL1, 0x94]);
        // 10 ms at 1600 Hz: 16 samples
        assert_eq!(rd(&mut c, 0.0101, FIFO_SAMPLES) & 0x3F, 16);
        let mut d = [OUT_X_L | 0x80, 0, 0, 0, 0, 0, 0];
        c.transfer(0.0101, &mut d);
        let y = i16::from_le_bytes([d[3], d[4]]);
        // chip y reads −1 g: about −2048 at ±16 g
        assert!((-2100..-2000).contains(&y), "{y}");
        assert_eq!(rd(&mut c, 0.0101, FIFO_SAMPLES) & 0x3F, 15);
        // 30 ms more: the FIFO overflows
        assert_eq!(rd(&mut c, 0.04, FIFO_SAMPLES) & 0xC0, 0xC0);
    }

    #[test]
    fn resonance_rings() {
        let mut c = Lis2dw12::new([50.0, 40.0, 0.02, 0.05], [100.0, 100.0]);
        c.transfer(0.0, &mut [CTRL6, 0x34]);
        c.transfer(0.0, &mut [CTRL1, 0x94]);
        c.transfer(0.0, &mut [FIFO_CTRL, 0xC0]);
        // a 0.5 mm jump of the carriage in X; the toolhead rings at 50 Hz
        c.carriage_moved(0.001, 100.5, 100.0);
        let mut xs = Vec::new();
        let mut t = 0.001;
        for _ in 0..10 {
            t += 0.01;
            while rd(&mut c, t, FIFO_SAMPLES) & 0x3F > 0 {
                let mut d = [OUT_X_L | 0x80, 0, 0, 0, 0, 0, 0];
                c.transfer(t, &mut d);
                xs.push(i16::from_le_bytes([d[1], d[2]]) as f64);
            }
        }
        // crossings of the ringing over 0.1 s: 5 periods at 50 Hz, two each
        let crossings = xs.windows(2).filter(|w| (w[0] > 300.0 && w[1] <= 300.0) || (w[0] < -300.0 && w[1] >= -300.0)).count();
        assert!(xs.len() >= 150 && (9..=11).contains(&crossings), "{} {crossings}", xs.len());
    }
}
