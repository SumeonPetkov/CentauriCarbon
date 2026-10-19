//! A TMC2209 behind the MCU's single-wire UART, as far as the host can
//! tell: datagram framing and CRC, node address (MS1/MS2 pins), IFCNT, the
//! register file with its power-on values and access types, SENDDELAY, and
//! the two things that change the machine:
//! * microstepping: from the MS1/MS2 pins (which are also the UART address:
//!   0 → 1/8, 1 → 1/32, 2 → 1/64, 3 → 1/16) until `mstep_reg_select` is set,
//!   then CHOPCONF.MRES;
//! * the DIAG output (StallGuard4): only in StealthChop, with SGTHRS > 0,
//!   the driver on (TOFF > 0) and TCOOLTHRS ≥ TSTEP > TPWMTHRS.

pub const GCONF: u8 = 0x00;
pub const GSTAT: u8 = 0x01;
pub const IFCNT: u8 = 0x02;
pub const SLAVECONF: u8 = 0x03;
pub const IOIN: u8 = 0x06;
pub const IHOLD_IRUN: u8 = 0x10;
pub const TPOWERDOWN: u8 = 0x11;
pub const TSTEP: u8 = 0x12;
pub const TPWMTHRS: u8 = 0x13;
pub const TCOOLTHRS: u8 = 0x14;
pub const SGTHRS: u8 = 0x40;
pub const SG_RESULT: u8 = 0x41;
pub const CHOPCONF: u8 = 0x6c;
pub const DRV_STATUS: u8 = 0x6f;
pub const PWMCONF: u8 = 0x70;

/// Internal clock (TSTEP is counted in its periods).
pub const FCLK: f64 = 12_000_000.0;

/// Register names (for probes and logs).
pub const NAMES: &[(&str, u8)] = &[
    ("gconf", GCONF),
    ("gstat", GSTAT),
    ("ifcnt", IFCNT),
    ("slaveconf", SLAVECONF),
    ("ihold_irun", IHOLD_IRUN),
    ("tpowerdown", TPOWERDOWN),
    ("tpwmthrs", TPWMTHRS),
    ("tcoolthrs", TCOOLTHRS),
    ("sgthrs", SGTHRS),
    ("chopconf", CHOPCONF),
    ("pwmconf", PWMCONF),
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Access {
    R,
    W,
    Rw,
    /// Read, write 1 to clear.
    Rwc,
}

fn access(reg: u8) -> Option<(Access, u32)> {
    Some(match reg {
        GCONF => (Access::Rw, 0x3ff),
        GSTAT => (Access::Rwc, 0x7),
        IFCNT => (Access::R, 0xff),
        SLAVECONF => (Access::W, 0xf00),
        0x04 => (Access::W, 0xffff),  // OTP_PROG
        0x05 => (Access::R, 0xffffff), // OTP_READ
        IOIN => (Access::R, 0xff0003ff),
        0x07 => (Access::Rw, 0x31f), // FACTORY_CONF
        IHOLD_IRUN => (Access::W, 0x000f1f1f),
        TPOWERDOWN => (Access::W, 0xff),
        TSTEP => (Access::R, 0xfffff),
        TPWMTHRS => (Access::W, 0xfffff),
        TCOOLTHRS => (Access::W, 0xfffff),
        0x22 => (Access::W, 0xffffff), // VACTUAL
        SGTHRS => (Access::W, 0xff),
        SG_RESULT => (Access::R, 0x3ff),
        0x42 => (Access::W, 0xffff), // COOLCONF
        0x6a => (Access::R, 0x3ff),       // MSCNT
        0x6b => (Access::R, 0x01ff01ff),  // MSCURACT
        CHOPCONF => (Access::Rw, 0xffffffff),
        DRV_STATUS => (Access::R, 0xc01f0fff),
        PWMCONF => (Access::Rw, 0xffffffff),
        0x71 => (Access::R, 0x01ff00ff), // PWM_SCALE
        0x72 => (Access::R, 0x00ff00ff), // PWM_AUTO
        _ => return None,
    })
}

/// What the motor is doing, for the read-only registers and DIAG.
#[derive(Debug, Clone, Copy, Default)]
pub struct Live {
    /// Seconds between the last two step pulses (0 = none yet).
    pub step_interval: f64,
    /// Seconds since the last step pulse.
    pub since_step: f64,
    pub enabled: bool,
    pub stalled: bool,
}

#[derive(Debug, Clone)]
pub struct Tmc2209 {
    pub addr: u8,
    regs: [u32; 128],
    pub ifcnt: u8,
    /// Valid write datagrams received.
    pub writes: u64,
    pub reads: u64,
}

pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            crc = if (crc >> 7) ^ (b & 1) != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
            b >>= 1;
        }
    }
    crc
}

impl Tmc2209 {
    /// Datasheet reset values (OTP defaults).
    pub fn power_on(addr: u8) -> Self {
        let mut regs = [0u32; 128];
        regs[GCONF as usize] = 0x0000_0101; // i_scale_analog, multistep_filt
        regs[GSTAT as usize] = 0x1; // reset
        regs[IHOLD_IRUN as usize] = 0x0001_1f10; // IHOLD 16, IRUN 31, IHOLDDELAY 1
        regs[TPOWERDOWN as usize] = 20;
        regs[CHOPCONF as usize] = 0x1000_0053; // TOFF 3, HSTRT 5, MRES 0, intpol
        regs[PWMCONF as usize] = 0xc10d_0024;
        Self { addr, regs, ifcnt: 0, writes: 0, reads: 0 }
    }

    pub fn reg(&self, r: u8) -> u32 {
        self.regs[(r & 0x7f) as usize]
    }

    /// Microsteps per full step.
    pub fn microsteps(&self) -> u32 {
        if self.reg(GCONF) & 0x80 != 0 {
            256 >> ((self.reg(CHOPCONF) >> 24) & 0xf).min(8)
        } else {
            match self.addr & 3 {
                0 => 8,
                1 => 32,
                2 => 64,
                _ => 16,
            }
        }
    }

    /// TSTEP for a step pulse interval (1/256 microsteps in FCLK periods).
    pub fn tstep(&self, live: &Live) -> u32 {
        if live.step_interval <= 0.0 || live.since_step > (live.step_interval * 4.0).max(0.02) {
            return 0xfffff;
        }
        let per_pulse = 256 / self.microsteps();
        ((live.step_interval * FCLK / per_pulse as f64) as u32).min(0xfffff)
    }

    fn stealthchop(&self, tstep: u32) -> bool {
        let tpwm = self.reg(TPWMTHRS) & 0xfffff;
        self.reg(GCONF) & 0x4 == 0 && (tpwm == 0 || tstep >= tpwm)
    }

    /// DIAG pin level (active high on the chip; the board inverts it).
    pub fn diag(&self, live: &Live) -> bool {
        let tstep = self.tstep(live);
        let tcool = self.reg(TCOOLTHRS) & 0xfffff;
        let tpwm = self.reg(TPWMTHRS) & 0xfffff;
        let toff = self.reg(CHOPCONF) & 0xf;
        let sg_active = self.reg(GCONF) & 0x4 == 0 && tcool >= tstep && tstep > tpwm && toff != 0;
        live.enabled && live.stalled && sg_active && self.reg(SGTHRS) & 0xff > 0
    }

    /// Reply delay after a read request, in bit times (SLAVECONF.SENDDELAY).
    pub fn senddelay_bits(&self) -> u32 {
        let s = (self.reg(SLAVECONF) >> 8) & 0xf;
        if s < 2 {
            8
        } else {
            (s | 1) * 8
        }
    }

    pub fn write(&mut self, reg: u8, val: u32) {
        self.writes += 1;
        self.ifcnt = self.ifcnt.wrapping_add(1);
        let Some((acc, mask)) = access(reg) else { return };
        let r = &mut self.regs[reg as usize];
        match acc {
            Access::W | Access::Rw => *r = val & mask,
            Access::Rwc => *r &= !(val & mask),
            Access::R => {}
        }
    }

    /// None for an address the chip does not have (no reply).
    pub fn read(&mut self, reg: u8, live: &Live) -> Option<u32> {
        let (acc, _) = access(reg)?;
        self.reads += 1;
        let tstep = self.tstep(live);
        Some(match reg {
            IFCNT => self.ifcnt as u32,
            IOIN => {
                let ms1 = (self.addr & 1) as u32;
                let ms2 = ((self.addr >> 1) & 1) as u32;
                0x21 << 24 | (!live.enabled as u32) | ms1 << 2 | ms2 << 3 | (self.diag(live) as u32) << 4 | 1 << 6
            }
            TSTEP => tstep,
            SG_RESULT => {
                if live.stalled {
                    0
                } else {
                    300
                }
            }
            DRV_STATUS => {
                let ii = self.reg(IHOLD_IRUN);
                let standstill = tstep == 0xfffff;
                let cs = if standstill { ii & 0x1f } else { (ii >> 8) & 0x1f };
                (standstill as u32) << 31 | (self.stealthchop(tstep) as u32) << 30 | cs << 16
            }
            _ if acc == Access::W => 0,
            _ => self.reg(reg),
        })
    }
}

/// Undo the UART framing of what the MCU shifted out: 10-bit frames (start
/// 0, 8 data bits LSB first, stop 1), packed LSB first. None on a framing
/// error.
pub fn unframe(bits: &[u8]) -> Option<Vec<u8>> {
    let n = bits.len() * 8 / 10;
    let bit = |i: usize| bits[i / 8] >> (i % 8) & 1;
    (0..n)
        .map(|f| {
            let base = f * 10;
            if bit(base) != 0 || bit(base + 9) != 1 {
                return None;
            }
            Some((0..8).fold(0u8, |acc, k| acc | bit(base + 1 + k) << k))
        })
        .collect()
}

/// UART framing of a reply datagram (what the MCU samples from the start bit).
pub fn frame(bytes: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; (bytes.len() * 10).div_ceil(8)];
    for (i, &d) in bytes.iter().enumerate() {
        let b = ((d as u32) << 1) | 0x200;
        for k in 0..10 {
            if b >> k & 1 != 0 {
                let pos = i * 10 + k;
                out[pos / 8] |= 1 << (pos % 8);
            }
        }
    }
    out
}

/// A parsed request datagram.
pub enum Request {
    Read { addr: u8, reg: u8 },
    Write { addr: u8, reg: u8, val: u32 },
}

/// Sync nibble, length and CRC check of a request.
pub fn parse_request(bytes: &[u8]) -> Option<Request> {
    if bytes.first()? & 0x0f != 0x05 {
        return None;
    }
    match bytes.len() {
        4 if crc8(&bytes[..3]) == bytes[3] && bytes[2] & 0x80 == 0 => Some(Request::Read { addr: bytes[1], reg: bytes[2] }),
        8 if crc8(&bytes[..7]) == bytes[7] && bytes[2] & 0x80 != 0 => {
            Some(Request::Write { addr: bytes[1], reg: bytes[2] & 0x7f, val: u32::from_be_bytes([bytes[3], bytes[4], bytes[5], bytes[6]]) })
        }
        _ => None,
    }
}

/// Reply datagram for a read (sync 0x05, master address 0xff).
pub fn reply(reg: u8, val: u32) -> [u8; 8] {
    let b = val.to_be_bytes();
    let mut r = [0x05, 0xff, reg, b[0], b[1], b[2], b[3], 0];
    r[7] = crc8(&r[..7]);
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framing_roundtrip() {
        let d = [0xf5u8, 0x03, 0xec, 0x14, 0x03, 0x00, 0x53, 0x9a];
        assert_eq!(unframe(&frame(&d)).unwrap(), d);
        let mut bad = frame(&d);
        bad[0] |= 1; // start bit high
        assert!(unframe(&bad).is_none());
    }

    #[test]
    fn microsteps_follow_address_pins_until_configured() {
        let mut t = Tmc2209::power_on(0);
        assert_eq!(t.microsteps(), 8);
        t.write(CHOPCONF, 0x1401_0053);
        assert_eq!(t.microsteps(), 8);
        t.write(GCONF, 0xc0);
        assert_eq!(t.microsteps(), 16);
        assert_eq!(t.ifcnt, 2);
    }

    #[test]
    fn stallguard_needs_the_homing_setup() {
        let mut t = Tmc2209::power_on(0);
        t.write(GCONF, 0xc0);
        t.write(CHOPCONF, 0x1401_0053);
        t.write(TPWMTHRS, 0xfffff);
        let live = Live { step_interval: 156e-6, since_step: 0.0, enabled: true, stalled: true };
        assert!(!t.diag(&live));
        // homing_move_begin
        t.write(TPWMTHRS, 0);
        t.write(TCOOLTHRS, 9345);
        t.write(SGTHRS, 120);
        assert!(t.diag(&live));
        // too slow for TCOOLTHRS
        assert!(!t.diag(&Live { step_interval: 0.02, ..live }));
        // SpreadCycle: no StallGuard
        t.write(GCONF, 0xc4);
        assert!(!t.diag(&live));
    }
}
