//! TMC2209 stepper drivers over the MCUs' bit-banged UART: a port of the
//! stock `extras/tmc.cpp`, `tmc2208.cpp`, `tmc2209.cpp` and `tmc_uart.cpp`.
//!
//! What the stock firmware does with the drivers, and so what ccos does:
//! * at connect (`klippy:connect`, drivers in name order: extruder,
//!   stepper_x, stepper_y, stepper_z): read DRV_STATUS, write SLAVECONF and
//!   GCONF, then every register that has a value, in name order, then read
//!   DRV_STATUS again;
//! * around each homing move on a `virtual_endstop` (X and Y): StallGuard on
//!   (TPWMTHRS 0, StealthChop, TCOOLTHRS for 1 mm/s, SGTHRS = driver_sgthrs),
//!   then back (TPWMTHRS as configured, TCOOLTHRS 0, SGTHRS 0);
//! * `SET_TMC_CURRENT_<stepper>` (G28 sets the homing current with it);
//! * `INIT_TMC`, `SET_TMC_FIELD`, `DUMP_TMC` — registered once per driver
//!   with the `STEPPER` key ignored, so the last driver registered (the
//!   extruder) answers all three.
//!
//! Every write is checked through the IFCNT write counter and retried up to
//! five times; reads are retried five times. Stepper enable/disable does not
//! touch the drivers (the stock reactor's `register_callback` is empty).

use std::collections::BTreeMap;

use crate::config::Config;

pub const TMC_FREQUENCY: f64 = 12_000_000.0;
pub const TMC_BAUD_RATE: f64 = 20_000.0;
const MAX_CURRENT: f64 = 2.0;
/// Sync byte the stock driver sends (TMC sync nibble 0x5 + reserved bits).
const SYNC: u8 = 0xf5;

/// `Registers` (init_Registers_2208 + init_Registers_2209).
pub const REGISTERS: &[(&str, u8)] = &[
    ("GCONF", 0x00),
    ("GSTAT", 0x01),
    ("IFCNT", 0x02),
    ("SLAVECONF", 0x03),
    ("OTP_PROG", 0x04),
    ("OTP_READ", 0x05),
    ("IOIN", 0x06),
    ("FACTORY_CONF", 0x07),
    ("IHOLD_IRUN", 0x10),
    ("TPOWERDOWN", 0x11),
    ("TSTEP", 0x12),
    ("TPWMTHRS", 0x13),
    ("TCOOLTHRS", 0x14),
    ("VACTUAL", 0x22),
    ("SGTHRS", 0x40),
    ("SG_RESULT", 0x41),
    ("COOLCONF", 0x42),
    ("MSCNT", 0x6a),
    ("MSCURACT", 0x6b),
    ("CHOPCONF", 0x6c),
    ("DRV_STATUS", 0x6f),
    ("PWMCONF", 0x70),
    ("PWM_SCALE", 0x71),
    ("PWM_AUTO", 0x72),
];

/// `Fields` (init_Fields_2208 + init_Fields_2209).
pub const FIELDS: &[(&str, &[(&str, u32)])] = &[
    (
        "GCONF",
        &[
            ("i_scale_analog", 1 << 0),
            ("internal_rsense", 1 << 1),
            ("en_spreadcycle", 1 << 2),
            ("shaft", 1 << 3),
            ("index_otpw", 1 << 4),
            ("index_step", 1 << 5),
            ("pdn_disable", 1 << 6),
            ("mstep_reg_select", 1 << 7),
            ("multistep_filt", 1 << 8),
            ("test_mode", 1 << 9),
        ],
    ),
    ("GSTAT", &[("reset", 1 << 0), ("drv_err", 1 << 1), ("uv_cp", 1 << 2)]),
    ("IFCNT", &[("ifcnt", 0xff)]),
    ("SLAVECONF", &[("senddelay", 0x0f << 8)]),
    ("OTP_PROG", &[("OTPBIT", 0x07), ("OTPBYTE", 0x03 << 4), ("OTPMAGIC", 0xff << 8)]),
    (
        "OTP_READ",
        &[
            ("OTP_FCLKTRIM", 0x1f),
            ("otp_OTTRIM", 1 << 5),
            ("otp_internalRsense", 1 << 6),
            ("otp_TBL", 1 << 7),
            ("OTP_PWM_GRAD", 0x0f << 8),
            ("otp_pwm_autograd", 1 << 12),
            ("OTP_TPWMTHRS", 0x07 << 13),
            ("otp_PWM_OFS", 1 << 16),
            ("otp_PWM_REG", 1 << 17),
            ("otp_PWM_FREQ", 1 << 18),
            ("OTP_IHOLDDELAY", 0x03 << 19),
            ("OTP_IHOLD", 0x03 << 21),
            ("otp_en_spreadCycle", 1 << 23),
        ],
    ),
    (
        "IOIN",
        &[
            ("enn", 1 << 0),
            ("ms1", 1 << 2),
            ("ms2", 1 << 3),
            ("diag", 1 << 4),
            ("pdn_uart", 1 << 6),
            ("step", 1 << 7),
            ("SPREAD_EN", 1 << 8),
            ("dir", 1 << 9),
            ("version", 0xff << 24),
        ],
    ),
    ("FACTORY_CONF", &[("fclktrim", 0x1f), ("ottrim", 0x03 << 8)]),
    ("IHOLD_IRUN", &[("ihold", 0x1f), ("irun", 0x1f << 8), ("iholddelay", 0x0f << 16)]),
    ("TPOWERDOWN", &[("tpowerdown", 0xff)]),
    ("TSTEP", &[("tstep", 0xfffff)]),
    ("TPWMTHRS", &[("tpwmthrs", 0xfffff)]),
    ("TCOOLTHRS", &[("TCOOLTHRS", 0xfffff)]),
    ("VACTUAL", &[("vactual", 0xffffff)]),
    ("SGTHRS", &[("sgthrs", 0xff)]),
    ("SG_RESULT", &[("SG_RESULT", 0x3ff)]),
    ("COOLCONF", &[("semin", 0x0f), ("seup", 0x03 << 5), ("semax", 0x0f << 8), ("sedn", 0x03 << 13), ("seimin", 1 << 15)]),
    ("MSCNT", &[("mscnt", 0x3ff)]),
    ("MSCURACT", &[("cur_a", 0x1ff), ("cur_b", 0x1ff << 16)]),
    (
        "CHOPCONF",
        &[
            ("toff", 0x0f),
            ("hstrt", 0x07 << 4),
            ("hend", 0x0f << 7),
            ("tbl", 0x03 << 15),
            ("vsense", 1 << 17),
            ("mres", 0x0f << 24),
            ("intpol", 1 << 28),
            ("dedge", 1 << 29),
            ("diss2g", 1 << 30),
            ("diss2vs", 1 << 31),
        ],
    ),
    (
        "DRV_STATUS",
        &[
            ("otpw", 1 << 0),
            ("ot", 1 << 1),
            ("s2ga", 1 << 2),
            ("s2gb", 1 << 3),
            ("s2vsa", 1 << 4),
            ("s2vsb", 1 << 5),
            ("ola", 1 << 6),
            ("olb", 1 << 7),
            ("t120", 1 << 8),
            ("t143", 1 << 9),
            ("t150", 1 << 10),
            ("t157", 1 << 11),
            ("cs_actual", 0x1f << 16),
            ("stealth", 1 << 30),
            ("stst", 1 << 31),
        ],
    ),
    (
        "PWMCONF",
        &[
            ("pwm_ofs", 0xff),
            ("pwm_grad", 0xff << 8),
            ("pwm_freq", 0x03 << 16),
            ("pwm_autoscale", 1 << 18),
            ("pwm_autograd", 1 << 19),
            ("freewheel", 0x03 << 20),
            ("pwm_reg", 0x0f << 24),
            ("pwm_lim", 0x0f << 28),
        ],
    ),
    ("PWM_SCALE", &[("pwm_scale_sum", 0xff), ("pwm_scale_auto", 0x1ff << 16)]),
    ("PWM_AUTO", &[("pwm_ofs_auto", 0xff), ("pwm_grad_auto", 0xff << 16)]),
];

/// `ReadRegisters` (read and discarded by `DUMP_TMC`).
pub const READ_REGISTERS: &[&str] =
    &["GCONF", "GSTAT", "IFCNT", "OTP_READ", "IOIN", "FACTORY_CONF", "TSTEP", "MSCNT", "MSCURACT", "CHOPCONF", "DRV_STATUS", "PWMCONF", "PWM_SCALE", "PWM_AUTO", "SG_RESULT"];

const SIGNED_FIELDS: &[&str] = &["cur_a", "cur_b", "pwm_scale_auto"];

/// `name_to_reg[name]`: unknown names (and "") give 0, like `std::map::operator[]`.
pub fn reg_addr(name: &str) -> u8 {
    REGISTERS.iter().find(|(n, _)| *n == name).map_or(0, |(_, a)| *a)
}

fn field_mask(reg: &str, field: &str) -> u32 {
    FIELDS.iter().find(|(r, _)| *r == reg).and_then(|(_, f)| f.iter().find(|(n, _)| *n == field)).map_or(0, |(_, m)| *m)
}

fn ffs(mask: u32) -> u32 {
    if mask == 0 {
        0
    } else {
        mask.trailing_zeros()
    }
}

fn bit_length(v: u32) -> u32 {
    32 - v.leading_zeros()
}

/// `FieldHelper`: the driver's register image, keyed by register name and
/// iterated in name order (`std::map`).
#[derive(Debug, Clone, Default)]
pub struct FieldHelper {
    pub registers: BTreeMap<String, u32>,
    /// Fields whose register was looked up with `operator[]` and not found
    /// (they map to the register "" from then on).
    unknown: Vec<String>,
}

impl FieldHelper {
    /// Register of a field, None if unknown.
    pub fn lookup_register(&self, field: &str) -> Option<&'static str> {
        FIELDS.iter().find(|(_, f)| f.iter().any(|(n, _)| *n == field)).map(|(r, _)| *r)
    }

    fn reg_of(&mut self, field: &str) -> String {
        match self.lookup_register(field) {
            Some(r) => r.to_string(),
            None => {
                if !self.unknown.iter().any(|u| u == field) {
                    self.unknown.push(field.to_string());
                }
                String::new()
            }
        }
    }

    /// `get_field(field, reg_value = 0)`: a 0 register value means "the
    /// stored one".
    pub fn get_field(&self, field: &str, reg_value: u32) -> i64 {
        let reg = self.lookup_register(field).unwrap_or("");
        let reg_value = if reg_value == 0 { self.registers.get(reg).copied().unwrap_or(0) } else { reg_value };
        let mask = field_mask(reg, field);
        let mut v = ((reg_value & mask) >> ffs(mask)) as i64;
        if SIGNED_FIELDS.contains(&field) && ((reg_value & mask) as u64) << 1 > mask as u64 {
            v -= 1 << bit_length(v as u32);
        }
        v
    }

    /// `set_field(field, value)`: the new register value (also stored).
    pub fn set_field(&mut self, field: &str, value: i64) -> u32 {
        let reg = self.reg_of(field);
        let old = self.registers.get(&reg).copied().unwrap_or(0);
        let mask = field_mask(&reg, field);
        let new = (old & !mask) | ((value as u32).wrapping_shl(ffs(mask)) & mask);
        self.registers.insert(reg, new);
        new
    }

    /// `set_config_field(section, field, default)`: from `driver_<field>`.
    pub fn set_config_field(&mut self, c: &Config, section: &str, field: &str, default: i64) -> u32 {
        let key = format!("driver_{field}");
        let reg = self.lookup_register(field).unwrap_or("");
        let mask = field_mask(reg, field);
        let maxval = (mask >> ffs(mask)) as i64;
        let v = if maxval == 1 {
            c.get_bool(section, &key, default != 0) as i64
        } else if SIGNED_FIELDS.contains(&field) {
            c.get_int(section, &key, default).clamp(-(maxval / 2 + 1), maxval / 2)
        } else {
            c.get_int(section, &key, default).clamp(0, maxval)
        };
        self.set_field(field, v)
    }
}

// ---- UART datagrams (MCU_TMC_uart_bitbang) ----

/// `_calc_crc8`.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            if (crc >> 7) ^ (b & 1) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            b >>= 1;
        }
    }
    crc
}

/// `_add_serial_bits`: each byte as start bit + 8 data bits (LSB first) +
/// stop bit, the bit stream packed LSB first into bytes.
fn add_serial_bits(data: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; (data.len() * 10).div_ceil(8)];
    for (i, &d) in data.iter().enumerate() {
        let b = ((d as u32) << 1) | 0x200;
        for bit in 0..10 {
            if b >> bit & 1 != 0 {
                let pos = i * 10 + bit;
                out[pos / 8] |= 1 << (pos % 8);
            }
        }
    }
    out
}

/// `_encode_read`: 5 bytes on the wire.
pub fn encode_read(addr: u8, reg: u8) -> Vec<u8> {
    let mut msg = [SYNC, addr, reg, 0];
    msg[3] = crc8(&msg[..3]);
    add_serial_bits(&msg)
}

/// `_encode_write` (reg already has bit 7 set for writes): 10 bytes.
pub fn encode_write(sync: u8, addr: u8, reg: u8, val: u32) -> Vec<u8> {
    let b = val.to_be_bytes();
    let mut msg = [sync, addr, reg, b[0], b[1], b[2], b[3], 0];
    msg[7] = crc8(&msg[..7]);
    add_serial_bits(&msg)
}

/// `_decode_read`: the 10 bytes the MCU sampled after a read request.
/// None if the length, start/stop bits or CRC are wrong (the reply is
/// re-encoded and compared, as the firmware does).
pub fn decode_read(reg: u8, data: &[u8]) -> Option<u32> {
    if data.len() != 10 {
        return None;
    }
    let mut mval = 0u128;
    for (i, &d) in data.iter().enumerate() {
        mval |= (d as u128) << (i * 8);
    }
    let byte = |pos: u32| ((mval >> pos) & 0xff) as u32;
    let val = (byte(31) << 24) | (byte(41) << 16) | (byte(51) << 8) | byte(61);
    (encode_write(0x05, 0xff, reg, val) == data).then_some(val)
}

// ---- one driver ----

/// How a driver talks to its MCU: one `tmcuart_send` and its reply bytes.
pub trait TmcBus {
    /// `send_with_response("tmcuart_send …", "tmcuart_response")`. A
    /// `print_time` delays the transfer until then (writes from G-code).
    fn transfer(&mut self, target: crate::Target, write: &[u8], read: u8, print_time: Option<f64>) -> Result<Vec<u8>, String>;
    /// Firmware error log (`LOG_E`, `GAM_ERR_printf`).
    fn log(&mut self, msg: &str);
    /// Firmware stdout (`printf`).
    fn debug(&mut self, _msg: &str) {}
}

/// One `[tmc2209 <stepper>]`.
#[derive(Debug, Clone)]
pub struct Tmc {
    /// "stepper_x".
    pub name: String,
    pub target: crate::Target,
    pub addr: u8,
    pub fields: FieldHelper,
    /// `MCU_TMC_uart::ifcnt` (None = not read yet).
    pub ifcnt: Option<u32>,
    pub sense_resistor: f64,
    /// Configured run/hold current (`fields->irun`/`ihold`).
    pub run_current: f64,
    pub hold_current: f64,
    /// `driver_sgthrs` (SGTHRS while homing).
    pub sgthrs_home: i64,
    pub threshold_velocity: f64,
    /// Endstop pin of the stepper is `tmc2209_<name>:virtual_endstop`.
    pub virtual_endstop: bool,
    pub diag_pin: String,
    en_pwm: i64,
    pwmthrs: i64,
}

impl Tmc {
    /// `TMC2209::TMC2209(section)`, then the virtual endstop setup. `step_dist`
    /// is the stepper's distance per (micro)step.
    pub fn from_config(c: &Config, section: &str, target: crate::Target, step_dist: f64) -> Tmc {
        let name = section.split(' ').next_back().unwrap_or("").to_string();
        let stepper_section: String = section.split(' ').skip(1).collect();
        let mut f = FieldHelper::default();
        let addr = c.get_int(section, "uart_address", 0).clamp(0, 3) as u8;
        f.set_field("senddelay", 2);
        // TMCCurrentHelper
        let run_current = c.get_double(section, "run_current", 1.0).clamp(0.0, MAX_CURRENT);
        let hold_current = c.get_double(section, "hold_current", run_current).clamp(0.0, MAX_CURRENT);
        let sense_resistor = c.get_double(section, "sense_resistor", 0.110);
        let sgthrs_home = c.get_int(section, "driver_sgthrs", 0);
        let (vsense, irun, ihold) = calc_current(sense_resistor, run_current, hold_current);
        f.set_field("vsense", vsense as i64);
        f.set_field("ihold", ihold as i64);
        f.set_field("irun", irun as i64);
        // TMCVirtualPinHelper: TMC2209 has no diag0_stall field
        let diag_pin = c.get_string(section, "diag_pin", "");
        f.set_field("pdn_disable", 1);
        f.set_field("mstep_reg_select", 1);
        // TMCMicrostepHelper
        let mut ms_section = stepper_section.clone();
        if c.get_int(&stepper_section, "microsteps", -1) == -1 && c.get_int(section, "microsteps", -1) != -1 {
            ms_section = section.to_string();
        }
        let ms = c.get_int(&ms_section, "microsteps", 16);
        let mres = (9 - bit_length(ms.max(0) as u32) as i64).clamp(0, 8);
        f.set_field("mres", mres);
        let mut intpol = c.get_bool(&ms_section, "interpolate", false);
        if !intpol {
            intpol = c.get_bool(&ms_section, "interpolate", true);
            if intpol {
                intpol = c.get_bool(section, "interpolate", true);
            }
        }
        f.set_field("intpol", intpol as i64);
        // TMCStealthchopHelper
        let velocity = c.get(section, "stealthchop_threshold").filter(|v| !v.is_empty()).map(crate::config::parse_double);
        let threshold_velocity = TMC_FREQUENCY * step_dist / (1u32 << f.get_field("mres", 0)) as f64;
        let mut en_pwm_mode = 0;
        if let Some(v) = velocity {
            // int(threshold_velocity / velocity + .5); 0 gives +inf, which
            // the printer's ARM float→int conversion saturates
            let threshold = (threshold_velocity / v + 0.5) as i32;
            f.set_field("tpwmthrs", threshold.clamp(0, 0xfffff) as i64);
            en_pwm_mode = 1;
        }
        f.set_field("en_spreadcycle", (en_pwm_mode == 0) as i64);
        // the rest of TMC2209::TMC2209
        f.set_field("senddelay", 2);
        f.set_field("i_scale_analog", 0);
        f.set_field("internal_rsense", 0);
        f.set_field("shaft", 0);
        f.set_field("pdn_disable", 1);
        f.set_field("mstep_reg_select", 1);
        f.set_config_field(c, section, "multistep_filt", 0);
        f.set_field("test_mode", 0);
        f.set_config_field(c, section, "toff", 3);
        f.set_config_field(c, section, "hstrt", 5);
        f.set_config_field(c, section, "hend", 0);
        f.set_config_field(c, section, "tbl", 2);
        f.set_config_field(c, section, "iholddelay", 8);
        f.set_config_field(c, section, "pwm_ofs", 36);
        f.set_config_field(c, section, "pwm_grad", 14);
        f.set_config_field(c, section, "pwm_freq", 1);
        f.set_config_field(c, section, "pwm_autoscale", 1);
        f.set_config_field(c, section, "pwm_autograd", 1);
        f.set_config_field(c, section, "pwm_reg", 8);
        f.set_config_field(c, section, "pwm_lim", 12);
        f.set_config_field(c, section, "sgthrs", 0);
        f.set_config_field(c, section, "tpowerdown", 20);
        // setup_pin("endstop", "virtual_endstop"), when the stepper uses it
        let chip = format!("tmc2209_{name}:virtual_endstop");
        let virtual_endstop = c.get_string(&stepper_section, "endstop_pin", "").replace(' ', "") == chip;
        let en_pwm = (f.get_field("en_spreadcycle", 0) == 0) as i64;
        let pwmthrs = f.get_field("tpwmthrs", 0);
        Tmc {
            name,
            target,
            addr,
            fields: f,
            ifcnt: None,
            sense_resistor,
            run_current,
            hold_current,
            sgthrs_home,
            threshold_velocity,
            virtual_endstop,
            diag_pin,
            en_pwm,
            pwmthrs,
        }
    }

    // ---- MCU_TMC_uart ----

    fn reg_read(&mut self, bus: &mut dyn TmcBus, reg: u8) -> Option<u32> {
        let data = encode_read(self.addr, reg);
        match bus.transfer(self.target, &data, 10, None) {
            Ok(r) => decode_read(reg, &r),
            Err(e) => {
                bus.log(&e);
                None
            }
        }
    }

    /// `get_register`: up to five reads. The firmware returns an undefined
    /// value after five failures; ccos reports the error.
    pub fn get_register(&mut self, bus: &mut dyn TmcBus, reg_name: &str) -> Result<u32, String> {
        let reg = reg_addr(reg_name);
        for _ in 0..5 {
            if let Some(v) = self.reg_read(bus, reg) {
                return Ok(v);
            }
        }
        let e = format!("Unable to read tmc uart '{}' register {reg_name}", self.name);
        bus.log(&e);
        Err(e)
    }

    /// `set_register`: write, then check that IFCNT went up by one; five
    /// tries. (The check compares without wrapping, as the firmware does:
    /// a write that takes IFCNT from 255 to 0 is written once more.)
    pub fn set_register(&mut self, bus: &mut dyn TmcBus, reg_name: &str, val: u32, print_time: Option<f64>) {
        let reg = reg_addr(reg_name);
        for _ in 0..5 {
            let mut ifcnt_old = self.ifcnt;
            if self.ifcnt.is_none() {
                self.ifcnt = self.get_register(bus, "IFCNT").ok();
                ifcnt_old = self.ifcnt;
            }
            let data = encode_write(SYNC, self.addr, reg | 0x80, val);
            if let Err(e) = bus.transfer(self.target, &data, 0, print_time) {
                bus.log(&e);
            }
            if self.ifcnt == Some(0) {
                // LOG_I of the register just written: reads it back
                let _ = self.get_register(bus, reg_name);
            }
            self.ifcnt = self.get_register(bus, "IFCNT").ok();
            if let (Some(new), Some(old)) = (self.ifcnt, ifcnt_old) {
                if new == old + 1 {
                    return;
                }
            }
        }
        bus.log(&format!("Unable to write tmc uart : {} register : {reg_name}", self.name));
    }

    // ---- TMCCommandHelper ----

    /// `_init_registers`.
    pub fn init_registers(&mut self, bus: &mut dyn TmcBus, print_time: Option<f64>) -> Result<(), String> {
        let first = self.get_register(bus, "DRV_STATUS");
        for r in ["SLAVECONF", "GCONF"] {
            if let Some(&v) = self.fields.registers.get(r) {
                self.set_register(bus, r, v, print_time);
            }
        }
        let regs: Vec<(String, u32)> = self.fields.registers.iter().map(|(k, v)| (k.clone(), *v)).collect();
        for (r, v) in regs {
            self.set_register(bus, &r, v, print_time);
        }
        let first = first?;
        bus.debug(&format!(" DRV_STATUS  register_val {first:x}"));
        let v = self.get_register(bus, "DRV_STATUS")?;
        bus.debug(&format!(" DRV_STATUS  register_val {v:x}"));
        Ok(())
    }

    /// `SET_TMC_FIELD`: the register written (an unknown field writes the
    /// register "" — address 0, GCONF — with value 0, as the firmware does).
    pub fn set_tmc_field(&mut self, bus: &mut dyn TmcBus, field: &str, value: i64, print_time: f64) {
        let reg = self.fields.lookup_register(field).map(str::to_string);
        if reg.is_none() {
            bus.log(&format!("Unknown field name '{field}'"));
        }
        let v = self.fields.set_field(field, value);
        self.set_register(bus, reg.as_deref().unwrap_or(""), v, Some(print_time));
    }

    /// `DUMP_TMC`: every readable register is read; nothing is printed.
    pub fn dump(&mut self, bus: &mut dyn TmcBus) -> Result<(), String> {
        for r in READ_REGISTERS {
            self.get_register(bus, r)?;
        }
        Ok(())
    }

    // ---- TMCCurrentHelper ----

    fn current_from_field(&self, field: &str) -> f64 {
        let bits = self.fields.get_field(field, 0) as f64;
        let vref = if self.fields.get_field("vsense", 0) != 0 { 0.18 } else { 0.32 };
        (bits + 1.0) * vref / (32.0 * (self.sense_resistor + 0.020) * 2f64.sqrt())
    }

    /// `get_current`: (run, hold) from the register image.
    pub fn get_current(&self) -> (f64, f64) {
        (self.current_from_field("irun"), self.current_from_field("ihold"))
    }

    pub fn set_current(&mut self, bus: &mut dyn TmcBus, run: f64, hold: f64, print_time: Option<f64>) {
        let (vsense, irun, ihold) = calc_current(self.sense_resistor, run, hold);
        if vsense as i64 != self.fields.get_field("vsense", 0) {
            let v = self.fields.set_field("vsense", vsense as i64);
            self.set_register(bus, "CHOPCONF", v, print_time);
        }
        self.fields.set_field("ihold", ihold as i64);
        let v = self.fields.set_field("irun", irun as i64);
        self.set_register(bus, "IHOLD_IRUN", v, print_time);
    }

    // ---- TMCVirtualPinHelper ----

    /// `handle_homing_move_begin`: StallGuard output on.
    pub fn homing_begin(&mut self, bus: &mut dyn TmcBus) {
        self.en_pwm = (self.fields.get_field("en_spreadcycle", 0) == 0) as i64;
        let v = self.fields.set_field("tpwmthrs", 0);
        self.set_register(bus, "TPWMTHRS", v, None);
        let v = self.fields.set_field("en_spreadcycle", 0);
        self.set_register(bus, "GCONF", v, None);
        let threshold = (self.threshold_velocity / 1.0 + 0.5) as i32;
        let v = self.fields.set_field("TCOOLTHRS", threshold.clamp(0, 0xfffff) as i64);
        self.set_register(bus, "TCOOLTHRS", v, None);
        let v = self.fields.set_field("sgthrs", self.sgthrs_home);
        self.set_register(bus, "SGTHRS", v, None);
    }

    /// `handle_homing_move_end`.
    pub fn homing_end(&mut self, bus: &mut dyn TmcBus) {
        let v = self.fields.set_field("tpwmthrs", self.pwmthrs);
        self.set_register(bus, "TPWMTHRS", v, None);
        let v = self.fields.set_field("en_spreadcycle", (self.en_pwm == 0) as i64);
        self.set_register(bus, "GCONF", v, None);
        let v = self.fields.set_field("TCOOLTHRS", 0);
        self.set_register(bus, "TCOOLTHRS", v, None);
        let v = self.fields.set_field("sgthrs", 0);
        self.set_register(bus, "SGTHRS", v, None);
    }
}

fn current_bits(sense_resistor: f64, current: f64, vsense: bool) -> u32 {
    let sr = sense_resistor + 0.020;
    let vref = if vsense { 0.18 } else { 0.32 };
    let cs = (32.0 * current * sr * 2f64.sqrt() / vref - 1.0 + 0.5) as i64;
    cs.clamp(0, 31) as u32
}

/// `_calc_current`: (vsense, irun, ihold).
pub fn calc_current(sense_resistor: f64, run: f64, hold: f64) -> (bool, u32, u32) {
    let hold = hold.min(run);
    let irun = current_bits(sense_resistor, run, false);
    let ihold = current_bits(sense_resistor, hold, false);
    if irun < 16 && ihold < 16 {
        return (true, current_bits(sense_resistor, run, true), current_bits(sense_resistor, hold, true));
    }
    (false, irun, ihold)
}

/// The `[tmc2209 …]` sections in file order (the order the firmware
/// creates them; the last one owns INIT_TMC / SET_TMC_FIELD / DUMP_TMC).
pub fn sections(c: &Config) -> Vec<String> {
    c.sections.iter().filter(|s| s.name.starts_with("tmc2209 ")).map(|s| s.name.clone()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::Machine;
    use crate::Target;

    /// Klipper's (Python) packing, as an independent check of Elegoo's
    /// hand-unrolled one.
    fn klipper_serial_bits(data: &[u8]) -> Vec<u8> {
        let mut out = 0u128;
        let mut pos = 0usize;
        for &d in data {
            out |= (((d as u128) << 1) | 0x200) << pos;
            pos += 10;
        }
        (0..pos.div_ceil(8)).map(|i| (out >> (i * 8)) as u8).collect()
    }

    #[test]
    fn datagrams_match_klipper() {
        let mut m = [0xf5, 3, 0x6f, 0];
        m[3] = crc8(&m[..3]);
        assert_eq!(encode_read(3, 0x6f), klipper_serial_bits(&m));
        let w = encode_write(0xf5, 1, 0x6c | 0x80, 0x1403_0053);
        let mut m = [0xf5, 1, 0xec, 0x14, 0x03, 0x00, 0x53, 0];
        m[7] = crc8(&m[..7]);
        assert_eq!(w, klipper_serial_bits(&m));
        // a reply decodes to its value and only to its own register
        let reply = encode_write(0x05, 0xff, 0x02, 0x0000_002a);
        assert_eq!(decode_read(0x02, &reply), Some(42));
        assert_eq!(decode_read(0x6f, &reply), None);
        assert_eq!(decode_read(0x02, &reply[..9]), None);
        let mut bad = reply.clone();
        bad[4] ^= 0x10;
        assert_eq!(decode_read(0x02, &bad), None);
    }

    #[test]
    fn currents_like_the_firmware() {
        // stepper_x/y: run 1.0 (hold = run), sense 0.1
        assert_eq!(calc_current(0.1, 1.0, 1.0), (false, 16, 16));
        // stepper_z: 0.8
        assert_eq!(calc_current(0.1, 0.8, 0.8), (true, 23, 23));
        // extruder: 0.8 / 0.5
        assert_eq!(calc_current(0.1, 0.8, 0.5), (true, 23, 14));
        // G28's homing current
        assert_eq!(calc_current(0.1, 0.8, 1.0), (true, 23, 23));
    }

    fn stock() -> (Config, Machine) {
        let c = Config::parse(crate::config::STOCK_PRINTER_CFG);
        let m = Machine::from_config(&c);
        (c, m)
    }

    fn driver(name: &str) -> Tmc {
        let (c, m) = stock();
        let i = ["stepper_x", "stepper_y", "stepper_z", "extruder"].iter().position(|n| *n == name).unwrap();
        Tmc::from_config(&c, &format!("tmc2209 {name}"), m.steppers[i].target, m.steppers[i].step_dist)
    }

    #[test]
    fn stock_register_images() {
        let x = driver("stepper_x");
        let r = |t: &Tmc, n: &str| t.fields.registers.get(n).copied();
        assert_eq!(x.addr, 0);
        assert!(x.virtual_endstop);
        assert_eq!(r(&x, "GCONF"), Some(0xc0));
        assert_eq!(r(&x, "CHOPCONF"), Some(0x1401_0053));
        assert_eq!(r(&x, "IHOLD_IRUN"), Some(0x0008_1010));
        assert_eq!(r(&x, "PWMCONF"), Some(0xc80d_0e24));
        assert_eq!(r(&x, "SGTHRS"), Some(120));
        assert_eq!(r(&x, "SLAVECONF"), Some(0x200));
        assert_eq!(r(&x, "TPOWERDOWN"), Some(20));
        // stealthchop_threshold 0 → 12e6·d/0 = +inf → 0xfffff
        assert_eq!(r(&x, "TPWMTHRS"), Some(0xfffff));
        assert_eq!(r(&x, "TCOOLTHRS"), None);
        let names: Vec<&String> = x.fields.registers.keys().collect();
        assert_eq!(names, ["CHOPCONF", "GCONF", "IHOLD_IRUN", "PWMCONF", "SGTHRS", "SLAVECONF", "TPOWERDOWN", "TPWMTHRS"]);
        let y = driver("stepper_y");
        assert_eq!(y.addr, 3);
        assert!(y.virtual_endstop);
        let z = driver("stepper_z");
        assert_eq!(z.addr, 1);
        assert!(!z.virtual_endstop);
        assert_eq!(r(&z, "CHOPCONF"), Some(0x1403_0053));
        assert_eq!(r(&z, "IHOLD_IRUN"), Some(0x0008_1717));
        // stealthchop_threshold 99999: 12e6 · (0.0025/16) / 99999 → 0
        assert_eq!(r(&z, "TPWMTHRS"), Some(0));
        let e = driver("extruder");
        assert_eq!(e.target, Target::Toolhead);
        assert_eq!(e.addr, 3);
        assert_eq!(r(&e, "IHOLD_IRUN"), Some(0x0008_170e));
        assert_eq!(r(&e, "SGTHRS"), Some(10));
        assert_eq!(r(&e, "TPWMTHRS"), Some(0xfffff));
        assert_eq!(sections(&stock().0), ["tmc2209 stepper_x", "tmc2209 stepper_y", "tmc2209 stepper_z", "tmc2209 extruder"]);
    }

    /// A bus with one perfect TMC2209 behind it.
    struct FakeBus {
        regs: BTreeMap<u8, u32>,
        ifcnt: u32,
        writes: Vec<(u8, u32)>,
        reads: Vec<u8>,
        drop_writes: usize,
        logs: Vec<String>,
    }

    impl FakeBus {
        fn new() -> Self {
            Self { regs: BTreeMap::new(), ifcnt: 0, writes: vec![], reads: vec![], drop_writes: 0, logs: vec![] }
        }
    }

    impl TmcBus for FakeBus {
        fn transfer(&mut self, _: Target, write: &[u8], read: u8, _: Option<f64>) -> Result<Vec<u8>, String> {
            // undo the serial bits
            let mut v = 0u128;
            for (i, &d) in write.iter().enumerate() {
                v |= (d as u128) << (i * 8);
            }
            let bytes: Vec<u8> = (0..write.len() * 8 / 10).map(|i| (v >> (i * 10 + 1)) as u8).collect();
            if bytes.len() == 8 {
                if self.drop_writes > 0 {
                    self.drop_writes -= 1;
                    return Ok(vec![]);
                }
                let reg = bytes[2] & 0x7f;
                let val = u32::from_be_bytes([bytes[3], bytes[4], bytes[5], bytes[6]]);
                self.writes.push((reg, val));
                self.regs.insert(reg, val);
                self.ifcnt = (self.ifcnt + 1) & 0xff;
                return Ok(vec![]);
            }
            assert_eq!(read, 10);
            let reg = bytes[2];
            self.reads.push(reg);
            let val = if reg == 0x02 { self.ifcnt } else { self.regs.get(&reg).copied().unwrap_or(0) };
            Ok(encode_write(0x05, 0xff, reg, val))
        }
        fn log(&mut self, msg: &str) {
            self.logs.push(msg.to_string());
        }
    }

    #[test]
    fn init_writes_in_the_stock_order() {
        let mut x = driver("stepper_x");
        let mut bus = FakeBus::new();
        bus.ifcnt = 7;
        x.init_registers(&mut bus, None).unwrap();
        let order: Vec<u8> = bus.writes.iter().map(|w| w.0).collect();
        // SLAVECONF, GCONF, then all in name order
        assert_eq!(order, [0x03, 0x00, 0x6c, 0x00, 0x10, 0x70, 0x40, 0x03, 0x11, 0x13]);
        assert_eq!(bus.regs[&0x6c], 0x1401_0053);
        // DRV_STATUS, IFCNT, then one IFCNT per write, DRV_STATUS
        assert_eq!(bus.reads.first(), Some(&0x6f));
        assert_eq!(bus.reads.last(), Some(&0x6f));
        assert_eq!(bus.reads.iter().filter(|r| **r == 0x02).count(), 11);
    }

    #[test]
    fn write_retried_until_ifcnt_moves() {
        let mut z = driver("stepper_z");
        let mut bus = FakeBus::new();
        bus.ifcnt = 3;
        bus.drop_writes = 2;
        z.set_register(&mut bus, "SGTHRS", 99, None);
        assert_eq!(bus.writes, [(0x40, 99)]);
        assert!(bus.logs.is_empty());
        bus.drop_writes = 5;
        z.set_register(&mut bus, "SGTHRS", 98, None);
        assert_eq!(bus.logs, ["Unable to write tmc uart : stepper_z register : SGTHRS"]);
    }

    #[test]
    fn homing_hooks() {
        let mut x = driver("stepper_x");
        let mut bus = FakeBus::new();
        x.homing_begin(&mut bus);
        let tv = x.threshold_velocity;
        assert_eq!(bus.writes, [(0x13, 0), (0x00, 0xc0), (0x14, (tv + 0.5) as u32), (0x40, 120)]);
        bus.writes.clear();
        x.homing_end(&mut bus);
        assert_eq!(bus.writes, [(0x13, 0xfffff), (0x00, 0xc0), (0x14, 0), (0x40, 0)]);
        // TCOOLTHRS is now part of the image (written by later INIT_TMC)
        assert_eq!(x.fields.registers.get("TCOOLTHRS"), Some(&0));
    }

    #[test]
    fn set_current_and_report() {
        let mut x = driver("stepper_x");
        let mut bus = FakeBus::new();
        x.set_current(&mut bus, 0.8, x.get_current().1, None);
        // 0.8 A switches to vsense: CHOPCONF first, then IHOLD_IRUN
        assert_eq!(bus.writes, [(0x6c, 0x1403_0053), (0x10, 0x0008_1717)]);
        let (run, hold) = x.get_current();
        assert_eq!(crate::gcode::parse::cpp_to_string(run), "0.795495");
        assert_eq!(crate::gcode::parse::cpp_to_string(hold), "0.795495");
    }

    #[test]
    fn unknown_field_writes_gconf_zero() {
        let mut e = driver("extruder");
        let mut bus = FakeBus::new();
        e.set_tmc_field(&mut bus, "nonsense", 5, 0.0);
        assert_eq!(bus.writes, [(0x00, 0)]);
        assert_eq!(e.fields.registers.keys().next().map(String::as_str), Some(""));
    }
}
