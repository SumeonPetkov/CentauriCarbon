//! `[adxl345 X]` commands (adx_lis2dw12.cpp): `ACCELEROMETER_MEASURE_X`,
//! `ACCELEROMETER_QUERY_X`, `ADXL345_DEBUG_READ_X`, `ADXL345_DEBUG_WRITE_X`.
//! Stock registers them as mux commands, but its `register_mux_command`
//! ignores the key: `CHIP=` is not looked at. The section is not named
//! "default", so the commands without the `_X` suffix do not exist.
//!
//! Kept stock behaviour:
//! * Nothing is printed to the console except `ACCELEROMETER_QUERY`'s
//!   values: the "measurements started/stopped" and register replies are
//!   commented out.
//! * `ACCELEROMETER_MEASURE` keeps only the first 10 s of samples (the
//!   buffer is sized for `measure_time` 10); later packets are dropped.
//! * An invalid `RATE` is only logged. The LIS2DW12 always runs at 1600 Hz
//!   (CTRL1 0x94); the rate only sets the MCU's poll interval (4 samples).
//! * `ACCELEROMETER_QUERY` waits for data without a time limit, logging
//!   "Timeout reading adxl345 data" every 0.1 s after 3 s.
//! * `REG`/`VAL` out of range (or missing) fall back to `INT32_MIN`, which
//!   the `uint8_t` conversion makes register/value 0.

use cc_proto::dict::Arg;

use super::parse::{cpp_double, Abort, CmdResult, GCmd};
use super::Exec;
use crate::accel::{self, AdxlCfg, AdxlType, Results};
use crate::clock::now;
use crate::machine::toolhead;
use crate::Target;

/// The chip state the G-code thread keeps (`m_query_rate`, …).
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct AdxlState {
    pub query_rate: i64,
    pub last_tx_time: f64,
    pub start1: f64,
    pub start2: f64,
}

/// `strftime("%Y%m%d_%H:%M:%S", localtime(now))`.
fn local_stamp() -> String {
    unsafe {
        let t = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        let mut buf = [0u8; 80];
        let n = libc::strftime(buf.as_mut_ptr() as *mut libc::c_char, buf.len(), c"%Y%m%d_%H:%M:%S".as_ptr(), &tm);
        String::from_utf8_lossy(&buf[..n]).into_owned()
    }
}

const COMMANDS: [&str; 4] = ["ACCELEROMETER_MEASURE", "ACCELEROMETER_QUERY", "ADXL345_DEBUG_READ", "ADXL345_DEBUG_WRITE"];

impl Exec {
    fn adxl_cfg(&self) -> Option<AdxlCfg> {
        self.core().m.adxl.clone()
    }

    pub(super) fn adxl_handles(&self, key: &str) -> bool {
        let c = self.core();
        let Some(a) = &c.m.adxl else { return false };
        COMMANDS.iter().any(|b| key.strip_prefix(b) == Some(a.suffix.as_str()))
    }

    pub(super) fn cmd_adxl(&mut self, key: &str, g: &GCmd) -> CmdResult {
        let Some(a) = self.adxl_cfg() else { return Ok(()) };
        let base = key.strip_suffix(a.suffix.as_str()).unwrap_or(key);
        match base {
            "ACCELEROMETER_MEASURE" => self.cmd_accelerometer_measure(&a, g),
            "ACCELEROMETER_QUERY" => self.cmd_accelerometer_query(&a),
            "ADXL345_DEBUG_READ" => {
                self.note_measuring();
                let reg = g.get_int("REG", i32::MIN as i64, Some(29), Some(57))? as u8;
                self.adxl_read_reg(&a, reg)?;
                Ok(())
            }
            _ => {
                self.note_measuring();
                let reg = g.get_int("REG", i32::MIN as i64, Some(29), Some(57))? as u8;
                let val = g.get_int("VAL", i32::MIN as i64, Some(0), Some(255))?;
                self.adxl_set_reg(&a, reg, (val & 0xFF) as u8, None)
            }
        }
    }

    fn note_measuring(&self) {
        if self.adxl.query_rate > 0 {
            self.log("adxl345 measurements in progress");
        }
    }

    fn adxl_sleep(&self, secs: f64) -> CmdResult {
        match self.p.wait(secs, "accelerometer", |c| (c.shutdown.is_some() || c.exit).then_some(())) {
            Ok(()) => Err(Abort(self.core().shutdown.clone().unwrap_or_else(|| "exit".into()))),
            Err(_) => Ok(()),
        }
    }

    /// `read_reg`: `spi_transfer` and wait for the reply.
    pub(super) fn adxl_read_reg(&self, _a: &AdxlCfg, reg: u8) -> Result<u8, Abort> {
        let data = [reg | 0x80, 0];
        let oid = toolhead::ACCEL_SPI as i64;
        let args = [("oid", Arg::Int(oid)), ("data", Arg::Bytes(&data))];
        let m = super::send_with_response(&self.p, Target::Toolhead, "spi_transfer", &args, "spi_transfer_response", oid).map_err(Abort)?;
        Ok(m.message().bytes("response").and_then(|r| r.get(1).copied()).unwrap_or(0))
    }

    /// `set_reg`: `spi_send` (after `minclock`, a print time), then read it
    /// back.
    pub(super) fn adxl_set_reg(&self, a: &AdxlCfg, reg: u8, val: u8, minclock: Option<f64>) -> CmdResult {
        {
            let mut c = self.core();
            let m = c.mcu(Target::Toolhead).map_err(|e| Abort(e.to_string()))?;
            let data = [reg, val];
            let msg = m.encode("spi_send", &[("oid", Arg::Int(toolhead::ACCEL_SPI as i64)), ("data", Arg::Bytes(&data))]);
            match minclock {
                Some(pt) => {
                    let clock = m.clock.clock(pt);
                    m.queue_held(clock, msg);
                }
                None => m.queue_msg(msg),
            }
            c.wake();
        }
        if minclock.is_some() {
            // stock sends both on the SPI's command queue: the read waits
            // behind the held write
            self.wait_until("spi_send", |c| c.mcus[Target::Toolhead as usize].as_ref().is_none_or(|m| m.held_len() == 0))?;
        }
        let stored = self.adxl_read_reg(a, reg)?;
        if stored != val {
            let chip = if a.kind == AdxlType::Lis2dw12 { "LIS2DW12" } else { "ADXL345" };
            self.log(format!("Failed to set {chip} register:{reg:x} set_val:{val:x} read_val:{stored:x}"));
        }
        Ok(())
    }

    fn adxl_is_initialized(&self, a: &AdxlCfg) -> Result<bool, Abort> {
        Ok(match a.kind {
            AdxlType::Lis2dw12 => self.adxl_read_reg(a, accel::REG_DEVID_LIS2DW12)? == accel::LIS2DW12_ID && self.adxl_read_reg(a, accel::LIS2DW12_CTRL6)? & 0x30 != 0,
            AdxlType::Adxl345 => self.adxl_read_reg(a, accel::REG_DEVID_ADXL345)? == accel::ADXL345_DEV_ID && self.adxl_read_reg(a, accel::REG_DATA_FORMAT_ADXL345)? & 0x0B != 0,
        })
    }

    /// `initialize`: power down, check the ID, set the data format. The
    /// error text is what stock prints when the ID is wrong.
    fn adxl_initialize(&self, a: &AdxlCfg) -> Result<Option<String>, Abort> {
        let (power, devid, id, format, fmt) = match a.kind {
            AdxlType::Lis2dw12 => (accel::LIS2DW12_CTRL1, accel::REG_DEVID_LIS2DW12, accel::LIS2DW12_ID, accel::LIS2DW12_CTRL6, 0x30),
            AdxlType::Adxl345 => (accel::REG_POWER_CTL_ADXL345, accel::REG_DEVID_ADXL345, accel::ADXL345_DEV_ID, accel::REG_DATA_FORMAT_ADXL345, 0x0B),
        };
        self.adxl_set_reg(a, power, 0, None)?;
        self.adxl_set_reg(a, power, 0, None)?;
        let dev_id = self.adxl_read_reg(a, devid)?;
        if dev_id != id {
            let e = format!("Invalid {} id (got {dev_id} vs {id} )", a.type_name());
            self.log(e.clone());
            return Ok(Some(e));
        }
        self.adxl_set_reg(a, format, fmt, None)?;
        Ok(None)
    }

    /// `start_measurements(rate, measure_time)`: Ok(Err(text)) when the chip
    /// does not answer with its ID; Ok(Ok(false)) when already measuring.
    pub(super) fn adxl_start(&mut self, a: &AdxlCfg, rate: Option<i64>, measure_time: i64) -> Result<Result<bool, String>, Abort> {
        if self.adxl.query_rate > 0 {
            return Ok(Ok(false));
        }
        let rate = rate.unwrap_or(a.data_rate);
        self.log(format!("start measurements rate:{rate}"));
        if !self.adxl_is_initialized(a)? {
            if let Some(e) = self.adxl_initialize(a)? {
                return Ok(Err(e));
            }
        }
        let clock = (self.adxl.last_tx_time != 0.0).then_some(self.adxl.last_tx_time);
        match a.kind {
            AdxlType::Lis2dw12 => {
                self.adxl_set_reg(a, accel::LIS2DW12_CTRL1, 0x00, clock)?;
                self.adxl_set_reg(a, accel::LIS2DW12_FIFO_CTRL, 0x00, None)?; // bypass
                self.adxl_set_reg(a, accel::LIS2DW12_CTRL6, 0x34, None)?; // ±16 g, low noise
                self.adxl_set_reg(a, accel::LIS2DW12_FIFO_CTRL, 0xC0, None)?; // stream
                self.adxl_set_reg(a, accel::LIS2DW12_CTRL1, 0x94, None)?; // 1600 Hz, high performance
            }
            AdxlType::Adxl345 => {
                let bw = accel::QUERY_RATES.iter().find(|(r, _)| *r == rate).map_or(0, |(_, b)| *b);
                self.adxl_set_reg(a, accel::REG_POWER_CTL_ADXL345, 0x00, clock)?;
                self.adxl_set_reg(a, accel::REG_FIFO_CTL_ADXL345, 0x00, None)?;
                self.adxl_set_reg(a, accel::REG_BW_RATE_ADXL345, bw, None)?;
                self.adxl_set_reg(a, accel::REG_FIFO_CTL_ADXL345, 0x80, None)?;
            }
        }
        let print_time = self.core().last_move_time();
        self.adxl.last_tx_time = print_time;
        self.adxl.query_rate = rate;
        let mut measure_time = measure_time;
        if measure_time > 200 {
            self.log(format!("start_measurements measure_time errer:{measure_time}"));
            measure_time = 200;
        }
        measure_time = measure_time.max(10);
        let mut c = self.core();
        c.accel = accel::RawSamples { max_size: (6 * rate * measure_time).max(0) as usize, size_per_pack: 48, ..Default::default() };
        let m = c.mcu(Target::Toolhead).map_err(|e| Abort(e.to_string()))?;
        let reqclock = m.clock.clock(print_time) as u32;
        let rest_ticks = (4.0 / rate as f64 * m.clock.freq) as u64 as u32;
        let name = if a.kind == AdxlType::Lis2dw12 { "query_lis2dw" } else { "query_adxl345" };
        let msg = m.encode(name, &[("oid", Arg::Int(toolhead::ACCEL as i64)), ("clock", Arg::Int(reqclock as i64)), ("rest_ticks", Arg::Int(rest_ticks as i64))]);
        m.queue_msg(msg);
        c.wake();
        drop(c);
        self.log(format!("rate : {rate}"));
        self.log(format!("reset_ticks : {rest_ticks}"));
        self.adxl.start1 = now();
        self.adxl.start2 = now();
        Ok(Ok(true))
    }

    /// `finish_measurements` (and the decode its callers run right after).
    pub(super) fn adxl_finish(&mut self, a: &AdxlCfg) -> Result<Results, Abort> {
        if self.adxl.query_rate <= 0 {
            self.log("is_measuring:false");
            return Ok(Results::default());
        }
        let print_time = self.core().last_move_time();
        let oid = toolhead::ACCEL as i64;
        let args = [("oid", Arg::Int(oid)), ("clock", Arg::Int(0)), ("rest_ticks", Arg::Int(0))];
        let (name, resp, seq_key) = match a.kind {
            AdxlType::Lis2dw12 => ("query_lis2dw", "lis2dw_status", "next_sequence"),
            AdxlType::Adxl345 => ("query_adxl345", "adxl345_end", "sequence"),
        };
        let m = super::send_with_response(&self.p, Target::Toolhead, name, &args, resp, oid).map_err(Abort)?;
        let end = now();
        self.adxl.last_tx_time = print_time;
        self.adxl.query_rate = 0;
        let c = self.core();
        let end_sequence = c.accel.convert_sequence(m.int(seq_key).unwrap_or(0));
        let overflows = m.int("limit_count").unwrap_or(0);
        let r = Results::decode(&c.accel, &a.axes_map, end_sequence, overflows, self.adxl.start2, end);
        let size = c.accel.size;
        drop(c);
        self.log(format!("finish measurements,raw_samples.size:{size} limit_count:{overflows}"));
        for l in &r.log {
            self.log(l.clone());
        }
        Ok(r)
    }

    fn cmd_accelerometer_measure(&mut self, a: &AdxlCfg, g: &GCmd) -> CmdResult {
        if self.adxl.query_rate > 0 {
            let name = g.get_string("NAME", &local_stamp());
            // end_query; stock writes the file without decoding first, from
            // an uninitialised sample count: ccos decodes
            let r = self.adxl_finish(a)?;
            let dir = self.core().hw.tmp_dir.clone();
            let _ = std::fs::create_dir_all(&dir);
            let path = dir.join(format!("adxl345-{}-{name}.csv", a.section));
            if let Err(e) = std::fs::write(&path, r.csv()) {
                self.log(format!("open file {} failed: {e}", path.display()));
            }
            return Ok(());
        }
        let rate = g.get_int("RATE", a.data_rate, None, None)?;
        if !accel::QUERY_RATES.iter().any(|(r, _)| *r == rate) {
            self.log(format!("Not a valid adxl345 query rate: {rate}"));
        }
        // a start that fails is only logged
        let _ = self.adxl_start(a, Some(rate), 10)?;
        Ok(())
    }

    fn cmd_accelerometer_query(&mut self, a: &AdxlCfg) -> CmdResult {
        let was = self.adxl.query_rate > 0;
        if was {
            self.log("adxl345 measurements in progress");
        }
        // a chip that does not answer leaves stock reading a null buffer
        if let Err(e) = self.adxl_start(a, None, 10)? {
            return Err(Abort(e));
        }
        let start = now();
        self.adxl_sleep(0.1)?;
        while self.core().accel.size == 0 {
            if now() > start + 3.0 {
                self.log("Timeout reading adxl345 data");
            }
            self.adxl_sleep(0.1)?;
        }
        let r = self.adxl_finish(a)?;
        let (x, y, z) = match r.x.len() {
            0 => (0.0, 0.0, 0.0),
            n => (r.x[n - 1], r.y[n - 1], r.z[n - 1]),
        };
        let line = format!("adxl345 values ({}, {}, {})", cpp_double(x), cpp_double(y), cpp_double(z));
        self.log(format!("adxl345_values = {line}"));
        self.respond(&line);
        Ok(())
    }
}
