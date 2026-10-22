//! `[adxl345 X]` (adx_lis2dw12.cpp): the accelerometer on the toolhead's
//! SPI1, a LIS2DW12 (`adxl_type: lis2dw12`) read by the toolhead MCU
//! (`config_lis2dw`, `query_lis2dw`). The MCU sends 8 samples (48 bytes) per
//! `lis2dw_data`; the handler appends them to one buffer (the stock global
//! `adxl_raw_samples`), which [`Results`] decodes when a measurement ends.

use crate::config::Config;
use crate::machine::{parse_pin, Pin};

/// mm/s² per LSB at ±16 g, 16-bit data (`SCALE_LIS2DW12`).
pub const SCALE_LIS2DW12: f64 = 4.7884033203125;
/// 3.9 mg/LSB (`SCALE_ADXL345`).
pub const SCALE_ADXL345: f64 = 0.0039 * 9.80665 * 1000.;

pub const REG_DEVID_LIS2DW12: u8 = 0x0F;
pub const LIS2DW12_CTRL1: u8 = 0x20;
pub const LIS2DW12_CTRL6: u8 = 0x25;
pub const LIS2DW12_FIFO_CTRL: u8 = 0x2E;
pub const LIS2DW12_ID: u8 = 0x44;
pub const REG_DEVID_ADXL345: u8 = 0x00;
pub const REG_BW_RATE_ADXL345: u8 = 0x2C;
pub const REG_POWER_CTL_ADXL345: u8 = 0x2D;
pub const REG_DATA_FORMAT_ADXL345: u8 = 0x31;
pub const REG_FIFO_CTL_ADXL345: u8 = 0x38;
pub const ADXL345_DEV_ID: u8 = 0xE5;

/// `QUERY_RATES`: valid `rate` values (and the ADXL345 BW_RATE codes).
pub const QUERY_RATES: [(i64, u8); 8] = [(25, 0x8), (50, 0x9), (100, 0xa), (200, 0xb), (400, 0xc), (800, 0xd), (1600, 0xe), (3200, 0xf)];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdxlType {
    Adxl345,
    Lis2dw12,
}

#[derive(Debug, Clone)]
pub struct AdxlCfg {
    /// Section name (`adxl345 X`): the chip's name in the CSV file names.
    pub section: String,
    /// `_` + the section's last word: the command suffix (`_X`).
    pub suffix: String,
    pub kind: AdxlType,
    /// Per printer axis: (chip axis index, scale), from `axes_map`.
    pub axes_map: Vec<(usize, f64)>,
    pub data_rate: i64,
    pub cs_pin: Pin,
    /// `resolve_bus_name`: "0".."3" from `spi0`..`spi3` anywhere in
    /// `spi_bus`, else "".
    pub spi_bus: String,
    pub spi_speed: i64,
    /// What the constructor printed (`axes_map`/`rate` complaints).
    pub warnings: Vec<String>,
}

impl AdxlCfg {
    /// The first `[adxl345 …]` section (stock makes one per section; the
    /// printer has one).
    pub fn from_config(c: &Config) -> Option<Self> {
        let mut log = Vec::new();
        let section = c.section_names().into_iter().find(|s| s.split(' ').next() == Some("adxl345"))?;
        let s = section.as_str();
        let kind = if c.get_string(s, "adxl_type", "adxl345") == "lis2dw12" { AdxlType::Lis2dw12 } else { AdxlType::Adxl345 };
        let scale = if kind == AdxlType::Lis2dw12 { SCALE_LIS2DW12 } else { SCALE_ADXL345 };
        let names = ["x", "y", "z", "-x", "-y", "-z"];
        let map: Vec<String> = c.get_string(s, "axes_map", "x,y,z").split(',').map(str::to_string).collect();
        for m in &map {
            if map.len() != 3 || !names.contains(&m.as_str()) {
                log.push("Invalid adxl345 axes_map parameter".into());
            }
        }
        let axes_map = map
            .iter()
            .filter_map(|m| names.iter().position(|n| n == m))
            .map(|i| (i % 3, if i < 3 { scale } else { -scale }))
            .collect();
        let data_rate = c.get_int_clamped(s, "rate", 3200, 0, 3200);
        if !QUERY_RATES.iter().any(|(r, _)| *r == data_rate) {
            log.push(format!("Invalid rate parameter:{data_rate}"));
        }
        let bus = c.get_string(s, "spi_bus", "");
        let spi_bus = ["spi0", "spi1", "spi2", "spi3"].iter().position(|b| bus.contains(b)).map_or(String::new(), |i| i.to_string());
        Some(Self {
            suffix: format!("_{}", s.split(' ').next_back().unwrap_or("")),
            section: section.clone(),
            kind,
            axes_map,
            data_rate,
            cs_pin: parse_pin(&c.get_string(s, "cs_pin", "")),
            spi_bus,
            spi_speed: c.get_int_clamped(s, "spi_speed", 5_000_000, 100_000, i64::MAX),
            warnings: log,
        })
    }

    pub fn type_name(&self) -> &'static str {
        match self.kind {
            AdxlType::Lis2dw12 => "lis2dw12",
            AdxlType::Adxl345 => "adxl345",
        }
    }
}

/// `struct spi_data`: every `lis2dw_data` payload of a measurement, back to
/// back.
#[derive(Debug, Clone, Default)]
pub struct RawSamples {
    pub datas: Vec<u8>,
    /// Packets stored.
    pub size: usize,
    pub size_per_pack: usize,
    pub size_last_pack: usize,
    pub error: u32,
    pub max_size: usize,
    /// `m_last_sequence`: the packet sequence, extended past 16 bits.
    pub last_sequence: i64,
}

impl RawSamples {
    /// `_handle_adxl345_data`.
    pub fn handle_data(&mut self, sequence16: i64, data: &[u8]) -> Option<String> {
        let last = self.last_sequence;
        let mut seq = (last & !0xffff) | sequence16;
        if seq < last {
            seq += 0x10000;
        }
        self.last_sequence = seq;
        if self.size_per_pack == 0 {
            self.size_per_pack = data.len();
        } else if self.size_per_pack != data.len() {
            if self.size_last_pack == 0 {
                self.size_last_pack = data.len();
            } else {
                self.error |= 0x100;
                return Some(format!("size_per_pack error {} {}", self.size_per_pack, self.size_last_pack));
            }
        }
        if self.size * self.size_per_pack + self.size_per_pack > self.max_size {
            self.error |= 0x200;
            return None;
        }
        let at = self.size * self.size_per_pack;
        if self.datas.len() < at + data.len() {
            self.datas.resize(at + data.len(), 0);
        }
        self.datas[at..at + data.len()].copy_from_slice(data);
        self.size += 1;
        None
    }

    /// `_convert_sequence`.
    pub fn convert_sequence(&self, sequence16: i64) -> i64 {
        let mut seq = (self.last_sequence & !0xffff) | sequence16;
        if seq < self.last_sequence {
            seq += 0x10000;
        }
        seq
    }
}

/// `ADXL345Results`: decoded samples (mm/s², printer axes).
#[derive(Debug, Clone, Default)]
pub struct Results {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub z: Vec<f64>,
    pub total_count: i64,
    pub time_per_sample: f64,
    pub seq_to_time: f64,
    pub drops: i64,
    pub overflows: i64,
    /// `m_samp_time`: the time of the last sample from the first.
    pub samp_time: f64,
    pub log: Vec<String>,
}

impl Results {
    /// `setup_data` + `decode_samples`. Times are the host's monotonic
    /// clock at the start/stop requests, as stock uses them.
    pub fn decode(raw: &RawSamples, axes_map: &[(usize, f64)], end_sequence: i64, overflows: i64, start2: f64, end2: f64) -> Results {
        let mut r = Results { overflows, ..Default::default() };
        if raw.size == 0 || end_sequence == 0 || axes_map.len() < 3 {
            return r;
        }
        let per = raw.size_per_pack as i64;
        r.total_count = (end_sequence - 1) * per / 6 + raw.size_last_pack as i64 / 6;
        let total_time = end2 - start2;
        r.time_per_sample = total_time / r.total_count as f64;
        r.seq_to_time = r.time_per_sample * per as f64 / 6.;
        // `int actual_count = … / 6.`: computed in double, truncated
        let actual = ((raw.size as f64 - 1.) * per as f64 / 6. + (raw.size_last_pack / 6) as f64) as i64;
        r.drops = r.total_count - actual;
        r.log.push(format!("m_time_per_sample={}", crate::gcode::parse::cpp_double(r.time_per_sample)));
        r.log.push(format!(
            "sample_hz={} total_time={} m_total_count={} actual_count={actual}",
            crate::gcode::parse::cpp_double(r.total_count as f64 / total_time),
            crate::gcode::parse::cpp_double(total_time),
            r.total_count
        ));
        let count0 = raw.size_per_pack;
        let mut sdata = vec![0.0f64; count0 / 2];
        let (mut i_last, mut j_last) = (0usize, 0usize);
        for i in 0..raw.size {
            let count_i = if i == raw.size - 1 && raw.size_last_pack != 0 { raw.size_last_pack } else { count0 };
            if count0 < count_i {
                continue;
            }
            let d = &raw.datas[i * count0..];
            let mut j = 0;
            while j + 1 < count0 {
                let (lo, hi) = (*d.get(j).unwrap_or(&0) as i32, *d.get(j + 1).unwrap_or(&0) as i32);
                sdata[j / 2] = ((lo | (hi << 8)) - ((hi & 0x80) << 9)) as f64;
                j += 2;
            }
            i_last = i;
            for j in 0..count_i / 6 {
                j_last = j;
                r.x.push(sdata[j * 3 + axes_map[0].0] * axes_map[0].1);
                r.y.push(sdata[j * 3 + axes_map[1].0] * axes_map[1].1);
                r.z.push(sdata[j * 3 + axes_map[2].0] * axes_map[2].1);
            }
        }
        r.samp_time = i_last as f64 * r.seq_to_time + j_last as f64 * r.time_per_sample;
        r
    }

    /// `write_to_file`: the sample index as the time column.
    pub fn csv(&self) -> String {
        let mut s = String::from("#time,accel_x,accel_y,accel_z\n");
        for i in 0..self.x.len() {
            s.push_str(&format!("{i},{:.6},{:.6},{:.6}\n", self.x[i], self.y[i], self.z[i]));
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_packs_and_axes() {
        // axes_map x,z,-y on a LIS2DW12
        let c = Config::parse("[adxl345 X]\nadxl_type : lis2dw12\naxes_map : x,z,-y\nrate : 1600\ncs_pin : stm32:PA4\nspi_bus : stm32:spi1\n");
        let a = AdxlCfg::from_config(&c).unwrap();
        assert!(a.warnings.is_empty());
        assert_eq!((a.suffix.as_str(), a.spi_bus.as_str(), a.spi_speed), ("_X", "1", 5_000_000));
        assert_eq!(a.axes_map, vec![(0, SCALE_LIS2DW12), (2, SCALE_LIS2DW12), (1, -SCALE_LIS2DW12)]);
        let mut raw = RawSamples { max_size: 6 * 1600 * 10, ..Default::default() };
        raw.size_per_pack = 48;
        let sample = |x: i16, y: i16, z: i16| [x.to_le_bytes(), y.to_le_bytes(), z.to_le_bytes()].concat();
        let pack: Vec<u8> = (0..8).flat_map(|_| sample(100, -2048, 10)).collect();
        assert!(raw.handle_data(0, &pack).is_none());
        assert!(raw.handle_data(1, &sample(1, 2, 3)).is_none());
        let r = Results::decode(&raw, &a.axes_map, raw.convert_sequence(2), 0, 0.0, 9.0 / 1600.0);
        assert_eq!(r.x.len(), 9);
        assert_eq!(r.total_count, 9);
        assert!((r.x[0] - 100.0 * SCALE_LIS2DW12).abs() < 1e-9);
        assert!((r.y[0] - 10.0 * SCALE_LIS2DW12).abs() < 1e-9);
        assert!((r.z[0] - 2048.0 * SCALE_LIS2DW12).abs() < 1e-9);
        assert!((r.x[8] - SCALE_LIS2DW12).abs() < 1e-9);
        // a third size is refused
        assert!(raw.handle_data(2, &[0; 12]).is_some());
        assert_eq!(raw.error, 0x100);
    }
}
