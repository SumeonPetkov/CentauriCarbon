//! The load cells under the bed and the direct Z control, as the stock
//! firmware configures them (extras/hx711s.cpp, extras/strain_gauge.cpp,
//! extras/dirzctl.cpp). The probing itself is in `gcode/loadcell.rs`.
//!
//! The load-cell board (`strain_gauge_mcu`) does the measuring: it tares
//! the four cells (`calibration_sample`), and once armed (`sg_probe_check`)
//! it reports a press with the MCU tick it happened at (`sg_resp`). The
//! host steps Z itself through `dirzctl` on the DSP, a timer that toggles
//! the step pin, and works out where Z was at that tick.

use crate::config::Config;
use crate::machine::{parse_pin, Pin};

/// `[hx711s]` (`HX711S::HX711S`). Out-of-range values are used as they are
/// (`GetInt` only logs).
#[derive(Debug, Clone)]
pub struct Hx711sCfg {
    pub enable_test: i64,
    pub count: i64,
    pub rest_ticks: i64,
    pub install_dir: i64,
    pub enable_hpf: i64,
    pub find_index_mode: i64,
    pub enable_shake_filter: i64,
    pub enable_channels: i64,
    pub sg_mode: i64,
    pub th_k: i64,
    pub k_slope: i64,
    pub bias_slope: i64,
    pub kalman_q: i64,
    pub kalman_r: i64,
    pub min_th: i64,
    pub max_th: i64,
    pub clk_pins: Vec<Pin>,
    pub sdo_pins: Vec<Pin>,
}

impl Hx711sCfg {
    pub fn from_config(c: &Config) -> Option<Self> {
        const S: &str = "hx711s";
        if !c.has_section(S) {
            return None;
        }
        let count = c.get_int(S, "count", 3);
        let pins = |kind: &str| (0..count.max(0)).map(|i| parse_pin(&c.get_string(S, &format!("sensor{i}_{kind}_pin"), ""))).collect();
        Some(Self {
            enable_test: c.get_int(S, "enable_test", 0),
            count,
            rest_ticks: c.get_int(S, "rest_ticks", 5000),
            install_dir: c.get_int(S, "install_dir", 15),
            enable_hpf: c.get_int(S, "enable_hpf", 1),
            find_index_mode: c.get_int(S, "find_index_mode", 1),
            enable_shake_filter: c.get_int(S, "enable_shake_filter", 1),
            enable_channels: c.get_int(S, "enable_channels", 31),
            sg_mode: c.get_int(S, "sg_mode", 4),
            th_k: c.get_int(S, "th_k", 2000),
            k_slope: c.get_int(S, "k_slope", 1000),
            bias_slope: c.get_int(S, "bias_slope", 120),
            kalman_q: c.get_int(S, "kalman_q", 1),
            kalman_r: c.get_int(S, "kalman_r", 10),
            min_th: c.get_int(S, "min_th", 2000),
            max_th: c.get_int(S, "max_th", 6000),
            clk_pins: pins("clk"),
            sdo_pins: pins("sdo"),
        })
    }

    /// `HX711S::_build_config`: the fields packed as the MCU unpacks them.
    pub fn config_cmds(&self, oid: u8, pin_number: impl Fn(&Pin) -> Option<u32>) -> Result<Vec<String>, String> {
        let hx711_count = ((self.install_dir & 0x0F) << 4) | (self.count & 0x0F);
        let rest_ticks = (((self.enable_test & 0x0F) << 28) | ((self.sg_mode & 0x0F) << 24) | (self.rest_ticks & 0xFF_FFFF)) as u32;
        let channels = ((self.enable_shake_filter & 0x0F) << 16) | ((self.find_index_mode & 0x0F) << 12) | ((self.enable_hpf & 0x0F) << 8) | (self.enable_channels & 0xFF);
        // int arithmetic: k_slope << 16 wraps like the C int
        let k = ((self.k_slope as i32).wrapping_shl(16)) | (((self.bias_slope & 0xFF) << 8) as i32) | ((self.th_k & 0xFF) as i32);
        let mut v = vec![format!(
            "config_hx711s oid={oid} hx711_count={hx711_count} channels={channels} rest_ticks={rest_ticks} kalman_q={} kalman_r={} max_th={} min_th={} k={}",
            self.kalman_q as i32 as u32, self.kalman_r as i32 as u32, self.max_th as i32 as u32, self.min_th as i32 as u32, k as u32
        )];
        for i in 0..self.clk_pins.len() {
            let clk = pin_number(&self.clk_pins[i]).ok_or_else(|| format!("bad pin {}", self.clk_pins[i].name))?;
            let sdo = pin_number(&self.sdo_pins[i]).ok_or_else(|| format!("bad pin {}", self.sdo_pins[i].name))?;
            v.push(format!("add_hx711s oid={oid} index={i} clk_pin={clk} sdo_pin={sdo}"));
        }
        Ok(v)
    }
}

/// `[strain_gauge]` (`StrainGaugeCFG`), the values the Z probing uses.
#[derive(Debug, Clone, Copy)]
pub struct StrainGaugeCfg {
    pub enable_z_home: bool,
    pub use_probe_by_step: bool,
    pub hot_min_temp: f64,
    pub g28_sta0_speed: f64,
    pub g28_sta1_speed: f64,
    pub g28_max_err: f64,
    pub g28_max_try: f64,
    pub g28_wait_cool_down: bool,
    /// `[stepper_z] position_max`: the longest probe.
    pub max_z: f64,
    pub g29_xy_speed: f64,
    pub g28_min_hold: i64,
    pub g28_max_hold: i64,
    pub shake_range: f64,
    pub self_z_offset: f64,
    pub g29_speed: f64,
    /// Probes go down at most Z − this (it is negative).
    pub g29_max_detection: f64,
    // PRTOUCH_READY, CHECK_BED_MESH, MEASURE_GAP_TEST
    pub base_count: i64,
    /// Integer in the stock code; the stock printer.cfg's 20 is clamped to 10.
    pub bed_max_err: i64,
    pub g29_rdy_speed: f64,
    pub shake_cnt: i64,
    pub need_measure_gap: bool,
    pub gap_dis_range: f64,
    pub check_bed_mesh_max_err: f64,
}

impl StrainGaugeCfg {
    pub fn from_config(c: &Config) -> Option<Self> {
        const S: &str = "strain_gauge";
        if !c.has_section(S) {
            return None;
        }
        Some(Self {
            enable_z_home: c.get_bool(S, "enable_z_home", true),
            use_probe_by_step: c.get_bool(S, "use_probe_by_step", false),
            hot_min_temp: c.get_double(S, "s_hot_min_temp", 140.0),
            g28_sta0_speed: c.get_double(S, "g28_sta0_speed", 2.0),
            g28_sta1_speed: c.get_double(S, "g28_sta1_speed", 2.5),
            g28_max_err: c.get_double(S, "g28_max_err", 0.1),
            g28_max_try: c.get_double(S, "g28_max_try", 4.0),
            g28_wait_cool_down: c.get_bool(S, "g28_wait_cool_down", false),
            max_z: c.get_double("stepper_z", "position_max", 300.0),
            g29_xy_speed: c.get_double(S, "g29_xy_speed", 150.0),
            g28_min_hold: c.get_int(S, "g28_min_hold", 3000),
            g28_max_hold: c.get_int(S, "g28_max_hold", 50000),
            shake_range: c.get_double(S, "shake_range", 0.5),
            self_z_offset: c.get_double(S, "self_z_offset", 0.0),
            g29_speed: c.get_double(S, "g29_speed", 2.0),
            g29_max_detection: c.get_double(S, "g29_max_detection", -5.0),
            base_count: c.get_int_clamped(S, "base_count", 40, 10, 100),
            bed_max_err: c.get_int_clamped(S, "bed_max_err", 2, 2, 10),
            g29_rdy_speed: c.get_double(S, "g29_rdy_speed", 2.5),
            shake_cnt: c.get_int_clamped(S, "shake_cnt", 4, 1, 512),
            need_measure_gap: c.get_bool(S, "need_measure_gap", true),
            gap_dis_range: c.get_double(S, "gap_dis_range", 0.6),
            check_bed_mesh_max_err: c.get_double(S, "check_bed_mesh_max_err", 0.2),
        })
    }
}

/// One `result_dirzctl`: full steps still to go, the DSP tick, and that
/// tick as host time.
#[derive(Debug, Clone, Copy)]
pub struct DirzResult {
    pub step: u32,
    pub tick: u32,
    pub time: f64,
}

/// What the response handlers keep (`HX711S::_handle_sg_resp`,
/// `_handle_debug_hx711s`, `DirZCtl::_handle_result_dirzctl`).
#[derive(Debug, Clone, Default)]
pub struct SgState {
    pub probe_calibration_response: u32,
    pub probe_cmd_response: u32,
    /// bit 7: tared; bits 0–3: that cell looked broken.
    pub is_calibration: u8,
    /// > 0: pressed (0x40 over max_th, 0x20/0x60 other rules; 0x80 =
    /// a cell mounted the wrong way round).
    pub is_trigger: u8,
    pub trigger_tick: u32,
    pub trigger_timestamp: f64,
    pub time_timestamp: f64,
    pub dirzctl: Vec<DirzResult>,
    /// `sg_resp` messages seen (diagnostics).
    pub reports: u64,
}
