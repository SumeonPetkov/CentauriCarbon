//! The load-cell board's firmware (`mcu/src/hx711s.c`, `local_util.c`,
//! `filter.c`, `stm32/hx717_exit.c`), for the board as it is strapped.
//!
//! `config_hx711s` ignores the host's `sg_mode` and reads the PB3/PD2
//! straps. The emulated board reads "HX717 half bridge" (3), matching
//! printer.cfg's `sg_mode: 3`: one HX717 measures the four half-bridge
//! cells as one bridge. It sits on sensor line 2 (SCK PC6, DOUT PB15), the
//! only one `HX717_init` gives an interrupt (`EXTI_init(2)`). It converts
//! continuously at 320 SPS. The task samples every 1.5 ms, works on the sum
//! of the four lines (the other three read 0), and reports a press with the
//! tick of the knee in the last 16 conversions.
//!
//! The firmware's quirks are kept:
//! * the filter statics (`data_num`, `last_out`, the calibration min/max)
//!   are never reset;
//! * while the window fills, the average is taken over the oldest slots;
//! * the "other trigger" rule (0x20) can never fire: its first slope test
//!   is 0 at i = 0;
//! * `out_index` keeps its old value when the first sample is the minimum;
//! * a stop multiplies `rest_ticks` by 1000.

/// `MAX_DATA_NUM`, `MAX_SENSOR_NUM` (local_util.h).
const MAX_DATA_NUM: usize = 30;
const MAX_SENSOR_NUM: usize = 5;

const SG_MODE_HX711_FULL_BRIDGE: u32 = 0;
const SG_MODE_HX717_HALF_BRIDGE: u32 = 3;
/// What `gpio_in_read(PB3) | gpio_in_read(PD2) << 1` gives on this board.
pub const STRAPPED_SG_MODE: u32 = SG_MODE_HX717_HALF_BRIDGE;
/// HX717 output data rate.
pub const HX717_PERIOD: f64 = 1.0 / 320.0;
/// The DOUT line with the interrupt.
pub const HX717_LINE: usize = 2;

const PROBE_CHECK_MODE_START: i32 = 1;
const PROBE_CHECK_MODE_STOP: i32 = 2;
const PROBE_CHECK_MODE_CALIBRATION: i32 = 3;

/// `filter.c`: first-order high-pass.
#[derive(Debug, Clone, Copy, Default)]
struct Hpf {
    vi: i32,
    vi_prev: i32,
    vo: i32,
    vo_prev: i32,
    cutoff_frq_hz: f32,
    acq_frq_hz: f32,
}

impl Hpf {
    fn new(cut: f32, acq: f32) -> Self {
        Self { cutoff_frq_hz: cut, acq_frq_hz: acq, ..Default::default() }
    }

    fn filter(&mut self, new_val: i32) -> i32 {
        self.vi = new_val;
        let rc = (1.0f64 / 2.0 / 3.14159 / self.cutoff_frq_hz as f64) as f32;
        let coff = rc / (rc + 1.0 / self.acq_frq_hz);
        self.vo = (self.vi.wrapping_sub(self.vi_prev).wrapping_add(self.vo_prev) as f32 * coff) as i32;
        self.vo_prev = self.vo;
        self.vi_prev = self.vi;
        self.vo
    }
}

/// One `sg_resp`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SgResp {
    pub vd: u8,
    pub it: u8,
    pub nt: u32,
    pub r: u32,
    pub tt: u32,
}

/// `debug_hx711s` arguments.
pub type Debug4 = [u32; 4];

pub struct Hx711s {
    pub oid: u8,
    count: u32,
    install_dir: u32,
    sg_mode: u32,
    max_data_num: usize,
    rest_ticks: u32,
    ori_rest_ticks: u32,
    enable_channels: u32,
    enable_hpf: u32,
    enable_shake_filter: u32,
    find_index_mode: u32,
    heartbeat_period: u32,
    kalman_q0: i32,
    kalman_r0: i32,
    max_th: i32,
    min_th: i32,
    th_k: i32,
    k_slope: i32,
    bias_slope: i32,
    start: bool,
    /// The task timer (`hx711s_timer`).
    pub wake: Option<u64>,
    times_read: i32,
    is_bottom_detection: bool,
    probe_check_cmd: i32,
    is_calibration: u8,
    is_trigger: u8,
    trigger_index: u8,
    now_trigger: u8,
    trigger_tick: u32,
    init_values: [i32; 4],
    amplitude_values: [i32; 4],
    sample_values: [i32; 4],
    calibration_values: [i32; 4],
    fusion_filter_value: i32,
    time_stamp: [u32; 5],
    // hx717_exit.c
    exi_data: [i32; 5],
    exi_tick: [u64; 5],
    /// Next HX717 conversion (DOUT falling edge).
    pub exi_next: u64,
    exi_period: u64,
    // statics (hx711s.c, local_util.c)
    cal_num: [usize; 4],
    cal_samples: Box<[[i32; 1000]; 4]>,
    sensor_min: [i32; 4],
    sensor_max: [i32; 4],
    data_list: [[i32; MAX_DATA_NUM]; MAX_SENSOR_NUM],
    filter_data_list: [[i32; MAX_DATA_NUM]; MAX_SENSOR_NUM],
    timestamp_list: [[u32; MAX_DATA_NUM]; MAX_SENSOR_NUM],
    sw_num: [usize; MAX_SENSOR_NUM],
    sw_last_out: [i32; MAX_SENSOR_NUM],
    hpf: Hpf,
    fusion_hpf: Hpf,
    out_index: i32,
    choosed_sensor: u8,
    task_loop: u32,
    last_rep_loop: u32,
    last_is_calibration: u8,
    last_is_trigger: u8,
    last_tick: u64,
}

fn slide<T: Copy>(a: &mut [T; MAX_DATA_NUM], n: usize, v: T, is_fifo: bool) {
    if is_fifo {
        a.copy_within(0..n - 1, 1);
        a[0] = v;
    } else {
        a.copy_within(1..n, 0);
        a[n - 1] = v;
    }
}

/// `median_filter`: sorts the first `len` in place; reads past them for
/// len 1 as the firmware does.
fn median_filter(a: &mut [i32; 1000], len: usize) -> i32 {
    a[..len].sort_unstable();
    if len & 1 == 1 {
        a[(len + 1) / 2]
    } else {
        a[len / 2].wrapping_add(a[len / 2 + 1]) / 2
    }
}

impl Hx711s {
    /// `command_config_hx711s`. Err = the shutdown it raises.
    pub fn config(a: [u32; 9], now: u64, freq: f64) -> Result<(Self, Debug4), &'static str> {
        let count = a[1] & 0x0F;
        if count > 4 {
            return Err("Max of 4 hx711");
        }
        if count < 1 {
            return Err("Min of 1 hx711");
        }
        let sg_mode = if STRAPPED_SG_MODE == SG_MODE_HX717_HALF_BRIDGE { SG_MODE_HX717_HALF_BRIDGE } else { SG_MODE_HX711_FULL_BRIDGE };
        let mut find_index_mode = (a[2] & 0xF000) >> 12;
        let (sample_period, max_data_num) = if sg_mode == SG_MODE_HX717_HALF_BRIDGE {
            find_index_mode = 0;
            (1500, 16)
        } else {
            (a[3] & 0xFF_FFFF, 12)
        };
        // CONFIG_CLOCK_FREQ / 1000000 * sample_period
        let rest_ticks = (freq as u32 / 1_000_000).wrapping_mul(sample_period);
        let exi_period = (HX717_PERIOD * freq) as u64;
        let h = Self {
            oid: a[0] as u8,
            count,
            install_dir: (a[1] >> 4) & 0x0F,
            sg_mode,
            max_data_num,
            rest_ticks,
            ori_rest_ticks: rest_ticks,
            enable_channels: a[2] & 0xFF,
            enable_hpf: (a[2] & 0x0F00) >> 8,
            enable_shake_filter: (a[2] & 0xF_0000) >> 16,
            find_index_mode,
            heartbeat_period: 2_000_000 / sample_period.max(1),
            kalman_q0: a[4] as i32,
            kalman_r0: a[5] as i32,
            // set after the half-bridge block: min_th is not forced to 0
            max_th: a[6] as i32,
            min_th: a[7] as i32,
            th_k: (a[8] & 0xFF) as i32,
            k_slope: (a[8] >> 16) as i32,
            bias_slope: ((a[8] & 0xFF00) >> 8) as i32,
            start: false,
            wake: None,
            times_read: 0,
            is_bottom_detection: false,
            probe_check_cmd: 0,
            is_calibration: 0,
            is_trigger: 0,
            trigger_index: 0,
            now_trigger: 0,
            trigger_tick: 0,
            init_values: [0; 4],
            amplitude_values: [0; 4],
            sample_values: [0; 4],
            calibration_values: [0; 4],
            fusion_filter_value: 0,
            time_stamp: [0; 5],
            exi_data: [0; 5],
            exi_tick: [0; 5],
            exi_next: now + exi_period,
            exi_period,
            cal_num: [0; 4],
            cal_samples: Box::new([[0; 1000]; 4]),
            sensor_min: [0; 4],
            sensor_max: [0; 4],
            data_list: [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM],
            filter_data_list: [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM],
            timestamp_list: [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM],
            sw_num: [0; MAX_SENSOR_NUM],
            sw_last_out: [0; MAX_SENSOR_NUM],
            hpf: Hpf::new(5.0, ((1_000_000 / sample_period) / count) as f32),
            fusion_hpf: Hpf::new(5.0, (1_000_000 / sample_period) as f32),
            out_index: 0,
            choosed_sensor: 4,
            task_loop: 0,
            last_rep_loop: 0,
            last_is_calibration: 0,
            last_is_trigger: 0,
            last_tick: 0,
        };
        Ok((h, [a[0], a[1], a[2], a[3]]))
    }

    /// `command_add_hx711s` (the pins are the HX717 driver's own).
    pub fn add(&self, a: [u32; 4]) -> Result<Debug4, &'static str> {
        if a[1] >= self.count {
            return Err("Set hx711 past maximum count");
        }
        Ok(a)
    }

    /// `command_query_hx711s`.
    pub fn query(&mut self, times_read: u32, now: u64) {
        self.times_read = times_read as u16 as i32;
        self.start = true;
        self.wake = Some(now + self.rest_ticks as u64);
    }

    /// `command_sg_probe_check`.
    pub fn probe_check(&mut self, x: i32, y: i32, z: i32, cmd: i32, now: u64) -> Debug4 {
        self.probe_check_cmd = cmd;
        self.wake = None;
        self.is_bottom_detection = x == 0 && y == 0 && z == 0 && cmd == PROBE_CHECK_MODE_START;
        self.is_trigger = 0;
        self.trigger_tick = 0;
        self.now_trigger = 0;
        self.trigger_index = 0;
        self.sample_values = [0; 4];
        self.data_list = [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM];
        self.filter_data_list = [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM];
        self.timestamp_list = [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM];
        if cmd == PROBE_CHECK_MODE_STOP {
            self.start = false;
            self.rest_ticks = self.rest_ticks.wrapping_mul(1000);
        } else if cmd == PROBE_CHECK_MODE_START || cmd == PROBE_CHECK_MODE_CALIBRATION {
            self.rest_ticks = self.ori_rest_ticks;
            self.start = true;
            self.wake = Some(now + self.rest_ticks as u64);
        }
        [self.oid as u32, cmd as u32, y as u32, z as u32]
    }

    /// `command_calibration_sample`.
    pub fn calibration_sample(&mut self, times_read: u32, now: u64) -> Debug4 {
        self.times_read = (times_read as u16 as i32).clamp(1, 1000);
        if times_read as u16 == 0 {
            self.times_read = 50;
        }
        self.rest_ticks = self.ori_rest_ticks;
        self.is_calibration = 0;
        self.init_values = [0; 4];
        self.calibration_values = [0; 4];
        self.sample_values = [0; 4];
        self.data_list = [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM];
        self.timestamp_list = [[0; MAX_DATA_NUM]; MAX_SENSOR_NUM];
        self.is_trigger = 0;
        self.trigger_tick = 0;
        self.probe_check_cmd = 0;
        self.start = true;
        self.wake = Some(now + self.rest_ticks as u64);
        // arg[2], arg[3] read past the two parameters
        [1, self.probe_check_cmd as u32, 0, 0]
    }

    /// `hx711s_shutdown`.
    pub fn shutdown(&mut self) {
        self.times_read = 0;
        self.start = false;
        self.wake = None;
    }

    /// The HX717 on line 2 finished a conversion (`EXTI15_10_IRQHandler`).
    pub fn conversion(&mut self, value: i32, now: u64) {
        // 24-bit two's complement, sign-extended
        let v = value.clamp(-0x80_0000, 0x7F_FFFF);
        self.exi_data[HX717_LINE] = v;
        self.exi_tick[HX717_LINE] = now;
        self.exi_next = now + self.exi_period;
    }

    /// `hx711s_event` then `hx711s_task`.
    pub fn timer(&mut self, now: u64) -> Option<SgResp> {
        let w = self.wake.unwrap_or(now);
        self.wake = self.start.then(|| w + self.rest_ticks as u64);
        self.task(now)
    }

    fn task(&mut self, now: u64) -> Option<SgResp> {
        if !self.start {
            return None;
        }
        // half bridge: no sensor rotation, choosed_sensor stays 4
        self.task_loop = self.task_loop.wrapping_add(1);
        if self.is_calibration & 0x80 == 0 {
            self.last_is_calibration = self.is_calibration;
            self.calibration();
            return None;
        }
        let mut now_tick = now as u32;
        if self.update_from_exi() == 0 {
            return None;
        }
        if (now_tick as u64) < self.last_tick {
            now_tick = now_tick.wrapping_add(0xFFFF_FFFF);
        }
        self.last_tick = now_tick as u64;
        let enable_hpf = self.enable_hpf;
        if self.fusion_filter(enable_hpf) {
            let cs = self.choosed_sensor;
            let trigger = self.trigger_check_new(cs);
            if self.is_trigger == 0 {
                self.is_trigger = (trigger & 0xFF) as u8;
                if trigger > 0 {
                    self.now_trigger = self.is_trigger;
                    self.trigger_index = ((trigger >> 8) & 0xFF) as u8;
                    let s = if self.is_trigger & 0x10 != 0 { 4 } else { cs as usize };
                    self.trigger_tick = self.timestamp_list[s][self.trigger_index as usize % MAX_DATA_NUM];
                }
            }
        }
        let heartbeat = self.heartbeat_period != 0 && self.task_loop % self.heartbeat_period == 0;
        let mut resp = None;
        if heartbeat || self.is_trigger > 0 || self.now_trigger > 0 || self.last_is_calibration != self.is_calibration {
            let since = (self.task_loop.wrapping_sub(self.last_rep_loop) as i32).wrapping_abs();
            if since > 80 || self.last_is_trigger == 0 {
                self.last_rep_loop = self.task_loop;
                resp = Some(SgResp { vd: self.is_calibration, it: self.trigger_index, nt: self.trigger_tick, r: self.now_trigger as u32, tt: self.last_tick as u32 });
                self.times_read = self.times_read.wrapping_sub(1);
            }
            self.last_is_trigger = self.now_trigger;
            self.last_is_calibration = self.is_calibration;
        }
        resp
    }

    /// `update_hx711_data_from_exi`.
    fn update_from_exi(&mut self) -> u8 {
        let mut ret = 0;
        let mut tmp_tick = self.exi_tick[0];
        if self.th_k < 1 {
            // the firmware then divides by the old value (0); 1 is meant
            self.th_k = 1;
        }
        let bits = self.th_k;
        for j in 0..self.count as usize {
            self.exi_data[j] -= self.exi_data[j] % bits;
            if self.sample_values[j] != self.exi_data[j] {
                self.time_stamp[j] = self.exi_tick[j] as u32;
                self.sample_values[j] = self.exi_data[j];
                tmp_tick = tmp_tick.max(self.exi_tick[j]);
                ret |= 1 << j;
            }
        }
        self.time_stamp[4] = tmp_tick as u32;
        ret
    }

    /// `calibration_strain_gauge` (HX717 path).
    fn calibration(&mut self) -> bool {
        let n = self.count as usize;
        if self.times_read == 0 {
            self.is_calibration = 0x80;
            for j in 0..n {
                if self.cal_num[j] > 0 {
                    self.amplitude_values[j] = self.sensor_max[j].wrapping_sub(self.sensor_min[j]);
                    self.init_values[j] = median_filter(&mut self.cal_samples[j], self.cal_num[j]);
                    if self.init_values[j].wrapping_abs() < self.kalman_q0.wrapping_abs() {
                        self.is_calibration |= 1 << j;
                    }
                    if self.amplitude_values[j] < self.kalman_r0.wrapping_abs() {
                        self.is_calibration |= 1 << j;
                    }
                } else {
                    self.is_calibration = 0;
                    return false;
                }
                self.cal_num[j] = 0;
            }
            return true;
        }
        if self.update_from_exi() != 0 {
            self.times_read -= 1;
            for j in 0..n {
                let v = self.sample_values[j];
                if self.sensor_min[j] == 0 {
                    self.sensor_min[j] = v;
                }
                if self.sensor_max[j] == 0 {
                    self.sensor_max[j] = v;
                }
                if self.cal_num[j] < 1000 {
                    self.cal_samples[j][self.cal_num[j]] = v;
                }
                self.sensor_min[j] = self.sensor_min[j].min(v);
                self.sensor_max[j] = self.sensor_max[j].max(v);
                self.cal_num[j] += 1;
            }
        }
        true
    }

    /// `hx711s_fusion_filter` (HX717 half bridge: the sum only).
    fn fusion_filter(&mut self, enable_hpf: u32) -> bool {
        if self.is_calibration == 0 {
            return false;
        }
        for i in 0..self.count as usize {
            self.calibration_values[i] = if (self.install_dir >> i) & 1 != 0 {
                self.init_values[i].wrapping_sub(self.sample_values[i])
            } else {
                self.sample_values[i].wrapping_sub(self.init_values[i])
            };
        }
        let mut fusion = self.calibration_values[..self.count as usize].iter().fold(0i32, |s, v| s.wrapping_add(*v));
        let n = self.max_data_num;
        let ts = self.time_stamp[4];
        let ok = self.sliding_window(4, n, n - 2, &mut fusion, ts, 600_000, false, enable_hpf);
        self.fusion_filter_value = fusion;
        ok
    }

    /// `sliding_window_avg_exception_filter`.
    #[allow(clippy::too_many_arguments)]
    fn sliding_window(&mut self, index: usize, max_n: usize, win_n: usize, data: &mut i32, ts: u32, exception_th: i32, is_fifo: bool, enable_hpf: u32) -> bool {
        if max_n < 2 || max_n < win_n || max_n > MAX_DATA_NUM || index >= MAX_SENSOR_NUM {
            return false;
        }
        let mut filter_data = *data;
        if enable_hpf != 0 {
            filter_data = if index == 4 { self.fusion_hpf.filter(*data) } else { self.hpf.filter(*data) };
        }
        if exception_th > 0 {
            let tmp = self.sw_last_out[index].wrapping_sub(filter_data);
            if tmp.wrapping_abs() > exception_th && self.sw_last_out[index] != 0 {
                *data = self.sw_last_out[index];
                return false;
            }
        }
        slide(&mut self.data_list[index], max_n, filter_data, is_fifo);
        slide(&mut self.filter_data_list[index], max_n, *data, is_fifo);
        slide(&mut self.timestamp_list[index], max_n, ts, is_fifo);
        let mut out;
        if self.sw_num[index] < max_n {
            // averages the first slots, which hold the oldest samples
            self.sw_num[index] += 1;
            let k = self.sw_num[index];
            out = self.data_list[index][..k].iter().fold(0i32, |s, v| s.wrapping_add(*v)) / k as i32;
        } else {
            let mut a = self.data_list[index];
            a[..max_n].sort_unstable();
            let start = (max_n - win_n) / 2;
            out = a[start..start + win_n].iter().fold(0i32, |s, v| s.wrapping_add(*v)) / win_n as i32;
        }
        if out == 0 {
            out = 1;
        }
        self.sw_last_out[index] = out;
        true
    }

    /// `trigger_check_new`: low byte = flags (0x10 fused, 0x40 over max_th),
    /// next byte = the index of the knee.
    fn trigger_check_new(&mut self, choosed: u8) -> i32 {
        if self.is_bottom_detection {
            self.enable_shake_filter = 0;
        }
        if (self.enable_channels >> 4) & 1 != 0 {
            let flag = self.check_tregger(4);
            if flag != 0 {
                let idx = self.find_trigger_index_new(4) as u8;
                return 0x10 | (flag & 0x60) as i32 | (idx as i32) << 8 | 1 << choosed;
            }
        }
        if (self.enable_channels >> choosed) & 1 != 0 {
            let s = choosed as usize % MAX_SENSOR_NUM;
            let flag = self.check_tregger(s);
            if flag != 0 {
                let idx = self.find_trigger_index_new(s) as u8;
                return 1 << choosed | (flag & 0x60) as i32 | (idx as i32) << 8;
            }
        }
        0
    }

    /// `check_tregger`.
    fn check_tregger(&self, s: usize) -> u8 {
        let d = &self.data_list[s];
        let n = self.max_data_num;
        let abs = |v: i32| v.wrapping_abs();
        if self.probe_check_cmd == PROBE_CHECK_MODE_CALIBRATION {
            let avg = d[..n].iter().fold(0i32, |s, v| s.wrapping_add(*v)) / n as i32;
            if avg < -self.min_th {
                return 0x60;
            }
        }
        if d[n - 1] <= -self.max_th && abs(d[0]) > 0 {
            if self.enable_shake_filter != 0 {
                let from = if self.sg_mode == SG_MODE_HX717_HALF_BRIDGE { n - 10 } else { 0 };
                if d[from..n].iter().any(|&v| v > self.min_th) {
                    return 0;
                }
            }
            return 0x40;
        }
        if !(abs(d[n - 1]) > abs(d[n - 2]) && abs(d[n - 2]) > abs(d[n - 3])) {
            return 0;
        }
        for i in 0..n - 3 {
            if abs(d[n - 1]) > d[i] || abs(d[n - 2]) > d[i] || abs(d[n - 3]) > d[i] {
                return 0;
            }
        }
        let (mut vmin, mut vmax) = (0xFFFF_FFFFu32 as f64, -(0x00FF_FFFF as f64));
        for &v in &d[..n] {
            vmin = vmin.min(v as f64);
            vmax = vmax.max(v as f64);
        }
        let err = vmax - vmin;
        let p: Vec<f64> = d[..n].iter().map(|&v| ((v as f64 - vmin) / err) as f32 as f64).collect();
        for i in 0..n - 1 {
            let k = (p[0] - p[i]) / ((n - i) as f32 / n as f32) as f64;
            // util_abs takes an int
            if ((k as i32).wrapping_abs() as f64) < 0.8 {
                return 0;
            }
            if self.enable_shake_filter != 0 && k > 0.8 {
                return 0;
            }
        }
        if abs(d[n - 1]) < self.min_th {
            return 0;
        }
        if self.enable_shake_filter != 0 && d[n - 1] > self.min_th {
            return 0;
        }
        0x20
    }

    /// `find_trigger_index_new`: the knee of the window, by rotating the
    /// normalised curve until its ends are level.
    fn find_trigger_index_new(&mut self, s: usize) -> i32 {
        let a = self.data_list[s];
        let n = self.max_data_num;
        let mut val = [0f64; MAX_DATA_NUM];
        let flip = a[n - 1].wrapping_sub(a[0]) < 0;
        for i in 0..n {
            val[i] = if flip { (a[i] as f32 * -1.0) as f64 } else { a[i] as f64 };
        }
        let (mut vmin, mut vmax) = (val[n - 1], val[0]);
        for &v in &val[..n] {
            vmin = if vmin > v { v } else { vmin };
            vmax = if vmax < v { v } else { vmax };
        }
        let err = vmax - vmin;
        let mut vt = [0f64; MAX_DATA_NUM];
        for i in 0..n {
            vt[i] = (val[i] - vmin) / err;
        }
        let angle = ((vt[n - 1] - vt[0]) / (n - 1) as f64).atan();
        let (sin, cos) = ((-angle).sin(), (-angle).cos());
        for (i, v) in vt.iter_mut().enumerate().take(n) {
            *v = i as f64 * sin + *v * cos;
        }
        let mut min_val = vt[0];
        for (i, &v) in vt.iter().enumerate().take(n) {
            if min_val > v {
                min_val = v;
                self.out_index = i as i32;
            }
        }
        let oi = self.out_index.clamp(0, n as i32 - 1) as usize;
        let kk = ((val[n - 1] - val[oi]) / (n as i32 - self.out_index) as f64) as i32;
        let fix = if self.find_index_mode & 0x08 != 0 {
            kk.wrapping_mul(self.k_slope) / 10000 - self.bias_slope / 10
        } else {
            match kk {
                k if k > 2300 => 11,
                k if k > 2200 => 10,
                k if k > 2100 => 9,
                k if k > 2000 => 8,
                k if k > 1900 => 7,
                k if k > 1700 => 6,
                k if k > 1600 => 5,
                k if k > 1500 => 4,
                k if k > 1300 => 3,
                k if k > 1000 => 2,
                k if k > 900 => 1,
                k if k > 800 => 0,
                k if k > 700 => -1,
                k if k > 600 => -2,
                k if k > 500 => -3,
                k if k > 400 => -4,
                _ => -5,
            }
        };
        let th = self.min_th as f64;
        match self.find_index_mode & 0x06 {
            0x02 => {
                if let Some(i) = (0..n).rev().find(|&i| th > val[i]) {
                    self.out_index = i as i32;
                }
            }
            0x04 => {
                if let Some(i) = (0..n).find(|&i| th < val[i]) {
                    self.out_index = i as i32;
                }
            }
            _ => {}
        }
        if self.find_index_mode & 0x01 != 0 {
            self.out_index = (self.out_index + fix).clamp(0, n as i32 - 1);
        }
        self.out_index
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const F: f64 = 84e6;

    /// printer.cfg [hx711s] as HX711S::_build_config packs it.
    fn configured() -> Hx711s {
        let k = (58u32 << 16) | (42 << 8) | 1;
        let (h, _) = Hx711s::config([0, 4, (1 << 16) | (4 << 12) | 16, (3 << 24) | 3125, 10000, 100, 6000, 2000, k], 0, F).unwrap();
        h
    }

    /// Drive the board: conversions every 1/320 s from `value(t)`, task
    /// every 1.5 ms; returns the reports.
    fn run(h: &mut Hx711s, from: f64, to: f64, value: impl Fn(f64) -> i32) -> Vec<(f64, SgResp)> {
        let mut out = Vec::new();
        let end = (to * F) as u64;
        let mut now = (from * F) as u64;
        loop {
            let next_task = h.wake.unwrap_or(u64::MAX);
            let next = next_task.min(h.exi_next);
            if next > end {
                break;
            }
            now = now.max(next);
            if h.exi_next <= next_task {
                h.conversion(value(now as f64 / F), now);
            } else if let Some(r) = h.timer(now) {
                out.push((now as f64 / F, r));
            }
        }
        out
    }

    #[test]
    fn tares_then_reports_a_press_at_its_knee() {
        let mut h = configured();
        assert_eq!(h.min_th, 2000, "min_th is set after the half-bridge block");
        let noise = |t: f64| ((t * 7919.0).sin() * 30.0) as i32;
        assert_eq!(h.calibration_sample(30, 0), [1, 0, 0, 0]);
        let r = run(&mut h, 0.0, 0.5, |t| 250_000 + noise(t));
        assert!(r.first().is_some_and(|(_, r)| r.vd & 0x80 != 0 && r.r == 0), "tared: {r:?}");
        // pressed from 1.0 s at 60000 counts/mm, 3 mm/s
        h.probe_check(130, 130, 2, 1, (0.6 * F) as u64);
        let press = |t: f64| 250_000 + noise(t) - (((t - 1.0).max(0.0) * 3.0) * 60_000.0) as i32;
        let r = run(&mut h, 0.6, 1.2, press);
        let (at, trig) = r.iter().find(|(_, r)| r.r != 0).copied().expect("triggered");
        assert_eq!(trig.r, 0x50, "fused, over max_th");
        // 6000 counts = 33 ms after contact; the knee is near contact
        assert!(at > 1.03 && at < 1.045, "reported at {at}");
        let knee = trig.nt as f64 / F;
        assert!((knee - 1.0).abs() < 0.005, "knee at {knee}");
    }

    #[test]
    fn stop_and_the_slow_timer() {
        let mut h = configured();
        h.probe_check(0, 0, 0, 2, 0);
        assert!(h.wake.is_none());
        assert_eq!(h.rest_ticks, 126_000 * 1000);
        h.probe_check(10, 10, 0, 1, 0);
        assert_eq!(h.rest_ticks, 126_000);
        assert!(!h.is_bottom_detection);
    }
}
