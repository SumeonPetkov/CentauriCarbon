//! Heaters as Elegoo's firmware runs them (firmware/core/klippy/extras/
//! heaters.cpp, verify_heater.cpp, pid_calibrate.cpp):
//!
//! * PID like Klipper's with `min_deriv_time = smooth_time` (1 s), initial
//!   previous temperature 25 °C;
//! * the PWM is re-sent only when it changes by ≥ 0.05 (or was 0), or every
//!   3.75 s, scheduled 0.5 s after the reading; the MCU switches the heater
//!   off by itself after `MAX_HEAT_TIME` (5 s) without an update;
//! * "reached" for M109/M190 = smoothed temperature within ±2 °C;
//! * an out-of-range target only prints a warning and is set anyway (the
//!   MCU's ADC range check at `max_temp` is the real limit);
//! * extrusion is allowed from `min_extrude_temp − 3 °C`;
//! * `verify_heater`: Klipper's gain check plus a fault when the sensor
//!   reads below −20 °C (open/short thermistor).

/// Klipper scales PID gains by 1/255 (`PID_PARAM_BASE`).
pub const PID_PARAM_BASE: f64 = 255.0;
/// MCU `max_duration` for heater outputs (Define.h `MAX_HEAT_TIME`).
pub const MAX_HEAT_TIME: f64 = 5.0;
/// Readings are applied this long after they were taken (`pwm_delay`).
pub const PWM_DELAY: f64 = 0.5;
pub const TUNE_PID_DELTA: f64 = 5.0;

#[derive(Debug, Clone, Copy)]
pub struct HeaterConfig {
    pub name: &'static str,
    pub beta: f64,
    pub min_temp: f64,
    pub max_temp: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub max_power: f64,
    pub smooth_time: f64,
    pub pwm_cycle_time: f64,
    pub min_extrude_temp: f64,
    pub max_error: f64,
    pub check_gain_time: f64,
    pub hysteresis: f64,
    pub heating_gain: f64,
}

/// What drives the heater.
#[derive(Debug, Clone)]
pub enum Control {
    Pid,
    /// PID_CALIBRATE in progress (Klipper's `ControlAutoTune`).
    AutoTune(AutoTune),
}

#[derive(Debug, Clone)]
pub struct AutoTune {
    /// PID_CALIBRATE TARGET.
    pub calibrate: f64,
    pub heating: bool,
    pub peak: f64,
    pub peak_time: f64,
    pub peaks: Vec<(f64, f64)>,
}

impl AutoTune {
    fn new(calibrate: f64) -> Self {
        Self { calibrate, heating: false, peak: 0.0, peak_time: 0.0, peaks: Vec::new() }
    }

    pub fn busy(&self) -> bool {
        self.heating || self.peaks.len() < 12
    }

    fn check_peaks(&mut self) {
        self.peaks.push((self.peak, self.peak_time));
        self.peak = if self.heating { 9_999_999.0 } else { -9_999_999.0 };
    }

    /// Astrom-Hagglund estimate, Ziegler-Nichols gains (scaled by 255).
    pub fn calc_pid(&self, pos: usize, max_power: f64) -> (f64, f64, f64) {
        let temp_diff = self.peaks[pos].0 - self.peaks[pos - 1].0;
        let time_diff = self.peaks[pos].1 - self.peaks[pos - 2].1;
        let amplitude = 0.5 * temp_diff.abs();
        let ku = 4.0 * max_power / (std::f64::consts::PI * amplitude);
        let tu = time_diff;
        let kp = 0.6 * ku * PID_PARAM_BASE;
        let ki = kp / (0.5 * tu);
        let kd = kp * (0.125 * tu);
        (kp, ki, kd)
    }

    pub fn calc_final_pid(&self, max_power: f64) -> (f64, f64, f64) {
        let mut cycles: Vec<(f64, usize)> = (4..self.peaks.len()).map(|pos| (self.peaks[pos].1 - self.peaks[pos - 2].1, pos)).collect();
        cycles.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mid = cycles[cycles.len() / 2].1;
        self.calc_pid(mid, max_power)
    }
}

/// What the core must do after a reading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PwmOut {
    None,
    /// Send this duty at this print time.
    Send(f64, f64),
}

#[derive(Debug, Clone)]
pub struct Heater {
    pub cfg: HeaterConfig,
    pub target: f64,
    /// Last reading (None until the first one).
    pub temp: Option<f64>,
    pub smoothed: f64,
    pub last_temp_time: f64,
    pub pwm: f64,
    prev_temp: f64,
    prev_time: f64,
    prev_deriv: f64,
    prev_integ: f64,
    last_pwm_value: f64,
    next_pwm_time: f64,
    pub control: Control,
    pub min_extrude_temp: f64,
    pub can_extrude_switch: bool,
    error: f64,
    approaching: bool,
    starting: bool,
    goal_temp: f64,
    goal_time: f64,
    last_target: f64,
    pub next_verify: f64,
}

impl Heater {
    pub fn new(cfg: HeaterConfig) -> Self {
        Self {
            min_extrude_temp: cfg.min_extrude_temp,
            cfg,
            target: 0.0,
            temp: None,
            smoothed: 0.0,
            last_temp_time: 0.0,
            pwm: 0.0,
            prev_temp: 25.0,
            prev_time: 0.0,
            prev_deriv: 0.0,
            prev_integ: 0.0,
            last_pwm_value: 0.0,
            next_pwm_time: 0.0,
            control: Control::Pid,
            can_extrude_switch: true,
            error: 0.0,
            approaching: false,
            starting: false,
            goal_temp: 0.0,
            goal_time: 0.0,
            last_target: 0.0,
            next_verify: 0.0,
        }
    }

    /// `temperature_callback`: a new reading taken at `read_time`.
    pub fn update(&mut self, read_time: f64, temp: f64) -> PwmOut {
        let time_diff = read_time - self.last_temp_time;
        if self.temp.is_none() {
            self.smoothed = temp;
        }
        self.temp = Some(temp);
        self.last_temp_time = read_time;
        let target = self.target;
        let (min_t, max_t, max_power) = (self.cfg.min_temp, self.cfg.max_temp, self.cfg.max_power);
        let value = match &mut self.control {
            Control::Pid => None,
            Control::AutoTune(at) => Some({
                // alter_target clamps to [min_temp, max_temp]
                if at.heating && temp >= target {
                    at.heating = false;
                    at.check_peaks();
                    self.target = (at.calibrate - TUNE_PID_DELTA).clamp(min_t, max_t);
                } else if !at.heating && temp <= target {
                    at.heating = true;
                    at.check_peaks();
                    self.target = at.calibrate.clamp(min_t, max_t);
                }
                if at.heating {
                    if temp < at.peak {
                        at.peak = temp;
                        at.peak_time = read_time;
                    }
                    max_power
                } else {
                    if temp > at.peak {
                        at.peak = temp;
                        at.peak_time = read_time;
                    }
                    0.0
                }
            }),
        };
        let value = match value {
            Some(v) => v,
            None => self.pid(read_time, temp),
        };
        if self.cfg.smooth_time > 0.0 {
            self.smoothed += (temp - self.smoothed) * (time_diff / self.cfg.smooth_time).min(1.0);
        }
        self.set_pwm(read_time, value)
    }

    fn pid(&mut self, read_time: f64, temp: f64) -> f64 {
        let (kp, ki, kd) = (self.cfg.kp / PID_PARAM_BASE, self.cfg.ki / PID_PARAM_BASE, self.cfg.kd / PID_PARAM_BASE);
        let min_deriv_time = self.cfg.smooth_time;
        let time_diff = read_time - self.prev_time;
        let temp_diff = temp - self.prev_temp;
        let deriv = if time_diff >= min_deriv_time { temp_diff / time_diff } else { (self.prev_deriv * (min_deriv_time - time_diff) + temp_diff) / min_deriv_time };
        let err = self.target - temp;
        let integ_max = if ki > 0.0 { self.cfg.max_power / ki } else { 0.0 };
        let integ = (self.prev_integ + err * time_diff).clamp(0.0, integ_max);
        let co = kp * err + ki * integ - kd * deriv;
        let bounded = co.clamp(0.0, self.cfg.max_power);
        self.prev_temp = temp;
        self.prev_time = read_time;
        self.prev_deriv = deriv;
        if co == bounded {
            self.prev_integ = integ;
        }
        bounded
    }

    /// `Heater::set_pwm`.
    fn set_pwm(&mut self, read_time: f64, mut value: f64) -> PwmOut {
        if self.target <= 0.0 {
            value = 0.0;
        }
        if (read_time < self.next_pwm_time || self.last_pwm_value == 0.0) && (value - self.last_pwm_value).abs() < 0.05 {
            return PwmOut::None;
        }
        let pwm_time = (read_time + PWM_DELAY) as f32 as f64;
        self.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME;
        self.last_pwm_value = value;
        self.pwm = value;
        PwmOut::Send(pwm_time, value)
    }

    /// Smoothed temperature can extrude (3 °C tolerance), or the check is off.
    pub fn can_extrude(&self) -> bool {
        let ok = if self.min_extrude_temp >= 3.0 { self.smoothed >= self.min_extrude_temp - 3.0 } else { self.smoothed >= self.min_extrude_temp };
        (ok && self.temp.is_some()) || !self.can_extrude_switch
    }

    /// `check_busy` (PID): |target − smoothed| > 2.
    pub fn busy(&self) -> bool {
        match &self.control {
            Control::Pid => (self.target - self.smoothed).abs() > 2.0,
            Control::AutoTune(at) => at.busy(),
        }
    }

    /// `set_temp`: warn when out of range, set anyway.
    pub fn set_target(&mut self, t: f64) {
        if t != 0.0 && (t < self.cfg.min_temp || t > self.cfg.max_temp) {
            crate::gcode::console(&format!("Requested temperature ({t:.1}) out of range ({:.1}:{:.1})", self.cfg.min_temp, self.cfg.max_temp));
        }
        self.target = t;
    }

    /// `set_control(autotune)`: target becomes 0 (set it afterwards).
    pub fn start_autotune(&mut self, calibrate: f64) {
        self.control = Control::AutoTune(AutoTune::new(calibrate));
        self.target = 0.0;
    }

    /// `set_control(old)`: back to PID, target 0.
    pub fn stop_autotune(&mut self) -> Option<AutoTune> {
        let old = std::mem::replace(&mut self.control, Control::Pid);
        self.target = 0.0;
        match old {
            Control::AutoTune(at) => Some(at),
            Control::Pid => None,
        }
    }

    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.cfg.kp = kp;
        self.cfg.ki = ki;
        self.cfg.kd = kd;
    }

    /// verify_heater, once per second. Err = heater fault (message).
    pub fn verify(&mut self, now: f64) -> Result<(), String> {
        if now < self.next_verify {
            return Ok(());
        }
        self.next_verify = now + 1.0;
        if self.temp.is_none() {
            return Ok(());
        }
        let temp = self.smoothed;
        let target = self.target;
        let c = self.cfg;
        if temp < -20.0 {
            return Err(format!("Heater {} thermistor fault (reading {temp:.1} °C)", c.name));
        }
        if temp >= target - c.hysteresis || target <= 0.0 {
            self.approaching = false;
            self.starting = false;
            if temp <= target + c.hysteresis {
                self.error = 0.0;
            }
            self.last_target = target;
            return Ok(());
        }
        self.error += (target - c.hysteresis) - temp;
        if !self.approaching {
            if target != self.last_target {
                self.approaching = true;
                self.starting = true;
                self.goal_temp = temp + c.heating_gain;
                self.goal_time = now + c.check_gain_time;
            } else if self.error >= c.max_error {
                return Err(format!("Heater {} not heating at expected rate", c.name));
            }
        } else if temp >= self.goal_temp {
            self.starting = false;
            self.error = 0.0;
            self.goal_temp = temp + c.heating_gain;
            self.goal_time = now + c.check_gain_time;
        } else if now >= self.goal_time {
            self.approaching = false;
        } else if self.starting {
            self.goal_temp = self.goal_temp.min(temp + c.heating_gain);
        }
        self.last_target = target;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::Machine;

    fn extruder() -> Heater {
        Heater::new(Machine::stock().heaters[0])
    }

    #[test]
    fn dead_heater_is_detected() {
        let mut h = extruder();
        h.set_target(200.0);
        let mut fault = None;
        for i in 0..200 {
            let t = i as f64 * 0.5;
            h.update(t, 25.0);
            if let Err(e) = h.verify(t) {
                fault = Some((t, e));
                break;
            }
        }
        let (t, e) = fault.expect("fault");
        assert!(e.contains("not heating"), "{e}");
        assert!(t > 20.0 && t < 30.0, "after check_gain_time: {t}");
    }

    #[test]
    fn pwm_is_sent_on_change_and_refreshed() {
        let mut h = extruder();
        h.set_target(220.0);
        assert_eq!(h.update(1.0, 25.0), PwmOut::Send(1.5, 1.0));
        assert_eq!(h.update(1.5, 26.0), PwmOut::None, "unchanged and not due");
        assert!(matches!(h.update(5.3, 27.0), PwmOut::Send(..)), "re-sent after 3.75 s");
        h.set_target(0.0);
        assert!(matches!(h.update(5.8, 27.0), PwmOut::Send(_, v) if v == 0.0));
    }

    #[test]
    fn out_of_range_target_is_still_set() {
        let mut h = extruder();
        h.set_target(400.0);
        assert_eq!(h.target, 400.0);
    }

    #[test]
    fn reached_within_two_degrees() {
        let mut h = extruder();
        h.set_target(200.0);
        h.update(0.0, 198.5);
        assert!(!h.busy());
        h.smoothed = 195.0;
        assert!(h.busy());
    }

    #[test]
    fn extrusion_tolerance_and_switch() {
        let mut h = extruder();
        h.update(0.0, 167.5);
        assert!(h.can_extrude(), "170 − 3 °C");
        h.smoothed = 166.0;
        assert!(!h.can_extrude());
        h.can_extrude_switch = false;
        assert!(h.can_extrude());
    }

    #[test]
    fn autotune_finds_gains_on_a_simple_plant() {
        let mut h = extruder();
        h.start_autotune(200.0);
        h.set_target(200.0);
        // first-order plant with lag, 0.5 s readings
        let (mut temp, mut duty, mut t) = (25.0f64, 0.0f64, 0.0f64);
        let mut hist = std::collections::VecDeque::from(vec![25.0; 4]);
        while matches!(h.control, Control::AutoTune(ref a) if a.busy()) && t < 3000.0 {
            for _ in 0..50 {
                temp += 0.01 * (60.0 * duty - 0.12 * (temp - 25.0)) / 11.0;
            }
            hist.push_back(temp);
            let lagged = hist.pop_front().unwrap();
            if let PwmOut::Send(_, v) = h.update(t, lagged) {
                duty = v;
            }
            t += 0.5;
        }
        let at = h.stop_autotune().unwrap();
        let (kp, ki, kd) = at.calc_final_pid(1.0);
        assert!(kp > 1.0 && ki > 0.0 && kd > 0.0, "{kp} {ki} {kd}");
        assert_eq!(h.target, 0.0, "heater left off");
    }
}
