//! Heater control and protection, ported from Klipper's `heaters.py`
//! (`ControlPID`) and `verify_heater.py`, with the stock printer.cfg values.
//!
//! Protection layers, all active in ccos:
//! 1. MCU side: soft-PWM output with `max_duration` 3 s — if the host stops
//!    sending updates the MCU shuts down and the heater turns off.
//! 2. MCU side: ADC range check (`min_temp`..`max_temp`) — an open or shorted
//!    thermistor shuts the MCU down.
//! 3. Host side: `verify_heater` — the heater must keep gaining temperature
//!    while below target, or ccos shuts everything down.

/// Klipper scales PID gains by 1/255 (`PID_PARAM_BASE`).
const PID_PARAM_BASE: f64 = 255.0;
const MIN_DERIV_TIME: f64 = 2.0;
/// MCU `max_duration` for heater outputs (Klipper `MAX_HEAT_TIME`).
pub const MAX_HEAT_TIME: f64 = 3.0;

#[derive(Debug, Clone, Copy)]
pub struct HeaterConfig {
    pub name: &'static str,
    pub beta: f64,
    pub min_temp: f64,
    pub max_temp: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub pwm_cycle_time: f64,
    pub max_error: f64,
    pub check_gain_time: f64,
    pub hysteresis: f64,
    pub heating_gain: f64,
}

/// printer.cfg `[extruder]` + `[verify_heater extruder]` (min_temp raised
/// from the stock -200 so a broken thermistor is caught).
pub const EXTRUDER: HeaterConfig = HeaterConfig {
    name: "extruder",
    beta: 4300.0,
    min_temp: 0.0,
    max_temp: 335.0,
    kp: 24.860427,
    ki: 2.702253,
    kd: 57.178292,
    pwm_cycle_time: 0.1,
    max_error: 120.0,
    check_gain_time: 20.0,
    hysteresis: 5.0,
    heating_gain: 2.0,
};

/// printer.cfg `[heater_bed]` + `[verify_heater heater_bed]`.
pub const BED: HeaterConfig = HeaterConfig {
    name: "heater_bed",
    beta: 3950.0,
    min_temp: 0.0,
    max_temp: 125.0,
    kp: 46.504839,
    ki: 1.453276,
    kd: 372.038710,
    pwm_cycle_time: 0.016,
    max_error: 120.0,
    check_gain_time: 120.0,
    hysteresis: 5.0,
    heating_gain: 2.0,
};

#[derive(Debug, Clone)]
pub struct Heater {
    pub cfg: HeaterConfig,
    pub target: f64,
    pub temp: Option<f64>,
    pub smoothed: f64,
    pub pwm: f64,
    last_time: f64,
    prev_temp: f64,
    prev_deriv: f64,
    prev_integ: f64,
    // verify_heater
    error: f64,
    approaching: bool,
    starting: bool,
    goal_temp: f64,
    goal_time: f64,
    last_target: f64,
    pub next_verify: f64,
    /// Last value sent to the MCU and when (print time).
    pub sent: Option<(f64, f64)>,
    /// Target that takes effect at a print time (queued with the motion).
    pub pending: Option<(f64, f64)>,
}

impl Heater {
    pub fn new(cfg: HeaterConfig) -> Self {
        Self {
            cfg,
            target: 0.0,
            temp: None,
            smoothed: 0.0,
            pwm: 0.0,
            last_time: 0.0,
            prev_temp: 0.0,
            prev_deriv: 0.0,
            prev_integ: 0.0,
            error: 0.0,
            approaching: false,
            starting: false,
            goal_temp: 0.0,
            goal_time: 0.0,
            last_target: 0.0,
            next_verify: 0.0,
            sent: None,
            pending: None,
        }
    }

    /// New reading at time `t`; returns the PWM to apply.
    pub fn update(&mut self, t: f64, temp: f64) -> f64 {
        let (kp, ki, kd) = (self.cfg.kp / PID_PARAM_BASE, self.cfg.ki / PID_PARAM_BASE, self.cfg.kd / PID_PARAM_BASE);
        if self.temp.is_none() {
            self.prev_temp = temp;
            self.last_time = t;
            self.smoothed = temp;
        }
        self.temp = Some(temp);
        let dt = (t - self.last_time).max(0.0);
        self.smoothed += (temp - self.smoothed) * (dt / 1.0).min(1.0);
        let temp_diff = temp - self.prev_temp;
        let deriv = if dt >= MIN_DERIV_TIME { temp_diff / dt } else { (self.prev_deriv * (MIN_DERIV_TIME - dt) + temp_diff) / MIN_DERIV_TIME };
        let err = self.target - temp;
        let integ = (self.prev_integ + err * dt).clamp(0.0, 1.0 / ki);
        let co = kp * err + ki * integ - kd * deriv;
        let bounded = co.clamp(0.0, 1.0);
        self.prev_temp = temp;
        self.last_time = t;
        self.prev_deriv = deriv;
        if co == bounded {
            self.prev_integ = integ;
        }
        self.pwm = if self.target <= 0.0 { 0.0 } else { bounded };
        self.pwm
    }

    /// Whether the PWM must be (re)sent: changed, or the MCU's
    /// `max_duration` would otherwise run out.
    pub fn needs_send(&self, t: f64) -> bool {
        match self.sent {
            None => true,
            Some((v, at)) => (v - self.pwm).abs() > 1e-4 || (self.pwm > 0.0 && t - at > MAX_HEAT_TIME * 0.5),
        }
    }

    /// Reached the target (M109/M190): within 1 °C, like Klipper's check_busy.
    pub fn at_target(&self) -> bool {
        self.target <= 0.0 || (self.smoothed - self.target).abs() <= 1.0
    }

    pub fn set_target(&mut self, t: f64) -> Result<(), String> {
        if t != 0.0 && !(self.cfg.min_temp..=self.cfg.max_temp).contains(&t) {
            return Err(format!("Requested temperature ({t:.1}) out of range ({:.1}:{:.1})", self.cfg.min_temp, self.cfg.max_temp));
        }
        self.target = t;
        Ok(())
    }

    /// verify_heater check, once per second. Err = heater fault.
    pub fn verify(&mut self, now: f64) -> Result<(), String> {
        if now < self.next_verify {
            return Ok(());
        }
        self.next_verify = now + 1.0;
        let Some(temp) = self.temp else { return Ok(()) };
        let target = self.target;
        let c = self.cfg;
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

    /// A heater that never warms up must be caught.
    #[test]
    fn dead_heater_is_detected() {
        let mut h = Heater::new(EXTRUDER);
        h.set_target(200.0).unwrap();
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
    fn pid_drives_full_power_when_cold_and_off_at_zero_target() {
        let mut h = Heater::new(EXTRUDER);
        h.set_target(220.0).unwrap();
        assert_eq!(h.update(0.0, 25.0), 1.0);
        h.set_target(0.0).unwrap();
        assert_eq!(h.update(0.3, 25.0), 0.0);
        assert!(h.set_target(400.0).is_err());
    }
}
