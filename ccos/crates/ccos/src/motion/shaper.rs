//! Input shaper pulse trains (chelper/kin_shaper.c): the shaped axis
//! position is Σ aᵢ · position(t + tᵢ), with the pulses shifted so that
//! constant-speed motion is unchanged.

use std::f64::consts::PI;

/// `INPUT_SHAPER_*` (and the `m_shapers` names of input_shaper.cpp).
pub const SHAPERS: [(&str, i32); 6] = [("zv", 0), ("zvd", 1), ("mzv", 2), ("ei", 3), ("2hump_ei", 4), ("3hump_ei", 5)];

const EI_SHAPER_VIB_TOL: f64 = 0.05;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pulses {
    pub n: usize,
    pub t: [f64; 5],
    pub a: [f64; 5],
}

fn zv_k(d: f64) -> f64 {
    if d == 0.0 {
        1.0
    } else {
        (-d * PI / (1.0 - d * d).sqrt()).exp()
    }
}

fn half_period(f: f64, d: f64) -> f64 {
    0.5 / (f * (1.0 - d * d).sqrt())
}

impl Pulses {
    fn set(t: &[f64], a: &[f64]) -> Pulses {
        let mut p = Pulses { n: t.len(), ..Default::default() };
        p.t[..t.len()].copy_from_slice(t);
        p.a[..a.len()].copy_from_slice(a);
        p
    }

    /// `init_shaper`: no pulses for an unknown type or a frequency ≤ 0.
    pub fn new(shaper_type: i32, freq: f64, damping: f64) -> Pulses {
        if !(0..6).contains(&shaper_type) || freq <= 0.0 {
            return Pulses::default();
        }
        let hp = half_period(freq, damping);
        let mut p = match shaper_type {
            0 => {
                let k = zv_k(damping);
                let inv_d = 1.0 / (1.0 + k);
                Pulses::set(&[-hp, 0.0], &[k * inv_d, inv_d])
            }
            1 => {
                let k = zv_k(damping);
                let k2 = k * k;
                let inv_d = 1.0 / (k2 + 2.0 * k + 1.0);
                Pulses::set(&[-2.0 * hp, -hp, 0.0], &[k2 * inv_d, 2.0 * k * inv_d, inv_d])
            }
            2 => {
                let k = (-0.75 * damping * PI / (1.0 - damping * damping).sqrt()).exp();
                let a1 = 1.0 - 1.0 / 2f64.sqrt();
                let a2 = (2f64.sqrt() - 1.0) * k;
                let a3 = a1 * k * k;
                let inv_d = 1.0 / (a1 + a2 + a3);
                Pulses::set(&[-1.5 * hp, -0.75 * hp, 0.0], &[a3 * inv_d, a2 * inv_d, a1 * inv_d])
            }
            3 => {
                let k = zv_k(damping);
                let a1 = 0.25 * (1.0 + EI_SHAPER_VIB_TOL);
                let a2 = 0.5 * (1.0 - EI_SHAPER_VIB_TOL) * k;
                let a3 = a1 * k * k;
                let inv_d = 1.0 / (a1 + a2 + a3);
                Pulses::set(&[-2.0 * hp, -hp, 0.0], &[a3 * inv_d, a2 * inv_d, a1 * inv_d])
            }
            4 => {
                let k = zv_k(damping);
                let v2 = EI_SHAPER_VIB_TOL * EI_SHAPER_VIB_TOL;
                let x = (v2 * ((1.0 - v2).sqrt() + 1.0)).powf(1.0 / 3.0);
                let a1 = (3.0 * x * x + 2.0 * x + 3.0 * v2) / (16.0 * x);
                let a2 = (0.5 - a1) * k;
                let a3 = a2 * k;
                let a4 = a1 * k * k * k;
                let inv_d = 1.0 / (a1 + a2 + a3 + a4);
                Pulses::set(&[-3.0 * hp, -2.0 * hp, -hp, 0.0], &[a4 * inv_d, a3 * inv_d, a2 * inv_d, a1 * inv_d])
            }
            _ => {
                let k = zv_k(damping);
                let k2 = k * k;
                let a1 = 0.0625 * (1.0 + 3.0 * EI_SHAPER_VIB_TOL + 2.0 * (2.0 * (EI_SHAPER_VIB_TOL + 1.0) * EI_SHAPER_VIB_TOL).sqrt());
                let a2 = 0.25 * (1.0 - EI_SHAPER_VIB_TOL) * k;
                let a3 = (0.5 * (1.0 + EI_SHAPER_VIB_TOL) - 2.0 * a1) * k2;
                let a4 = a2 * k2;
                let a5 = a1 * k2 * k2;
                let inv_d = 1.0 / (a1 + a2 + a3 + a4 + a5);
                Pulses::set(&[-4.0 * hp, -3.0 * hp, -2.0 * hp, -hp, 0.0], &[a5 * inv_d, a4 * inv_d, a3 * inv_d, a2 * inv_d, a1 * inv_d])
            }
        };
        // shift_pulses
        let ts: f64 = (0..p.n).map(|i| p.a[i] * p.t[i]).sum();
        for i in 0..p.n {
            p.t[i] -= ts;
        }
        p
    }

    /// `input_shaper_get_step_generation_window`.
    pub fn window(&self) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        (-self.t[0]).max(self.t[self.n - 1])
    }
}

/// Shaper type number from its name (`parse_shaper`), None if unknown.
pub fn parse(name: &str) -> Option<i32> {
    SHAPERS.iter().find(|(n, _)| *n == name).map(|(_, v)| *v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pulses_are_normalised_and_centred() {
        for ty in 0..6 {
            let p = Pulses::new(ty, 50.0, 0.1);
            let sa: f64 = (0..p.n).map(|i| p.a[i]).sum();
            let sat: f64 = (0..p.n).map(|i| p.a[i] * p.t[i]).sum();
            assert!((sa - 1.0).abs() < 1e-12 && sat.abs() < 1e-12, "type {ty}");
        }
        assert_eq!(Pulses::new(2, 0.0, 0.1).n, 0);
        assert_eq!(Pulses::new(9, 50.0, 0.1).n, 0);
    }

    #[test]
    fn stock_windows() {
        // [input_shaper] of the stock printer.cfg
        let x = Pulses::new(2, 54.8, 0.06);
        assert_eq!(x.n, 3);
        // zv at 46.4 Hz (input_shaper.cpp computes its window with damping_ratio_x)
        let y = Pulses::new(0, 46.4, 0.06);
        assert!(x.window() > 0.005 && x.window() < 0.01, "{}", x.window());
        assert!(y.window() > 0.004 && y.window() < 0.008, "{}", y.window());
        // zv: two pulses half a period apart
        let hp = 0.5 / (46.4 * (1.0f64 - 0.06 * 0.06).sqrt());
        assert!((y.t[1] - y.t[0] - hp).abs() < 1e-12);
    }
}
