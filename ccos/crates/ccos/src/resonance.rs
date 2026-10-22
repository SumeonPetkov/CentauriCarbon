//! `[resonance_tester]` (resonance_tester.cpp) settings and the input
//! shaper fit (shaper_calibrate.cpp). Elegoo replaced Klipper's frequency
//! sweep with a pulse test: half-period moves out and back, at an
//! acceleration of `accel_per_hz` × the frequency, from `min_freq` to
//! `max_freq` at `hz_per_sec`.
//!
//! The math is the stock port of Klipper's `shaper_calibrate.py`, with its
//! own pieces kept:
//! * the PSD uses a direct DFT whose twiddle table is built from π ≈
//!   3.141592653 by adding `-2π/n` up; the test frequencies' ω uses the
//!   same π;
//! * `normalize_to_frequencies` divides copies of the spectra that are
//!   never used again, so the fit runs on the raw sum of the three axes'
//!   PSDs (low frequencies are not cut and not divided by the frequency);
//! * only `mzv` is tried (from 23 Hz), at a target acceleration of
//!   10000 mm/s²;
//! * the smoothing of a shaper counts all its impulses and adds the 90°
//!   offset only for impulses at or after the centre, ×√2.

use crate::config::Config;

/// π as `my_math.h` defines it.
pub const PI_APPROX: f64 = 3.141592653;

const MAX_FREQ: f64 = 200.;
const WINDOW_T_SEC: f64 = 0.5;
const MAX_SHAPER_FREQ: f64 = 150.;
const SHAPER_DAMPING_RATIO: f64 = 0.1;
const TEST_DAMPING_RATIOS: [f64; 3] = [0.075, 0.1, 0.15];
const TARGET_SMOOTHING: f64 = 0.12;
const TARGET_ACCEL: f64 = 10000.;
const SHAPER_VIBRATION_REDUCTION: f64 = 20.;

/// `INPUT_SHAPERS`: name, impulses, lowest frequency tried.
const INPUT_SHAPERS: [(&str, fn(f64, f64) -> Shaper, f64); 1] = [("mzv", get_mzv_shaper, 23.)];

/// `VibrationPulseTest` + `ResonanceTester` settings.
#[derive(Debug, Clone)]
pub struct ResonanceCfg {
    pub min_freq: f64,
    pub max_freq: f64,
    pub accel_per_hz: f64,
    pub hz_per_sec: f64,
    pub probe_points: Vec<Vec<f64>>,
    pub move_speed: f64,
    /// (axes, chip section): `[("xy", "adxl345 X")]` on the printer.
    pub chips: Vec<(String, String)>,
    pub max_smoothing: f64,
    /// What the constructor printed.
    pub warnings: Vec<String>,
}

impl ResonanceCfg {
    /// The object is loaded with or without a `[resonance_tester]`
    /// section (`load_object("resonance_tester")`).
    pub fn from_config(c: &Config) -> Self {
        let s = "resonance_tester";
        let mut warnings = Vec::new();
        let mut above = |key: &str, def: f64| {
            let v = c.get_double(s, key, def);
            if v <= 0.0 {
                warnings.push(format!("error on {s}: {key} must be above 0"));
            }
            v
        };
        let accel_per_hz = above("accel_per_hz", 75.);
        let move_speed = above("move_speed", 50.);
        let min_freq = c.get_double_min(s, "min_freq", 5., 1.);
        // parse_probe_points: lines, then commas; `std::stod` on each
        let mut probe_points = Vec::new();
        for line in c.get_string(s, "probe_points", "").split('\n') {
            let mut point = Vec::new();
            for coord in line.trim().split(',') {
                match crate::gcode::parse::stod(coord.trim()) {
                    Ok(v) => point.push(v),
                    // stod throws: stock would not start
                    Err(_) => warnings.push(format!("error on {s}: probe_points: invalid number '{}'", coord.trim())),
                }
            }
            probe_points.push(point);
        }
        let chip = |k: &str| c.get_string(s, k, "").trim().to_string();
        let mut chips = if chip("accel_chip_x").is_empty() {
            vec![("xy".to_string(), chip("accel_chip"))]
        } else {
            vec![("x".to_string(), chip("accel_chip_x")), ("y".to_string(), chip("accel_chip_y"))]
        };
        if chips.len() == 2 && chips[0].1 == chips[1].1 {
            chips = vec![("xy".to_string(), chips[0].1.clone())];
        }
        Self {
            min_freq,
            max_freq: c.get_double_range(s, "max_freq", 10000. / 75., min_freq, 200.),
            accel_per_hz,
            hz_per_sec: c.get_double_range(s, "hz_per_sec", 1., 0.1, 2.),
            probe_points,
            move_speed,
            chips,
            max_smoothing: c.get_double_min(s, "max_smoothing", 0., 0.),
            warnings,
        }
    }
}

/// `VibrationPulseTest::run_test_time`: the test's length (s).
pub fn run_test_time(freq_start: f64, freq_end: f64, hz_per_sec: f64) -> f64 {
    let (mut time, mut freq) = (0.0, freq_start);
    while freq <= freq_end + 0.000001 {
        let t_seg = 0.25 / freq;
        freq += 2. * t_seg * hz_per_sec;
        time += 2. * t_seg;
    }
    time
}

/// Impulse amplitudes and times.
pub type Shaper = (Vec<f64>, Vec<f64>);

pub fn get_mzv_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let df = (1. - damping_ratio.powi(2)).sqrt();
    let k = (-0.75 * damping_ratio * std::f64::consts::PI / df).exp();
    let t_d = 1. / (shaper_freq * df);
    let a1 = 1. - 1. / 2f64.sqrt();
    let a2 = (2f64.sqrt() - 1.) * k;
    let a3 = a1 * k * k;
    (vec![a1, a2, a3], vec![0., 0.375 * t_d, 0.75 * t_d])
}

/// `get_shaper_smoothing(shaper, accel, scv = 5)`.
pub fn get_shaper_smoothing(shaper: &Shaper, accel: f64) -> f64 {
    let scv = 5.;
    let half_accel = accel * 0.5;
    let (a, t) = shaper;
    let inv_d = 1. / a.iter().sum::<f64>();
    let ts = a.iter().zip(t).map(|(a, t)| a * t).sum::<f64>() * inv_d;
    let (mut offset_90, mut offset_180) = (0., 0.);
    for (a, t) in a.iter().zip(t) {
        if *t >= ts {
            offset_90 += a * (scv + half_accel * (t - ts)) * (t - ts);
        }
        offset_180 += a * half_accel * (t - ts).powi(2);
    }
    offset_90 *= inv_d * 2f64.sqrt();
    offset_180 *= inv_d;
    offset_90.max(offset_180)
}

/// `std::cyl_bessel_i(0, x)`: the power series (x ≤ 6 here).
fn bessel_i0(x: f64) -> f64 {
    let q = x * x / 4.;
    let (mut term, mut sum, mut k) = (1.0, 1.0, 1.0);
    while term > sum * 1e-17 {
        term *= q / (k * k);
        sum += term;
        k += 1.;
    }
    sum
}

/// `nc::kaiser(m, beta)` (over `nc::linspace`).
fn kaiser(m: usize, beta: f64) -> Vec<f64> {
    let md = m as f64;
    let half = (md - 1.) / 2.;
    let sq = (md - 1.).powi(2);
    let i0_beta = bessel_i0(beta);
    let step = (2. * half) / (md - 1.);
    (0..m)
        .map(|i| {
            let n = if i == m - 1 { half } else { -half + i as f64 * step };
            bessel_i0(beta * (1. - (4. * n * n) / sq).sqrt()) / i0_beta
        })
        .collect()
}

/// `CDft1`: the twiddle table, built the stock way.
struct Dft {
    cos: Vec<f64>,
    sin: Vec<f64>,
}

impl Dft {
    fn new(n: usize) -> Self {
        let fixed = (-2. * PI_APPROX) / n as f64;
        let mut power = 0.0f64;
        let (mut cos, mut sin) = (Vec::with_capacity(n), Vec::with_capacity(n));
        for _ in 0..n {
            cos.push(power.cos());
            sin.push(power.sin());
            power += fixed;
        }
        Self { cos, sin }
    }

    /// `dft_mod_fast`: |X(u)|² · scale / fs for u = 0..=n/2.
    fn power(&self, x: &[f64], scale: f64, fs: f64, out: &mut Vec<f64>) {
        let n = x.len();
        out.clear();
        for u in 0..n / 2 + 1 {
            let (mut re, mut im) = (0.0, 0.0);
            let mut p = 0usize;
            for &v in x {
                re += v * self.cos[p];
                im += v * self.sin[p];
                p = (p + u) & (n - 1);
            }
            out.push((re * re + im * im) * scale / fs);
        }
    }
}

/// `ShaperCalibrate::psd`: Welch's method with a Kaiser window (β 6),
/// half-overlapping windows of `nfft` samples.
pub fn psd(x: &[f64], fs: f64, nfft: usize) -> Vec<f64> {
    let window = kaiser(nfft, 6.);
    let scale = 2.0 / window.iter().map(|w| w * w).sum::<f64>();
    let overlap = nfft / 2;
    let step = nfft - overlap;
    let n_windows = (x.len().saturating_sub(overlap)) / step;
    let dft = Dft::new(nfft);
    let mut sum = vec![0.0; nfft / 2 + 1];
    let (mut seg, mut out) = (vec![0.0; nfft], Vec::new());
    for i in 0..n_windows {
        let col = &x[i * step..i * step + nfft];
        let mean = col.iter().sum::<f64>() / nfft as f64;
        for j in 0..nfft {
            seg[j] = (col[j] - mean) * window[j];
        }
        dft.power(&seg, scale, fs, &mut out);
        out[0] /= 2.;
        let last = out.len() - 1;
        if last > 0 {
            out[last] /= 2.;
        }
        for (s, o) in sum.iter_mut().zip(&out) {
            *s += o;
        }
    }
    sum.iter().map(|s| s / n_windows as f64).collect()
}

/// `CalibrationData`.
#[derive(Debug, Clone)]
pub struct CalibrationData {
    pub freq_bins: Vec<f64>,
    pub psd_sum: Vec<f64>,
    pub psd_x: Vec<f64>,
    pub psd_y: Vec<f64>,
    pub psd_z: Vec<f64>,
}

/// `bit_length`.
fn bit_length(mut n: i64) -> u32 {
    let mut r = 0;
    while n != 0 {
        n /= 2;
        r += 1;
    }
    r
}

/// `calc_freq_response`: None when there are not more samples than one
/// window.
pub fn calc_freq_response(x: &[f64], y: &[f64], z: &[f64], samp_time: f64) -> Option<CalibrationData> {
    let n = x.len() as f64;
    let sampling_freq = n / samp_time;
    let m = 1usize << bit_length((sampling_freq * WINDOW_T_SEC - 1.) as i64);
    if n <= m as f64 {
        return None;
    }
    let (psd_x, psd_y, psd_z) = (psd(x, sampling_freq, m), psd(y, sampling_freq, m), psd(z, sampling_freq, m));
    // rfftfreq(m, 1 / fs)
    let d = 1. / sampling_freq;
    let freq_bins = (0..=m / 2).map(|i| i as f64 / (m as f64 * d)).collect();
    let psd_sum = (0..psd_x.len()).map(|i| psd_x[i] + psd_y[i] + psd_z[i]).collect();
    Some(CalibrationData { freq_bins, psd_sum, psd_x, psd_y, psd_z })
}

/// `estimate_shaper`: the shaper's vibration ratio at each frequency.
fn estimate_shaper(shaper: &Shaper, test_damping_ratio: f64, test_freqs: &[f64]) -> Vec<f64> {
    let (a, t) = shaper;
    let inv_d = 1. / a.iter().sum::<f64>();
    let n = t.len();
    test_freqs
        .iter()
        .map(|f| {
            let omega = 2. * PI_APPROX * f;
            let damping = test_damping_ratio * omega;
            let omega_d = omega * (1. - test_damping_ratio.powi(2)).sqrt();
            let (mut s, mut c) = (0.0, 0.0);
            for j in 0..n {
                // the times reversed, where Klipper has T[-1] - T
                let w = (-damping * t[n - 1 - j]).exp() * a[j];
                s += w * (omega_d * t[j]).sin();
                c += w * (omega_d * t[j]).cos();
            }
            (s * s + c * c).sqrt() * inv_d
        })
        .collect()
}

fn estimate_remaining_vibrations(shaper: &Shaper, dr: f64, freq_bins: &[f64], psd: &[f64]) -> (f64, Vec<f64>) {
    let vals = estimate_shaper(shaper, dr, freq_bins);
    let threshold = psd.iter().cloned().fold(f64::NEG_INFINITY, f64::max) / SHAPER_VIBRATION_REDUCTION;
    let remaining: f64 = vals.iter().zip(psd).map(|(v, p)| (v * p - threshold).max(0.)).sum();
    let all: f64 = psd.iter().map(|p| (p - threshold).max(0.)).sum();
    (remaining / all, vals)
}

/// `shaper_t`.
#[derive(Debug, Clone, Default)]
pub struct ShaperFit {
    pub name: String,
    pub freq: f64,
    pub vals: Vec<f64>,
    pub vibrs: f64,
    pub smoothing: f64,
    pub score: f64,
    pub max_accel: f64,
}

/// `bisect` for the largest acceleration with smoothing ≤ 0.12.
fn find_shaper_max_accel(shaper: &Shaper) -> f64 {
    let ok = |accel: f64| get_shaper_smoothing(shaper, accel) <= TARGET_SMOOTHING;
    let (mut left, mut right) = (1.0f64, 1.0f64);
    while !ok(left) {
        right = left;
        left *= 0.5;
    }
    if right == left {
        while ok(right) {
            right *= 2.;
        }
    }
    while right - left > 1e-8 {
        let middle = (left + right) * 0.5;
        if ok(middle) {
            left = middle;
        } else {
            right = middle;
        }
    }
    left
}

fn fit_shaper(name: &str, func: fn(f64, f64) -> Shaper, min_freq: f64, data: &CalibrationData, max_smoothing: f64) -> ShaperFit {
    let mut test_freqs = Vec::new();
    let mut f = min_freq;
    while f < MAX_SHAPER_FREQ {
        test_freqs.push(f);
        f += 0.2;
    }
    let (mut freq_bins, mut psd) = (Vec::new(), Vec::new());
    for (i, &b) in data.freq_bins.iter().enumerate() {
        if b <= MAX_FREQ {
            freq_bins.push(b);
            psd.push(data.psd_sum[i]);
        }
    }
    let mut best: Option<ShaperFit> = None;
    let mut results: Vec<ShaperFit> = Vec::new();
    for &test_freq in test_freqs.iter().rev() {
        let shaper = func(test_freq, SHAPER_DAMPING_RATIO);
        let smoothing = get_shaper_smoothing(&shaper, TARGET_ACCEL);
        if max_smoothing != 0.0 && smoothing > max_smoothing {
            if let Some(b) = best {
                return b;
            }
        }
        let mut vibrs = 0.0f64;
        let mut vals = vec![0.0f64; freq_bins.len()];
        for dr in TEST_DAMPING_RATIOS {
            let (v, dvals) = estimate_remaining_vibrations(&shaper, dr, &freq_bins, &psd);
            for (a, b) in vals.iter_mut().zip(&dvals) {
                *a = a.max(*b);
            }
            if v > vibrs {
                vibrs = v;
            }
        }
        let max_accel = find_shaper_max_accel(&shaper);
        let score = smoothing * (vibrs.powf(1.5) + vibrs * 0.2 + 0.01);
        let r = ShaperFit { name: name.into(), freq: test_freq, vals, vibrs, smoothing, score, max_accel };
        if best.as_ref().is_none_or(|b| b.vibrs > r.vibrs) {
            best = Some(r.clone());
        }
        results.push(r);
    }
    let best = best.unwrap_or_default();
    let mut selected = best.clone();
    for r in results.iter().rev() {
        if r.vibrs < best.vibrs * 1.1 && r.score < selected.score {
            selected = r.clone();
        }
    }
    selected
}

/// `find_best_shaper`: the best fit and every shaper's.
pub fn find_best_shaper(data: &CalibrationData, max_smoothing: f64) -> (ShaperFit, Vec<ShaperFit>) {
    let mut all = Vec::new();
    let mut best: Option<ShaperFit> = None;
    for (name, func, min_freq) in INPUT_SHAPERS {
        let s = fit_shaper(name, func, min_freq, data, max_smoothing);
        all.push(s.clone());
        let better = match &best {
            None => true,
            Some(b) => s.score * 1.2 < b.score || (s.score * 1.05 < b.score && s.smoothing * 1.1 < b.smoothing),
        };
        if better {
            best = Some(s);
        }
    }
    (best.unwrap_or_default(), all)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kaiser_and_bessel() {
        assert!((bessel_i0(6.) - 67.23440697647798).abs() < 1e-9);
        let w = kaiser(1024, 6.);
        assert!((w[0] - 0.014873337104763204).abs() < 1e-12);
        assert!((w[511] - w[512]).abs() < 1e-12 && w[511] > 0.9999);
    }

    #[test]
    fn psd_finds_a_tone_and_the_fit_follows() {
        // a decaying 50 Hz ring every 0.25 s, sampled at 1600 Hz
        let fs = 1600.;
        let n = 16000;
        let x: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let tr = t % 0.25;
                1000. * (-tr * 20.).exp() * (2. * std::f64::consts::PI * 50. * tr).sin()
            })
            .collect();
        let zero = vec![0.0; n];
        let d = calc_freq_response(&x, &zero, &zero, n as f64 / fs).unwrap();
        assert_eq!(d.freq_bins.len(), 513);
        let peak = (0..d.psd_x.len()).max_by(|a, b| d.psd_x[*a].total_cmp(&d.psd_x[*b])).unwrap();
        assert!((d.freq_bins[peak] - 50.).abs() < 2., "{}", d.freq_bins[peak]);
        let (best, all) = find_best_shaper(&d, 0.);
        assert_eq!((best.name.as_str(), all.len()), ("mzv", 1));
        assert!(best.freq > 35. && best.freq < 70., "{}", best.freq);
        assert!(best.max_accel > 1000.);
    }

    #[test]
    fn test_time() {
        // 1 Hz/s from 5 to 75 Hz: about 70 s
        let t = run_test_time(5., 75., 1.);
        assert!((t - 70.).abs() < 0.5, "{t}");
    }
}
