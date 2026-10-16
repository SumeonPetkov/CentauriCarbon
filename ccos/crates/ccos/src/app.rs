//! Application state, the UI [`Model`] implementation, and the worker thread
//! that owns the MCU links (so slow I/O never stalls the screen).
//!
//! What ccos drives today is deliberately limited to **read-only sensors, the
//! chamber light and the hotend cooling fan** (on above 50 °C, like the stock
//! `heater_fan`): thermistor ADCs on the DSP and the toolhead, MCU
//! probing/resets. Heaters and motors are not configured until a printer core
//! with full safety checks exists (see docs/re/08-rust-host-roadmap.md).

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cc_ui::{Model, TextBuf};

use crate::ui::{Flag, Value};
use crate::Target;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LinkState {
    #[default]
    Unknown,
    Ok,
    Failed,
}

#[derive(Debug, Clone, Default)]
pub struct LinkStatus {
    pub state: LinkState,
    pub detail: String,
}

pub struct State {
    pub nozzle: Option<f64>,
    pub nozzle_target: f64,
    pub bed: Option<f64>,
    pub bed_target: f64,
    pub chamber: Option<f64>,
    pub light: bool,
    pub links: [LinkStatus; 3],
    pub status: String,
    pub log: VecDeque<String>,
    pub backlight: u8,
    pub rss_kib: u64,
    pub cpu_pct: f32,
    pub started: Instant,
}

impl Default for State {
    fn default() -> Self {
        Self {
            nozzle: None,
            nozzle_target: 0.0,
            bed: None,
            bed_target: 0.0,
            chamber: None,
            light: false,
            links: Default::default(),
            status: "Starting…".into(),
            log: VecDeque::with_capacity(8),
            backlight: 191,
            rss_kib: 0,
            cpu_pct: 0.0,
            started: Instant::now(),
        }
    }
}

impl State {
    pub fn log(&mut self, s: impl Into<String>) {
        let s = s.into();
        eprintln!("ccos: {s}");
        if self.log.len() == 6 {
            self.log.pop_front();
        }
        self.log.push_back(s);
    }

    pub fn set_status(&mut self, s: impl Into<String>) {
        self.status = s.into();
        let st = self.status.clone();
        self.log(st);
    }
}

fn temp(out: &mut TextBuf, t: Option<f64>) {
    match t {
        Some(t) => {
            let _ = write!(out, "{t:.0}°");
        }
        None => out.push_str("--"),
    }
}

fn target(out: &mut TextBuf, t: f64) {
    if t > 0.0 {
        let _ = write!(out, "→ {t:.0}°");
    } else {
        out.push_str("off");
    }
}

impl Model<Value, Flag> for State {
    fn text(&self, v: Value, out: &mut TextBuf) {
        match v {
            Value::NozzleTemp => temp(out, self.nozzle),
            Value::NozzleTarget => target(out, self.nozzle_target),
            Value::BedTemp => temp(out, self.bed),
            Value::BedTarget => target(out, self.bed_target),
            Value::ChamberTemp => temp(out, self.chamber),
            Value::Status => out.push_str(&self.status),
            Value::Links => {
                for (i, t) in Target::ALL.iter().enumerate() {
                    let mark = match self.links[i].state {
                        LinkState::Ok => "✓",
                        LinkState::Failed => "✗",
                        LinkState::Unknown => "…",
                    };
                    let _ = write!(out, "{}{} {}", if i > 0 { "   " } else { "" }, t.title(), mark);
                }
            }
            Value::Log1 | Value::Log2 | Value::Log3 | Value::Log4 | Value::Log5 | Value::Log6 => {
                let i = match v {
                    Value::Log1 => 0,
                    Value::Log2 => 1,
                    Value::Log3 => 2,
                    Value::Log4 => 3,
                    Value::Log5 => 4,
                    _ => 5,
                };
                if let Some(l) = self.log.get(i) {
                    out.push_str(l);
                }
            }
            Value::Backlight => {}
            Value::BacklightPct => {
                let _ = write!(out, "{}%", self.backlight as u32 * 100 / 255);
            }
            Value::Version => {
                let _ = write!(out, "ccos {}", env!("CARGO_PKG_VERSION"));
            }
            Value::Memory => {
                let (f, i) = crate::ui::ASSET_BYTES;
                let _ = write!(out, "RAM {} KiB resident · UI assets {} KiB", self.rss_kib, (f + i) / 1024);
            }
            Value::Cpu => {
                let _ = write!(out, "CPU {:.1}% of one core", self.cpu_pct);
            }
            Value::Uptime => {
                let s = self.started.elapsed().as_secs();
                let _ = write!(out, "Up {}h {:02}m {:02}s", s / 3600, s / 60 % 60, s % 60);
            }
        }
    }

    fn level(&self, v: Value) -> f32 {
        match v {
            Value::Backlight => self.backlight as f32 / 255.0,
            _ => 0.0,
        }
    }

    fn flag(&self, f: Flag) -> bool {
        match f {
            Flag::NozzleHeating => self.nozzle_target > 0.0,
            Flag::BedHeating => self.bed_target > 0.0,
            Flag::LightOn => self.light,
        }
    }
}

/// Work for the MCU thread.
#[derive(Debug, Clone, Copy)]
pub enum Job {
    Probe,
    Reset(Target),
    ToggleLight,
    Gcode(&'static str),
}

pub type Shared = Arc<Mutex<State>>;

/// Start the MCU thread. Dropping the returned `Sender` stops it; join the
/// handle so the DSP ring buffer is released before the process exits.
#[cfg(target_os = "linux")]
pub fn spawn_worker(state: Shared, restart_dsp: bool) -> (Sender<Job>, std::thread::JoinHandle<()>) {
    let (tx, rx) = std::sync::mpsc::channel();
    let h = std::thread::Builder::new()
        .name("mcu".into())
        .stack_size(256 * 1024)
        .spawn(move || worker::run(state, rx, restart_dsp))
        .expect("spawn worker");
    (tx, h)
}

#[cfg(target_os = "linux")]
mod worker {
    use super::*;
    use crate::mcu::{ntc_temp, Mcu};
    use cc_proto::clock::{ADC_MAX, DSP_CLOCK_FREQ, STM32_CLOCK_FREQ};
    use cc_proto::dict::pin_number;

    const SAMPLES: u32 = 8;

    /// One attached MCU with the sensors we configured on it.
    struct Node {
        mcu: Mcu,
        /// oid -> (sensor, beta)
        sensors: &'static [(Sensor, f64)],
        /// Digital output: chamber light (DSP) or hotend fan (toolhead).
        out_oid: Option<u8>,
        out_on: bool,
    }

    #[derive(Clone, Copy)]
    enum Sensor {
        Nozzle,
        Bed,
        Chamber,
    }

    fn lock(s: &Shared) -> std::sync::MutexGuard<'_, State> {
        s.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Configure ADCs (and one digital output) on a fresh MCU and start sampling.
    fn configure(m: &mut Mcu, t: Target, adcs: &[&str], out: Option<&str>) -> crate::mcu::Result<Option<u8>> {
        let cfg = m.request("get_config", "config")?;
        if cfg.int("is_config") == Some(1) {
            return Err(format!("{} is already configured by another host; reset it first", t.title()).into());
        }
        let ppb = t.pins_per_bank();
        let n = adcs.len() + out.is_some() as usize;
        m.send(&format!("allocate_oids count={n}"))?;
        for (oid, pin) in adcs.iter().enumerate() {
            m.send(&format!("config_analog_in oid={oid} pin={}", pin_number(pin, ppb).unwrap()))?;
        }
        let out_oid = out.map(|_| adcs.len() as u8);
        if let (Some(oid), Some(p)) = (out_oid, out) {
            m.send(&format!("config_digital_out oid={oid} pin={} value=0 default_value=0 max_duration=0", pin_number(p, ppb).unwrap()))?;
        }
        m.send("finalize_config crc=0")?;
        let freq = if t == Target::Dsp { DSP_CLOCK_FREQ } else { STM32_CLOCK_FREQ };
        let now = m.clock()?;
        for oid in 0..adcs.len() {
            // Same cadence as Klipper: 8 samples 1 ms apart, report every 300 ms.
            let start = now.wrapping_add((freq * 0.1) as u32 + oid as u32 * (freq * 0.01) as u32);
            m.send(&format!(
                "query_analog_in oid={oid} clock={start} sample_ticks={} sample_count={SAMPLES} rest_ticks={} min_value=0 max_value=65535 range_check_count=1",
                (freq * 0.001) as u32,
                (freq * 0.3) as u32
            ))?;
        }
        Ok(out_oid)
    }

    fn attach(state: &Shared, t: Target, restart_dsp: bool) -> Option<Node> {
        let idx = t as usize;
        let r = (|| -> crate::mcu::Result<Node> {
            if t != Target::Dsp {
                t.power_cycle()?;
            }
            let mut m = Mcu::new(t.label(), t.open(restart_dsp)?);
            m.sync()?;
            let hz = m.measure_clock()?;
            let (sensors, out): (&'static [(Sensor, f64)], Option<u8>) = match t {
                // printer.cfg: [heater_bed] sensor_pin PB14, [temperature_sensor box] PB13, light PG15
                Target::Dsp => (&[(Sensor::Bed, 3950.0), (Sensor::Chamber, 3950.0)], configure(&mut m, t, &["PB14", "PB13"], Some("PG15"))?),
                // printer.cfg: [extruder] sensor_pin stm32:PA3 (NTC 100K beta 4300),
                // [heater_fan hotend_cooling_fan] pin stm32:PC8, heater_temp 50
                Target::Toolhead => (&[(Sensor::Nozzle, 4300.0)], configure(&mut m, t, &["PA3"], Some("PC8"))?),
                // Load cells: link check only for now.
                Target::StrainGauge => (&[], None),
            };
            lock(state).links[idx] = LinkStatus { state: LinkState::Ok, detail: format!("{:.1} MHz", hz / 1e6) };
            Ok(Node { mcu: m, sensors, out_oid: out, out_on: false })
        })();
        match r {
            Ok(n) => {
                lock(state).log(format!("{}: connected", t.title()));
                Some(n)
            }
            Err(e) => {
                let mut s = lock(state);
                s.links[idx] = LinkStatus { state: LinkState::Failed, detail: e.to_string() };
                s.log(format!("{}: {e}", t.title()));
                None
            }
        }
    }

    pub fn run(state: Shared, rx: Receiver<Job>, restart_dsp: bool) {
        if crate::sys::stock_app_running() {
            lock(&state).set_status("Stock app is running — stop it to use the MCUs");
            for job in rx {
                lock(&state).log(format!("ignored {job:?}: stock app owns the MCUs"));
            }
            return;
        }
        lock(&state).set_status("Connecting to MCUs…");
        let mut nodes: [Option<Node>; 3] = [None, None, None];
        for t in Target::ALL {
            nodes[t as usize] = attach(&state, t, restart_dsp);
        }
        let ok = nodes.iter().filter(|n| n.is_some()).count();
        lock(&state).set_status(format!("{ok} of 3 MCUs connected · sensors live"));

        loop {
            // Jobs from the UI
            loop {
                match rx.try_recv() {
                    Ok(job) => handle(&state, &mut nodes, job, restart_dsp),
                    Err(std::sync::mpsc::TryRecvError::Empty) => break,
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => return,
                }
            }
            // Sensor reports
            let mut idle = true;
            for (i, slot) in nodes.iter_mut().enumerate() {
                let Some(node) = slot else { continue };
                let mut seqs = [0u8; 32];
                if let Err(e) = node.mcu.pump(Duration::from_millis(30), &mut seqs) {
                    // Link gone (USB unplugged, DSP stopped...): drop it so we
                    // neither spin nor report stale values. "Reset" re-attaches.
                    let mut s = lock(&state);
                    s.links[i] = LinkStatus { state: LinkState::Failed, detail: e.to_string() };
                    s.log(format!("{}: link lost: {e}", Target::ALL[i].title()));
                    match Target::ALL[i] {
                        Target::Dsp => (s.bed, s.chamber) = (None, None),
                        Target::Toolhead => s.nozzle = None,
                        Target::StrainGauge => {}
                    }
                    drop(s);
                    *slot = None;
                    continue;
                }
                while let Some(i) = node.mcu.inbox.iter().position(|m| m.name() == "analog_in_state") {
                    idle = false;
                    let m = node.mcu.inbox.remove(i).unwrap();
                    let oid = m.int("oid").unwrap_or(-1) as usize;
                    let frac = m.int("value").unwrap_or(0) as f64 / (SAMPLES as f64 * ADC_MAX as f64);
                    if let Some((sensor, beta)) = node.sensors.get(oid) {
                        let t = ntc_temp(frac, *beta);
                        {
                            let mut s = lock(&state);
                            match sensor {
                                Sensor::Nozzle => s.nozzle = t,
                                Sensor::Bed => s.bed = t,
                                Sensor::Chamber => s.chamber = t,
                            }
                        }
                        if let Sensor::Nozzle = sensor {
                            // Like the stock heater_fan: hotend fan on from 50 °C,
                            // off below 45 °C. An unreadable sensor counts as hot.
                            let want = match t {
                                Some(c) if c >= 50.0 => true,
                                Some(c) if c < 45.0 => false,
                                Some(_) => node.out_on,
                                None => true,
                            };
                            if want != node.out_on {
                                if let Some(fan) = node.out_oid {
                                    if node.mcu.send(&format!("update_digital_out oid={fan} value={}", want as u8)).is_ok() {
                                        node.out_on = want;
                                    }
                                }
                            }
                        }
                    }
                }
                // Anything else (shutdowns, stats) is logged once and dropped.
                while let Some(m) = node.mcu.inbox.pop_front() {
                    if m.name() == "shutdown" || m.name() == "is_shutdown" {
                        let id = m.int("static_string_id").unwrap_or(0) as u16;
                        let why = node.mcu.dict.static_string(id).unwrap_or("?");
                        lock(&state).log(format!("{}: MCU shutdown: {why}", node.mcu.name));
                    }
                }
            }
            if idle && nodes.iter().all(|n| n.is_none()) {
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    fn handle(state: &Shared, nodes: &mut [Option<Node>; 3], job: Job, restart_dsp: bool) {
        match job {
            Job::Probe => {
                for t in Target::ALL {
                    let i = t as usize;
                    let r = match &mut nodes[i] {
                        Some(n) => n.mcu.measure_clock().map(|hz| format!("{:.2} MHz", hz / 1e6)),
                        None => Err("not connected".into()),
                    };
                    let mut s = lock(state);
                    match r {
                        Ok(d) => {
                            s.links[i] = LinkStatus { state: LinkState::Ok, detail: d.clone() };
                            s.log(format!("{}: ok, clock {d}", t.title()));
                        }
                        Err(e) => {
                            s.links[i] = LinkStatus { state: LinkState::Failed, detail: e.to_string() };
                            s.log(format!("{}: {e}", t.title()));
                        }
                    }
                }
            }
            Job::Reset(t) => {
                lock(state).set_status(format!("Resetting {}…", t.title()));
                nodes[t as usize] = None;
                nodes[t as usize] = attach(state, t, restart_dsp);
                lock(state).set_status(format!("{} reset done", t.title()));
            }
            Job::ToggleLight => {
                let on = !lock(state).light;
                let r = match &mut nodes[Target::Dsp as usize] {
                    Some(Node { mcu, out_oid: Some(oid), .. }) => mcu.send(&format!("update_digital_out oid={oid} value={}", on as u8)),
                    _ => Err("DSP not connected".into()),
                };
                let mut s = lock(state);
                match r {
                    Ok(()) => s.light = on,
                    Err(e) => s.log(format!("light: {e}")),
                }
            }
            Job::Gcode(g) => {
                let line = g.replace('\n', " · ");
                lock(state).set_status(format!("Not executed (no motion core yet): {line}"));
            }
        }
    }
}
