//! What the screen shows ([`State`]) and how live values and flags named in
//! `ui/*.toml` map to it (the [`Model`] implementation).
//!
//! The printer core and the G-code thread write into the state; the UI
//! thread only reads it, so slow I/O never stalls the screen.

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Instant;

use cc_ui::{Matrix, Model, TextBuf};

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Phase {
    #[default]
    Idle,
    Printing,
    Paused,
    Done,
    Cancelled,
    Failed,
}

#[derive(Debug, Clone, Default)]
pub struct PrintInfo {
    pub phase: Phase,
    pub file: String,
    pub progress: f32,
    pub started: Option<Instant>,
    pub elapsed: f64,
    pub error: String,
    pub layer: u32,
    pub total_layers: u32,
}

pub struct State {
    pub nozzle: Option<f64>,
    pub nozzle_target: f64,
    pub bed: Option<f64>,
    pub bed_target: f64,
    pub chamber: Option<f64>,
    pub light: bool,
    pub fan: f64,
    pub links: [LinkStatus; 3],
    pub status: String,
    pub log: VecDeque<String>,
    pub backlight: u8,
    pub rss_kib: u64,
    pub cpu_pct: f32,
    pub started: Instant,
    pub pos: [f64; 4],
    pub homed: [bool; 3],
    pub print: PrintInfo,
    /// Printable files, newest first.
    pub files: Vec<String>,
    pub gcode_dir: String,
    pub shutdown: Option<String>,
    pub motion_lock: Option<&'static str>,
    /// The G-code thread is executing something.
    pub busy: bool,
    /// Mirror log lines to stderr.
    pub echo: bool,
    /// Saved bed meshes (filled by the G-code thread).
    pub meshes: Vec<crate::mesh::ProfileView>,
    /// Section of the mesh in use, if any.
    pub mesh_active: Option<String>,
    /// The mesh shown on the bed-mesh screen.
    pub mesh_sel: usize,
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
            fan: 0.0,
            links: Default::default(),
            status: "Starting…".into(),
            log: VecDeque::with_capacity(8),
            backlight: 191,
            rss_kib: 0,
            cpu_pct: 0.0,
            started: Instant::now(),
            pos: [0.0; 4],
            homed: [false; 3],
            print: PrintInfo::default(),
            files: Vec::new(),
            gcode_dir: String::new(),
            shutdown: None,
            motion_lock: None,
            busy: false,
            echo: true,
            meshes: Vec::new(),
            mesh_active: None,
            mesh_sel: 0,
        }
    }
}

impl State {
    pub fn log(&mut self, s: impl Into<String>) {
        let s = s.into();
        if self.echo {
            eprintln!("ccos: {s}");
        }
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

    /// The bed mesh shown on the bed-mesh screen.
    pub fn shown_mesh(&self) -> Option<&crate::mesh::ProfileView> {
        self.meshes.get(self.mesh_sel.min(self.meshes.len().saturating_sub(1)))
    }

    /// Step through the saved meshes (wrapping).
    pub fn mesh_step(&mut self, forward: bool) {
        let n = self.meshes.len();
        if n > 0 {
            let cur = self.mesh_sel.min(n - 1);
            self.mesh_sel = if forward { (cur + 1) % n } else { (cur + n - 1) % n };
        }
    }
}

/// Lowest, highest and mean probed height.
fn mesh_stats(m: &crate::mesh::ProfileView) -> (f64, f64, f64) {
    let lo = m.points.iter().cloned().fold(f64::INFINITY, f64::min);
    let hi = m.points.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let mean = m.points.iter().sum::<f64>() / m.points.len().max(1) as f64;
    (lo, hi, mean)
}

/// Shared handle to the screen state.
#[derive(Clone, Default)]
pub struct Ui(pub Arc<Mutex<State>>);

impl Ui {
    pub fn lock(&self) -> MutexGuard<'_, State> {
        self.0.lock().unwrap_or_else(|p| p.into_inner())
    }
    pub fn log(&self, s: impl Into<String>) {
        self.lock().log(s);
    }
    pub fn set_status(&self, s: impl Into<String>) {
        self.lock().set_status(s);
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

fn duration(out: &mut TextBuf, s: f64) {
    let s = s as u64;
    if s >= 3600 {
        let _ = write!(out, "{}h {:02}m", s / 3600, s / 60 % 60);
    } else {
        let _ = write!(out, "{}m {:02}s", s / 60, s % 60);
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
            Value::File1 | Value::File2 | Value::File3 | Value::File4 | Value::File5 => {
                let i = match v {
                    Value::File1 => 0,
                    Value::File2 => 1,
                    Value::File3 => 2,
                    Value::File4 => 3,
                    _ => 4,
                };
                if let Some(f) = self.files.get(i) {
                    out.push_str(f);
                }
            }
            Value::FilesHint => {
                let _ = write!(out, "No .gcode files in {}", self.gcode_dir);
            }
            Value::PrintState => out.push_str(match self.print.phase {
                Phase::Idle => "Ready",
                Phase::Printing => "Printing",
                Phase::Paused => "Paused",
                Phase::Done => "Finished",
                Phase::Cancelled => "Cancelled",
                Phase::Failed => "Failed",
            }),
            Value::PrintFile => {
                if self.print.phase == Phase::Failed {
                    out.push_str(&self.print.error);
                } else if let Some(l) = self.motion_lock.filter(|_| self.print.phase == Phase::Idle) {
                    out.push_str(l);
                } else {
                    out.push_str(&self.print.file);
                }
            }
            Value::PrintProgress => {}
            Value::PrintProgressText => {
                if self.print.phase != Phase::Idle {
                    let _ = write!(out, "{:.0}% · ", self.print.progress * 100.0);
                    let elapsed = match (self.print.phase, self.print.started) {
                        (Phase::Printing | Phase::Paused, Some(t)) => t.elapsed().as_secs_f64(),
                        _ => self.print.elapsed,
                    };
                    duration(out, elapsed);
                }
            }
            Value::Position => {
                let p = self.pos;
                let _ = write!(out, "X{:.1} Y{:.1} Z{:.2}", p[0], p[1], p[2]);
            }
            Value::MeshMap => {}
            Value::MeshHint => out.push_str("No saved bed meshes yet.

BED_MESH_CALIBRATE probes the bed; add S<°C> to heat the bed first and a name to save it under that name, e.g.
BED_MESH_CALIBRATE S75 pla"),
            Value::MeshPage => {
                if !self.meshes.is_empty() {
                    let _ = write!(out, "{} / {}", self.mesh_sel.min(self.meshes.len() - 1) + 1, self.meshes.len());
                }
            }
            Value::MeshTitle => {
                if let Some(m) = self.shown_mesh() {
                    out.push_str(&m.name);
                }
            }
            Value::MeshDetail => {
                if let Some(m) = self.shown_mesh() {
                    let _ = write!(out, "{} plate\n{}×{} · {}", m.material, m.x_count, m.y_count, m.algo);
                }
            }
            Value::MeshState => {
                if self.shown_mesh().is_some() {
                    out.push_str(if self.flag(Flag::MeshInUse) { "In use" } else { "Not in use" });
                }
            }
            Value::MeshHigh | Value::MeshLow | Value::MeshRange | Value::MeshMean => {
                if let Some(m) = self.shown_mesh().filter(|m| !m.points.is_empty()) {
                    let (lo, hi, mean) = mesh_stats(m);
                    let _ = match v {
                        Value::MeshHigh => write!(out, "High {hi:+.3} mm"),
                        Value::MeshLow => write!(out, "Low {lo:+.3} mm"),
                        Value::MeshRange => write!(out, "Range {:.3} mm", hi - lo),
                        _ => write!(out, "Mean {mean:+.3} mm"),
                    };
                }
            }
        }
    }

    fn matrix(&self, v: Value, out: &mut Matrix) {
        if v != Value::MeshMap {
            return;
        }
        let Some(m) = self.shown_mesh() else { return };
        if m.points.len() == m.x_count * m.y_count && out.resize(m.x_count, m.y_count) {
            for (o, z) in out.z.iter_mut().zip(&m.points) {
                *o = *z as f32;
            }
        }
    }

    fn level(&self, v: Value) -> f32 {
        match v {
            Value::Backlight => self.backlight as f32 / 255.0,
            Value::PrintProgress => self.print.progress,
            _ => 0.0,
        }
    }

    fn flag(&self, f: Flag) -> bool {
        let p = self.print.phase;
        match f {
            Flag::NozzleHeating => self.nozzle_target > 0.0,
            Flag::BedHeating => self.bed_target > 0.0,
            Flag::LightOn => self.light,
            Flag::HasFile1 => !self.files.is_empty(),
            Flag::HasFile2 => self.files.len() > 1,
            Flag::HasFile3 => self.files.len() > 2,
            Flag::HasFile4 => self.files.len() > 3,
            Flag::HasFile5 => self.files.len() > 4,
            Flag::Printing => p == Phase::Printing,
            Flag::Paused => p == Phase::Paused,
            Flag::PrintActive => matches!(p, Phase::Printing | Phase::Paused),
            Flag::Shutdown => self.shutdown.is_some(),
            Flag::HasMesh => !self.meshes.is_empty(),
            Flag::MeshMany => self.meshes.len() > 1,
            Flag::MeshInUse => self.shown_mesh().is_some_and(|m| self.mesh_active.as_deref() == Some(m.section.as_str())),
        }
    }
}
