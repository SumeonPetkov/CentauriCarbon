//! `ccsim` — the ccos dev emulator.
//!
//! Runs the real ccos (UI, printer core, G-code thread) against a virtual
//! Centauri Carbon from `cc-sim`: three firmware-accurate MCUs on
//! pseudo-terminals, fake sysfs power pins, and a physics model of the
//! machine. Only the touchscreen and framebuffer are replaced (by a window
//! or by a script).

pub mod script;
#[cfg(feature = "window")]
pub mod window;

use std::fmt::Write as _;
use std::io;
use std::path::{Path, PathBuf};

use cc_sim::{Sim, SimConfig, WorldConfig};
use cc_ui::{Canvas, Model, Touch};
use ccos::hw::Hardware;
use ccos::runtime::{Event, Runtime};

/// Sample G-code shipped with the emulator (copied into the G-code folder).
pub const SAMPLES: &[(&str, &str)] = &[
    ("cube_20mm.gcode", include_str!("../../../sim/gcodes/cube_20mm.gcode")),
    ("arc_rings.gcode", include_str!("../../../sim/gcodes/arc_rings.gcode")),
    ("speed_zigzag.gcode", include_str!("../../../sim/gcodes/speed_zigzag.gcode")),
    ("root_gate.gcode", include_str!("../../../sim/gcodes/root_gate.gcode")),
];

#[derive(Debug, Clone)]
pub struct Options {
    /// Simulation directory (ptys, GPIO tree, G-code folder).
    pub dir: PathBuf,
    pub gcode_dir: Option<PathBuf>,
    /// Thermal speed-up (1 = real time).
    pub speed: f64,
    /// STM32 boards start in their bootloader (cold power-on).
    pub bootloader: bool,
    /// Probability of corrupting each byte the host sends.
    pub corrupt: f64,
    pub verbose: bool,
    /// Where relative screenshot paths go.
    pub out_dir: PathBuf,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            dir: std::env::temp_dir().join(format!("ccsim-{}", std::process::id())),
            gcode_dir: None,
            speed: 1.0,
            bootloader: false,
            corrupt: 0.0,
            verbose: false,
            out_dir: PathBuf::from("."),
        }
    }
}

/// A value an expression can test.
#[derive(Debug, Clone, PartialEq)]
pub enum Val {
    Num(f64),
    Str(String),
}

impl std::fmt::Display for Val {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Val::Num(n) => write!(f, "{n:.3}"),
            Val::Str(s) => write!(f, "{s:?}"),
        }
    }
}

/// Names usable in scripts (`expect`, `wait_until`) and shown in the window.
pub const QUERIES: &[(&str, &str)] = &[
    ("nozzle, bed, chamber", "true temperatures in the simulated machine (°C)"),
    ("host_nozzle, host_bed", "temperatures as ccos reads them"),
    ("x, y, z, e", "true carriage position / nozzle-bed gap / filament fed (mm)"),
    ("host_x, host_y, host_z", "position ccos shows"),
    ("extruded, retracted, cold_extrusion", "filament (mm) pushed through a hot nozzle / pulled back / pushed while cold"),
    ("layers, bbox_x0, bbox_y0, bbox_x1, bbox_y1, bbox_w, bbox_h, max_z", "what was printed"),
    ("push_x, push_y, push_z", "deepest push past a frame limit / into the bed (mm)"),
    ("lost_steps", "steps sent to disabled drivers"),
    ("tmc_<x|y|z|e>_<reg>", "TMC2209 register (gconf, gstat, ifcnt, slaveconf, ihold_irun, tpowerdown, tpwmthrs, tcoolthrs, sgthrs, chopconf, pwmconf), microsteps, writes"),
    ("pressure_advance, smooth_time, kin_flush_delay", "ccos's extruder settings (s for the times)"),
    ("shaper_type_<x|y>, shaper_freq_<x|y>, damping_<x|y>", "ccos's input shaper (type as its name)"),
    ("light, part_fan, hotend_fan, nozzle_pwm, bed_pwm", "outputs (0..1)"),
    ("mcu_shutdown", "MCU shutdown reasons (\"toolhead: ADC out of range; …\"), or \"none\""),
    ("shutdown", "ccos shutdown reason, or \"none\""),
    ("screen, status, print_state, homed", "what ccos shows (homed: e.g. \"xyz\")"),
    ("print_progress", "0..1"),
    ("links", "number of MCUs ccos is connected to"),
    ("busy", "1 while ccos executes a command or prints"),
    ("layer", "current layer as ccos counts it (;LAYER: markers)"),
    ("console", "the last 256 lines of ccos's G-code console (use ~ / !~)"),
    ("retransmits, naks", "protocol recoveries so far"),
];

pub struct Emu {
    // Field order = drop order: stop ccos before the virtual printer.
    pub rt: Runtime,
    pub sim: Sim,
    pub opts: Options,
    pub gcode_dir: PathBuf,
}

impl Emu {
    pub fn start(opts: Options) -> io::Result<Emu> {
        std::fs::create_dir_all(&opts.dir)?;
        let sim = Sim::start(SimConfig {
            dir: opts.dir.clone(),
            world: WorldConfig { thermal_speed: opts.speed, ..WorldConfig::default() },
            mcus_running: !opts.bootloader,
            corrupt_rate: opts.corrupt,
            verbose: opts.verbose,
        })?;
        let gcode_dir = opts.gcode_dir.clone().unwrap_or_else(|| opts.dir.join("gcodes"));
        std::fs::create_dir_all(&gcode_dir)?;
        for (name, text) in SAMPLES {
            let p = gcode_dir.join(name);
            if !p.exists() {
                std::fs::write(p, text)?;
            }
        }
        let mut hw = Hardware::sim(&opts.dir);
        hw.gcode_dir = gcode_dir.clone();
        let rt = Runtime::start(hw);
        rt.state().echo = opts.verbose;
        Ok(Emu { rt, sim, opts, gcode_dir })
    }

    /// One UI frame.
    pub fn frame(&mut self, touch: Touch) -> Option<Event> {
        self.rt.frame(touch)
    }

    pub fn screen_name(&self) -> &'static str {
        self.rt.engine.screen().name
    }

    /// ccos's motion settings (`SET_PRESSURE_ADVANCE`, `SET_INPUT_SHAPER`).
    fn motion_probe(&self, name: &str) -> Option<Val> {
        let c = self.rt.printer.lock();
        let sh = &c.shaper;
        let ty = |t: i32| {
            let n = ccos::motion::shaper::SHAPERS.iter().find(|(_, v)| *v == t).map_or("?", |(n, _)| *n);
            Some(Val::Str(n.into()))
        };
        match name {
            "pressure_advance" => Some(Val::Num(c.pressure_advance.0)),
            "smooth_time" => Some(Val::Num(c.pressure_advance.1)),
            "kin_flush_delay" => Some(Val::Num(c.kin_flush_delay)),
            "shaper_type_x" => ty(sh.type_x),
            "shaper_type_y" => ty(sh.type_y),
            "shaper_freq_x" => Some(Val::Num(sh.freq_x)),
            "shaper_freq_y" => Some(Val::Num(sh.freq_y)),
            "damping_x" => Some(Val::Num(sh.damping_x)),
            "damping_y" => Some(Val::Num(sh.damping_y)),
            _ => None,
        }
    }

    pub fn query(&self, name: &str) -> Option<Val> {
        use cc_sim::world::Role2;
        let num = |v: f64| Some(Val::Num(v));
        {
            let p = self.sim.lock();
            let w = &p.world;
            let bb = w.stats.bbox.unwrap_or([0.0; 4]);
            let r = match name {
                "nozzle" => num(w.hotend.temp),
                "bed" => num(w.bed.temp),
                "chamber" => num(w.chamber.temp),
                "nozzle_max" => num(w.hotend.max_temp_seen),
                "x" => num(w.pos[0]),
                "y" => num(w.pos[1]),
                "z" => num(w.pos[2]),
                "e" => num(w.e_mm),
                "extruded" => num(w.stats.extruded_mm),
                "retracted" => num(w.stats.retracted_mm),
                "cold_extrusion" => num(w.stats.cold_extrusion_mm),
                "layers" => num(w.stats.layers.len() as f64),
                "layer_heights" => Some(Val::Str(w.stats.layers.iter().map(|z| format!("{z:.2}")).collect::<Vec<_>>().join(" "))),
                "bbox_x0" => num(bb[0]),
                "bbox_y0" => num(bb[1]),
                "bbox_x1" => num(bb[2]),
                "bbox_y1" => num(bb[3]),
                "bbox_w" => num(bb[2] - bb[0]),
                "bbox_h" => num(bb[3] - bb[1]),
                "max_z" => num(w.stats.max_z_printed),
                "push_x" => num(w.push_mm[0]),
                "push_y" => num(w.push_mm[1]),
                "push_z" => num(w.push_mm[2]),
                "lost_steps" => num(w.motors.iter().map(|m| m.lost_steps).sum::<u64>() as f64),
                "light" => num(w.duty(Role2::Light)),
                "part_fan" => num(w.duty(Role2::PartFan)),
                "hotend_fan" => num(w.duty(Role2::HotendFan)),
                "nozzle_pwm" => num(w.hotend.duty),
                "bed_pwm" => num(w.bed.duty),
                "mcu_shutdown" => {
                    let all: Vec<String> = p.mcus.iter().filter_map(|m| m.is_shutdown().map(|s| format!("{}: {s}", m.kind.name()))).collect();
                    Some(Val::Str(if all.is_empty() { "none".into() } else { all.join("; ") }))
                }
                "naks" => num(p.mcus.iter().map(|m| m.naks).sum::<u64>() as f64),
                n if n.starts_with("tmc_") => {
                    // tmc_<x|y|z|e>_<register|microsteps|writes>
                    let mut it = n[4..].splitn(2, '_');
                    let m = match it.next() {
                        Some("x") => 0,
                        Some("y") => 1,
                        Some("z") => 2,
                        Some("e") => 3,
                        _ => return None,
                    };
                    let d = &w.tmc[m];
                    match it.next()? {
                        "microsteps" => num(d.microsteps() as f64),
                        "writes" => num(d.writes as f64),
                        r => cc_sim::tmc::NAMES.iter().find(|(k, _)| *k == r).and_then(|(_, a)| num(d.reg(*a) as f64)),
                    }
                }
                _ => None,
            };
            if r.is_some() {
                return r;
            }
        }
        if name == "retransmits" {
            let c = self.rt.printer.lock();
            return num(c.mcus.iter().flatten().map(|m| m.retransmits).sum::<u64>() as f64);
        }
        if let Some(r) = self.motion_probe(name) {
            return Some(r);
        }
        if name == "screen" {
            return Some(Val::Str(self.screen_name().into()));
        }
        let s = self.rt.state();
        let text = |v: ccos::ui::Value| {
            let mut b = cc_ui::TextBuf::default();
            s.text(v, &mut b);
            Some(Val::Str(b.as_str().to_string()))
        };
        match name {
            "host_nozzle" => num(s.nozzle.unwrap_or(f64::NAN)),
            "host_bed" => num(s.bed.unwrap_or(f64::NAN)),
            "host_x" => num(s.pos[0]),
            "host_y" => num(s.pos[1]),
            "host_z" => num(s.pos[2]),
            "status" => Some(Val::Str(s.status.clone())),
            "print_state" => text(ccos::ui::Value::PrintState),
            "print_progress" => num(s.print.progress as f64),
            "homed" => Some(Val::Str(['x', 'y', 'z'].iter().zip(s.homed).filter(|(_, h)| *h).map(|(c, _)| *c).collect())),
            "shutdown" => Some(Val::Str(s.shutdown.clone().unwrap_or_else(|| "none".into()))),
            "links" => num(s.links.iter().filter(|l| l.state == ccos::app::LinkState::Ok).count() as f64),
            "busy" => num(s.busy as u8 as f64),
            "layer" => num(s.print.layer as f64),
            "console" => Some(Val::Str(ccos::gcode::console_lines().join("\n"))),
            _ => None,
        }
    }

    /// The screen as a PNG.
    pub fn screenshot(&self, path: &Path) -> io::Result<()> {
        std::fs::write(path, cc_ui::png::encode(&self.rt.canvas))
    }

    /// Top view of what was printed, as a PNG.
    pub fn bedshot(&self, path: &Path) -> io::Result<()> {
        let size = 512;
        let mut c = Canvas::new(size as u16, size as u16);
        self.sim.lock().world.render_bed(&mut c.pixels, size);
        std::fs::write(path, cc_ui::png::encode(&c))
    }

    /// One-line summary of the simulated machine.
    pub fn summary(&self) -> Vec<String> {
        let p = self.sim.lock();
        let w = &p.world;
        let mut v = Vec::new();
        let mut l = String::new();
        let _ = write!(
            l,
            "nozzle {:.1}° ({:.0}%)  bed {:.1}° ({:.0}%)  chamber {:.1}°",
            w.hotend.temp,
            w.hotend.duty * 100.0,
            w.bed.temp,
            w.bed.duty * 100.0,
            w.chamber.temp
        );
        v.push(l);
        v.push(format!("carriage X{:.2} Y{:.2}  gap Z{:.3}  E{:.2}", w.pos[0], w.pos[1], w.pos[2], w.e_mm));
        v.push(format!(
            "printed {:.0} mm filament, {} layers  cold {:.1} mm",
            w.stats.extruded_mm,
            w.stats.layers.len(),
            w.stats.cold_extrusion_mm
        ));
        let mcus: Vec<String> = p
            .mcus
            .iter()
            .map(|m| {
                let st = match (m.power, m.is_shutdown()) {
                    (_, Some(s)) => format!("SHUTDOWN {s}"),
                    (cc_sim::mcu::Power::Off, _) => "off".into(),
                    (cc_sim::mcu::Power::Bootloader { .. }, _) => "bootloader".into(),
                    _ if m.is_configured() => format!("ok, {} moves queued", m.moves_used()),
                    _ => "not configured".into(),
                };
                format!("{}: {st}", m.kind.name())
            })
            .collect();
        v.push(mcus.join("  "));
        if let Some((t, msg)) = w.log.last() {
            v.push(format!("[{t:.1}s] {msg}"));
        }
        v
    }
}
