//! The physical printer behind the virtual MCUs: motors, belts and carriage
//! (CoreXY), Z, extruder, heaters and thermistors, fans, the chamber light,
//! TMC2209 stall outputs and the Z endstop — wired exactly as the stock
//! `printer.cfg` says (pins, direction and enable inversions, step distances).
//!
//! What the world checks for you:
//! * motors that step while disabled (lost position),
//! * carriage pushed against a frame limit or nozzle pushed into the bed
//!   (`push_mm` per axis — homing pushes a little, a crash pushes a lot),
//! * extrusion below 170 °C (`cold_extrusion_mm`),
//! * what was printed: a 2 px/mm height map of the bed, bounding box,
//!   layers, extruded length.

use std::collections::HashMap;

use crate::mcu::{Hardware, McuKind};
use cc_proto::dict::pin_number;

/// Faults a test can inject.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Fault {
    /// Heater cartridge disconnected: PWM has no effect.
    HotendHeaterDead,
    BedHeaterDead,
    /// Thermistor wire broken: ADC reads full scale.
    HotendThermistorOpen,
    /// Thermistor shorted: ADC reads 0.
    HotendThermistorShort,
    BedThermistorOpen,
    /// Thermistor fell out of the heater block: reads ambient.
    HotendThermistorLoose,
    /// Toolhead board stops answering (USB cable unplugged).
    ToolheadUnplugged,
}

impl Fault {
    pub const ALL: [Fault; 7] = [
        Fault::HotendHeaterDead,
        Fault::BedHeaterDead,
        Fault::HotendThermistorOpen,
        Fault::HotendThermistorShort,
        Fault::BedThermistorOpen,
        Fault::HotendThermistorLoose,
        Fault::ToolheadUnplugged,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Fault::HotendHeaterDead => "hotend_heater_dead",
            Fault::BedHeaterDead => "bed_heater_dead",
            Fault::HotendThermistorOpen => "hotend_thermistor_open",
            Fault::HotendThermistorShort => "hotend_thermistor_short",
            Fault::BedThermistorOpen => "bed_thermistor_open",
            Fault::HotendThermistorLoose => "hotend_thermistor_loose",
            Fault::ToolheadUnplugged => "toolhead_unplugged",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|f| f.name() == s)
    }
}

#[derive(Debug, Clone)]
pub struct WorldConfig {
    /// Heat/cool this many times faster than real (1 = realistic).
    pub thermal_speed: f64,
    pub ambient: f64,
    /// Where the carriage and bed are at power-on (unknown to the host).
    pub start: [f64; 3],
    pub x_limits: (f64, f64),
    pub y_limits: (f64, f64),
    /// Nozzle-to-bed gap limits; 0 = nozzle touching the bed.
    pub z_limits: (f64, f64),
    /// Gap at which the Z endstop (PG8) reads triggered.
    pub z_trigger: f64,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            thermal_speed: 1.0,
            ambient: 24.0,
            start: [117.3, 141.9, 37.25],
            // stock printer.cfg position_min/max (X/Y home against the frame)
            x_limits: (-2.0, 256.5),
            y_limits: (-2.5, 270.0),
            z_limits: (0.0, 258.0),
            // PG8 is a separate sensor (not the load cells) that closes with
            // the bed ~4.5 mm below the nozzle (printer.cfg position_endstop_extra)
            z_trigger: 4.5,
        }
    }
}

/// Motors: 0 = A (stepper_x), 1 = B (stepper_y), 2 = Z, 3 = extruder.
pub const MOTOR_NAMES: [&str; 4] = ["A (stepper_x)", "B (stepper_y)", "Z", "extruder"];

#[derive(Debug, Clone, Copy)]
struct MotorWiring {
    mcu: McuKind,
    step: &'static str,
    /// Documentation only: the MCU reports the dir pin level with each step.
    #[allow(dead_code)]
    dir: &'static str,
    enable: &'static str,
    /// `!` on dir_pin in printer.cfg: physical + when the dir pin is low.
    dir_inverted: bool,
    step_dist: f64,
}

const XY_STEP: f64 = 0.012460;
const Z_STEP: f64 = 0.0025;
/// rotation_distance 28.8, gear 52:10, 200 full steps, 16 microsteps.
pub const E_STEP: f64 = 28.8 / (200.0 * 16.0 * 5.2);

const MOTORS: [MotorWiring; 4] = [
    MotorWiring { mcu: McuKind::Dsp, step: "PE8", dir: "PE7", enable: "PE10", dir_inverted: true, step_dist: XY_STEP },
    MotorWiring { mcu: McuKind::Dsp, step: "PE14", dir: "PE13", enable: "PE15", dir_inverted: true, step_dist: XY_STEP },
    MotorWiring { mcu: McuKind::Dsp, step: "PG3", dir: "PG2", enable: "PG1", dir_inverted: true, step_dist: Z_STEP },
    MotorWiring { mcu: McuKind::Toolhead, step: "PC13", dir: "PC14", enable: "PC15", dir_inverted: false, step_dist: E_STEP },
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    Step(usize),
    Enable(usize),
    HotendHeater,
    BedHeater,
    HotendThermistor,
    BedThermistor,
    ChamberThermistor,
    PartFan,
    HotendFan,
    AuxFan,
    ChamberFan,
    BoardFan,
    Light,
    DiagX,
    DiagY,
    ZEndstop,
}

const PINS: &[(McuKind, &str, Role)] = &[
    (McuKind::Dsp, "PG13", Role::BedHeater),
    (McuKind::Dsp, "PB14", Role::BedThermistor),
    (McuKind::Dsp, "PB13", Role::ChamberThermistor),
    (McuKind::Dsp, "PG15", Role::Light),
    (McuKind::Dsp, "PG17", Role::AuxFan),
    (McuKind::Dsp, "PG18", Role::ChamberFan),
    (McuKind::Dsp, "PG16", Role::BoardFan),
    (McuKind::Dsp, "PG4", Role::DiagX),
    (McuKind::Dsp, "PF6", Role::DiagY),
    (McuKind::Dsp, "PG8", Role::ZEndstop),
    (McuKind::Toolhead, "PB6", Role::HotendHeater),
    (McuKind::Toolhead, "PA3", Role::HotendThermistor),
    (McuKind::Toolhead, "PB5", Role::PartFan),
    (McuKind::Toolhead, "PC8", Role::HotendFan),
];

#[derive(Debug, Clone, Default)]
pub struct Motor {
    /// Physical position in steps (+ = positive axis direction).
    pub steps: i64,
    pub enabled: bool,
    /// Steps that arrived while the driver was disabled.
    pub lost_steps: u64,
    pub total_steps: u64,
}

/// A lumped thermal mass: `C dT/dt = P·duty − k (T − T_amb)`.
#[derive(Debug, Clone)]
pub struct Thermal {
    pub temp: f64,
    pub duty: f64,
    pub watts: f64,
    pub capacity: f64,
    pub loss: f64,
    pub max_temp_seen: f64,
}

impl Thermal {
    fn new(ambient: f64, watts: f64, capacity: f64, loss: f64) -> Self {
        Self { temp: ambient, duty: 0.0, watts, capacity, loss, max_temp_seen: ambient }
    }
}

/// Print raster resolution.
pub const RASTER_PX_PER_MM: f64 = 2.0;
pub const RASTER_SIZE: usize = 512;

#[derive(Debug, Clone, Default)]
pub struct PrintStats {
    pub extruded_mm: f64,
    pub retracted_mm: f64,
    pub cold_extrusion_mm: f64,
    /// Bounding box of extrusion: x0, y0, x1, y1.
    pub bbox: Option<[f64; 4]>,
    pub layers: Vec<f64>,
    pub max_z_printed: f64,
}

pub struct World {
    pub cfg: WorldConfig,
    pub motors: [Motor; 4],
    /// Carriage x, y and nozzle-to-bed gap z (mm).
    pub pos: [f64; 3],
    offset: [f64; 3],
    pub stalled: [bool; 3],
    /// Deepest push past a limit, per axis (mm). Homing pushes ~0.
    pub push_mm: [f64; 3],
    pub e_mm: f64,
    pub hotend: Thermal,
    pub bed: Thermal,
    pub chamber: Thermal,
    pub outputs: HashMap<Role2, f64>,
    roles: HashMap<(McuKind, u32), Role>,
    pub faults: std::collections::HashSet<Fault>,
    pub raster: Vec<f32>,
    pub stats: PrintStats,
    pub log: Vec<(f64, String)>,
    noise: u64,
    pub t: f64,
}

/// Named outputs, for display and scripts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Role2 {
    HotendHeater,
    BedHeater,
    PartFan,
    HotendFan,
    AuxFan,
    ChamberFan,
    BoardFan,
    Light,
}

impl Role2 {
    pub const ALL: [Role2; 8] =
        [Role2::HotendHeater, Role2::BedHeater, Role2::PartFan, Role2::HotendFan, Role2::AuxFan, Role2::ChamberFan, Role2::BoardFan, Role2::Light];
    pub fn name(self) -> &'static str {
        match self {
            Role2::HotendHeater => "hotend_heater",
            Role2::BedHeater => "bed_heater",
            Role2::PartFan => "part_fan",
            Role2::HotendFan => "hotend_fan",
            Role2::AuxFan => "aux_fan",
            Role2::ChamberFan => "chamber_fan",
            Role2::BoardFan => "board_fan",
            Role2::Light => "light",
        }
    }
}

fn role2(r: Role) -> Option<Role2> {
    Some(match r {
        Role::HotendHeater => Role2::HotendHeater,
        Role::BedHeater => Role2::BedHeater,
        Role::PartFan => Role2::PartFan,
        Role::HotendFan => Role2::HotendFan,
        Role::AuxFan => Role2::AuxFan,
        Role::ChamberFan => Role2::ChamberFan,
        Role::BoardFan => Role2::BoardFan,
        Role::Light => Role2::Light,
        _ => return None,
    })
}

/// 100 kΩ NTC on a 4.7 kΩ pull-up → 12-bit ADC count.
pub fn ntc_adc(temp_c: f64, beta: f64) -> f64 {
    let (r0, t0, pullup) = (100_000.0f64, 25.0 + 273.15, 4700.0);
    let r = r0 * (beta * (1.0 / (temp_c + 273.15) - 1.0 / t0)).exp();
    4095.0 * r / (r + pullup)
}

impl World {
    pub fn new(cfg: WorldConfig) -> Self {
        let mut roles = HashMap::new();
        for (i, m) in MOTORS.iter().enumerate() {
            let ppb = m.mcu.pins_per_bank();
            roles.insert((m.mcu, pin_number(m.step, ppb).unwrap()), Role::Step(i));
            roles.insert((m.mcu, pin_number(m.enable, ppb).unwrap()), Role::Enable(i));
        }
        for (mcu, p, r) in PINS {
            roles.insert((*mcu, pin_number(p, mcu.pins_per_bank()).unwrap()), *r);
        }
        let a = cfg.ambient;
        Self {
            pos: cfg.start,
            offset: cfg.start,
            motors: Default::default(),
            stalled: [false; 3],
            push_mm: [0.0; 3],
            e_mm: 0.0,
            // ~60 W cartridge, small aluminium block: 25→220 °C in under a minute.
            hotend: Thermal::new(a, 60.0, 11.0, 0.12),
            // 350 W AC bed: 60 °C in ~1.5 min, 100 °C in ~5 min.
            bed: Thermal::new(a, 350.0, 900.0, 2.4),
            chamber: Thermal::new(a, 0.0, 6000.0, 3.0),
            outputs: HashMap::new(),
            roles,
            faults: Default::default(),
            raster: vec![f32::NAN; RASTER_SIZE * RASTER_SIZE],
            stats: PrintStats::default(),
            log: Vec::new(),
            noise: 0x9e37_79b9_7f4a_7c15,
            t: 0.0,
            cfg,
        }
    }

    pub fn duty(&self, r: Role2) -> f64 {
        self.outputs.get(&r).copied().unwrap_or(0.0)
    }

    pub fn note_event(&mut self, t: f64, msg: impl Into<String>) {
        if self.log.len() >= 4096 {
            self.log.drain(..1024);
        }
        self.log.push((t, msg.into()));
    }

    fn rand(&mut self) -> f64 {
        // xorshift64*
        self.noise ^= self.noise >> 12;
        self.noise ^= self.noise << 25;
        self.noise ^= self.noise >> 27;
        (self.noise.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Integrate heaters up to time `t`.
    pub fn advance_thermal(&mut self, t: f64) {
        let mut dt = (t - self.t).max(0.0) * self.cfg.thermal_speed;
        self.t = t;
        let amb = self.cfg.ambient;
        let part_fan = self.duty(Role2::PartFan);
        let hot_dead = self.faults.contains(&Fault::HotendHeaterDead);
        let bed_dead = self.faults.contains(&Fault::BedHeaterDead);
        while dt > 0.0 {
            let h = dt.min(0.05);
            dt -= h;
            let he = &mut self.hotend;
            let p = if hot_dead { 0.0 } else { he.watts * he.duty };
            let k = he.loss * (1.0 + 0.6 * part_fan);
            he.temp += h * (p - k * (he.temp - amb)) / he.capacity;
            let b = &mut self.bed;
            let p = if bed_dead { 0.0 } else { b.watts * b.duty };
            b.temp += h * (p - b.loss * (b.temp - amb)) / b.capacity;
            let bed_t = self.bed.temp;
            let c = &mut self.chamber;
            let fan = self.outputs.get(&Role2::ChamberFan).copied().unwrap_or(0.0);
            c.temp += h * (1.2 * (bed_t - c.temp) - c.loss * (1.0 + 2.0 * fan) * (c.temp - amb)) / c.capacity;
        }
        for th in [&mut self.hotend, &mut self.bed, &mut self.chamber] {
            th.max_temp_seen = th.max_temp_seen.max(th.temp);
        }
    }

    fn carriage_moved(&mut self, axis_mask: u8) {
        let a = self.motors[0].steps as f64 * XY_STEP;
        let b = self.motors[1].steps as f64 * XY_STEP;
        let z = self.motors[2].steps as f64 * Z_STEP;
        let cand = [(a + b) * 0.5 + self.offset[0], (a - b) * 0.5 + self.offset[1], z + self.offset[2]];
        let lim = [self.cfg.x_limits, self.cfg.y_limits, self.cfg.z_limits];
        for i in 0..3 {
            if axis_mask & (1 << i) == 0 {
                continue;
            }
            let c = cand[i].clamp(lim[i].0, lim[i].1);
            if c != cand[i] {
                // Belt/leadscrew slips: the motor turns, the carriage does not.
                self.offset[i] += c - cand[i];
                self.push_mm[i] = self.push_mm[i].max((c - cand[i]).abs());
                self.stalled[i] = true;
            } else {
                self.stalled[i] = false;
            }
            self.pos[i] = c;
        }
    }

    fn extrude(&mut self, de: f64) {
        self.e_mm += de;
        if de < 0.0 {
            self.stats.retracted_mm -= de;
            return;
        }
        if self.hotend.temp < 170.0 {
            self.stats.cold_extrusion_mm += de;
            return;
        }
        self.stats.extruded_mm += de;
        let [x, y, z] = self.pos;
        let bb = self.stats.bbox.get_or_insert([x, y, x, y]);
        bb[0] = bb[0].min(x);
        bb[1] = bb[1].min(y);
        bb[2] = bb[2].max(x);
        bb[3] = bb[3].max(y);
        let zq = (z * 100.0).round() / 100.0;
        if !self.stats.layers.contains(&zq) {
            self.stats.layers.push(zq);
        }
        self.stats.max_z_printed = self.stats.max_z_printed.max(z);
        let px = (x * RASTER_PX_PER_MM).floor();
        let py = (y * RASTER_PX_PER_MM).floor();
        if px >= 0.0 && py >= 0.0 && (px as usize) < RASTER_SIZE && (py as usize) < RASTER_SIZE {
            let i = py as usize * RASTER_SIZE + px as usize;
            let v = &mut self.raster[i];
            if v.is_nan() || (z as f32) > *v {
                *v = z as f32;
            }
        }
    }

    /// Clear the print record (new job).
    pub fn clear_print(&mut self) {
        self.raster.fill(f32::NAN);
        self.stats = PrintStats::default();
    }

    /// Top-down view of the bed: printed areas coloured by height.
    pub fn render_bed(&self, out: &mut [u32], size: usize) {
        let maxz = self.stats.max_z_printed.max(0.2) as f32;
        for py in 0..size {
            for px in 0..size {
                // image y grows downwards; bed y grows away from the viewer
                let rx = px * RASTER_SIZE / size;
                let ry = (size - 1 - py) * RASTER_SIZE / size;
                let v = self.raster[ry * RASTER_SIZE + rx];
                let grid = (rx % 40 == 0 || ry % 40 == 0) as u32;
                out[py * size + px] = if v.is_nan() {
                    if grid == 1 { 0x3a3a3c } else { 0x2a2a2c }
                } else {
                    let f = (v / maxz).clamp(0.0, 1.0);
                    let r = (40.0 + 215.0 * f) as u32;
                    let g = (200.0 - 80.0 * f) as u32;
                    let b = (255.0 - 200.0 * f) as u32;
                    (r << 16) | (g << 8) | b
                };
            }
        }
        // nozzle marker
        let cx = (self.pos[0] * RASTER_PX_PER_MM) as i64 * size as i64 / RASTER_SIZE as i64;
        let cy = size as i64 - 1 - (self.pos[1] * RASTER_PX_PER_MM) as i64 * size as i64 / RASTER_SIZE as i64;
        for d in -6i64..=6 {
            for (x, y) in [(cx + d, cy), (cx, cy + d)] {
                if x >= 0 && y >= 0 && (x as usize) < size && (y as usize) < size {
                    out[y as usize * size + x as usize] = 0xff4040;
                }
            }
        }
    }
}

impl Hardware for World {
    fn step(&mut self, mcu: McuKind, step_pin: u32, dir: bool, t: f64) {
        let Some(Role::Step(i)) = self.roles.get(&(mcu, step_pin)).copied() else { return };
        let w = MOTORS[i];
        let m = &mut self.motors[i];
        m.total_steps += 1;
        if !m.enabled {
            if m.lost_steps == 0 {
                self.note_event(t, format!("motor {} stepped while disabled (steps are lost)", MOTOR_NAMES[i]));
            }
            self.motors[i].lost_steps += 1;
            return;
        }
        let d = if dir != w.dir_inverted { 1 } else { -1 };
        m.steps += d;
        match i {
            0 | 1 => self.carriage_moved(0b011),
            2 => {
                let was = self.stalled[2];
                self.carriage_moved(0b100);
                if self.stalled[2] && !was && self.pos[2] <= self.cfg.z_limits.0 {
                    self.note_event(t, format!("nozzle pressed into the bed at x={:.1} y={:.1}", self.pos[0], self.pos[1]));
                }
            }
            _ => self.extrude(d as f64 * w.step_dist),
        }
    }

    fn output(&mut self, mcu: McuKind, pin: u32, duty: f64, t: f64) {
        let Some(r) = self.roles.get(&(mcu, pin)).copied() else { return };
        if let Role::Enable(i) = r {
            // All enable pins are active-low ("!PE10" etc.).
            let en = duty < 0.5;
            if self.motors[i].enabled != en {
                self.motors[i].enabled = en;
                if !en {
                    self.note_event(t, format!("motor {} disabled", MOTOR_NAMES[i]));
                }
            }
            return;
        }
        self.advance_thermal(t);
        match r {
            Role::HotendHeater => self.hotend.duty = duty,
            Role::BedHeater => self.bed.duty = duty,
            _ => {}
        }
        if let Some(r2) = role2(r) {
            self.outputs.insert(r2, duty);
        }
    }

    fn read_pin(&mut self, mcu: McuKind, pin: u32, _t: f64) -> bool {
        match self.roles.get(&(mcu, pin)) {
            // TMC2209 DIAG is active-low in printer.cfg (`diag_pin: !PG4`).
            Some(Role::DiagX) => !self.stalled[0],
            Some(Role::DiagY) => !self.stalled[1],
            Some(Role::ZEndstop) => self.pos[2] <= self.cfg.z_trigger + 1e-9,
            _ => false,
        }
    }

    fn read_adc(&mut self, mcu: McuKind, pin: u32, t: f64) -> u16 {
        self.advance_thermal(t);
        let n = (self.rand() - 0.5) * 3.0;
        let v = match self.roles.get(&(mcu, pin)) {
            Some(Role::HotendThermistor) => {
                if self.faults.contains(&Fault::HotendThermistorOpen) {
                    4095.0
                } else if self.faults.contains(&Fault::HotendThermistorShort) {
                    0.0
                } else if self.faults.contains(&Fault::HotendThermistorLoose) {
                    ntc_adc(self.cfg.ambient + 5.0, 4300.0) + n
                } else {
                    ntc_adc(self.hotend.temp, 4300.0) + n
                }
            }
            Some(Role::BedThermistor) => {
                if self.faults.contains(&Fault::BedThermistorOpen) {
                    4095.0
                } else {
                    ntc_adc(self.bed.temp, 3950.0) + n
                }
            }
            Some(Role::ChamberThermistor) => ntc_adc(self.chamber.temp, 3950.0) + n,
            _ => 2048.0 + n,
        };
        v.round().clamp(0.0, 4095.0) as u16
    }

    fn note(&mut self, mcu: McuKind, t: f64, msg: &str) {
        self.note_event(t, format!("{}: {msg}", mcu.name()));
    }
}
