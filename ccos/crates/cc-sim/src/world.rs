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

use std::collections::{HashMap, VecDeque};

use crate::mcu::{Hardware, McuKind};
use crate::tmc::{self, Tmc2209};
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
    /// stepper_x's DIAG wire broken: X homing never triggers.
    XDiagDead,
}

impl Fault {
    pub const ALL: [Fault; 8] = [
        Fault::HotendHeaterDead,
        Fault::BedHeaterDead,
        Fault::HotendThermistorOpen,
        Fault::HotendThermistorShort,
        Fault::BedThermistorOpen,
        Fault::HotendThermistorLoose,
        Fault::ToolheadUnplugged,
        Fault::XDiagDead,
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
            Fault::XDiagDead => "x_diag_dead",
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

/// TMC2209 UARTs: (MCU, tx pin, rx pin, [(node address, motor)]). The
/// address is strapped with MS1/MS2 on the board.
const UARTS: [(McuKind, &str, &str, &[(u8, usize)]); 2] =
    [(McuKind::Dsp, "PE9", "PE6", &[(0, 0), (3, 1), (1, 2)]), (McuKind::Toolhead, "PC6", "PC7", &[(3, 3)])];
/// Microsteps the step distances above assume (printer.cfg `microsteps`).
const CFG_MICROSTEPS: f64 = 16.0;

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
    /// Step pulses, signed by direction.
    pub steps: i64,
    /// Rotor position in 1/16 steps (what `step_dist` is per): a pulse
    /// moves 16/µsteps of them, µsteps as the driver is set.
    pub travel: f64,
    pub enabled: bool,
    /// Steps that arrived while the driver was disabled.
    pub lost_steps: u64,
    pub total_steps: u64,
    last_step_t: f64,
    step_interval: f64,
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
    /// Heights (0.01 mm) where at least `LAYER_MIN_MM` of filament went
    /// (pressure advance smoothing lets a trace out during Z moves).
    pub layers: Vec<f64>,
    /// Filament per height, before it counts as a layer.
    per_height: Vec<(f64, f64)>,
    pub max_z_printed: f64,
}

const LAYER_MIN_MM: f64 = 0.05;

/// The bed rests on four load cells: the nozzle can push it down this far
/// (gap below 0) before Z slips.
pub const BED_GIVE: f64 = 0.5;
/// The bridge reading falls by this much per mm the bed is pushed down
/// (the HX717 reads the four half-bridge cells as one bridge).
pub const LOAD_COUNTS_PER_MM: f64 = 60_000.0;
/// Bridge reading with the bed unloaded, and its noise (1 σ, counts).
pub const BRIDGE_ZERO: f64 = 251_000.0;
pub const BRIDGE_NOISE: f64 = 40.0;
/// Nozzle flow lag (s): the stock pressure_advance value is tuned for it.
pub const MELT_TAU: f64 = 0.04;
/// Melt is laid down this far behind the newest step (longer than an MCU's
/// burst of steps).
const FLOW_LAG: f64 = 0.05;

fn insert_by_time<T>(q: &mut VecDeque<(f64, T)>, e: (f64, T)) {
    let i = q.iter().rposition(|x| x.0 <= e.0).map_or(0, |i| i + 1);
    q.insert(i, e);
}

pub struct World {
    pub cfg: WorldConfig,
    pub motors: [Motor; 4],
    /// The motors' TMC2209 drivers (A, B, Z, extruder).
    pub tmc: [Tmc2209; 4],
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
    /// Molten filament in the nozzle not yet laid down (mm of filament).
    pub melt: f64,
    flow_t: f64,
    unmarked: f64,
    /// Filament fed (time, mm), and where the nozzle was (time, x/y/z),
    /// not yet laid down: each MCU applies its steps in bursts, so they
    /// arrive out of order across MCUs. `hist_base` is the position before
    /// the first entry of `nozzle_hist`.
    feed: VecDeque<(f64, f64)>,
    nozzle_hist: VecDeque<(f64, [f64; 3])>,
    hist_base: [f64; 3],
    latest_t: f64,
    /// Recent gaps (time, z) for the load cells, which sample on their own
    /// MCU's clock.
    z_hist: VecDeque<(f64, f64)>,
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
            tmc: [0usize, 1, 2, 3].map(|m| {
                let addr = UARTS.iter().flat_map(|u| u.3.iter()).find(|(_, mm)| *mm == m).map_or(0, |(a, _)| *a);
                Tmc2209::power_on(addr)
            }),
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
            melt: 0.0,
            flow_t: 0.0,
            unmarked: 0.0,
            feed: VecDeque::new(),
            nozzle_hist: VecDeque::new(),
            hist_base: cfg.start,
            latest_t: 0.0,
            z_hist: VecDeque::new(),
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
        self.flow(t);
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

    /// What a driver sees of its motor at time `t`.
    fn live(&self, i: usize, t: f64) -> tmc::Live {
        let m = &self.motors[i];
        let stalled = match i {
            0 | 1 => self.stalled[0] || self.stalled[1],
            2 => self.stalled[2],
            _ => false,
        };
        tmc::Live { step_interval: m.step_interval, since_step: t - m.last_step_t, enabled: m.enabled, stalled }
    }

    fn carriage_moved(&mut self, axis_mask: u8) {
        let a = self.motors[0].travel * XY_STEP;
        let b = self.motors[1].travel * XY_STEP;
        let z = self.motors[2].travel * Z_STEP;
        let cand = [(a + b) * 0.5 + self.offset[0], (a - b) * 0.5 + self.offset[1], z + self.offset[2]];
        let z_lim = (self.cfg.z_limits.0 - BED_GIVE, self.cfg.z_limits.1);
        let lim = [self.cfg.x_limits, self.cfg.y_limits, z_lim];
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
        // the bed pushed down on its load cells counts as a push into it
        self.push_mm[2] = self.push_mm[2].max(self.cfg.z_limits.0 - self.pos[2]);
    }

    /// Nozzle-to-bed gap at `t` (from the recent Z steps).
    fn gap_at(&self, t: f64) -> f64 {
        match self.z_hist.iter().rposition(|(ht, _)| *ht <= t) {
            Some(i) => self.z_hist[i].1,
            None => self.z_hist.front().map_or(self.pos[2], |(_, z)| *z),
        }
    }

    /// Filament fed by the extruder. Hot filament becomes melt in the
    /// nozzle, which flows out with a lag (`deposit`); a retraction pulls
    /// melt back.
    fn extrude(&mut self, de: f64, t: f64) {
        self.e_mm += de;
        if de < 0.0 {
            self.stats.retracted_mm -= de;
        } else if self.hotend.temp < 170.0 {
            self.stats.cold_extrusion_mm += de;
            return;
        }
        insert_by_time(&mut self.feed, (t, de));
    }

    /// Melt flow up to `FLOW_LAG` behind the newest event, in time order.
    fn flow(&mut self, t: f64) {
        self.latest_t = self.latest_t.max(t);
        let until = self.latest_t - FLOW_LAG;
        while let Some(&(ft, de)) = self.feed.front() {
            if ft > until {
                break;
            }
            self.feed.pop_front();
            self.deposit(ft);
            self.melt += de;
        }
        self.deposit(until);
    }

    /// Where the nozzle was at `t` (forgets what came before).
    fn nozzle_at(&mut self, t: f64) -> [f64; 3] {
        while let Some(&(ht, p)) = self.nozzle_hist.front() {
            if ht > t {
                break;
            }
            self.nozzle_hist.pop_front();
            self.hist_base = p;
        }
        self.hist_base
    }

    /// Melt flow up to time `t`: out at melt/τ (τ = MELT_TAU), laid down where
    /// the nozzle was. Pressure advance exists to cancel this lag.
    fn deposit(&mut self, t: f64) {
        let [x, y, z] = self.nozzle_at(t);
        let dt = (t - self.flow_t).max(0.0);
        self.flow_t = self.flow_t.max(t);
        if self.melt <= 0.0 || dt == 0.0 {
            return;
        }
        let amount = self.melt * (1.0 - (-dt / MELT_TAU).exp());
        self.melt -= amount;
        self.stats.extruded_mm += amount;
        // marked once 1e-4 mm has come out, however often this is called
        self.unmarked += amount;
        if self.unmarked < 1e-4 {
            return;
        }
        let de = std::mem::take(&mut self.unmarked);
        let bb = self.stats.bbox.get_or_insert([x, y, x, y]);
        bb[0] = bb[0].min(x);
        bb[1] = bb[1].min(y);
        bb[2] = bb[2].max(x);
        bb[3] = bb[3].max(y);
        let zq = (z * 100.0).round() / 100.0;
        if !self.stats.layers.contains(&zq) {
            let ph = &mut self.stats.per_height;
            let i = ph.iter().position(|(h, _)| *h == zq).unwrap_or_else(|| {
                ph.push((zq, 0.0));
                ph.len() - 1
            });
            ph[i].1 += de;
            if ph[i].1 >= LAYER_MIN_MM {
                self.stats.layers.push(zq);
            }
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
        self.flow(t);
        let w = MOTORS[i];
        let scale = CFG_MICROSTEPS / self.tmc[i].microsteps() as f64;
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
        m.travel += d as f64 * scale;
        if m.last_step_t > 0.0 {
            m.step_interval = t - m.last_step_t;
        }
        m.last_step_t = t;
        match i {
            0 | 1 => self.carriage_moved(0b011),
            2 => {
                let was = self.stalled[2];
                self.carriage_moved(0b100);
                if self.stalled[2] && !was && self.pos[2] <= self.cfg.z_limits.0 - BED_GIVE {
                    self.note_event(t, format!("nozzle pressed the bed down to its stop at x={:.1} y={:.1}", self.pos[0], self.pos[1]));
                }
                while self.z_hist.len() > 2 && self.z_hist[1].0 < t - 0.2 {
                    self.z_hist.pop_front();
                }
                self.z_hist.push_back((t, self.pos[2]));
            }
            _ => return self.extrude(d as f64 * scale * w.step_dist, t),
        }
        insert_by_time(&mut self.nozzle_hist, (t, self.pos));
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

    fn read_pin(&mut self, mcu: McuKind, pin: u32, t: f64) -> bool {
        match self.roles.get(&(mcu, pin)) {
            // DIAG of the A/B drivers through the board's inverter
            // (`diag_pin: !PG4`); only asserted with StallGuard set up.
            Some(Role::DiagX) => !(self.stalled[0] && self.tmc[0].diag(&self.live(0, t)) && !self.faults.contains(&Fault::XDiagDead)),
            Some(Role::DiagY) => !(self.stalled[1] && self.tmc[1].diag(&self.live(1, t))),
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

    fn load_cell(&mut self, mcu: McuKind, t: f64) -> i32 {
        if mcu != McuKind::LoadCell {
            return 0;
        }
        let depth = (self.cfg.z_limits.0 - self.gap_at(t)).max(0.0);
        // ~normal noise: sum of four uniforms
        let n: f64 = (0..4).map(|_| self.rand() - 0.5).sum::<f64>() * BRIDGE_NOISE * 3f64.sqrt();
        (BRIDGE_ZERO + n - LOAD_COUNTS_PER_MM * depth).round() as i32
    }

    fn uart(&mut self, mcu: McuKind, tx_pin: u32, rx_pin: u32, sent: &[u8], t: f64) -> Option<(Vec<u8>, u32)> {
        let ppb = mcu.pins_per_bank();
        let (_, _, _, nodes) = UARTS.iter().find(|u| u.0 == mcu && pin_number(u.1, ppb) == Some(tx_pin) && pin_number(u.2, ppb) == Some(rx_pin))?;
        let Some(bytes) = tmc::unframe(sent) else {
            self.note_event(t, format!("{}: TMC UART framing error", mcu.name()));
            return None;
        };
        let Some(req) = tmc::parse_request(&bytes) else {
            self.note_event(t, format!("{}: TMC datagram rejected (sync/CRC): {bytes:02x?}", mcu.name()));
            return None;
        };
        match req {
            tmc::Request::Write { addr, reg, val } => {
                match nodes.iter().find(|(a, _)| *a == addr) {
                    Some((_, m)) => self.tmc[*m].write(reg, val),
                    None => self.note_event(t, format!("{}: TMC write to absent node {addr}", mcu.name())),
                }
                None
            }
            tmc::Request::Read { addr, reg } => {
                let (_, m) = *nodes.iter().find(|(a, _)| *a == addr)?;
                let live = self.live(m, t);
                let v = self.tmc[m].read(reg, &live)?;
                Some((tmc::frame(&tmc::reply(reg, v)), self.tmc[m].senddelay_bits()))
            }
        }
    }
}
