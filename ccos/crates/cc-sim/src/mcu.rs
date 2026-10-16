//! A virtual Klipper MCU that behaves like Elegoo's firmware.
//!
//! Every command the Centauri Carbon host can send is decoded with the
//! printer's fixed message table, and the commands that matter for printing
//! are executed with the same rules as the C sources in this repository
//! (`dsp/.../klipper_r528/printer/stepper.c`, `trsync.c`, `hal_call/gpiocmds.c`,
//! `endstop.c`, `adccmds.c`, `basecmd.c`, `command.c`) — including the
//! shutdowns a real MCU raises when the host gets timing wrong:
//!
//! | Shutdown | When |
//! |---|---|
//! | `Timer too close` | a command schedules something in the MCU's past |
//! | `Stepper too far in past` | the next step move starts > 1 ms before the previous ended |
//! | `Move queue overflow` | more than `move_count` (1024) queued step/digital-out events |
//! | `Missed scheduling of next digital out event` | a heater's `max_duration` ran out with no update |
//! | `Scheduled digital out event will exceed max_duration` | update scheduled too late |
//! | `ADC out of range` | thermistor reading outside `min_value..max_value` |
//! | `Can't reset time when stepper active`, `Invalid count parameter`, … | as in the firmware |
//!
//! Time is a real number of seconds shared by the whole simulation (`t`);
//! each MCU's clock counts from its own boot. Events (steps, PWM loads, ADC
//! samples, endstop samples, trsync timers) fire at their exact clock, in
//! order, when [`VirtualMcu::advance`] runs.

use std::collections::VecDeque;

use cc_proto::dict::Arg;
use cc_proto::{encode_frame, Dictionary, FrameDecoder, MsgBuf};

/// Size of the firmware's move pool (`move_count` in `get_config`).
pub const MOVE_COUNT: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum McuKind {
    /// R528 HiFi4 DSP: X/Y/Z steppers, bed heater, thermistors, fans, light.
    Dsp,
    /// STM32 toolhead board: extruder, hotend heater/thermistor, fans.
    Toolhead,
    /// STM32 load-cell board under the bed.
    LoadCell,
}

impl McuKind {
    pub const ALL: [McuKind; 3] = [McuKind::Dsp, McuKind::Toolhead, McuKind::LoadCell];

    pub fn freq(self) -> f64 {
        match self {
            McuKind::Dsp => cc_proto::clock::DSP_CLOCK_FREQ,
            _ => cc_proto::clock::STM32_CLOCK_FREQ,
        }
    }

    pub fn pins_per_bank(self) -> u32 {
        if self == McuKind::Dsp { 32 } else { 16 }
    }

    pub fn name(self) -> &'static str {
        match self {
            McuKind::Dsp => "dsp",
            McuKind::Toolhead => "toolhead",
            McuKind::LoadCell => "loadcell",
        }
    }

    /// STM32 boards sit in Elegoo's YMODEM bootloader after power-up until
    /// the host sends `a` (see `mcu/bootloader/Utils/Src/ymodem.c`).
    pub fn has_bootloader(self) -> bool {
        self != McuKind::Dsp
    }
}

/// `PE8`-style name of a wire pin number.
pub fn pin_name(pin: u32, pins_per_bank: u32) -> String {
    format!("P{}{}", (b'A' + (pin / pins_per_bank) as u8) as char, pin % pins_per_bank)
}

/// What the MCU's pins are connected to (the physical printer).
pub trait Hardware {
    /// One step pulse on `step_pin`; `dir` is the level of the stepper's dir pin.
    fn step(&mut self, mcu: McuKind, step_pin: u32, dir: bool, t: f64);
    /// Output level (0..1 duty) of a digital/PWM output changed.
    fn output(&mut self, mcu: McuKind, pin: u32, duty: f64, t: f64);
    fn read_pin(&mut self, mcu: McuKind, pin: u32, t: f64) -> bool;
    /// One 12-bit ADC sample.
    fn read_adc(&mut self, mcu: McuKind, pin: u32, t: f64) -> u16;
    /// Diagnostics (shutdowns, protocol errors).
    fn note(&mut self, mcu: McuKind, t: f64, msg: &str);
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Power {
    Off,
    /// In Elegoo's bootloader: prints `C` once a second, waits for `a`.
    Bootloader { next_c: f64 },
    App,
}

struct StepMove {
    interval: u32,
    count: u16,
    add: i16,
    toggle_dir: bool,
}

struct Stepper {
    step_pin: u32,
    step_pulse_ticks: u32,
    /// Firmware `position` (biased, sign-flipped on direction changes).
    position: u32,
    last_dir: bool,
    next_dir: bool,
    need_reset: bool,
    queue: VecDeque<StepMove>,
    /// Steps left in the active move (0 = idle).
    remaining: u32,
    interval: u32,
    add: i16,
    /// Ideal time of the next step (accumulates interval/add exactly).
    next_step_time: u64,
    /// When the step timer actually fires (≥ previous step + step_pulse_ticks).
    wake: u64,
    /// Level of the dir pin (toggled when a move with a direction change loads).
    dir_level: bool,
    stop_trsync: Option<u8>,
}

const POSITION_BIAS: u32 = 0x4000_0000;

impl Stepper {
    fn get_position(&self) -> i32 {
        let mut p = self.position.wrapping_sub(self.remaining);
        if p & 0x8000_0000 != 0 {
            p = p.wrapping_neg();
        }
        p.wrapping_sub(POSITION_BIAS) as i32
    }
}

struct DigitalOut {
    pin: u32,
    default_on: bool,
    toggling: bool,
    check_end: bool,
    end_time: u64,
    max_duration: u32,
    cycle_ticks: u32,
    queue: VecDeque<(u64, u32)>,
    load_at: Option<u64>,
}

struct PwmOut {
    pin: u32,
    cycle_ticks: u32,
    max: f64,
    queue: VecDeque<(u64, u16)>,
}

struct Analog {
    pin: u32,
    wake: Option<u64>,
    rest_ticks: u32,
    sample_ticks: u32,
    next_begin: u64,
    sample_count: u8,
    state: u8,
    value: u16,
    min_value: u16,
    max_value: u16,
    range_check_count: u8,
    invalid_count: u8,
}

struct Endstop {
    pin: u32,
    wake: Option<u64>,
    oversampling: bool,
    sample_ticks: u32,
    rest_ticks: u32,
    nextwake: u64,
    sample_count: u8,
    trigger_count: u8,
    pin_high: bool,
    homing: bool,
    trsync: u8,
    reason: u8,
}

#[derive(Default)]
struct Trsync {
    can_trigger: bool,
    report: bool,
    trigger_reason: u8,
    expire_reason: u8,
    report_at: Option<u64>,
    report_ticks: u32,
    expire_at: Option<u64>,
    signals: Vec<u8>,
}

enum Obj {
    Unused,
    Stepper(Stepper),
    DigitalOut(DigitalOut),
    PwmOut(PwmOut),
    Analog(Analog),
    Endstop(Endstop),
    Trsync(Trsync),
    /// Load-cell, TMC UART, fan tach…: accepted, not simulated.
    Other,
}

/// Commands the STM32 firmware still runs while shut down (`HF_IN_SHUTDOWN`).
const IN_SHUTDOWN: &[&str] =
    &["identify", "get_config", "get_clock", "get_uptime", "emergency_stop", "clear_shutdown", "reset", "debug_ping", "debug_nop", "debug_read", "debug_write"];

pub struct VirtualMcu {
    pub kind: McuKind,
    freq: f64,
    dict: Dictionary,
    dec: FrameDecoder,
    next_seq: u8,
    out: Vec<u8>,
    pub power: Power,
    boot_t: f64,
    now: u64,
    objs: Vec<Obj>,
    oids_allocated: bool,
    finalized: bool,
    config_crc: u32,
    moves_used: usize,
    shutdown: Option<u16>,
    identify: Vec<u8>,
    /// Commands executed / frames NAKed (statistics).
    pub commands: u64,
    pub naks: u64,
    boots: u32,
    /// Command being executed (diagnostics).
    current: String,
}

impl VirtualMcu {
    /// A powered MCU with its firmware running (as left by a previous host).
    pub fn new(kind: McuKind, t: f64) -> Self {
        let identify = format!(
            "{{\"app\":\"ccsim\",\"mcu\":\"{}\",\"note\":\"virtual MCU; real Centauri firmware returns a stale identify blob\",\"CLOCK_FREQ\":{}}}",
            kind.name(),
            kind.freq()
        )
        .into_bytes();
        let mut m = Self {
            kind,
            freq: kind.freq(),
            dict: Dictionary::centauri(),
            dec: FrameDecoder::new(),
            next_seq: 0,
            out: Vec::with_capacity(4096),
            power: Power::App,
            boot_t: t,
            now: 0,
            objs: Vec::new(),
            oids_allocated: false,
            finalized: false,
            config_crc: 0,
            moves_used: 0,
            shutdown: None,
            identify,
            commands: 0,
            naks: 0,
            boots: 0,
            current: String::new(),
        };
        m.boot(t);
        m
    }

    fn boot(&mut self, t: f64) {
        self.boots += 1;
        self.power = Power::App;
        self.boot_t = t;
        self.now = 0;
        self.dec = FrameDecoder::new();
        self.next_seq = 0;
        self.objs.clear();
        self.oids_allocated = false;
        self.finalized = false;
        self.config_crc = 0;
        self.moves_used = 0;
        self.shutdown = None;
    }

    pub fn is_shutdown(&self) -> Option<&'static str> {
        self.shutdown.map(|id| self.dict.static_string(id).unwrap_or("?"))
    }

    pub fn is_configured(&self) -> bool {
        self.finalized
    }

    /// Move-pool entries in use (queued step moves and digital-out events).
    pub fn moves_used(&self) -> usize {
        self.moves_used
    }

    /// The MCU's clock at simulation time `t`.
    pub fn clock_at(&self, t: f64) -> u64 {
        ((t - self.boot_t).max(0.0) * self.freq) as u64
    }

    fn time_of(&self, clock: u64) -> f64 {
        self.boot_t + clock as f64 / self.freq
    }

    /// Extend a 32-bit clock from the host to 64 bits around `now`.
    fn clk(&self, c32: i64) -> u64 {
        cc_proto::clock::clock_from_clock32(self.now, c32 as u32)
    }

    /// Bytes for the host.
    pub fn take_output(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.out)
    }

    /// Power pin changed (STM32 boards only).
    pub fn set_power(&mut self, on: bool, t: f64, hw: &mut dyn Hardware) {
        match (on, self.power) {
            (false, Power::Off) => {}
            (false, _) => {
                if self.power == Power::App {
                    self.all_outputs_off(t, hw);
                }
                self.power = Power::Off;
                hw.note(self.kind, t, "power off");
            }
            (true, Power::Off) => {
                self.power = if self.kind.has_bootloader() { Power::Bootloader { next_c: t } } else { Power::App };
                if self.power == Power::App {
                    self.boot(t);
                }
                hw.note(self.kind, t, "power on: bootloader waiting for YMODEM or 'a'");
            }
            (true, _) => {}
        }
    }

    fn all_outputs_off(&mut self, t: f64, hw: &mut dyn Hardware) {
        for o in &self.objs {
            match o {
                Obj::DigitalOut(d) => hw.output(self.kind, d.pin, 0.0, t),
                Obj::PwmOut(p) => hw.output(self.kind, p.pin, 0.0, t),
                _ => {}
            }
        }
    }

    /// Bytes from the host at time `t`.
    pub fn receive(&mut self, data: &[u8], t: f64, hw: &mut dyn Hardware) {
        match self.power {
            Power::Off => return,
            Power::Bootloader { .. } => {
                if data.iter().any(|&b| b == b'a' || b == b'A') {
                    hw.note(self.kind, t, "bootloader: abort received, starting application");
                    self.boot(t);
                }
                return;
            }
            Power::App => {}
        }
        self.advance(t, hw);
        self.dec.push(data);
        while let Some(f) = self.dec.next_frame() {
            if f.seq != self.next_seq {
                // command_find_block: wrong sequence -> NAK with the expected one.
                self.naks += 1;
                self.frame(&[]);
                continue;
            }
            self.next_seq = (self.next_seq + 1) & 0x0f;
            let mut msgs = Vec::new();
            let mut bad = false;
            for m in self.dict.decode(f.payload.as_slice()) {
                match m {
                    Ok(m) => msgs.push(m.to_owned()),
                    Err(_) => {
                        bad = true;
                        break;
                    }
                }
            }
            let boots = self.boots;
            for m in msgs {
                self.commands += 1;
                self.dispatch(m.message(), hw);
                if self.boots != boots || self.power != Power::App {
                    return; // `reset`: the rest of the block is lost, no ack
                }
            }
            if bad {
                self.do_shutdown(5, hw); // Invalid command
            }
            self.frame(&[]); // ack
        }
    }

    fn frame(&mut self, payload: &[u8]) {
        let f = encode_frame(self.next_seq, payload).expect("response fits a frame");
        self.out.extend_from_slice(&f);
    }

    fn send(&mut self, name: &str, args: &[(&str, Arg<'_>)]) {
        let mut b = MsgBuf::new();
        self.dict.encode(name, args, &mut b).expect("response encodes");
        self.frame(b.as_slice());
    }

    fn do_shutdown(&mut self, reason: u16, hw: &mut dyn Hardware) {
        if self.shutdown.is_some() {
            return;
        }
        self.shutdown = Some(reason);
        let why = self.dict.static_string(reason).unwrap_or("?");
        let t = self.time_of(self.now);
        hw.note(self.kind, t, &format!("SHUTDOWN: {why}"));
        // Shutdown handlers: outputs to default, steppers stopped, all
        // timers and queued moves dropped.
        for i in 0..self.objs.len() {
            match &mut self.objs[i] {
                Obj::DigitalOut(d) => {
                    d.queue.clear();
                    d.load_at = None;
                    d.toggling = false;
                    d.check_end = false;
                    let (pin, on) = (d.pin, d.default_on);
                    hw.output(self.kind, pin, on as u8 as f64, t);
                }
                Obj::PwmOut(p) => {
                    p.queue.clear();
                    let pin = p.pin;
                    hw.output(self.kind, pin, 0.0, t);
                }
                Obj::Stepper(_) => self.stepper_stop(i as u8),
                Obj::Analog(a) => a.wake = None,
                Obj::Endstop(e) => {
                    e.wake = None;
                    e.homing = false;
                }
                Obj::Trsync(ts) => *ts = Trsync::default(),
                _ => {}
            }
        }
        self.moves_used = 0;
        self.send("shutdown", &[("clock", Arg::Int(self.now as u32 as i64)), ("static_string_id", Arg::Int(reason as i64))]);
    }

    fn shutdown_named(&mut self, s: &str, hw: &mut dyn Hardware) {
        let id = (0..64u16).find(|&i| self.dict.static_string(i) == Some(s)).unwrap_or(5);
        self.do_shutdown(id, hw);
    }

    fn move_alloc(&mut self, hw: &mut dyn Hardware) -> bool {
        if !self.finalized || self.moves_used >= MOVE_COUNT {
            self.shutdown_named("Move queue overflow", hw);
            return false;
        }
        self.moves_used += 1;
        true
    }

    fn move_free(&mut self) {
        self.moves_used = self.moves_used.saturating_sub(1);
    }

    /// sched_add_timer: scheduling into the past is fatal.
    fn add_timer(&mut self, waketime: u64, hw: &mut dyn Hardware) -> bool {
        if waketime < self.now {
            let late = (self.now - waketime) as f64 / self.freq * 1e3;
            let t = self.time_of(self.now);
            hw.note(self.kind, t, &format!("'{}' scheduled {late:.3} ms in the past", self.current));
            self.shutdown_named("Timer too close", hw);
            return false;
        }
        true
    }

    fn dispatch(&mut self, m: cc_proto::dict::Message<'_>, hw: &mut dyn Hardware) {
        let name = m.name();
        self.current = m.to_string();
        let a = |k: &str| m.int(k).unwrap_or(0);
        if self.shutdown.is_some() && !IN_SHUTDOWN.contains(&name) {
            let id = self.shutdown.unwrap();
            self.send("is_shutdown", &[("static_string_id", Arg::Int(id as i64))]);
            return;
        }
        let t = self.time_of(self.now);
        match name {
            "identify" => {
                let off = a("offset") as usize;
                let cnt = a("count") as usize;
                let end = (off + cnt).min(self.identify.len());
                let chunk = self.identify.get(off.min(end)..end).unwrap_or(&[]).to_vec();
                self.send("identify_response", &[("offset", Arg::Int(off as i64)), ("data", Arg::Bytes(&chunk))]);
            }
            "get_config" => {
                let args = [
                    ("is_config", Arg::Int(self.finalized as i64)),
                    ("crc", Arg::Int(self.config_crc as i64)),
                    ("move_count", Arg::Int(if self.finalized { MOVE_COUNT as i64 } else { 0 })),
                    ("is_shutdown", Arg::Int(self.shutdown.is_some() as i64)),
                ];
                self.send("config", &args);
            }
            "get_clock" => self.send("clock", &[("clock", Arg::Int(self.now as u32 as i64))]),
            "get_uptime" => {
                self.send("uptime", &[("high", Arg::Int((self.now >> 32) as i64)), ("clock", Arg::Int(self.now as u32 as i64))])
            }
            "allocate_oids" => {
                if self.oids_allocated {
                    self.shutdown_named("oids already allocated", hw);
                    return;
                }
                self.oids_allocated = true;
                self.objs = (0..a("count")).map(|_| Obj::Unused).collect();
            }
            "finalize_config" => {
                if self.finalized {
                    self.shutdown_named("Already finalized", hw);
                    return;
                }
                self.finalized = true;
                self.config_crc = a("crc") as u32;
            }
            "emergency_stop" => self.do_shutdown(8, hw), // "Command request"
            "clear_shutdown" => {
                if self.shutdown.is_none() {
                    self.shutdown_named("Shutdown cleared when not shutdown", hw);
                } else {
                    self.shutdown = None;
                }
            }
            "reset" => {
                // armcm_reset: NVIC reset. STM32 boards land in the bootloader again.
                self.all_outputs_off(t, hw);
                if self.kind.has_bootloader() {
                    self.power = Power::Bootloader { next_c: t };
                    hw.note(self.kind, t, "reset: bootloader waiting for YMODEM or 'a'");
                } else {
                    self.boot(t);
                    hw.note(self.kind, t, "reset: firmware restarted");
                }
            }
            "debug_ping" => {
                let d = m.bytes("data").unwrap_or(&[]).to_vec();
                self.send("pong", &[("data", Arg::Bytes(&d))]);
            }
            "debug_nop" | "debug_read" | "debug_write" => {}

            "config_stepper" => {
                let s = Stepper {
                    step_pin: a("step_pin") as u32,
                    step_pulse_ticks: a("step_pulse_ticks") as u32,
                    position: POSITION_BIAS.wrapping_neg(),
                    last_dir: false,
                    next_dir: false,
                    need_reset: false,
                    queue: VecDeque::new(),
                    remaining: 0,
                    interval: 0,
                    add: 0,
                    next_step_time: 0,
                    wake: 0,
                    dir_level: false,
                    stop_trsync: None,
                };
                self.set_obj(a("oid"), Obj::Stepper(s), hw);
            }
            "queue_step" => self.queue_step(a("oid") as u8, a("interval") as u32, a("count") as u16, a("add") as i16, hw),
            "set_next_step_dir" => {
                if let Some(Obj::Stepper(s)) = self.objs.get_mut(a("oid") as usize) {
                    s.next_dir = a("dir") != 0;
                }
            }
            "reset_step_clock" => {
                let clock = self.clk(a("clock"));
                let active = matches!(self.objs.get(a("oid") as usize), Some(Obj::Stepper(s)) if s.remaining > 0);
                if active {
                    self.shutdown_named("Can't reset time when stepper active", hw);
                    return;
                }
                if let Some(Obj::Stepper(s)) = self.objs.get_mut(a("oid") as usize) {
                    s.next_step_time = clock;
                    s.need_reset = false;
                }
            }
            "stepper_get_position" => {
                let oid = a("oid");
                if let Some(Obj::Stepper(s)) = self.objs.get(oid as usize) {
                    let pos = s.get_position();
                    self.send("stepper_position", &[("oid", Arg::Int(oid)), ("pos", Arg::Int(pos as i64))]);
                }
            }
            "stepper_stop_on_trigger" => {
                let (oid, ts) = (a("oid") as u8, a("trsync_oid") as u8);
                let busy = match self.objs.get(oid as usize) {
                    Some(Obj::Stepper(s)) => s.stop_trsync.is_some(),
                    _ => return,
                };
                if busy {
                    self.shutdown_named("Can't add signal that is already active", hw);
                    return;
                }
                if let Some(Obj::Trsync(t)) = self.objs.get_mut(ts as usize) {
                    t.signals.push(oid);
                } else {
                    return;
                }
                if let Some(Obj::Stepper(s)) = self.objs.get_mut(oid as usize) {
                    s.stop_trsync = Some(ts);
                }
            }

            "config_digital_out" => {
                let (pin, value) = (a("pin") as u32, a("value") != 0);
                hw.output(self.kind, pin, value as u8 as f64, t);
                let d = DigitalOut {
                    pin,
                    default_on: a("default_value") != 0,
                    toggling: false,
                    check_end: false,
                    end_time: 0,
                    max_duration: a("max_duration") as u32,
                    cycle_ticks: 0,
                    queue: VecDeque::new(),
                    load_at: None,
                };
                self.set_obj(a("oid"), Obj::DigitalOut(d), hw);
            }
            "set_digital_out_pwm_cycle" => {
                let pending = matches!(self.objs.get(a("oid") as usize), Some(Obj::DigitalOut(d)) if !d.queue.is_empty());
                if pending {
                    self.shutdown_named("Can not set soft pwm cycle ticks while updates pending", hw);
                    return;
                }
                if let Some(Obj::DigitalOut(d)) = self.objs.get_mut(a("oid") as usize) {
                    d.cycle_ticks = a("cycle_ticks") as u32;
                }
            }
            "queue_digital_out" => self.queue_digital_out(a("oid") as usize, self.clk(a("clock")), a("on_ticks") as u32, hw),
            "update_digital_out" => {
                let oid = a("oid") as usize;
                let now = self.now;
                let Some(Obj::DigitalOut(d)) = self.objs.get_mut(oid) else { return };
                d.load_at = None;
                if !d.queue.is_empty() {
                    self.shutdown_named("update_digital_out not valid with active queue", hw);
                    return;
                }
                let on = a("value") != 0;
                d.toggling = false;
                if on != d.default_on && d.max_duration > 0 {
                    d.end_time = now + d.max_duration as u64;
                    d.load_at = Some(d.end_time);
                    d.check_end = true;
                } else {
                    d.check_end = false;
                }
                let pin = d.pin;
                hw.output(self.kind, pin, on as u8 as f64, t);
            }
            "set_digital_out" => hw.output(self.kind, a("pin") as u32, (a("value") != 0) as u8 as f64, t),

            "config_pwm_out" => {
                let max = if self.kind == McuKind::Dsp { cc_proto::clock::DSP_PWM_MAX } else { cc_proto::clock::STM32_PWM_MAX } as f64;
                let p = PwmOut { pin: a("pin") as u32, cycle_ticks: a("cycle_ticks") as u32, max, queue: VecDeque::new() };
                hw.output(self.kind, p.pin, a("value") as f64 / max, t);
                let _ = p.cycle_ticks;
                self.set_obj(a("oid"), Obj::PwmOut(p), hw);
            }
            "queue_pwm_out" => {
                let (oid, clock, value) = (a("oid") as usize, self.clk(a("clock")), a("value") as u16);
                if !matches!(self.objs.get(oid), Some(Obj::PwmOut(_))) {
                    return;
                }
                if !self.move_alloc(hw) || !self.add_timer(clock, hw) {
                    return;
                }
                if let Some(Obj::PwmOut(p)) = self.objs.get_mut(oid) {
                    p.queue.push_back((clock, value));
                }
            }
            "set_pwm_out" => {
                let max = if self.kind == McuKind::Dsp { cc_proto::clock::DSP_PWM_MAX } else { cc_proto::clock::STM32_PWM_MAX } as f64;
                hw.output(self.kind, a("pin") as u32, a("value") as f64 / max, t);
            }

            "config_analog_in" => {
                let pin = a("pin") as u32;
                // DSP: gpio_adc_setup(pin - (32+13)) accepts PB13..PB15 only.
                if self.kind == McuKind::Dsp && !(45..=47).contains(&pin) {
                    self.shutdown_named("ADC out of range", hw);
                }
                let an = Analog {
                    pin,
                    wake: None,
                    rest_ticks: 0,
                    sample_ticks: 0,
                    next_begin: 0,
                    sample_count: 0,
                    state: 1,
                    value: 0,
                    min_value: 0,
                    max_value: 0,
                    range_check_count: 0,
                    invalid_count: 0,
                };
                self.set_obj(a("oid"), Obj::Analog(an), hw);
            }
            "query_analog_in" => {
                let clock = self.clk(a("clock"));
                let oid = a("oid") as usize;
                let Some(Obj::Analog(an)) = self.objs.get_mut(oid) else { return };
                an.next_begin = clock;
                an.sample_ticks = a("sample_ticks") as u32;
                an.sample_count = a("sample_count") as u8;
                an.state = an.sample_count.wrapping_add(1);
                an.rest_ticks = a("rest_ticks") as u32;
                an.min_value = a("min_value") as u16;
                an.max_value = a("max_value") as u16;
                an.range_check_count = a("range_check_count") as u8;
                an.wake = None;
                if an.sample_count == 0 {
                    return;
                }
                if self.add_timer(clock, hw) {
                    if let Some(Obj::Analog(an)) = self.objs.get_mut(oid) {
                        an.wake = Some(clock);
                    }
                }
            }

            "config_endstop" => {
                let e = Endstop {
                    pin: a("pin") as u32,
                    wake: None,
                    oversampling: false,
                    sample_ticks: 0,
                    rest_ticks: 0,
                    nextwake: 0,
                    sample_count: 0,
                    trigger_count: 0,
                    pin_high: false,
                    homing: false,
                    trsync: 0,
                    reason: 0,
                };
                self.set_obj(a("oid"), Obj::Endstop(e), hw);
            }
            "endstop_home" => {
                let clock = self.clk(a("clock"));
                let oid = a("oid") as usize;
                let Some(Obj::Endstop(e)) = self.objs.get_mut(oid) else { return };
                e.wake = None;
                e.sample_ticks = a("sample_ticks") as u32;
                e.sample_count = a("sample_count") as u8;
                if e.sample_count == 0 {
                    e.homing = false;
                    e.pin_high = false;
                    return;
                }
                e.rest_ticks = a("rest_ticks") as u32;
                e.oversampling = false;
                e.trigger_count = e.sample_count;
                e.homing = true;
                e.pin_high = a("pin_value") != 0;
                e.trsync = a("trsync_oid") as u8;
                e.reason = a("trigger_reason") as u8;
                if self.add_timer(clock, hw) {
                    if let Some(Obj::Endstop(e)) = self.objs.get_mut(oid) {
                        e.wake = Some(clock);
                    }
                }
            }
            "endstop_query_state" => {
                let oid = a("oid");
                if let Some(Obj::Endstop(e)) = self.objs.get(oid as usize) {
                    let (homing, next, pin) = (e.homing, e.nextwake, e.pin);
                    let v = hw.read_pin(self.kind, pin, t);
                    self.send(
                        "endstop_state",
                        &[("oid", Arg::Int(oid)), ("homing", Arg::Int(homing as i64)), ("next_clock", Arg::Int(next as u32 as i64)), ("pin_value", Arg::Int(v as i64))],
                    );
                }
            }

            "config_trsync" => self.set_obj(a("oid"), Obj::Trsync(Trsync::default()), hw),
            "trsync_start" => {
                let oid = a("oid") as usize;
                self.trsync_clear(oid);
                let report_clock = self.clk(a("report_clock"));
                let report_ticks = a("report_ticks") as u32;
                let Some(Obj::Trsync(ts)) = self.objs.get_mut(oid) else { return };
                ts.can_trigger = true;
                ts.report_ticks = report_ticks;
                ts.expire_reason = a("expire_reason") as u8;
                if report_ticks > 0 && self.add_timer(report_clock, hw) {
                    if let Some(Obj::Trsync(ts)) = self.objs.get_mut(oid) {
                        ts.report_at = Some(report_clock);
                    }
                }
            }
            "trsync_set_timeout" => {
                let oid = a("oid") as usize;
                let clock = self.clk(a("clock"));
                let can = matches!(self.objs.get(oid), Some(Obj::Trsync(ts)) if ts.can_trigger);
                if can && self.add_timer(clock, hw) {
                    if let Some(Obj::Trsync(ts)) = self.objs.get_mut(oid) {
                        ts.expire_at = Some(clock);
                    }
                }
            }
            "trsync_trigger" => {
                let oid = a("oid") as usize;
                self.trsync_do_trigger(oid, a("reason") as u8);
                if let Some(Obj::Trsync(ts)) = self.objs.get_mut(oid) {
                    ts.report_at = None;
                    ts.expire_at = None;
                    ts.can_trigger = false;
                    ts.report = false;
                    let r = ts.trigger_reason;
                    self.send(
                        "trsync_state",
                        &[("oid", Arg::Int(oid as i64)), ("can_trigger", Arg::Int(0)), ("trigger_reason", Arg::Int(r as i64)), ("clock", Arg::Int(0))],
                    );
                }
            }

            // Accepted but not simulated: TMC UART, load-cell ADCs, fan tachs,
            // accelerometers, buttons, displays, direct Z control.
            n if n.starts_with("config_") => self.set_obj(a("oid"), Obj::Other, hw),
            _ => {}
        }
    }

    fn set_obj(&mut self, oid: i64, o: Obj, hw: &mut dyn Hardware) {
        match self.objs.get_mut(oid as usize) {
            Some(slot @ Obj::Unused) => *slot = o,
            Some(_) => self.shutdown_named("Can't assign oid", hw),
            None => self.shutdown_named("Can't assign oid", hw),
        }
    }

    // ---- steppers (stepper.c) ----

    fn queue_step(&mut self, oid: u8, interval: u32, count: u16, add: i16, hw: &mut dyn Hardware) {
        if !matches!(self.objs.get(oid as usize), Some(Obj::Stepper(_))) {
            return;
        }
        if !self.move_alloc(hw) {
            return;
        }
        if count == 0 {
            self.shutdown_named("Invalid count parameter", hw);
            return;
        }
        let Some(Obj::Stepper(s)) = self.objs.get_mut(oid as usize) else { unreachable!() };
        let toggle_dir = s.last_dir != s.next_dir;
        let m = StepMove { interval, count, add, toggle_dir };
        if s.remaining > 0 {
            if toggle_dir {
                s.last_dir = !s.last_dir;
            }
            s.queue.push_back(m);
        } else if s.need_reset {
            self.move_free();
        } else {
            if toggle_dir {
                s.last_dir = !s.last_dir;
            }
            s.queue.push_back(m);
            self.stepper_load_next(oid, None, hw);
            let wake = match &self.objs[oid as usize] {
                Obj::Stepper(s) => s.wake,
                _ => unreachable!(),
            };
            self.add_timer(wake, hw);
        }
    }

    /// Pop the next move. `curtime` is set when called from a step event
    /// (enables the "too far in past" check).
    fn stepper_load_next(&mut self, oid: u8, curtime: Option<u64>, hw: &mut dyn Hardware) {
        let Some(Obj::Stepper(s)) = self.objs.get_mut(oid as usize) else { return };
        let Some(m) = s.queue.pop_front() else {
            s.remaining = 0;
            return;
        };
        s.add = m.add;
        s.interval = m.interval.wrapping_add(m.add as i32 as u32);
        s.next_step_time = s.next_step_time.wrapping_add(m.interval as u64);
        s.wake = s.next_step_time;
        s.remaining = m.count as u32;
        if m.toggle_dir {
            s.position = s.position.wrapping_neg().wrapping_add(m.count as u32);
            s.dir_level = !s.dir_level;
        } else {
            s.position = s.position.wrapping_add(m.count as u32);
        }
        let (wake, pulse) = (s.next_step_time, s.step_pulse_ticks as u64);
        self.move_free();
        if let Some(cur) = curtime {
            let min_next = cur + pulse;
            if wake < min_next && (min_next - wake) as f64 > self.freq * 0.001 {
                self.shutdown_named("Stepper too far in past", hw);
            }
        }
    }

    fn stepper_event(&mut self, oid: u8, hw: &mut dyn Hardware) {
        let now = self.now;
        let t = self.time_of(now);
        let Some(Obj::Stepper(s)) = self.objs.get_mut(oid as usize) else { return };
        hw.step(self.kind, s.step_pin, s.dir_level, t);
        let min_next = now + s.step_pulse_ticks as u64;
        s.remaining -= 1;
        if s.remaining > 0 {
            s.next_step_time = s.next_step_time.wrapping_add(s.interval as u64);
            s.interval = s.interval.wrapping_add(s.add as i32 as u32);
            // stepper_event_full: never closer than step_pulse_ticks.
            s.wake = s.next_step_time.max(min_next);
            return;
        }
        self.stepper_load_next(oid, Some(now), hw);
        if let Some(Obj::Stepper(s)) = self.objs.get_mut(oid as usize) {
            s.wake = s.wake.max(min_next);
        }
    }

    fn stepper_stop(&mut self, oid: u8) {
        let mut freed = 0;
        if let Some(Obj::Stepper(s)) = self.objs.get_mut(oid as usize) {
            let pos = s.get_position();
            s.position = (pos as u32).wrapping_add(POSITION_BIAS).wrapping_neg();
            s.next_step_time = 0;
            s.wake = 0;
            s.remaining = 0;
            s.dir_level = false;
            s.need_reset = true;
            s.last_dir = false;
            s.next_dir = false;
            s.stop_trsync = None;
            freed = s.queue.len();
            s.queue.clear();
        }
        for _ in 0..freed {
            self.move_free();
        }
    }

    // ---- digital out (gpiocmds.c) ----

    fn queue_digital_out(&mut self, oid: usize, time: u64, on: u32, hw: &mut dyn Hardware) {
        if !matches!(self.objs.get(oid), Some(Obj::DigitalOut(_))) {
            return;
        }
        if !self.move_alloc(hw) {
            return;
        }
        let Some(Obj::DigitalOut(d)) = self.objs.get_mut(oid) else { unreachable!() };
        d.queue.push_back((time, on));
        if d.queue.len() > 1 {
            return;
        }
        if d.check_end && d.end_time < time {
            self.shutdown_named("Scheduled digital out event will exceed max_duration", hw);
            return;
        }
        let toggling = d.toggling;
        d.end_time = time;
        d.check_end = true;
        d.load_at = Some(time);
        if !toggling {
            self.add_timer(time, hw);
        }
    }

    fn digital_load_event(&mut self, oid: usize, hw: &mut dyn Hardware) {
        let now = self.now;
        let t = self.time_of(now);
        let Some(Obj::DigitalOut(d)) = self.objs.get_mut(oid) else { return };
        let Some((_, on)) = d.queue.pop_front() else {
            d.load_at = None;
            self.shutdown_named("Missed scheduling of next digital out event", hw);
            return;
        };
        let pin = d.pin;
        let full = on >= d.cycle_ticks;
        let duty = if on == 0 { 0.0 } else if full { 1.0 } else { on as f64 / d.cycle_ticks as f64 };
        let mut check_end = false;
        let mut end_time = 0;
        let toggling = on != 0 && !full;
        if !toggling {
            if (on == 0) != !d.default_on && d.max_duration > 0 {
                end_time = now + d.max_duration as u64;
                check_end = true;
            }
        } else if d.max_duration > 0 {
            end_time = now + d.max_duration as u64;
            check_end = true;
        }
        let mut overrun = false;
        if let Some(&(wake, _)) = d.queue.front() {
            if check_end && end_time < wake {
                overrun = true;
            }
            end_time = wake;
            check_end = true;
        }
        d.toggling = toggling;
        d.check_end = check_end;
        d.end_time = end_time;
        d.load_at = check_end.then_some(end_time);
        self.move_free();
        hw.output(self.kind, pin, duty, t);
        if overrun {
            self.shutdown_named("Scheduled digital out event will exceed max_duration", hw);
        }
    }

    // ---- trsync / endstop ----

    fn trsync_clear(&mut self, oid: usize) {
        let sigs = match self.objs.get_mut(oid) {
            Some(Obj::Trsync(ts)) => {
                let s = std::mem::take(&mut ts.signals);
                *ts = Trsync::default();
                s
            }
            _ => return,
        };
        for s in sigs {
            if let Some(Obj::Stepper(st)) = self.objs.get_mut(s as usize) {
                st.stop_trsync = None;
            }
        }
    }

    fn trsync_do_trigger(&mut self, oid: usize, reason: u8) {
        let sigs = match self.objs.get_mut(oid) {
            Some(Obj::Trsync(ts)) if ts.can_trigger => {
                ts.trigger_reason = reason;
                ts.can_trigger = false;
                ts.report = true;
                std::mem::take(&mut ts.signals)
            }
            _ => return,
        };
        for s in sigs {
            self.stepper_stop(s);
        }
    }

    fn endstop_event(&mut self, oid: usize, hw: &mut dyn Hardware) {
        let t = self.time_of(self.now);
        let Some(Obj::Endstop(e)) = self.objs.get(oid) else { return };
        let pin = e.pin;
        let val = hw.read_pin(self.kind, pin, t);
        let Some(Obj::Endstop(e)) = self.objs.get_mut(oid) else { return };
        let w = e.wake.unwrap_or(self.now);
        let not_triggered = val != e.pin_high;
        if !e.oversampling {
            let next = w + e.rest_ticks as u64;
            if not_triggered {
                e.wake = Some(next);
                return;
            }
            e.nextwake = next;
            e.oversampling = true;
        } else if not_triggered {
            e.oversampling = false;
            e.wake = Some(e.nextwake);
            e.trigger_count = e.sample_count;
            return;
        }
        // oversample
        let count = e.trigger_count.wrapping_sub(1);
        if count == 0 {
            e.wake = None;
            let (ts, reason) = (e.trsync as usize, e.reason);
            self.trsync_do_trigger(ts, reason);
            return;
        }
        e.trigger_count = count;
        e.wake = Some(w + e.sample_ticks as u64);
    }

    fn analog_event(&mut self, oid: usize, hw: &mut dyn Hardware) {
        let t = self.time_of(self.now);
        let Some(Obj::Analog(an)) = self.objs.get(oid) else { return };
        let sample = hw.read_adc(self.kind, an.pin, t);
        let Some(Obj::Analog(an)) = self.objs.get_mut(oid) else { return };
        let w = an.wake.unwrap_or(self.now);
        let mut value = sample;
        let mut state = an.state;
        if state >= an.sample_count {
            state = 0;
        } else {
            value = value.wrapping_add(an.value);
        }
        an.value = value;
        an.state = state + 1;
        if an.state < an.sample_count {
            an.wake = Some(w + an.sample_ticks as u64);
            return;
        }
        let mut fail = false;
        if an.value >= an.min_value && an.value <= an.max_value {
            an.invalid_count = 0;
        } else {
            an.invalid_count += 1;
            if an.invalid_count >= an.range_check_count {
                fail = true;
                an.invalid_count = 0;
            }
        }
        an.next_begin += an.rest_ticks as u64;
        an.wake = Some(an.next_begin);
        let (v, next) = (an.value, an.next_begin);
        // analog_in_task reports the completed sum.
        an.state += 1;
        self.send("analog_in_state", &[("oid", Arg::Int(oid as i64)), ("next_clock", Arg::Int(next as u32 as i64)), ("value", Arg::Int(v as i64))]);
        if fail {
            self.shutdown_named("ADC out of range", hw);
        }
    }

    fn next_event(&self, limit: u64) -> Option<(u64, usize, u8)> {
        let mut best: Option<(u64, usize, u8)> = None;
        let mut consider = |w: u64, oid: usize, k: u8| {
            if w <= limit && best.is_none_or(|(bw, _, _)| w < bw) {
                best = Some((w, oid, k));
            }
        };
        for (oid, o) in self.objs.iter().enumerate() {
            match o {
                Obj::Stepper(s) if s.remaining > 0 => consider(s.wake, oid, 0),
                Obj::DigitalOut(d) => {
                    if let Some(w) = d.load_at {
                        consider(w, oid, 0)
                    }
                }
                Obj::PwmOut(p) => {
                    if let Some(&(w, _)) = p.queue.front() {
                        consider(w, oid, 0)
                    }
                }
                Obj::Analog(a) => {
                    if let Some(w) = a.wake {
                        consider(w, oid, 0)
                    }
                }
                Obj::Endstop(e) => {
                    if let Some(w) = e.wake {
                        consider(w, oid, 0)
                    }
                }
                Obj::Trsync(ts) => {
                    if let Some(w) = ts.report_at {
                        consider(w, oid, 1)
                    }
                    if let Some(w) = ts.expire_at {
                        consider(w, oid, 2)
                    }
                }
                _ => {}
            }
        }
        best
    }

    /// Run every timer due up to time `t`.
    pub fn advance(&mut self, t: f64, hw: &mut dyn Hardware) {
        match self.power {
            Power::Off => return,
            Power::Bootloader { mut next_c } => {
                while t >= next_c {
                    self.out.push(b'C');
                    next_c += 1.0;
                }
                self.power = Power::Bootloader { next_c };
                return;
            }
            Power::App => {}
        }
        let end = self.clock_at(t).max(self.now);
        if self.shutdown.is_none() {
            while let Some((w, oid, k)) = self.next_event(end) {
                self.now = w.max(self.now);
                match &self.objs[oid] {
                    Obj::Stepper(_) => self.stepper_event(oid as u8, hw),
                    Obj::DigitalOut(_) => self.digital_load_event(oid, hw),
                    Obj::PwmOut(p) => {
                        let tt = self.time_of(self.now);
                        let (pin, max) = (p.pin, p.max);
                        if let Some(Obj::PwmOut(p)) = self.objs.get_mut(oid) {
                            let (_, v) = p.queue.pop_front().unwrap();
                            self.move_free();
                            hw.output(self.kind, pin, v as f64 / max, tt);
                        }
                    }
                    Obj::Analog(_) => self.analog_event(oid, hw),
                    Obj::Endstop(_) => self.endstop_event(oid, hw),
                    Obj::Trsync(_) => {
                        if k == 1 {
                            if let Some(Obj::Trsync(ts)) = self.objs.get_mut(oid) {
                                ts.report = true;
                                ts.report_at = ts.report_at.map(|w| w + ts.report_ticks as u64);
                            }
                        } else {
                            let reason = match &self.objs[oid] {
                                Obj::Trsync(ts) => ts.expire_reason,
                                _ => 0,
                            };
                            if let Some(Obj::Trsync(ts)) = self.objs.get_mut(oid) {
                                ts.expire_at = None;
                            }
                            self.trsync_do_trigger(oid, reason);
                        }
                    }
                    _ => {}
                }
                self.trsync_task();
                if self.shutdown.is_some() {
                    break;
                }
            }
        }
        self.now = end;
    }

    fn trsync_task(&mut self) {
        for oid in 0..self.objs.len() {
            let Obj::Trsync(ts) = &mut self.objs[oid] else { continue };
            if !ts.report {
                continue;
            }
            ts.report = false;
            if !ts.can_trigger {
                ts.report_at = None;
            }
            let (can, reason, clock) = (ts.can_trigger, ts.trigger_reason, ts.report_at.unwrap_or(self.now));
            self.send(
                "trsync_state",
                &[
                    ("oid", Arg::Int(oid as i64)),
                    ("can_trigger", Arg::Int(can as i64)),
                    ("trigger_reason", Arg::Int(reason as i64)),
                    ("clock", Arg::Int(clock as u32 as i64)),
                ],
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Rec {
        steps: Vec<(u32, bool, f64)>,
        outputs: Vec<(u32, f64, f64)>,
        pin_after: Option<f64>,
        notes: Vec<String>,
    }

    impl Hardware for Rec {
        fn step(&mut self, _: McuKind, pin: u32, dir: bool, t: f64) {
            self.steps.push((pin, dir, t));
        }
        fn output(&mut self, _: McuKind, pin: u32, duty: f64, t: f64) {
            self.outputs.push((pin, duty, t));
        }
        fn read_pin(&mut self, _: McuKind, _: u32, t: f64) -> bool {
            self.pin_after.is_some_and(|a| t >= a)
        }
        fn read_adc(&mut self, _: McuKind, _: u32, _: f64) -> u16 {
            1000
        }
        fn note(&mut self, _: McuKind, _: f64, msg: &str) {
            self.notes.push(msg.to_string());
        }
    }

    /// Host side of a test: frames commands, decodes replies.
    struct Host {
        seq: u8,
        dec: FrameDecoder,
        dict: Dictionary,
    }

    impl Host {
        fn new() -> Self {
            Self { seq: 0, dec: FrameDecoder::new(), dict: Dictionary::centauri() }
        }
        fn send(&mut self, m: &mut VirtualMcu, hw: &mut Rec, t: f64, cmds: &[&str]) -> Vec<String> {
            let mut p = MsgBuf::new();
            for c in cmds {
                self.dict.encode_str(c, &mut p).unwrap();
            }
            let f = encode_frame(self.seq, p.as_slice()).unwrap();
            self.seq = (self.seq + 1) & 15;
            m.receive(&f, t, hw);
            self.replies(m)
        }
        fn replies(&mut self, m: &mut VirtualMcu) -> Vec<String> {
            self.dec.push(&m.take_output());
            let mut v = Vec::new();
            while let Some(f) = self.dec.next_frame() {
                for msg in self.dict.decode(f.payload.as_slice()) {
                    v.push(msg.unwrap().to_string());
                }
            }
            v
        }
    }

    const F: f64 = cc_proto::clock::DSP_CLOCK_FREQ;

    fn configured(cmds: &[&str]) -> (VirtualMcu, Rec, Host) {
        let mut m = VirtualMcu::new(McuKind::Dsp, 0.0);
        let mut hw = Rec::default();
        let mut h = Host::new();
        h.send(&mut m, &mut hw, 0.001, &["allocate_oids count=4"]);
        for c in cmds {
            h.send(&mut m, &mut hw, 0.001, &[c]);
        }
        h.send(&mut m, &mut hw, 0.001, &["finalize_config crc=7"]);
        (m, hw, h)
    }

    #[test]
    fn queue_step_matches_firmware_formula() {
        let (mut m, mut hw, mut h) = configured(&["config_stepper oid=0 step_pin=136 dir_pin=135 invert_step=0 step_pulse_ticks=40"]);
        let c0 = (0.1 * F) as u64;
        h.send(&mut m, &mut hw, 0.01, &["reset_step_clock oid=0 clock=20000000", "set_next_step_dir oid=0 dir=1", "queue_step oid=0 interval=1000 count=5 add=10"]);
        assert_eq!(c0, 20_000_000);
        m.advance(0.2, &mut hw);
        let clocks: Vec<u64> = hw.steps.iter().map(|s| (s.2 * F).round() as u64).collect();
        let want: Vec<u64> = (1..=5u64).map(|k| c0 + k * 1000 + 10 * k * (k - 1) / 2).collect();
        assert_eq!(clocks, want);
        assert!(hw.steps.iter().all(|s| s.1), "dir pin high");
        let r = h.send(&mut m, &mut hw, 0.2, &["stepper_get_position oid=0"]);
        assert_eq!(r, ["stepper_position oid=0 pos=5"]);
        // reverse 2 steps
        h.send(&mut m, &mut hw, 0.2, &["reset_step_clock oid=0 clock=50000000", "set_next_step_dir oid=0 dir=0", "queue_step oid=0 interval=100000 count=2 add=0"]);
        m.advance(0.3, &mut hw);
        assert_eq!(hw.steps.len(), 7);
        assert!(!hw.steps[6].1);
        let r = h.send(&mut m, &mut hw, 0.3, &["stepper_get_position oid=0"]);
        assert_eq!(r, ["stepper_position oid=0 pos=3"]);
        assert!(m.is_shutdown().is_none());
        assert_eq!(m.moves_used(), 0);
    }

    #[test]
    fn late_step_is_timer_too_close() {
        let (mut m, mut hw, mut h) = configured(&["config_stepper oid=0 step_pin=136 dir_pin=135 invert_step=0 step_pulse_ticks=40"]);
        let r = h.send(&mut m, &mut hw, 0.5, &["reset_step_clock oid=0 clock=1000", "queue_step oid=0 interval=1000 count=5 add=0"]);
        assert_eq!(m.is_shutdown(), Some("Timer too close"));
        assert!(r[0].starts_with("shutdown clock="), "{r:?}");
        // commands are refused afterwards, queries still work
        let r = h.send(&mut m, &mut hw, 0.6, &["queue_step oid=0 interval=1000 count=5 add=0", "get_config"]);
        assert_eq!(r[0], "is_shutdown static_string_id=3");
        assert!(r[1].ends_with("is_shutdown=1"));
    }

    #[test]
    fn wrong_sequence_is_naked() {
        let mut m = VirtualMcu::new(McuKind::Toolhead, 0.0);
        let mut hw = Rec::default();
        let f = encode_frame(5, &[]).unwrap();
        m.receive(&f, 0.0, &mut hw);
        let mut d = FrameDecoder::new();
        d.push(&m.take_output());
        let fr = d.next_frame().unwrap();
        assert_eq!(fr.seq, 0, "NAK carries the expected sequence");
        assert_eq!(m.naks, 1);
    }

    #[test]
    fn heater_without_updates_shuts_down() {
        // 3 s max_duration, 0.1 s soft PWM, 30% duty, then no further updates.
        let (mut m, mut hw, mut h) = configured(&["config_digital_out oid=0 pin=205 value=0 default_value=0 max_duration=600000000"]);
        h.send(&mut m, &mut hw, 0.01, &["set_digital_out_pwm_cycle oid=0 cycle_ticks=20000000", "queue_digital_out oid=0 clock=40000000 on_ticks=6000000"]);
        m.advance(1.0, &mut hw);
        assert!((hw.outputs.last().unwrap().1 - 0.3).abs() < 1e-9);
        assert!(m.is_shutdown().is_none());
        m.advance(3.3, &mut hw);
        assert_eq!(m.is_shutdown(), Some("Missed scheduling of next digital out event"));
        assert_eq!(hw.outputs.last().unwrap().1, 0.0, "heater forced off");
    }

    #[test]
    fn move_queue_overflows_at_1024() {
        let (mut m, mut hw, mut h) = configured(&["config_stepper oid=0 step_pin=136 dir_pin=135 invert_step=0 step_pulse_ticks=40"]);
        h.send(&mut m, &mut hw, 0.01, &["reset_step_clock oid=0 clock=100000000"]);
        for _ in 0..MOVE_COUNT + 1 {
            h.send(&mut m, &mut hw, 0.01, &["queue_step oid=0 interval=1000000 count=1 add=0"]);
        }
        // one move is loaded (its slot freed), so 1025 fit; the next overflows
        assert!(m.is_shutdown().is_none());
        h.send(&mut m, &mut hw, 0.01, &["queue_step oid=0 interval=1000000 count=1 add=0"]);
        assert_eq!(m.is_shutdown(), Some("Move queue overflow"));
    }

    #[test]
    fn endstop_stops_steppers_through_trsync() {
        let (mut m, mut hw, mut h) = configured(&[
            "config_stepper oid=0 step_pin=136 dir_pin=135 invert_step=0 step_pulse_ticks=40",
            "config_endstop oid=1 pin=200 pull_up=0",
            "config_trsync oid=2",
        ]);
        hw.pin_after = Some(0.15);
        h.send(
            &mut m,
            &mut hw,
            0.01,
            &[
                "trsync_start oid=2 report_clock=4000000 report_ticks=20000000 expire_reason=4",
                "stepper_stop_on_trigger oid=0 trsync_oid=2",
                "trsync_set_timeout oid=2 clock=200000000",
                "endstop_home oid=1 clock=4000000 sample_ticks=2000 sample_count=4 rest_ticks=20000 pin_value=1 trsync_oid=2 trigger_reason=1",
                "reset_step_clock oid=0 clock=20000000",
                "queue_step oid=0 interval=200000 count=1000 add=0",
            ],
        );
        m.advance(0.5, &mut hw);
        let r = h.replies(&mut m);
        assert!(r.iter().any(|s| s.starts_with("trsync_state oid=2 can_trigger=0 trigger_reason=1")), "{r:?}");
        // 1 kHz steps from 0.1 s, trigger at ~0.15 s
        let n = hw.steps.len();
        assert!((49..=52).contains(&n), "{n} steps");
        let r = h.send(&mut m, &mut hw, 0.5, &["stepper_get_position oid=0"]);
        assert_eq!(r, [format!("stepper_position oid=0 pos=-{n}")]);
        // further steps are dropped until reset_step_clock
        h.send(&mut m, &mut hw, 0.5, &["queue_step oid=0 interval=200000 count=10 add=0"]);
        m.advance(1.0, &mut hw);
        assert_eq!(hw.steps.len(), n);
        assert!(m.is_shutdown().is_none());
    }

    #[test]
    fn analog_reports_the_sample_sum() {
        let (mut m, mut hw, mut h) = configured(&["config_analog_in oid=0 pin=46"]);
        h.send(&mut m, &mut hw, 0.01, &["query_analog_in oid=0 clock=4000000 sample_ticks=200000 sample_count=8 rest_ticks=60000000 min_value=0 max_value=65535 range_check_count=1"]);
        m.advance(0.7, &mut hw);
        let r = h.replies(&mut m);
        assert_eq!(r.len(), 3, "reports at 0.02, 0.32, 0.62 s: {r:?}");
        assert!(r[0].ends_with("value=8000"));
        // out of range -> shutdown
        h.send(&mut m, &mut hw, 0.7, &["query_analog_in oid=0 clock=150000000 sample_ticks=200000 sample_count=8 rest_ticks=60000000 min_value=0 max_value=4000 range_check_count=1"]);
        m.advance(1.0, &mut hw);
        assert_eq!(m.is_shutdown(), Some("ADC out of range"));
    }

    #[test]
    fn stm32_waits_in_bootloader_after_power_up() {
        let mut m = VirtualMcu::new(McuKind::Toolhead, 0.0);
        let mut hw = Rec::default();
        m.set_power(false, 1.0, &mut hw);
        m.set_power(true, 2.0, &mut hw);
        m.advance(3.5, &mut hw);
        assert_eq!(m.take_output(), b"CC");
        m.receive(&encode_frame(0, &[]).unwrap(), 3.6, &mut hw);
        assert!(m.take_output().is_empty(), "frames are ignored by the bootloader");
        m.receive(b"aaaa", 4.0, &mut hw);
        assert_eq!(m.power, Power::App);
        assert_eq!(m.clock_at(5.0), cc_proto::clock::STM32_CLOCK_FREQ as u64, "clock restarts at boot");
    }
}
