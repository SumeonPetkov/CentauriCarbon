//! Test scripts for the emulator: one command per line, `#` comments.
//!
//! ```text
//! option speed 20                 # thermal speed-up (also: corrupt <p>, bootloader)
//! option config strain_gauge enable_z_home 1   # a user_printer.cfg entry
//! option bed_shape 0.3 -0.2 0.1   # bed tilt in X, in Y, dome (mm)
//! wait_until links == 3 timeout 20
//! tap "Light"                     # a visible widget by its text (or: tap 120 200)
//! expect light == 1
//! goto print                      # switch screen
//! gcode M104 S200                 # like a button with `tap = "gcode …"`
//! command toggle_light            # any screen command by name
//! print cube_20mm.gcode           # start a file from the G-code folder
//! wait 1.5
//! wait_until print_state == "Finished" timeout 300
//! expect cold_extrusion < 0.01
//! inject hotend_heater_dead       # fault; `clear <fault>` removes it
//! screenshot out/home.png         # LCD as PNG; bedshot = printed part, top view
//! power_cut                       # mains lost (ccos saves the print, then stops)
//! power_on                        # mains back: ccos boots again
//! echo any text
//! show x host_x nozzle             # print values
//! ```
//!
//! Expressions: `<name> <op> <value>` with op one of `< <= > >= == != ~ !~`
//! (`~` = contains). Names: see [`crate::QUERIES`].

use std::path::Path;
use std::time::Instant;

use cc_sim::Fault;
use cc_ui::{Model, Touch};

use crate::{Emu, Options, Val};

#[derive(Debug, Clone)]
pub enum Op {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
    Contains,
    NotContains,
}

#[derive(Debug, Clone)]
pub struct Cond {
    pub name: String,
    pub op: Op,
    pub rhs: Val,
}

#[derive(Debug, Clone)]
pub enum Step {
    Wait(f64),
    WaitUntil(Cond, f64),
    TapXY(i32, i32),
    TapText(String),
    Goto(String),
    Gcode(String),
    Command(String),
    Print(String),
    Expect(Cond),
    Screenshot(String),
    Bedshot(String),
    Inject(Fault, bool),
    Echo(String),
    Power(bool),
    Show(Vec<String>),
}

pub struct Script {
    pub steps: Vec<(usize, Step)>,
}

fn unquote(s: &str) -> String {
    let s = s.trim();
    s.strip_prefix('"').and_then(|s| s.strip_suffix('"')).unwrap_or(s).to_string()
}

fn parse_cond(s: &str) -> Result<Cond, String> {
    let ops = [("<=", Op::Le), (">=", Op::Ge), ("==", Op::Eq), ("!=", Op::Ne), ("!~", Op::NotContains), ("<", Op::Lt), (">", Op::Gt), ("~", Op::Contains)];
    for (tok, op) in ops {
        if let Some((l, r)) = s.split_once(&format!(" {tok} ")) {
            let r = r.trim();
            let hex = r.strip_prefix("0x").and_then(|h| u64::from_str_radix(h, 16).ok()).map(|v| Val::Num(v as f64));
            let rhs = if r.starts_with('"') {
                Val::Str(unquote(r))
            } else if let Some(h) = hex {
                h
            } else {
                r.parse::<f64>().map(Val::Num).unwrap_or_else(|_| Val::Str(r.to_string()))
            };
            return Ok(Cond { name: l.trim().to_string(), op, rhs });
        }
    }
    Err(format!("expected '<name> <op> <value>' (ops: < <= > >= == != ~ !~), got '{s}'"))
}

/// Parse a script; `opts` takes the `option` lines.
pub fn parse(text: &str, opts: &mut Options) -> Result<Script, String> {
    let mut steps = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let n = i + 1;
        let line = match raw.find(" #") {
            Some(p) if !raw[..p].contains('"') || raw[..p].matches('"').count() % 2 == 0 => &raw[..p],
            _ => raw,
        };
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (cmd, rest) = line.split_once(char::is_whitespace).map_or((line, ""), |(c, r)| (c, r.trim()));
        let err = |e: String| format!("line {n}: {e}");
        let step = match cmd {
            "option" => {
                let (k, v) = rest.split_once(char::is_whitespace).unwrap_or((rest, ""));
                match k {
                    "speed" => opts.speed = v.trim().parse().map_err(|_| err("option speed <number>".into()))?,
                    "corrupt" => opts.corrupt = v.trim().parse().map_err(|_| err("option corrupt <probability>".into()))?,
                    "bootloader" => opts.bootloader = true,
                    "config" => {
                        let mut it = v.split_whitespace();
                        match (it.next(), it.next(), it.next()) {
                            (Some(s), Some(k), Some(val)) => opts.config.push((s.into(), k.into(), val.into())),
                            _ => return Err(err("option config <section> <key> <value>".into())),
                        }
                    }
                    "bed_shape" => {
                        let v: Vec<f64> = v.split_whitespace().filter_map(|s| s.parse().ok()).collect();
                        let [a, b, c] = v[..] else { return Err(err("option bed_shape <tilt x> <tilt y> <dome> (mm)".into())) };
                        opts.bed_shape = [a, b, c];
                    }
                    _ => return Err(err(format!("unknown option '{k}'"))),
                }
                continue;
            }
            "wait" => Step::Wait(rest.parse().map_err(|_| err("wait <seconds>".into()))?),
            "wait_until" => {
                let (c, t) = match rest.rsplit_once(" timeout ") {
                    Some((c, t)) => (c, t.trim().parse().map_err(|_| err("timeout <seconds>".into()))?),
                    None => (rest, 60.0),
                };
                Step::WaitUntil(parse_cond(c).map_err(err)?, t)
            }
            "expect" => Step::Expect(parse_cond(rest).map_err(err)?),
            "tap" => {
                let mut it = rest.split_whitespace();
                match (it.next().and_then(|x| x.parse().ok()), it.next().and_then(|y| y.parse().ok())) {
                    (Some(x), Some(y)) => Step::TapXY(x, y),
                    _ => Step::TapText(unquote(rest)),
                }
            }
            "goto" => Step::Goto(rest.to_string()),
            "gcode" => Step::Gcode(rest.replace("\\n", "\n")),
            "command" => Step::Command(rest.to_string()),
            "print" => Step::Print(rest.to_string()),
            "screenshot" => Step::Screenshot(rest.to_string()),
            "bedshot" => Step::Bedshot(rest.to_string()),
            "inject" | "clear" => {
                let f = Fault::parse(rest).ok_or_else(|| err(format!("unknown fault '{rest}' (known: {})", Fault::ALL.map(|f| f.name()).join(", "))))?;
                Step::Inject(f, cmd == "inject")
            }
            "echo" => Step::Echo(rest.to_string()),
            "power_cut" => Step::Power(false),
            "power_on" => Step::Power(true),
            "show" => Step::Show(rest.split_whitespace().map(String::from).collect()),
            _ => return Err(err(format!("unknown command '{cmd}'"))),
        };
        steps.push((n, step));
    }
    Ok(Script { steps })
}

fn eval(emu: &Emu, c: &Cond) -> Result<(bool, Val), String> {
    let v = emu.query(&c.name).ok_or_else(|| format!("unknown name '{}' (see ccsim --help)", c.name))?;
    let ok = match (&v, &c.rhs) {
        (Val::Num(a), Val::Num(b)) => match c.op {
            Op::Lt => a < b,
            Op::Le => a <= b,
            Op::Gt => a > b,
            Op::Ge => a >= b,
            Op::Eq => (a - b).abs() < 1e-9,
            Op::Ne => (a - b).abs() >= 1e-9,
            Op::Contains | Op::NotContains => return Err("~ compares text".into()),
        },
        (Val::Str(a), Val::Str(b)) => match c.op {
            Op::Eq => a == b,
            Op::Ne => a != b,
            Op::Contains => a.contains(b.as_str()),
            Op::NotContains => !a.contains(b.as_str()),
            _ => return Err("< and > compare numbers".into()),
        },
        (Val::Str(a), Val::Num(b)) => match c.op {
            Op::Eq => a == &b.to_string(),
            Op::Ne => a != &b.to_string(),
            _ => return Err(format!("'{}' is text ({a:?})", c.name)),
        },
        (Val::Num(_), Val::Str(_)) => return Err(format!("'{}' is a number", c.name)),
    };
    Ok((ok, v))
}

/// Find a visible widget on the current screen showing `text`.
fn find_widget(emu: &Emu, text: &str) -> Option<(i32, i32)> {
    let s = emu.rt.state();
    for w in emu.rt.engine.screen().widgets {
        if let Some(c) = w.show_if {
            if s.flag(c.flag) == c.negate {
                continue;
            }
        }
        if w.tap.is_none() {
            continue;
        }
        let mut shown = cc_ui::TextBuf::default();
        match w.value {
            Some(v) => s.text(v, &mut shown),
            None => shown.push_str(w.text),
        }
        let label = shown.as_str().replace('\n', " ");
        if label == text || w.text.replace('\n', " ") == text {
            return Some(((w.rect.x + w.rect.w / 2) as i32, (w.rect.y + w.rect.h / 2) as i32));
        }
    }
    None
}

#[derive(Debug, PartialEq)]
pub enum Status {
    Running,
    Done,
    Failed(String),
}

/// Executes a script one UI frame at a time.
pub struct Runner {
    script: Script,
    idx: usize,
    started: Option<Instant>,
    tap: Option<(i32, i32, u8)>,
    pub echo: bool,
}

impl Runner {
    pub fn new(script: Script) -> Self {
        Self { script, idx: 0, started: None, tap: None, echo: true }
    }

    /// The touch to feed this frame (taps press for one frame, release the next).
    pub fn touch(&mut self) -> Touch {
        match &mut self.tap {
            Some((x, y, phase)) => {
                let t = Touch { x: *x, y: *y, pressed: *phase == 0 };
                *phase += 1;
                t
            }
            None => Touch::default(),
        }
    }

    /// Advance after a frame. `last_touch` tells whether the tap finished.
    pub fn step(&mut self, emu: &mut Emu) -> Status {
        loop {
            let Some((line, step)) = self.script.steps.get(self.idx).cloned() else { return Status::Done };
            let fail = |m: String| Status::Failed(format!("line {line}: {m}"));
            let t0 = *self.started.get_or_insert_with(Instant::now);
            let elapsed = t0.elapsed().as_secs_f64();
            match &step {
                Step::Wait(s) => {
                    if elapsed < *s {
                        return Status::Running;
                    }
                }
                Step::WaitUntil(c, timeout) => match eval(emu, c) {
                    Err(e) => return fail(e),
                    Ok((true, _)) => {}
                    Ok((false, v)) => {
                        let watching_faults = ["shutdown", "mcu_shutdown", "status", "print_state"].contains(&c.name.as_str());
                        if !watching_faults {
                            if let Some(s) = emu.rt.state().shutdown.clone() {
                                return fail(format!("ccos shut down while waiting for {} {:?} {}: {s}", c.name, c.op, c.rhs));
                            }
                        }
                        if c.name == "print_state" && emu.query("print_state") == Some(Val::Str("Failed".into())) && c.rhs != Val::Str("Failed".into()) {
                            return fail(format!("print failed: {}", emu.rt.state().status));
                        }
                        if elapsed > *timeout {
                            return fail(format!("timed out after {timeout} s waiting for {} {:?} {} (is {v}); status: {}", c.name, c.op, c.rhs, emu.rt.state().status));
                        }
                        return Status::Running;
                    }
                },
                Step::Expect(c) => match eval(emu, c) {
                    Err(e) => return fail(e),
                    Ok((true, _)) => {}
                    Ok((false, v)) => return fail(format!("expected {} {:?} {}, but it is {v}", c.name, c.op, c.rhs)),
                },
                Step::TapXY(x, y) => {
                    if !self.tap_done(*x, *y) {
                        return Status::Running;
                    }
                }
                Step::TapText(t) => {
                    let pos = match self.tap {
                        Some((x, y, _)) => (x, y),
                        None => match find_widget(emu, t) {
                            Some(p) => p,
                            None => return fail(format!("no visible button \"{t}\" on screen '{}'", emu.screen_name())),
                        },
                    };
                    if !self.tap_done(pos.0, pos.1) {
                        return Status::Running;
                    }
                }
                Step::Goto(name) => match ccos::ui::SCREENS.iter().position(|s| s.name == name) {
                    Some(i) => emu.rt.engine.goto(i as u8),
                    None => return fail(format!("no screen '{name}'")),
                },
                Step::Gcode(g) => emu.rt.send(ccos::gcode::Job::Gcode(g.clone())),
                Step::Command(c) => match ccos::ui::Cmd::ALL.iter().find(|x| x.name() == c) {
                    Some(&cmd) => {
                        emu.rt.command(cmd);
                    }
                    None => return fail(format!("no command '{c}'")),
                },
                Step::Print(f) => {
                    let p = emu.gcode_dir.join(f);
                    if !p.exists() {
                        return fail(format!("{} does not exist", p.display()));
                    }
                    emu.sim.lock().world.clear_print();
                    emu.rt.print_path(p);
                }
                Step::Screenshot(p) | Step::Bedshot(p) => {
                    let path = emu.opts.out_dir.join(p);
                    if let Some(d) = path.parent() {
                        let _ = std::fs::create_dir_all(d);
                    }
                    let r = if matches!(step, Step::Screenshot(_)) { emu.screenshot(&path) } else { emu.bedshot(&path) };
                    if let Err(e) = r {
                        return fail(format!("{}: {e}", path.display()));
                    }
                    if self.echo {
                        println!("ccsim: wrote {}", path.display());
                    }
                }
                Step::Inject(f, on) => emu.sim.lock().inject(*f, *on),
                Step::Echo(t) => println!("ccsim: {t}"),
                Step::Power(true) => emu.power_on(),
                Step::Power(false) => emu.power_cut(),
                Step::Show(names) => {
                    let mut out = String::new();
                    for n in names {
                        match emu.query(n) {
                            Some(v) => out.push_str(&format!("{n}={v} ")),
                            None => return fail(format!("unknown name '{n}'")),
                        }
                    }
                    println!("ccsim: {out}");
                }
            }
            if self.echo {
                println!("ccsim: [{:7.2}s] ok line {line}: {}", emu.sim.lock().now(), describe(&step));
            }
            self.idx += 1;
            self.started = None;
            self.tap = None;
        }
    }

    /// Two frames: press, release.
    fn tap_done(&mut self, x: i32, y: i32) -> bool {
        match self.tap {
            None => {
                self.tap = Some((x, y, 0));
                false
            }
            Some((_, _, phase)) => phase >= 2,
        }
    }
}

fn describe(s: &Step) -> String {
    match s {
        Step::Wait(t) => format!("wait {t}"),
        Step::WaitUntil(c, _) => format!("wait_until {} {:?} {}", c.name, c.op, c.rhs),
        Step::Expect(c) => format!("expect {} {:?} {}", c.name, c.op, c.rhs),
        Step::TapXY(x, y) => format!("tap {x} {y}"),
        Step::TapText(t) => format!("tap \"{t}\""),
        Step::Goto(n) => format!("goto {n}"),
        Step::Gcode(g) => format!("gcode {}", g.replace('\n', "\\n")),
        Step::Command(c) => format!("command {c}"),
        Step::Print(f) => format!("print {f}"),
        Step::Screenshot(p) => format!("screenshot {p}"),
        Step::Bedshot(p) => format!("bedshot {p}"),
        Step::Inject(f, on) => format!("{} {}", if *on { "inject" } else { "clear" }, f.name()),
        Step::Echo(t) => format!("echo {t}"),
        Step::Power(on) => (if *on { "power_on" } else { "power_cut" }).into(),
        Step::Show(n) => format!("show {}", n.join(" ")),
    }
}

/// Run a script headless; Ok(()) when every step passed.
pub fn run_file(path: &Path, mut opts: Options) -> Result<(), String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let script = parse(&text, &mut opts)?;
    run(script, opts, None)
}

/// Run a parsed script, optionally in a window (`show` renders each frame).
pub fn run(script: Script, opts: Options, mut show: Option<&mut dyn FnMut(&Emu) -> bool>) -> Result<(), String> {
    let mut emu = Emu::start(opts).map_err(|e| format!("starting the simulation: {e}"))?;
    let mut runner = Runner::new(script);
    let frame = std::time::Duration::from_millis(16);
    loop {
        let t = Instant::now();
        let touch = runner.touch();
        emu.frame(touch);
        match runner.step(&mut emu) {
            Status::Running => {}
            Status::Done => return Ok(()),
            Status::Failed(e) => {
                let mut msg = e;
                for l in emu.summary() {
                    msg.push_str("\n  sim: ");
                    msg.push_str(&l);
                }
                let s = emu.rt.state();
                for l in &s.log {
                    msg.push_str("\n  ccos: ");
                    msg.push_str(l);
                }
                return Err(msg);
            }
        }
        if let Some(f) = show.as_mut() {
            if !f(&emu) {
                return Err("window closed".into());
            }
        }
        if let Some(d) = frame.checked_sub(t.elapsed()) {
            std::thread::sleep(d);
        }
    }
}
