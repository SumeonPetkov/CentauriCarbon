//! `ccos` — runs on the printer (or renders its UI on a PC with `--png`).
//!
//! Stop Elegoo's `app` process first: only one program may own the MCU links,
//! the framebuffer and the touch panel. A reboot returns to stock.

use std::time::{Duration, Instant};

use cc_gui::text::Font;
use cc_gui::ui::{InputState, Theme, Ui};
use cc_gui::Canvas;
use ccos::dashboard::{self, Action, State};
use ccos::mcu::{ntc_temp, Mcu, Result};
use ccos::{paths, Target};

const USAGE: &str = "\
ccos — custom host for the Elegoo Centauri Carbon

USAGE:
  ccos probe [dsp|toolhead|sg|all] [--trace]   identify MCUs, read config/clock
  ccos console <dsp|toolhead|sg>               raw Klipper command console (stdin)
  ccos temps                                   read bed/chamber thermistors via the DSP
  ccos power-cycle <toolhead|sg>               reset an STM32 board via its power pin (as the stock host does)
  ccos ui [--png out.png] [--font font.ttc]    run the touchscreen UI (or render one frame)

Kill the stock `app` process before using probe/console/temps/ui on the printer.";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.first().map(String::as_str) {
        Some("probe") => probe(&args[1..]),
        Some("console") => console(&args[1..]),
        Some("temps") => temps(),
        Some("power-cycle") => power_cycle(&args[1..]),
        Some("ui") => ui(&args[1..]),
        _ => {
            println!("{USAGE}");
            Ok(())
        }
    };
    if let Err(e) = r {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn flag_value<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).map(String::as_str)
}

#[cfg(target_os = "linux")]
fn connect(t: Target, trace: bool) -> Result<Mcu> {
    let mut m = Mcu::new(t.label(), t.open()?);
    m.trace = trace;
    m.sync()?;
    Ok(m)
}

#[cfg(not(target_os = "linux"))]
fn connect(_t: Target, _trace: bool) -> Result<Mcu> {
    Err("MCU links need Linux".into())
}

fn probe_one(t: Target, trace: bool) -> Result<String> {
    let mut m = connect(t, trace)?;
    let ident = m.identify()?;
    // Stock firmware returns a stale (but valid) dictionary; ids come from the fixed table.
    let ver = if cc_proto::Dictionary::from_identify(&ident).is_ok() { "dict ok" } else { "dict unreadable" };
    let cfg = m.request("get_config", "config")?;
    let up = m.request("get_uptime", "uptime")?;
    let hz = m.measure_clock()?;
    Ok(format!(
        "identify {} bytes ({ver}); is_config={} crc={:#x} move_count={} shutdown={}; uptime_hi={}; clock≈{:.1} MHz (nominal {:.0})",
        ident.len(),
        cfg.int("is_config").unwrap_or(-1),
        cfg.int("crc").unwrap_or(0),
        cfg.int("move_count").unwrap_or(0),
        cfg.int("is_shutdown").unwrap_or(-1),
        up.int("high").unwrap_or(0),
        hz / 1e6,
        t.nominal_clock() / 1e6
    ))
}

fn probe(args: &[String]) -> Result<()> {
    let trace = args.iter().any(|a| a == "--trace");
    let which = args.iter().find(|a| !a.starts_with("--")).map(String::as_str).unwrap_or("all");
    let targets = if which == "all" { vec![Target::Dsp, Target::Toolhead, Target::StrainGauge] } else { vec![Target::parse(which).ok_or("unknown mcu")?] };
    for t in targets {
        match probe_one(t, trace) {
            Ok(s) => println!("{:9} OK   {s}", t.label()),
            Err(e) => println!("{:9} FAIL {e}", t.label()),
        }
    }
    Ok(())
}

fn console(args: &[String]) -> Result<()> {
    let t = Target::parse(args.first().map(String::as_str).unwrap_or("")).ok_or("usage: ccos console <dsp|toolhead|sg>")?;
    let mut m = connect(t, true)?;
    eprintln!("connected to {}; type Klipper commands (e.g. get_uptime), 'help' lists formats", t.label());
    let stdin = std::io::stdin();
    let mut line = String::new();
    loop {
        line.clear();
        if stdin.read_line(&mut line)? == 0 {
            return Ok(());
        }
        let l = line.trim();
        if l.is_empty() {
            continue;
        }
        if l == "help" {
            let mut f: Vec<_> = m.dict.formats().map(|f| (f.id, f.format.clone())).collect();
            f.sort();
            for (id, s) in f {
                println!("{id:3} {s}");
            }
            continue;
        }
        if let Err(e) = m.send(l) {
            eprintln!("error: {e}");
        }
        let _ = m.wait_for("__none__", Duration::from_millis(200));
        while let Some(msg) = m.inbox.pop_front() {
            println!("{msg}");
        }
    }
}

#[cfg(target_os = "linux")]
fn power_cycle(args: &[String]) -> Result<()> {
    let t = Target::parse(args.first().map(String::as_str).unwrap_or("")).ok_or("usage: ccos power-cycle <toolhead|sg>")?;
    t.power_cycle()?;
    println!("{} power-cycled", t.label());
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn power_cycle(_: &[String]) -> Result<()> {
    Err("needs Linux".into())
}

/// Configure two analog inputs on the DSP and print temperatures.
/// Requires an unconfigured DSP (fresh boot, stock app never started).
fn temps() -> Result<()> {
    let mut m = connect(Target::Dsp, false)?;
    let cfg = m.request("get_config", "config")?;
    if cfg.int("is_config") == Some(1) {
        return Err("DSP is already configured (stock app ran since boot). Reboot without starting `app`.".into());
    }
    let pin = |n: &str| cc_proto::dict::pin_number(n, 32).unwrap();
    // printer.cfg: [heater_bed] sensor_pin PB14, [temperature_sensor box] PB13
    let sensors = [("bed", pin("PB14")), ("chamber", pin("PB13"))];
    m.send(&format!("allocate_oids count={}", sensors.len()))?;
    for (oid, (_, p)) in sensors.iter().enumerate() {
        m.send(&format!("config_analog_in oid={oid} pin={p}"))?;
    }
    m.send("finalize_config crc=0")?;
    let freq = cc_proto::clock::DSP_CLOCK_FREQ;
    let now = m.request("get_clock", "clock")?.int("clock").unwrap_or(0) as u32;
    let samples = 8u32;
    for oid in 0..sensors.len() {
        let start = now.wrapping_add((freq * 0.2) as u32 + oid as u32 * 1000);
        m.send(&format!(
            "query_analog_in oid={oid} clock={start} sample_ticks={} sample_count={samples} rest_ticks={} min_value=0 max_value=65535 range_check_count=1",
            (freq * 0.001) as u32,
            (freq * 0.5) as u32
        ))?;
    }
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(3600) {
        let msg = m.wait_for("analog_in_state", Duration::from_secs(2))?;
        let oid = msg.int("oid").unwrap_or(0) as usize;
        let v = msg.int("value").unwrap_or(0) as f64 / (samples as f64 * cc_proto::clock::ADC_MAX as f64);
        if let Some((name, _)) = sensors.get(oid) {
            println!("{name:8} adc={v:.4} temp={:.1}°C", ntc_temp(v, 3950.0).unwrap_or(f64::NAN));
        }
    }
    Ok(())
}

fn load_font(args: &[String]) -> Result<Font> {
    let path = flag_value(args, "--font").unwrap_or(paths::FONT);
    Ok(Font::load(path, 0)?)
}

fn ui(args: &[String]) -> Result<()> {
    let mut font = load_font(args)?;
    let mut canvas = Canvas::new(cc_gui::SCREEN_W, cc_gui::SCREEN_H);
    let mut state = State::default();
    let theme = Theme::default();

    if let Some(out) = flag_value(args, "--png") {
        // Host preview: render each page once.
        for (page, suffix) in [(dashboard::Page::Home, ""), (dashboard::Page::System, "_sys")] {
            state.page = page;
            state.nozzle = (212.0, 220.0);
            state.bed = (59.6, 60.0);
            state.chamber = 31.0;
            state.mcus[0].ok = Some(true);
            state.mcus[0].detail = "200.0 MHz, configured".into();
            state.mcus[1].ok = Some(true);
            state.mcus[1].detail = "84.0 MHz via /dev/ttyACM0".into();
            state.mcus[2].ok = Some(false);
            state.mcus[2].detail = "no response on /dev/ttyS4".into();
            let mut ui = Ui::new(&mut canvas, &mut font, InputState::default(), theme);
            dashboard::frame(&mut ui, &mut state);
            let path = if suffix.is_empty() { out.to_string() } else { out.replace(".png", &format!("{suffix}.png")) };
            std::fs::write(&path, canvas.to_png())?;
            println!("wrote {path}");
        }
        return Ok(());
    }
    run_device_ui(&mut canvas, &mut font, &mut state, theme)
}

#[cfg(target_os = "linux")]
fn run_device_ui(canvas: &mut Canvas, font: &mut Font, state: &mut State, theme: Theme) -> Result<()> {
    use cc_gui::fb::{set_backlight, Framebuffer};
    use cc_gui::touch::Touch;
    let mut fb = Framebuffer::open(paths::FRAMEBUFFER)?;
    let mut touch = Touch::open(paths::TOUCH)?;
    let mut input = InputState::default();
    let _ = set_backlight(state.backlight);
    let frame_time = Duration::from_millis(33);
    loop {
        let t0 = Instant::now();
        let ts = touch.poll();
        input.update(ts.x, ts.y, ts.pressed);
        let mut ui = Ui::new(canvas, font, input, theme);
        let action = dashboard::frame(&mut ui, state);
        fb.present(canvas);
        match action {
            Action::Exit => return Ok(()),
            Action::Backlight(v) => {
                let _ = set_backlight(v);
            }
            Action::ScreenOff => {
                let _ = set_backlight(0);
                state.log.push("screen off — tap to wake".into());
                while !touch.poll().pressed {
                    std::thread::sleep(Duration::from_millis(50));
                }
                let _ = set_backlight(state.backlight);
            }
            Action::Probe => {
                for (i, t) in [Target::Dsp, Target::Toolhead, Target::StrainGauge].into_iter().enumerate() {
                    let r = probe_one(t, false);
                    state.mcus[i].ok = Some(r.is_ok());
                    state.mcus[i].detail = match r {
                        Ok(s) => s.split(';').last().unwrap_or("").trim().to_string(),
                        Err(e) => e.to_string(),
                    };
                    state.log.push(format!("{}: {}", t.label(), state.mcus[i].detail));
                }
            }
            Action::None => {}
        }
        if let Some(rest) = frame_time.checked_sub(t0.elapsed()) {
            std::thread::sleep(rest);
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn run_device_ui(_: &mut Canvas, _: &mut Font, _: &mut State, _: Theme) -> Result<()> {
    Err("device UI needs Linux; use --png".into())
}
