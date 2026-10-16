//! `ccos` — lightweight native host for the Elegoo Centauri Carbon.
//!
//! On the printer, stop Elegoo's `/app/app` first (it owns the MCU links,
//! the framebuffer and the touchscreen). A reboot returns to stock unless you
//! change the init scripts. On a PC, `ccsim` runs ccos against a virtual
//! printer; `CCOS_SIM=<dir> ccos probe all` talks to one started with
//! `ccsim --printer-only`.

use std::time::Duration;

use cc_ui::{Canvas, Engine, Touch};
use ccos::app::State;
use ccos::hw::Hardware;
use ccos::mcu::{Mcu, Result};
use ccos::ui;
use ccos::Target;

const USAGE: &str = "\
ccos — lightweight native host for the Elegoo Centauri Carbon

USAGE:
  ccos [ui] [--keep-dsp]          run the touchscreen UI (default)
  ccos preview <dir>              render every screen to <dir>/<screen>.png (works on a PC)
  ccos probe [dsp|toolhead|loadcell|all] [--trace] [--keep-dsp]
  ccos console <dsp|toolhead|loadcell>   raw Klipper commands from stdin
  ccos power-cycle <toolhead|loadcell>   reset an STM32 board and boot its firmware
  ccos dsp-log                    follow the DSP firmware's printf log

--keep-dsp: never restart the DSP through remoteproc (fail instead if another
host attached to it earlier in this boot).

Environment: CCOS_SIM=<dir> (use a ccsim virtual printer), CCOS_DSP_PORT,
CCOS_TOOLHEAD_PORT, CCOS_LOADCELL_PORT, CCOS_GPIO_ROOT, CCOS_GCODE_DIR,
CCOS_ENABLE_MOTION=1 (allow heaters/motors on real hardware — untested there).";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let restart_dsp = !args.iter().any(|a| a == "--keep-dsp");
    let hw = Hardware::from_env(restart_dsp);
    let pos: Vec<&str> = args.iter().map(String::as_str).filter(|a| !a.starts_with("--")).collect();
    let r = match pos.as_slice() {
        [] | ["ui"] => run_ui(hw),
        ["preview", dir] => preview(dir),
        ["probe", rest @ ..] => probe(&hw, rest.first().copied().unwrap_or("all"), args.iter().any(|a| a == "--trace")),
        ["console", t] => console(&hw, t),
        ["power-cycle", t] => power_cycle(&hw, t),
        ["dsp-log"] => dsp_log(),
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

/// Demo data for screenshots.
pub fn demo_state() -> State {
    let mut s = State::default();
    s.echo = false;
    s.nozzle = Some(214.6);
    s.nozzle_target = 220.0;
    s.bed = Some(59.8);
    s.bed_target = 60.0;
    s.chamber = Some(31.2);
    s.light = true;
    s.status = "3 of 3 MCUs connected".into();
    for (i, d) in ["200.0 MHz", "84.0 MHz", "84.0 MHz"].iter().enumerate() {
        s.links[i] = ccos::app::LinkStatus { state: ccos::app::LinkState::Ok, detail: d.to_string() };
    }
    for l in ["DSP: connected", "Toolhead: connected", "Load cell: connected", "DSP: ok, clock 200.0000 MHz", "Toolhead: ok, clock 84.0000 MHz", "Load cell: ok, clock 84.0000 MHz"] {
        s.log.push_back(l.into());
    }
    s.files = vec!["benchy_0.2mm_PLA.gcode".into(), "calibration_cube.gcode".into(), "phone_stand.gcode".into()];
    s.print.phase = ccos::app::Phase::Printing;
    s.print.file = "benchy_0.2mm_PLA.gcode".into();
    s.print.progress = 0.42;
    s.print.elapsed = 1312.0;
    s.pos = [112.4, 98.7, 4.2, 0.0];
    s.rss_kib = ccos::sys::rss_kib().unwrap_or(0);
    s
}

/// Render every screen with demo data (for designing the UI on a PC).
fn preview(dir: &str) -> Result<()> {
    std::fs::create_dir_all(dir)?;
    let s = demo_state();
    let mut canvas = Canvas::new(480, 272);
    for (i, sc) in ui::SCREENS.iter().enumerate() {
        let mut e = Engine::new(&ui::SCREENS, i as u8);
        e.update(&s, Touch::default(), &mut canvas);
        let path = format!("{dir}/{}.png", sc.name);
        std::fs::write(&path, cc_ui::png::encode(&canvas))?;
        println!("{path}");
    }
    Ok(())
}

#[cfg(target_os = "linux")]
fn run_ui(hw: Hardware) -> Result<()> {
    use cc_ui::fb::{set_backlight, Framebuffer};
    use ccos::runtime::{Event, Runtime};
    use std::time::Instant;

    let mut fb = Framebuffer::open(ccos::paths::FRAMEBUFFER)?;
    let mut touch = cc_ui::touch::Touch::open(ccos::paths::TOUCH)?;
    let mut rt = Runtime::start(hw);
    let _ = set_backlight(rt.state().backlight);

    let mut stats_at = Instant::now();
    let mut cpu_at = ccos::sys::cpu_seconds().unwrap_or(0.0);
    loop {
        // Sleep until a touch event or the 250 ms refresh tick: ~0% CPU idle.
        let mut pfd = libc::pollfd { fd: touch.fd(), events: libc::POLLIN, revents: 0 };
        unsafe { libc::poll(&mut pfd, 1, 250) };
        let t = touch.poll();

        if stats_at.elapsed() >= Duration::from_secs(1) {
            let cpu = ccos::sys::cpu_seconds().unwrap_or(cpu_at);
            let mut s = rt.state();
            s.cpu_pct = ((cpu - cpu_at) / stats_at.elapsed().as_secs_f64() * 100.0) as f32;
            s.rss_kib = ccos::sys::rss_kib().unwrap_or(0);
            drop(s);
            cpu_at = cpu;
            stats_at = Instant::now();
        }

        let ev = rt.frame(Touch { x: t.x, y: t.y, pressed: t.pressed });
        for r in rt.dirty() {
            fb.present_rect(&rt.canvas, *r);
        }
        match ev {
            Some(Event::Backlight(b)) => {
                let _ = set_backlight(b);
            }
            Some(Event::ScreenOff) => {
                let _ = set_backlight(0);
                // Wait for a full tap, then swallow it.
                while !touch.poll().pressed {
                    let mut pfd = libc::pollfd { fd: touch.fd(), events: libc::POLLIN, revents: 0 };
                    unsafe { libc::poll(&mut pfd, 1, -1) };
                }
                while touch.poll().pressed {
                    std::thread::sleep(Duration::from_millis(20));
                }
                let _ = set_backlight(rt.state().backlight);
                rt.engine.invalidate();
            }
            Some(Event::Exit) => {
                rt.stop();
                return Ok(());
            }
            None => {}
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn run_ui(_: Hardware) -> Result<()> {
    Err("the device UI needs Linux; use `ccos preview <dir>` on a PC".into())
}

#[cfg(target_os = "linux")]
fn connect(hw: &Hardware, t: Target, trace: bool) -> Result<Mcu> {
    let mut m = Mcu::new(t.label(), hw.open(t)?, t.nominal_clock(), if t == Target::Dsp { 1024 } else { 192 });
    m.trace = trace;
    m.sync()?;
    Ok(m)
}

#[cfg(not(target_os = "linux"))]
fn connect(_: &Hardware, _: Target, _: bool) -> Result<Mcu> {
    Err("MCU links need Linux".into())
}

fn probe(hw: &Hardware, which: &str, trace: bool) -> Result<()> {
    if ccos::sys::stock_app_running() && !hw.simulated {
        return Err("the stock app is running and owns the MCUs; stop it first".into());
    }
    let targets: Vec<Target> = if which == "all" { Target::ALL.to_vec() } else { vec![Target::parse(which).ok_or("unknown mcu")?] };
    for t in targets {
        let r = (|| -> Result<String> {
            let mut m = connect(hw, t, trace)?;
            let ident = m.identify()?;
            let cfg = m.request("get_config", "config")?;
            let hz = m.measure_clock(0.5)?;
            Ok(format!(
                "identify {} bytes; is_config={} crc={:#x} shutdown={}; clock {:.2} MHz (host assumes {:.0})",
                ident.len(),
                cfg.int("is_config").unwrap_or(-1),
                cfg.int("crc").unwrap_or(0),
                cfg.int("is_shutdown").unwrap_or(-1),
                hz / 1e6,
                t.nominal_clock() / 1e6
            ))
        })();
        match r {
            Ok(s) => println!("{:9} OK   {s}", t.label()),
            Err(e) => println!("{:9} FAIL {e}", t.label()),
        }
    }
    Ok(())
}

fn console(hw: &Hardware, t: &str) -> Result<()> {
    let t = Target::parse(t).ok_or("usage: ccos console <dsp|toolhead|loadcell>")?;
    let mut m = connect(hw, t, true)?;
    eprintln!("connected to {}; type Klipper commands (e.g. get_uptime); 'help' lists formats", t.label());
    let mut line = String::new();
    loop {
        line.clear();
        if std::io::stdin().read_line(&mut line)? == 0 {
            return Ok(());
        }
        let l = line.trim();
        if l.is_empty() {
            continue;
        }
        if l == "help" {
            for f in m.dict.formats() {
                println!("{:3} {}", f.id, f.format);
            }
            continue;
        }
        if let Err(e) = m.send_wait(l) {
            eprintln!("error: {e}");
        }
        let _ = m.wait_for("\0", Duration::from_millis(200));
        while let Some(msg) = m.inbox.pop_front() {
            println!("{}", msg.message());
        }
    }
}

#[cfg(target_os = "linux")]
fn power_cycle(hw: &Hardware, t: &str) -> Result<()> {
    let t = Target::parse(t).ok_or("usage: ccos power-cycle <toolhead|loadcell>")?;
    hw.power_cycle(t)?;
    println!("{} power-cycled and booted", t.label());
    Ok(())
}

#[cfg(target_os = "linux")]
fn dsp_log() -> Result<()> {
    let mut log = cc_transport::dsp::DspLog::open()?;
    let mut lines = Vec::new();
    loop {
        log.poll(&mut lines);
        for l in lines.drain(..) {
            println!("{l}");
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

#[cfg(not(target_os = "linux"))]
fn power_cycle(_: &Hardware, _: &str) -> Result<()> {
    Err("needs Linux".into())
}

#[cfg(not(target_os = "linux"))]
fn dsp_log() -> Result<()> {
    Err("needs Linux".into())
}
