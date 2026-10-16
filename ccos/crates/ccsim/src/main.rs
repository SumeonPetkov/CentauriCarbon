//! `ccsim` — run ccos against a virtual Centauri Carbon.

use std::path::PathBuf;

use ccsim::{script, Emu, Options};

const USAGE: &str = "\
ccsim — ccos dev emulator (virtual Centauri Carbon)

USAGE:
  ccsim [OPTIONS]                 window: LCD (click = touch) + bed view + machine state
  ccsim [OPTIONS] <script.ccsim>  run a test script (headless unless --window); exit 1 on failure
  ccsim --printer-only [OPTIONS]  only the virtual printer: point a separate ccos at it

OPTIONS:
  --dir <dir>          simulation directory (default: $TMPDIR/ccsim-<pid>)
  --gcode-dir <dir>    G-code folder (default: <dir>/gcodes, with sample files)
  --speed <n>          heat/cool n× faster than real (default 1; scripts: `option speed`)
  --bootloader         STM32 boards start in their bootloader (cold power-on)
  --corrupt <p>        corrupt each host→MCU byte with probability p (tests retransmits)
  --out <dir>          where script screenshots go (default .)
  --headless           no window (scripts)
  --window             show the window while a script runs
  --verbose            print MCU/physics events and ccos log lines

Script commands: option, wait, wait_until, expect, tap, goto, gcode, command,
print, inject, clear, screenshot, bedshot, echo — see crates/ccsim/src/script.rs.
Names for expect/wait_until:";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut o = Options::default();
    let (mut headless, mut window, mut printer_only) = (false, false, false);
    let mut script_path: Option<PathBuf> = None;
    let mut it = args.iter();
    let need = |v: Option<&String>, f: &str| -> String {
        v.cloned().unwrap_or_else(|| {
            eprintln!("{f} needs a value");
            std::process::exit(2)
        })
    };
    while let Some(a) = it.next() {
        match a.as_str() {
            "--dir" => o.dir = need(it.next(), a).into(),
            "--gcode-dir" => o.gcode_dir = Some(need(it.next(), a).into()),
            "--speed" => o.speed = need(it.next(), a).parse().unwrap_or(1.0),
            "--corrupt" => o.corrupt = need(it.next(), a).parse().unwrap_or(0.0),
            "--out" => o.out_dir = need(it.next(), a).into(),
            "--bootloader" => o.bootloader = true,
            "--headless" => headless = true,
            "--window" => window = true,
            "--verbose" => o.verbose = true,
            "--printer-only" => printer_only = true,
            "-h" | "--help" => {
                println!("{USAGE}");
                for (n, d) in ccsim::QUERIES {
                    println!("  {n:<62} {d}");
                }
                return;
            }
            s if !s.starts_with("--") && script_path.is_none() => script_path = Some(s.into()),
            s => {
                eprintln!("unknown option {s} (see --help)");
                std::process::exit(2);
            }
        }
    }

    if printer_only {
        let sim = match cc_sim::Sim::start(cc_sim::SimConfig {
            dir: o.dir.clone(),
            world: cc_sim::WorldConfig { thermal_speed: o.speed, ..Default::default() },
            mcus_running: !o.bootloader,
            corrupt_rate: o.corrupt,
            verbose: true,
        }) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        };
        println!("virtual printer running in {}; in another terminal:", o.dir.display());
        println!("  CCOS_SIM={} ccos probe all", o.dir.display());
        for (k, v) in sim.env() {
            println!("  ({k}={v})");
        }
        loop {
            std::thread::sleep(std::time::Duration::from_secs(5));
            if let Some((k, why)) = sim.lock().shutdown() {
                println!("{}: shutdown: {why}", k.name());
            }
        }
    }

    let result = match script_path {
        Some(p) => {
            let text = match std::fs::read_to_string(&p) {
                Ok(t) => t,
                Err(e) => {
                    eprintln!("error: {}: {e}", p.display());
                    std::process::exit(1);
                }
            };
            match script::parse(&text, &mut o) {
                Err(e) => Err(format!("{}: {e}", p.display())),
                Ok(s) if window && !headless => run_script_window(s, o),
                Ok(s) => script::run(s, o, None),
            }
        }
        None if headless => Err("nothing to do: give a script, or drop --headless for the window".into()),
        None => interactive(o),
    };
    match result {
        Ok(()) => println!("ccsim: PASS"),
        Err(e) => {
            eprintln!("ccsim: FAIL {e}");
            std::process::exit(1);
        }
    }
}

#[cfg(feature = "window")]
fn interactive(o: Options) -> Result<(), String> {
    let mut view = ccsim::window::View::open("ccsim — Centauri Carbon emulator (ccos)")?;
    let mut emu = Emu::start(o).map_err(|e| e.to_string())?;
    println!("ccsim: G-code folder {}", emu.gcode_dir.display());
    while view.is_open() {
        let t = view.touch();
        if let Some(ccos::runtime::Event::Exit) = emu.frame(t) {
            break;
        }
        if !view.draw(&emu) {
            break;
        }
    }
    Ok(())
}

#[cfg(not(feature = "window"))]
fn interactive(_: Options) -> Result<(), String> {
    Err("built without the `window` feature; run a script instead".into())
}

#[cfg(feature = "window")]
fn run_script_window(s: script::Script, o: Options) -> Result<(), String> {
    let mut view = ccsim::window::View::open("ccsim — script")?;
    let mut show = |emu: &Emu| view.draw(emu);
    script::run(s, o, Some(&mut show))
}

#[cfg(not(feature = "window"))]
fn run_script_window(s: script::Script, o: Options) -> Result<(), String> {
    script::run(s, o, None)
}
