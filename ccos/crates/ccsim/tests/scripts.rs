//! Every script in `ccos/sim/scripts/` must pass: ccos against the virtual
//! printer, headless. One at a time (they run in real time).

use std::path::PathBuf;
use std::sync::Mutex;

static SERIAL: Mutex<()> = Mutex::new(());

fn run(name: &str) {
    let _guard = SERIAL.lock().unwrap_or_else(|p| p.into_inner());
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../sim/scripts");
    let tmp = std::env::temp_dir().join(format!("ccsim-test-{}-{name}", std::process::id()));
    let opts = ccsim::Options { dir: tmp.join("sim"), out_dir: tmp.join("out"), ..Default::default() };
    let r = ccsim::script::run_file(&root.join(name), opts);
    let _ = std::fs::remove_dir_all(&tmp);
    if let Err(e) = r {
        panic!("{name}: {e}");
    }
}

#[test]
fn connect_and_ui() {
    run("01_connect_ui.ccsim");
}

#[test]
fn heat_and_home() {
    run("02_heat_home.ccsim");
}

#[test]
fn print_cube() {
    run("03_print_cube.ccsim");
}

#[test]
fn faults() {
    run("04_faults.ccsim");
}

#[test]
fn stress() {
    run("05_stress.ccsim");
}

#[test]
fn gcode_parity() {
    run("06_gcode_parity.ccsim");
}

#[test]
fn tmc_drivers() {
    run("07_tmc.ccsim");
}

#[test]
fn load_cell_probing() {
    run("08_loadcell.ccsim");
}

#[test]
fn load_cell_z_homing() {
    run("09_loadcell_home.ccsim");
}

#[test]
fn bed_mesh() {
    run("10_bed_mesh.ccsim");
}

#[test]
fn power_loss_resume() {
    run("11_power_loss.ccsim");
}

#[test]
fn probe_commands() {
    run("12_probe.ccsim");
}

#[test]
fn accelerometer_and_resonance() {
    run("13_resonance.ccsim");
}

/// Every script file has a test above.
#[test]
fn all_scripts_listed() {
    let src = include_str!("scripts.rs");
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../sim/scripts");
    for e in std::fs::read_dir(dir).unwrap() {
        let name = e.unwrap().file_name().to_string_lossy().to_string();
        if name.ends_with(".ccsim") {
            assert!(src.contains(&format!("run(\"{name}\")")), "add a test for {name}");
        }
    }
}

#[test]
fn calibrate_screen() {
    run("14_calibration.ccsim");
}
