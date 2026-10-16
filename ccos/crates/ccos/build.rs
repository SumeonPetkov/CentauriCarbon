//! Compiles ../../ui (theme, screens, fonts, icons) into static Rust tables.
fn main() {
    if let Err(e) = cc_ui_codegen::build("../../ui") {
        eprintln!("\nUI build error:\n  {e}\n");
        std::process::exit(1);
    }
}
