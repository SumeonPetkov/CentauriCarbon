# ccos — Rust host toolkit for the Elegoo Centauri Carbon

Foundation for replacing the Centauri Carbon's userspace (`/app/app`) with
your own Rust code, plus tools to edit the stock UI today. Background and
protocol details: [`docs/re/`](../docs/re/README.md).

![dashboard rendered by `ccos ui --png`](../docs/re/img/ccos_dashboard.png)

## Crates

| Crate | What |
|---|---|
| `cc-proto` | Klipper wire protocol with the printer's fixed 115-message id table |
| `cc-transport` | Links: UART/USB serial, R528 DSP shared-memory ring + rpmsg doorbell |
| `cc-res` | `widget.bin`, `translation.bin`, STM32 update images (read/write/CSV/JSON) |
| `cc-gui` | Framebuffer + touch GUI: canvas, TTF/CJK text, PNG, widgets, stock-screen preview |
| `ccos` | Binaries `ccos` (on the printer) and `ccres` (on your PC) |

## Build

```sh
cargo test --release                       # 39 tests, use the shipped resources in ../firmware
cargo build --release                      # host tools: target/release/ccres, ccos
rustup target add armv7-unknown-linux-musleabihf
cargo build --release --target armv7-unknown-linux-musleabihf
# -> target/armv7-unknown-linux-musleabihf/release/{ccos,ccres}: static ARM binaries
```

No cross C toolchain is needed (linked with `rust-lld`, see `.cargo/config.toml`).

## On your PC: `ccres`

```sh
R=../firmware/resources/e100
ccres widget list $R/widget.csv
ccres widget show $R/widget.csv 1            # decoded styles of window 1
ccres preview $R all previews/ --annotate     # every stock screen as PNG
ccres widget compile my_widgets.csv widget.bin
ccres tr decompile $R/translation.bin strings.csv
ccres mcu info ../firmware/resources/firmware/upgrade_sg.bin
ccres mcu pack out/klipper.bin upgrade_extruder.bin --version 1.2.4
ccres proto encode "get_uptime" 3
ccres proto decode 08 10 01 00 28 5e 9f 7e
```

## On the printer: `ccos`

Copy `ccos` and stop Elegoo's `app` (only one process can own the MCU links,
framebuffer and touchscreen). Reboot to go back to stock.

```sh
ccos probe all              # talk to DSP, toolhead and load-cell MCUs
ccos console toolhead       # raw Klipper commands
ccos power-cycle sg         # reset an STM32 board like the stock host does
ccos temps                  # bed + chamber temperature via DSP ADC (fresh DSP only)
ccos ui                     # touch dashboard on /dev/fb0 + /dev/input/event1
ccos ui --png out.png --font ../firmware/resources/e100/fonts/font.ttc   # render on a PC
```

**Hardware status:** the protocol, ring buffer and resource code are tested
against Elegoo's C implementation and shipped files. The device-facing parts
(DSP ioctls, framebuffer, touch, GPIO) follow the stock source but have **not
been run on a printer yet**. `ccos` deliberately never drives heaters or
motors.

## Writing your own UI

`cc_gui::ui::Ui` is immediate-mode: describe the screen every frame and react
to the return values. Start from `crates/ccos/src/dashboard.rs`:

```rust
ui.clear();
ui.temp_tile(Rect::new(80, 6, 126, 90), "Nozzle", cur, target);
if ui.button(Rect::new(362, 222, 104, 36), "Home") {
    // queue G28
}
```

`Canvas` also implements `embedded_graphics::DrawTarget`, so any
embedded-graphics shapes, charts or image decoders draw straight onto it.

Next steps toward a full printing host are in
[`docs/re/08-rust-host-roadmap.md`](../docs/re/08-rust-host-roadmap.md).
