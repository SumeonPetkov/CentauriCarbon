# 08 — Roadmap: a Rust host for the Centauri Carbon

"Your own OS" realistically means **your own userspace** on the existing
Linux kernel: the kernel, bootloader, DSP firmware and STM32 firmware already
do their jobs and are either signed, proprietary-toolchain, or plain Klipper.
Replacing `/app/app` gives full control of motion, UI and networking while
keeping the platform. `ccos/` is the start of that.

## What exists now (`ccos/`)

| Crate | Status |
|---|---|
| `cc-proto` | Complete: framing, CRC, VLQ, fixed id table, encode/decode of all 115 messages, clock helpers. Byte-exact against Elegoo's C. |
| `cc-transport` | Serial (termios) and DSP shared-memory link. Ring logic tested against a simulated DSP; **device ioctls untested on hardware**. |
| `cc-res` | `widget.bin`, `translation.bin`, MCU update images: parse/build/CSV/JSON, byte-exact round-trips of shipped files. |
| `cc-gui` | Canvas (embedded-graphics `DrawTarget`), TTF/TTC text with CJK, PNG, rounded shapes, fbdev, evdev, immediate-mode widgets, stock-screen preview. |
| `ccos` (bin) | `probe`, `console`, `temps`, `power-cycle`, `ui` (touch dashboard / PNG render). |
| `ccres` (bin) | PC-side resource/firmware/protocol tool. |

![ccos dashboard](img/ccos_dashboard.png)

## What a printing host still needs

In dependency order. Each step is testable on its own.

1. **On-device validation of the links** — run `ccos probe all` with the stock
   app stopped; fix whatever the real `/dev/dsp_debug` / kbuf driver does
   differently from the source.
2. **Windowed, clock-scheduled send queue** — port `serialqueue.c` semantics:
   `receive_window` 192 bytes, retransmit on NAK/timeout, per-command
   `min_clock`/`req_clock`, notify ids. Clock sync (`clocksync.cpp`):
   periodic `get_clock`, linear regression → `ClockEstimate`.
3. **MCU configuration** — build the `config_*` list per MCU from a config file
   (keep `printer.cfg` syntax so stock calibrations carry over), CRC it,
   `allocate_oids`/`finalize_config`, handle `is_config`/`crc` on reconnect.
4. **Heaters + safety first** — ADC → thermistor → PID → `queue_digital_out`/
   `queue_pwm_out`, plus `verify_heater` runaway checks and MCU-side
   `max_duration`. Nothing else should be enabled before this is solid.
5. **Steppers** — `config_stepper`, `queue_step`, `set_next_step_dir`,
   `reset_step_clock`; a step compressor equivalent to `stepcompress.c`
   (this is the hot path: write it in Rust with no allocation per step, or
   reuse the C file via `cc` + FFI first and port later).
6. **Kinematics & planning** — trapezoid queue (`trapq.c`), lookahead
   (`toolhead.cpp`), CoreXY `itersolve`, extruder with pressure advance, input
   shaper (`kin_shaper.c`). Again FFI to Elegoo's C first is a valid shortcut.
7. **TMC2209 over UART** (`tmcuart_send`), StallGuard homing on X/Y,
   `trsync`/`endstop_home` for Z.
8. **Load-cell probing** — the hard, Centauri-specific part: HX717 streaming
   from the strain-gauge MCU, filters (`strain_gauge*.cpp`, `filter.cpp`,
   `tiny_ekf`), trigger detection, bed mesh with bicubic/lagrange interpolation.
   Port by reading Elegoo's C++ closely.
9. **G-code front end** — parser, `virtual_sdcard`, pause/resume, power-loss
   resume (`break_save`), print stats, thumbnails from G-code headers.
10. **Network** — the slicer protocol server isn't in the source drop. Either
    implement Moonraker's API (works with Orca/Mainsail/Fluidd), or
    reverse-engineer Elegoo's SDCP from the slicer side.
11. **Camera/timelapse** — V4L2 capture + ffmpeg (already on the device).

## Performance notes

* The stock host runs the printer loop as one `SCHED_RR` thread plus a thread
  per MCU. In Rust, keep the step generation on a dedicated real-time thread
  (`sched_setscheduler`), pin it with `sched_setaffinity` to one A7 core, and
  run UI/network on the other.
* The UI in `cc-gui` redraws the full 480×272 frame (~520 KB) per frame; at
  30 fps that is trivial for the A7, but add dirty-rect tracking if you add
  heavy content.
* Build with `lto = "fat"`, `codegen-units = 1`, `target-cpu=cortex-a7`
  (already set in `ccos/`).

## Safety

Heaters and motion are physical. Until steps 2–4 are done and tested, `ccos`
never configures heaters or steppers — the only output it configures is ADC
sampling (`ccos temps`). Keep it that way while experimenting, and always keep
a way back to the stock `app` (it stays on disk; a reboot restores it unless
you change the init scripts).
