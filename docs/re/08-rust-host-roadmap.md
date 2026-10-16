# 08 — `ccos`: a lightweight native host

"Your own OS" realistically means **your own userspace** on the existing
platform: the kernel, bootloader, DSP firmware and STM32 firmware already do
their jobs and are either signed, built with a proprietary toolchain, or plain
Klipper. Replacing `/app/app` gives full control of motion, UI and networking
while keeping the platform. `ccos/` is that replacement, built to be as light
as possible on a 2x Cortex-A7 / 128 MB board.

## Design rules

1. **Nothing is interpreted at runtime.** No Python, no scripting, no config
   parser in the hot path. The UI is described in `ccos/ui/*.toml` but
   compiled into the binary by `build.rs`; MCU message tables are compiled from
   JSON into static arrays.
2. **Nothing is allocated per frame or per message.** Frames, messages, text
   buffers and the frame decoder are fixed-size values. The only large
   allocation is one 480×272 back buffer (510 KiB), made once.
3. **Nothing is redrawn unless it changed.** The UI engine hashes each live
   widget's content; only changed rectangles are redrawn and copied to
   `/dev/fb0`. When idle, the main loop sleeps in `poll()` on the touch device.
4. **Only `libc` ships.** Fonts are pre-rasterised to 4-bit glyphs, icons are
   pre-converted (single-colour ones to 4-bit masks), so no font engine, PNG
   decoder, TOML or JSON parser ends up on the printer.
5. **One static binary.** musl, LTO, `panic = abort`, `target-cpu=cortex-a7`.
   It runs regardless of the printer's glibc version.

Measured: the ARMv7 release binary is **~600 KB** static, of which **~45 KB**
is the whole UI (4 fonts with 110 glyphs each, 18 icons). `opt-level = "s"` would only save 7%, so
the build stays at `-O3`. Expected resident memory on the printer is about
1 MB (the back buffer plus small fixed buffers per MCU); not yet measured on
hardware.

## What exists now

| Crate | Ships to printer | Status |
|---|---|---|
| `cc-proto` | yes | Framing, CRC, VLQ, the fixed 115-message table as static arrays, zero-allocation encode/decode, clock helpers. Byte-exact against Elegoo's C (test vectors generated from `msgblock.c`). |
| `cc-transport` | yes | Serial (termios; hang-up detection) and the DSP shared-memory link (once-per-boot handshake, remoteproc restart, kbuf alloc/free, DSP log). Ring logic tested against a simulated DSP; serial tested on a pty; **DSP device ioctls untested on hardware**. |
| `cc-ui` | yes | Canvas, baked-font text, icons, framebuffer (dirty rects), evdev touch, screen engine, tiny PNG writer. |
| `cc-ui-codegen` | no (build only) | `ui/*.toml` + TTF + PNG → static Rust. Validates everything with readable errors. |
| `ccos` | yes | The app: touchscreen UI, MCU worker thread (bed/chamber/nozzle thermistors, chamber light, hotend fan, probe, power-cycle), CLI tools. End-to-end tested against a fake Klipper MCU on a pty. |
| `ccres`, `cc-res`, `cc-gui` | no (PC tools) | Stock UI resources (`widget.bin`, `translation.bin`), MCU update images, protocol encode/decode, stock-screen previews. |

![home](img/ccos_home.png) ![control](img/ccos_control.png)
![tools](img/ccos_tools.png) ![system](img/ccos_system.png)

`ccos` drives only **read-only sensors, the chamber light and the hotend
cooling fan** (on from 50 °C, off below 45 °C, like the stock `heater_fan`)
today. G-code buttons are wired through but report "not executed" until a
printer core exists.

## Compared with COSMOS

OpenCentauri's COSMOS already runs upstream Klipper/Kalico on the stock board
and prints well; its docs warn that the board has "very little overhead" left
for anything else. `ccos` takes the opposite trade-off: a single native process
with no interpreter, built for minimal CPU and RAM. That makes it attractive
for a fast, responsive printer, but it means re-implementing what Klipper does.
Until `ccos` prints, use COSMOS or the patched stock firmware for real jobs.

## What a printing host still needs

In dependency order. Each step is testable on its own.

1. **On-device validation of the links** — run `ccos probe all` with the stock
   app stopped. Expect fixes around the DSP restart path (where remoteproc loads
   the image from) and the bootloader timing of the STM32 boards.
2. **Windowed, clock-scheduled send queue** — port `serialqueue.c` semantics:
   `receive_window` 192 bytes, retransmit on NAK/timeout, per-command
   `min_clock`/`req_clock`, notify ids. Clock sync (`clocksync.cpp`):
   periodic `get_clock`, linear regression → `ClockEstimate`.
3. **MCU configuration from a config file** — keep `printer.cfg` syntax so
   stock calibrations carry over; compute the config CRC; handle
   `is_config`/`crc` on reconnect.
4. **Heaters + safety first** — ADC → thermistor → PID → `queue_digital_out` /
   `queue_pwm_out` with MCU-side `max_duration`, plus `verify_heater` runaway
   checks. Nothing else should be enabled before this is solid; the bed is
   mains-powered through an SSR.
5. **Steppers** — `config_stepper`, `queue_step`, `set_next_step_dir`,
   `reset_step_clock`; a step compressor equivalent to `stepcompress.c`. This is
   the hot path: run it on a `SCHED_FIFO` thread pinned to one A7 core, with
   no allocation. Linking Elegoo's C (`stepcompress.c`, `itersolve.c`,
   `trapq.c`, `kin_*.c`) through FFI is a valid first step.
6. **Kinematics & planning** — trapezoid queue, lookahead (`toolhead.cpp`),
   CoreXY, extruder with pressure advance, input shaper (`kin_shaper.c`).
7. **TMC2209 over UART** (`tmcuart_send`), StallGuard homing on X/Y,
   `trsync`/`endstop_home` for Z.
8. **Load-cell probing** — the hard, Centauri-specific part: HX711/HX717
   streaming from the bed MCU, filters (`strain_gauge*.cpp`, `filter.cpp`,
   `tiny_ekf`), trigger detection, bed mesh with bicubic/lagrange interpolation.
   Port by reading Elegoo's C++ closely; COSMOS's load-cell work is a second
   reference.
9. **G-code front end** — parser, file printing, pause/resume, power-loss
   resume (`break_save`), print stats, thumbnails from G-code headers.
10. **Network** — Moonraker-compatible HTTP/websocket API (works with
    OrcaSlicer, Mainsail, Fluidd) and/or SDCP (websocket on :3030, UDP
    discovery on :3000; documented in OpenCentauri `docs/software/api.md`).
11. **Camera/timelapse** — V4L2 MJPEG passthrough is cheap; avoid re-encoding
    on the A7.

## Safety

Heaters and motion are physical. Until steps 2–4 are done and tested, `ccos`
never configures heaters or motors — the only outputs it configures are the
chamber light and the hotend cooling fan. Keep it that way while
experimenting, and keep a way back to the stock `app` (it stays on disk; a
reboot restores it unless you change the init scripts).
