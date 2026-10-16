# Elegoo Centauri Carbon — reverse-engineering notes

These notes describe how the Centauri Carbon (CC1, internal codename **E100**)
software actually works, based on Elegoo's GPL source drop in this repo. They
were checked against the code and cross-checked against the community
research in [OpenCentauri](https://github.com/OpenCentauri/OpenCentauri)
(their docs site, `dsp-to-serial` and `mcu-flasher` tools). Anything not
verified on real hardware is marked as such.

| Document | What's in it |
|---|---|
| [01-architecture.md](01-architecture.md) | Chips, OS, partitions, processes, threads, the C++ Klipper port |
| [02-hardware.md](02-hardware.md) | The three MCUs, full pin map, sensors, fans, heaters, screen |
| [03-mcu-protocol.md](03-mcu-protocol.md) | Wire protocol, **fixed message ids**, hard-coded constants, the DSP shared-memory link |
| [04-storage-boot-update.md](04-storage-boot-update.md) | Filesystem, config files, update formats (signed vs. unsigned), getting a shell |
| [05-ui-resources.md](05-ui-resources.md) | `widget.bin` / `translation.bin` formats and how to edit the stock UI |
| [06-gcode.md](06-gcode.md) | Custom G-codes (M8xxx config editing, `ROOT`, strain-gauge homing…) |
| [07-building.md](07-building.md) | What can be built from this repo and what's missing |
| [08-rust-host-roadmap.md](08-rust-host-roadmap.md) | `ccos`: what exists, design choices, what a full printing host still needs |
| [mcu-command-table.md](mcu-command-table.md) | All 115 MCU messages with their ids |

The Rust code that comes out of this work lives in [`/ccos`](../../ccos):
`ccos` (the lightweight on-printer host with an English UI compiled into the
binary) and `ccres` (PC tool for the stock UI resources and MCU images).

## One-page summary

```
                 Allwinner R528-S3 (sun8iw20) · 128 MB DDR3 · 8 GB eMMC
┌───────────────────────────────────────────────────────────────────────────┐
│  2x Cortex-A7 · Tina Linux (OpenWrt-based) · kernel 5.4.61 · glibc 2.23   │
│  ┌─────────────────────────────────────────────────────────────────────┐  │
│  │ /app/app  (one C++ binary, this repo's firmware/)                   │  │
│  │   LVGL 8 UI ──┐   app/e100/*  (screens)                             │  │
│  │               ├── simplebus (pub/sub + request/response)            │  │
│  │   services ───┘   web UI + SDCP websocket (:3030, not in this repo) │  │
│  │   klippy C++ port: gcode, toolhead, kinematics, input shaper,       │  │
│  │   heaters/PID, bed mesh, strain-gauge probing, print job, ...       │  │
│  │   chelper (C): serialqueue, stepcompress, itersolve, trapq          │  │
│  └──────┬─────────────────────────────┬──────────────────────┬────────┘  │
│         │ kbuf 16 KiB shared RAM      │ /dev/ttyACM0         │ /dev/ttyS4 │
│         │ + rpmsg doorbell            │ (USB CDC)            │ 115200     │
│  ┌──────▼──────────────────────┐      │                      │            │
│  │ HiFi4 DSP (Xtensa), FreeRTOS │      │                      │            │
│  │ Klipper MCU firmware (dsp/)  │      │                      │            │
│  │ X/Y/Z steppers, TMC2209 UART │      │                      │            │
│  │ bed SSR, fans, light, Z stop │      │                      │            │
│  └──────────────────────────────┘      │                      │            │
└────────────────────────────────────────┼──────────────────────┼────────────┘
                             ┌───────────▼─────────┐  ┌─────────▼───────────┐
                             │ STM32F402 toolhead  │  │ STM32F402 bed board  │
                             │ extruder, hotend,   │  │ 4 load cells (HX711/ │
                             │ fans, LED, LIS2DW12 │  │ HX717) under the bed │
                             └─────────────────────┘  └──────────────────────┘
```

**Key findings**

1. **It's Klipper, rewritten in C++.** Elegoo ported Klipper's Python host
   (`klippy`) to C++ (`firmware/core/klippy/`, ~85 "extras" modules) and runs it
   inside the same process as the touchscreen UI. The MCU side is Klipper
   v0.9.1-616 (`28f60f7e`) with Elegoo changes (`mcu/src`, `dsp/…/klipper_r528`).
2. **The main "MCU" is the SoC's own DSP.** X/Y/Z, bed heater SSR, chamber fans,
   light and the TMC2209 UART all hang off R528 GPIOs driven by Klipper running
   on the HiFi4 DSP under FreeRTOS. Linux talks to it through a 16 KiB
   non-cached shared buffer plus an rpmsg "doorbell". The DSP accepts a host
   **once per DSP boot**.
3. **Message ids are hard-coded.** The host does not use Klipper's identify
   dictionary; it has a fixed 115-entry table. The dictionaries baked into the
   MCU firmware are stale (they claim an STM32F446 @ 180 MHz from 2022). Clock
   frequencies (DSP 200 MHz, STM32 84 MHz) and PWM ranges are hard-coded too.
4. **MCU firmware updates are unsigned.** The toolhead/bed images are a 32-byte
   header + MD5 + raw STM32 binary, pushed by YMODEM to Elegoo's bootloader at
   every boot. You can rebuild and flash those boards freely.
5. **The main OS update is encrypted and signed.** `update.bin` is AES-encrypted
   (key published by OpenCentauri); inside, `update.swu` is applied by
   `swupdate -k /etc/swupdate_public.pem` into A/B partitions. The practical way
   to get root is OpenCentauri's patched firmware (SSH `root` / `OpenCentauri`).
6. **The stock UI is data-driven.** Every screen is a row-per-widget table
   (`widget.csv` → `widget.bin`); all text is in `translation.csv` →
   `translation.bin`; icons are numbered PNGs (see [05](05-ui-resources.md)).
7. **Not in the source drop:** the LAN/cloud protocol server (SDCP websocket on
   port 3030, UDP discovery on 3000), the AES step of the OTA decryptor, the
   DSP RTOS SDK and the cross toolchain.

## Ways forward

| | **Stock + OpenCentauri patched** | **COSMOS** (OpenCentauri) | **`ccos`** (this repo) |
|---|---|---|---|
| What runs | Elegoo's `app`, patched | Upstream Klipper/Kalico (Python) + Moonraker + web UI | One native Rust binary, ~600 KB |
| UI | Stock (restyle with `ccres`) | Their screen UI + Mainsail/Fluidd | English UI compiled from `ccos/ui/*.toml` |
| Prints today | Yes | Yes | **No** — sensors, light, MCU tools only |
| Resource use | Stock | Heavy for this board ("very little overhead" per their docs) | Minimal: no interpreter, no parsing at runtime |
| Effort to extend | Patching binaries | Klipper config/macros | Rust; roadmap in [08](08-rust-host-roadmap.md) |

`ccos` is the route for a host that is as light as possible and fully yours;
until its printer core exists, COSMOS or the patched stock firmware are what
actually print. `ccos` runs on top of the OpenCentauri patched firmware: copy
the binary over SSH, stop `app`, run it. Rebooting brings the stock app back.
