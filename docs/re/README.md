# Elegoo Centauri Carbon — reverse-engineering notes

These notes describe how the Centauri Carbon (CC1, internal codename **E100**)
software actually works, based on Elegoo's GPL source drop in this repo. They
were checked against the code; anything not verified on real hardware is
marked as such.

| Document | What's in it |
|---|---|
| [01-architecture.md](01-architecture.md) | Chips, processes, threads, boot of the `app` binary, the C++ Klipper port |
| [02-hardware.md](02-hardware.md) | The three MCUs, full pin map, sensors, fans, heaters |
| [03-mcu-protocol.md](03-mcu-protocol.md) | Wire protocol, **fixed message ids**, hard-coded constants, the DSP shared-memory link |
| [04-storage-boot-update.md](04-storage-boot-update.md) | Filesystem layout, config files, firmware update formats (what is signed, what isn't) |
| [05-ui-resources.md](05-ui-resources.md) | `widget.bin` / `translation.bin` formats and how to edit the stock UI |
| [06-gcode.md](06-gcode.md) | Custom G-codes (M8xxx config editing, `ROOT`, strain-gauge homing…) |
| [07-building.md](07-building.md) | What can be built from this repo and what's missing |
| [08-rust-host-roadmap.md](08-rust-host-roadmap.md) | Plan for a full Rust replacement of the host software |
| [mcu-command-table.md](mcu-command-table.md) | All 115 MCU messages with their ids |

The Rust toolkit that comes out of this work lives in [`/ccos`](../../ccos).

## One-page summary

```
                    Allwinner R528 (sun8iw20) SoC
┌───────────────────────────────────────────────────────────────────────────┐
│  2x Cortex-A7 @ Linux 5.4.61 (Tina / OpenWrt, glibc)                      │
│  ┌─────────────────────────────────────────────────────────────────────┐  │
│  │ /app/app  (one C++ binary, this repo's firmware/)                   │  │
│  │   LVGL 8 UI ──┐   app/e100/*  (screens)                             │  │
│  │               ├── simplebus (pub/sub + request/response)            │  │
│  │   services ───┘                                                     │  │
│  │   klippy C++ port: gcode, toolhead, kinematics, input shaper,       │  │
│  │   heaters/PID, bed mesh, strain-gauge probing, print job, ...       │  │
│  │   chelper (C): serialqueue, stepcompress, itersolve, trapq          │  │
│  └──────┬─────────────────────────────┬──────────────────────┬────────┘  │
│         │ kbuf 16 KiB shared RAM      │ /dev/ttyACM0 1 Mbaud │ /dev/ttyS4 │
│         │ + rpmsg doorbell            │ (USB CDC)            │ 115200     │
│  ┌──────▼──────────────────────┐      │                      │            │
│  │ HiFi4 DSP (Xtensa), FreeRTOS │      │                      │            │
│  │ Klipper MCU firmware (dsp/)  │      │                      │            │
│  │ X/Y/Z steppers, TMC2209 UART │      │                      │            │
│  │ bed heater, fans, Z endstop  │      │                      │            │
│  └──────────────────────────────┘      │                      │            │
└────────────────────────────────────────┼──────────────────────┼────────────┘
                              ┌──────────▼─────────┐  ┌─────────▼──────────┐
                              │ STM32F401 toolhead │  │ STM32F401 load-cell │
                              │ extruder stepper,  │  │ board: 4x HX717     │
                              │ hotend, fans, LED, │  │ strain gauges under │
                              │ LIS2DW accel (SPI) │  │ the bed (Z probing) │
                              └────────────────────┘  └─────────────────────┘
```

**Key findings**

1. **It's Klipper, rewritten in C++.** Elegoo ported Klipper's Python host
   (`klippy`) to C++ (`firmware/core/klippy/`, ~85 "extras" modules) and runs it
   inside the same process as the touchscreen UI. The MCU side is upstream-ish
   Klipper C (`mcu/src`, `dsp/…/klipper_r528`).
2. **The main "MCU" is the SoC's own DSP.** X/Y/Z, bed heater, chamber fans and
   the TMC2209 UART all hang off R528 GPIOs driven by Klipper running on the
   HiFi4 DSP under FreeRTOS. The Linux side talks to it through a 16 KiB
   non-cached shared buffer plus an rpmsg "doorbell".
3. **Message ids are hard-coded.** The host does not use Klipper's identify
   dictionary; it has a fixed 115-entry table. The dictionaries baked into the
   MCU firmware are stale (they claim an STM32F446 @ 180 MHz from 2022). Clock
   frequencies (DSP 200 MHz, STM32 84 MHz) and PWM ranges are hard-coded too.
4. **MCU firmware updates are unsigned.** `upgrade_extruder.bin` /
   `upgrade_sg.bin` are a 32-byte header + MD5 + raw STM32 binary. You can
   rebuild and flash the toolhead and load-cell boards freely.
5. **The main OS update is signed.** `update.bin` → zip → `update.swu` is
   applied by `swupdate -k /etc/swupdate_public.pem` into A/B partitions.
   Replacing the Linux userspace therefore needs a shell on the printer (or
   eMMC access), not the official update path.
6. **The UI is data-driven.** Every screen is a row-per-widget table
   (`widget.csv` → `widget.bin`); all text is in `translation.csv` →
   `translation.bin`; icons are numbered PNGs. These can be edited without
   recompiling anything (see [05](05-ui-resources.md) and `ccres`).
7. **Not in the source drop:** the LAN/cloud protocol server (the SDCP
   websocket the slicer talks to), the AES step of the OTA decryptor, the DSP
   RTOS SDK and the cross toolchain.

## Two ways forward

| | **A. Mod the stock app** | **B. Your own host in Rust (`ccos`)** |
|---|---|---|
| UI changes | Edit `widget.csv` / `translation.csv` / PNGs, compile with `ccres`, copy to `/app/resources/` | Draw anything with `cc-gui`; full control |
| Printing logic | Keep Elegoo's tuned C++ Klipper | Must be (re)implemented — see roadmap |
| Effort | Hours | Months for full feature parity |
| Risk | Low; resources are data files | Low while you only *add* a binary; the stock app stays on disk |
| Needs | Root shell to copy files | Root shell to copy files; stop `app` while `ccos` runs |

A practical path is to do both: restyle the stock UI now, while growing `ccos`
until it can print.
