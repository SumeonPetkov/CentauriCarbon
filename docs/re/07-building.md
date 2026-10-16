# 07 — What can be built from this repo

| Component | Buildable here? | Needs |
|---|---|---|
| `firmware/` (`app`) | With a toolchain | `arm-openwrt-linux-gnueabi` g++ from the Allwinner Tina SDK for R528 (glibc). `CMakeLists.txt` hard-codes `arm-openwrt-linux-gcc/g++`; the ARM `.so`/`.a` libraries are in `firmware/lib/` (some removed from git in this fork — see `.gitignore`). `lib/ubToolenvLib.a` is prebuilt. |
| `firmware/tools/widget`, `tools/tr` | Yes, host gcc | `gcc -o widget main.c widget.c` (same for `tr`). Replaced by `ccres`. |
| `mcu/` (STM32 Klipper) | Yes | `arm-none-eabi-gcc`, `make`; `./build.sh -v 1.2.4 -t extruder` (or `-t sg`) → `out/upgrade_<t>_<v>.bin` + `_full_pack.bin`. `add_magic.c` needs OpenSSL dev headers. |
| `mcu/bootloader/` | With STM32Cube/Keil | Keil MDK project (`MDK-ARM/`), prebuilt `bootloader.bin` in `mcu/board/*/bootloader/`. |
| `dsp/` (HiFi4 Klipper) | **No** | Allwinner Tina RTOS SDK (`rtos-hal`, FreeRTOS Xtensa port, `aw-alsa-lib`, `console.h`, `imgdts.h`…) and Cadence Xtensa XCC with a HiFi4 core config — proprietary, not in the repo. Only `projects/r528/dsp0` sources and `defconfig` are here. |
| `ccos/` (Rust) | Yes | `rustup target add armv7-unknown-linux-musleabihf`, then `cargo build --release --target armv7-unknown-linux-musleabihf`. Uses `rust-lld` (no cross C toolchain needed); produces static ~750 KB binaries. |

## Notes on the C++ build

* Compile flags: `-std=c++14 -O3 -g -pthread`, defines `Q_OS_LINUX`,
  `MG_ARCH=MG_ARCH_UNIX`, `KVS_USE_OPENSSL`; `ENABLE_MANUTEST` adds factory
  test code (`manu_test/` — not present).
* `config.h` selects features (`CONFIG_SUPPORT_TLP` timelapse, `CONFIG_SUPPORT_AIC`
  AI camera = 0, camera, network, OTA, `MCU_UPDATE`).
* `jenkins.h` holds the version string shown in the UI (`JENKINS_VERSION`).
* Switch model by swapping `CMakeLists.txt.e100` / `.e100_lite`.

## Deploying a rebuilt `app`

With root access: back up `/app/app`, copy the new binary, reboot. The binary
links against the libraries already in the root filesystem, so it must be
built with the matching Tina toolchain and library versions.
