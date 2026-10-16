# 02 — Hardware map

Source: `firmware/resources/e100/configs/printer.cfg`, `mcu/board/*/config/config`,
`firmware/core/klippy/serialhdl.cpp`. Pin names prefixed `stm32:` are on the
toolhead board, `strain_gauge_mcu:` on the load-cell board; everything else is
an R528 GPIO driven by the DSP.

## MCUs

| printer.cfg | Chip | Link | Clock (host constant) | PWM max | Pins/bank |
|---|---|---|---|---|---|
| `[mcu]` `mem_interface: mem_interface_DSP` | R528 HiFi4 DSP | shared memory + rpmsg | 200 MHz | 255 | 32 |
| `[mcu stm32]` | STM32F401xC, 24 MHz xtal, 256 KiB flash, 64 KiB RAM | `/dev/ttyACM0` USB CDC (VID 1d50 PID 614e), "1 Mbaud" | 84 MHz | 2 000 000 | 16 |
| `[mcu strain_gauge_mcu]` | STM32F401xC | `/dev/ttyS4` USART2 @ 115200 | 84 MHz | 2 000 000 | 16 |

Both STM32 boards: app at `0x0800C000` behind a 48 KiB bootloader
(`board/*/bootloader/bootloader.bin`, source in `mcu/bootloader/`, STM32Cube
"f4_bootloader", YMODEM receiver). Restart method `power`: at every connect
the host power-cycles each board through an R528 GPIO it drives **directly
from Linux** (`gpio_init/gpio_set_value`, not via the DSP) — toolhead `PE12`,
load-cell board `PG9` — then streams the update image over YMODEM before
starting the Klipper session (`MCU::mcu_identify`, `mcu.cpp`).

## Motion (corexy)

| Axis | step | dir | enable | Endstop | Notes |
|---|---|---|---|---|---|
| X | PE8 | !PE7 | !PE10 | TMC2209 StallGuard (`diag !PG4`) | 0.01246 mm/step @16 µsteps, 256.5 mm travel |
| Y | PE14 | !PE13 | !PE15 | StallGuard (`diag !PF6`) | 270 mm travel |
| Z | PG3 | !PG2 | !PG1 | PG8 (also bed-mesh probe pin) | 0.0025 mm/step, 258 mm travel |
| E | stm32:PC13 | stm32:PC14 | !stm32:PC15 | — | 52:10 gear, rotation_distance 28.8 |

TMC2209 drivers share one UART: DSP `tx_pin PE9`, `uart_pin PE6`, addresses
X=0, Z=1, Y=3. Extruder TMC2209 on stm32 `PC6/PC7`, address 3.

Limits: 500 mm/s (300 silent), 20 000 mm/s² accel, square-corner velocity 9.
Input shaper defaults: X mzv 54.8 Hz, Y zv 46.4 Hz.

## Thermal

| Heater/sensor | Pins | Sensor | Limits |
|---|---|---|---|
| Hotend | heater stm32:PB6, sensor stm32:PA3 | NTC 100K β4300 | max 335 °C, PID 24.86/2.70/57.18 |
| Bed | heater PG13, sensor PB14 | NTC 100K β3950 | max 125 °C, PID 46.50/1.45/372.04 |
| Chamber ("box") | sensor PB13 | NTC 100K β3950 | read-only |

Pull-ups default to 4.7 kΩ (`thermistor.cpp`).

## Fans, lights

| Name | Pin | Tach |
|---|---|---|
| Part cooling (`fan model`) | stm32:PB5 | stm32:PA0 |
| Hotend fan | stm32:PC8 | stm32:PA1 |
| Aux part fan (`model_helper_fan`) | PG17 | — |
| Chamber fan (`box_fan`) | PG18 | — |
| Mainboard fan | PG16 | PG6 |
| Toolhead LED | stm32:PC9 | |
| Chamber LED / box LED | PG15 | |
| USB-C LED | PC2 | |

## Sensors

* **Accelerometer**: LIS2DW12 on toolhead SPI1, CS stm32:PA4, 5 MHz, axes `x,z,-y`
  (configured as `[adxl345 X]` with `adxl_type: lis2dw12`).
* **Load cells**: four HX717 (24-bit ADC) half-bridges under the bed, read by
  the strain-gauge STM32 — clk/data pins PB13/PB14, PC7/PC8, PC6/PB15, PC9/PA8.
  Used for Z homing (`STRAINGAUGE_Z_HOME`), bed mesh probing and nozzle-contact
  detection; filtered with a Kalman filter (`strain_gauge_kalman_filter.cpp`,
  `tiny_ekf`).
* **Camera**: USB UVC via V4L2 (`hl_camera.c`), timelapse encoded with
  ffmpeg/x264.

## Display & input

* 480×272 LCD, `/dev/fb0`, 32-bpp (LVGL `LV_COLOR_DEPTH 32`), backlight via
  sunxi `dispdbg` debugfs.
* Touch: evdev `/dev/input/event1`, raw ABS_X/ABS_Y already in screen
  coordinates (no calibration in the stock driver).
* Beeper (`devices/beep.cpp`).
