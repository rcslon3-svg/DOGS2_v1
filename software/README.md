# DOGS² firmware

This directory contains the current firmware for the DOGS² v1 engineering
sample. It targets an ESP32-WROOM module and uses ESP-IDF through PlatformIO.

## Implemented modes

| Display mode | Current implementation |
|---|---|
| Power Source | Two output setpoints, output enable, CV/CC status, voltage/current monitoring, input voltage, temperature, and frequency |
| Generator | Square-wave frequency, duty cycle, and output control |
| UART | Configurable serial terminal with Bluetooth forwarding |
| LIN | Configurable serial receive path and filter UI; full LIN decoder is in development |
| 1-Wire | UI placeholder; decoder is in development |
| RS485 | Configurable half-duplex terminal |
| CAN | Configurable bitrate and filter, receive display, Bluetooth reporting, and frame transmission |
| I2C Sniffer | Passive capture with address filter |
| I2C Master | 100 kHz scan, register/raw read, and register/raw write commands |
| Settings | Protection thresholds and user settings |

Bluetooth SPP exposes commands and telemetry without consuming another USB
port. The default device name is `DOGS2_A001`.

## Build

Install [PlatformIO](https://platformio.org/), open this directory as the
project root, and run:

```text
pio run
```

The default environment is `logic_v2_engineering_sample`. The latest committed
firmware was build-tested with this environment before the repository
reorganization.

## Layout

| Path | Purpose |
|---|---|
| `include/` | Public module headers and board configuration |
| `src/` | ESP-IDF application and drivers |
| `tools/` | Font and splash-image generation helpers |
| `assets/` | Source/preview artwork used by the firmware |
| `docs/` | Focused implementation notes and terminal command reference |

Start with [`docs/MEASUREMENT_MODES.md`](docs/MEASUREMENT_MODES.md) for a
high-level map of the running firmware.
