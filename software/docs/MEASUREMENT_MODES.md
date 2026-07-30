# DOGS² firmware modes

This document describes the current v1 engineering-sample build. It replaces
the earlier Logic_v1/Cheap Yellow Display notes.

## Power Source

The main screen controls both output channels and displays their measured
voltage and current. The firmware communicates with the power converters,
current monitors, DAC, and control-panel I/O over the main I²C bus. It also
handles channel enable, current limiting, status reporting, temperature,
input-voltage measurement, and output-voltage trim.

## Generator

The generator output uses the dedicated engineering-sample output pin.
Frequency, duty cycle, and output state are adjustable from the front panel.

## UART, LIN, and RS485

UART and RS485 provide configurable terminal operation and forward received
data over Bluetooth SPP. LIN currently uses the shared serial receive path and
its own baud/filter controls; protocol-aware LIN decoding remains in
development.

## CAN

The CAN mode uses the board transceiver and ESP32 TWAI controller. It supports
selectable bitrate, receive filtering, frame display and Bluetooth reporting,
plus frame transmission through Bluetooth commands.

## I²C

- **I²C Sniffer** passively captures traffic and applies an address filter.
- **I²C Master** provides scan and read/write commands at 100 kHz.

See [`I2C_MASTER_TERMINAL.md`](I2C_MASTER_TERMINAL.md) for the command syntax.

## Analog and frequency inputs

The analog path measures the external probe/input voltage and classifies logic
levels. The dedicated frequency input is counted by the ESP32 pulse counter.
These paths are described in [`ANALOG_CHANNEL.md`](ANALOG_CHANNEL.md) and
[`FREQUENCY_TIME.md`](FREQUENCY_TIME.md).

## In development

- 1-Wire protocol decoding;
- full protocol-aware LIN decoding;
- continued calibration, protection tuning, and hardware characterization.
