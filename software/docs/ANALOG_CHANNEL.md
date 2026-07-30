# Analog input

The current firmware reads the DOGS² v1 engineering-sample analog path through
the ESP32 ADC on GPIO36. The board-specific divider is selected at compile time
by `LOGIC_V2_BOARD_REV`.

For the engineering sample, the divider model in `include/probe_config.h` uses:

```text
VIN = VADC × 11.87234
```

The channel reports measured input voltage and classifies the probe state using
the current firmware thresholds:

```text
LOW          ≤ 0.8 V
HIGH         ≥ 2.6 V
OVERVOLTAGE  ≥ 15 V
```

Intermediate readings are reported as undefined/high-impedance according to
the active measurement state. ADC sampling, averaging, display hysteresis, and
the exact thresholds are defined in `include/probe_config.h`; that file is the
source of truth while calibration continues.

The old document described a temporary Logic_v1/CYD resistor network and
GPIO27 injection test. That wiring does not describe the current v1
engineering sample and has been removed from this documentation.
