# Frequency input

The DOGS² v1 engineering sample has a dedicated frequency input on GPIO13.
The firmware uses the ESP32 pulse counter to count rising edges in a timed
window and calculates:

```text
frequency = edge count / elapsed time
```

The displayed value is cleared after the configured no-signal timeout. The
current implementation intentionally does **not** claim pulse-width, polarity,
edge-event, or duty-cycle measurement on this input.

Relevant implementation:

- `src/timing_input.c`
- `include/probe_config.h`
- `FREQUENCY_INPUT_GPIO`
- `SIGNAL_MISSING_MS`

The separate generator mode has its own adjustable frequency and duty cycle;
those output settings are not measurements made by this input.
