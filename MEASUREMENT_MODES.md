# Logic_v1 measurement modes

This firmware is being debugged on an ESP32 Cheap Yellow Display board.
The current hardware wiring is:

- ADC voltage input: GPIO35 / ADC1_CH7.
- Digital timing input: GPIO22.
- Diagnostic IO26 output: High-Z by default; can be DAC, waveform or UART TX.
- Weak analog bias / future open-test output: GPIO27 through 300 kOhm.
- TFT backlight: GPIO21.

## 1. Static voltage mode

Static voltage mode is used when no digital edges are being detected on GPIO22.

The ADC path reads GPIO35 several times, averages the calibrated ADC voltage,
then reconstructs the probe-node voltage using the temporary external divider
scale:

```text
Vprobe = Vadc * ADC_INPUT_SCALE
ADC_INPUT_SCALE = (220k + 68k) / 68k = 4.235294
```

Current classification thresholds:

```text
LOW   Vprobe <= 0.8 V
HIGH  Vprobe >= 2.6 V
MID   between LOW and HIGH
```

The display shows:

```text
LOW/HIGH/MID
VOLT x.xx V  PP x.xx
```

`PP` is the peak-to-peak value seen inside the ADC sampling window. In this
temporary analog front end it is useful mainly as a noise/activity hint, not as
a calibrated oscilloscope reading.

## 2. Automatic open / continuity probing

Automatic open probing is enabled, but it is deliberately treated as a low
priority background test.

Normal voltage measurement is always made with GPIO27 driven LOW through
300 kOhm.  Approximately once per second, and only when IO22 has been quiet for
`OPEN_TEST_QUIET_MS`, the firmware performs a short synchronous test:

```text
GPIO27 LOW   -> measure probe voltage
GPIO27 HIGH  -> measure probe voltage again
GPIO27 LOW   -> immediately return to the normal condition
```

If the measured probe-node movement is at least `OPEN_TEST_DELTA_MV`, the input
is considered responsive to the weak injected signal.  That means the probe is
probably open/high-Z.  A real logic output should suppress this small current.

The OPEN decision is debounced:

```text
OPEN_DETECT_COUNT   consecutive visible test bursts are needed to enter OPEN
OPEN_RELEASE_COUNT  consecutive suppressed test bursts are needed to leave OPEN
```

This prevents the display from flickering between MID and OPEN because of ESP32
ADC noise or marginal wiring.

Important display rule: OPEN is no longer a "black screen" mode.  It is shown as
a normal state line, while the RGB LED remains off.  The bottom UART line also
remains visible.

The manual `f` command performs the same LOW/HIGH one-shot analog test and
returns:

```text
base=<normal voltage>, down=<GPIO27 low>, up=<GPIO27 high>, span=<difference>
```

## 3. Digital timing mode

Digital timing mode is used when GPIO22 receives edges.

The primary measurement peripheral is MCPWM Capture:

- both edge directions are timestamped;
- edge polarity is decided from the actual GPIO22 level after the capture event:
  HIGH means rising, LOW means falling;
- edge-to-edge intervals shorter than `MIN_EDGE_INTERVAL_US` are rejected as
  ringing/noise;
- rising-to-rising interval gives period and frequency;
- rising-to-falling interval gives HIGH time;
- falling-to-rising interval gives LOW time;
- duty is calculated as:

```text
duty = high_us / (high_us + low_us) * 100
```

The screen shows:

```text
PULSE
DIGITAL SIGNAL
FREQ ...
DUTY ...
EVENT ...
```

For signals below 1 Hz the `FREQ` and `DUTY` lines are hidden.  Very slow
signals are easier to understand as edge/pulse events than as a mostly stale
frequency/duty display.

Event display, held for `PULSE_EVENT_HOLD_MS`:

```text
EVENT UP   rising edge
EVENT DN   falling edge
PULSE P    positive pulse, LOW-HIGH-LOW
PULSE U    negative pulse, HIGH-LOW-HIGH
```

The current smooth font contains ASCII only, so the requested Cyrillic `П` is
temporarily displayed as ASCII `P`.  A real `П` requires adding a Cyrillic glyph
to the generated font table.

The ADC voltage line is intentionally hidden in this mode because the ADC samples
a waveform at arbitrary phases and can make the display jump between LOW, MID
and HIGH.

## 4. Signal-present / signal-missing decision

Important implementation detail: MCPWM Capture timestamps and `esp_timer` use
different clocks. They must not be compared directly.

The firmware therefore detects signal presence by watching the captured edge
counter from the normal task:

1. The capture callback increments `s_edge_count` on every edge.
2. `probe_update()` copies `s_edge_count`.
3. If the copied value changed since the previous update, the firmware stamps
   "last activity" using `esp_timer_get_time()`.
4. `signal_missing` is based on that `esp_timer` activity timestamp.

This avoids the false "frequency = 0" condition caused by comparing unrelated
timer domains.

## 5. Half-period / double-frequency protection

During tests, 1 kHz sometimes appeared as 2 kHz and 10 kHz sometimes appeared
as 20 kHz. That symptom means the firmware has accidentally treated a half-cycle
as a full cycle.

The current protection is:

1. MCPWM Capture still timestamps both edge directions.
2. The ISR samples GPIO22 immediately after the capture event.
3. If GPIO22 is HIGH, the edge is processed as rising.
4. If GPIO22 is LOW, the edge is processed as falling.
5. Very short intervals below `MIN_EDGE_INTERVAL_US` are ignored.

The period is updated only on rising-to-rising intervals, so a clean square wave
should not become double-frequency merely because both edges are being captured.

## 6. PCNT status

PCNT is initialized but is not currently used as the displayed frequency source.

During testing PCNT reported roughly three times the oscilloscope/MCPWM
frequency. Until that is isolated, MCPWM Capture is the trusted source for
frequency and duty.

## 7. IO26 diagnostic module

GPIO26 is a diagnostic output only.  By default it is High-Z:

```text
GPIO26 input enabled
GPIO26 output disabled
pull-up disabled
pull-down disabled
```

Only one IO26 mode can be active at a time.  Starting DAC, frequency output or
UART output first stops the previous IO26 peripheral owner.  Command `r` always
returns IO26 to High-Z.

Commands are accepted over USB UART and Bluetooth.  Multi-character commands are
line-oriented: send Enter/CR/LF after the command.  Single-character `h` and `r`
also work immediately.

```text
h          help
r          reset IO26 to High-Z

v0         DAC output, approximately 0 mV
v800       DAC output, approximately 800 mV
v1500      DAC output, approximately 1500 mV
v2700      DAC output, approximately 2700 mV
v3100      DAC output, approximately 3100 mV

f10-50     10 Hz, 50% duty
f100-50    100 Hz, 50% duty
f1000-25   1000 Hz, 25% duty
f10000-75  10000 Hz, 75% duty
f02-2      0.2 Hz, 2% duty
f02-98     0.2 Hz, 98% duty
f02-50     0.2 Hz, 50% duty

u115       UART2 TX on IO26, 115200 baud, sends "UARTB <counter>"
u9         UART2 TX on IO26, 9600 baud, sends "UARTB <counter>"
```

The DAC is ESP32 DAC2, so the voltage is only approximate and not a calibrated
precision source.  Sub-Hz frequency commands use `esp_timer`; 1 Hz and above use
LEDC PWM.

## 8. UART-on-the-same-pin experiment

UART mode is used to test whether IO22 can feed two ESP32 peripherals at the
same time:

- MCPWM Capture still receives IO22 for edge timing.
- UART A also receives on IO22.
- UART B is no longer enabled by default; use `u115` or `u9` to explicitly route
  UART2 TX to IO26.
- UART0 remains reserved for USB flashing, logs and command input.

Port assignment:

```text
UART A = UART1 RX on IO22
UART B = UART2 TX on IO26, only after u115/u9 command
```

Connect IO26 to IO22 to loop UART B into UART A. The display bottom line shows
the first 15 characters of the last completed line received by UART A plus the
UART A error counter:

```text
UARTB 123       E:0
```

`E` increments when the UART driver reports receive-side problems:

- frame error;
- parity error;
- RX FIFO overflow;
- RX ring-buffer full.

`BREAK` is intentionally not counted.  In this instrument IO22 is also the
logic-probe input, so shorting the probe to ground holds UART RX at LOW and
would otherwise make the UART error counter grow forever.  A framing error is
also ignored when the line is currently held LOW, for the same reason.

For the current 8N1/no-parity test, a wrong baud rate normally appears as bad
text and/or a growing `E` counter.

UART B test line format:

```text
UARTB <counter>\r\n
```
