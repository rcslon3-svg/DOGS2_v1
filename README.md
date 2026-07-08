# Logic_v2

ESP32-WROOM logic probe / dual PSU firmware forked from Logic_v1.

Pins:
- GPIO35: ADC input
- GPIO27: weak continuous test square wave through 300 kOhm
- GPIO22: timing input and UART RX
- GPIO26: diagnostic output only, owned by io26_diag.c
- ST7789 1.9" 170x320 SPI TFT:
  - SCL/SCLK: GPIO15
  - SDA/MOSI: GPIO2
  - RES: GPIO4
  - DC: GPIO16
  - CS: GPIO17
  - BLK: GPIO5
- Encoder moved off GPIO5 because BLK uses it:
  - A: GPIO13
  - B: GPIO12
  - encoder button: GPIO14
  - external UI: GPIO18, same action as encoder button
  - all inputs use ESP32 internal pull-ups
  - one button advances through values and editable digits

Commands:
- h: help
- r: IO26 High-Z
- v<mV>: DAC output on IO26
- f<freq>-<duty>: test square wave on IO26
- u115 / u9: UART TX test on IO26
