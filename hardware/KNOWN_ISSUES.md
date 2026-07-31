# DOGS2 v1 hardware known issues

These notes describe known issues in the current `v1` engineering-sample
hardware and the intended direction for the next board revision.

1. The LM reset rail should be tied to the input voltage.
2. The DAC supply is noisy and needs a dedicated 3.3 V LDO from `Vin` such as `SE8633`. The current layout also needs rework around the DAC capacitors and the jumper/via area to make that regulator retrofit practical.
3. Replace the 3.6 kOhm resistor at the DAC output with a `1.8 kOhm + 0.1 uF to ground + 1.8 kOhm` network.
4. Raise the TPS switching frequency to at least 600 kHz by using a 33 kOhm resistor instead of 100 kOhm.
5. Enable frequency doubling when the converter enters buck-boost mode.
6. Output A rings under load. Re-check the channel with the updated compensation network because the compensation still needs tuning.
7. The current TPS compensation target is `4.7 kOhm + 0.1 uF`.
8. On the control board, move the `BOOT` pull-up resistor `R12` from `+3.3SW` to the always-on `+3.3` rail.
9. Add a bulk electrolytic capacitor in the `100-470 uF` range to the control-panel power input.
10. The TPS current limit behaves incorrectly: it trips below the setpoint, and below 0.2 A it drops into `CC` even without a load. Recalculate the shunt for a 1.5 A maximum; about 30 mOhm is the current estimate.
11. The LM channel likely needs a larger shunt for more accurate output-current measurement. The working target is 5 A maximum and 3 A nominal.
