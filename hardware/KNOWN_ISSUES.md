# DOGS2 v1 hardware known issues

These notes describe known issues in the current `v1` engineering-sample
hardware and the intended direction for the next board revision.

1. The U10 LM51772 reset needs to be pulled up to the input voltage by connecting pins 38 and 36 — there is a test point on the board.
2. Dirty DAC power — a separate 3.3 V LDO from Vin is needed — SE8633.
Remove capacitors or move them, drill out the via, solder the capacitors (don't forget — the jumper will also be drilled out), solder the regulator onto the DAC capacitors on top.
This can also be skipped, but then the ripple at output A will be 20 mV p-p.
3. Replace R66 3.6 kOhm with an RCR network: 1.8 + 0.1 µF to ground + 1.8 kOhm, and another 10k in parallel with one of the resistors. This also reduces ripple at output A.
4. Raise the TPS frequency to 600 kHz — resistor R10 33 kOhm instead of 100 kOhm.
5. Replace the TPS feedback loop compensation with R14 4.7 kOhm + C34 0.1 uF.
6. Remove R13 — the resistor on CDC.
7. On the control board, switch the BOOT pull-up resistor R12 from +3.3SW to +3.3 constant supply (next to R12 there is a programming connector pad with +3.3) — otherwise it won't turn on.
8. Electrolytic capacitors are needed: 220uF * 16V on the control panel board and 220uF * 35V on the power board; the footprints are there.
9. Turn off the input voltage monitoring LED LED3 and dim the +3.3 monitoring LED LED2 by moving the 12k resistor from R24 to R22 — there will be a dim standby backlight underneath.
