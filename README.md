# DOGS²

<p align="center">
  <img src="assets/photos/dogs2-working-prototype-front.png"
       alt="DOGS² v1 working engineering prototype"
       width="900">
</p>

**DOGS² (Dual Output Generator Supply Station)** is an open-source, standalone
board bring-up station. It combines two programmable power outputs, current and
voltage monitoring, a signal generator, and common embedded-interface tools in
one compact instrument.

This repository documents the current **v1 engineering samples**. The hardware
works and the firmware is under active development and tuning; published limits
and accuracy figures should therefore be treated as engineering-sample data,
not final product specifications.

## Repository structure

| Directory | Contents |
|---|---|
| [`hardware/`](hardware/) | Schematics, block diagram, editable design archive, PCB manufacturing files, BOMs, renders, and hardware known issues |
| [`mechanical/`](mechanical/) | Mechanical-design status and future enclosure files |
| [`software/`](software/) | Current ESP32 firmware, build configuration, tools, software documentation, and software known issues |

## Current capabilities

- two independently controlled power channels with CV/CC operation;
- voltage and current monitoring;
- standalone TFT display, encoder, and front-panel controls;
- square-wave generator;
- UART and RS485 terminal modes;
- CAN receive, filtering, and transmission;
- I2C sniffer and I2C master terminal;
- analog probe and frequency input;
- Bluetooth SPP command and telemetry connection.

The 1-Wire decoder and full LIN protocol support are still in development.

## Project status

Five v1 engineering samples have been assembled. Hardware characterization,
firmware development, documentation, enclosure work, and preparation for
external beta testing are ongoing.

Project page: [dogs2.smartmoto.asia](https://dogs2.smartmoto.asia)

## Working hardware

<table>
  <tr>
    <td width="33%">
      <img src="assets/photos/dogs2-working-prototype-top.png"
           alt="DOGS² v1 engineering prototype viewed from above">
    </td>
    <td width="33%">
      <img src="assets/photos/dogs2-signal-generator.jpg"
           alt="DOGS² signal generator mode">
    </td>
    <td width="33%">
      <img src="assets/photos/dogs2-rs485-test.png"
           alt="DOGS² during an RS485 test">
    </td>
  </tr>
  <tr>
    <td align="center">v1 engineering prototype</td>
    <td align="center">Signal generator</td>
    <td align="center">RS485 test</td>
  </tr>
</table>

<table>
  <tr>
    <td width="50%">
      <img src="assets/photos/dogs2-assembled-bottom.png"
           alt="DOGS² v1 assembled prototype viewed from below">
    </td>
    <td width="50%">
      <img src="assets/photos/dogs2-power-board-top-angle-closeup.jpg"
           alt="DOGS² v1 power board top-angle close-up">
    </td>
  </tr>
  <tr>
    <td align="center">Assembled prototype — bottom view</td>
    <td align="center">Power board — top-angle close-up</td>
  </tr>
</table>

## Documentation map

- Hardware overview: [`hardware/README.md`](hardware/README.md)
- Hardware schematics: [`hardware/schematic/`](hardware/schematic/)
- Firmware overview: [`software/README.md`](software/README.md)

## Licensing

The v1 hardware design is licensed under
[CERN-OHL-W-2.0](hardware/LICENSE). Firmware and other repository content will
receive their own explicit licences before a public release.
