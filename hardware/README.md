# DOGS² v1 hardware

This directory contains the source, reference, and manufacturing files for the
current **DOGS² v1 engineering sample**.

Parameters are still being tuned and verified. The files describe the boards
that were built; they are not yet a production release.

## System architecture

[![DOGS² v1 hardware block diagram](docs/DOGS2_v1_block_diagram.png)](docs/DOGS2_v1_block_diagram.pdf)

The diagram separates the power board from the control board and shows the
power paths, two programmable outputs, measurements, processor, controls, probe
connections, and digital-interface drivers.

- [View the block diagram as PDF](docs/DOGS2_v1_block_diagram.pdf)
- [Edit the block diagram in draw.io](design/DOGS2_v1_block_diagram.drawio)
- [View the complete schematic](docs/DOGS2_v1_schematic.pdf)

## Directory contents

### `design/`

| File | Purpose |
|---|---|
| `DOGS2_v1_easyeda_project.zip` | Complete editable EasyEDA project archive |
| `DOGS2_v1_block_diagram.drawio` | Editable block-diagram source |

### `docs/`

| File | Purpose |
|---|---|
| `DOGS2_v1_block_diagram.pdf` | Publication-quality system block diagram |
| `DOGS2_v1_block_diagram.png` | GitHub preview generated from the PDF |
| `DOGS2_v1_schematic.pdf` | Power-board and control-panel schematics |
| `DOGS2_v1_known_issues_ru.txt` | Known v1 issues and planned corrections, in Russian |

### `manufacturing/`

- `control_panel/`: Gerbers, BOM, and pick-and-place data for the control panel.
- `power_board/`: Gerbers, BOM, and pick-and-place data for the power board.
- `DOGS2_v1_system_bom.xlsx`: system-level BOM for boards, display, enclosure
  parts, and purchased items.

### `renders/`

Top/bottom power-board and front/rear control-panel renders.

## Revision and manufacture

All files in this directory belong to hardware revision **v1**. Before ordering
boards, read [`docs/DOGS2_v1_known_issues_ru.txt`](docs/DOGS2_v1_known_issues_ru.txt).
Manufacturing parameters have intentionally not been prescribed here: use the
requirements contained in the Gerber packages and confirm them with the chosen
PCB assembler.

## License

The DOGS² v1 hardware design files are licensed under the
**CERN Open Hardware Licence Version 2 — Weakly Reciprocal
(CERN-OHL-W-2.0)**. See [`LICENSE`](LICENSE).
