# STM32 F450 module-carrier PCB — Rev A

This folder is the KiCad 10 prototype package for the module carrier
used by the STM32 flight-controller firmware.

## Open and inspect

- `stm32_f450_carrier_rev_a.kicad_pro` — KiCad project.
- `stm32_f450_carrier_rev_a.kicad_pcb` — routed two-layer PCB.
- `board-preview.png` and `board-top.png` — rendered board views.
- `assembled-preview.png` — populated view with a published Blue Pill model
  and illustrative sensor module models.
- `3dmodels/carrier_assembly.wrl` — portable assembly model attached to J1;
  keep this folder beside the board file when opening KiCad's 3D Viewer.
- `board-layout.svg` — printable 1:1 fit-check drawing.
- `pinmap.csv` — connector-to-MCU mapping.
- `BOM.csv` — assembly bill of materials.
- `drc.rpt` — KiCad DRC result (zero violations; zero unconnected pads).
- `functional-schematic.svg` — firmware-level wiring reference from the source repository.
- There is no native `.kicad_sch` schematic or ERC report yet.
- `fabrication/` — Gerbers, plated/non-plated drill files, and drill maps.

## Fabrication assumptions

- 2 layers, FR-4, 1.6 mm thickness, 1 oz copper.
- Default finish and solder-mask color are acceptable.
- Board outline is 90 × 70 mm.

## Prototype warning

The preview images use the complete published YAAJ Blue Pill CAD assembly,
including sockets, USB, jumpers, components and SWD header. That third-party
model is not bundled in Git because its redistribution license is unconfirmed.
See `MODEL_SOURCES.md` for the optional download and placement instructions.
Without it, KiCad will omit the Blue Pill body from the 3D view; copper and
fabrication outputs are unaffected. The sensor models remain approximate illustrations.
The revised placement removes the modeled overlap and sensor overhang.
Verify real module envelopes and orientation before manufacture; copper DRC
does not establish module compatibility. Previous exports used incorrect
Blue Pill socket spacing and pin ordering and are superseded by this update.
GPS, receiver, telemetry radio and ESC units are external; their carrier
connectors are shown, not those external units. Model labels may not appear
in KiCad because its VRML importer does not render Text nodes.

This is flight-critical prototype hardware, not a certified flight controller.
The DRC report lists ignored checks, including missing footprint courtyards;
zero reported violations is not a complete mechanical-clearance review.
The fuse/diode protect the J11 input path; J6/ESC1 connects directly to the
5 V rail. Review backfeeding and power-source selection before connecting
USB, an external supply or any ESC BEC together.
Verify actual clone-module pin order and socket spacing before manufacture.
Continuity-test every power rail, bring it up with current limiting, validate
all sensor axes and motor mapping, and perform bench tests with propellers
removed before any restrained or free-flight test.
