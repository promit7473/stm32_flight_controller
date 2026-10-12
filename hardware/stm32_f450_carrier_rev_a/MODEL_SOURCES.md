# Model provenance and limits

Blue Pill: complete community CAD assembly from
[piit79/Kicad-STM32](https://github.com/piit79/Kicad-STM32),
`Packages3d/YAAJ_BluePill_PinSockets_H_SWD.zip`, revision
`a6735f09fd0b003810c872f5f54c5c9e6135c137`.
The local preview uses the unmodified WRL at original scale. Source footprint
`Footprints/YAAJ_BluePill_1.kicad_mod` defines 15.24 mm row spacing;
`Symbols/YAAJ_BluePill.lib` supplied the pin-to-header mapping.
This is community CAD, not manufacturer certification. The source repository
does not contain a license file in the inspected tree; retain attribution
and check rights before redistributing the model publicly. The WRL is therefore
excluded from this Git repository; the model reference remains in the PCB.

## Optional Blue Pill model for local viewing

The upstream archive contains the model used in the previews
[piit79/Kicad-STM32 (accessed 2026/10), Packages3d]. Review the upstream terms
before downloading or reusing it. To reproduce the populated view:

1. Download the [pinned upstream model archive](https://github.com/piit79/Kicad-STM32/raw/a6735f09fd0b003810c872f5f54c5c9e6135c137/Packages3d/YAAJ_BluePill_PinSockets_H_SWD.zip).
2. Extract `YAAJ_BluePill_PinSockets_H_SWD_cp.wrl` into this folder's
   `3dmodels/` directory. The filename must match exactly.
3. Reopen KiCad's 3D Viewer. The existing project-relative reference loads it
   without moving any footprints or tracks.

The optional file is ignored by Git to prevent accidental redistribution.

## Sensor visualization status

GY-521 / MPU6050, GY-271 / HMC5883L and GY-63 / MS5611:
locally authored approximate geometry, not full sourced CAD. Their visible
PCB envelopes are respectively 21×16, 13×15 and 17×15 mm. These approximate
models are used only for visualization and provisional placement checks.

Detailed GY-521 and GY-271 STEP listings were found at Marathon OS. The
native downloads are labelled free, but selecting them opens a login form.
No account was created or payment made. A usable complete GY-63 CAD model
has not been obtained. Therefore the request for complete sourced models
for every sensor is still outstanding.

- [GY-521 listing](https://marathon-os.com/library/gy521-mpu6050-accelerometer-and-68155822fcc7c6ee36fcf89e)
- [GY-271 listing](https://marathon-os.com/library/gy271-hmc5883-magnetometer-compass-3d-681559d8fcc7c6ee36fcf8a6)

## Source reference

[piit79/Kicad-STM32, accessed 2026/10] piit79. "Kicad-STM32."
GitHub, revision `a6735f09fd0b003810c872f5f54c5c9e6135c137`.
https://github.com/piit79/Kicad-STM32/tree/a6735f09fd0b003810c872f5f54c5c9e6135c137
