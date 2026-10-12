# STM32 F450 carrier — Revision A

## Mechanical definition

- Board: 90 × 70 × 1.6 mm, two copper layers.
- Mounting: four 3.2 mm holes on an 80 × 60 mm rectangle.
- Hole centers: 5 mm from each board edge.
- Intended airframe: DJI F450-style center plate (approximately 110 × 107.5 mm).
- Blue Pill socket: two 1×20 rows at 2.54 mm pitch and 15.24 mm row spacing,
  corrected against the published YAAJ BluePill footprint.
- Front/USB direction is marked on the front silkscreen.

The 80 × 60 mm pattern is for an adapter plate or custom standoffs; it is not
claimed as a universal F450 hole pattern. Use nylon standoffs and vibration
isolation, and confirm the board clears the F450 power-distribution plate,
battery straps, and arm fasteners.

## Electrical status

The PCB is fully routed. KiCad 10 DRC reports zero violations and zero
unconnected pads. The design includes a 500 mA resettable fuse, SS14 reverse
polarity diode, 10 kΩ/1 kΩ battery divider, local filtering, status LEDs,
module sockets, GPS, telemetry, PPM-sum receiver input, and four ESC outputs.

Only J6/ESC1 has its +5 V pin connected. The other three ESC BEC pins are
deliberately isolated to prevent paralleling multiple BEC outputs.

## Mandatory fit checks before ordering

Layout update: compass U3 is upper-left, IMU U2 is lower-left, and barometer
U4 is lower-right. Telemetry J5 moved below U2. All modeled sensor PCB
envelopes are within the carrier outline and separated from the Blue Pill.
The old 22.86 mm socket spacing and old socket net order were incorrect;
this update supersedes earlier exports.

Clone modules are not mechanically standardized. Print `board-layout.svg` at
100%, place the real modules over it, and verify:

1. Blue Pill header row spacing and USB/front orientation.
2. GY-521, GY-271, and GY-63 header order and pitch.
3. GPS connector TX/RX order and supply voltage.
4. Receiver output is PPM/CPPM sum on PA0, not six separate PWM leads.
5. Battery-sense input will not exceed the STM32 ADC range for the chosen pack.

Do the first power-up with the Blue Pill and sensor modules removed. Then test
with a current-limited 5 V bench supply and no propellers fitted.
