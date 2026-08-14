"""Generate the wiring diagram as SVG, from facts the firmware can confirm.

Every connection here is cross-checked against the source: TIM4 compare
registers give the ESC pins, TIM2 input capture gives the receiver pin,
Serial1 gives the GPS pins, HWire(2,...) gives the I2C pins, and the battery
divider is confirmed arithmetically by the scaling constant in the code
(4095 / (3.3 V * 11) = 112.8, which is the divisor the firmware uses).
"""
import os

W, H = 1180, 690
BG = "#ffffff"
INK = "#1a1d21"
MUTED = "#6b7280"
WIRE = "#2563eb"
PWR = "#dc2626"
GND = "#111827"
BUS = "#059669"
BOX = "#f8fafc"
EDGE = "#cbd5e1"

OUT = r"C:\Users\Meraj Hossain Promit\Desktop\download_YMFC-32_auto\docs\wiring_diagram.svg"


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def box(x, y, w, h, title, lines, accent=EDGE):
    o = ['<rect x="%d" y="%d" width="%d" height="%d" rx="6" fill="%s" '
         'stroke="%s" stroke-width="1.5"/>' % (x, y, w, h, BOX, accent)]
    o.append('<text x="%d" y="%d" font-family="DejaVu Sans, Arial" '
             'font-size="13" font-weight="700" fill="%s">%s</text>'
             % (x + 10, y + 20, INK, esc(title)))
    for i, ln in enumerate(lines):
        o.append('<text x="%d" y="%d" font-family="DejaVu Sans, Arial" '
                 'font-size="11" fill="%s">%s</text>'
                 % (x + 10, y + 38 + i * 14, MUTED, esc(ln)))
    return "".join(o)


def wire(pts, colour=WIRE, width=2, dash=None):
    d = "M " + " L ".join("%d %d" % p for p in pts)
    extra = ' stroke-dasharray="%s"' % dash if dash else ""
    return ('<path d="%s" fill="none" stroke="%s" stroke-width="%d" '
            'stroke-linejoin="round" stroke-linecap="round"%s/>'
            % (d, colour, width, extra))


def label(x, y, text, size=11, colour=INK, weight="400", anchor="start"):
    return ('<text x="%d" y="%d" font-family="DejaVu Sans, Arial" '
            'font-size="%d" font-weight="%s" fill="%s" text-anchor="%s">%s</text>'
            % (x, y, size, weight, colour, anchor, esc(text)))


s = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
     'viewBox="0 0 %d %d">' % (W, H, W, H),
     '<rect width="%d" height="%d" fill="%s"/>' % (W, H, BG)]

s.append(label(28, 34, "Quadcopter flight controller — wiring", 19, INK, "700"))
s.append(label(28, 54,
               "Connections derived from the firmware: timer channels, serial "
               "ports, I2C bus and the battery scaling constant.", 12, MUTED))

# ---- MCU -------------------------------------------------------------------
MX, MY, MW, MH = 470, 118, 240, 330
s.append('<rect x="%d" y="%d" width="%d" height="%d" rx="8" fill="#eef2ff" '
         'stroke="#4f46e5" stroke-width="2"/>' % (MX, MY, MW, MH))
s.append(label(MX + MW // 2, MY + 26, "STM32F103C8T6", 15, INK, "700", "middle"))
s.append(label(MX + MW // 2, MY + 44, "72 MHz Cortex-M3, no FPU", 11, MUTED,
               "400", "middle"))
s.append(label(MX + MW // 2, MY + 60, "3.3 V logic", 11, MUTED, "400", "middle"))

left_pins = [("PA0", "TIM2 CH1", 100), ("PB10", "I2C2 SCL", 130),
             ("PB11", "I2C2 SDA", 155), ("PA9", "USART1 TX", 185),
             ("PA10", "USART1 RX", 210), ("PB0", "GPIO", 240),
             ("PA4", "ADC", 270)]
right_pins = [("PB6", "TIM4 CH1", 100), ("PB7", "TIM4 CH2", 125),
              ("PB8", "TIM4 CH3", 150), ("PB9", "TIM4 CH4", 175),
              ("PB3", "GPIO", 215), ("PB4", "GPIO", 240),
              ("PC13", "GPIO", 265)]
for name, fn, dy in left_pins:
    s.append('<circle cx="%d" cy="%d" r="3.5" fill="%s"/>' % (MX, MY + dy, INK))
    s.append(label(MX + 10, MY + dy + 4, name, 11, INK, "600"))
    s.append(label(MX + 52, MY + dy + 4, fn, 9, MUTED))
for name, fn, dy in right_pins:
    s.append('<circle cx="%d" cy="%d" r="3.5" fill="%s"/>'
             % (MX + MW, MY + dy, INK))
    s.append(label(MX + MW - 10, MY + dy + 4, name, 11, INK, "600", "end"))
    s.append(label(MX + MW - 52, MY + dy + 4, fn, 9, MUTED, "400", "end"))

# ---- peripherals -----------------------------------------------------------
s.append(box(40, 158, 200, 62, "Receiver (6-channel PPM)",
             ["PPM on one wire", "Do not connect its +5 V BEC"]))
s.append(wire([(240, 189), (360, 189), (360, MY + 100), (MX, MY + 100)]))
s.append(label(250, 182, "PPM", 10, WIRE, "600"))

s.append(box(40, 244, 200, 90, "I2C sensor bus  400 kHz",
             ["MPU-6050  0x68  IMU", "HMC5883L  0x1E  compass",
              "MS5611    0x77  barometer  (3.3 V)"], BUS))
s.append(wire([(240, 280), (340, 280), (340, MY + 130), (MX, MY + 130)], BUS))
s.append(wire([(240, 298), (325, 298), (325, MY + 155), (MX, MY + 155)], BUS))
s.append(label(250, 274, "SCL", 10, BUS, "600"))
s.append(label(250, 312, "SDA", 10, BUS, "600"))

s.append(box(40, 358, 200, 62, "GNSS receiver",
             ["u-blox, UBX binary", "9600 then 57600 baud"]))
s.append(wire([(240, 384), (310, 384), (310, MY + 185), (MX, MY + 185)]))
s.append(wire([(240, 402), (295, 402), (295, MY + 210), (MX, MY + 210)]))
s.append(label(250, 378, "RX", 10, WIRE, "600"))
s.append(label(250, 416, "TX", 10, WIRE, "600"))

s.append(box(40, 444, 200, 48, "Telemetry radio 433 MHz",
             ["Serial in, bit-banged 1 byte/loop"]))
s.append(wire([(240, 468), (280, 468), (280, MY + 240), (MX, MY + 240)]))

s.append(box(40, 516, 260, 100, "Battery sense  10k / 1k divider",
             ["3-cell LiPo, 11.1 V nominal",
              "Divider ratio 1:11 -> 36.3 V full scale",
              "Firmware divides the count by 112.81,",
              "and 4095 / (3.3 V x 11) = 112.8. Confirmed."], PWR))
s.append(wire([(300, 548), (400, 548), (400, MY + 270), (MX, MY + 270)], PWR))

# ---- ESCs and motors -------------------------------------------------------
escs = [("ESC 1", "front right", "CCW", 100),
        ("ESC 2", "rear right", "CW", 125),
        ("ESC 3", "rear left", "CCW", 150),
        ("ESC 4", "front left", "CW", 175)]
EX = 800
s.append('<rect x="%d" y="%d" width="%d" height="%d" rx="6" fill="%s" '
         'stroke="%s" stroke-width="1.5"/>' % (EX, MY + 82, 320, 118, BOX, EDGE))
s.append(label(EX + 10, MY + 100, "Electronic speed controllers", 12, INK, "700"))
for name, pos, spin, dy in escs:
    y = MY + dy + 18
    s.append(wire([(MX + MW, MY + dy), (EX, y)]))
    s.append(label(EX + 12, y + 4, "%s  %s" % (name, pos), 10, INK))
    s.append(label(EX + 190, y + 4, spin, 10, MUTED))
    s.append(label(EX + 235, y + 4, "1000-2000 us", 9, MUTED))

s.append(box(800, MY + 215, 320, 74, "Motor rotation",
             ["Diagonal pairs spin the same way. Swap any two of the",
              "three motor wires to reverse a direction."]))

s.append(box(800, MY + 305, 320, 62, "Indicator LEDs  100 ohm series",
             ["PB3 green: flight mode.  PB4 red: error.",
              "PC13 on-board: GPS fix (inverted)."]))
for dy in (215, 240, 265):
    s.append(wire([(MX + MW, MY + dy), (770, MY + dy), (770, MY + 330),
                   (800, MY + 330)], MUTED, 1.5, "4 3"))

# ---- notes -----------------------------------------------------------------
s.append('<line x1="28" y1="636" x2="%d" y2="636" stroke="%s" '
         'stroke-width="1"/>' % (W - 28, EDGE))
s.append(label(28, 658,
               "Power: ESCs feed the battery rail. Do not connect the +5 V BEC "
               "wire of more than one ESC. The MS5611 runs from 3.3 V, not 5 V.",
               11, MUTED))
s.append(label(28, 676,
               "Loop rate 250 Hz. ESC pulses are generated by TIM4 in PWM mode; "
               "the receiver PPM stream is decoded by TIM2 input capture.",
               11, MUTED))

s.append("</svg>")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, "w", encoding="utf-8").write("\n".join(s))
print("wrote", OUT)
print("%.1f KB" % (os.path.getsize(OUT) / 1024))
