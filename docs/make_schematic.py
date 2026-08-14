"""Generate the flight controller schematic sheet as SVG.

Drawn the way a schematic is actually drawn, rather than as labelled boxes:

  * Net labels instead of routed wires. Nothing with fifteen nets routes long
    lines across a page; short tagged stubs remove the crossings entirely.
  * Real symbols for the parts that are real circuitry: the divider resistors,
    the LED series resistors, ground and rail flags.
  * A power distribution block, because how 11.1 V becomes 5 V becomes 3.3 V
    is the first thing anyone building this needs to know.
  * A title block.

Every net here is taken from the firmware, not from another drawing. The timer
compare registers give the ESC pins, the input capture gives the receiver pin,
Serial1 gives the GPS pins, HWire(2,...) gives the I2C pins, and the ADC
scaling constant confirms the divider ratio.
"""
import os

W, H = 1520, 1000
INK = "#111827"
LINE = "#374151"
MUTED = "#6b7280"
NET = "#1d4ed8"
PWR = "#b91c1c"
GNDC = "#111827"
FILL = "#ffffff"
SOFT = "#f9fafb"
ACC = "#4338ca"

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "schematic.svg")
F = "font-family=\"Consolas, DejaVu Sans Mono, monospace\""
FS = "font-family=\"DejaVu Sans, Arial, Helvetica, sans-serif\""


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def txt(x, y, s, size=10, fill=INK, weight="400", anchor="start", mono=True):
    return ('<text x="%g" y="%g" %s font-size="%g" font-weight="%s" '
            'fill="%s" text-anchor="%s">%s</text>'
            % (x, y, F if mono else FS, size, weight, fill, anchor, esc(s)))


def rect(x, y, w, h, fill=FILL, stroke=LINE, sw=1.4, rx=0):
    return ('<rect x="%g" y="%g" width="%g" height="%g" rx="%g" fill="%s" '
            'stroke="%s" stroke-width="%g"/>' % (x, y, w, h, rx, fill, stroke, sw))


def poly(pts, stroke=LINE, sw=1.4, fill="none"):
    d = " ".join("%g,%g" % p for p in pts)
    return ('<polyline points="%s" fill="%s" stroke="%s" stroke-width="%g" '
            'stroke-linejoin="round"/>' % (d, fill, stroke, sw))


def line(x1, y1, x2, y2, stroke=LINE, sw=1.4):
    return ('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="%s" '
            'stroke-width="%g"/>' % (x1, y1, x2, y2, stroke, sw))


def junction(x, y):
    return '<circle cx="%g" cy="%g" r="2.6" fill="%s"/>' % (x, y, LINE)


def net_label(x, y, name, direction="right", colour=NET):
    """A tagged stub. The flag points the way the signal travels."""
    w = 11 + 7.0 * len(name)
    if direction == "right":
        pts = [(x, y - 9), (x + w - 9, y - 9), (x + w, y),
               (x + w - 9, y + 9), (x, y + 9)]
        tx, anchor = x + 6, "start"
    else:
        pts = [(x, y - 9), (x - w + 9, y - 9), (x - w, y),
               (x - w + 9, y + 9), (x, y + 9)]
        tx, anchor = x - 6, "end"
    return ('<polygon points="%s" fill="#eef2ff" stroke="%s" '
            'stroke-width="1.2"/>%s'
            % (" ".join("%g,%g" % p for p in pts), colour,
               txt(tx, y + 3.5, name, 9.5, colour, "700", anchor)))


def gnd(x, y):
    return "".join([line(x, y, x, y + 8, GNDC, 1.6),
                    line(x - 9, y + 8, x + 9, y + 8, GNDC, 1.8),
                    line(x - 5.5, y + 12, x + 5.5, y + 12, GNDC, 1.6),
                    line(x - 2.5, y + 16, x + 2.5, y + 16, GNDC, 1.4)])


def rail(x, y, label, colour=PWR):
    return "".join([line(x, y, x, y - 9, colour, 1.8),
                    line(x - 11, y - 9, x + 11, y - 9, colour, 2.0),
                    txt(x, y - 15, label, 9.5, colour, "700", "middle")])


def resistor(x, y, label, value, vertical=True):
    """IEC-style zigzag."""
    o = []
    if vertical:
        o.append(line(x, y, x, y + 7))
        zig = [(x, y + 7)]
        step = 4.6
        for i in range(6):
            zig.append((x + (6 if i % 2 == 0 else -6), y + 7 + step * (i + 0.5)))
        zig.append((x, y + 7 + step * 6))
        o.append(poly(zig))
        o.append(line(x, y + 7 + step * 6, x, y + 7 + step * 6 + 7))
        o.append(txt(x + 12, y + 18, label, 9.5, INK, "700"))
        o.append(txt(x + 12, y + 30, value, 9.5, MUTED))
    return "".join(o)


def led(x, y, label, colour):
    """Triangle and bar, cathode down."""
    o = [line(x, y, x, y + 6),
         '<polygon points="%g,%g %g,%g %g,%g" fill="%s" stroke="%s" '
         'stroke-width="1.2"/>' % (x - 7, y + 6, x + 7, y + 6, x, y + 18,
                                   colour, LINE),
         line(x - 8, y + 18, x + 8, y + 18, LINE, 1.8),
         line(x, y + 18, x, y + 25),
         txt(x + 12, y + 14, label, 9.5, INK, "700")]
    # emission arrows
    o.append(poly([(x + 8, y + 2), (x + 14, y - 4)], MUTED, 1.1))
    o.append(poly([(x + 11, y - 4), (x + 14, y - 4), (x + 14, y - 1)], MUTED, 1.1))
    return "".join(o)


s = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
     'viewBox="0 0 %d %d">' % (W, H, W, H),
     '<rect width="%d" height="%d" fill="%s"/>' % (W, H, FILL)]

# sheet border
s.append(rect(14, 14, W - 28, H - 28, "none", LINE, 1.8))
s.append(rect(20, 20, W - 40, H - 40, "none", "#d1d5db", 0.8))

# ============================ MCU ==========================================
MX, MY, MW, MH = 640, 150, 300, 470
s.append(rect(MX, MY, MW, MH, SOFT, ACC, 2.0))
s.append(txt(MX + MW / 2, MY + 26, "STM32F103C8T6", 14, INK, "700", "middle", False))
s.append(txt(MX + MW / 2, MY + 43, "U1   72 MHz Cortex-M3", 9.5, MUTED, "400", "middle"))
s.append(txt(MX + MW / 2, MY + 57, "20 KB RAM   no FPU   3.3 V", 9.5, MUTED, "400", "middle"))
s.append(line(MX, MY + 68, MX + MW, MY + 68, ACC, 1.0))

left = [("PA0", "TIM2_CH1", "PPM", 100),
        ("PB10", "I2C2_SCL", "SCL", 140),
        ("PB11", "I2C2_SDA", "SDA", 165),
        ("PA9", "USART1_TX", "GPS_RX", 205),
        ("PA10", "USART1_RX", "GPS_TX", 230),
        ("PB0", "GPIO", "TLM_TX", 270),
        ("PA4", "ADC_IN4", "VBAT_SNS", 310)]
right = [("PB6", "TIM4_CH1", "ESC1", 100),
         ("PB7", "TIM4_CH2", "ESC2", 125),
         ("PB8", "TIM4_CH3", "ESC3", 150),
         ("PB9", "TIM4_CH4", "ESC4", 175),
         ("PB3", "GPIO", "LED_GRN", 220),
         ("PB4", "GPIO", "LED_RED", 245),
         ("PC13", "GPIO", "LED_BRD", 270)]

for pin, fn, net, dy in left:
    y = MY + dy
    s.append(line(MX - 34, y, MX, y))
    s.append(junction(MX, y))
    s.append(txt(MX + 9, y + 3.5, pin, 10, INK, "700"))
    s.append(txt(MX + 62, y + 3.5, fn, 8.5, MUTED))
    s.append(net_label(MX - 40, y, net, "left"))
for pin, fn, net, dy in right:
    y = MY + dy
    s.append(line(MX + MW, y, MX + MW + 34, y))
    s.append(junction(MX + MW, y))
    s.append(txt(MX + MW - 9, y + 3.5, pin, 10, INK, "700", "end"))
    s.append(txt(MX + MW - 62, y + 3.5, fn, 8.5, MUTED, "400", "end"))
    s.append(net_label(MX + MW + 40, y, net, "right"))

# 3V3 / GND on the MCU
s.append(line(MX + 60, MY, MX + 60, MY - 26))
s.append(rail(MX + 60, MY - 26, "+3V3"))
s.append(line(MX + MW - 60, MY + MH, MX + MW - 60, MY + MH + 22))
s.append(gnd(MX + MW - 60, MY + MH + 22))

# ============================ POWER ========================================
PX, PY, PW, PH = 60, 90, 430, 330
s.append(rect(PX, PY, PW, PH, "none", "#e5e7eb", 1.2, 4))
s.append(txt(PX + 10, PY + 18, "POWER DISTRIBUTION", 10, MUTED, "700"))

s.append(rect(PX + 20, PY + 36, 120, 54))
s.append(txt(PX + 80, PY + 56, "BT1", 10, INK, "700", "middle"))
s.append(txt(PX + 80, PY + 70, "3S LiPo 11.1 V", 9, MUTED, "400", "middle"))
s.append(txt(PX + 80, PY + 82, "20C, 2200-3800 mAh", 8.5, MUTED, "400", "middle"))
s.append(line(PX + 140, PY + 52, PX + 200, PY + 52, PWR, 2.0))
s.append(rail(PX + 200, PY + 52, "VBAT"))
s.append(line(PX + 80, PY + 90, PX + 80, PY + 108))
s.append(gnd(PX + 80, PY + 108))

s.append(rect(PX + 230, PY + 36, 150, 54))
s.append(txt(PX + 305, PY + 56, "ESC1-4 BEC", 10, INK, "700", "middle"))
s.append(txt(PX + 305, PY + 70, "5 V out", 9, MUTED, "400", "middle"))
s.append(txt(PX + 305, PY + 82, "connect ONE only", 8.5, PWR, "400", "middle"))
s.append(line(PX + 305, PY + 36, PX + 305, PY + 20))
s.append(rail(PX + 305, PY + 20, "+5V"))

s.append(txt(PX + 20, PY + 142, "+5V -> on-board regulator -> +3V3", 9.5, INK))
s.append(txt(PX + 20, PY + 157, "MS5611 runs from +3V3, not +5V", 9.5, PWR))

# battery divider, drawn as the circuit it is
DX, DY = PX + 45, PY + 205
s.append(line(DX, DY - 12, DX, DY))
s.append(rail(DX, DY - 12, "VBAT"))
s.append(resistor(DX, DY, "R3", "10k"))
mid = DY + 7 + 4.6 * 6 + 7
s.append(junction(DX, mid))
s.append(line(DX, mid, DX + 92, mid, NET, 1.4))
s.append(net_label(DX + 92, mid, "VBAT_SNS", "right"))
s.append(resistor(DX, mid, "R4", "1k"))
bot = mid + 7 + 4.6 * 6 + 7
s.append(line(DX, bot, DX, bot + 6))
s.append(gnd(DX, bot + 6))
s.append(txt(DX + 210, mid - 16, "divider 1:11", 9.5, INK, "700"))
s.append(txt(DX + 210, mid - 2, "36.3 V full scale", 9, MUTED))
s.append(txt(DX + 210, mid + 12, "4095/(3.3x11) = 112.8", 9, MUTED))

# ============================ LEFT PERIPHERALS =============================
def blk(x, y, w, ref, name, rows, note=None):
    h = 34 + 15 * len(rows) + (14 if note else 0)
    o = [rect(x, y, w, h)]
    o.append(txt(x + 9, y + 17, ref, 10, INK, "700"))
    o.append(txt(x + 42, y + 17, name, 9.5, INK))
    o.append(line(x, y + 24, x + w, y + 24, "#e5e7eb", 1.0))
    for i, (pin, net) in enumerate(rows):
        yy = y + 40 + 15 * i
        o.append(txt(x + 9, yy, pin, 9, MUTED))
        o.append(net_label(x + w, yy - 3.5, net, "right"))
        o.append(line(x + w - 1, yy - 3.5, x + w, yy - 3.5))
    if note:
        o.append(txt(x + 9, y + h - 6, note, 8.5, PWR))
    return "".join(o), h


y = 470
for ref, name, rows, note in [
    ("J1", "RC receiver 6ch", [("PPM out", "PPM")], "do not connect its +5V BEC"),
    ("U2", "MPU-6050  0x68", [("SCL", "SCL"), ("SDA", "SDA")], None),
    ("U3", "HMC5883L 0x1E", [("SCL", "SCL"), ("SDA", "SDA")], None),
    ("U4", "MS5611   0x77", [("SCL", "SCL"), ("SDA", "SDA")], "VCC = +3V3"),
]:
    part, hh = blk(60, y, 200, ref, name, rows, note)
    s.append(part)
    y += hh + 12

y2 = 470
for ref, name, rows, note in [
    ("U5", "u-blox GNSS", [("RX", "GPS_RX"), ("TX", "GPS_TX")], "57600 baud, 5 Hz"),
    ("U6", "433 MHz telemetry", [("RXD", "TLM_TX")], None),
]:
    part, hh = blk(350, y2, 210, ref, name, rows, note)
    s.append(part)
    y2 += hh + 12

s.append(txt(60, 456, "I2C2 bus, 400 kHz, pulled to +3V3", 9, MUTED))

# ============================ RIGHT: ESCs and LEDs =========================
EX = 1055
s.append(rect(EX, 150, 400, 190))
s.append(txt(EX + 9, 170, "ESC1-4 + motors", 10, INK, "700"))
s.append(line(EX, 178, EX + 400, 178, "#e5e7eb", 1.0))
s.append(txt(EX + 9, 194, "ref", 8.5, MUTED))
s.append(txt(EX + 52, 194, "position", 8.5, MUTED))
s.append(txt(EX + 168, 194, "rotation", 8.5, MUTED))
s.append(txt(EX + 252, 194, "signal", 8.5, MUTED))
for i, (ref, pos, rot, net) in enumerate([
        ("M1", "front right", "CCW", "ESC1"), ("M2", "rear right", "CW", "ESC2"),
        ("M3", "rear left", "CCW", "ESC3"), ("M4", "front left", "CW", "ESC4")]):
    yy = 216 + i * 26
    s.append(txt(EX + 9, yy, ref, 9.5, INK, "700"))
    s.append(txt(EX + 52, yy, pos, 9.5, MUTED))
    s.append(txt(EX + 168, yy, rot, 9.5, MUTED))
    s.append(net_label(EX + 252, yy - 3.5, net, "right"))
s.append(txt(EX + 9, 328, "1000-2000 us at 250 Hz. Swap any two motor wires to reverse.",
             8.5, MUTED))

# LEDs as real symbols
LX, LY = EX + 30, 400
s.append(rect(EX, 370, 400, 190, "none", "#e5e7eb", 1.2, 4))
s.append(txt(EX + 9, 390, "INDICATORS", 10, MUTED, "700"))
for i, (net, ref, rref, colour, what) in enumerate([
        ("LED_GRN", "D1", "R1", "#16a34a", "flight mode"),
        ("LED_RED", "D2", "R2", "#dc2626", "error")]):
    x = LX + i * 150
    s.append(net_label(x, LY, net, "right"))
    s.append(line(x + 11 + 7.0 * len(net), LY, x + 120, LY))
    s.append(line(x + 120, LY, x + 120, LY + 6))
    s.append(resistor(x + 120, LY + 6, rref, "100R"))
    yy = LY + 6 + 7 + 4.6 * 6 + 7
    s.append(led(x + 120, yy, ref, colour))
    s.append(gnd(x + 120, yy + 25))
    s.append(txt(x + 120, yy + 58, what, 8.5, MUTED, "400", "middle"))
s.append(txt(EX + 9, 548, "PC13 drives the on-board LED for GPS fix; logic is inverted.",
             8.5, MUTED))

# ============================ NOTES + TITLE BLOCK ==========================
s.append(txt(640, 700, "NOTES", 10, MUTED, "700"))
for i, n in enumerate([
        "1  Nets of the same name are connected. No wire is drawn between them.",
        "2  All three I2C devices share SCL and SDA with the MCU.",
        "3  Signal grounds are common with the battery negative.",
        "4  Pin assignments are taken from the firmware, not from a drawing."]):
    s.append(txt(640, 720 + i * 15, n, 9, MUTED))

TB_X, TB_Y, TB_W, TB_H = W - 446, H - 132, 420, 100
s.append(rect(TB_X, TB_Y, TB_W, TB_H, FILL, LINE, 1.6))
s.append(line(TB_X, TB_Y + 34, TB_X + TB_W, TB_Y + 34, LINE, 1.2))
s.append(line(TB_X, TB_Y + 67, TB_X + TB_W, TB_Y + 67, LINE, 1.2))
s.append(line(TB_X + 250, TB_Y + 34, TB_X + 250, TB_Y + TB_H, LINE, 1.2))
s.append(txt(TB_X + 12, TB_Y + 22, "STM32 QUADCOPTER FLIGHT CONTROLLER",
             11, INK, "700"))
s.append(txt(TB_X + 12, TB_Y + 50, "Interconnect schematic", 9.5, INK))
s.append(txt(TB_X + 12, TB_Y + 62, "Nets derived from firmware", 8.5, MUTED))
s.append(txt(TB_X + 262, TB_Y + 50, "Meraj Hossain Promit", 9.5, INK))
s.append(txt(TB_X + 262, TB_Y + 62, "drawn by", 8.5, MUTED))
s.append(txt(TB_X + 12, TB_Y + 84, "Sheet 1 of 1", 9, MUTED))
s.append(txt(TB_X + 262, TB_Y + 84, "Rev A", 9, MUTED))

s.append("</svg>")
open(OUT, "w", encoding="utf-8").write("\n".join(s))
print("wrote", OUT, "(%.1f KB)" % (os.path.getsize(OUT) / 1024))
