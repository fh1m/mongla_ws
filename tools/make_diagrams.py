#!/usr/bin/env python3
"""Generate the site's technical drawings.

A diagram should BE the thing it describes. A timing story is drawn as a time
axis; an optics story is drawn as a lens with rays; a budget is drawn to scale.
Boxes with arrows are a slide, not a drawing.

Palette is the site's: black field, #ff0000 for the machine, #004eff for the
medium, #788294 for labels. Type is Zed Mono, which the page already loads.
"""
import math, pathlib

OUT = pathlib.Path('docs/assets/diagrams')
RED, RED_L = '#ff0000', '#ff2a14'   # RED_L == site.css --red-lit, the one lit red
SEA, SEA_L = '#004eff', '#5a8dff'
INK, DIM, RULE = '#e4e8ef', '#788294', '#1a2030'
MONO = 'Zed Mono,ui-monospace,monospace'


def head(w, h, label):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
            f'width="{w}" height="{h}" role="img" aria-label="{label}">'
            f'<style>text{{font-family:{MONO}}}'
            f'.l{{font-size:9.5px;fill:{DIM};letter-spacing:.14em}}'
            f'.v{{font-size:11px;fill:{INK}}}'
            f'.r{{font-size:10px;fill:{RED_L};letter-spacing:.06em}}'
            f'.b{{font-size:10px;fill:{SEA_L};letter-spacing:.06em}}'
            f'.h{{font-size:17px;fill:#fff;font-weight:700;letter-spacing:-.02em}}'
            f'.big{{font-size:30px;fill:#fff;font-weight:700;letter-spacing:-.04em}}'
            f'</style>')


def txt(x, y, s, cls='l', anchor='start'):
    return f'<text x="{x:.1f}" y="{y:.1f}" class="{cls}" text-anchor="{anchor}">{s}</text>'


# ── 1. the loop rate, drawn as two combs on one axis ────────────────────
def loop_rate():
    W, H = 1160, 410
    x0, x1 = 176, W - 56
    px = lambda t: x0 + (x1 - x0) * t / 100.0
    o = [head(W, H, 'The same disturbance on a 20 Hz loop and a 500 Hz loop')]
    o.append(txt(0, 22, 'How long a mistake is allowed to stand', 'h'))
    o.append(txt(0, 42, 'ONE DISTURBANCE · TWO ARCHITECTURES · EACH TICK IS A CHANCE TO ANSWER'))

    dx = px(12)
    for label, sub, period, col, lit, y, note in (
        ('BEFORE', 'Pixhawk + ArduSub, corrected by our 20 Hz outer loop',
         50.0, SEA, SEA_L, 132, 'up to 50 ms with nobody answering'),
        ('NOW', 'the loop is on the board, on a core nothing may interrupt',
         2.0, RED, RED_L, 262, 'up to 2 ms'),
    ):
        o.append(txt(0, y - 26, label, 'v'))
        o.append(txt(0, y - 10, sub))
        o.append(f'<line x1="{x0}" y1="{y}" x2="{x1}" y2="{y}" stroke="{RULE}" stroke-width="1"/>')
        t, first = 0.0, None
        while t <= 100.0001:
            x = px(t)
            hgt = 15 if abs(t % 10) < 1e-9 else 9
            o.append(f'<line x1="{x:.1f}" y1="{y - hgt}" x2="{x:.1f}" y2="{y}" '
                     f'stroke="{col}" stroke-width="1" opacity=".9"/>')
            if first is None and t > 12:
                first = t
            t += period
        fx = px(first)
        o.append(f'<rect x="{dx:.1f}" y="{y - 22}" width="{fx - dx:.1f}" height="22" '
                 f'fill="{col}" opacity=".20"/>')
        o.append(f'<line x1="{dx:.1f}" y1="{y - 32}" x2="{fx:.1f}" y2="{y - 32}" '
                 f'stroke="{lit}" stroke-width="1"/>')
        wide = (fx - dx) > 150
        o.append(txt((dx + fx) / 2 if wide else fx + 9, y - 40, note,
                     'r' if col == RED else 'b', 'middle' if wide else 'start'))

    o.append(f'<line x1="{dx:.1f}" y1="60" x2="{dx:.1f}" y2="278" stroke="{RED}" '
             f'stroke-width="1" stroke-dasharray="3 4"/>')
    o.append(txt(dx + 7, 56, 'A DISTURBANCE ARRIVES', 'r'))

    ay = 310
    o.append(f'<line x1="{x0}" y1="{ay}" x2="{x1}" y2="{ay}" stroke="{RULE}" stroke-width="1"/>')
    for t in (0, 20, 40, 60, 80, 100):
        x = px(t)
        o.append(f'<line x1="{x:.1f}" y1="{ay}" x2="{x:.1f}" y2="{ay + 5}" stroke="{RULE}"/>')
        if t < 100:
            o.append(txt(x, ay + 19, str(t), 'l', 'middle'))
    o.append(txt(x1, ay + 19, '100 ms', 'l', 'end'))
    o.append(txt(0, 388, '25×', 'big'))
    o.append(txt(62, 380, 'corrections underneath every command the host sends.', 'v'))
    o.append(txt(62, 396, 'Nothing about our code got faster. The boundary moved.'))
    o.append('</svg>')
    return '\n'.join(o)


# ── 2. the field of view, drawn as an actual lens ───────────────────────
def fov():
    W, H = 1160, 470
    cx, cy = 150, H / 2
    o = [head(W, H, 'The camera sees 46.7 degrees in water, not the 63.8 the datasheet states')]
    o.append(txt(0, 22, 'The lens the datasheet describes is not the lens you have', 'h'))
    o.append(txt(0, 42, 'FLAT PORT · SNELL REFRACTION · MEASURED IN WATER, ±0.7°'))

    # the cone has to FIT: derive reach from the half-height available rather
    # than picking a length and discovering it overruns the frame
    avail = (H - 150) / 2
    for ang, col, lit, lab, dash in (
        (63.8, SEA, SEA_L, '63.8°  in air — the datasheet', '4 5'),
        (46.7, RED, RED_L, '46.7°  in water — measured', None),
    ):
        reach = min(avail / math.tan(math.radians(ang / 2)), W - cx - 330)
        half = math.radians(ang / 2)
        for sgn in (-1, 1):
            ey = cy + reach * math.tan(half) * sgn
            o.append(f'<line x1="{cx}" y1="{cy}" x2="{cx + reach}" y2="{ey:.1f}" '
                     f'stroke="{col}" stroke-width="1.4"'
                     + (f' stroke-dasharray="{dash}"' if dash else '') + '/>')
        ty = cy + reach * math.tan(half)
        o.append(f'<path d="M{cx + reach},{cy - reach * math.tan(half):.1f} '
                 f'A{reach},{reach} 0 0 1 {cx + reach},{ty:.1f}" fill="none" '
                 f'stroke="{col}" stroke-width="1" opacity=".5"'
                 + (f' stroke-dasharray="{dash}"' if dash else '') + '/>')
        # put each label ON its own cone edge, angled with it -- two labels at
        # the same y collide and the first attempt did exactly that
        lx = cx + reach * 0.62
        ly = cy - reach * 0.62 * math.tan(half) - 9
        deg = -math.degrees(half)
        o.append(f'<text x="{lx:.1f}" y="{ly:.1f}" class="'
                 + ('b' if col == SEA else 'r')
                 + f'" transform="rotate({deg:.1f} {lx:.1f} {ly:.1f})">{lab}</text>')

    o.append(f'<circle cx="{cx}" cy="{cy}" r="9" fill="none" stroke="{INK}" stroke-width="1.4"/>')
    o.append(f'<line x1="{cx - 26}" y1="{cy - 34}" x2="{cx - 26}" y2="{cy + 34}" '
             f'stroke="{INK}" stroke-width="2"/>')
    o.append(txt(cx - 34, cy - 44, 'FLAT PORT', 'l', 'end'))
    o.append(txt(cx + 16, cy + 30, 'SENSOR'))
    o.append(f'<line x1="{cx}" y1="{cy}" x2="{W - 40}" y2="{cy}" stroke="{RULE}" '
             f'stroke-width="1" stroke-dasharray="2 6"/>')

    o.append(f'<line x1="0" y1="{H - 74}" x2="{W}" y2="{H - 74}" stroke="{RULE}"/>')
    o.append(txt(0, H - 40, '27%', 'big'))
    o.append(txt(86, H - 48, 'less world than the datasheet promises.', 'v'))
    o.append(txt(86, H - 32, 'Every search pattern tuned against the air figure sweeps a view that does not exist.'))
    o.append('</svg>')
    return '\n'.join(o)


# ── 3. one frame's journey, drawn to scale on a time axis ───────────────
def frame_budget():
    W, H = 1160, 400
    x0, x1 = 120, W - 60
    ms = lambda t: x0 + (x1 - x0) * t / 40.0
    o = [head(W, H, 'One frame from photons to a thruster correction, drawn to scale')]
    o.append(txt(0, 22, "One frame, photon to thrust", 'h'))
    o.append(txt(0, 42, 'DRAWN TO SCALE · EVERY SPAN MEASURED · 40 ms OF WALL CLOCK'))

    lanes = [
        ('CAMERA',     0.0,  3.0,  RED,  '#5c0500', 'exposure + transfer'),
        ('HAILO-8',    3.0, 15.4,  SEA,  '#001a5c', 'inference · 98.0 Hz on the chip'),
        ('ROS GRAPH', 15.4, 18.0,  RULE, '#11141c', 'publish · mailbox keeps the newest frame'),
    ]
    y = 86
    for name, a, b, stroke, fill, note in lanes:
        o.append(txt(0, y + 13, name))
        o.append(f'<rect x="{ms(a):.1f}" y="{y}" width="{ms(b) - ms(a):.1f}" height="20" '
                 f'fill="{fill}" stroke="{stroke}" stroke-width="1"/>')
        o.append(txt(ms(b) + 9, y + 14, note, 'r' if stroke == RED else ('b' if stroke == SEA else 'l')))
        y += 42

    o.append(f'<line x1="{ms(18):.1f}" y1="76" x2="{ms(18):.1f}" y2="266" stroke="{RED}" '
             f'stroke-width="1" stroke-dasharray="4 4"/>')
    o.append(txt(ms(18) + 8, 232, '18.0 ms — the detection exists', 'r'))

    o.append(f'<line x1="{ms(0):.1f}" y1="248" x2="{ms(18.55):.1f}" y2="248" stroke="{RED_L}"/>')
    o.append(f'<path d="M{ms(0):.1f},244 v8 M{ms(18.55):.1f},244 v8" stroke="{RED_L}"/>')
    o.append(txt(ms(18.55) + 8, 252, 'reaches the vision verb · 53.9 Hz through the graph; it commands at 50 Hz', 'r'))

    ay = 282
    o.append(f'<line x1="{x0}" y1="{ay}" x2="{x1}" y2="{ay}" stroke="{RULE}"/>')
    for t in range(0, 41, 5):
        x = ms(t)
        o.append(f'<line x1="{x:.1f}" y1="{ay}" x2="{x:.1f}" y2="{ay + 5}" stroke="{RULE}"/>')
        o.append(txt(x, ay + 19, f'{t}', 'l', 'middle'))
    o.append(txt(x1 + 8, ay + 19, 'ms'))

    cy = ay + 46
    t = 0.0
    while t <= 40.0001:
        x = ms(t)
        o.append(f'<line x1="{x:.1f}" y1="{cy}" x2="{x:.1f}" y2="{cy + 13}" '
                 f'stroke="{RED}" stroke-width="1" opacity=".75"/>')
        t += 2.0
    o.append(txt(x0, cy + 30, 'THE BOARD, UNDERNEATH ALL OF IT · 500 Hz · ONE TICK EVERY 2 ms', 'r'))
    o.append('</svg>')
    return '\n'.join(o)


# ── 4. the six packages, drawn as the one road between them ─────────────
def package_map():
    """Not a box per package: the ROAD. Everything that commands the vehicle is
    a client of one action, and exactly one node touches the board. Perception
    (blue, the medium) feeds that node; the command path (red, the machine)
    leaves it down one cable. Counts are deliberately absent -- they live in
    .claude/context/packages/README.md, the one copy."""
    W, H = 1160, 600
    o = [head(W, H, 'Six packages: every command is one goal on one action, and one node talks to the board')]
    o.append(f'<rect width="{W}" height="{H}" fill="#05070a"/>')   # shown on GitHub too, either theme
    o.append(txt(32, 46, 'Six packages, and the one road between them', 'h'))
    o.append(txt(32, 68, 'EVERY COMMAND IS ONE GOAL ON ONE ACTION · EXACTLY ONE NODE TOUCHES THE BOARD'))

    def box(x, y, w, h, title, lines, col, code=None):
        o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#0a0d13" stroke="{col}" stroke-width="1"/>')
        o.append(txt(x + 16, y + 24, title, 'v'))
        for i, l in enumerate(lines):
            o.append(txt(x + 16, y + 46 + 17 * i, l))
        if code:
            o.append(txt(x + 16, y + h - 14, code, 'r' if col in (RED, RED_L) else 'b' if col == SEA else 'v'))

    def arrow(x1, y1, x2, y2, col, dash=False):
        d = ' stroke-dasharray="5 4"' if dash else ''
        o.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{col}" stroke-width="1.6"{d}/>')
        ang = math.atan2(y2 - y1, x2 - x1)
        a1, a2 = ang + 2.7, ang - 2.7
        o.append(f'<path d="M{x2},{y2} L{x2 + 9 * math.cos(a1):.1f},{y2 + 9 * math.sin(a1):.1f} '
                 f'L{x2 + 9 * math.cos(a2):.1f},{y2 + 9 * math.sin(a2):.1f} z" fill="{col}"/>')

    # the clients
    box(32, 100, 300, 130, 'mongla_planner', ['the mission DSL, the `mongla` CLI,', 'missions and state machines'],
        DIM, "mongla arm · mission sauvc_full")
    box(32, 262, 300, 130, 'mongla_vision', ['cameras → Hailo-8 → tracker →', 'the lock ladder · optical flow'],
        SEA, 'detections · lock · velocity')
    box(32, 424, 300, 130, 'mongla_localization', ['right-invariant EKF on the board IMU;', 'late measurements replayed'],
        SEA, '/mongla/odom')

    # the road: one action, one state topic
    box(400, 100, 250, 130, 'mongla_interfaces', ['one action:  /mongla/move', 'one topic:   /mongla/state'],
        RED, '23 verbs, one goal at a time')
    arrow(332, 165, 396, 165, RED_L)
    o.append(f'<path d="M332,310 H690 V200 H712" fill="none" stroke="{SEA_L}" stroke-width="1.6"/>'
             f'<path d="M716,200 l-9,-4 v8 z" fill="{SEA_L}"/>')
    o.append(txt(352, 302, 'detections · lock · flow -> the vision loops', 'b'))
    arrow(182, 392, 182, 420, SEA_L)
    o.append(txt(192, 410, 'flow velocity', 'b'))

    # the one node, the library, the cable, the board
    box(720, 100, 408, 130, 'mongla_manager', ['auv_manager_node: the ONLY node',
        'on the board. Serves the action,', 'runs the vision loops, publishes state'], RED,
        'start · connect · bringup_check')
    arrow(650, 165, 716, 165, RED_L)
    box(720, 262, 408, 130, 'mongla_control', ['the verbs and the board driver.',
        'A library, no nodes: one class,', 'one board, one link'], RED, 'SrotFC · srot_protocol')
    arrow(924, 230, 924, 258, RED_L)

    # the cable, drawn as a cable, and the board
    o.append(f'<path d="M924,392 C924,430 924,440 924,470" fill="none" stroke="{RED}" stroke-width="3"/>')
    o.append(txt(908, 424, 'ONE USB-C CABLE', 'r', 'end'))
    o.append(txt(908, 440, 'MAVLINK 2 · COMPID 191', 'r', 'end'))
    o.append(f'<rect x="720" y="470" width="408" height="84" fill="#120607" stroke="{RED}" stroke-width="1.4"/>')
    o.append(txt(736, 496, 'SROT board · firmware Hengla', 'v'))
    o.append(txt(736, 518, 'holds attitude, depth and heading at 500 Hz;'))
    o.append(txt(736, 535, 'runs and brakes every move by itself'))
    arrow(980, 466, 980, 398, DIM, dash=True)
    o.append(txt(992, 424, 'IMU · DEPTH'))
    o.append(txt(992, 440, 'ESC RPM'))
    o.append('</svg>')
    return '\n'.join(o)


for name, fn in (('loop-rate', loop_rate), ('field-of-view', fov), ('frame-budget', frame_budget),
                 ('package-map', package_map)):
    p = OUT / f'{name}.svg'
    p.write_text(fn())
    import xml.etree.ElementTree as ET
    ET.parse(p)                      # a diagram that will not parse renders as nothing
    print(f'{name:18} {p.stat().st_size // 1024:>3} KB  parses clean')
