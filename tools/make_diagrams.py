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


# ── 5. the first swim: the route from above, the depth by phase, the turn ─
def first_swim():
    """The control-only route, drawn as the route. The numbers are read from
    missions/out_and_back.py and the leg limit from mongla.py, so the drawing
    cannot disagree with the code. Phases the board ends by itself (the dive,
    the turn, the climb) have no measured duration yet: they get a fixed width
    marked 'unmeasured', not a guess."""
    import ast

    def consts(path):
        tree = ast.parse(pathlib.Path(path).read_text())
        out = {}
        for n in tree.body:
            if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name):
                try:
                    out[n.targets[0].id] = ast.literal_eval(n.value)
                except ValueError:
                    pass                    # a computed value; none of these is read
        return out
    C = consts('src/mongla_planner/mongla_planner/missions/out_and_back.py')
    leg = consts('src/mongla_control/mongla_control/mongla.py')['MAX_TURN_LEG_DEG']
    W, H = 1160, 530
    o = [head(W, H, 'The control-only swim: out through the gate, a clockwise half turn, back, and the kill switch')]
    o.append(f'<rect width="{W}" height="{H}" fill="#05070a"/>')
    o.append(txt(32, 46, 'The first swim, from above, in depth, and in the turn', 'h'))
    o.append(txt(32, 68, 'BOARD VERBS ONLY · NO CAMERA · NO ODOMETRY · NUMBERS READ FROM out_and_back.py'))

    # from above: east is out; the hull turns clockwise and comes home below
    yo, yb, xs, xg, xt = 180, 260, 170, 300, 600
    r = (yb - yo) / 2
    o.append(txt(32, 108, 'FROM ABOVE', 'v'))
    o.append(txt(32, 124, 'leg lengths are durations: the speed at each gain is not measured yet'))
    for y in (yo - 30, yo + 30):                                   # the gate, two posts
        o.append(f'<rect x="{xg - 4}" y="{y - 8}" width="8" height="16" fill="{INK}"/>')
    o.append(txt(xg, yo + 54, 'GATE', 'v', 'middle'))
    o.append(f'<path d="M{xs},{yo} H{xt} A{r},{r} 0 0 1 {xt},{yb} H{xs}" fill="none" stroke="{RED}" stroke-width="2"/>')
    o.append(f'<line x1="{xs}" y1="{yb}" x2="60" y2="{yb}" stroke="{RED}" stroke-width="2" stroke-dasharray="4 4"/>')
    for x, y, d in ((420, yo, 1), (420, yb, -1)):                  # direction of travel
        o.append(f'<path d="M{x + 5 * d},{yo if y == yo else yb} l{-10 * d},-5 v10 z" fill="{RED}"/>')
    o.append(f'<circle cx="{xs}" cy="{yo}" r="5" fill="#05070a" stroke="{INK}" stroke-width="1.4"/>')
    o.append(txt(xs - 12, yo + 4, 'START', 'v', 'end'))
    o.append(txt(460, yo - 12, f"move_forward {C['OUT_S']:g} s at {C['GAIN']:g} %", 'r', 'middle'))
    o.append(txt(xt + r + 10, (yo + yb) / 2 - 4, f"yaw_right {C['TURN_DEG']:g}", 'r'))
    o.append(txt(xt + r + 10, (yo + yb) / 2 + 12, f"sent as 2 x {C['TURN_DEG'] / 2:g}"))
    o.append(txt(460, yb + 22, f"move_forward {C['BACK_S']:g} s at {C['GAIN']:g} %", 'r', 'middle'))
    o.append(txt(60, yb + 22, f"{C['HOLD_S']:g} s at {C['HOLD_GAIN']:g} %", 'r'))
    o.append(txt(60, yb + 38, 'a diver turns the kill switch'))

    # in depth: width is the mission's own seconds; fixed where the board decides
    y0, y1, x0, x1, unk = 380, 440, 32, 712, 70.0
    o.append(txt(32, 330, 'IN DEPTH', 'v'))
    o.append(txt(32, 346, 'width is the seconds the mission sets; the board ends the dive, the turn and the climb itself'))
    # (name, seconds or None, depth at start, depth at end, third line)
    phases = [('countdown', C['TETHER_S'], 0, 0, 'tether on'), ('dive', None, 0, 1, ''),
              ('out', C['OUT_S'], 1, 1, ''), ('turn', None, 1, 1, ''),
              ('back', C['BACK_S'], 1, 1, ''), ('hold', C['HOLD_S'], 1, 1, 'kill switch'),
              ('climb', None, 1, 0, 'surface')]
    known = sum(p[1] for p in phases if p[1])
    scale = (x1 - x0 - unk * sum(1 for p in phases if not p[1])) / known
    o.append(f'<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}" stroke="{RULE}" stroke-width="1"/>')
    o.append(txt(x1 + 8, y0 + 4, '0 m'))
    o.append(txt(x1 + 8, y1 + 4, f"{C['DEPTH_M']:g} m"))
    x, pts = x0, []
    for name, sec, d0, d1, note in phases:
        w = sec * scale if sec else unk
        pts += [(x, y1 if d0 else y0), (x + w, y1 if d1 else y0)]
        o.append(f'<line x1="{x:.1f}" y1="{y0 - 6}" x2="{x:.1f}" y2="{y1 + 6}" stroke="{RULE}" stroke-width="1"/>')
        o.append(txt(x + w / 2, y1 + 28, name, 'v' if sec else 'b', 'middle'))
        o.append(txt(x + w / 2, y1 + 44, f'{sec:g} s' if sec else 'unmeasured', 'l', 'middle'))
        if note:
            o.append(txt(x + w / 2, y1 + 60, note, 'l', 'middle'))
        x += w
    o.append('<path d="M' + ' L'.join(f'{px:.1f},{py}' for px, py in pts)
             + f'" fill="none" stroke="{SEA_L}" stroke-width="2"/>')

    # the turn: why the half turn is sent as two quarter turns
    cx, cy, R = 980, 250, 74
    o.append(txt(800, 108, 'WHY THE HALF TURN IS SPLIT', 'v'))
    o.append(txt(800, 124, 'the board turns the short way round'))
    o.append(f'<circle cx="{cx}" cy="{cy}" r="{R}" fill="none" stroke="{RULE}" stroke-width="1"/>')
    o.append(f'<line x1="{cx}" y1="{cy}" x2="{cx}" y2="{cy - R + 6}" stroke="{INK}" stroke-width="1.6"/>')
    o.append(f'<line x1="{cx}" y1="{cy}" x2="{cx}" y2="{cy + R - 6}" stroke="{INK}" stroke-width="1.6" stroke-dasharray="3 3"/>')
    o.append(txt(cx, cy - R - 10, 'heading now', 'v', 'middle'))
    o.append(txt(cx, cy + R + 20, 'target, 180 away', 'v', 'middle'))
    o.append(f'<path d="M{cx + 10},{cy - R + 14} A{R - 16},{R - 16} 0 0 1 {cx + 10},{cy + R - 14}" fill="none" stroke="{RED_L}" stroke-width="2"/>')
    o.append(f'<path d="M{cx - 10},{cy - R + 14} A{R - 16},{R - 16} 0 0 0 {cx - 10},{cy + R - 14}" fill="none" stroke="{SEA_L}" stroke-width="2" stroke-dasharray="5 4"/>')
    o.append(txt(cx + R + 8, cy + 4, 'clockwise', 'r'))
    o.append(txt(cx - R - 8, cy + 4, 'other way', 'b', 'end'))
    for i, ln in enumerate(('At 180 neither way is shorter. With', '0.2 deg of yaw noise, a float32 copy',
                            "of the board's arithmetic went clockwise", '0.498 of the time. So any turn over',
                            f'{leg:g} deg is sent as equal legs, and a', '90 deg leg has a short way round.')):
        o.append(txt(800, 382 + 16 * i, ln, 'l'))
    o.append(txt(800, 484, 'measured-bars §68 · MAX_TURN_LEG_DEG', 'r'))
    o.append('</svg>')
    return '\n'.join(o)

for name, fn in (('loop-rate', loop_rate), ('field-of-view', fov), ('frame-budget', frame_budget),
                 ('package-map', package_map), ('first-swim', first_swim)):
    p = OUT / f'{name}.svg'
    p.write_text(fn())
    import xml.etree.ElementTree as ET
    ET.parse(p)                      # a diagram that will not parse renders as nothing
    print(f'{name:18} {p.stat().st_size // 1024:>3} KB  parses clean')
