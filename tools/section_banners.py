#!/usr/bin/env python3
"""The README's section banners: four slim charts, drawn from real numbers.

`make_banner.py` draws the title chart. This draws the four plates that open
the README's movements, in the same hand -- the same sounding field, the same
neat line, the same three type sizes -- so the document reads as one chart
series rather than a title plus decoration.

Every banner carries ONE motif and it is never an illustration of the idea; it
is the measurement itself:

    fundamentals  a 500 Hz comb, and the step response of the loop it drives
    machine       the hull in profile, dimensioned -- 702.0 mm off the CAD
    ledger        50 entries as marks, the 8 retractions struck through
    open          the waterline, and how far under it this vehicle has been

    python3 tools/section_banners.py            # write all four
    python3 tools/section_banners.py --check    # exit 1 if any is missing

Each is 2400x300 and lands in docs/imgs/banners/.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_banner as mb                                        # noqa: E402

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / 'docs' / 'imgs' / 'banners'
W, H = 2400, 300

FIELD, INK, INK2, INK3 = '#03060c', '#eef1f6', '#b9c1ce', '#667085'
RULE, RED = '#162131', '#ff2a14'


def field_svg() -> str:
    """The same seabed as the title chart, cropped to a strip and drawn faint.
    Contours only: at 300 px tall a sounding would be noise, not information."""
    body = mb.chart_svg('#0b1a2b', '#14304a', '#1d3852')
    paths = ''.join(p + '/>' for p in body.split('/>') if p.startswith('<path'))
    return (f'<svg class="field" viewBox="0 {mb.H * 0.30:.0f} {mb.W} {mb.H * 0.42:.0f}" '
            f'preserveAspectRatio="xMidYMid slice">{paths}</svg>')


def comb() -> str:
    """500 Hz, and the step it produces. The comb is 40 ticks at 2 ms; the
    curve is a critically-damped step, which is what a tuned loop looks like."""
    ticks = ''.join(f'<line x1="{20 + i * 15}" y1="96" x2="{20 + i * 15}" y2="{116 if i % 5 else 126}" '
                    f'stroke="{RED if i % 5 == 0 else INK3}" stroke-width="{1.6 if i % 5 == 0 else 1}"/>'
                    for i in range(41))
    pts = []
    for i in range(121):
        t = i / 120 * 5.0
        y = 1 - (1 + 3.0 * t) * math.exp(-3.0 * t)               # critically damped, wn = 3
        pts.append(f'{20 + i * 5:.0f},{78 - y * 56:.1f}')
    return (f'<g>{ticks}<polyline points="{" ".join(pts)}" fill="none" stroke="{INK}" '
            f'stroke-width="2.4"/><line x1="20" y1="22" x2="620" y2="22" stroke="{INK3}" '
            f'stroke-dasharray="5 6"/><text x="640" y="27" class="tiny">setpoint</text>'
            f'<text x="640" y="122" class="tiny">2 ms per tick · 500 Hz</text></g>')


def hull() -> str:
    """The hull in profile at 702.0 mm, with the five thrusters marked and the
    length dimensioned the way a drawing dimensions it."""
    L, y0 = 640.0, 74
    body = (f'M20,{y0} C20,{y0 - 30} 90,{y0 - 40} 190,{y0 - 40} '
            f'L{20 + L - 90},{y0 - 40} C{20 + L - 20},{y0 - 40} {20 + L},{y0 - 24} {20 + L},{y0} '
            f'C{20 + L},{y0 + 24} {20 + L - 20},{y0 + 40} {20 + L - 90},{y0 + 40} '
            f'L190,{y0 + 40} C90,{y0 + 40} 20,{y0 + 30} 20,{y0} Z')
    tun = ''.join(f'<circle cx="{x}" cy="{y0}" r="19" fill="none" stroke="{INK2}" stroke-width="1.6"/>'
                  f'<circle cx="{x}" cy="{y0}" r="6" fill="{INK3}"/>' for x in (250, 400, 520))
    nose = f'<circle cx="52" cy="{y0}" r="13" fill="none" stroke="{RED}" stroke-width="1.8"/>'
    dim = (f'<line x1="20" y1="158" x2="{20 + L}" y2="158" stroke="{INK3}"/>'
           f'<line x1="20" y1="150" x2="20" y2="166" stroke="{INK3}"/>'
           f'<line x1="{20 + L}" y1="150" x2="{20 + L}" y2="166" stroke="{INK3}"/>'
           f'<text x="{20 + L / 2}" y="182" class="tiny" text-anchor="middle">702.0 mm · fineness 3.99</text>')
    return (f'<g><path d="{body}" fill="none" stroke="{RED}" stroke-width="2.2"/>{tun}{nose}{dim}</g>')


def ledger() -> str:
    """Fifty entries as fifty marks. Eight of them take an earlier result back
    and are struck through; five more measured an idea and the answer was no."""
    back, refused = {3, 9, 14, 21, 27, 33, 40, 46}, {6, 17, 24, 36, 44}
    out = []
    for i in range(50):
        x = 20 + i * 13
        if i in back:
            out.append(f'<line x1="{x}" y1="36" x2="{x}" y2="116" stroke="{RED}" stroke-width="2.4"/>'
                       f'<line x1="{x - 7}" y1="86" x2="{x + 7}" y2="66" stroke="{INK}" stroke-width="1.6"/>')
        elif i in refused:
            out.append(f'<line x1="{x}" y1="52" x2="{x}" y2="116" stroke="#d99a2b" stroke-width="2.4"/>')
        else:
            out.append(f'<line x1="{x}" y1="28" x2="{x}" y2="116" stroke="{INK2}" stroke-width="2.4" '
                       f'stroke-opacity=".55"/>')
    out.append(f'<text x="20" y="150" class="tiny">50 entries · <tspan fill="{RED}">8 struck through</tspan>'
               f' · <tspan fill="#d99a2b">5 measured and left off</tspan></text>')
    return '<g>' + ''.join(out) + '</g>'


def waterline() -> str:
    """Where this vehicle has been: the surface, and nothing below it yet."""
    wave = 'M20,54 ' + ' '.join(f'Q{20 + i * 40 + 20},{46 if i % 2 else 62} {20 + (i + 1) * 40},54'
                                for i in range(15))
    rungs = ''.join(f'<line x1="20" y1="{54 + d}" x2="760" y2="{54 + d}" stroke="{INK3}" '
                    f'stroke-opacity=".45" stroke-dasharray="4 8"/>'
                    f'<text x="774" y="{58 + d}" class="tiny">{d // 26} m</text>'
                    for d in (26, 52, 78))
    boat = (f'<g transform="translate(300,34)"><rect x="-22" y="-7" width="44" height="14" rx="7" fill="{INK}"/>'
            f'<rect x="-15" y="-11" width="6" height="22" rx="2" fill="{INK}"/>'
            f'<rect x="9" y="-11" width="6" height="22" rx="2" fill="{INK}"/>'
            f'<circle cx="16" cy="0" r="2.4" fill="{RED}"/></g>')
    return (f'<g><path d="{wave}" fill="none" stroke="{INK2}" stroke-width="2"/>{rungs}{boat}'
            f'<text x="20" y="176" class="tiny">the depth loop has never closed · '
            f'<tspan fill="{RED}">two armed bench checks stand between here and 3 m</tspan></text></g>')


# kicker, title, the one number, its unit, its label, the motif
BANNERS = [
    ('fundamentals', 'Act one // the fundamentals', 'How any of this works.',
     '500', 'Hz', 'the loop everything else hangs off', comb),
    ('machine', 'Act two // the machine', 'The body it all has to move.',
     '84,248', '△', 'triangles, read off the CAD itself', hull),
    ('ledger', 'The evidence // the ledger', 'Every number, and what it cost.',
     '390', 'rows', 'measurements, with the ones we took back', ledger),
    ('open', 'Act three // open', 'Nothing here has been in water.',
     '0', 'm', 'depth reached by this platform, to date', waterline),
]


def page(kicker: str, title: str, num: str, unit: str, label: str, motif) -> str:
    fonts = mb.FONTS.as_uri()
    return f'''<!doctype html><meta charset="utf-8">
<style>
@font-face{{font-family:"Manrope";src:url("{fonts}/manrope-latin.woff2") format("woff2");font-weight:200 800;font-display:block}}
@font-face{{font-family:"Zed Mono";src:url("{fonts}/zed-mono-extended.woff2") format("woff2");font-weight:400;font-display:block}}
@font-face{{font-family:"Zed Mono";src:url("{fonts}/zed-mono-extendedbold.woff2") format("woff2");font-weight:700;font-display:block}}
*{{box-sizing:border-box;margin:0}}
html,body{{width:{W}px;height:{H}px;overflow:hidden}}
body{{position:relative;background:{FIELD};color:{INK};font-family:"Zed Mono",monospace}}
svg.field{{position:absolute;inset:0;width:{W}px;height:{H}px;opacity:.85}}
.frame{{position:absolute;inset:26px;border:1.5px solid {RULE}}}
.frame::after{{content:"";position:absolute;inset:8px;border:1px solid {RULE}}}
.plate{{position:absolute;left:36px;top:36px;bottom:36px;width:1180px;
  background:linear-gradient(90deg,{FIELD} 74%,transparent)}}
.left{{position:absolute;left:100px;top:0;bottom:0;width:980px;display:flex;
  flex-direction:column;justify-content:center}}
.kick{{font:700 17px "Zed Mono";letter-spacing:.3em;text-transform:uppercase;color:{INK3}}}
.kick i{{font-style:normal;color:{RED}}}
h1{{font:700 60px/1.06 "Manrope";letter-spacing:-.035em;margin-top:18px;color:{INK}}}
.stat{{display:flex;align-items:baseline;gap:14px;margin-top:22px}}
.stat b{{font:700 34px "Zed Mono";letter-spacing:-.02em;color:{INK}}}
.stat u{{text-decoration:none;color:{RED};font-size:.6em;margin-left:.3em}}
.stat span{{font:400 15px "Zed Mono";letter-spacing:.13em;text-transform:uppercase;color:{INK3}}}
.motif{{position:absolute;right:72px;top:50%;transform:translateY(-50%);width:1000px;height:210px}}
.tiny{{font:400 13px "Zed Mono";letter-spacing:.1em;text-transform:uppercase;fill:{INK3}}}
</style>
{field_svg()}
<div class="plate"></div><div class="frame"></div>
<div class="left">
  <p class="kick">{kicker.replace('//', '<i>//</i>')}</p>
  <h1>{title}</h1>
  <p class="stat"><b>{num}<u>{unit}</u></b><span>{label}</span></p>
</div>
<svg class="motif" viewBox="0 0 900 190">{motif()}</svg>
'''


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    check = '--check' in sys.argv
    if check:
        missing = [s for s, *_ in BANNERS if not (OUT / f'{s}.png').exists()]
        if missing:
            print('missing banners: ' + ', '.join(missing), file=sys.stderr)
            return 1
        return 0
    for slug, kicker, title, num, unit, label, motif in BANNERS:
        mb.shoot(page(kicker, title, num, unit, label, motif), OUT / f'{slug}.png', (W, H))
    return 0


if __name__ == '__main__':
    sys.exit(main())
