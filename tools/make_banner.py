#!/usr/bin/env python3
"""The README banner: a sounding chart, drawn by a browser.

Earlier banners showed the CAD hull, which put the least settled thing about the
project -- a hull still being designed -- at the very top of it. This one draws
what an autonomous underwater vehicle actually lives in: a nautical chart. The
project is named for the Port of Mongla, so the banner is a chart of water "off
Mongla": depth contours, soundings, a dive track through the course's real
tasks, and the hull at the end of it with its sonar open. The name is set on it
the way a chart sets its title -- in English and in Bangla -- small, and with
room around it. Elegance here is restraint: three type sizes that differ a lot,
and nothing competing with the chart.

The contour field is seeded, so the chart is identical on every run. The three
numbers are measured (each is in .claude/context/measured-bars.md).

It is HTML in the site's own faces, screenshotted by headless Chrome:

    docs/imgs/mongla-banner.png        dark   2560x1100  (a night chart)
    docs/imgs/mongla-banner-light.png  light  2560x1100  (a paper chart)

    python3 tools/make_banner.py              # write both
    python3 tools/make_banner.py --site       # the same chart as the site's hero and page sea
    python3 tools/make_banner.py --keep-html  # also keep the HTML to open in a browser
"""
from __future__ import annotations

import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
OUT_DARK = REPO / 'docs' / 'imgs' / 'mongla-banner.png'
OUT_LIGHT = REPO / 'docs' / 'imgs' / 'mongla-banner-light.png'
FONTS = REPO / 'docs' / 'assets' / 'fonts'
W, H = 2560, 1100

# three, not five: a banner that lists everything says nothing
CHIPS = [('500', 'Hz', 'control loop'),
         ('18.0', 'ms', 'photon to detection'),
         ('46.7', '°', 'field of view, in water')]

# the dive track: the course's tasks in running order, in chart pixels
TRACK = [(1330, 900, 'start'), (1500, 780, 'gate'), (1700, 700, 'slalom'),
         (1905, 745, 'bins'), (2085, 590, 'torpedo'), (2225, 420, 'octagon')]


def depth_field(seed: int = 7):
    """A smooth, deterministic seabed: broad swells over a floor that deepens
    seaward, and one channel -- the way a river mouth shoals toward its bank."""
    rng = np.random.default_rng(seed)
    X, Y = np.meshgrid(np.linspace(0, W, 260), np.linspace(0, H, 112))
    Z = 4.0 + 18.0 * (X / W) ** 1.2
    for _ in range(14):
        cx, cy = rng.uniform(0, W), rng.uniform(0, H)
        sx, sy = rng.uniform(160, 520), rng.uniform(120, 380)
        Z += rng.uniform(-4.5, 4.5) * np.exp(-(((X - cx) / sx) ** 2 + ((Y - cy) / sy) ** 2))
    Z += 5.0 * np.exp(-(((Y - (980 - 0.34 * X)) / 150) ** 2))
    return X, Y, Z


def chart_svg(line: str, strong: str, label: str) -> str:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    X, Y, Z = depth_field()
    cs = plt.contour(X, Y, Z, levels=np.arange(2, 32, 1.0))
    out = []
    for lvl, segs in zip(cs.levels, cs.allsegs):
        major = int(lvl) % 5 == 0
        for seg in segs:
            if len(seg) < 8:
                continue
            pts = seg[::2] if len(seg) > 40 else seg          # halve the points; invisible at this scale
            d = 'M' + ' L'.join(f'{x:.0f},{y:.0f}' for x, y in pts)
            out.append(f'<path d="{d}" stroke="{strong if major else line}" '
                       f'stroke-width="{1.4 if major else 0.7}" fill="none"/>')
            if major and len(seg) > 60:
                x, y = seg[len(seg) // 2]
                out.append(f'<text x="{x:.0f}" y="{y:.0f}" class="clab" fill="{label}">{int(lvl)}</text>')
    plt.close('all')
    # soundings, sparse: a chart prints them where there is room, not everywhere
    rng = np.random.default_rng(11)
    for gx in range(1280, W - 80, 150):
        for gy in range(110, H - 90, 130):
            x, y = gx + rng.uniform(-34, 34), gy + rng.uniform(-30, 30)
            z = Z[int(y / H * (Z.shape[0] - 1)), int(x / W * (Z.shape[1] - 1))]
            out.append(f'<text x="{x:.0f}" y="{y:.0f}" class="snd" fill="{label}">{int(z)}'
                       f'<tspan dy="3" class="snd2">{int((z % 1) * 10)}</tspan></text>')
    return ''.join(out)


def track_svg(red: str, ink: str, ink3: str) -> str:
    pts = [(x, y) for x, y, _ in TRACK]
    d = f'M{pts[0][0]},{pts[0][1]}'
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        mx = (x0 + x1) / 2
        d += f' C{mx},{y0} {mx},{y1} {x1},{y1}'
    out = [f'<path d="{d}" stroke="{red}" stroke-width="2.2" stroke-dasharray="10 9" fill="none"/>']
    for x, y, name in TRACK[1:]:
        out.append(f'<g transform="translate({x},{y})"><rect x="-6" y="-6" width="12" height="12" '
                   f'transform="rotate(45)" fill="none" stroke="{red}" stroke-width="1.8"/>'
                   f'<text x="15" y="-11" class="wp" fill="{ink3}">{name}</text></g>')
    sx, sy, _ = TRACK[0]
    out.append(f'<circle cx="{sx}" cy="{sy}" r="7" fill="none" stroke="{ink3}" stroke-width="1.6"/>')
    hx, hy = 2320, 318
    ang = math.degrees(math.atan2(hy - TRACK[-1][1], hx - TRACK[-1][0]))
    wedge = ''.join(
        f'<path d="M0,0 L{r * math.cos(math.radians(-30)):.1f},{r * math.sin(math.radians(-30)):.1f} '
        f'A{r},{r} 0 0 1 {r * math.cos(math.radians(30)):.1f},{r * math.sin(math.radians(30)):.1f} Z" '
        f'fill="{ink}" fill-opacity="{op}"/>'
        for r, op in ((130, .10), (200, .07), (270, .045)))
    out.append(f'<path d="M{TRACK[-1][0]},{TRACK[-1][1]} L{hx},{hy}" stroke="{red}" stroke-width="2.2" '
               f'stroke-dasharray="10 9"/>')
    out.append(f'<g transform="translate({hx},{hy}) rotate({ang:.1f})">{wedge}'
               f'<rect x="-26" y="-8" width="52" height="16" rx="8" fill="{ink}"/>'
               f'<rect x="-18" y="-13" width="7" height="26" rx="2" fill="{ink}"/>'
               f'<rect x="11" y="-13" width="7" height="26" rx="2" fill="{ink}"/>'
               f'<circle cx="19" cy="0" r="2.6" fill="{red}"/></g>')
    return ''.join(out)


def page(dark: bool) -> str:
    if dark:
        field, ink, ink2, ink3 = '#03060c', '#eef1f6', '#b9c1ce', '#667085'
        line, strong, label, rule = '#0e2033', '#1a3a59', '#28496b', '#162131'
        red = '#ff2a14'
    else:
        field, ink, ink2, ink3 = '#f2ede1', '#101826', '#39414f', '#72767e'
        line, strong, label, rule = '#cfdbe6', '#94b3cd', '#7390ab', '#d6ceba'
        red = '#d8210f'
    fonts = FONTS.as_uri()
    chips = ''.join(f'<div class="chip"><b>{v}<u>{u}</u></b><span>{lab}</span></div>'
                    for v, u, lab in CHIPS)
    ticks = ''.join(f'<i style="left:{x}px"></i>' for x in range(80, W - 60, 80))
    vticks = ''.join(f'<i style="top:{y}px"></i>' for y in range(80, H - 60, 80))
    return f'''<!doctype html><meta charset="utf-8">
<style>
@font-face{{font-family:"Manrope";src:url("{fonts}/manrope-latin.woff2") format("woff2");font-weight:200 800;font-display:block}}
@font-face{{font-family:"Zed Mono";src:url("{fonts}/zed-mono-extended.woff2") format("woff2");font-weight:400;font-display:block}}
@font-face{{font-family:"Zed Mono";src:url("{fonts}/zed-mono-extendedbold.woff2") format("woff2");font-weight:700;font-display:block}}
@font-face{{font-family:"Zed Sans";src:url("{fonts}/zed-sans-extendeditalic.woff2") format("woff2");font-weight:400;font-style:italic;font-display:block}}
*{{box-sizing:border-box;margin:0}}
html,body{{width:{W}px;height:{H}px;overflow:hidden}}
body{{position:relative;background:{field};color:{ink};font-family:"Zed Mono",monospace}}
svg.chart{{position:absolute;inset:0}}
.clab{{font:700 13px "Zed Mono";letter-spacing:.04em}}
.snd{{font:400 15px "Zed Mono"}} .snd2{{font-size:10.5px}}
.wp{{font:700 17px "Zed Mono";letter-spacing:.2em;text-transform:uppercase}}
.frame{{position:absolute;inset:40px;border:1.5px solid {rule}}}
.frame::after{{content:"";position:absolute;inset:9px;border:1px solid {rule}}}
.tk{{position:absolute;left:40px;right:40px;top:40px;height:0}}
.tk i{{position:absolute;top:-7px;width:1px;height:8px;background:{ink3};opacity:.7}}
.tk.b{{top:auto;bottom:40px}} .tk.b i{{top:-1px}}
.vk{{position:absolute;top:40px;bottom:40px;left:40px;width:0}}
.vk i{{position:absolute;left:-7px;height:1px;width:8px;background:{ink3};opacity:.7}}
.vk.r{{left:auto;right:40px}} .vk.r i{{left:-1px}}
/* the land is left blank on a chart; the title sits on that blank */
.plate{{position:absolute;left:50px;top:50px;bottom:50px;width:1200px;
  background:linear-gradient(90deg,{field} 70%,transparent)}}
.left{{position:absolute;left:170px;top:0;bottom:0;width:980px;display:flex;flex-direction:column;
  justify-content:center}}
.tag{{font:700 21px "Zed Mono";letter-spacing:.3em;text-transform:uppercase;color:{ink3}}}
.tag i{{font-style:normal;color:{red}}}
.name{{display:flex;align-items:baseline;gap:34px;margin-top:40px}}
h1{{font:700 176px/.86 "Manrope";letter-spacing:-.045em;color:{ink}}}
h1 em{{font-style:normal;color:{red}}}
.bn{{font:500 66px/1 "Noto Sans Bengali","Noto Serif Bengali";color:{ink3}}}
.origin{{margin-top:32px;font:400 20px/1.75 "Zed Mono";letter-spacing:.12em;text-transform:uppercase;color:{ink3}}}
.origin b{{font-weight:400;color:{ink2}}}
.sub{{font:italic 400 34px/1.35 "Zed Sans";margin-top:44px;color:{ink2};max-width:900px}}
.chips{{display:flex;gap:72px;margin-top:58px}}
.chip b{{display:block;white-space:nowrap;font:700 40px "Zed Mono";letter-spacing:-.02em;color:{ink}}}
.chip u{{text-decoration:none;color:{red};font-size:.55em;margin-left:.25em}}
.chip span{{display:block;margin-top:9px;font:400 17px "Zed Mono";letter-spacing:.12em;
  text-transform:uppercase;color:{ink3}}}
.title-block{{position:absolute;right:92px;bottom:88px;padding:18px 22px;border:1px solid {rule};
  background:{field};font:400 16px/1.75 "Zed Mono";color:{ink3};letter-spacing:.16em;text-transform:uppercase}}
.title-block b{{display:block;font:700 17px "Zed Mono";letter-spacing:.24em;color:{ink2}}}
.scale{{display:flex;margin-top:10px}} .scale i{{width:44px;height:6px;border:1px solid {ink3}}}
.scale i:nth-child(odd){{background:{ink3}}}
.rose{{position:absolute;left:1290px;top:96px;width:118px;height:118px}}
</style>
<svg class="chart" viewBox="0 0 {W} {H}">{chart_svg(line, strong, label)}{track_svg(red, ink, ink3)}</svg>
<div class="plate"></div>
<div class="frame"></div>
<div class="tk">{ticks}</div><div class="tk b">{ticks}</div>
<div class="vk">{vticks}</div><div class="vk r">{vticks}</div>
<svg class="rose" viewBox="-75 -75 150 150">
  <circle r="60" fill="none" stroke="{ink3}" stroke-width="1.2"/>
  <circle r="46" fill="none" stroke="{rule}" stroke-width="1"/>
  <path d="M0,-64 L7,0 L0,8 L-7,0 Z" fill="{red}"/><path d="M0,64 L7,0 L0,-8 L-7,0 Z" fill="{ink3}"/>
  <text y="-70" text-anchor="middle" class="wp" fill="{ink2}">N</text>
</svg>
<div class="left">
  <div class="tag">an autonomy stack <i>//</i> for underwater vehicles</div>
  <div class="name"><h1>Mongla<em>.</em></h1><span class="bn">মোংলা</span></div>
  <p class="origin">named for <b>the port of Mongla</b> · Pasur River, Bangladesh<br>
    the gateway to the Sundarbans</p>
  <p class="sub">Machines that have to work when nobody is watching.</p>
  <div class="chips">{chips}</div>
</div>
<div class="title-block"><b>Chart 01 · off Mongla</b>soundings in metres<br>
  dive track · the course, in order<div class="scale"><i></i><i></i><i></i><i></i></div></div>
'''


SITE_HERO = REPO / 'docs' / 'assets' / 'chart-hero.svg'
SITE_PAGE = REPO / 'docs' / 'assets' / 'chart-page.svg'


def site_svg(full: bool) -> str:
    """The same chart, for the site. `full` is the hero: contours, soundings,
    the dive track, the rose and the graduated neat line. The page version is
    contours only, for the faint sea behind every section. Text in an SVG used
    as an image cannot reach the page's webfonts, so the soundings name a
    monospace fallback and stay small enough that the swap does not show."""
    line, strong, label, rule = '#0e2033', '#1a3a59', '#28496b', '#162131'
    ink, ink3, red = '#eef1f6', '#667085', '#ff2a14'
    body = chart_svg(line, strong, label)
    if not full:
        body = ''.join(p + '/>' for p in body.split('/>') if p.startswith('<path'))
        return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
                f'preserveAspectRatio="xMidYMid slice">{body}</svg>')
    ticks = ''.join(f'<line x1="{x}" y1="40" x2="{x}" y2="48" stroke="{ink3}" stroke-opacity=".6"/>'
                    f'<line x1="{x}" y1="{H - 48}" x2="{x}" y2="{H - 40}" stroke="{ink3}" stroke-opacity=".6"/>'
                    for x in range(80, W - 60, 80))
    ticks += ''.join(f'<line x1="40" y1="{y}" x2="48" y2="{y}" stroke="{ink3}" stroke-opacity=".6"/>'
                     f'<line x1="{W - 48}" y1="{y}" x2="{W - 40}" y2="{y}" stroke="{ink3}" stroke-opacity=".6"/>'
                     for y in range(80, H - 60, 80))
    rose = (f'<g transform="translate(1349,155)"><circle r="60" fill="none" stroke="{ink3}" stroke-width="1.2"/>'
            f'<circle r="46" fill="none" stroke="{rule}"/>'
            f'<path d="M0,-64 L7,0 L0,8 L-7,0 Z" fill="{red}"/><path d="M0,64 L7,0 L0,-8 L-7,0 Z" fill="{ink3}"/></g>')
    style = ('<style>.clab{font:700 13px ui-monospace,monospace}.snd{font:400 15px ui-monospace,monospace}'
             '.snd2{font-size:10.5px}.wp{font:700 17px ui-monospace,monospace;letter-spacing:.2em;'
             'text-transform:uppercase}</style>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
            f'preserveAspectRatio="xMidYMid slice">{style}{body}{track_svg(red, ink, ink3)}{rose}'
            f'<rect x="40" y="40" width="{W - 80}" height="{H - 80}" fill="none" stroke="{rule}" stroke-width="1.5"/>'
            f'<rect x="49" y="49" width="{W - 98}" height="{H - 98}" fill="none" stroke="{rule}"/>{ticks}</svg>')


def shoot(html: str, out: Path, size: tuple[int, int] | None = None) -> None:
    chrome = shutil.which('google-chrome-stable') or shutil.which('google-chrome')
    if not chrome:
        raise SystemExit('google-chrome not found; cannot render the banner')
    w, h = size or (W, H)
    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / 'banner.html'
        src.write_text(html, encoding='utf-8')
        subprocess.run([chrome, '--headless=new', '--disable-gpu', '--hide-scrollbars',
                        '--allow-file-access-from-files', '--force-device-scale-factor=1',
                        f'--window-size={w},{h}', f'--screenshot={out}',
                        f'--user-data-dir={td}/profile', '--virtual-time-budget=6000',
                        src.as_uri()],
                       check=True, capture_output=True, timeout=180)
    print(f'{out.relative_to(REPO)}  {out.stat().st_size // 1024} KB')


def main() -> int:
    if '--site' in sys.argv:
        for full, out in ((True, SITE_HERO), (False, SITE_PAGE)):
            out.write_text(site_svg(full), encoding='utf-8')
            print(f'{out.relative_to(REPO)}  {out.stat().st_size // 1024} KB')
        return 0
    keep = '--keep-html' in sys.argv
    for dark, out in ((True, OUT_DARK), (False, OUT_LIGHT)):
        html = page(dark)
        if keep:
            out.with_suffix('.html').write_text(html, encoding='utf-8')
        shoot(html, out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
