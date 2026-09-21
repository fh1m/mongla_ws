#!/usr/bin/env python3
"""The README banner: the real hull, the real palette, rendered by a browser.

The first banner drew a vehicle that does not exist. The second was laid out by
hand in PIL, which meant every kerning decision was a magic number and the type
never matched the site. This one is HTML -- the same faces, the same palette,
the same bracketed labels as docs/assets/site.css -- screenshotted by headless
Chrome at an exact pixel size. What the README shows is what a browser drew
from the design system.

Two variants, because GitHub has a light mode and a banner that assumes dark
puts black type on black for half its readers:

    docs/imgs/mongla-banner.png        dark   2400x760
    docs/imgs/mongla-banner-light.png  light  2400x760

The README chooses between them with <picture> + prefers-color-scheme.

    python3 tools/make_banner.py
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT_DARK = REPO / 'docs' / 'imgs' / 'mongla-banner.png'
OUT_LIGHT = REPO / 'docs' / 'imgs' / 'mongla-banner-light.png'
RENDER = REPO / 'docs' / 'imgs' / 'cad' / 'mongla-quarter-aft.webp'
FONTS = REPO / 'docs' / 'assets' / 'fonts'
W, H = 2400, 760

# the render's own ground is pure black, so the bay is too -- any other value
# draws the image's rectangle as a visible box
# every figure is measured; each one is in .claude/context/measured-bars.md
CHIPS = [('500', 'Hz', 'control loop, on the board'),
         ('18.0', 'ms', 'photon to detection'),
         ('53.9', 'Hz', 'detection through ROS'),
         ('46.7', '°', 'field of view, in water'),
         ('3\u202f311', '', 'tests passing')]


def page(dark: bool) -> str:
    ink = '#e4e8ef' if dark else '#14161c'
    ink3 = '#7b8598' if dark else '#646d7b'
    strong = '#ffffff' if dark else '#0a0d12'
    field = '#000000' if dark else '#f4f5f6'
    rule = '#1a2030' if dark else '#d5d8dd'
    fonts = FONTS.as_uri()
    chips = ''.join(
        f'<div class="chip"><b>{v}<u>{u}</u></b><span>{lab}</span></div>' for v, u, lab in CHIPS)
    return f'''<!doctype html><meta charset="utf-8">
<style>
@font-face{{font-family:"Manrope";src:url("{fonts}/manrope-latin.woff2") format("woff2");
  font-weight:200 800;font-display:block}}
@font-face{{font-family:"Zed Mono";src:url("{fonts}/zed-mono-extended.woff2") format("woff2");
  font-weight:400;font-display:block}}
@font-face{{font-family:"Zed Mono";src:url("{fonts}/zed-mono-extendedbold.woff2") format("woff2");
  font-weight:700;font-display:block}}
@font-face{{font-family:"Zed Sans";src:url("{fonts}/zed-sans-extendeditalic.woff2") format("woff2");
  font-weight:400;font-style:italic;font-display:block}}
*{{box-sizing:border-box;margin:0}}
html,body{{width:{W}px;height:{H}px;overflow:hidden}}
body{{position:relative;background:{field};color:{ink};font-family:"Zed Mono",monospace}}
/* the hull sits in its own dark bay: in light mode the render's black ground
   becomes a deliberate instrument window instead of a mistake */
.bay{{position:absolute;top:0;right:0;bottom:0;width:1180px;background:#000;overflow:hidden;
  box-shadow:inset 0 1px 0 rgba(255,255,255,.06),inset 1px 0 0 {rule}}}
.bay img{{position:absolute;right:-60px;top:50%;width:1320px;transform:translateY(-50%)}}
.bay .tick{{position:absolute;left:34px;font-size:14px;letter-spacing:.2em;color:#5d6678;
  text-transform:uppercase}}
.t1{{top:34px}} .t2{{bottom:34px}}
.left{{position:absolute;left:96px;top:0;bottom:0;width:1080px;display:flex;flex-direction:column;
  justify-content:center}}
.tag{{font-family:"Manrope";font-weight:800;font-size:18px;letter-spacing:.24em;
  text-transform:uppercase;color:{ink3}}}
.tag i{{font-style:normal;color:#ff2a14}}
h1{{font-family:"Manrope";font-weight:800;font-size:188px;line-height:.84;letter-spacing:-.058em;
  margin-top:28px;color:{strong}}}
h1 em{{font-style:normal;color:#ff0000}}
.sub{{font-family:"Zed Sans";font-style:italic;font-size:31px;line-height:1.3;margin-top:58px;color:{ink}}}
.chips{{display:flex;margin-top:54px;padding-top:24px;border-top:1px solid {rule}}}
.chip{{padding-right:30px;margin-right:30px;border-right:1px solid {rule}}}
.chip:last-child{{border-right:0}}

.chip b{{display:block;white-space:nowrap;font-weight:700;font-size:40px;letter-spacing:-.03em;color:{strong}}}
.chip u{{text-decoration:none;color:#ff2a14;font-size:.5em;margin-left:.22em}}
.chip span{{display:block;margin-top:10px;font-size:14px;letter-spacing:.02em;color:{ink3};max-width:190px}}
.foot{{position:absolute;left:96px;bottom:40px;font-size:14px;letter-spacing:.2em;
  text-transform:uppercase;color:{ink3}}}
.foot i{{font-style:normal;color:#ff2a14}}
</style>
<div class="bay"><img src="{RENDER.as_uri()}" alt="">
  <span class="tick t1">[ hull · 702.0 × 176.1 × 172.1 mm ]</span>
  <span class="tick t2">[ in water: never · every number measured on the bench ]</span></div>
<div class="left">
  <div class="tag">[ an autonomy stack <i>//</i> autonomous underwater vehicles ]</div>
  <h1>Mongla<em>.</em></h1>
  <p class="sub">Machines that have to work when nobody is watching.</p>
  <div class="chips">{chips}</div>
</div>
<div class="foot">Muhammad Fahim Faisal <i>//</i> firmware, Rakibul Islam</div>
'''


def shoot(html: str, out: Path) -> None:
    chrome = shutil.which('google-chrome-stable') or shutil.which('google-chrome')
    if not chrome:
        raise SystemExit('google-chrome not found; cannot render the banner')
    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / 'banner.html'
        src.write_text(html, encoding='utf-8')
        subprocess.run([chrome, '--headless=new', '--disable-gpu', '--hide-scrollbars',
                        '--allow-file-access-from-files', '--force-device-scale-factor=1',
                        f'--window-size={W},{H}', f'--screenshot={out}',
                        f'--user-data-dir={td}/profile', '--virtual-time-budget=5000',
                        src.as_uri()],
                       check=True, capture_output=True, timeout=120)
    print(f'{out.relative_to(REPO)}  {out.stat().st_size // 1024} KB')


def main() -> int:
    if not RENDER.exists():
        print(f'missing {RENDER}', file=sys.stderr)
        return 2
    shoot(page(dark=True), OUT_DARK)
    shoot(page(dark=False), OUT_LIGHT)
    return 0


if __name__ == '__main__':
    sys.exit(main())
