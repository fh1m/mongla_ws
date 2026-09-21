#!/usr/bin/env python3
"""What the water leaves of each colour, measured on our own footage.

Everyone repeats that water eats red first. This measures it, on the real
2025 archive, and writes the chart into docs/index.html between the
`colour:begin` / `colour:end` markers.

Method, stated on the page because it is the only thing that makes the
numbers mean anything:

  * the frames are the ORIGINAL archive captures, not the published stills --
    those carry a red-and-blue HUD overlay that would contaminate the channels;
  * every Nth frame of each venue folder is sampled (`--stride`), not a
    hand-picked one, because picking the prettiest frame is how you measure
    your own taste;
  * each frame's mean B, G, R is taken over all pixels, then averaged over the
    folder, and the three are normalised to the brightest channel so venues of
    different exposure can be compared at all.

The archive is 20 GB and lives outside the repository. Point MONGLA_ARCHIVE at
it; there is deliberately NO default, because a hardcoded path is how a figure
silently becomes un-reproducible.

    MONGLA_ARCHIVE=/path/to/2025 python3 tools/colour_loss.py
    python3 tools/colour_loss.py --check      # uses the cached measurement
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _site_splice import splice  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'docs' / 'imgs' / 'real' / 'manifest.json'
CACHE = ROOT / 'docs' / 'imgs' / 'real' / 'colour.json'
EXTS = {'.png', '.jpg', '.jpeg', '.webp'}


def venues_from_manifest() -> dict[str, list[Path]]:
    """Venue -> the archive folders its published stills came from."""
    scenes = json.loads(MANIFEST.read_text())['scenes']
    out: dict[str, list[Path]] = {}
    for s in scenes:
        folder = Path(s['source_frame']).parent
        out.setdefault(s['venue'], [])
        if folder not in out[s['venue']]:
            out[s['venue']].append(folder)
    return out


def measure(archive: Path, stride: int) -> list[dict]:
    import cv2
    import numpy as np
    rows = []
    for venue, folders in venues_from_manifest().items():
        sums = np.zeros(3, dtype=np.float64)
        n = 0
        for folder in folders:
            d = archive / folder
            if not d.is_dir():
                print(f'missing: {d}', file=sys.stderr)
                continue
            files = sorted(f for f in d.iterdir() if f.suffix.lower() in EXTS)
            for f in files[::stride]:
                img = cv2.imread(str(f))
                if img is None:
                    continue
                sums += img.reshape(-1, 3).mean(axis=0)   # OpenCV order: B, G, R
                n += 1
        if not n:
            continue
        b, g, r = (sums / n)
        top = max(b, g, r)
        rows.append({'venue': venue, 'frames': n,
                     'b': round(b, 1), 'g': round(g, 1), 'r': round(r, 1),
                     'bn': round(b / top, 3), 'gn': round(g / top, 3), 'rn': round(r / top, 3),
                     'folders': [str(f) for f in folders]})
    return rows


def render(rows: list[dict], stride: int) -> str:
    bars = []
    for row in rows:
        chans = ''.join(
            f'<div class="chan chan--{k}"><span class="silk">{k}</span>'
            f'<i style="--w:{row[k + "n"] * 100:.1f}%"></i>'
            f'<b>{row[k]:.0f}</b></div>'
            for k in ('r', 'g', 'b'))
        bars.append(
            f'<li><div class="colour__head"><b>{row["venue"]}</b>'
            f'<span class="silk">{row["frames"]} frames</span></div>{chans}</li>')
    total = sum(r['frames'] for r in rows)
    worst = min(rows, key=lambda r: r['rn'])
    # A venue whose channels come out level is NOT evidence that water spares
    # red there; it is a different camera or a different white balance, and
    # saying so is the whole difference between a measurement and a slogan.
    neutral = [r for r in rows if r['rn'] > 0.9]
    note = ''
    if neutral:
        names = ' and '.join(r['venue'] for r in neutral)
        note = (f' {names} comes back level — that footage is near-neutral, which says the '
                f'camera or its white balance differed, not that the water there was kinder. '
                f'Two venues, two answers, from the same pipeline: that is why the number is '
                f'measured per venue instead of quoted once.')
    return (
        f'  <figure class="colour" aria-label="Mean colour channel level per venue, '
        f'measured over {total} real frames">\n'
        f'   <ol class="colour__rows">{"".join(bars)}</ol>\n'
        f'   <figcaption>Mean red, green and blue over <strong>{total}</strong> frames of our own '
        f'2025 footage — every {stride}th frame of each venue\'s folders, original captures with '
        f'no HUD overlay — each venue scaled to its own brightest channel. At '
        f'<strong>{worst["venue"]}</strong> red falls to <strong>{worst["rn"] * 100:.0f}%</strong> '
        f'of the strongest channel.{note} Measured by <code>tools/colour_loss.py</code>.'
        f'</figcaption>\n'
        f'  </figure>')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='exit 1 if the page has drifted')
    ap.add_argument('--stride', type=int, default=20, help='sample every Nth frame (default 20)')
    args = ap.parse_args()

    archive = os.environ.get('MONGLA_ARCHIVE')
    if archive:
        rows = measure(Path(archive), args.stride)
        if rows:
            CACHE.write_text(json.dumps({'stride': args.stride, 'rows': rows}, indent=1) + '\n')
    elif CACHE.exists():
        # --check has to work on a machine without the archive, so the
        # measurement is cached beside the stills it belongs to
        cached = json.loads(CACHE.read_text())
        rows, args.stride = cached['rows'], cached['stride']
    else:
        print('set MONGLA_ARCHIVE to the 2025 archive to measure', file=sys.stderr)
        return 2
    if not rows:
        print('no frames measured', file=sys.stderr)
        return 2
    rc = splice('colour', render(rows, args.stride), args.check)
    if rc == 0 and not args.check:
        for r in rows:
            print(f'{r["venue"]:10} {r["frames"]:5} frames   R {r["r"]:6.1f}  G {r["g"]:6.1f}  '
                  f'B {r["b"]:6.1f}   red/brightest {r["rn"]:.2f}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
