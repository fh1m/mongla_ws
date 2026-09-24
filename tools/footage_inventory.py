#!/usr/bin/env python3
"""Every clip, its venue, its date, and whether it is CLEAN -- before training.

⛔ WHY THIS EXISTS. A fine-tune was run on 4 304 frames of which 60 % carried
YOLO bounding boxes, class names and confidences BURNED INTO THE PIXELS. The
network was taught to match hard-edged synthetic rectangles and crisp glyphs
that do not exist at inference, and the measured cost was a uniform ~19-inlier
median drop on held-out AND trained-on venues alike -- 1.5 hours of GPU spent
learning furniture.

Nothing is extracted or trained until this inventory says a clip is clean. The
check is per-CLIP and by PIXELS, not by directory name, because a name is a
guess and this already cost a run.

It also reports DATE and VENUE, so the training/held-out split can be made on
something real rather than on whichever folder happened to be convenient.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys

import cv2
import numpy as np

VIDEO_EXT = ('.mp4', '.mkv', '.avi', '.mov', '.MP4', '.MKV', '.AVI')

# ⛔ THE FIRST DETECTOR HERE WAS WRONG AND WOULD HAVE THROWN THE ARCHIVE AWAY.
# It flagged "saturated AND bright" pixels as overlay ink -- but bright
# turquoise pool water is exactly that, so it rejected 32 of 33 clips
# including `torpedo_shark_up_3.mkv` and `bin_front_#2.mp4`, which are raw
# clips from the murky table. It was detecting water.
#
# What actually separates them is GEOMETRY, not colour: a YOLO box is an
# AXIS-ALIGNED rectangle, and pool grout lines are rarely axis-aligned in a
# moving camera. Calibrated on 18 known-annotated and 18 known-raw clips:
#
#     axis-aligned long lines   annotated: median 8, p75 16, max 31
#                                     raw: median 1, p75  2, max  7
#
#     threshold 8 -> catches 56 % of annotated, flags 0 % of raw
#
# ⚠ 56 % IS NOT A GOOD DETECTOR, and it is not relied on as one. The real
# defence is that the contaminated archive is simply not a source any more;
# this gate is belt-and-braces for anything annotated that got filed with the
# raw footage. It is tuned for ZERO false rejection, because throwing away
# real underwater footage is the worse error.
AXIS_TOL_DEG = 1.5
AXIS_MIN_LEN = 55
AXIS_LINES_MAX = 8
PROBE_FRAMES = 5


def annotated_score(path: str, n: int = PROBE_FRAMES):
    """Median count of long AXIS-ALIGNED lines. High = drawn rectangles."""
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if total <= 2:
        cap.release()
        return None, 0, (0, 0)
    vals, shape = [], (0, 0)
    for i in np.linspace(total * 0.15, total * 0.85, n).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, im = cap.read()
        if not ok or im is None:
            continue
        shape = im.shape[:2]
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ls = cv2.HoughLinesP(cv2.Canny(g, 80, 200), 1, np.pi / 180, 60,
                             minLineLength=AXIS_MIN_LEN, maxLineGap=3)
        c = 0
        if ls is not None:
            for x1, y1, x2, y2 in ls[:, 0]:
                a = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180
                if min(abs(a), abs(a - 90), abs(a - 180)) < AXIS_TOL_DEG:
                    c += 1
        vals.append(c)
    cap.release()
    if not vals:
        return None, total, shape
    return float(np.median(vals)), total, shape


def colour(path: str):
    """R/B and G/B at mid-clip -- the water's own signature."""
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if total <= 2:
        cap.release()
        return None, None
    cap.set(cv2.CAP_PROP_POS_FRAMES, total // 2)
    ok, im = cap.read()
    cap.release()
    if not ok:
        return None, None
    b, g, r = (float(im[:, :, i].mean()) for i in range(3))
    b = max(b, 1e-6)
    return r / b, g / b


def venue_of(path: str, roots: dict) -> str:
    for name, root in roots.items():
        if path.startswith(root):
            rel = os.path.relpath(path, root)
            first = rel.split(os.sep)[0]
            return f'{name}/{first}' if first != os.path.basename(path) else name
    return 'unknown'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/footage_inventory.json')
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    from archive_root import archive_root
    A = archive_root()
    # ⛔ TWO SOURCES ONLY: the raw archive and the current season. The
    # annotated-prediction archive is not a source at all -- that is what
    # cost a training run, and no detector is trusted in its place.
    # `A` is .../<year>/raw_videos, so the season root is TWO levels up, not
    # one; the first version of this line looked for 2026 inside 2025.
    roots = {'raw_videos': A,
             'season2026': os.path.join(
                 os.path.dirname(os.path.dirname(A)), '2026')}

    rows = []
    for vname, root in roots.items():
        if not os.path.isdir(root):
            print(f'{vname}: missing at {root}')
            continue
        for dirpath, _d, files in os.walk(root):
            for f in files:
                if not f.endswith(VIDEO_EXT):
                    continue
                p = os.path.join(dirpath, f)
                score, frames, shape = annotated_score(p)
                if score is None:
                    continue
                rb, gb = colour(p)
                rows.append(dict(
                    path=p, venue=venue_of(p, roots),
                    date=dt.datetime.fromtimestamp(
                        os.path.getmtime(p)).strftime('%Y-%m-%d'),
                    frames=frames, h=shape[0], w=shape[1],
                    annotated_score=score,
                    clean=bool(score < AXIS_LINES_MAX),
                    rb=None if rb is None else round(rb, 3),
                    gb=None if gb is None else round(gb, 3)))

    clean = [r for r in rows if r['clean']]
    dirty = [r for r in rows if not r['clean']]
    print(f'{len(rows)} clips: {len(clean)} CLEAN, {len(dirty)} ANNOTATED\n')

    by = {}
    for r in clean:
        key = (r['venue'], r['date'])
        by.setdefault(key, []).append(r)
    print(f'{"venue":<46}{"date":<12}{"clips":>6}{"frames":>9}{"R/B":>7}')
    for (v, d), rs in sorted(by.items()):
        rb = [x['rb'] for x in rs if x['rb']]
        print(f'{v[:45]:<46}{d:<12}{len(rs):>6}{sum(x["frames"] for x in rs):>9}'
              f'{(np.median(rb) if rb else float("nan")):>7.3f}')

    if dirty:
        print(f'\n⛔ REJECTED as annotated ({len(dirty)}):')
        for r in sorted(dirty, key=lambda x: -x['annotated_score'])[:10]:
            print(f'   {r["annotated_score"]:.3f}  {r["path"][-72:]}')
        if len(dirty) > 10:
            print(f'   ... and {len(dirty) - 10} more')

    with open(args.out, 'w') as fh:
        json.dump(rows, fh, indent=1)
    print(f'\nwritten: {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
