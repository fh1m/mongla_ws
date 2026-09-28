#!/usr/bin/env python3
"""Does the RULEBOOK's colour find the prop, with no model at all?

⭐ THE OBSERVATION NOBODY ACTED ON. Eight of the eleven SAUVC classes are named
by their colour, because the rulebook FIXES it worldwide:

    qualification gate   "orange markings on both port and starboard sides"
    main gate            "striped red and green markings on port and
                          starboard sides respectively"
    orange flare         "~15cm diameter ... and orange in color"
    flares               "red in color" / "yellow in color" / "blue in color"
    drums                "one drum is blue in color, while the rest are red"

These are not our pool's numbers to be measured on a deck -- they are published
constants, exactly like the World Aquatics lane geometry that `pool_lines.py`
already exploits. A detector built on them needs **no model, no training set
and no venue calibration**, and it cannot be wrong about a venue it has never
seen.

⭐⭐ AND THE MAIN GATE'S RED/GREEN RESOLVES PORT FROM STARBOARD -- a geometric
ambiguity a class-only detector structurally cannot answer. That is Bumblebee
reflex 2, separate GEOMETRY from IDENTITY, handed to us by the rulebook.

⛔ WHY THIS IS NOT THE COLOUR RUNG SECTION 58 REJECTED. That one built a
histogram of a RoboSub gate -- a structure whose colour the rules do not fix
and which is, in our water, the same hue-saturation cluster as the water behind
it. It lost to a decoy box on 69 % of frames and was rightly killed. Here the
colour is a published property of the object, and the question is different:
not "can a histogram of this patch find it again" but "is the rulebook's colour
separable from pool water at all".

⚠ UNDERWATER, RED DIES FIRST. Absorption removes red within roughly two metres,
so a red flare goes dark rather than red at range while blue and green survive.
A chromatic prior must therefore be measured PER CLASS and PER RANGE, not
asserted -- which is what this does, against labelled ground truth.

    python3 tools/chromatic_prior_check.py --dataset sim/datasets/<run>

⛔ WHAT WOULD FALSIFY IT: prop chromaticity inside the labelled box that is not
separable from the same frame's water. Reported as the gap between the two,
per class, so it is a number rather than an impression.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import sys

import cv2
import numpy as np


def lab_chroma(bgr):
    """(a*, b*) from CIE Lab -- chromaticity with LIGHTNESS REMOVED.

    ⛔ NOT HSV. Hue is undefined at low saturation and wraps, so a dim prop and
    a dim patch of water get hues that are noise and a mean that is nonsense --
    which is how the section 58 histogram rung fooled itself. Lab's a*/b* plane
    is a real Euclidean space: distance in it means colour difference, it does
    not wrap, and lightness (which the water column destroys with depth) is a
    separate axis that can simply be dropped.
    """
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
    return lab[:, :, 1].astype(np.float32) - 128.0, \
        lab[:, :, 2].astype(np.float32) - 128.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--camera', default='front')
    ap.add_argument('--limit', type=int, default=400)
    a = ap.parse_args()

    root = pathlib.Path(a.dataset)
    names = [s.strip() for s in
             (root / 'classes.txt').read_text().split('\n') if s.strip()]
    lab_dir = root / 'labels' / a.camera
    img_dir = root / 'frames' / a.camera
    files = sorted(p for p in lab_dir.glob('*.txt') if p.stat().st_size)
    if not files:
        raise SystemExit(f'no non-empty labels in {lab_dir}')

    per: dict = {}
    for p in files[:a.limit]:
        img = cv2.imread(str(img_dir / (p.stem + '.png')))
        if img is None:
            continue
        H, W = img.shape[:2]
        A, B = lab_chroma(img)
        # The water is what the frame is mostly made of: its median chroma.
        wa, wb = float(np.median(A)), float(np.median(B))
        for line in p.read_text().split('\n'):
            f = line.split()
            if len(f) < 5:
                continue
            c = int(f[0])
            cx, cy, bw, bh = (float(v) for v in f[1:5])
            x1 = max(0, int((cx - bw / 2) * W)); x2 = min(W, int((cx + bw / 2) * W))
            y1 = max(0, int((cy - bh / 2) * H)); y2 = min(H, int((cy + bh / 2) * H))
            if x2 - x1 < 3 or y2 - y1 < 3:
                continue
            pa = float(np.median(A[y1:y2, x1:x2]))
            pb = float(np.median(B[y1:y2, x1:x2]))
            per.setdefault(c, []).append(
                (pa - wa, pb - wb, float(np.hypot(pa - wa, pb - wb)),
                 (bw * W + bh * H) / 2.0))

    print(f'\n{os.path.basename(str(root))}/{a.camera}   '
          f'{len(files[:a.limit])} labelled frames')
    print('chroma is CIE Lab a*/b*, PROP MINUS THE SAME FRAME\'S WATER\n')
    print(f'{"class":<18}{"n":>6}{"d a*":>8}{"d b*":>8}{"distance":>10}'
          f'{"p10":>8}   reads as')
    for c in sorted(per):
        v = np.array(per[c])
        if len(v) < 8:
            continue
        da, db = float(np.median(v[:, 0])), float(np.median(v[:, 1]))
        d = float(np.median(v[:, 2]))
        p10 = float(np.percentile(v[:, 2], 10))
        # a* positive = red, negative = green. b* positive = yellow, negative
        # = blue. Name what the numbers actually say rather than trusting the
        # class name -- the point of the test is that they might disagree.
        reads = []
        if abs(da) > 4:
            reads.append('redder' if da > 0 else 'greener')
        if abs(db) > 4:
            reads.append('yellower' if db > 0 else 'bluer')
        nm = names[c] if c < len(names) else str(c)
        print(f'{nm:<18}{len(v):>6}{da:>+8.1f}{db:>+8.1f}{d:>10.1f}{p10:>8.1f}'
              f'   {" + ".join(reads) if reads else "-- no chromatic offset"}')
    print('\n⚠ `distance` is the median separation from the water in the same '
          'frame; `p10` is the worst decile, which is the number a detector '
          'would actually have to clear. A class whose p10 is near zero is NOT '
          'findable by colour however good its median looks.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
