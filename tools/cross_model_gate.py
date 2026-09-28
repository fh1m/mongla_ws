#!/usr/bin/env python3
"""Two models, one class: does AGREEMENT kill the false positives?

⛔ THE SITUATION NOBODY ACTED ON. We ship two graphs that both detect `gate`:

    gate_rescue_repair   claims a gate on 90.8 % of gate-free frames at the
                         shipped 0.15, half-frame boxes over empty turquoise
                         at 0.44-0.56 (B-59). Separation between gate-present
                         and gate-absent footage: +0.0 points.
    gate_sharks          at 0.30 -- 100 % positive, 15.8 % worst negative,
                         +84.2 points of separation.

Only one runs. The other is a false-positive machine -- but a false-positive
machine whose errors have NO REASON to coincide with the good model's, because
they were trained on different data with different classes.

⭐ BUMBLEBEE REFLEX 1, LITERALLY. "When you cannot pick a threshold, RUN BOTH."
Their 2026 gate runs FIVE pose estimators in parallel. The ensemble literature
says the same thing for detection: an intersection-based ensemble filters false
positives, because two independent errors rarely land in the same place.

THE TEST. Per frame, on clips asserted to contain the prop and clips asserted
not to, compare:

    A alone          the shipped model
    B alone          the other one
    A AND B          both fire, and their boxes overlap by >= --iou

and report the positive rate and the worst negative rate for each. The
question is whether AND buys separation that neither has alone, and at what
cost in true detections.

⚠ `detector_dual_node` already runs two models off ONE decode, so the vehicle
cost of this is one more Hailo group, not one more camera pipeline.
"""
from __future__ import annotations

import argparse
import os

import numpy as np


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def boxes_for(res, names, want, conf):
    out = []
    for r in res:
        for b in r.boxes:
            c = int(b.cls[0])
            if names.get(c, str(c)).strip().lower() != want:
                continue
            s = float(b.conf[0])
            if s < conf:
                continue
            out.append(tuple(float(v) for v in b.xyxy[0]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model-a', required=True, help='the shipped one')
    ap.add_argument('--model-b', required=True)
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--positive', nargs='*', default=[])
    ap.add_argument('--negative', nargs='*', default=[])
    ap.add_argument('--conf', type=float, default=0.30)
    ap.add_argument('--iou', type=float, default=0.30)
    ap.add_argument('--limit', type=int, default=300)
    a = ap.parse_args()

    from ultralytics import YOLO
    A, B = YOLO(a.model_a), YOLO(a.model_b)
    na = A.names if isinstance(A.names, dict) else dict(enumerate(A.names))
    nb = B.names if isinstance(B.names, dict) else dict(enumerate(B.names))
    want = a.cls.strip().lower()
    for tag, names in (('A', na), ('B', nb)):
        if want not in [str(v).strip().lower() for v in names.values()]:
            raise SystemExit(f'model {tag} has no class {want!r}: '
                             f'{sorted(names.values())}')

    import cv2
    print(f'\nA={os.path.basename(a.model_a)}  B={os.path.basename(a.model_b)}'
          f'  class={want!r}  conf>={a.conf}  iou>={a.iou}')
    print(f'{"clip":<34}{"truth":>9}{"n":>6}{"A":>8}{"B":>8}{"A AND B":>9}')
    rows = []
    for truth, clips in (('present', a.positive), ('ABSENT', a.negative)):
        for clip in clips:
            cap = cv2.VideoCapture(clip)
            if not cap.isOpened():
                print(f'  cannot open {clip}')
                continue
            n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            hits = [0, 0, 0]
            seen = 0
            for j in range(a.limit):
                cap.set(cv2.CAP_PROP_POS_FRAMES,
                        int(n * (0.05 + 0.9 * j / a.limit)))
                ok, f = cap.read()
                if not ok:
                    continue
                seen += 1
                ba = boxes_for(A.predict(f, conf=a.conf, verbose=False),
                               na, want, a.conf)
                bb = boxes_for(B.predict(f, conf=a.conf, verbose=False),
                               nb, want, a.conf)
                hits[0] += bool(ba)
                hits[1] += bool(bb)
                hits[2] += any(_iou(x, y) >= a.iou for x in ba for y in bb)
            cap.release()
            if not seen:
                continue
            r = [100.0 * h / seen for h in hits]
            rows.append((truth, r))
            print(f'{os.path.basename(clip)[:33]:<34}{truth:>9}{seen:>6}'
                  f'{r[0]:>7.1f}%{r[1]:>7.1f}%{r[2]:>8.1f}%')

    pos = [r for t, r in rows if t == 'present']
    neg = [r for t, r in rows if t == 'ABSENT']
    if pos and neg:
        print(f'\n{"":<34}{"worst +":>9}{"worst -":>9}{"separation":>12}')
        for i, tag in enumerate(('A alone', 'B alone', 'A AND B')):
            wp = min(r[i] for r in pos)
            wn = max(r[i] for r in neg)
            print(f'{tag:<34}{wp:>8.1f}%{wn:>8.1f}%{wp - wn:>+11.1f} pts')
        print('\n⚠ AND can only LOSE true detections -- it is an intersection. '
              'The question is whether the false positives it removes are '
              'worth the true ones it costs, and both columns are printed so '
              'that is a decision rather than a slogan.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
