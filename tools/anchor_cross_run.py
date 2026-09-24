#!/usr/bin/env python3
"""Can a reference snapped on PRACTICE DAY lock a target on the RUN?

The operator's question, and it is the one that decides whether a bank can be
preloaded rather than learned live. Preloading is worth a great deal: on the
run there may be no confident detection to enrol from at the moment the anchor
is needed, which is exactly when the detector is failing.

THE TEST. Two clips of the same venue and the same prop, recorded as separate
runs, are the closest thing the archive has to "practice then competition":

    mirpur_torpedo  <-> mirpur_torpedo_1     same venue, same prop, two runs

References are snapped across clip A and matched against frames of clip B.
Within-clip matching is measured alongside as the control, because a
cross-run number means nothing without the same-run number beside it.

⚠ WHAT THIS CANNOT SETTLE. Two runs on one day share water, light and
turbidity. A practice session days earlier, or a different pool, is a larger
domain shift than this measures, and a positive result here is necessary
evidence rather than sufficient. Stated so the number is not over-claimed.
"""
from __future__ import annotations

import argparse
import importlib.util
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from archive_root import archive_root
ARCHIVE = archive_root()

PAIRS = [
    ('mirpur_torpedo', f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo.mkv',
     'mirpur_torpedo_1', f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo_1.mkv'),
    ('octagon_1', f'{ARCHIVE}/robosub/clips/octagon/octagon_1.mp4',
     'octagon_2', f'{ARCHIVE}/robosub/clips/octagon/octagon_2.mp4'),
    ('torpedo_up_1', f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
     'torpedo_up_2', f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_2.mkv'),
]


def load_backend(model, threads):
    p = os.path.join(HERE, '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'xfeat_onnx.py')
    spec = importlib.util.spec_from_file_location('xfeat_onnx', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m.XFeatONNX(model, top_k=1024, threads=threads)


def inliers(net, k0, d0, k1, d1):
    if len(d0) < 4 or len(d1) < 4:
        return 0
    i0, i1 = net.match(d0, d1)
    if len(i0) < 8:
        return 0
    src = k0[i0].astype(np.float32).reshape(-1, 1, 2)
    dst = k1[i1].astype(np.float32).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, 3.0)
    return 0 if H is None or mask is None else int(mask.sum())


def sample(net, path, count):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return []
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    out = []
    for j in range(count):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * (0.15 + 0.7 * j / max(1, count - 1))))
        ok, f = cap.read()
        if ok:
            out.append(net.detect(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)))
    cap.release()
    return out


def best_over_bank(net, bank, q):
    return max((inliers(net, k0, d0, q[0], q[1]) for k0, d0 in bank),
               default=0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--refs', type=int, default=8)
    ap.add_argument('--queries', type=int, default=12)
    ap.add_argument('--bar', type=int, default=15)
    a = ap.parse_args()

    net = load_backend(a.model, a.threads)
    print(f'model={os.path.basename(a.model)} refs={a.refs} '
          f'queries={a.queries} bar={a.bar} inliers')
    print(f'{"bank from":<15} {"queried on":<15} {"median":>7} {"p10":>5} '
          f'{"max":>5} {"over bar":>9}')

    for an, ap_, bn, bp in PAIRS:
        bank = sample(net, ap_, a.refs)
        if not bank:
            print(f'{an:<15} UNREADABLE')
            continue
        for qn, qp in ((an, ap_), (bn, bp)):
            qs = sample(net, qp, a.queries)
            if not qs:
                print(f'{an:<15} {qn:<15} UNREADABLE')
                continue
            vals = [best_over_bank(net, bank, q) for q in qs]
            v = np.array(vals, float)
            tag = 'same run' if qn == an else '⭐ CROSS-RUN'
            print(f'{an:<15} {qn:<15} {np.median(v):>7.0f} '
                  f'{np.percentile(v, 10):>5.0f} {v.max():>5.0f} '
                  f'{(v >= a.bar).mean():>8.0%}  {tag}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
