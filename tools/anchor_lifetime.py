#!/usr/bin/env python3
"""How long does ONE XFeat reference stay usable?

The bank refreshes a checkpoint when the best one decays, but "decays" needed a
timescale. `measured-bars.md` §19.1 sampled four offsets (+1/3/5/8 s) and showed
every clip's minimum at +8 s; that is enough to prove decay and not enough to
set a cadence.

This sweeps the offset finely and reports, per clip:

  half-life   the elapsed time at which inliers fall to half the +0 s value
  T_bar       the elapsed time at which inliers cross MIN_INLIERS, below which
              a homography is not trusted at all -- the useful lifetime
  T_refresh   the crossing of REFRESH_INLIERS, which is when the bank SHOULD
              take a new photograph rather than when it must

Several start times per clip, because a reference snapped during a fast turn
and one snapped during a hold are different questions, and one start time would
answer whichever happened to be sampled.

⚠ This is an ARCHIVE measurement: it describes footage from a moving vehicle
with a moving target, so what decays is the combination. It is the right number
for setting a refresh cadence and the wrong one for claiming anything about
XFeat in isolation.
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

CLIPS = {
    'mirpur_torpedo':   f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo.mkv',
    'mirpur_torpedo_1': f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo_1.mkv',
    'mirpur_gate':      f'{ARCHIVE}/Mirpur/Sun_June_21/gate.mkv',
    'octagon':          f'{ARCHIVE}/robosub/clips/octagon/octagon_1.mp4',
    'torpedo_clear':    f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
}


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


def crossing(offsets, values, level):
    """First offset at which the curve falls to `level` and stays there.

    'And stays there' matters: these curves are noisy -- a single occluded
    frame dips and recovers -- and reporting the first dip would understate the
    lifetime badly. Reports NaN when it never crosses, which is an answer.
    """
    for i, v in enumerate(values):
        if v < level and all(x < level for x in values[i:]):
            return offsets[i]
    return float('nan')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--max-offset', type=float, default=30.0)
    ap.add_argument('--step', type=float, default=1.0)
    ap.add_argument('--starts', type=int, default=4)
    ap.add_argument('--min-inliers', type=int, default=15)
    ap.add_argument('--refresh-inliers', type=int, default=40)
    a = ap.parse_args()

    net = load_backend(a.model, a.threads)
    offs = np.arange(a.step, a.max_offset + 1e-9, a.step)

    print(f'model={os.path.basename(a.model)} offsets {a.step}..{a.max_offset}s '
          f'starts={a.starts} bar={a.min_inliers} refresh={a.refresh_inliers}')
    print(f'{"clip":<18} {"n0":>6} {"half-life":>10} {"T_refresh":>10} '
          f'{"T_bar":>8}   inliers at 1/5/10/20/30 s')

    for name, path in CLIPS.items():
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f'{name:<18} UNREADABLE')
            continue
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        curves, n0s = [], []
        for s in range(a.starts):
            base = int(n * (0.2 + 0.15 * s))
            cap.set(cv2.CAP_PROP_POS_FRAMES, base)
            ok, f = cap.read()
            if not ok:
                continue
            k0, d0 = net.detect(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
            row = []
            for o in offs:
                i = base + int(round(fps * o))
                if i >= n:
                    row.append(0)
                    continue
                cap.set(cv2.CAP_PROP_POS_FRAMES, i)
                ok, g = cap.read()
                if not ok:
                    row.append(0)
                    continue
                k1, d1 = net.detect(cv2.cvtColor(g, cv2.COLOR_BGR2GRAY))
                row.append(inliers(net, k0, d0, k1, d1))
            curves.append(row)
            n0s.append(len(d0))
        cap.release()
        if not curves:
            print(f'{name:<18} no usable starts')
            continue
        med = np.median(np.array(curves, float), axis=0)
        n0 = float(np.median(n0s))
        peak = float(med[0])
        hl = crossing(offs, med, peak * 0.5)
        tr = crossing(offs, med, a.refresh_inliers)
        tb = crossing(offs, med, a.min_inliers)
        picks = [int(round(np.interp(t, offs, med))) for t in (1, 5, 10, 20, 30)
                 if t <= a.max_offset]
        print(f'{name:<18} {n0:>6.0f} {hl:>9.1f}s {tr:>9.1f}s {tb:>7.1f}s   '
              f'{picks}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
