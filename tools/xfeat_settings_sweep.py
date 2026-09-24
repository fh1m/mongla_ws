#!/usr/bin/env python3
"""Find XFeat's best operating point FOR THIS VEHICLE, on our own footage.

The shipped settings were inherited, not chosen: `top_k=1024` because the
anchor could afford it, `detection_threshold=0.05` because it is upstream's
default and nobody in the prototype lineage ever varied it, 320x240 because it
was the size that fit the rate budget.

This sweeps the three against the thing that actually matters here -- inliers
surviving MAGSAC on the murky archive clips at a realistic time offset -- and
prices each setting in milliseconds on this box.

⚠ MORE INLIERS IS NOT AUTOMATICALLY BETTER. A setting that doubles inliers and
triples latency is a loss for a rung that must answer inside a 333 ms period,
and one that raises inliers on the CLEAR clip while lowering them on the murky
ones is tuned to the wrong water. Both columns are reported per clip so the
trade is visible rather than summarised away.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from archive_root import archive_root
A = archive_root()
CLIPS = {
    'mirpur_torpedo':   f'{A}/Mirpur/Sun_June_21/torpedo.mkv',
    'mirpur_torpedo_1': f'{A}/Mirpur/Sun_June_21/torpedo_1.mkv',
    'mirpur_gate':      f'{A}/Mirpur/Sun_June_21/gate.mkv',
    'torpedo_clear':    f'{A}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
}


def load(model, top_k, thresh, threads):
    p = os.path.join(HERE, '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'xfeat_onnx.py')
    spec = importlib.util.spec_from_file_location('xfeat_onnx', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m.XFeatONNX(model, top_k=top_k, det_thresh=thresh, threads=threads)


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


def pairs(path, offsets, starts=3):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return []
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    N = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    out = []
    for s in range(starts):
        base = int(N * (0.2 + 0.2 * s))
        cap.set(cv2.CAP_PROP_POS_FRAMES, base)
        ok, f0 = cap.read()
        if not ok:
            continue
        g0 = cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY)
        for o in offsets:
            i = base + int(round(fps * o))
            if i >= N:
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, f1 = cap.read()
            if ok:
                out.append((g0, cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY)))
    cap.release()
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', required=True,
                    help='CSV of onnx paths, e.g. 320 and 640 exports')
    ap.add_argument('--top-k', default='512,1024,2048')
    ap.add_argument('--thresh', default='0.02,0.05,0.10')
    ap.add_argument('--offsets', default='1,3,5')
    ap.add_argument('--threads', type=int, default=2)
    a = ap.parse_args()

    offsets = [float(v) for v in a.offsets.split(',')]
    data = {n: pairs(p, offsets) for n, p in CLIPS.items()}
    murky = [n for n in data if n != 'torpedo_clear']

    print(f'{"model":<22} {"top_k":>6} {"thr":>5} {"murky p50":>10} '
          f'{"murky min":>10} {"clear p50":>10} {"detect ms":>10} {"match ms":>9}')
    best = None
    for mp in a.models.split(','):
        for tk in (int(v) for v in a.top_k.split(',')):
            for th in (float(v) for v in a.thresh.split(',')):
                net = load(mp, tk, th, a.threads)
                res, td, tm = {}, [], []
                for name, ps in data.items():
                    vals = []
                    for g0, g1 in ps:
                        t0 = time.perf_counter()
                        k0, d0 = net.detect(g0)
                        k1, d1 = net.detect(g1)
                        td.append((time.perf_counter() - t0) * 500)
                        t0 = time.perf_counter()
                        vals.append(inliers(net, k0, d0, k1, d1))
                        tm.append((time.perf_counter() - t0) * 1000)
                    res[name] = np.array(vals, float)
                mk = np.concatenate([res[n] for n in murky])
                cl = res['torpedo_clear']
                row = (float(np.percentile(mk, 50)), float(mk.min()),
                       float(np.percentile(cl, 50)),
                       float(np.median(td)), float(np.median(tm)))
                print(f'{os.path.basename(mp):<22} {tk:>6} {th:>5.2f} '
                      f'{row[0]:>10.0f} {row[1]:>10.0f} {row[2]:>10.0f} '
                      f'{row[3]:>10.1f} {row[4]:>9.1f}')
                # Rank on the MURKY floor, not the median: the rung exists for
                # the worst frame, and a setting that lifts the median while
                # dropping the floor has made the failure case worse.
                score = (row[1], row[0])
                if best is None or score > best[0]:
                    best = (score, os.path.basename(mp), tk, th, row)
    if best:
        _, mp, tk, th, row = best
        print(f'\n  best by MURKY FLOOR: {mp} top_k={tk} thresh={th} '
              f'-> floor {row[1]:.0f}, p50 {row[0]:.0f}, '
              f'{row[3]:.1f} ms detect + {row[4]:.1f} ms match')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
