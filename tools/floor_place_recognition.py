#!/usr/bin/env python3
"""Can the DOWNWARD camera recognise a place it has seen before?

If it can, the bank becomes a waypoint memory: revisit a spot, recognise it,
and hand the localiser a loop closure -- the one correction a dead-reckoning
filter with no GPS and no DVL cannot generate for itself.

⚠ THE STANDING PREDICTION IS THAT THIS FAILS. The feature sweep holds that
place recognition over a tiled floor is defeated by aliasing ("nothing on a
tiled floor -- aliasing is total"), and §22 measured a generic structural view
matching a different run of itself on only 8 % of frames. This measures it
rather than inheriting it.

THE METHOD. Sample N frames along a downward clip and match every pair. Then
split the pairs by how far apart in TIME they are:

    near pairs   |dt| <= near_s      -- should match: overlapping ground
    far  pairs   |dt| >= far_s       -- should NOT match, unless the floor
                                        aliases or the vehicle revisited

The number that matters is the SEPARATION between the two distributions. A
usable place recogniser needs a threshold that keeps near pairs and rejects far
ones; if the far distribution sits on top of the near one, the floor is a
repeating pattern and no threshold exists.

⛔ A REVISIT IS INDISTINGUISHABLE FROM AN ALIAS HERE, and that is honest: both
produce a far pair that matches. So a high far-tail is a REASON TO LOOK, not a
proof of aliasing. The clip's own trajectory decides which, and we do not have
ground truth for it -- stated rather than glossed.
"""
from __future__ import annotations

import argparse
import importlib.util
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
A = '/home/fh1m/Work/Projects/Duburi/2025/raw_videos'

DOWNWARD = {
    'octagon_1 (caustics)': f'{A}/robosub/clips/octagon/octagon_1.mp4',
    'octagon_Bottom': f'{A}/final_run/octagon_Bottom.mkv',
    'bin': f'{A}/final_run/bin.mkv',
}


def backend(model, threads):
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--n', type=int, default=18)
    ap.add_argument('--near', type=float, default=2.0)
    ap.add_argument('--far', type=float, default=10.0)
    ap.add_argument('--bar', type=int, default=15)
    a = ap.parse_args()

    net = backend(a.model, a.threads)
    print(f'model={os.path.basename(a.model)} n={a.n} '
          f'near<={a.near}s far>={a.far}s bar={a.bar}')
    print(f'{"clip":<22} {"pairs":>6} {"near p50":>9} {"far p50":>8} '
          f'{"far p90":>8} {"near>=bar":>10} {"far>=bar":>9}  separation')

    for name, path in DOWNWARD.items():
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f'{name:<22} UNREADABLE')
            continue
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        N = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        # Sampled in PAIRS, not evenly: an even spread over a whole clip
        # produces no pair within `near` seconds at all, so the near
        # distribution would be empty and the comparison vacuous. Each site
        # contributes two frames a short hop apart -- that hop is the "same
        # place" case -- and different sites are the "different place" case.
        ts, feats = [], []
        hop = max(1, int(round(fps * a.near * 0.5)))
        for j in range(a.n // 2):
            base = int(N * (0.08 + 0.84 * j / max(1, a.n // 2 - 1)))
            for idx in (base, min(N - 1, base + hop)):
                cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
                ok, f = cap.read()
                if not ok:
                    continue
                ts.append(idx / fps)
                feats.append(net.detect(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)))
        cap.release()
        near, far = [], []
        for i in range(len(feats)):
            for j in range(i + 1, len(feats)):
                dt = abs(ts[i] - ts[j])
                if dt <= a.near:
                    bucket = near
                elif dt >= a.far:
                    bucket = far
                else:
                    continue
                bucket.append(inliers(net, feats[i][0], feats[i][1],
                                      feats[j][0], feats[j][1]))
        if not near or not far:
            print(f'{name:<22} too few pairs (near={len(near)} far={len(far)})')
            continue
        nn, ff = np.array(near, float), np.array(far, float)
        sep = ('SEPARATES' if np.percentile(ff, 90) < np.percentile(nn, 50)
               else 'OVERLAPS')
        print(f'{name:<22} {len(nn)+len(ff):>6} {np.percentile(nn,50):>9.0f} '
              f'{np.percentile(ff,50):>8.0f} {np.percentile(ff,90):>8.0f} '
              f'{100*(nn>=a.bar).mean():>9.0f}% {100*(ff>=a.bar).mean():>8.0f}%  {sep}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
