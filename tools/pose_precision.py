#!/usr/bin/env python3
"""How PRECISELY does XFeat recover a pose at 320x240 versus 640x480?

⚠ RECALL AND PRECISION ARE DIFFERENT QUESTIONS, and this project has only ever
measured the first. §19 found 640 WORSE on inliers (33 against 69 on murky
footage) and the shipped backend is 320. But "how often does it match" and
"how exactly does it match when it does" are not the same number, and a
homography good enough to hold a lock can still be too coarse to stand behind
a metric pose.

⭐ TRUTH IS CONSTRUCTED, NOT BORROWED. A known homography is applied to a real
frame and the estimate is compared against it. Comparing 320 against 640
directly would measure AGREEMENT, which cannot rank them -- the mistake this
codebase has a rule against.

THE NUMBER. Mean corner reprojection error, in pixels OF THE SOURCE FRAME, so
the two resolutions are compared in one common unit rather than each in its
own. A 320 error of 1 px and a 640 error of 1 px are not the same error.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src', 'mongla_vision'))
from archive_root import archive_root                        # noqa: E402

A = archive_root()
CLIPS = {
    'mirpur_gate': f'{A}/Mirpur/gate.mkv',
    'octagon': f'{A}/robosub/clips/octagon/octagon_1.mp4',
    'bin': f'{A}/final_run/bin.mkv',
}


def known_H(w, h, deg, tx, ty, scale):
    """A homography we know exactly, about the image centre."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    cx, cy = w / 2.0, h / 2.0
    R = np.array([[c * scale, -s * scale, 0.0],
                  [s * scale,  c * scale, 0.0],
                  [0.0, 0.0, 1.0]])
    T1 = np.array([[1.0, 0, -cx], [0, 1.0, -cy], [0, 0, 1.0]])
    T2 = np.array([[1.0, 0, cx + tx], [0, 1.0, cy + ty], [0, 0, 1.0]])
    return T2 @ R @ T1


def corner_err(H_true, H_est, w, h):
    """Mean corner displacement, source-frame pixels. The single number that
    says how wrong a homography is, independent of how it is parameterised."""
    pts = np.array([[0, 0], [w, 0], [w, h], [0, h]], np.float32).reshape(-1, 1, 2)
    a = cv2.perspectiveTransform(pts, H_true).reshape(-1, 2)
    b = cv2.perspectiveTransform(pts, H_est).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(a - b, axis=1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx320', default='/home/fh1m/hailo/work/xfeat_320x240.onnx')
    ap.add_argument('--onnx640', default='/home/fh1m/hailo/work/xfeat_hailo_640x480.onnx')
    ap.add_argument('--frames', type=int, default=12)
    args = ap.parse_args()

    from mongla_vision.anchor.xfeat_onnx import XFeatONNX
    from mongla_vision.anchor.anchor import Anchor

    backends = {'320x240': XFeatONNX(args.onnx320, top_k=1024),
                '640x480': XFeatONNX(args.onnx640, top_k=1024)}

    # Warps chosen to span what a vehicle actually does between two views:
    # a small drift, a real rotation, and a scale change from closing range.
    WARPS = [('drift',  dict(deg=0.0,  tx=18.0, ty=-12.0, scale=1.00)),
             ('rotate', dict(deg=12.0, tx=0.0,  ty=0.0,   scale=1.00)),
             ('close',  dict(deg=0.0,  tx=0.0,  ty=0.0,   scale=1.18)),
             ('all',    dict(deg=8.0,  tx=14.0, ty=9.0,   scale=1.10))]

    print(f'{"clip":<14}{"warp":<9}' + ''.join(f'{k:>22}' for k in backends))
    print(' ' * 23 + ''.join(f'{"mean px / n / fail":>22}' for _ in backends))
    totals = {k: [] for k in backends}
    for name, path in CLIPS.items():
        if not os.path.exists(path):
            continue
        cap = cv2.VideoCapture(path)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        idx = np.linspace(total * 0.1, total * 0.9, args.frames).astype(int)
        imgs = []
        for i in idx:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
            ok, im = cap.read()
            if ok:
                imgs.append(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
        cap.release()
        if not imgs:
            continue
        h, w = imgs[0].shape[:2]

        for wname, kw in WARPS:
            Ht = known_H(w, h, **kw)
            row = f'{name:<14}{wname:<9}'
            for bname, be in backends.items():
                errs, fails = [], 0
                for g in imgs:
                    cur = cv2.warpPerspective(g, Ht, (w, h))
                    a = Anchor(be)
                    if a.snap(g, roi=None) <= 0:
                        fails += 1
                        continue
                    p = a.locate(cur)
                    if not p.ok or getattr(p, 'H', None) is None:
                        fails += 1
                        continue
                    # The anchor works on the BACKEND grid; lift its H back to
                    # source pixels or the two resolutions are not comparable.
                    sx, sy = be.w / float(w), be.h / float(h)
                    S = np.array([[sx, 0, 0], [0, sy, 0], [0, 0, 1.0]])
                    H_src = np.linalg.inv(S) @ np.asarray(p.H, float) @ S
                    errs.append(corner_err(Ht, H_src, w, h))
                if errs:
                    totals[bname].extend(errs)
                    row += f'{np.mean(errs):>12.2f} {len(errs):>4} {fails:>4}'
                else:
                    row += f'{"--":>12} {0:>4} {fails:>4}'
            print(row)

    print('\n=== OVERALL (source-frame px, lower is better) ===')
    for k, v in totals.items():
        if v:
            v = np.array(v)
            print(f'{k:<10} n={len(v):<5} mean={v.mean():7.2f}  '
                  f'median={np.median(v):7.2f}  p90={np.percentile(v,90):7.2f}')
    ks = [k for k in totals if totals[k]]
    if len(ks) == 2:
        a, b = (np.mean(totals[k]) for k in ks)
        better = ks[0] if a < b else ks[1]
        print(f'\n⭐ {better} is more precise '
              f'({min(a,b):.2f} px against {max(a,b):.2f} px, '
              f'{max(a,b)/max(min(a,b),1e-9):.2f}x)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
