#!/usr/bin/env python3
"""Do the ladder's two carried rungs fail TOGETHER? Redundancy of two, or of one?

The question nobody asked. The lock ladder carries two rungs when the
detector has nothing: the LK follower (sparse corners, frame-to-frame) and the
XFeat anchor (learned descriptors, frame-to-reference). They are treated as
independent -- `arbitrate` cross-checks them precisely because their errors are
assumed uncorrelated.

But BOTH need the same physical thing: local image structure. `goodFeatures
ToTrack` wants corners; XFeat's keypoint head wants repeatable texture. In
turbid water at range, the target is a low-contrast BLOB with neither -- and if
both rungs die on the same frames, the ladder has redundancy of ONE wearing the
costume of two, and Bumblebee reflex 1 ("redundancy beats optimisation") is not
actually satisfied.

This measures it on real footage. Per frame, inside a tracked ROI:
    lk      surviving LK points as a fraction of the seed
    xf      XFeat matches (the anchor's own currency)
    hist    peak of a colour back-projection against the seed histogram
            -- a rung that needs NO corners and NO descriptors, only that the
            target's COLOUR differs from the water

and reports how often each is dead, and -- the number that matters -- how often
they are dead AT THE SAME TIME versus what independence would predict.

    python3 tools/rung_independence.py --video clip.mkv --box cx,cy,w,h
    python3 tools/rung_independence.py --video clip.mkv     # centre box
    python3 tools/rung_independence.py --video clip.mkv --xfeat ~/models/xfeat_320x240.onnx

A joint-failure rate above the independent prediction is the finding. It does
not prove a third rung helps; it proves the existing two do not cover each
other. Whether `hist` survives those frames is the separate column.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

_GFTT = dict(maxCorners=120, qualityLevel=0.01, minDistance=5, blockSize=7)
_LK = dict(winSize=(21, 21), maxLevel=3,
           criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

# Below these a rung has nothing to offer the ladder.
LK_DEAD = 0.30          # fraction of seeded points surviving
HIST_DEAD = 0.20        # peak back-projection response, normalised
# The anchor needs MIN_INLIERS (15) geometric inliers to answer. Matches are an
# upper bound on inliers, so fewer than 15 matches is certainly dead.
XF_DEAD = 15


def hist_model(bgr, box):
    x1, y1, x2, y2 = box
    roi = bgr[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Hue alone is wrong underwater. Everything is one hue -- measured cast
    # B-R of +90 to +124 across the archive -- so a hue histogram cannot
    # separate a prop from the water it floats in. Hue AND saturation, which
    # is what actually differs: props are painted, water is not.
    m = cv2.inRange(hsv, (0, 30, 30), (180, 255, 255))
    h = cv2.calcHist([hsv], [0, 1], m, [24, 24], [0, 180, 0, 256])
    return cv2.normalize(h, h, 0, 255, cv2.NORM_MINMAX)


def hist_peak(bgr, model, box):
    """Peak response of the back-projection, searched near the last box."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    bp = cv2.calcBackProject([hsv], [0, 1], model, [0, 180, 0, 256], 1)
    cv2.filter2D(bp, -1, np.ones((9, 9), np.float32) / 81.0, bp)
    x1, y1, x2, y2 = box
    pad = max(20, (x2 - x1) // 2)
    h, w = bp.shape
    sub = bp[max(0, y1 - pad):min(h, y2 + pad), max(0, x1 - pad):min(w, x2 + pad)]
    return float(sub.max()) / 255.0 if sub.size else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--box', default='', help='cx,cy,w,h in pixels')
    ap.add_argument('--frames', type=int, default=400)
    ap.add_argument('--reseed', type=int, default=60,
                    help='re-seed every N frames, standing in for a detection')
    ap.add_argument('--xfeat', default='',
                    help='an xfeat_*.onnx export (tools/xfeat_export.py); '
                         'without it the xf column reads --')
    a = ap.parse_args()

    # The anchor's own matcher, so the xf column counts what the anchor would
    # count: mutual nearest-neighbour matches between the reference and now.
    xfeat = None
    if a.xfeat:
        sys.path.insert(0, 'src/mongla_vision')
        from mongla_vision.anchor.xfeat_onnx import XFeatONNX
        xfeat = XFeatONNX(a.xfeat)
    else:
        print('(no --xfeat model given; the xf column will read --)')
    ref_desc = None

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {a.video}')
    ok, f0 = cap.read()
    if not ok:
        raise SystemExit('empty clip')
    H, W = f0.shape[:2]
    if a.box:
        cx, cy, bw, bh = (int(v) for v in a.box.split(','))
    else:
        cx, cy, bw, bh = W // 2, H // 2, W // 4, H // 4
    box = (cx - bw // 2, cy - bh // 2, cx + bw // 2, cy + bh // 2)

    prev = cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY)
    pts = cv2.goodFeaturesToTrack(prev[box[1]:box[3], box[0]:box[2]], **_GFTT)
    n0 = len(pts) if pts is not None else 0
    if pts is not None:
        pts = pts.reshape(-1, 2) + np.array([box[0], box[1]], np.float32)
    model = hist_model(f0, box)
    ref_gray = prev.copy()

    rows = []
    for i in range(a.frames):
        ok, f = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)

        lk = 0.0
        if pts is not None and len(pts) >= 4:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(
                prev, gray, pts.reshape(-1, 1, 2), None, **_LK)
            if p1 is not None and st is not None:
                keep = st.reshape(-1) == 1
                lk = float(keep.sum()) / max(n0, 1)
                pts = p1.reshape(-1, 2)[keep]

        xf = float('nan')
        if xfeat is not None:
            if ref_desc is None:
                ref_desc = xfeat.detect(ref_gray)[1]
            i0, _ = XFeatONNX.match(ref_desc, xfeat.detect(gray)[1])
            xf = float(len(i0))

        hp = hist_peak(f, model, box)
        rows.append((lk, xf, hp))

        if (i + 1) % a.reseed == 0:                # stand in for a detection
            p = cv2.goodFeaturesToTrack(gray[box[1]:box[3], box[0]:box[2]],
                                        **_GFTT)
            n0 = len(p) if p is not None else 0
            pts = (p.reshape(-1, 2) + np.array([box[0], box[1]], np.float32)
                   if p is not None else None)
            ref_gray = gray.copy()
            ref_desc = None
        prev = gray
    cap.release()

    if not rows:
        raise SystemExit('no frames')
    v = np.array(rows, dtype=float)
    lk_dead = v[:, 0] < LK_DEAD
    hist_dead = v[:, 2] < HIST_DEAD
    n = len(v)
    print(f'\n{os.path.basename(a.video)}  {n} frames  box {bw}x{bh}  '
          f'reseed every {a.reseed}')
    q = lambda c, p_: float(np.nanpercentile(v[:, c], p_))       # noqa: E731
    print(f'  LK   survive  p10 {q(0, 10):.2f}  p50 {q(0, 50):.2f}  '
          f'p90 {q(0, 90):.2f}')
    print(f'  HIST peak     p10 {q(2, 10):.2f}  p50 {q(2, 50):.2f}  '
          f'p90 {q(2, 90):.2f}')
    print(f'  LK   dead (<{LK_DEAD:.0%} survive) : {lk_dead.mean():6.1%}')
    print(f'  HIST dead (peak <{HIST_DEAD:.2f})   : {hist_dead.mean():6.1%}')
    joint = float((lk_dead & hist_dead).mean())
    indep = float(lk_dead.mean() * hist_dead.mean())
    print(f'  LK and HIST both dead         : {joint:6.1%}   '
          f'(independence predicts {indep:6.1%})')
    shared = joint > indep
    if not np.isnan(v[:, 1]).all():
        xf_dead = v[:, 1] < XF_DEAD
        print(f'  XF   matches  p10 {q(1, 10):.0f}  p50 {q(1, 50):.0f}  '
              f'p90 {q(1, 90):.0f}')
        print(f'  XF   dead (<{XF_DEAD} matches)     : {xf_dead.mean():6.1%}')
        joint_xf = float((lk_dead & xf_dead).mean())
        indep_xf = float(lk_dead.mean() * xf_dead.mean())
        print(f'  LK and XF both dead           : {joint_xf:6.1%}   '
              f'(independence predicts {indep_xf:6.1%})')
        shared = shared or joint_xf > indep_xf
    if lk_dead.mean() > 0:
        print(f'  HIST alive while LK dead      : '
              f'{float((lk_dead & ~hist_dead).sum()) / max(lk_dead.sum(), 1):6.1%} '
              f'of the frames LK could not carry')
    if shared:
        print('\nWARNING: joint failure is ABOVE the independent prediction: the '
              'rungs share a failure mode, so the ladder\'s redundancy is smaller '
              'than its rung count suggests.')
    else:
        print('\nok: no pair of rungs fails together more often than independence predicts.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
