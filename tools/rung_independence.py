#!/usr/bin/env python3
"""Do the ladder's two carried rungs fail TOGETHER? Redundancy of two, or of one?

⛔ THE QUESTION NOBODY ASKED. The lock ladder carries two rungs when the
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

⚠ A JOINT-FAILURE RATE ABOVE THE INDEPENDENT PREDICTION IS THE FINDING. It does
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
XF_DEAD = 12            # matches
HIST_DEAD = 0.20        # peak back-projection response, normalised


def hist_model(bgr, box):
    x1, y1, x2, y2 = box
    roi = bgr[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # ⛔ HUE ALONE IS WRONG UNDERWATER. Everything is one hue -- measured cast
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
    a = ap.parse_args()

    try:
        sys.path.insert(0, 'src/mongla_vision')
        from mongla_vision.anchor.xfeat import XFeat          # noqa: F401
        have_xf = True
    except Exception as exc:                                  # noqa: BLE001
        print(f'(no XFeat here: {exc}; xf column will read --)')
        have_xf = False

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
        if have_xf:
            try:
                from mongla_vision.anchor.xfeat import match_pair
                xf = float(match_pair(ref_gray, gray))
            except Exception:                                 # noqa: BLE001
                xf = float('nan')

        hp = hist_peak(f, model, box)
        rows.append((lk, xf, hp))

        if (i + 1) % a.reseed == 0:                # stand in for a detection
            p = cv2.goodFeaturesToTrack(gray[box[1]:box[3], box[0]:box[2]],
                                        **_GFTT)
            n0 = len(p) if p is not None else 0
            pts = (p.reshape(-1, 2) + np.array([box[0], box[1]], np.float32)
                   if p is not None else None)
            ref_gray = gray.copy()
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
    print(f'  BOTH dead                     : {joint:6.1%}   '
          f'(independence predicts {indep:6.1%})')
    if lk_dead.mean() > 0:
        print(f'  HIST alive while LK dead      : '
              f'{float((lk_dead & ~hist_dead).sum()) / max(lk_dead.sum(), 1):6.1%} '
              f'of the frames LK could not carry')
    print('\n⚠ Joint failure ABOVE the independent prediction means the rungs '
          'share a failure mode, so the ladder\'s redundancy is smaller than '
          'its rung count suggests.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
