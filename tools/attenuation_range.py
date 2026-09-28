#!/usr/bin/env python3
"""Is the WATER ITSELF a rangefinder? Turbidity as signal, not noise.

⛔ THE FIRST-PRINCIPLES ARGUMENT. Every method this repo has tried for holding
a target needs local image structure -- LK corners, XFeat descriptors, NCC
patches, ECC gradients. Section 58 measured all four failing together in the
same regime, because the failure is information-theoretic: a flat patch carries
0.92 bits and there is nothing to track.

But a flat patch is not empty. Underwater image formation is

    I(x) = J(x) t(x) + A (1 - t(x)),        t(x) = exp(-beta * range(x))

where A is the veiling light and beta the attenuation coefficient. The
transmission t is a function of RANGE ALONE. So range is encoded in the haze,
per pixel, with **no texture required** -- and the signal is STRONGEST exactly
where texture is weakest, because both come from turbidity.

⭐ That makes it the first candidate whose failure mode is genuinely
ANTI-correlated with the ladder's rungs, rather than merely different. Bumblebee
reflex 1 wants redundancy; redundancy is worth most when the new source fails
where the old ones succeed and vice versa.

⚠ AND IT IS NOT THE THING P4 REJECTED. That was `depth_anything_v2_small.onnx`
-- 99 MB, a learned model, measured at 1 211 ms/frame on the Pi. This is a dark
channel and a percentile: no model, no training data, no accelerator.

THE TEST. A detected box gives apparent size, which is proportional to 1/range
for a fixed object. If the transmission inside that box tracks it, the prior
carries range. Spearman rather than Pearson -- the relation only has to be
monotonic to be useful, and beta is unknown.

    python3 tools/attenuation_range.py --model M.onnx --video A.mkv

⛔ WHAT WOULD FALSIFY IT: a correlation near zero, or a sign that flips between
clips. Both are reported rather than summarised away.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onnxruntime as ort                                        # noqa: E402
from conf_vs_range import detect, spearman                       # noqa: E402

# Patch size for the dark channel's min-filter. Big enough that a bright
# speck cannot dominate, small enough to stay local to the box.
DCP_PATCH = 15


def dark_channel(bgr, patch=DCP_PATCH):
    """Per-pixel min over colour channels, then a spatial min-filter.

    ⚠ UNDERWATER THE RED CHANNEL IS USUALLY THE DARK ONE -- it is absorbed
    within a couple of metres, which is precisely why the prior carries range
    here. Keeping all three channels rather than assuming red makes it work in
    green and in blue water without a switch.
    """
    m = bgr.min(axis=2)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (patch, patch))
    return cv2.erode(m, k)


def veiling_light(bgr, dc, frac=0.001):
    """A, from the brightest 0.1 % of the dark channel -- the standard
    estimator. Those pixels are the most haze-dominated, i.e. furthest."""
    n = max(1, int(dc.size * frac))
    idx = np.argpartition(dc.ravel(), -n)[-n:]
    return bgr.reshape(-1, 3)[idx].mean(axis=0)


def transmission(bgr, A, omega=0.95):
    """t = 1 - omega * min_c( I_c / A_c ), the He et al. form.

    `omega` keeps a little haze so a far surface does not transmit exactly 0,
    which would make log-range infinite.
    """
    norm = bgr.astype(np.float32) / np.maximum(A, 1e-3)
    return 1.0 - omega * cv2.erode(
        norm.min(axis=2),
        cv2.getStructuringElement(cv2.MORPH_RECT, (DCP_PATCH, DCP_PATCH)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--video', action='append', required=True)
    ap.add_argument('--frames', type=int, default=150)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--decimate', type=int, default=4)
    a = ap.parse_args()

    sess = ort.InferenceSession(a.model, providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name

    import time
    for clip in a.video:
        cap = cv2.VideoCapture(clip)
        if not cap.isOpened():
            print(f'cannot open {clip}')
            continue
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        per, costs = {}, []
        for j in range(a.frames):
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * (0.05 + 0.9 * j / a.frames)))
            ok, f = cap.read()
            if not ok:
                continue
            H, W = f.shape[:2]
            # ⚠ DECIMATE. Transmission is a smooth field -- it has no
            # high-frequency content to lose -- and the min-filter is the cost.
            # Full 1080p measured 40 ms/frame, which is a whole frame budget.
            small = cv2.resize(f, (W // a.decimate, H // a.decimate),
                               interpolation=cv2.INTER_AREA)
            t0 = time.perf_counter()
            dc = dark_channel(small)
            A = veiling_light(small, dc)
            tmap = transmission(small, A)
            costs.append((time.perf_counter() - t0) * 1e3)
            sh, sw = tmap.shape
            for d in detect(sess, iname, f, a.conf):
                x1, y1, x2, y2 = d.xyxy(W, H)
                x1, x2 = x1 * sw // W, max(x1 * sw // W + 1, x2 * sw // W)
                y1, y2 = y1 * sh // H, max(y1 * sh // H + 1, y2 * sh // H)
                box_t = tmap[y1:y2, x1:x2]
                if box_t.size < 4:
                    continue
                # ⭐ THE BOX AGAINST THE FRAME, not the box alone. Absolute
                # transmission moves with the water and the exposure; what
                # range changes is how much LESS hazy the target is than the
                # scene behind it. That contrast is the range signal.
                per.setdefault(int(d.cls), []).append(
                    (d.h / H, float(np.median(box_t)),
                     float(np.median(box_t) - np.median(tmap))))
        cap.release()
        print(f'\n{os.path.basename(clip)}  transmission cost p50 '
              f'{np.median(costs):.1f} ms/frame at /{a.decimate}'
              if costs else clip)
        for c in sorted(per):
            v = np.array(per[c])
            if len(v) < 12:
                continue
            s_abs = spearman(v[:, 0], v[:, 1])
            s_rel = spearman(v[:, 0], v[:, 2])
            print(f'  cls {c}  n={len(v):4d}   spearman(size, t_box) '
                  f'{s_abs:+.2f}   spearman(size, t_box - t_frame) '
                  f'{s_rel:+.2f}')
    print('\n⚠ A correlation near zero, or a SIGN THAT FLIPS between clips, '
          'falsifies the prior as a rangefinder here.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
