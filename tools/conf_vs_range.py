#!/usr/bin/env python3
"""Does detector confidence actually RISE as the target gets closer?

The premise behind approach-driven acquisition: take a faint detection, let the
anchor hold it, drive closer, and the detection strengthens until it is
trustworthy. That is a good loop IF confidence is a function of apparent size.
If confidence is flat with range, closing the distance buys nothing and the
loop is a story.

Nobody here has measured it. This does, on the real archive, with the real
YOLO graphs we ship -- run through onnxruntime on the dev box so the numbers
are about the MODEL, not about the Hailo.

Per class it reports Spearman rank correlation between apparent size
(sqrt of box area, i.e. linear in 1/range for a fixed object) and confidence,
plus mean confidence by size quartile. Spearman rather than Pearson because the
relationship only has to be MONOTONIC to make the loop work -- it does not have
to be linear.

⚠ CONFOUND, STATED. A closer target is also better lit, less backscattered and
more centred, so this measures the whole approach, not size alone. That is the
right quantity for the decision at hand -- "does driving closer help?" -- and
the wrong one for a claim about scale sensitivity in isolation.
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np
import onnxruntime as ort


def letterbox(img, size=640):
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    out = np.full((size, size, 3), 114, np.uint8)
    out[:nh, :nw] = cv2.resize(img, (nw, nh))
    return out, r


def detect(sess, name, img, conf_th):
    lb, r = letterbox(img)
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    y = sess.run(None, {name: x})[0][0]           # (4+nc, 8400)
    boxes, scores = y[:4], y[4:]
    cls = scores.argmax(axis=0)
    conf = scores.max(axis=0)
    keep = conf >= conf_th
    out = []
    for i in np.nonzero(keep)[0]:
        cx, cy, bw, bh = boxes[:, i] / r
        out.append((int(cls[i]), float(conf[i]), float(bw), float(bh)))
    return out


def spearman(a, b):
    if len(a) < 8:
        return float('nan')
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean(); rb -= rb.mean()
    d = float(np.linalg.norm(ra) * np.linalg.norm(rb))
    return float(ra @ rb / d) if d > 0 else float('nan')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--video', required=True)
    ap.add_argument('--frames', type=int, default=160)
    ap.add_argument('--conf', type=float, default=0.05,
                    help='deliberately LOW: the point is to see faint ones')
    a = ap.parse_args()

    sess = ort.InferenceSession(a.model, providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {a.video}')
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    per = {}
    for j in range(a.frames):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * (0.05 + 0.9 * j / a.frames)))
        ok, f = cap.read()
        if not ok:
            continue
        for c, conf, bw, bh in detect(sess, iname, f, a.conf):
            per.setdefault(c, []).append((float(np.sqrt(max(1.0, bw * bh))), conf))
    cap.release()

    print(f'model={os.path.basename(a.model)} clip={os.path.basename(a.video)} '
          f'conf>={a.conf}')
    print(f'{"cls":>4} {"n":>5} {"spearman":>9}   mean conf by size quartile '
          f'(Q1 smallest -> Q4 largest)')
    for c in sorted(per):
        v = np.array(per[c])
        if len(v) < 8:
            continue
        s = spearman(v[:, 0], v[:, 1])
        qs = np.quantile(v[:, 0], [0.25, 0.5, 0.75])
        means = []
        for lo, hi in ((-np.inf, qs[0]), (qs[0], qs[1]),
                       (qs[1], qs[2]), (qs[2], np.inf)):
            m = v[(v[:, 0] > lo) & (v[:, 0] <= hi)]
            means.append(m[:, 1].mean() if len(m) else float('nan'))
        print(f'{c:>4} {len(v):>5} {s:>9.2f}   '
              + '  '.join(f'{m:.3f}' for m in means)
              + f'   (px {v[:,0].min():.0f}..{v[:,0].max():.0f})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
