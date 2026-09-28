#!/usr/bin/env python3
"""Where does confidence actually peak, in the unit the CONTROLLER uses?

⛔ THE GAP THIS CLOSES. Section 23 measured detector confidence against apparent
size in PIXELS (sqrt of box area) and reported it by quartile -- a real
measurement, and the one that established the peak is not at minimum range.
`mongla_vision/approach.py` then shipped a band of **0.25-0.45 fraction of
frame height** and cited section 23 for it.

Those are not the same quantity. A quartile boundary in pixels of sqrt-area
does not convert to a fraction of frame height without knowing the aspect ratio
and the frame size, and nothing recorded that conversion. So the shipped
numbers are an invention with a citation attached, which is exactly the failure
mode `measured-bars.md` exists to prevent -- and it is the reason `approach.py`
must not be wired until this runs.

This measures the real thing in the controller's own unit: mean confidence per
bin of `box_height / frame_height`, which is what `advise(box_h_px,
frame_h_px)` compares against, and what `_fill(sample, 'height')` computes on
the vehicle.

    python3 tools/approach_band.py --model src/mongla_vision/models/<m>.onnx \
        --video A.mkv --video B.mkv

⚠ SAME CONFOUND AS SECTION 23, RESTATED: a closer target is also better lit,
less backscattered and more centred, so this measures the whole approach rather
than scale alone. That is the right quantity for "where should the controller
stop?" and the wrong one for a claim about scale sensitivity in isolation.

⚠ A BIN WITH FEW SAMPLES IS NOT AN ANSWER. Bins below `--min-bin` print `--`
rather than a mean, because one lucky detection in an empty bin is how a peak
gets invented.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onnxruntime as ort                                        # noqa: E402
from conf_vs_range import detect                                 # noqa: E402

# Fraction of frame HEIGHT. Deliberately finer around the shipped 0.25-0.45
# band than outside it, so the question "is that band where the peak is?" has
# more than one bin to answer with.
BINS = ((0.00, 0.10), (0.10, 0.20), (0.20, 0.30), (0.30, 0.45),
        (0.45, 0.60), (0.60, 0.80), (0.80, 1.20))


def sample(model: str, clips, frames: int, conf: float):
    sess = ort.InferenceSession(model, providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name
    per: dict = {}
    for clip in clips:
        cap = cv2.VideoCapture(clip)
        if not cap.isOpened():
            print(f'  cannot open {clip}', file=sys.stderr)
            continue
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        for j in range(frames):
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * (0.05 + 0.9 * j / frames)))
            ok, f = cap.read()
            if not ok:
                continue
            fh = f.shape[0]
            for c, cf, _bw, bh in detect(sess, iname, f, conf):
                per.setdefault(int(c), []).append((bh / fh, cf))
        cap.release()
    return per


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--video', action='append', default=[], required=True)
    ap.add_argument('--frames', type=int, default=200)
    ap.add_argument('--conf', type=float, default=0.05,
                    help='deliberately LOW: faint detections are the point')
    ap.add_argument('--min-bin', type=int, default=8,
                    help='below this many samples a bin prints -- not a mean')
    a = ap.parse_args()

    per = sample(a.model, a.video, a.frames, a.conf)
    print(f'\nmodel={os.path.basename(a.model)}  clips={len(a.video)}  '
          f'conf>={a.conf}  frames/clip={a.frames}')
    print('bins are box_height / frame_height -- the unit approach.advise uses')
    print(f'{"cls":>4} {"n":>6}   '
          + '  '.join(f'{lo:.2f}-{hi:.2f}' for lo, hi in BINS))
    any_row = False
    for c in sorted(per):
        v = np.array(per[c])
        if len(v) < 4 * a.min_bin:
            continue
        any_row = True
        means, counts = [], []
        for lo, hi in BINS:
            m = v[(v[:, 0] >= lo) & (v[:, 0] < hi)]
            means.append(float(m[:, 1].mean()) if len(m) >= a.min_bin
                         else float('nan'))
            counts.append(len(m))
        best = (int(np.nanargmax(means)) if not all(np.isnan(x) for x in means)
                else -1)
        cells = '  '.join('  --   ' if np.isnan(x)
                          else (f'*{x:.3f} ' if i == best else f' {x:.3f} ')
                          for i, x in enumerate(means))
        print(f'{c:>4} {len(v):>6}   {cells}')
        print(f'{"":>4} {"n=":>6}   '
              + '  '.join(f'{n:>7d}' for n in counts))
        if best >= 0:
            print(f'{"":>4} {"peak":>6}   {BINS[best][0]:.2f}-{BINS[best][1]:.2f} '
                  f'of frame height')
    if not any_row:
        print('  no class had enough detections to bin -- lower --conf, add '
              'clips, or this model sees nothing here')
    print('\n⚠ Report the PEAK BIN, not a band you liked the look of. '
          'approach.py ships BAND_LO/BAND_HI and they must come from here.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
