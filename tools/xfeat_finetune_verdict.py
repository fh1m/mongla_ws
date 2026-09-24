#!/usr/bin/env python3
"""Did the fine-tune help, or did it learn OUR POOLS? The decisive test.

A single crossing column is not a result. This widens the evidence until a
lucky checkpoint cannot carry the verdict:

  * MANY query offsets, not four -- one column crossing a bar by one inlier
    is within run-to-run scatter, and the bar is a cliff that hides the size
    of a change.
  * TOTAL INLIERS, not just passing columns -- the pass/fail count throws away
    how much better or worse each pair got.
  * BOTH venue sets. ⭐ THIS IS THE ONE THAT MATTERS: if the fine-tune wins on
    venues it trained on and loses on the held-out venue, that is overfitting,
    and it is the failure the whole held-out split exists to catch.

⛔ No pass/fail verdict is printed from one number here. The point is to see
the shape.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, '/tmp/xfeat_src')
from archive_root import archive_root                        # noqa: E402
from xfeat_finetune_score import Torch320, inliers, load_net  # noqa: E402

A = archive_root()

HELD_OUT = {
    'mirpur_torpedo':   f'{A}/Mirpur/Sun_June_21/torpedo.mkv',
    'mirpur_torpedo_1': f'{A}/Mirpur/Sun_June_21/torpedo_1.mkv',
    'mirpur_gate':      f'{A}/Mirpur/Sun_June_21/gate.mkv',
}
TRAINED_ON = {
    'octagon':       f'{A}/robosub/clips/octagon/octagon_1.mp4',
    'torpedo_clear': f'{A}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
    'bin':           f'{A}/final_run/bin.mkv',
    'octagon_bot':   f'{A}/final_run/octagon_Bottom.mkv',
}


def pairs(be, path, n_ref=6, offs=(1, 2, 3, 4, 5, 6, 8, 10)):
    """Many reference points and many offsets, so the sample is not four
    numbers from one instant of one clip."""
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if total <= 0:
        cap.release()
        return []
    out = []
    for frac in np.linspace(0.15, 0.75, n_ref):
        ri = int(total * frac)
        cap.set(cv2.CAP_PROP_POS_FRAMES, ri)
        ok, im = cap.read()
        if not ok:
            continue
        k0, d0 = be.detect(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
        for s in offs:
            j = ri + int(s * fps)
            if j >= total:
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, j)
            ok, im2 = cap.read()
            if not ok:
                continue
            k1, d1 = be.detect(cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY))
            out.append(inliers(k0, d0, k1, d1))
    cap.release()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stock', default='/tmp/xfeat_src/weights/xfeat.pt')
    ap.add_argument('--tuned', required=True)
    ap.add_argument('--bar', type=int, default=15)
    args = ap.parse_args()
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    arms = {'stock': Torch320(load_net(args.stock, dev), dev),
            'tuned': Torch320(load_net(args.tuned, dev), dev)}

    for label, clips in (('HELD-OUT (never trained on)', HELD_OUT),
                         ('TRAINED-ON venues', TRAINED_ON)):
        print(f'\n=== {label} ===')
        agg = {'stock': [], 'tuned': []}
        for name, path in clips.items():
            if not os.path.exists(path):
                continue
            row = f'  {name:<16}'
            for tag in ('stock', 'tuned'):
                v = pairs(arms[tag], path)
                agg[tag].extend(v)
                if v:
                    a = np.array(v)
                    row += (f'{tag} n={len(a):<3} med={np.median(a):6.1f} '
                            f'pass={100*(a>=args.bar).mean():5.1f}%   ')
            print(row)
        if agg['stock'] and agg['tuned']:
            s, t = np.array(agg['stock']), np.array(agg['tuned'])
            ps, pt = 100*(s >= args.bar).mean(), 100*(t >= args.bar).mean()
            print(f'  {"TOTAL":<16}stock n={len(s)} med={np.median(s):.1f} '
                  f'pass={ps:.1f}%   tuned n={len(t)} '
                  f'med={np.median(t):.1f} pass={pt:.1f}%')
            print(f'  {"delta":<16}median {np.median(t)-np.median(s):+.1f}   '
                  f'pass-rate {pt-ps:+.1f} pp')
    return 0


if __name__ == '__main__':
    sys.exit(main())
