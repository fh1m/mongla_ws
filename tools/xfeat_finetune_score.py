#!/usr/bin/env python3
"""Did fine-tuning on our water actually help? Scored on a HELD-OUT venue.

⛔ THE ONLY NUMBER THAT COUNTS IS THE HELD-OUT ONE. Our detector's recall
swings 29.2 / 72.7 / 68.3 % across venues, so a descriptor fine-tuned on our
archive can overfit to OUR POOLS. `mirpur` was excluded from training
entirely; the murky table's two hardest clips come from it. A fine-tune scored
on its training venues would be the most flattering and least true number in
this project.

THE PROTOCOL IS §19.1's, UNCHANGED: reference at 40 % of the clip, queries at
+1/3/5/8 s, mutual-NN with min_cossim, USAC_MAGSAC, a pass at 15 inliers. Only
the weights differ, so a difference in the table is a difference in the
weights.
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

A = archive_root()
BAR = 15
OFFS = (1, 3, 5, 8)

# ⭐ HELD OUT of training. `mirpur_torpedo_1` is the canary: it reads 3/4 on
# stock weights, and its +8 s column is the one already below the bar.
# Paths taken from `feature_murky_control.py`, which is the harness that
# produced the recorded table -- so the same three files are scored.
CLIPS = {
    'mirpur_torpedo':   f'{A}/Mirpur/Sun_June_21/torpedo.mkv',
    'mirpur_torpedo_1': f'{A}/Mirpur/Sun_June_21/torpedo_1.mkv',
    'mirpur_gate':      f'{A}/Mirpur/Sun_June_21/gate.mkv',
}


def load_net(weights, dev):
    from modules.model import XFeatModel
    net = XFeatModel().to(dev).eval()
    sd = torch.load(weights, map_location=dev)
    sd = sd.get('state_dict', sd) if isinstance(sd, dict) else sd
    net.load_state_dict(sd, strict=True)
    return net


class Torch320:
    """The SHIPPED geometry -- 320x240, top_k 1024 -- driven by torch weights,
    so stock and fine-tuned go through one identical code path."""
    w, h, top_k = 320, 240, 1024

    def __init__(self, net, dev):
        self.net, self.dev = net, dev

    def detect(self, gray):
        g = cv2.resize(gray, (self.w, self.h), interpolation=cv2.INTER_AREA)
        t = torch.from_numpy(g).float()[None, None].to(self.dev) / 255.0
        with torch.inference_mode():
            feats, kpts, heat = self.net(t)
        feats = torch.nn.functional.normalize(feats, dim=1)
        hm = heat[0, 0]
        k = min(self.top_k, hm.numel())
        flat = torch.topk(hm.flatten(), k)
        ys = (flat.indices // hm.shape[1]).float()
        xs = (flat.indices % hm.shape[1]).float()
        # heatmap grid -> feature grid (both are /8 of the input here)
        fx = (xs / hm.shape[1] * feats.shape[3]).clamp(0, feats.shape[3] - 1)
        fy = (ys / hm.shape[0] * feats.shape[2]).clamp(0, feats.shape[2] - 1)
        d = feats[0, :, fy.long(), fx.long()].T.cpu().numpy()
        kp = np.stack([xs.cpu().numpy() / hm.shape[1] * self.w,
                       ys.cpu().numpy() / hm.shape[0] * self.h], 1)
        return kp.astype(np.float32), d.astype(np.float32)


def inliers(k0, d0, k1, d1, min_cossim=0.82):
    if d0 is None or d1 is None or len(d0) < 8 or len(d1) < 8:
        return 0
    S = d0 @ d1.T
    i12 = S.argmax(1)
    i21 = S.argmax(0)
    keep = i21[i12] == np.arange(len(i12))
    keep &= S[np.arange(len(i12)), i12] >= min_cossim
    if keep.sum() < 8:
        return 0
    src = k0[keep].reshape(-1, 1, 2)
    dst = k1[i12[keep]].reshape(-1, 1, 2)
    H, m = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, 3.0)
    return 0 if H is None or m is None else int(m.sum())


def run(be, path):
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if total <= 0:
        cap.release()
        return None
    ref_i = int(total * 0.40)
    cap.set(cv2.CAP_PROP_POS_FRAMES, ref_i)
    ok, im = cap.read()
    if not ok:
        cap.release()
        return None
    k0, d0 = be.detect(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
    out = []
    for s in OFFS:
        j = ref_i + int(s * fps)
        if j >= total:
            out.append(None)
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, j)
        ok, im = cap.read()
        if not ok:
            out.append(None)
            continue
        k1, d1 = be.detect(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
        out.append(inliers(k0, d0, k1, d1))
    cap.release()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stock', default='/tmp/xfeat_src/weights/xfeat.pt')
    ap.add_argument('--tuned', required=True)
    args = ap.parse_args()
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'

    arms = {}
    for tag, w in (('stock', args.stock), ('tuned', args.tuned)):
        if not os.path.exists(w):
            print(f'{tag}: missing {w}')
            return 1
        arms[tag] = Torch320(load_net(w, dev), dev)

    print('HELD-OUT venue: mirpur (never trained on)\n')
    print(f'{"clip":<20}{"stock +1/3/5/8":>26}{"ok":>5}'
          f'{"tuned +1/3/5/8":>26}{"ok":>5}')
    tot = {'stock': 0, 'tuned': 0}
    worst = {'stock': [], 'tuned': []}
    for name, path in CLIPS.items():
        if not os.path.exists(path):
            print(f'{name:<20} missing')
            continue
        row = f'{name:<20}'
        for tag in ('stock', 'tuned'):
            v = run(arms[tag], path)
            if v is None:
                row += f'{"--":>26}{"--":>5}'
                continue
            ok = sum(1 for x in v if x and x >= BAR)
            tot[tag] += ok
            worst[tag].append(min([x for x in v if x is not None] or [0]))
            row += f'{str(v):>26}{f"{ok}/4":>5}'
        print(row)

    print(f'\ntotal passing columns:  stock {tot["stock"]}   tuned {tot["tuned"]}')
    if worst['stock'] and worst['tuned']:
        print(f'worst column (the canary):  '
              f'stock {min(worst["stock"])}   tuned {min(worst["tuned"])}')
    if tot['tuned'] > tot['stock']:
        print('\n⭐ THE FINE-TUNE WINS on a venue it never saw.')
    elif tot['tuned'] == tot['stock']:
        print('\n⚠ NO DIFFERENCE in passing columns -- compare the raw inliers.')
    else:
        print('\n⛔ THE FINE-TUNE LOSES. Round 7 closes NO; keep stock weights.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
