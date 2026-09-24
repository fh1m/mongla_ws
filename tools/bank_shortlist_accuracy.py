#!/usr/bin/env python3
"""Does a cheap signature retrieve the RIGHT reference out of a large bank?

`bank_scaling_bench.py` measured that a shortlist makes bank size free: 100
references cost what 10 do, because only the top-k are matched. That is a
speed result on random descriptors and it proves nothing about correctness.

This is the correctness half, on real footage. The signature under test is the
one we can afford -- the L2-normalised MEAN of a reference's XFeat descriptors,
64 numbers, already computed. No new network, no DINOv2 (measured 3064 ms on
our Pi), no NetVLAD.

THE PROTOCOL. Snap references across a clip at a fixed stride. For each query
frame, match it against EVERY reference to find which one truly yields the most
inliers -- that is ground truth, obtained the expensive way. Then rank the
references by signature similarity and ask whether the true best is inside the
top-k. That is recall@k, and it is the number that decides whether the bank can
grow.

⚠ WHY THE FLOOR MATTERS MORE THAN THE MEAN. A shortlist that is right 90 % of
the time and catastrophically wrong 10 % of the time is not 90 % as good: the
misses are where the lock breaks. So this reports recall@1/3/5 AND how much
inlier yield is lost when the shortlist is wrong, which is the quantity the
ladder actually feels.
"""
from __future__ import annotations

import argparse
import importlib.util
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCHIVE = '/home/fh1m/Work/Projects/Duburi/2025/raw_videos'

CLIPS = {
    'mirpur_torpedo':   f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo.mkv',
    'mirpur_torpedo_1': f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo_1.mkv',
    'mirpur_gate':      f'{ARCHIVE}/Mirpur/Sun_June_21/gate.mkv',
    'octagon':          f'{ARCHIVE}/robosub/clips/octagon/octagon_1.mp4',
    'torpedo_clear':    f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
}


def load_backend(model, threads):
    p = os.path.join(HERE, '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'xfeat_onnx.py')
    spec = importlib.util.spec_from_file_location('xfeat_onnx', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m, m.XFeatONNX(model, top_k=1024, threads=threads)


def signature(desc):
    if len(desc) == 0:
        return np.zeros(64, np.float32)
    v = desc.mean(axis=0)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def inliers(mod, net, k0, d0, k1, d1, ransac_px=3.0):
    if len(d0) < 4 or len(d1) < 4:
        return 0
    i0, i1 = net.match(d0, d1)
    if len(i0) < 8:
        return 0
    src = k0[i0].astype(np.float32).reshape(-1, 1, 2)
    dst = k1[i1].astype(np.float32).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, ransac_px)
    return 0 if H is None or mask is None else int(mask.sum())


def frames(path, stride_s, limit):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return []
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    step = max(1, int(round(fps * stride_s)))
    out = []
    for i in range(0, n, step):
        if len(out) >= limit:
            break
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = cap.read()
        if ok:
            out.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
    cap.release()
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--stride', type=float, default=4.0,
                    help='seconds between bank references')
    ap.add_argument('--refs', type=int, default=24)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--clips', default='')
    a = ap.parse_args()

    mod, net = load_backend(a.model, a.threads)
    want = [c.strip() for c in a.clips.split(',') if c.strip()] or list(CLIPS)

    print(f'model={os.path.basename(a.model)} stride={a.stride}s '
          f'refs<={a.refs}  signature=mean XFeat descriptor (64-D)')
    print(f'{"clip":<18} {"refs":>5} {"queries":>8} {"r@1":>7} {"r@3":>7} '
          f'{"r@5":>7} {"inlier loss @5":>15}')

    tot = {1: 0, 3: 0, 5: 0}
    tot_q = 0
    losses = []
    for name in want:
        fs = frames(CLIPS[name], a.stride, a.refs)
        if len(fs) < 4:
            print(f'{name:<18} UNREADABLE or too short')
            continue
        feats = [net.detect(f) for f in fs]
        sigs = np.stack([signature(d) for _, d in feats])

        hit = {1: 0, 3: 0, 5: 0}
        q = 0
        # Every frame is a query against all the OTHERS as a bank, which is the
        # honest version: a reference never retrieves itself.
        for qi in range(len(fs)):
            idx = [j for j in range(len(fs)) if j != qi]
            kq, dq = feats[qi]
            if len(dq) == 0:
                continue
            truth = [inliers(mod, net, feats[j][0], feats[j][1], kq, dq)
                     for j in idx]
            best = int(np.argmax(truth))
            if truth[best] <= 0:
                continue                      # nothing to retrieve; not a miss
            order = np.argsort(-(sigs[idx] @ sigs[qi]))
            q += 1
            for k in (1, 3, 5):
                topk = order[:k]
                if best in topk:
                    hit[k] += 1
                if k == 5:
                    got = max(truth[t] for t in topk)
                    losses.append(1.0 - got / max(1, truth[best]))
        if q == 0:
            print(f'{name:<18} no usable queries')
            continue
        tot_q += q
        for k in (1, 3, 5):
            tot[k] += hit[k]
        ml = float(np.mean(losses[-q:])) if q else 0.0
        print(f'{name:<18} {len(fs):>5} {q:>8} '
              f'{hit[1]/q:>6.1%} {hit[3]/q:>6.1%} {hit[5]/q:>6.1%} '
              f'{ml:>14.1%}')

    if tot_q:
        print(f'{"ALL":<18} {"":>5} {tot_q:>8} '
              f'{tot[1]/tot_q:>6.1%} {tot[3]/tot_q:>6.1%} {tot[5]/tot_q:>6.1%} '
              f'{float(np.mean(losses)):>14.1%}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
