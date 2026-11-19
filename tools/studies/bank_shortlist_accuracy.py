#!/usr/bin/env python3
"""Does a cheap signature retrieve the RIGHT reference out of a large bank?

`bank_scaling_bench.py` measured that a shortlist makes bank size free: 100
references cost what 10 do, because only the top-k are matched. That is a
speed result on random descriptors and it proves nothing about correctness.

This is the correctness half, on real footage. The signature under test is the
one we can afford -- the L2-normalised MEAN of a reference's XFeat descriptors,
64 numbers, already computed. No new network, no DINOv2 (measured 3064 ms on
our Pi), no NetVLAD.

The protocol. Snap references across a clip at a fixed stride. For each query
frame, match it against EVERY reference to find which one truly yields the most
inliers -- that is ground truth, obtained the expensive way. Then rank the
references by signature similarity and ask whether the true best is inside the
top-k. That is recall@k, and it is the number that decides whether the bank can
grow.

Why the floor matters more than the mean. A shortlist that is right 90 % of
the time and catastrophically wrong 10 % of the time is not 90 % as good: the
misses are where the lock breaks. So this reports recall@1/3/5 AND how much
inlier yield is lost when the shortlist is wrong, which is the quantity the
ladder actually feels.

Other signatures (`--signatures mean,vlad16,vlad64`), each scored against the
same truth on the same queries. VLAD sums each descriptor's residual to its
nearest of K centroids, then power- and L2-normalises (Jegou et al. 2010):
K x 64 numbers from the descriptors the bank already holds, no new network.
Its vocabulary is trained on the OTHER clips only, so no clip is ranked by
words learnt from itself. `--set downward` adds the floor clips section 20.2
owes a number for.

Bar, written before the first run (bead po7.15.4): a signature replaces the
mean when, pooled over the forward clips, Recall@1 rises by at least 10
points with the lower 95 % bound of the paired bootstrap above zero, the
downward Recall@1 is not lower by more than 2 points, and loss@5 does not
rise. Anything less keeps the mean.
"""
from __future__ import annotations

import argparse
import importlib.util
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
_sys.path.insert(1, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))  # tools/: shared libraries
from archive_root import archive_root
ARCHIVE = archive_root()

CLIPS = {
    'mirpur_torpedo':   f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo.mkv',
    'mirpur_torpedo_1': f'{ARCHIVE}/Mirpur/Sun_June_21/torpedo_1.mkv',
    'mirpur_gate':      f'{ARCHIVE}/Mirpur/Sun_June_21/gate.mkv',
    'octagon':          f'{ARCHIVE}/robosub/clips/octagon/octagon_1.mp4',
    'torpedo_clear':    f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
}
DOWNWARD = {
    'octagon_bottom':   f'{ARCHIVE}/final_run/octagon_Bottom.mkv',
    'bin':              f'{ARCHIVE}/final_run/bin.mkv',
}


def load_backend(model, threads):
    p = os.path.join(HERE, '..', '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'xfeat_onnx.py')
    spec = importlib.util.spec_from_file_location('xfeat_onnx', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m, m.XFeatONNX(model, top_k=1024, threads=threads)


def _place_signature():
    """The shipped module, loaded by path like the backend: one copy of vlad()."""
    p = os.path.join(HERE, '..', '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'place_signature.py')
    spec = importlib.util.spec_from_file_location('place_signature', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


vlad = _place_signature().vlad


def signature(desc):
    if len(desc) == 0:
        return np.zeros(64, np.float32)
    v = desc.mean(axis=0)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def vocabulary(descs, k, seed=0):
    """K centroids of the pooled descriptors, k-means++ (OpenCV)."""
    x = np.concatenate([d for d in descs if len(d)]).astype(np.float32)
    rng = np.random.default_rng(seed)
    if len(x) > 60000:
        x = x[rng.choice(len(x), 60000, replace=False)]
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    cv2.setRNGSeed(seed)
    _, _, c = cv2.kmeans(x, k, None, crit, 3, cv2.KMEANS_PP_CENTERS)
    return c


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
    ap.add_argument('--set', choices=('forward', 'downward', 'both'), default='forward')
    ap.add_argument('--signatures', default='mean')
    ap.add_argument('--boot', type=int, default=10000)
    ap.add_argument('--write-vocab', default='',
                    help='after scoring, learn the 16 words from EVERY clip read and write '
                         'them here (the shipped config/vlad16.npz)')
    a = ap.parse_args()

    mod, net = load_backend(a.model, a.threads)
    pool = {**(CLIPS if a.set != 'downward' else {}), **(DOWNWARD if a.set != 'forward' else {})}
    want = [c.strip() for c in a.clips.split(',') if c.strip()] or list(pool)
    sigs_wanted = [x.strip() for x in a.signatures.split(',') if x.strip()]
    print(f'model={os.path.basename(a.model)} stride={a.stride}s refs<={a.refs} '
          f'signatures={sigs_wanted}')

    feats = {}
    for name in want:
        fs = frames(pool[name], a.stride, a.refs)
        if len(fs) < 4:
            print(f'{name:<18} UNREADABLE or too short')
            continue
        feats[name] = [net.detect(f) for f in fs]

    # Truth once per clip: for every query, the inliers against every other
    # frame. Every signature is then ranked against the same numbers.
    truth = {}
    for name, fe in feats.items():
        rows = []
        for qi in range(len(fe)):
            idx = [j for j in range(len(fe)) if j != qi]
            kq, dq = fe[qi]
            t = [inliers(mod, net, fe[j][0], fe[j][1], kq, dq) for j in idx] if len(dq) else []
            rows.append((idx, t))
        truth[name] = rows

    KS = (1, 2, 3, 5, 8)
    hdr = ' '.join(f'{"r@" + str(k):>6}' for k in KS)
    hit1 = {}                                      # (sig, clip) -> per-query hit@1
    summary = {}
    for sig in sigs_wanted:
        print(f'\n[{sig}]')
        print(f'{"clip":<18} {"refs":>5} {"q":>4} {hdr}  {"loss@5":>7}')
        for group, members in (('forward', CLIPS), ('downward', DOWNWARD)):
            tot, tq, losses = {k: 0 for k in KS}, 0, []
            for name in [n for n in feats if n in members]:
                fe = feats[name]
                if sig == 'mean':
                    S = np.stack([signature(d) for _, d in fe])
                else:
                    words = vocabulary([d for o, f2 in feats.items() if o != name for _, d in f2],
                                       int(sig[len('vlad'):]))
                    S = np.stack([vlad(d, words) for _, d in fe])
                hit, q, h1, ls = {k: 0 for k in KS}, 0, [], []
                for qi, (idx, t) in enumerate(truth[name]):
                    if not t or max(t) <= 0:
                        continue                   # nothing to retrieve; not a miss
                    best = int(np.argmax(t))
                    order = np.argsort(-(S[idx] @ S[qi]))
                    q += 1
                    for k in KS:
                        hit[k] += best in order[:k]
                    h1.append(float(best == order[0]))
                    ls.append(1.0 - max(t[o] for o in order[:5]) / t[best])
                if not q:
                    continue
                hit1[(sig, name)] = h1
                losses += ls
                tq += q
                for k in KS:
                    tot[k] += hit[k]
                r = ' '.join(f'{hit[k] / q:>6.1%}' for k in KS)
                print(f'{name:<18} {len(fe):>5} {q:>4} {r}  {float(np.mean(ls)):>7.1%}')
            if tq:
                r = ' '.join(f'{tot[k] / tq:>6.1%}' for k in KS)
                print(f'{"ALL " + group:<18} {"":>5} {tq:>4} {r}  {float(np.mean(losses)):>7.1%}')
                summary[(sig, group)] = (tot[1] / tq, float(np.mean(losses)))

    if a.write_vocab:
        words = vocabulary([d for fe in feats.values() for _, d in fe], 16)
        os.makedirs(os.path.dirname(os.path.abspath(a.write_vocab)), exist_ok=True)
        np.savez(a.write_vocab, words=words.astype(np.float32),
                 trained_on=np.array(sorted(feats)), seed=np.array([0]),
                 model=np.array([os.path.basename(a.model)]))
        print(f'wrote {a.write_vocab}: 16 words from {sorted(feats)}')
    if 'mean' not in sigs_wanted:
        return 0
    rng = np.random.default_rng(0)
    for sig in [x for x in sigs_wanted if x != 'mean']:
        verdict = []
        for group, members in (('forward', CLIPS), ('downward', DOWNWARD)):
            names = [n for n in feats if n in members and (sig, n) in hit1]
            if not names:
                continue
            d = np.concatenate([np.array(hit1[(sig, n)]) - np.array(hit1[('mean', n)]) for n in names])
            b = d[rng.integers(0, len(d), (a.boot, len(d)))].mean(1)
            lo, hi = np.percentile(b, (2.5, 97.5))
            dl = summary[(sig, group)][1] - summary[('mean', group)][1]
            print(f'{sig} - mean, {group}: Recall@1 {100 * d.mean():+.1f} points '
                  f'(95 % {100 * lo:+.1f} to {100 * hi:+.1f}, n={len(d)}), loss@5 {100 * dl:+.1f} points')
            verdict.append((group, d.mean(), lo, dl))
        g = {v[0]: v for v in verdict}
        ok = ('forward' in g and g['forward'][1] >= 0.10 and g['forward'][2] > 0 and g['forward'][3] <= 0
              and ('downward' not in g or g['downward'][1] >= -0.02))
        print(f'VERDICT {sig}: {"ADOPT" if ok else "KEEP the mean"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
