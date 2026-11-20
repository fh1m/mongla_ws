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

Learned signatures (`LEARNED`: cosplace, megaloc, boq_r50, anyloc_s) are
whole-frame networks loaded through torch.hub (measured-bars 20.4, 20.5);
they need torch, so it is imported only when one is asked for. Every network
sees the clip's own colour frame. The archive's downward clips are colour
although Mongla's downward camera is mono, so on the floor the learned
signatures are flattered, not handicapped.
`--against` names the incumbent each candidate is scored against.

`--revisit` asks the question the bank exists for: the bank is one run, the
queries a later run past the same props (REVISITS), and besides recall it
reports, per shortlist width, how often the shortlist still holds a
reference that verifies (measured-bars 20.5).

Bar, written before the first run (bead po7.15.4): a signature replaces the
incumbent when, pooled over the forward clips, Recall@1 rises by at least 10
points with the lower 95 % bound of the paired bootstrap above zero, the
downward Recall@1 is not lower by more than 2 points, and loss@5 does not
rise. Anything less keeps the mean.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import time

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


# Learned signatures, each a torch.hub network fed the whole frame. Only one
# is held at a time (the dev box has little memory to spare). anyloc_s returns
# DINOv2 patch tokens, which go through VLAD like XFeat's descriptors.
LEARNED = {
    'cosplace': (dict(repo='gmberton/cosplace', model='get_trained_model', backbone='ResNet18',
                      fc_output_dim=512), (320, 240)),
    'megaloc': (dict(repo='gmberton/MegaLoc', model='get_trained_model'), (322, 238)),
    'boq_r50': (dict(repo='amaralibey/bag-of-queries', model='get_trained_boq',
                     backbone_name='resnet50', output_dim=16384), (320, 320)),
    'anyloc_s': (dict(repo='facebookresearch/dinov2', model='dinov2_vits14'), (322, 238)),
}
_NET = {}
_WORDS = {}      # (signature, excluded clips) -> VLAD words, learnt once


def learned(sig, bgr):
    """One whole-frame signature (unit vector), or patch tokens for anyloc_s."""
    import torch
    kw, (w, h) = LEARNED[sig]
    if sig not in _NET:
        _NET.clear()
        kw = dict(kw)
        _NET[sig] = torch.hub.load(kw.pop('repo'), kw.pop('model'), trust_repo=True,
                                   skip_validation=True, **kw).eval()
    m = _NET[sig]
    x = cv2.resize(bgr, (w, h))[:, :, ::-1].astype(np.float32) / 255.0
    x = (x - (0.485, 0.456, 0.406)) / (0.229, 0.224, 0.225)
    x = torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1), np.float32))[None]
    with torch.no_grad():
        if sig == 'anyloc_s':
            y = m.forward_features(x)['x_norm_patchtokens'][0].numpy()
            return y / np.linalg.norm(y, axis=1, keepdims=True)
        y = m(x)
    y = (y[0] if isinstance(y, (tuple, list)) else y)[0].numpy().astype(np.float32)
    return y / np.linalg.norm(y)


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
            out.append(f)
    cap.release()
    return out


# Runs past the same props on different occasions: the bank is one, the
# queries the other, both ways round (measured-bars 20.5, the revisit).
REVISITS = {
    'mirpur_torpedo': ['mirpur_torpedo_1'],
    'octagon_1': ['octagon_2'],
    'bin_front_1': ['bin_front_2', 'bin_front_3'],
    'bin_front_2': ['bin_front_3'],
    'octagon_front_1': ['octagon_front_2'],
    'shark_up_1': ['shark_up_2', 'shark_up_3'],
    'shark_up_2': ['shark_up_3'],
}
REVISIT_CLIPS = {
    'mirpur_torpedo': CLIPS['mirpur_torpedo'], 'mirpur_torpedo_1': CLIPS['mirpur_torpedo_1'],
    'octagon_1': f'{ARCHIVE}/robosub/clips/octagon/octagon_1.mp4',
    'octagon_2': f'{ARCHIVE}/robosub/clips/octagon/octagon_2.mp4',
    **{f'bin_front_{i}': f'{ARCHIVE}/robosub/clips/bin_front/bin_front_#{i}.mp4' for i in (1, 2, 3)},
    'octagon_front_1': f'{ARCHIVE}/robosub/clips/octagon_front/octagon_front_1.mp4',
    'octagon_front_2': f'{ARCHIVE}/robosub/clips/octagon_front/octagone_front_2.mp4',
    'shark_up_1': f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
    'shark_up_2': f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_2.mkv',
    'shark_up_3': f'{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_3.mkv',
}


def shipped(module, name):
    """A threshold read from the anchor module that ships it, not a copy."""
    import re
    p = os.path.join(HERE, '..', '..', 'src', 'mongla_vision', 'mongla_vision', 'anchor', module)
    return int(re.search(rf'^{name}\s*=\s*(\d+)', open(p).read(), re.M).group(1))


def read_all(net, paths, names, stride, refs, sigs):
    feats, images, raw = {}, {}, {}
    for name in names:
        fs = frames(paths[name], stride, refs)
        if len(fs) < 4:
            print(f'{name:<18} UNREADABLE or too short')
            continue
        feats[name] = [net.detect(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)) for f in fs]
        if any(x in LEARNED for x in sigs):                  # raw frames only when a network needs them
            raw[name] = fs
    for sig in [x for x in sigs if x in LEARNED]:            # one network at a time
        n = sum(len(f) for f in raw.values())
        if n == 0:
            continue
        learned(sig, next(iter(raw.values()))[0])            # load the network outside the timer
        t0 = time.perf_counter()
        for name, fs in raw.items():
            images[(sig, name)] = [learned(sig, f) for f in fs]
        print(f'{sig}: {1000 * (time.perf_counter() - t0) / n:.0f} ms a frame on this machine')
    return feats, images


def stack(sig, name, feats, images, exclude):
    """One clip's signatures; VLAD words are learnt only on clips outside `exclude`."""
    fe = feats[name]
    if sig == 'mean':
        return np.stack([signature(d) for _, d in fe])
    if sig in LEARNED and sig != 'anyloc_s':
        return np.stack(images[(sig, name)])
    key = (sig, frozenset(exclude))                          # the words depend on nothing else
    if key not in _WORDS:
        if sig == 'anyloc_s':
            _WORDS[key] = vocabulary([d for (g, o), ds in images.items()
                                      if g == sig and o not in exclude for d in ds], 16)
        else:
            _WORDS[key] = vocabulary([d for o, f2 in feats.items() if o not in exclude
                                      for _, d in f2], int(sig[len('vlad'):]))
    words = _WORDS[key]
    if sig == 'anyloc_s':
        return np.stack([vlad(d, words) for d in images[(sig, name)]])
    return np.stack([vlad(d, words) for _, d in fe])


def boot_verdict(per, sigs, inc, groups, boot, loss):
    """Paired bootstrap of Recall@1, each signature against the incumbent."""
    rng = np.random.default_rng(0)
    out = {}
    for sig in [x for x in sigs if x != inc]:
        for group in groups:
            if (sig, group) not in per:
                continue
            d = np.array(per[(sig, group)]) - np.array(per[(inc, group)])
            b = d[rng.integers(0, len(d), (boot, len(d)))].mean(1)
            lo, hi = np.percentile(b, (2.5, 97.5))
            dl = loss[(sig, group)] - loss[(inc, group)]
            print(f'{sig} - {inc}, {group}: Recall@1 {100 * d.mean():+.1f} points '
                  f'(95 % {100 * lo:+.1f} to {100 * hi:+.1f}, n={len(d)}), loss@5 {100 * dl:+.1f} points')
            out[(sig, group)] = (d.mean(), lo, dl)
    return out


KS = (1, 2, 3, 5, 8)


def revisit(a, mod, net, sigs):
    floor = shipped('anchor.py', 'MIN_INLIERS')
    identity = shipped('bank.py', 'IDENTITY_INLIERS')
    names = sorted(set(REVISITS) | {q for qs in REVISITS.values() for q in qs})
    feats, images = read_all(net, REVISIT_CLIPS, names, a.stride, a.refs, sigs)
    pairs = [(b, q) for b, qs in REVISITS.items() for q in qs]
    pairs += [(q, b) for b, q in pairs]
    truth = {}
    for b, q in pairs:
        if b in feats and q in feats:
            truth[(b, q)] = [[inliers(mod, net, kb, db, kq, dq) for kb, db in feats[b]]
                             for kq, dq in feats[q]]
    print(f'revisit: {len(truth)} bank/query pairs, a query counts at >= {floor} inliers')
    per, loss = {}, {}
    for sig in sigs:
        h1, h5, ls = [], [], []
        kept = {(f, k): [] for f in (floor, identity) for k in KS}
        print(f'\n[{sig}]  {"bank":<16} {"query":<16} {"q":>3} {"r@1":>6} {"r@5":>6}')
        for (b, q), rows in truth.items():
            Sb, Sq = (stack(sig, n, feats, images, {b, q}) for n in (b, q))
            ph1 = []
            for qi, t in enumerate(rows):
                if max(t) < floor:
                    continue                       # the place is not in this bank
                best, order = int(np.argmax(t)), np.argsort(-(Sb @ Sq[qi]))
                ph1.append(float(best == order[0]))
                h5.append(float(best in order[:5]))
                ls.append(1.0 - max(t[o] for o in order[:5]) / t[best])
                for f in (floor, identity):         # does the shortlist keep a usable match?
                    if t[best] >= f:
                        for k in KS:
                            kept[(f, k)].append(max(t[o] for o in order[:k]) >= f)
            h1 += ph1
            if ph1:
                n5 = h5[-len(ph1):]
                print(f'{"":<{len(sig) + 3}}{b:<16} {q:<16} {len(ph1):>3} {np.mean(ph1):>6.1%} '
                      f'{np.mean(n5):>6.1%}')
        if h1:
            print(f'  ALL revisit: q={len(h1)} r@1 {np.mean(h1):.1%} r@5 {np.mean(h5):.1%} '
                  f'loss@5 {np.mean(ls):.1%}')
            per[(sig, 'revisit')], loss[(sig, 'revisit')] = h1, float(np.mean(ls))
            for f, what in ((floor, 'a match (MIN_INLIERS)'), (identity, 'identity (IDENTITY_INLIERS)')):
                n = len(kept[(f, KS[0])])
                print(f'  {what} kept, of {n}: ' + '  '.join(
                    f'k={k} {np.mean(kept[(f, k)]):.1%}' for k in KS))
    if a.against in sigs:
        boot_verdict(per, sigs, a.against, ('revisit',), a.boot, loss)
    return 0


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
    ap.add_argument('--against', default='mean', help='the incumbent each other signature is scored against')
    ap.add_argument('--boot', type=int, default=10000)
    ap.add_argument('--revisit', action='store_true',
                    help='bank from one run, queries from another run past the same props')
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

    if a.revisit:
        return revisit(a, mod, net, sigs_wanted)
    feats, images = read_all(net, pool, want, a.stride, a.refs, sigs_wanted)

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
                S = stack(sig, name, feats, images, {name})
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
    inc = a.against
    if inc not in sigs_wanted:
        return 0
    per = {(sig, g): [x for n in feats if n in members and (sig, n) in hit1 for x in hit1[(sig, n)]]
           for sig in sigs_wanted for g, members in (('forward', CLIPS), ('downward', DOWNWARD))}
    per = {k: v for k, v in per.items() if v}
    loss = {k: v[1] for k, v in summary.items()}
    g = boot_verdict(per, sigs_wanted, inc, ('forward', 'downward'), a.boot, loss)
    for sig in [x for x in sigs_wanted if x != inc]:
        f, d = g.get((sig, 'forward')), g.get((sig, 'downward'))
        ok = f is not None and f[0] >= 0.10 and f[1] > 0 and f[2] <= 0 and (d is None or d[0] >= -0.02)
        print(f'VERDICT {sig}: {"ADOPT" if ok else "KEEP " + inc}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
