"""Do section 92's XFeat gains hold on REAL pairs, seconds apart?

Section 92 scored every change on one archive frame warped by a known
homography: exact truth, but the same instant in both images, the same
particles and no change of viewpoint in depth. The anchor matches a
reference stored seconds earlier, from somewhere else.

Truth for a real pair (frame A, frame B a gap later) without a rig: the
homography CHAINED through the frames between them, at a 3-frame step where
any matcher is easy and right. Two chains are built, one with LighterGlue
and one with the cosine matcher, and a pair is kept only when they agree
(corners within CHAIN_AGREE_PX) and every step had at least
CHAIN_MIN_INLIERS. Then each method matches A to B directly and is scored
against the chain: correct matches within 3 px, and a fit right when its
corners land within 3 px of the chain's (whole frame and centre target box,
as section 92). A homography is exact only for a plane, so the downward
floor clips are the clean case and the forward clips are reported apart.

Bar, written before the run: section 92's decisions stand if, pooled over
kept pairs, (1) shipped beats the pre-upstream code on correct matches by
at least 15 % and on target-box fits by at least 3 points, (2) semi-dense
fits fewer pairs than shipped, (3) LighterGlue at 1024 fits more target
boxes than shipped. Any of the three failing is reported as a reversal.

The chains favour no method by construction except that both chain
matchers are themselves candidates; a step at 3 frames is matched right by
every method, which `--self-test` cannot show and the agreement filter is
there to catch.

    python3 tools/studies/xfeat_real_pairs.py --model xfeat_320x240.onnx \\
        --glue xfeat_lighterglue.onnx --old <pre-upstream xfeat_onnx.py>
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.dirname(HERE))
import xfeat_reliability as xr  # noqa: E402
from archive_root import archive_root  # noqa: E402

FORWARD = xr.CLIPS
DOWNWARD = ('robosub/clips/octagon/octagon_1.mp4', 'final_run/octagon_Bottom.mkv',
            'final_run/bin.mkv')
STEP = 3
GAPS = (15, 30, 60)
CHAIN_MIN_INLIERS = 60
CHAIN_AGREE_PX = 1.5
PX = 3.0


def corners(w, h, roi):
    if roi:
        x0, y0 = w * 0.3, h * 0.3
        return np.float32([[x0, y0], [w - x0, y0], [w - x0, h - y0], [x0, h - y0]])
    return np.float32([[0, 0], [w, 0], [w, h], [0, h]])


def corner_err(Ha, Hb, w, h, roi=False):
    c = corners(w, h, roi).reshape(-1, 1, 2)
    a = cv2.perspectiveTransform(c, Ha).reshape(-1, 2)
    b = cv2.perspectiveTransform(c, Hb).reshape(-1, 2)
    return float(np.linalg.norm(a - b, axis=1).mean())


def fit(p0, p1):
    if len(p0) < 8:
        return None, 0
    H, m = cv2.findHomography(p0.reshape(-1, 1, 2).astype(np.float32),
                              p1.reshape(-1, 1, 2).astype(np.float32),
                              cv2.USAC_MAGSAC, 3.0)
    return (H, int(m.sum())) if H is not None and m is not None else (None, 0)


def read(path, start, n, w, h):
    cap = cv2.VideoCapture(path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    out = []
    for _ in range(n):
        ok, f = cap.read()
        if not ok:
            break
        out.append(cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (w, h)))
    cap.release()
    return out


def run(a) -> int:
    xf = xr._xfeat_module()
    spec = importlib.util.spec_from_file_location('xfeat_old', a.old)
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    net = xr.Heads(a.model)
    lg = xr._glue(a.glue)
    w, h = net.w, net.h
    match = xf.XFeatONNX.match

    def cos(f0, f1):
        (k0, d0), (k1, d1) = f0, f1
        if len(d0) < 2 or len(d1) < 2:
            return k0[:0], k1[:0]
        i0, i1 = match(d0, d1)
        return k0[i0], k1[i1]

    def glue(f0, f1):
        (k0, d0), (k1, d1) = f0, f1
        i0, i1 = lg(k0, d0, k1, d1, w, h)
        return k0[i0], k1[i1]

    methods = {
        'pre-upstream': lambda hA, hB: cos(old.keypoints_and_descriptors(hA[0], hA[1], 0.05, 1024),
                                          old.keypoints_and_descriptors(hB[0], hB[1], 0.05, 1024)),
        'shipped': lambda hA, hB: cos(xr.detect(xf, hA, True), xr.detect(xf, hB, True)),
        'semi-dense': lambda hA, hB: cos(xr.dense(hA), xr.dense(hB)),
        'LighterGlue 1024': lambda hA, hB: glue(xr.detect(xf, hA, True), xr.detect(xf, hB, True)),
        'LighterGlue 768': lambda hA, hB: glue(xr.detect(xf, hA, True, top_k=768),
                                               xr.detect(xf, hB, True, top_k=768)),
    }
    A = archive_root()
    stats = {}
    kept = tried = 0
    for group, clips in (('downward', DOWNWARD), ('forward', FORWARD)):
        for rel in clips:
            cap = cv2.VideoCapture(f'{A}/{rel}')
            total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            cap.release()
            span = max(GAPS) + 1
            for s in np.linspace(total * 0.05, total * 0.95 - span, a.starts).astype(int):
                fr = read(f'{A}/{rel}', int(s), span, w, h)
                if len(fr) < span:
                    continue
                heads = {i: net(fr[i]) for i in range(0, span, STEP)}
                feats = {i: xr.detect(xf, heads[i], True) for i in heads}
                Hg, Hc, ok = np.eye(3), np.eye(3), True
                chain = {}
                for i in range(STEP, span, STEP):
                    Sg, ng = fit(*glue(feats[i - STEP], feats[i]))
                    Sc, nc = fit(*cos(feats[i - STEP], feats[i]))
                    if Sg is None or Sc is None or min(ng, nc) < CHAIN_MIN_INLIERS:
                        ok = False
                    if not ok:
                        break
                    Hg, Hc = Sg @ Hg, Sc @ Hc
                    chain[i] = (Hg, Hc)
                for g in GAPS:
                    tried += 1
                    if g not in chain:
                        continue
                    Hg, Hc = chain[g]
                    if corner_err(Hg, Hc, w, h) > CHAIN_AGREE_PX:
                        continue
                    kept += 1
                    T = (Hg + Hc) / 2
                    for name, fn in methods.items():
                        p0, p1 = fn(heads[0], heads[g])
                        n, c = xr.score_points(p0, p1, T)
                        H, _ = fit(p0, p1)
                        rf = H is not None and corner_err(H, T, w, h) < PX
                        rb = H is not None and corner_err(H, T, w, h, roi=True) < PX
                        st = stats.setdefault((group, g, name), [0, 0, 0, 0, 0])
                        st[0] += 1
                        st[1] += n
                        st[2] += c
                        st[3] += rf
                        st[4] += rb
    print(f'{kept} of {tried} real pairs kept (chains agreed and held); '
          f'step {STEP} frames, gaps {GAPS} frames')
    pooled = {}
    for (group, g, name), (k, n, c, rf, rb) in sorted(stats.items()):
        print(f'{group:9s} gap {g:3d} {name:17s} pairs {k:3d}  correct {c / k:6.1f}  '
              f'precision {100 * c / max(1, n):5.1f}%  fit right {100 * rf / k:5.1f}% '
              f'(box {100 * rb / k:5.1f}%)')
        p = pooled.setdefault(name, [0, 0, 0, 0, 0])
        for j, v in enumerate((k, n, c, rf, rb)):
            p[j] += v
    print('pooled:')
    for name, (k, n, c, rf, rb) in pooled.items():
        print(f'  {name:17s} correct {c / k:6.1f}  precision {100 * c / max(1, n):5.1f}%  '
              f'fit right {100 * rf / k:5.1f}% (box {100 * rb / k:5.1f}%)')
    if not pooled:
        return 1
    P = {m: v for m, v in pooled.items()}
    k = P['shipped'][0]
    v1 = (P['shipped'][2] >= 1.15 * P['pre-upstream'][2]
          and P['shipped'][4] - P['pre-upstream'][4] >= 0.03 * k)
    v2 = P['semi-dense'][3] < P['shipped'][3]
    v3 = P['LighterGlue 1024'][4] > P['shipped'][4]
    for ok, what in ((v1, 'upstream XFeat beats the pre-upstream code'),
                     (v2, 'semi-dense fits fewer pairs than shipped'),
                     (v3, 'LighterGlue fits more target boxes than shipped')):
        print(('HOLDS    ' if ok else 'REVERSED ') + what)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', required=True)
    ap.add_argument('--glue', required=True)
    ap.add_argument('--old', required=True, help='xfeat_onnx.py from before section 92')
    ap.add_argument('--starts', type=int, default=12, help='start frames per clip')
    return run(ap.parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
