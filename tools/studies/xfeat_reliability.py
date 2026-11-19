#!/usr/bin/env python3
"""Does XFeat's reliability head pick better keypoints for the anchor?

XFeat has three heads. Upstream (verlab/accelerated_features, `xfeat.py`
detectAndCompute) ranks the NMS keypoints by keypoint probability TIMES the
reliability map sampled at each one, and keeps the top-k of that product. Our
backends ranked by keypoint probability alone, so whenever NMS leaves more than
`top_k`, the anchor keeps a different set than the method's authors would.

Truth by construction, not a self-fitted inlier count: a real frame from the
murky archive is warped by a homography chosen here, with noise and blur, so
where every keypoint SHOULD land is known. A match is correct when the
keypoint in the warped frame is within 3 px (backend grid) of where the true
homography sends its partner.

Bar, written before the run:
  * fidelity: with reliability, our keypoints are upstream's (torch,
    detectAndCompute) on >= 99 % of keypoints, frame by frame. Without it they
    must differ on frames where NMS exceeds top_k, or the comparison is moot.
  * adopt reliability ranking if, pooled over every clip, correct matches rise
    by >= 5 % AND precision (correct / matches) falls by no more than 1 point
    on any clip. Otherwise keep keypoint-only and record why.
  * frames where NMS leaves <= top_k cannot differ; their share is reported,
    because a gain measured only where the two agree would be noise.

    python3 tools/studies/xfeat_reliability.py --self-test
    python3 tools/studies/xfeat_reliability.py --model X.onnx --reference <accelerated_features>
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(1, os.path.dirname(HERE))
from archive_root import archive_root  # noqa: E402

CLIPS = ('Mirpur/Sun_June_21/torpedo.mkv', 'Mirpur/Sun_June_21/torpedo_1.mkv',
         'Mirpur/Sun_June_21/gate.mkv', 'robosub/clips/torpedo/torpedo_shark_up_1.mp4')
PX = 3.0


def _xfeat_module():
    p = os.path.join(HERE, '..', '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'xfeat_onnx.py')
    spec = importlib.util.spec_from_file_location('xfeat_onnx', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def random_h(rng, w, h):
    """A plausible view change: rotation, scale, a little perspective, shift."""
    a = np.deg2rad(rng.uniform(-15, 15))
    s = rng.uniform(0.85, 1.15)
    c, sn = np.cos(a) * s, np.sin(a) * s
    cx, cy = w / 2, h / 2
    A = np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]])
    T0 = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1]])
    T1 = np.array([[1, 0, cx + rng.uniform(-0.08, 0.08) * w],
                   [0, 1, cy + rng.uniform(-0.08, 0.08) * h], [0, 0, 1]])
    P = np.eye(3)
    P[2, 0], P[2, 1] = rng.uniform(-4e-4, 4e-4), rng.uniform(-4e-4, 4e-4)
    return T1 @ P @ A @ T0


def degrade(img, rng):
    g = img.astype(np.float32) * rng.uniform(0.8, 1.2) + rng.uniform(-15, 15)
    g = cv2.GaussianBlur(g, (0, 0), rng.uniform(0.3, 1.0))
    g += rng.normal(0, 3.0, g.shape)
    return np.clip(g, 0, 255).astype(np.uint8)


def score_pair(match, k0, d0, k1, d1, H):
    """(matches, correct) for one pair under the true homography."""
    if len(d0) < 2 or len(d1) < 2:
        return 0, 0
    i0, i1 = match(d0, d1)
    if len(i0) == 0:
        return 0, 0
    p = cv2.perspectiveTransform(k0[i0].reshape(-1, 1, 2).astype(np.float64), H)
    err = np.linalg.norm(p.reshape(-1, 2) - k1[i1], axis=1)
    return len(i0), int((err < PX).sum())


def frames(path, n, w, h):
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
    out = []
    for i in np.linspace(total * 0.05, total * 0.95, n).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, f = cap.read()
        if ok:
            out.append(cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (w, h)))
    cap.release()
    return out


class Heads:
    """The ONNX model's raw heads, so both rankings see the same forward pass."""

    def __init__(self, model):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = 2
        self.s = ort.InferenceSession(model, so, providers=['CPUExecutionProvider'])
        self.inp = self.s.get_inputs()[0]
        self.h, self.w = self.inp.shape[2], self.inp.shape[3]

    def __call__(self, gray):
        x = gray.astype(np.float32)[None, None] / 255.0
        return self.s.run(None, {self.inp.name: x})


def detect(xf, heads, rel_on, top_k=1024, thresh=0.05):
    feats, kl, rel = heads
    return xf.keypoints_and_descriptors(feats, kl, thresh, top_k,
                                        rel=rel if rel_on else None)


def nms_count(xf, heads, thresh=0.05):
    return len(xf._nms(xf._pixel_shuffle_scores(heads[1][0]), thresh))


def _upstream(ref_dir):
    import torch
    if ref_dir not in sys.path:
        sys.path.insert(0, ref_dir)
    from modules.xfeat import XFeat
    ref = XFeat(weights=os.path.join(ref_dir, 'weights', 'xfeat.pt'), top_k=1024,
                detection_threshold=0.05)
    ref.dev = torch.device('cpu')
    ref.net = ref.net.cpu().eval()
    torch.set_num_threads(2)
    return ref


def fidelity(xf, ref, gray_frames):
    """Our selection against upstream's own code, on the SAME network output.

    Upstream's detectAndCompute resizes to a multiple of 32 first (240 -> 224),
    so comparing its output with ours compares two grids, not two rankings.
    Here upstream's own functions (get_kpts_heatmap, NMS, InterpolateSparse2d)
    run on its net's heads at our size, and ours run on the same heads."""
    import torch
    from modules.interpolator import InterpolateSparse2d
    rows = []
    for g in gray_frames:
        x = torch.tensor(g[None, None] / 255.0, dtype=torch.float32)
        with torch.no_grad():
            M1, K1, H1 = ref.net(x)
            K1h = ref.get_kpts_heatmap(K1)
            mk = ref.NMS(K1h, threshold=0.05, kernel_size=5)
            _H, _W = x.shape[-2:]
            sc = (InterpolateSparse2d('nearest')(K1h, mk, _H, _W)
                  * InterpolateSparse2d('bilinear')(H1, mk, _H, _W)).squeeze(-1)
            sc[torch.all(mk == 0, dim=-1)] = -1
            idx = torch.argsort(-sc)[0, :1024]
            rk = {tuple(p) for p in mk[0, idx].numpy().astype(int)}
        heads = (M1.numpy(), K1.numpy(), H1.numpy())
        share = []
        for on in (True, False):
            k, _ = detect(xf, heads, on)
            share.append(len({tuple(p) for p in k.astype(int)} & rk) / max(1, len(rk)))
        rows.append((nms_count(xf, heads), *share))
    return rows


def dense(heads):
    """`anchor_semi_dense` as the Hailo backend ships it: every cell centre."""
    f = heads[0][0]
    c, fh, fw = f.shape
    d = f.reshape(c, -1).T.astype(np.float32)
    d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-8)
    ys, xs = np.mgrid[0:fh, 0:fw]
    k = np.stack([(xs.ravel() + 0.5) * 8.0, (ys.ravel() + 0.5) * 8.0], 1)
    return k.astype(np.float32), d


def upstream_pairs(ref, g0, g1, star):
    """Upstream's published methods end to end: keypoints in input pixels."""
    import torch
    t0 = torch.tensor(g0[None, None] / 255.0, dtype=torch.float32)
    t1 = torch.tensor(g1[None, None] / 255.0, dtype=torch.float32)
    with torch.no_grad():
        if star == 'glue':
            a = ref.detectAndCompute(t0)[0]
            b = ref.detectAndCompute(t1)[0]
            a['image_size'] = b['image_size'] = (g0.shape[1], g0.shape[0])
            m0, m1, _ = ref.match_lighterglue(a, b)
            return np.asarray(m0), np.asarray(m1)
        if star == 'single':
            # match_xfeat_star with multiscale=False: one forward pass, as a
            # fixed-size HEF allows.
            o0 = ref.detectAndComputeDense(t0, top_k=1024, multiscale=False)
            o1 = ref.detectAndComputeDense(t1, top_k=1024, multiscale=False)
            idx = ref.batch_match(o0['descriptors'], o1['descriptors'])
            m = ref.refine_matches(o0, o1, matches=idx, batch_idx=0).numpy()
            return m[:, :2], m[:, 2:]
        if star:
            p0, p1 = ref.match_xfeat_star(t0, t1, top_k=1024)   # B == 1: two arrays
            return np.asarray(p0), np.asarray(p1)
        a = ref.detectAndCompute(t0)[0]
        b = ref.detectAndCompute(t1)[0]
        i0, i1 = ref.match(a['descriptors'], b['descriptors'], 0.82)
        return a['keypoints'][i0].numpy(), b['keypoints'][i1].numpy()


MIN_INLIERS, RANSAC_PX, ROI = 15, 3.0, 0.4      # the anchor's bar and estimator


def homography_ok(p0, p1, H, w, h, roi=False):
    """Does the anchor's own estimator recover the true view change?

    MAGSAC at 3 px from the matches, accepted at >= 15 inliers as
    `anchor.locate_features` does; then the region's corners mapped by the fit
    against the truth. With `roi`, only matches whose first point lies in a
    centred box 40 % of the frame on a side are used, as when the anchor holds
    a target-sized region."""
    x0, x1 = w * (1 - ROI) / 2, w * (1 + ROI) / 2
    y0, y1 = h * (1 - ROI) / 2, h * (1 + ROI) / 2
    if roi and len(p0):
        keep = (p0[:, 0] >= x0) & (p0[:, 0] <= x1) & (p0[:, 1] >= y0) & (p0[:, 1] <= y1)
        p0, p1 = p0[keep], p1[keep]
    if len(p0) < 8:
        return False
    Hf, mask = cv2.findHomography(p0.reshape(-1, 1, 2).astype(np.float32),
                                  p1.reshape(-1, 1, 2).astype(np.float32),
                                  cv2.USAC_MAGSAC, RANSAC_PX)
    if Hf is None or mask is None or int(mask.sum()) < MIN_INLIERS:
        return False
    c = (np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]) if roi
         else np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])).reshape(-1, 1, 2)
    e = np.linalg.norm(cv2.perspectiveTransform(c.astype(np.float64), Hf)
                       - cv2.perspectiveTransform(c.astype(np.float64), H), axis=2)
    return float(e.mean()) < 3.0


def score_points(p0, p1, H):
    if len(p0) == 0:
        return 0, 0
    q = cv2.perspectiveTransform(p0.reshape(-1, 1, 2).astype(np.float64), H)
    return len(p0), int((np.linalg.norm(q.reshape(-1, 2) - p1, axis=1) < PX).sum())


MODES = ('ours keypoint', 'ours reliability', 'ours dense', 'upstream sparse',
         'upstream XFeat*', 'upstream XFeat* single-scale', 'upstream LighterGlue', 'rel + norm-first bilinear', 'rel + norm-first bicubic',
         'rel + raw bicubic', 'gate 0.05', 'gate 0.1', 'gate 0.2', 'gate 0.3')


def resample(heads, k, how, w, h):
    """Descriptors at our keypoints, sampled the way upstream does
    (InterpolateSparse2d: (W-1) normalisation, align_corners=False, zeros
    outside). `how` is 'norm-first bilinear', 'norm-first bicubic' or
    'raw bicubic'; ours samples the raw map bilinearly with edge clamping."""
    import torch
    import torch.nn.functional as F
    from modules.interpolator import InterpolateSparse2d
    M = torch.from_numpy(np.ascontiguousarray(heads[0]))
    if how.startswith('norm-first'):
        M = F.normalize(M, dim=1)
    mode = how.split()[-1]
    with torch.no_grad():
        d = InterpolateSparse2d(mode)(M, torch.from_numpy(k[None]).float(), h, w)[0]
        d = F.normalize(d, dim=-1)
    return d.numpy().astype(np.float32)


def run(model, ref_dir, n_frames, seed, only=None):
    import time
    xf = _xfeat_module()
    net = Heads(model)
    ref = _upstream(ref_dir) if ref_dir else None
    modes = [m for m in (only or MODES)
             if m in MODES or m.split()[0] in ('cos', 'topk', 'thresh')]
    modes = [m for m in modes if ref or not (m.startswith('upstream') or m.startswith('rel +'))]
    A = archive_root()
    rng = np.random.default_rng(seed)
    match = xf.XFeatONNX.match
    per_clip = {}
    cost = {m: 0.0 for m in modes}
    okf = {m: 0 for m in modes}
    okr = {m: 0 for m in modes}
    over = cnt = 0
    some = []
    for rel in CLIPS:
        fr = frames(f'{A}/{rel}', n_frames, net.w, net.h)
        some += fr[:4]
        acc = {m: [0, 0] for m in modes}
        for g in fr:
            H = random_h(rng, net.w, net.h)
            g1 = degrade(cv2.warpPerspective(g, H, (net.w, net.h)), rng)
            h0, h1 = net(g), net(g1)
            cnt += 1
            over += nms_count(xf, h0) > 1024
            for m in modes:
                t = time.perf_counter()
                if m.startswith('upstream'):
                    star = ('single' if m.endswith('single-scale') else
                            'glue' if m.endswith('LighterGlue') else m.endswith('*'))
                    p0, p1 = upstream_pairs(ref, g, g1, star=star)
                else:
                    if m.startswith('rel +'):
                        how = m[len('rel + '):]
                        k0, _ = detect(xf, h0, True)
                        k1, _ = detect(xf, h1, True)
                        d0 = resample(h0, k0, how, net.w, net.h)
                        d1 = resample(h1, k1, how, net.w, net.h)
                    elif m.split()[0] in ('cos', 'topk', 'thresh'):
                        # One inherited setting moved, the rest as shipped.
                        kind, val = m.split()
                        tk = int(val) if kind == 'topk' else 1024
                        th = float(val) if kind == 'thresh' else 0.05
                        k0, d0 = detect(xf, h0, True, top_k=tk, thresh=th)
                        k1, d1 = detect(xf, h1, True, top_k=tk, thresh=th)
                        if kind == 'cos':
                            if len(d0) >= 2 and len(d1) >= 2:
                                i0, i1 = match(d0, d1, float(val))
                            else:
                                i0 = i1 = np.zeros(0, int)
                            k0, k1 = k0[i0], k1[i1]
                            d0 = d1 = None
                    elif m.startswith('gate'):
                        # Reliability as a per-keypoint gate on top of the
                        # ranking: drop keypoints below the bar before matching.
                        tau = float(m.split()[1])
                        k0, d0 = detect(xf, h0, True)
                        k1, d1 = detect(xf, h1, True)
                        r0 = xf.reliability_at(h0[2], k0, net.w, net.h) >= tau
                        r1 = xf.reliability_at(h1[2], k1, net.w, net.h) >= tau
                        k0, d0, k1, d1 = k0[r0], d0[r0], k1[r1], d1[r1]
                    elif m == 'ours dense':
                        k0, d0 = dense(h0)
                        k1, d1 = dense(h1)
                    else:
                        on = m == 'ours reliability'
                        k0, d0 = detect(xf, h0, on)
                        k1, d1 = detect(xf, h1, on)
                    if d0 is None:                      # matched above
                        p0, p1 = k0, k1
                    else:
                        if len(d0) >= 2 and len(d1) >= 2:
                            i0, i1 = match(d0, d1)
                        else:
                            i0 = i1 = np.zeros(0, int)
                        p0, p1 = k0[i0], k1[i1]
                n, c = score_points(p0, p1, H)
                okf[m] += homography_ok(p0, p1, H, net.w, net.h)
                okr[m] += homography_ok(p0, p1, H, net.w, net.h, roi=True)
                cost[m] += time.perf_counter() - t
                acc[m][0] += n
                acc[m][1] += c
        per_clip[os.path.basename(rel)] = acc

    print(f'{cnt} pairs over {len(CLIPS)} clips; NMS left more than top_k on '
          f'{over} ({100.0 * over / max(1, cnt):.0f} %) of them')
    print(f'{"clip":24s} {"mode":18s} {"matches":>8s} {"correct":>8s} {"precision":>9s}')
    tot = {m: [0, 0] for m in modes}
    prec = {}
    for name, acc in per_clip.items():
        for m in modes:
            n, c = acc[m]
            tot[m][0] += n
            tot[m][1] += c
            prec[(name, m)] = c / max(1, n)
            print(f'{name:24s} {m:18s} {n:8d} {c:8d} {100 * prec[(name, m)]:8.1f}%')
    print('pooled, per pair:')
    for m in modes:
        print(f'  {m:18s} correct {tot[m][1] / cnt:7.1f}  precision '
              f'{100 * tot[m][1] / max(1, tot[m][0]):5.1f}%  '
              f'homography ok {100 * okf[m] / cnt:5.1f}% (target box '
              f'{100 * okr[m] / cnt:5.1f}%)  {1000 * cost[m] / cnt:6.1f} ms/pair')
    print('  (ms: dev box CPU; upstream includes its forward pass, ours does not)')
    if 'ours keypoint' not in tot or 'ours reliability' not in tot:
        return 0
    gain = 100.0 * (tot['ours reliability'][1] - tot['ours keypoint'][1]) \
        / max(1, tot['ours keypoint'][1])
    drop = max(100 * (prec[(c, 'ours keypoint')] - prec[(c, 'ours reliability')])
               for c in per_clip)
    print(f'reliability vs keypoint: correct {gain:+.1f} %, worst per-clip '
          f'precision drop {drop:.1f} pt')
    print('VERDICT:', 'ADOPT reliability ranking' if gain >= 5.0 and drop <= 1.0
          else 'KEEP keypoint-only ranking (bar not met)')
    if ref:
        rows = fidelity(xf, ref, some)
        withr = [r[1] for r in rows]
        without = [r[2] for r in rows if r[0] > 1024]
        print(f'fidelity, same heads, upstream selection code: with reliability '
              f'min {100 * min(withr):.1f} % mean {100 * np.mean(withr):.1f} %; '
              f'keypoint-only on the {len(without)} frames past top_k: mean '
              f'{100 * np.mean(without) if without else float("nan"):.1f} %')
    return 0


FIXTURE = os.path.join(HERE, '..', '..', 'src', 'mongla_vision', 'test', 'data',
                       'xfeat_upstream_reference.npz')
FIX_SEED, FIX_H, FIX_W, FIX_TOPK = 7, 96, 128, 48


def fixture_heads(seed=FIX_SEED, h=FIX_H, w=FIX_W):
    """Seeded synthetic heads (the test regenerates them; only upstream's
    answer is stored)."""
    rng = np.random.default_rng(seed)
    feats = rng.standard_normal((1, 64, h // 8, w // 8)).astype(np.float32)
    kl = (3.0 * rng.standard_normal((1, 65, h // 8, w // 8))).astype(np.float32)
    rel = rng.uniform(0.05, 1.0, (1, 1, h // 8, w // 8)).astype(np.float32)
    # Strong keypoints on the last column and row, which upstream drops: the
    # sub-cell channel is dy * 8 + dx, so dx = 7 is the last pixel column.
    kl[0, 3 * 8 + 7, ::2, -1] = 12.0
    kl[0, 7 * 8 + 3, -1, ::2] = 12.0
    return feats, kl, rel


def write_fixture(ref_dir):
    """Upstream's detectAndCompute on the fixture heads, its net replaced by
    them, so the answer is upstream's own selection and sampling code."""
    import torch
    ref = _upstream(ref_dir)
    heads = fixture_heads()
    class Fixed(torch.nn.Module):
        def forward(self, _x):
            return tuple(torch.from_numpy(a) for a in heads)
    ref.net = Fixed()
    with torch.no_grad():
        o = ref.detectAndCompute(torch.zeros(1, 1, FIX_H, FIX_W), top_k=FIX_TOPK)[0]
    os.makedirs(os.path.dirname(FIXTURE), exist_ok=True)
    np.savez_compressed(FIXTURE, keypoints=o['keypoints'].numpy().astype(np.int16),
                        descriptors=o['descriptors'].numpy().astype(np.float32))
    print(f'wrote {FIXTURE}: {len(o["keypoints"])} keypoints')
    return 0


def self_test():
    """The scorer, on cases whose answer is known before it runs."""
    rng = np.random.default_rng(0)
    w, h = 320, 240
    k0 = rng.uniform(10, [w - 10, h - 10], (300, 2)).astype(np.float32)
    d0 = rng.normal(size=(300, 64)).astype(np.float32)
    d0 /= np.linalg.norm(d0, axis=1, keepdims=True)
    H = random_h(rng, w, h)
    k1 = cv2.perspectiveTransform(k0.reshape(-1, 1, 2).astype(np.float64), H).reshape(-1, 2)
    xf = _xfeat_module()
    m, c = score_pair(xf.XFeatONNX.match, k0, d0, k1, d0.copy(), H)
    assert m == 300 and c == 300, (m, c)                    # perfect features
    perm = rng.permutation(300)
    m, c = score_pair(xf.XFeatONNX.match, k0, d0, k1[perm], d0.copy(), H)
    assert m == 300 and c <= 3, (m, c)                      # right descriptors, wrong places
    k1n = k1 + rng.normal(0, 1.0, k1.shape)
    m, c = score_pair(xf.XFeatONNX.match, k0, d0, k1n, d0.copy(), H)
    assert c >= 0.95 * 300, c                               # 1 px noise is inside 3 px
    # The reliability sample: a ramp map, read at known points, must give the
    # value upstream's grid_sample gives (align_corners=False, (W-1) norm).
    rel = np.arange(30 * 40, dtype=np.float32).reshape(1, 1, 30, 40)
    pts = np.array([[0, 0], [319, 239], [160, 120], [7, 3]], np.float32)
    got = xf.reliability_at(rel, pts, w, h)
    u = pts[:, 0] * 40 / (w - 1) - 0.5
    v = pts[:, 1] * 30 / (h - 1) - 0.5
    want = []
    for uu, vv in zip(u, v):
        x0, y0 = int(np.floor(uu)), int(np.floor(vv))
        acc = 0.0
        for dx in (0, 1):
            for dy in (0, 1):
                xi, yi = x0 + dx, y0 + dy
                wgt = (1 - abs(uu - xi)) * (1 - abs(vv - yi))
                if 0 <= xi < 40 and 0 <= yi < 30:
                    acc += wgt * rel[0, 0, yi, xi]
        want.append(acc)
    assert np.allclose(got, want, atol=1e-4), (got, want)
    print('self-test ok: the scorer counts perfect, shuffled and noisy matches '
          'as it must, and the reliability sample is upstream\'s')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--write-fixture', action='store_true',
                    help="record upstream's answer for test_xfeat_is_upstream.py (needs --reference)")
    ap.add_argument('--model', help='xfeat_<w>x<h>.onnx')
    ap.add_argument('--reference', default='', help='checkout of verlab/accelerated_features')
    ap.add_argument('--frames', type=int, default=40, help='per clip')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--modes', default='', help='comma-separated subset of MODES')
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    if a.write_fixture:
        return write_fixture(a.reference)
    if not a.model:
        ap.error('--model is required')
    only = [m.strip() for m in a.modes.split(',') if m.strip()] or None
    return run(a.model, a.reference, a.frames, a.seed, only)


if __name__ == '__main__':
    sys.exit(main())
