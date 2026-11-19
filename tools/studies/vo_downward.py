"""Should XFeat feed the downward camera's motion fit, or does LK already do it?

The flow node already fits ONE similarity transform per interval
(`flow_math.solve_planar_motion`): translation of the optical axis, image yaw
rate and scale rate, from LK tracks. An XFeat visual-odometry rung can only
earn its chip time by giving that SAME solver better correspondences. So both
trackers feed the shipped solver here, and nothing else differs.

Truth is exact: a real downward archive frame (three clips, caustics included)
moved by a known similarity per step, each frame independently degraded
(blur, noise, gain) so neither tracker sees a clean copy. Two regimes:
  small  1-3 px, <= 0.5 deg, <= 0.3 % scale a step   (180 fps, slow vehicle)
  large  8-30 px, 2-8 deg, <= 2 % scale a step       (30 fps, or a fast turn)
Then real consecutive frames from the same clips, where only availability
(does the fit succeed, and how well one motion explains the points) can be
read, because there is no truth.

Bar, written before the run: XFeat earns a VO rung only if, in the large
regime, it refuses at least 20 points fewer intervals than LK or its yaw error
is at most 0.7x LK's, with translation error no worse; and in the small regime
it is within 10 % of LK on both errors. Otherwise LK stays the tracker.

    python3 tools/studies/vo_downward.py --model xfeat_320x240.onnx
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, '..', '..')
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.dirname(HERE))
sys.path.insert(2, os.path.join(REPO, 'src', 'mongla_vision'))
import xfeat_reliability as xr  # noqa: E402
from archive_root import archive_root  # noqa: E402
from mongla_vision.flow.flow_constants import _FEATURE_PARAMS, _LK_PARAMS  # noqa: E402
from mongla_vision.flow.flow_math import solve_planar_motion  # noqa: E402

DOWNWARD = ('robosub/clips/octagon/octagon_1.mp4', 'final_run/octagon_Bottom.mkv',
            'final_run/bin.mkv')
REGIMES = {'small': (1.0, 3.0, 0.5, 0.003), 'large': (8.0, 30.0, 8.0, 0.02)}


def similarity(dx, dy, th_deg, s, cx, cy):
    """2x3 similarity about (cx, cy): rotate and scale about the centre, then
    translate by (dx, dy). Its motion of the centre is exactly (dx, dy)."""
    t = math.radians(th_deg)
    a, b = s * math.cos(t), s * math.sin(t)
    R = np.array([[a, -b], [b, a]])
    c = np.array([cx, cy])
    return np.hstack([R, (c - R @ c + [dx, dy])[:, None]])


def compose(A, B):
    """A after B, both 2x3."""
    A3, B3 = np.vstack([A, [0, 0, 1]]), np.vstack([B, [0, 0, 1]])
    return (A3 @ B3)[:2]


def lk(prev, nxt):
    p = cv2.goodFeaturesToTrack(prev, mask=None, **_FEATURE_PARAMS)
    if p is None:
        return None, None, None
    q, st, _ = cv2.calcOpticalFlowPyrLK(prev, nxt, p, None, **_LK_PARAMS)
    return p, q, st


def run(a) -> int:
    xf = xr._xfeat_module()
    net = xr.Heads(a.model)
    w, h = net.w, net.h
    cx, cy = w / 2.0, h / 2.0
    rng = np.random.default_rng(a.seed)
    A = archive_root()

    def xfeat(prev, nxt, cache):
        k0, d0 = cache.get(id(prev)) or xr.detect(xf, net(prev), True)
        k1, d1 = xr.detect(xf, net(nxt), True)
        cache.clear()
        cache[id(nxt)] = (k1, d1)
        if len(d0) < 2 or len(d1) < 2:
            return None, None, None
        i0, i1 = xf.XFeatONNX.match(d0, d1)
        return k0[i0], k1[i1], np.ones(len(i0), np.uint8)

    trackers = {'LK (shipped)': lambda p, n, c: lk(p, n), 'XFeat': xfeat}
    dt = 1.0                      # yaw_rate = theta / dt, so rates read as per-step angles
    out = {}
    for regime, (lo, hi, th_max, s_max) in REGIMES.items():
        for rel in DOWNWARD:
            for base in xr.frames(f'{A}/{rel}', a.bases, w, h):
                M = similarity(0, 0, 0, 1, cx, cy)
                frames, steps = [], []
                for i in range(a.steps + 1):
                    if i:
                        mag, ang = rng.uniform(lo, hi), rng.uniform(0, 2 * math.pi)
                        S = (mag * math.cos(ang), mag * math.sin(ang),
                             rng.uniform(-th_max, th_max), 1 + rng.uniform(-s_max, s_max))
                        M = compose(similarity(*S, cx, cy), M)
                        steps.append(S)
                    img = cv2.warpAffine(base, M, (w, h), flags=cv2.INTER_LINEAR,
                                         borderMode=cv2.BORDER_REFLECT)
                    frames.append(xr.degrade(img, rng))
                for name, tr in trackers.items():
                    st = out.setdefault((regime, name), {'t': [], 'y': [], 'ref': 0, 'n': 0,
                                                         'ms': 0.0, 'drift': []})
                    cache = {}
                    sx = sy = sth = 0.0
                    for i, S in enumerate(steps):
                        t0 = time.perf_counter()
                        p0, p1, ok = tr(frames[i], frames[i + 1], cache)
                        pm = (solve_planar_motion(p0, p1, ok, dt, cx=cx, cy=cy)
                              if p0 is not None else None)
                        st['ms'] += time.perf_counter() - t0
                        st['n'] += 1
                        if pm is None or not pm.ok:
                            st['ref'] += 1
                            continue
                        st['t'].append(math.hypot(pm.dx_px - S[0], pm.dy_px - S[1]))
                        st['y'].append(abs(math.degrees(pm.yaw_rate) - S[2]))
                        sx += pm.dx_px - S[0]
                        sy += pm.dy_px - S[1]
                        sth += math.degrees(pm.yaw_rate) - S[2]
                    st['drift'].append((math.hypot(sx, sy), abs(sth)))
    print(f'{a.bases} base frames x {len(DOWNWARD)} clips x {a.steps} steps a regime; '
          f'{w}x{h}; same solver for both trackers')
    res = {}
    for (regime, name), st in out.items():
        t, y = np.array(st['t']), np.array(st['y'])
        dr = np.array(st['drift'])
        res[(regime, name)] = (100 * st['ref'] / st['n'], np.median(t), np.median(y))
        print(f'{regime:5s} {name:13s} refused {100 * st["ref"] / st["n"]:5.1f}%  '
              f'translation err p50 {np.median(t):5.2f} p90 {np.percentile(t, 90):5.2f} px  '
              f'yaw err p50 {np.median(y):5.3f} p90 {np.percentile(y, 90):5.3f} deg  '
              f'end drift {np.median(dr[:, 0]):5.1f} px / {np.median(dr[:, 1]):5.2f} deg  '
              f'{1000 * st["ms"] / st["n"]:5.1f} ms/step')

    # Real consecutive frames: availability only.
    print('real consecutive frames (no truth: fit success, inliers, residual):')
    for gap in (1, 6):
        for name, tr in trackers.items():
            ok = n = 0
            inl, res_px = [], []
            for rel in DOWNWARD:
                cap = cv2.VideoCapture(f'{A}/{rel}')
                total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                for s in np.linspace(total * 0.05, total * 0.9, a.bases * 2).astype(int):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, int(s))
                    fr = []
                    for _ in range(gap + 1):
                        okr, f = cap.read()
                        if okr:
                            fr.append(cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (w, h)))
                    if len(fr) < gap + 1:
                        continue
                    p0, p1, okk = tr(fr[0], fr[gap], {})
                    pm = solve_planar_motion(p0, p1, okk, 1.0, cx=cx, cy=cy) if p0 is not None else None
                    n += 1
                    if pm is not None and pm.ok:
                        ok += 1
                        inl.append(pm.n_inliers)
                        res_px.append(pm.residual_px)
                cap.release()
            print(f'  gap {gap} frame(s) {name:13s} fit {100 * ok / max(1, n):5.1f}% of {n}  '
                  f'inliers p50 {np.median(inl) if inl else 0:5.0f}  '
                  f'residual p50 {np.median(res_px) if res_px else float("nan"):4.2f} px')

    L, X = res[('large', 'LK (shipped)')], res[('large', 'XFeat')]
    Ls, Xs = res[('small', 'LK (shipped)')], res[('small', 'XFeat')]
    large_ok = ((L[0] - X[0] >= 20 or X[2] <= 0.7 * L[2]) and X[1] <= L[1])
    small_ok = Xs[1] <= 1.1 * Ls[1] and Xs[2] <= 1.1 * Ls[2]
    print('VERDICT:', 'XFeat earns a VO rung' if large_ok and small_ok else
          'LK stays the tracker' + ('' if small_ok else ' (XFeat worse at small steps)')
          + ('' if large_ok else ' (XFeat not better enough at large steps)'))
    return 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', required=True)
    ap.add_argument('--bases', type=int, default=6, help='base frames per clip')
    ap.add_argument('--steps', type=int, default=20)
    ap.add_argument('--seed', type=int, default=5)
    sys.exit(run(ap.parse_args()))
