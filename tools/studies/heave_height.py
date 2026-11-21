#!/usr/bin/env python3
"""Height above the floor from a depth change: the image's magnification over
the barometer's displacement. Bead dg7; the bar there was written first.

Why. Flow needs the height above the floor. The vehicle measures only its own
depth, so until now height was the floor's depth (typed, or read off a survey
at a position localization must first fix) minus the hull's. On a sloping
floor before any prop fix, that is wrong by up to the floor's spread.

The physics. Sink by dd and the floor's image magnifies by
s = h_before / h_after, so h_after = dd / (s - 1). The barometer gives dd to
millimetres; the tracked points give s. The focal length, the port's
refraction and the floor's depth all cancel: no calibration, map or position
enters. Small drones use the same idea with a commanded climb rate (de Croon,
Bioinspir. Biomim. 2016; Chirarattananon, 2018); a barometer measuring the
displacement is the stronger form.

    python3 tools/studies/heave_height.py --self-test      # bar set 1: two-point, missed
    python3 tools/studies/heave_height.py --descent        # bar set 2: fit over a descent, passes
    python3 tools/studies/heave_height.py --aqualoc ROOT   # real harbour water: FALSIFIED

Outcome (measured-bars section 100): the descent fit passes every synthetic bar
and fails real water, a 36 % median error on the AQUALOC harbour against a 15 %
falsifier. Nothing on the vehicle uses it; it stays here to be rerun on pool
footage, where the floor is flat and tiled.

Synthetic: a textured floor rendered through the downward camera at known
heights, a hull sinking dd with barometer noise, lateral drift and yaw, frames
tracked by the shipped chain (detect_corners, pyramidal LK, forward-backward
check) and chained frame to frame, as the vehicle would.

Real: AQUALOC harbour, a seabed-facing camera with a pressure sensor. Truth for
a window is the COLMAP velocity over the same window divided by flow's
velocity at 1 m, the per-window form of aqualoc_replay.fit_height. COLMAP was
rescaled with the same pressure readings, so it shares the barometer but not
the image scale: an agreement is evidence, a disagreement falsifies.
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys

from typing import Optional, Sequence, Tuple

import numpy as np

_REPO = pathlib.Path(__file__).resolve().parents[2]
for _p in ('src/mongla_localization', 'src/mongla_vision', 'tools/studies'):
    sys.path.insert(0, str(_REPO / _p))

from mongla_vision.flow.flow_features import (  # noqa: E402
    detect_corners, forward_backward_error)
from mongla_vision.flow.flow_math import floor_depth_at  # noqa: E402

from aqualoc_replay import FB_MAX_PX, LK  # noqa: E402  (one copy of the tracker settings)

W, H, FX = 640, 360, 1027.87 * 0.5 * 1.333   # downward camera, 640 wide, in water


# ── synthetic floor ──────────────────────────────────────────────────────────

# The estimator lives here, with its only caller: it failed its real-water
# falsifier (bead dg7), so nothing on the vehicle uses it.
HEAVE_MIN_M = 0.08


def image_scale(p0: np.ndarray, p1: np.ndarray, *, iters: int = 200,
                inlier_px: float = 1.5, rng=None) -> Optional[Tuple[float, float, int]]:
    """Magnification s of p1 against p0 (Nx2, undistorted px), its sigma, inliers.

    A similarity fit (scale, rotation, shift) by RANSAC over point pairs, then
    least squares on the inliers. The scale is the ratio of distances between
    tracked points, so the focal length, the port's refraction and the
    principal point all cancel: nothing calibrated enters it. The sigma comes
    from the inlier residuals, propagated through the fit.
    """
    p0 = np.asarray(p0, float).reshape(-1, 2)
    p1 = np.asarray(p1, float).reshape(-1, 2)
    n = len(p0)
    if n < 8 or len(p1) != n:
        return None
    rng = rng if rng is not None else np.random.default_rng(0)
    z0 = p0[:, 0] + 1j * p0[:, 1]
    z1 = p1[:, 0] + 1j * p1[:, 1]

    def fit(idx):
        a0, a1 = z0[idx], z1[idx]
        m0, m1 = a0.mean(), a1.mean()
        d0, d1 = a0 - m0, a1 - m1
        den = np.vdot(d0, d0).real
        if den <= 1e-9:
            return None
        a = np.vdot(d0, d1) / den           # z1 = a z0 + b, |a| = scale
        return a, m1 - a * m0

    best = None
    for _ in range(iters):
        m = fit(rng.choice(n, 2, replace=False))
        if m is None:
            continue
        inl = np.abs(m[0] * z0 + m[1] - z1) < inlier_px
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 8:
        return None
    m = fit(np.flatnonzero(best))
    if m is None:
        return None
    a, b = m
    r = z1[best] - (a * z0[best] + b)
    k = int(best.sum())
    s2 = float(np.vdot(r, r).real) / max(1, 2 * k - 4)   # px^2 per coordinate
    d0 = z0[best] - z0[best].mean()
    var_a = s2 / float(np.vdot(d0, d0).real)              # var of a's real part
    return float(abs(a)), math.sqrt(var_a), k


def height_from_descent(deeper_m: Sequence[float], scales: Sequence[float],
                        depth_sigma_m: float, scale_sigma_end: float = 0.0
                        ) -> Optional[Tuple[float, float]]:
    """Height above the floor at the END of a depth change, from every sample of it.

    `deeper_m[i]` is how far the hull has sunk since the first sample (metres,
    from the barometer), `scales[i]` the floor's magnification against the
    first frame at the same instant. A flat floor gives
    deeper = h0 * (1 - 1/s) exactly, so a least-squares line through all the
    pairs has slope h0 (the height at the start), and the height at the end is
    h0 minus the last fitted sink. Fitting every sample averages the
    barometer's noise (16.4 mm per sample, measured-bars "Bar30 noise"), which
    a two-point difference cannot: one 0.15 m heave carries 11 % of noise, a
    0.5 m descent fitted over its samples well under 1 %.

    `scale_sigma_end` is the image fit's sigma on the last frame. Every later
    frame is tracked from the first, so an error in the chain is common to the
    whole descent: no refit over its samples can see it, and it enters the
    height as h_end^2 / sink.

    None when the change is too small (HEAVE_MIN_M) or the fit cannot see a slope.
    """
    d = np.asarray(deeper_m, float)
    s = np.asarray(scales, float)
    ok = np.isfinite(d) & np.isfinite(s) & (s > 0)
    d, s = d[ok], s[ok]
    if len(d) < 5:
        return None
    x = 1.0 - 1.0 / s
    if abs(x[-1]) < 1e-6 or np.ptp(d) < HEAVE_MIN_M:
        return None
    A = np.column_stack([x, np.ones_like(x)])
    (h0, c), *_ = np.linalg.lstsq(A, d, rcond=None)
    if not h0 > 0.0:
        return None
    r = d - A @ np.array([h0, c])
    s2 = max(float(r @ r) / max(1, len(d) - 2), depth_sigma_m ** 2 * 0.25)
    cov = s2 * np.linalg.inv(A.T @ A)
    # h_end = h0 * (1 - x_end) = h0 / s_end
    h_end = h0 * (1.0 - x[-1])
    sig = math.sqrt(max(cov[0, 0], 0.0)) * (1.0 - x[-1])
    # The least-squares sigma assumes independent residuals; a tracked chain's
    # scale errors are correlated frame to frame, and that sigma read 5.4x
    # small at p99 (bead dg7). A block jackknife over contiguous fifths sees
    # the correlation: each refit leaves out one stretch of the descent.
    k = 5
    if len(d) >= 4 * k:
        edges = np.linspace(0, len(d), k + 1).astype(int)
        hs = []
        for i in range(k):
            keep = np.ones(len(d), bool)
            keep[edges[i]:edges[i + 1]] = False
            (hj, _), *_ = np.linalg.lstsq(A[keep], d[keep], rcond=None)
            hs.append(hj * (1.0 - x[-1]))
        hs = np.asarray(hs)
        jack = math.sqrt((k - 1) / k * float(np.sum((hs - hs.mean()) ** 2)))
        sig = max(sig, jack)
    sink = float(h0 - h_end)
    if scale_sigma_end > 0.0 and sink > 0.0:
        sig = math.hypot(sig, h_end * h_end / sink * scale_sigma_end)
    return float(h_end), float(sig)


def height_from_heave(scale: float, scale_sigma: float, deeper_m: float,
                      deeper_sigma_m: float = 0.005) -> Optional[Tuple[float, float]]:
    """Height above the floor AFTER a depth change, and its sigma, or None.

    The floor's image magnifies by s = h_before / h_after when the hull sinks
    by `deeper_m` = h_before - h_after (negative when it rises), so

        h_after = deeper_m / (s - 1)

    Both inputs are measured: the barometer gives `deeper_m` to millimetres,
    the image gives s. No floor depth, map or position enters, so it holds on
    any slope. None when the change is too small to carry the scale
    (HEAVE_MIN_M), or the image scale did not move beyond three sigma.
    """
    ds = scale - 1.0
    if abs(deeper_m) < HEAVE_MIN_M or abs(ds) < 3.0 * scale_sigma or ds * deeper_m <= 0:
        return None
    h = deeper_m / ds
    sig = math.hypot(deeper_sigma_m / abs(ds), abs(deeper_m) * scale_sigma / (ds * ds))
    return h, sig


def texture(rng, size=4096, metres=4.0):
    """A pool floor: multi-scale grain, so corners exist at every height.

    1024 px per metre, so even at 0.5 m an image pixel covers more than one
    texture pixel. At 256 px/m (the first version) a low hull saw an upsampled
    blur and the study measured the simulator, not the method (bead dg7).
    """
    import cv2
    img = np.zeros((size, size), np.float32)
    for octave, amp in ((8, 1.0), (32, 0.6), (128, 0.4), (512, 0.25), (2048, 0.15)):
        n = rng.standard_normal((octave, octave)).astype(np.float32)
        img += amp * cv2.resize(n, (size, size), interpolation=cv2.INTER_CUBIC)
    img = (img - img.min()) / (img.max() - img.min()) * 200 + 25
    return img.astype(np.uint8), size / metres


def render(tex, px_per_m, h, x, y, yaw, rng, noise=2.0):
    """The floor seen from height h, camera over (x, y), yawed; pinhole, flat floor."""
    import cv2
    c, s = math.cos(yaw), math.sin(yaw)
    k = h / FX * px_per_m                    # texture px per image px
    cx_t = tex.shape[1] / 2 + x * px_per_m
    cy_t = tex.shape[0] / 2 + y * px_per_m
    # image (u, v) -> texture: centre, scale, rotate, shift
    M = np.array([[k * c, -k * s, cx_t - k * (c * W / 2 - s * H / 2)],
                  [k * s, k * c, cy_t - k * (s * W / 2 + c * H / 2)]], np.float32)
    img = cv2.warpAffine(tex, M, (W, H), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    img = img.astype(np.float32) + rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def track_chain(frames):
    """Points from the first frame followed through every frame (shipped chain)."""
    import cv2
    p0 = detect_corners(frames[0])
    if p0 is None or len(p0) < 8:
        return None, None
    start, cur = p0.copy(), p0.copy()
    for a, b in zip(frames[:-1], frames[1:]):
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(a, b, cur, None, **LK)
        fb = forward_backward_error(a, b, cur, nxt, LK)
        ok = st.reshape(-1).astype(bool)
        if fb is not None:
            ok &= np.asarray(fb).reshape(-1) < FB_MAX_PX
        start, cur = start[ok], nxt[ok]
        if len(cur) < 8:
            return None, None
    return start.reshape(-1, 2), cur.reshape(-1, 2)


def heave_trial(rng, tex, ppm, h0, dd, *, steps=12, drift=0.05, yaw_deg=2.0, baro_sigma=0.005):
    """One heave: returns (true h_after, estimate or None)."""
    x, y = rng.uniform(-1, 1, 2)
    yaw0 = rng.uniform(-math.pi, math.pi)
    frames = []
    for i in range(steps + 1):
        f = i / steps
        frames.append(render(tex, ppm, h0 - dd * f, x + drift * f, y + 0.5 * drift * f,
                             yaw0 + math.radians(yaw_deg) * f, rng))
    p0, p1 = track_chain(frames)
    if p0 is None:
        return h0 - dd, None
    sc = image_scale(p0, p1, rng=rng)
    if sc is None:
        return h0 - dd, None
    measured_dd = dd + rng.normal(0, baro_sigma) - rng.normal(0, baro_sigma)
    return h0 - dd, height_from_heave(sc[0], sc[1], measured_dd, baro_sigma * math.sqrt(2))


BARO_SIGMA = 0.0164      # m per sample, measured-bars "Bar30 noise"
BARO_HZ, CAM_HZ, SINK_MPS = 22.0, 30.0, 0.2


def track_scales(frames, undistort=None):
    """Magnification of every frame against the first, along one tracked chain.

    `undistort` maps Nx2 raw pixels to undistorted ones (a real lens); the
    synthetic camera is a pinhole and passes None."""
    import cv2
    p0 = detect_corners(frames[0])
    if p0 is None or len(p0) < 8:
        return None
    start, cur = p0.copy(), p0.copy()
    out = [1.0]
    track_scales.last_sigma = 0.0
    for a, b in zip(frames[:-1], frames[1:]):
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(a, b, cur, None, **LK)
        fb = forward_backward_error(a, b, cur, nxt, LK)
        ok = st.reshape(-1).astype(bool)
        if fb is not None:
            ok &= np.asarray(fb).reshape(-1) < FB_MAX_PX
        start, cur = start[ok], nxt[ok]
        if len(cur) < 8:
            return None
        a0, a1 = start.reshape(-1, 2), cur.reshape(-1, 2)
        if undistort is not None:
            a0, a1 = undistort(a0), undistort(a1)
        sc = image_scale(a0, a1)
        out.append(np.nan if sc is None else sc[0])
        if sc is not None:
            track_scales.last_sigma = sc[1]
    return np.array(out)


def descent_trial(rng, tex, ppm, h_end, dd, *, drift=0.05, yaw_deg=2.0):
    """A descent of dd ending at h_end, frames at CAM_HZ, barometer at BARO_HZ."""
    dur = dd / SINK_MPS
    n = max(6, int(round(dur * CAM_HZ)) + 1)
    t_cam = np.linspace(0, dur, n)
    x, y = rng.uniform(-1, 1, 2)
    yaw0 = rng.uniform(-math.pi, math.pi)
    h0 = h_end + dd
    frames = [render(tex, ppm, h0 - SINK_MPS * t, x + drift * t / dur, y,
                     yaw0 + math.radians(yaw_deg) * t / dur, rng) for t in t_cam]
    s = track_scales(frames)
    if s is None:
        return h_end, None
    # the barometer at its own rate, each sample with the measured noise, read at
    # the nearest camera frame: what the node would pair
    t_b = np.arange(0, dur + 1e-9, 1.0 / BARO_HZ)
    d_b = SINK_MPS * t_b + rng.normal(0, BARO_SIGMA, len(t_b))
    s_b = np.interp(t_b, t_cam, s)
    return h_end, height_from_descent(d_b - d_b[0], s_b, BARO_SIGMA,
                                      track_scales.last_sigma)


def descent_test(trials=200) -> int:
    rng = np.random.default_rng(2)
    tex, ppm = texture(rng)
    fails = []

    def check(ok, what):
        print(('ok   ' if ok else 'FAIL ') + what)
        if not ok:
            fails.append(what)

    for dd, label in ((0.5, '(a) 0.5 m descent'), (0.15, '(c) 0.15 m heave')):
        errs, z = [], []
        for _ in range(trials if dd == 0.5 else max(40, trials // 4)):
            truth, est = descent_trial(rng, tex, ppm, rng.uniform(0.5, 1.6), dd)
            if est is not None:
                errs.append(est[0] - truth)
                z.append(abs(est[0] - truth) / est[1])
        errs, z = np.array(errs), np.array(z)
        rms, worst = float(np.sqrt(np.mean(errs ** 2))), float(np.max(np.abs(errs)))
        print(f'     {label}: {len(errs)} answers, RMS {rms * 100:.2f} cm, worst {worst * 100:.2f} cm, '
              f'bias {errs.mean() * 100:+.2f} cm, |err|/sigma p99 {np.percentile(z, 99):.2f}')
        if dd == 0.5:
            check(rms < 0.03, f'(a) RMS {rms * 100:.2f} cm < 3 cm')
            check(worst < 0.08, f'(a) worst {worst * 100:.2f} cm < 8 cm')
        check(np.mean(z < 3) >= 0.99, f'(b) honest sigma for {label}')

    survey = [(0.0, 0.0, 1.28), (17.5, 0.0, 2.00)]
    mid = 0.5 * (1.28 + 2.00)
    worst_mid = worst_d = 0.0
    for xi in np.linspace(0, 17.5, 15):
        truth = floor_depth_at(survey, xi, 0.0)[0] - 0.5
        worst_mid = max(worst_mid, abs((mid - 0.5) - truth))
        t, est = descent_trial(rng, tex, ppm, truth, 0.5)
        if est is not None:
            worst_d = max(worst_d, abs(est[0] - t))
    print(f'     (d) slope: midrange worst {worst_mid:.3f} m, descent worst {worst_d:.3f} m')
    check(worst_d < 0.08, f'(d) descent worst {worst_d:.3f} m < 0.08 m')
    print('PASS' if not fails else f'FAIL ({len(fails)})')
    return 0 if not fails else 1


def self_test(trials=200) -> int:
    rng = np.random.default_rng(1)
    tex, ppm = texture(rng)
    fails = []

    def check(ok, what):
        print(('ok   ' if ok else 'FAIL ') + what)
        if not ok:
            fails.append(what)

    # Bars 1 and 2: accuracy and an honest sigma.
    errs, z = [], []
    for _ in range(trials):
        h0 = rng.uniform(0.5 + 0.15, 1.6)
        truth, est = heave_trial(rng, tex, ppm, h0, 0.15)
        if est is None:
            continue
        errs.append(est[0] - truth)
        z.append(abs(est[0] - truth) / est[1])
    errs = np.array(errs)
    got = len(errs)
    rms, worst = float(np.sqrt(np.mean(errs ** 2))), float(np.max(np.abs(errs)))
    print(f'     {got}/{trials} answered; RMS {rms * 100:.2f} cm, worst {worst * 100:.2f} cm, '
          f'bias {errs.mean() * 100:+.2f} cm, |err|/sigma p99 {np.percentile(z, 99):.2f}')
    check(got >= 0.95 * trials, f'answers on at least 95 % of heaves ({got}/{trials})')
    check(rms < 0.03, f'bar 1: RMS {rms * 100:.2f} cm < 3 cm')
    check(worst < 0.08, f'bar 1: worst {worst * 100:.2f} cm < 8 cm')
    check(np.mean(np.array(z) < 3) >= 0.99, 'bar 2: |err|/sigma < 3 for 99 % of answers')

    # Bar 3: too small a change is no answer, never a number.
    none = sum(heave_trial(rng, tex, ppm, 1.0, 0.03)[1] is None for _ in range(20))
    check(none == 20, f'bar 3: a 3 cm change gives no answer ({none}/20 refused)')
    check(height_from_heave(1.0, 0.001, HEAVE_MIN_M / 2) is None, 'bar 3: under HEAVE_MIN_M refused')
    check(height_from_heave(0.9, 0.001, 0.15) is None, 'bar 3: scale against the depth change refused')

    # Bar 4: on the slope, before any position fix.
    survey = [(0.0, 0.0, 1.28), (17.5, 0.0, 2.00)]
    mid = 0.5 * (1.28 + 2.00)
    worst_mid = worst_heave = 0.0
    for xi in np.linspace(0, 17.5, 15):
        floor = floor_depth_at(survey, xi, 0.0)[0]
        hull = 0.5                           # metres below the surface after the heave
        truth = floor - hull
        worst_mid = max(worst_mid, abs((mid - hull) - truth))
        t, est = heave_trial(rng, tex, ppm, truth + 0.15, 0.15)
        if est is not None:
            worst_heave = max(worst_heave, abs(est[0] - t))
    print(f'     slope: midrange worst {worst_mid:.3f} m, heave worst {worst_heave:.3f} m')
    check(worst_mid > 0.3, 'bar 4: the midrange hazard is real (> 0.30 m)')
    check(worst_heave < 0.08, f'bar 4: heave worst {worst_heave:.3f} m < 0.08 m')

    print('PASS' if not fails else f'FAIL ({len(fails)})')
    return 0 if not fails else 1


# ── AQUALOC ──────────────────────────────────────────────────────────────────
def aqualoc(root: pathlib.Path, seqs, min_dd=0.15, max_s=8.0) -> int:
    """Each depth change of at least min_dd within max_s: heave height vs COLMAP's."""
    import cv2
    from aqualoc_replay import (load_sequence, frame_flows, fit_height,  # noqa: E402
                                gt_track)
    rows = []
    for n in seqs:
        seq = load_sequence(root, n)
        t_img = seq['img_t']
        # AQUALOC's pressure log carries glitch samples (-266 m, +271 m): drop
        # anything off the harbour's range or more than 0.3 m from its
        # neighbours' median, then read depth at each frame.
        dt_, dv_ = seq['depth_t'], seq['depth']
        med = np.array([np.median(dv_[max(0, k - 3):k + 4]) for k in range(len(dv_))])
        good = (dv_ > -0.5) & (dv_ < 30.0) & (np.abs(dv_ - med) < 0.3)
        dep = np.interp(t_img, dt_[good], dv_[good])   # metres, positive down
        gt_t = gt_track(seq)[0]
        i = 0
        while i < len(t_img):
            j = np.searchsorted(t_img, t_img[i] + max_s)
            j = min(j, len(t_img) - 1)
            window = dep[i:j + 1]
            k = int(np.argmax(np.abs(window - dep[i])))
            dd = window[k] - dep[i]
            if abs(dd) < min_dd or k < 2:
                i += 5
                continue
            a, b = i, i + k
            if t_img[a] < gt_t[0] or t_img[b] > gt_t[-1]:
                i = b
                continue
            fx, fy, cx, cy = seq['K']
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
            D = seq['dist'].reshape(4, 1)
            frames = [cv2.imread(str(p), cv2.IMREAD_GRAYSCALE) for p in seq['img_path'][a:b + 1]]
            if any(f is None for f in frames):
                i = b
                continue
            def und(pts, K=K, D=D):
                return cv2.fisheye.undistortPoints(pts.reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)
            s = track_scales(frames, und)
            if s is None:
                i = b
                continue
            # depth at each frame, as sunk since the first; AQUALOC's pressure
            # rate and noise are its own, so its sigma is estimated by the fit
            sink = dep[a:b + 1] - dep[a]
            est = height_from_descent(sink, s, 0.01, track_scales.last_sigma)
            # truth: COLMAP velocity against flow at 1 m, in a window after the heave
            c, d = b, min(b + 60, len(t_img) - 1)
            truth = fit_height(seq, frame_flows(seq, c, d)) if d > c + 5 else float('nan')
            # The heave gives h at frame b; COLMAP's window starts there. Report both.
            rows.append((n, t_img[a], dd, None if est is None else est[0],
                         None if est is None else est[1], truth))
            print(f'seq {n:02d} t {t_img[a]:6.1f} s  dd {dd:+.2f} m  '
                  f'heave {"-" if est is None else f"{est[0]:.2f} +- {est[1]:.2f}"} m  '
                  f'colmap {truth:.2f} m', flush=True)
            i = b
    good = [(e, tr) for (_, _, _, e, _, tr) in rows if e is not None and math.isfinite(tr)]
    if not good:
        print('no heave with both an estimate and a COLMAP window')
        return 1
    rel = np.array([abs(e - tr) / tr for e, tr in good])
    print(f'{len(good)} heaves: median |err| {np.median(rel) * 100:.1f} %, '
          f'p90 {np.percentile(rel, 90) * 100:.1f} % (falsifier: median > 15 %)')
    return 0 if np.median(rel) <= 0.15 else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--self-test', action='store_true',
                    help='bar set 1: two-point heave, 5 mm barometer (missed; kept as the record)')
    ap.add_argument('--descent', action='store_true',
                    help='bar set 2: a fit over every sample of a descent, real Bar30 noise')
    ap.add_argument('--trials', type=int, default=200)
    ap.add_argument('--aqualoc', type=pathlib.Path, help='AQUALOC harbour folder')
    ap.add_argument('--seqs', type=int, nargs='+', default=[1, 2, 3, 4, 5, 6, 7])
    a = ap.parse_args(argv)
    if a.descent:
        return descent_test(a.trials)
    if a.aqualoc:
        return aqualoc(a.aqualoc, a.seqs)
    return self_test(a.trials)


if __name__ == '__main__':
    sys.exit(main())
