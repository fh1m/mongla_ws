#!/usr/bin/env python3
"""Does the tile grating read a REAL pool floor? Bead v8j; the bar there was written first.

Flow takes its height from the tile grating first (`flow/tile_grating.py`:
h = f * tile_m / period_px), so on a sloping pool it is the height source that
matters most, and until now it had only seen rendered grids.

There is no tape in the archive footage, so truth comes from a second witness
in the same frame: the dark lane line. Its width and the tile's size are both
fixed, so lane_px / period_px is the same at every height. A grating that
locks a harmonic reads x2 or x0.5, one fooled by tilt or caustics wanders; the
ratio shows both without knowing the height. The lane line also runs along
the tile axes, so its direction is an independent check of the grating's
heading.

    python3 tools/studies/tile_grating_real.py --self-test
    python3 tools/studies/tile_grating_real.py VIDEO [--every 0.5] [--ungraded]
    python3 tools/studies/tile_grating_real.py VIDEO --tape-m 0.80 --tile-m 0.15 [--water]

The last is the metric truth the lane cannot give: our calibrated downward
camera over tiles at a tape-measured height (lens to floor), any tiled floor
in air, the pool in water. A tile_m or focal error scales every reading the
same way and passes the lane ratio; only the tape catches it. Bar: median
height error < 3 %, p95 < 6 % (bead v8j).

--self-test renders a tiled floor with a lane line at known heights and
angles: it fixes the angle convention between the two witnesses on truth by
construction, before any real frame is read, and checks that the scorer
fails a grating forced onto a harmonic.
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'mongla_vision'))

from mongla_vision.flow.tile_grating import measure  # noqa: E402

DARK_V = 70          # lane line is near black; grout is mid grey
MIN_LANE_PX = 3000   # a lane in view covers more than this many pixels


def lane(img_bgr):
    """(width_px, direction_deg in [0, 180)) of the dark lane line, or None.

    The darkest large blob, opened to remove grout lines, its direction by PCA
    and its width as the median, over slices along that direction, of the
    p95 - p5 extent across it. A blob that is short along its own axis is an
    end or a shadow, not a stripe, and is refused.
    """
    import cv2
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 2] < DARK_V).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < MIN_LANE_PX:
        return None
    ys, xs = np.nonzero(lab == k)
    P = np.column_stack([xs, ys]).astype(float)
    P -= P.mean(0)
    w, v = np.linalg.eigh(P.T @ P)
    d = v[:, 1]                       # along the stripe
    nrm = v[:, 0]                     # across it
    a = P @ d
    c = P @ nrm
    if np.ptp(a) < 3 * max(1.0, np.ptp(c) / 2):
        return None                   # not elongated: an end, not a stripe
    edges = np.linspace(a.min(), a.max(), 21)
    widths = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        s = c[(a >= lo) & (a < hi)]
        if len(s) > 50:
            widths.append(np.percentile(s, 95) - np.percentile(s, 5))
    if len(widths) < 8:
        return None
    ang = math.degrees(math.atan2(d[1], d[0])) % 180.0
    return float(np.median(widths)) / 0.9, ang     # p5..p95 is 90 % of the width


def mod90_diff(a, b):
    d = (a - b) % 90.0
    return min(d, 90.0 - d)


def render(period_px, angle_deg, lane_px, W=640, H=480, rng=None):
    """A tiled floor seen from above: light tiles, grey grout, one dark lane."""
    import cv2
    rng = rng or np.random.default_rng(0)
    yy, xx = np.mgrid[0:H, 0:W].astype(float)
    t = math.radians(angle_deg)
    u = (xx - W / 2) * math.cos(t) + (yy - H / 2) * math.sin(t)
    v = -(xx - W / 2) * math.sin(t) + (yy - H / 2) * math.cos(t)
    gw = max(1.5, period_px * 0.06)
    grout = (np.mod(u, period_px) < gw) | (np.mod(v, period_px) < gw)
    g = np.where(grout, 120, 200).astype(float)
    g[np.abs(v - period_px * 0.5) < lane_px / 2] = 25
    g += rng.normal(0, 6, g.shape)
    g = cv2.GaussianBlur(np.clip(g, 0, 255).astype(np.uint8), (3, 3), 0)
    bgr = cv2.merge([g, g, g])
    return bgr


def score(rows, label, graded=True):
    """rows: (t, period_px, grating_angle, lane_px, lane_angle). Prints, returns fails."""
    r = np.array([lp / p for _, p, _, lp, _ in rows])
    da = np.array([mod90_diff(ga, la) for _, _, ga, _, la in rows])
    med = float(np.median(r))
    mad = float(np.median(np.abs(r - med)))
    off = np.abs(r / med - 1.0) > 0.15
    print(f'     {label}: n {len(r)}, lane/period median {med:.3f}, MAD {mad / med * 100:.2f} %, '
          f'off by >15 %: {off.mean() * 100:.1f} %, angle diff p50 {np.median(da):.2f} '
          f'p95 {np.percentile(da, 95):.2f} deg')
    if not graded:
        return []
    fails = []
    for ok, what in ((len(r) >= 30, f'(n) {len(r)} >= 30 frames with both witnesses'),
                     (mad / med < 0.05, f'(1) robust spread {mad / med * 100:.2f} % < 5 %'),
                     (off.mean() < 0.02, f'(2) {off.mean() * 100:.1f} % off by >15 %, < 2 %'),
                     (np.mean(da < 3.0) >= 0.95,
                      f'(3) angle within 3 deg for {np.mean(da < 3.0) * 100:.0f} % >= 95 %')):
        print(('ok   ' if ok else 'FAIL ') + what)
        if not ok:
            fails.append(what)
    return fails


def self_test() -> int:
    import cv2
    rng = np.random.default_rng(1)
    rows, harm = [], []
    for _ in range(40):
        period = rng.uniform(40, 90)            # heights spanning ~2x
        ang = rng.uniform(0, 90)
        img = render(period, ang, lane_px=period * 0.8, rng=rng)
        g = measure(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        ln = lane(img)
        if g is None or ln is None:
            continue
        rows.append((0, g.period_px, g.angle_deg, ln[0], ln[1]))
        # the injected defect: a grating that locked the second harmonic
        harm.append((0, g.period_px / (2 if len(harm) % 3 == 0 else 1), g.angle_deg, ln[0], ln[1]))
    print('synthetic, truth by construction:')
    good = score(rows, 'clean')
    print('injected: a third of the readings on the second harmonic (must FAIL):')
    bad = score(harm, 'harmonic')
    ok = not good and bool(bad)
    print('PASS' if ok else 'FAIL: the scorer cannot tell a good grating from a broken one')
    return 0 if ok else 1


def run(video, every_s, graded) -> int:
    import cv2
    cap = cv2.VideoCapture(str(video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, int(round(every_s * fps)))
    rows, seen, g_only, l_only = [], 0, 0, 0
    for i in range(0, n, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, img = cap.read()
        if not ok:
            break
        seen += 1
        g = measure(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        ln = lane(img)
        g_only += g is not None
        l_only += ln is not None
        if g is not None and ln is not None:
            rows.append((i / fps, g.period_px, g.angle_deg, ln[0], ln[1]))
    print(f'{video.name}: {seen} frames sampled, grating answered {g_only} '
          f'({g_only / max(1, seen) * 100:.0f} %), lane seen {l_only}, both {len(rows)}')
    if not rows:
        print('no frame with both witnesses')
        return 1
    p = np.array([r[1] for r in rows])
    print(f'     period px: p5 {np.percentile(p, 5):.1f}, median {np.median(p):.1f}, '
          f'p95 {np.percentile(p, 95):.1f} (a height range of {np.percentile(p, 95) / np.percentile(p, 5):.2f}x)')
    fails = score(rows, 'real', graded)
    if graded:
        print('PASS' if not fails else f'FAIL ({len(fails)})')
    return 0 if not fails else 1


CALIB = ROOT / 'src' / 'mongla_vision' / 'config' / 'calibration' / 'pi_downward_1280x720.json'
WATER_N = 1.333     # flat port: the focal length grows by n (calibration file's own note)


def tape(video, every_s, tape_m, tile_m, water) -> int:
    """Height from the grating against a tape, through the shipped calibration."""
    import cv2
    import json
    cal = json.loads(CALIB.read_text())
    cap = cv2.VideoCapture(str(video))
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    f = cal['camera_matrix'][0][0] * w / cal['image_width'] * (WATER_N if water else 1.0)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    hs, seen = [], 0
    for i in range(0, n, max(1, int(round(every_s * fps)))):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, img = cap.read()
        if not ok:
            break
        seen += 1
        g = measure(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if g is not None:
            hs.append(g.height_m(f, tile_m))
    if not hs:
        print(f'{video.name}: the grating never answered ({seen} frames)')
        return 1
    e = np.abs(np.array(hs) / tape_m - 1.0)
    print(f'{video.name}: {len(hs)}/{seen} answered, f {f:.1f} px ({"water" if water else "air"}), '
          f'height median {np.median(hs):.3f} m against tape {tape_m:.3f} m; '
          f'error median {np.median(e) * 100:.1f} %, p95 {np.percentile(e, 95) * 100:.1f} %')
    ok = np.median(e) < 0.03 and np.percentile(e, 95) < 0.06
    print('PASS' if ok else 'FAIL (median < 3 %, p95 < 6 %)')
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('video', nargs='?', type=pathlib.Path)
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--every', type=float, default=0.5, help='seconds between sampled frames')
    ap.add_argument('--ungraded', action='store_true', help='report only (a stress case)')
    ap.add_argument('--tape-m', type=float, help='lens-to-floor height on the tape, metres')
    ap.add_argument('--tile-m', type=float, help='tile pitch, grout centre to grout centre, metres')
    ap.add_argument('--water', action='store_true', help='footage under water (focal x 1.333)')
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    if not a.video:
        ap.error('a video, or --self-test')
    if a.tape_m:
        if not a.tile_m:
            ap.error('--tape-m needs --tile-m')
        return tape(a.video, a.every, a.tape_m, a.tile_m, a.water)
    return run(a.video, a.every, not a.ungraded)


if __name__ == '__main__':
    sys.exit(main())
