"""Sun caustics: detected, and tracked through.

Refracted sunlight draws bright filaments across an outdoor pool floor that move
with the surface waves, not the hull. Measured on real RoboSub 2025 caustics
composited over a real tiled floor with a known 13.4 px shift: raw LK erred
11.19 px with zero refusals, eroded 7x7 erred 0.09 px.

The caustic field here is SYNTHETIC (moving bright ridges) so the test needs no
footage; thresholds come from the real archive (see flow_math).
"""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from duburi_vision.flow.flow_math import (          # noqa: E402
    CAUSTIC_TOPHAT_MAX, caustic_score, detect_corners, robust_flow,
    suppress_caustics)

LK = dict(winSize=(31, 31), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
H, W = 240, 320


def _tiles(shift_x, shift_y):
    """Light tiles with 3 px dark grout and some dark speckle, shifted."""
    rng = np.random.default_rng(1)
    big = np.full((H + 80, W + 80), 170, np.float32)
    big[::40, :] = 60
    big[1::40, :] = 60
    big[2::40, :] = 60
    big[:, ::40] = 60
    big[:, 1::40] = 60
    big[:, 2::40] = 60
    for _ in range(300):
        y, x = rng.integers(0, big.shape[0] - 3), rng.integers(0, big.shape[1] - 3)
        big[y:y + 3, x:x + 3] = 90
    M = np.float32([[1, 0, shift_x], [0, 1, shift_y]])
    return cv2.warpAffine(big, M, (big.shape[1], big.shape[0]))[40:40 + H, 40:40 + W]


def _caustic(t):
    """Thin bright filaments drifting DIAGONALLY at 4 px/frame -- a coherent
    motion that competes with the hull's."""
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    ph = (xx + yy + 4.0 * t)
    a = np.abs(np.sin(ph / 9.0)) ** 16 + np.abs(np.sin((xx - yy - 3.0 * t) / 11.0)) ** 16
    return 1.0 + 1.6 * a


def _frame(t, vx, vy, caustics):
    img = _tiles(vx * t, vy * t)
    if caustics:
        img = img * _caustic(t)
    return np.clip(img, 0, 255).astype(np.uint8)


def _flow(a, b):
    p = np.asarray(detect_corners(a), np.float32).reshape(-1, 1, 2)
    n, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p, None, **LK)
    return robust_flow(p, n, st)


def test_the_detector_separates_sun_from_shade():
    assert caustic_score(_frame(0, 0, 0, False)) < CAUSTIC_TOPHAT_MAX
    assert caustic_score(_frame(0, 0, 0, True)) > CAUSTIC_TOPHAT_MAX


def test_a_black_frame_is_not_caustic():
    assert caustic_score(np.zeros((H, W), np.uint8)) == 0.0


def test_raw_tracking_reports_the_waves_and_eroded_tracking_reports_the_hull():
    vx, vy, B = 2.0, 1.0, 5
    raw_err, ero_err = [], []
    for t in range(0, 20, 4):
        a, b = _frame(t, vx, vy, True), _frame(t + B, vx, vy, True)
        f = _flow(a, b)
        raw_err.append(np.hypot(f[0] - vx * B, f[1] - vy * B))
        g = _flow(suppress_caustics(a), suppress_caustics(b))
        ero_err.append(np.hypot(g[0] - vx * B, g[1] - vy * B))
    assert np.median(raw_err) > 2.0, raw_err       # the control: the defect is real here
    assert np.median(ero_err) < 0.5, ero_err


def test_erosion_costs_nothing_without_caustics():
    vx, vy, B = 2.0, 1.0, 5
    a, b = _frame(0, vx, vy, False), _frame(B, vx, vy, False)
    f = _flow(suppress_caustics(a), suppress_caustics(b))
    assert np.hypot(f[0] - vx * B, f[1] - vy * B) < 0.3
