"""The FOLLOW rung must carry SIZE, not only position.

⛔ WHY IT MATTERS. Section 23 measured detector confidence against apparent
size and found it peaks mid-range, which is why the approach controller reads
size rather than range. A follower that can only translate reports a constant
size while the target grows, so the controller sees a target that never gets
closer and drives through the band.

⭐ THE RECIPE IS OLD AND PROVEN: pyramidal Lucas-Kanade, a forward-backward
check, and a RANSAC similarity fit. A 2026 mobile-robot comparison puts
optical flow at RMSE 10.79 px @ 30 fps against CSRT's 252.35 px @ 4 fps. We
already had the first two legs; this is the third.
"""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from mongla_vision.tracking.follower import Follower


def textured(w=640, h=480, seed=0):
    """A frame with enough corners for LK to have anything to track."""
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, (h, w), dtype=np.uint8)
    return cv2.GaussianBlur(img, (3, 3), 0)


def warp(img, s=1.0, tx=0.0, ty=0.0):
    h, w = img.shape[:2]
    cx, cy = w / 2.0, h / 2.0
    M = np.array([[s, 0.0, cx - s * cx + tx],
                  [0.0, s, cy - s * cy + ty]], np.float32)
    return cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REFLECT)


def test_a_pure_translation_is_followed():
    f = Follower()
    img = textured()
    assert f.reset(img, (220, 160, 420, 320)) > 0
    r = f.step(warp(img, 1.0, 12.0, 0.0))
    assert r.ok
    assert r.xyxy[0] == pytest.approx(232, abs=6)


def test_an_approaching_target_grows_the_box():
    """⭐ THE POINT. Translation alone reports a constant size while the
    target closes; the controller then never sees it arrive."""
    f = Follower()
    img = textured(seed=1)
    assert f.reset(img, (220, 160, 420, 320)) > 0
    r = f.step(warp(img, 1.08))
    assert r.ok
    w0, w1 = 420 - 220, r.xyxy[2] - r.xyxy[0]
    assert w1 > w0 * 1.02, f'box did not grow: {w0} -> {w1}'
    assert r.scale > 1.0


def test_a_receding_target_shrinks_the_box():
    f = Follower()
    img = textured(seed=2)
    assert f.reset(img, (220, 160, 420, 320)) > 0
    r = f.step(warp(img, 0.94))
    assert r.ok and r.scale < 1.0
    assert (r.xyxy[2] - r.xyxy[0]) < (420 - 220)


def test_an_absurd_scale_is_refused_not_trusted():
    """⛔ A target does not double in one frame. A fit that says so has
    latched onto something else, and the box falls back to translation."""
    f = Follower()
    img = textured(seed=3)
    assert f.reset(img, (220, 160, 420, 320)) > 0
    r = f.step(warp(img, 1.6))
    if r.ok:
        assert 0.90 <= r.scale <= 1.11, f'unclamped scale {r.scale}'


def test_scale_is_one_when_the_fit_is_refused():
    """The contract: scale == 1.0 means 'carried by translation', not
    'measured as unchanged'."""
    f = Follower()
    img = textured(seed=4)
    assert f.reset(img, (220, 160, 420, 320)) > 0
    f._min_fit_pts = 10_000          # force every fit to be refused
    r = f.step(warp(img, 1.08))
    assert r.ok and r.scale == 1.0


def test_the_forward_backward_check_is_still_there():
    """The similarity fit is an addition, not a replacement -- a fit computed
    on points that never passed FB would be confidently wrong."""
    f = Follower()
    img = textured(seed=5)
    assert f.reset(img, (220, 160, 420, 320)) > 0
    r = f.step(warp(img, 1.0, 6.0, 0.0))
    assert r.ok and np.isfinite(r.fb_median)
