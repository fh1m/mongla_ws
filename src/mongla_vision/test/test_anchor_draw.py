"""The match panel -- the view that says WHAT the lock is on.

A crosshair says the lock moved. It cannot say what it moved onto, and that is
the question an operator actually has. These tests are about the two things
that would make the panel LIE while still looking right:

  * a line landing on the correct pixel of the wrong-sized panel, because the
    two images are drawn at different scales from the backend's working
    resolution;
  * a panel that renders green and confident from a pose that is not locked.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.anchor.anchor import AnchorPose        # noqa: E402
from mongla_vision.anchor.draw import match_panel         # noqa: E402


def _pose(n=20, **kw):
    ref = np.random.default_rng(0).random((n, 2)).astype(np.float32) * [320, 240]
    return AnchorPose(ok=True, tx=0.0, ty=0.0, theta=0.0, scale=1.0,
                      inliers=n, matches=n * 2, ref_pts=ref, live_pts=ref,
                      corners=np.array([[10, 10], [300, 10],
                                        [300, 220], [10, 220]], np.float32),
                      **kw)


def _green_frac(img):
    g = img.astype(np.int16)
    return float(((g[..., 1] - g[..., 0] > 40) & (g[..., 1] - g[..., 2] > 40)
                  ).mean())


def test_the_panel_is_reference_beside_live():
    ref = np.zeros((240, 320), np.uint8)
    live = np.zeros((480, 640, 3), np.uint8)
    out = match_panel(ref, live, _pose(), ref_width=260)
    assert out.shape[1] == 260 + 640
    assert out.shape[0] >= 480


def test_a_locked_pose_draws_correspondences_and_an_UNLOCKED_one_does_not():
    """The failure this guards is a panel that looks confident on a pose the
    anchor refused. Green is the operator's signal that the match is real."""
    ref = np.zeros((240, 320), np.uint8)
    live = np.zeros((480, 640, 3), np.uint8)
    locked = _green_frac(match_panel(ref, live, _pose()))
    lost = _green_frac(match_panel(ref, live, AnchorPose(ok=False, inliers=4)))
    assert locked > 0.002, 'a locked pose drew nothing'
    assert lost < locked / 5, 'an unlocked pose drew a confident panel'


def test_live_points_are_scaled_to_the_LIVE_panels_resolution():
    """The anchor works at 320x240; the live frame may be 640x480. Drawing a
    backend-resolution point straight onto it puts every line in the top-left
    quadrant -- a plausible-looking panel that is wrong everywhere."""
    ref = np.zeros((240, 320), np.uint8)
    live = np.zeros((480, 640, 3), np.uint8)
    p = _pose(n=200)
    p.live_pts = np.array([[319.0, 239.0]] * 200, np.float32)   # far corner
    out = match_panel(ref, live, p, ref_width=260)
    right = out[:, 260:]
    ys, xs = np.where((right[..., 1].astype(np.int16)
                       - right[..., 0].astype(np.int16)) > 40)
    assert xs.max() > 600 and ys.max() > 450, \
        'backend-resolution points were drawn unscaled'


def test_lines_are_subsampled_EVENLY_not_by_taking_the_first_N():
    """Keypoints arrive in raster order, so the first N are all near one image
    edge -- a panel that looks like the lock is only on the top of the prop."""
    ref = np.zeros((240, 320), np.uint8)
    live = np.zeros((240, 320, 3), np.uint8)
    n = 400
    pts = np.stack([np.full(n, 160.0),
                    np.linspace(5, 235, n)], 1).astype(np.float32)
    p = _pose(n=n)
    p.ref_pts = p.live_pts = pts
    out = match_panel(ref, live, p, max_lines=20)
    right = out[:, out.shape[1] - 320:]
    ys = np.where((right[..., 1].astype(np.int16)
                   - right[..., 0].astype(np.int16) > 40).any(axis=1))[0]
    assert ys.min() < 40 and ys.max() > 200, 'lines clustered at one end'
