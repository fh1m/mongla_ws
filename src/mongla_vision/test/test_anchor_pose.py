"""Metric 6-DoF pose of a known-size planar target -- the firing solution.

Ground truth is CONSTRUCTED: project a patch of known size from a known pose
and check what comes back. That is the right test for geometry, and it is the
only way to catch a sign error -- a stable, plausible, mirrored answer looks
correct in every recorded clip.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.anchor.pose import TargetPose, target_pose   # noqa: E402

K = np.array([[1027.873, 0.0, 617.323],
              [0.0, 1033.857, 373.022],
              [0.0, 0.0, 1.0]])
W_PX, H_PX, WIDTH_M = 240.0, 190.0, 0.60
_REF = np.stack([np.random.default_rng(0).uniform(0, W_PX, 120),
                 np.random.default_rng(1).uniform(0, H_PX, 120)], 1)


def _render(yaw=0.0, pitch=0.0, roll=0.0, dist=2.0, noise=0.5, seed=0,
            ref=_REF):
    """Project the patch from a known pose. Truth by construction."""
    import cv2
    mpp = WIDTH_M / W_PX
    obj = np.zeros((len(ref), 3))
    obj[:, 0] = (ref[:, 0] - W_PX / 2) * mpp
    obj[:, 1] = (ref[:, 1] - H_PX / 2) * mpp
    ry, rp, rr = math.radians(yaw), math.radians(pitch), math.radians(roll)
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0],
                   [-math.sin(ry), 0, math.cos(ry)]])
    Rp = np.array([[1, 0, 0], [0, math.cos(rp), -math.sin(rp)],
                   [0, math.sin(rp), math.cos(rp)]])
    Rr = np.array([[math.cos(rr), -math.sin(rr), 0],
                   [math.sin(rr), math.cos(rr), 0], [0, 0, 1]])
    rvec, _ = cv2.Rodrigues(Rp @ Ry @ Rr)
    px, _ = cv2.projectPoints(obj, rvec, np.array([[0.], [0.], [dist]]),
                              K, np.zeros((1, 5)))
    px = px.reshape(-1, 2)
    if noise:
        px = px + np.random.default_rng(seed).normal(0, noise, px.shape)
    return px


def _pose(**kw):
    return target_pose(_REF, _render(**kw), K, width_m=WIDTH_M,
                       ref_size_px=(W_PX, H_PX))


# --------------------------------------------------------------------------- #
#  The metric answer
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('dist', [1.0, 1.5, 2.0, 3.0])
def test_RANGE_is_metric_and_that_is_the_point(dist):
    """A homography gives a scale RATIO; this gives metres, because the patch's
    real width is known. Range is what turns "aligned" into "aligned at the
    standoff the launcher was characterised at"."""
    p = _pose(yaw=12.0, dist=dist)
    assert p.ok, p.reason
    assert p.range_m == pytest.approx(dist, abs=0.01)


def test_YAW_SIGN_matches_the_constructed_rotation():
    """THE load-bearing sign, pinned the way the anchor's findHomography
    argument order is. A mirrored yaw sends the hull the wrong way to square
    up, and it looks entirely plausible on every frame."""
    assert _pose(yaw=20.0).yaw_deg == pytest.approx(20.0, abs=1.5)
    assert _pose(yaw=-20.0).yaw_deg == pytest.approx(-20.0, abs=1.5)


def test_PITCH_SIGN_matches_too_and_the_axes_do_not_swap():
    p = _pose(pitch=15.0)
    assert p.pitch_deg == pytest.approx(15.0, abs=1.5)
    assert abs(p.yaw_deg) < 3.0, 'pitch leaked into yaw'
    q = _pose(yaw=15.0)
    assert abs(q.pitch_deg) < 3.0, 'yaw leaked into pitch'


def test_ROLL_is_reported_separately():
    p = _pose(roll=25.0, dist=2.0)
    assert p.ok and abs(p.roll_deg) == pytest.approx(25.0, abs=3.0)


def test_accuracy_under_realistic_matcher_noise():
    errs = [(_pose(yaw=20.0, pitch=-10.0, dist=1.5, seed=s)) for s in range(20)]
    ok = [e for e in errs if e.ok]
    assert len(ok) >= 18
    assert np.median([e.yaw_deg for e in ok]) == pytest.approx(20.0, abs=1.5)
    assert np.median([e.range_m for e in ok]) == pytest.approx(1.5, abs=0.01)


# --------------------------------------------------------------------------- #
#  The flip ambiguity -- reported as a width, not resolved by luck
# --------------------------------------------------------------------------- #
def test_HEAD_ON_is_answered_not_refused():
    """The gating inversion. At true 0 deg the two branches are formally
    ambiguous (reprojection ratio ~0.85) and they disagree by ~3 deg, BOTH
    saying "you are square" -- which is the answer a firing gate wants. An
    ambiguity-ratio refusal would discard it in exactly the geometry where it
    is needed."""
    p = _pose(yaw=0.0, pitch=0.0)
    assert p.ok, p.reason
    assert p.off_axis_deg < 3.0


def test_the_spread_GROWS_with_tilt_and_is_reported():
    """Near head-on the mirror branch is close, so the interval is narrow. Off
    to one side the branches separate, and the caller must be told."""
    near = _pose(yaw=0.0)
    far = _pose(yaw=25.0)
    assert near.yaw_spread_deg < 6.0
    assert far.yaw_spread_deg > 20.0


def test_square_within_uses_the_WORSE_branch():
    """Both branches are legitimate answers, so 'square' only counts if the
    worse one is also inside tolerance. Reading `off_axis_deg` alone fires on
    the lucky branch -- which is the whole failure this module exists to avoid.
    """
    p = _pose(yaw=15.0)
    assert p.off_axis_deg == pytest.approx(15.0, abs=2.0)
    # its own best estimate is 15 deg off, so 20 deg tolerance would pass on
    # `off_axis_deg` alone -- but the branches are ~29 deg apart
    assert p.yaw_spread_deg > 20.0
    assert p.square_within(20.0) is False
    assert _pose(yaw=0.0).square_within(8.0) is True


# --------------------------------------------------------------------------- #
#  Refusals
# --------------------------------------------------------------------------- #
def test_it_refuses_rather_than_guessing():
    live = _render(yaw=10.0)
    assert not target_pose(_REF[:3], live[:3], K, width_m=WIDTH_M,
                           ref_size_px=(W_PX, H_PX)).ok
    assert not target_pose(_REF, live[:10], K, width_m=WIDTH_M,
                           ref_size_px=(W_PX, H_PX)).ok
    assert not target_pose(_REF, live, K, width_m=0.0,
                           ref_size_px=(W_PX, H_PX)).ok
    assert not target_pose(_REF, live, K, width_m=WIDTH_M,
                           ref_size_px=(0.0, H_PX)).ok


def test_garbage_correspondences_are_refused_on_REPROJECTION():
    """A pose that does not explain the pixels it was fitted to is not a pose,
    whatever the ambiguity says."""
    rng = np.random.default_rng(3)
    live = rng.uniform(0, 600, (len(_REF), 2))
    p = target_pose(_REF, live, K, width_m=WIDTH_M, ref_size_px=(W_PX, H_PX))
    assert not p.ok and 'reprojection' in p.reason


def test_a_refused_pose_is_never_SQUARE():
    assert TargetPose(ok=False).square_within(90.0) is False
