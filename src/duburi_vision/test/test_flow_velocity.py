"""Flow -> body velocity, against constructed truth.

The properties that matter are not "a number comes out". They are:
  * pure translation is recovered exactly;
  * pure ROTATION reads as zero velocity -- the whole reason a gyro is here;
  * the regime where the answer is a small difference of large numbers is
    refused rather than returned;
  * an unknown height is a refusal, not a default.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.distance.flow_velocity import (          # noqa: E402
    MIN_NET_FLOW_PX, ROT_FRACTION_MAX, flow_velocity)

F = 513.9          # the vehicle's forward camera at 640x360
DT = 0.05


def _render(vx=0.0, vy=0.0, h=1.5, pitch=0.0, roll=0.0):
    """Image shift produced by a known motion. Truth by construction.

    TRANSLATION scales with 1/h; ROTATION does not. Writing the forward model
    this way is what makes the tests meaningful -- they invert exactly the
    relation the module claims.
    """
    dx = F * (vy / h) * DT + F * roll * DT
    dy = F * (vx / h) * DT + F * pitch * DT
    return dx, dy


def test_pure_TRANSLATION_is_recovered():
    for vx, vy, h in ((0.5, 0.0, 1.5), (0.0, -0.3, 2.0), (0.65, 0.2, 1.0)):
        dx, dy = _render(vx=vx, vy=vy, h=h)
        r = flow_velocity(dx, dy, DT, f_px=F, height_m=h)
        assert r.ok, r.reason
        assert r.vx == pytest.approx(vx, abs=1e-6)
        assert r.vy == pytest.approx(vy, abs=1e-6)


def test_pure_ROTATION_reads_as_NO_VELOCITY():
    """THE POINT OF THE GYRO. A vehicle rotating on the spot produces large
    image flow and is not moving. Without the subtraction this reads as metres
    per second of translation that never happened -- and it would be integrated
    by whatever consumes it."""
    for rate in (0.2, -0.5, 1.0):
        dx, dy = _render(pitch=rate, roll=rate * 0.5)
        r = flow_velocity(dx, dy, DT, f_px=F, height_m=1.5,
                          pitch_rate=rate, roll_rate=rate * 0.5)
        assert not r.ok, f'pure rotation returned a velocity: {r}'
        assert r.net_flow_px < MIN_NET_FLOW_PX


def test_rotation_is_removed_BEFORE_scaling_by_height():
    """Rotational flow does not depend on range, so scaling it by height is
    meaningless. If the subtraction happened after the scaling, the SAME
    rotation at two different heights would leave different residuals."""
    rate = 0.3
    got = []
    for h in (1.0, 3.0):
        dx, dy = _render(vx=0.4, h=h, pitch=rate)
        r = flow_velocity(dx, dy, DT, f_px=F, height_m=h, pitch_rate=rate)
        assert r.ok, r.reason
        got.append(r.vx)
    assert got[0] == pytest.approx(0.4, abs=1e-6)
    assert got[1] == pytest.approx(0.4, abs=1e-6)


def test_a_ROTATION_DOMINATED_interval_is_REFUSED():
    """The failure that does not announce itself: flow and rotation are two
    large numbers whose DIFFERENCE is the answer, so ordinary noise dominates
    while the result still looks like a speed."""
    rate = 2.0                       # fast spin, barely moving
    dx, dy = _render(vx=0.02, h=1.5, pitch=rate)
    r = flow_velocity(dx, dy, DT, f_px=F, height_m=1.5, pitch_rate=rate)
    assert not r.ok and 'rotation-dominated' in r.reason
    assert r.rot_fraction > ROT_FRACTION_MAX


def test_an_UNMEASURABLE_interval_is_not_reported_as_ZERO():
    """A stationary-looking frame is 'we cannot tell', not '0 m/s'. Zero is a
    MEASUREMENT, and feeding a confident zero to a filter is worse than feeding
    it nothing -- it will believe it."""
    r = flow_velocity(0.1, 0.05, DT, f_px=F, height_m=1.5)
    assert not r.ok and 'noise floor' in r.reason
    assert math.isnan(r.vx)


def test_NO_HEIGHT_is_a_REFUSAL_not_a_default():
    """A default height turns an unknown SCALE into a confident wrong speed --
    a clean multiplier, invisible in every plot, and integrated downstream."""
    dx, dy = _render(vx=0.5, h=1.5)
    for bad in (None, 0.0, -1.0):
        assert not flow_velocity(dx, dy, DT, f_px=F, height_m=bad).ok
    assert not flow_velocity(dx, dy, DT, f_px=0.0, height_m=1.5).ok
    assert not flow_velocity(dx, dy, 0.0, f_px=F, height_m=1.5).ok


def test_a_HEIGHT_ERROR_is_a_clean_MULTIPLIER():
    """Stated because it is the dominant real-world error and it is
    well-behaved: a 10 % height error is a 10 % speed error and nothing
    stranger. Worth knowing so nobody hunts a bias that is a scale."""
    dx, dy = _render(vx=0.5, h=1.5)
    r = flow_velocity(dx, dy, DT, f_px=F, height_m=1.65)      # +10 %
    assert r.ok and r.vx == pytest.approx(0.55, rel=1e-6)


def test_the_forward_camera_uses_the_SAME_equation_with_RANGE():
    """`height` is 'distance to the plane being observed'. For a downward
    camera that is altitude; for a forward camera looking at a known-size
    target it is the pose's range. The equation does not care, which is what
    makes a bench test with a wall meaningful."""
    dx, dy = _render(vx=0.25, h=3.0)
    r = flow_velocity(dx, dy, DT, f_px=F, height_m=3.0)
    assert r.ok and r.vx == pytest.approx(0.25, abs=1e-6)
