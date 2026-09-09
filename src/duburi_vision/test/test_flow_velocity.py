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

from duburi_vision.flow.flow_velocity import (          # noqa: E402
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


def test_INCOHERENT_flow_is_refused_however_LARGE_it_is():
    """A SANITY bound, not the noise discriminator -- see the constant.

    Set at 0.5 on contaminated evidence (the camera had been knocked), it
    discarded 60 % of a real 50 cm slide. Real motion runs to a ratio p90 of
    1.02-3.76 because a downward camera is never exactly fronto-parallel to the
    floor. 5.0 now rejects only wildly incoherent flow.
    """
    dx, dy = _render(vx=0.5, h=1.5)          # a large, real-looking flow
    coherent = flow_velocity(dx, dy, DT, f_px=F, height_m=1.5,
                             dispersion_px=0.2)
    assert coherent.ok, coherent.reason
    incoherent = flow_velocity(dx, dy, DT, f_px=F, height_m=1.5,
                               dispersion_px=abs(dy) * 9.0)
    assert not incoherent.ok and 'incoherent' in incoherent.reason
    assert incoherent.dispersion_ratio > 5.0


def test_dispersion_is_OPTIONAL_so_existing_callers_are_unchanged():
    """A caller that cannot measure dispersion still gets the other guards,
    rather than being refused for not supplying something it does not have."""
    dx, dy = _render(vx=0.5, h=1.5)
    assert flow_velocity(dx, dy, DT, f_px=F, height_m=1.5).ok


# --------------------------------------------------------------------------- #
#  The dispersion measure itself
# --------------------------------------------------------------------------- #
def test_flow_dispersion_is_small_for_a_COHERENT_translation():
    import numpy as np
    from duburi_vision.flow.flow_math import flow_dispersion
    p0 = np.random.default_rng(0).uniform(0, 300, (40, 2))
    p1 = p0 + np.array([4.0, -2.0])                 # every point moves alike
    st = np.ones(40)
    assert flow_dispersion(p0, p1, st) < 1e-9


def test_flow_dispersion_is_LARGE_for_scattered_motion():
    import numpy as np
    from duburi_vision.flow.flow_math import flow_dispersion
    rng = np.random.default_rng(1)
    p0 = rng.uniform(0, 300, (40, 2))
    p1 = p0 + rng.normal(0, 6.0, (40, 2))           # no common motion
    assert flow_dispersion(p0, p1, np.ones(40)) > 3.0


def test_flow_dispersion_returns_NONE_not_ZERO_when_unmeasurable():
    """0.0 would read as PERFECT coherence and sail through the gate. Absence
    must be absence -- the same rule the health surface is built on."""
    import numpy as np
    from duburi_vision.flow.flow_math import flow_dispersion
    assert flow_dispersion(None, None, None) is None
    p = np.zeros((2, 2))
    assert flow_dispersion(p, p, np.ones(2)) is None


def test_the_coherence_gate_PASSES_realistic_motion():
    """The regression that matters: measured moving intervals run to a
    dispersion/magnitude ratio p90 of ~3.8, and a gate that rejects those
    silently deletes displacement from the integral -- 60 % of a 50 cm slide
    when this was 0.5."""
    from duburi_vision.flow.flow_velocity import MAX_DISPERSION_RATIO
    assert MAX_DISPERSION_RATIO >= 3.8
    dx, dy = _render(vx=0.4, h=0.5)
    mag = math.hypot(dx, dy)
    for ratio in (0.4, 0.9, 1.5, 3.5):
        r = flow_velocity(dx, dy, DT, f_px=F, height_m=0.5,
                          dispersion_px=mag * ratio)
        assert r.ok, f'rejected real motion at dispersion ratio {ratio}'


def test_a_motionless_camera_is_not_reported_as_rotation_dominated():
    """The reason an operator reads has to point at the real cause.

    Measured on the bench console: a rig with roll/pitch rates of 0.006 rad/s
    and 0.15 px of flow was refused as 'rotation-dominated (1.83 of the flow)'.
    Both quantities were noise, so their ratio meant nothing -- but it sent the
    operator looking for a rotation fault that did not exist.
    """
    v = flow_velocity(0.10, 0.05, 0.03, f_px=513.94, height_m=0.72,
                      roll_rate=-0.006, pitch_rate=-0.002)
    assert not v.ok
    assert 'no measurable flow' in v.reason, v.reason
    assert 'rotation-dominated' not in v.reason


def test_real_rotation_is_still_reported_as_rotation():
    """The negative control: the reorder must not mask a genuine rotation.

    Large total flow, nearly all of it explained by the body rates.
    """
    dt, f = 0.03, 513.94
    rot_px = f * 1.2 * dt                      # ~18 px from a 1.2 rad/s roll
    v = flow_velocity(rot_px, 0.0, dt, f_px=f, height_m=0.72,
                      roll_rate=1.2, pitch_rate=0.0)
    assert not v.ok
    assert 'rotation-dominated' in v.reason, v.reason
