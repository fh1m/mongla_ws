"""Pure-math sanity checks for the easing curves.

These run without any ROS / MAVLink dependency and are the cheapest
way to spot a regression in the smoothing -> drive_eased / yaw_glide
pipeline.
"""

import pytest

from duburi_control.motion_easing import (
    smoothstep, smootherstep, trapezoid_ramp,
)


@pytest.mark.parametrize('curve', [smoothstep, smootherstep])
def test_endpoints_clamp(curve):
    assert curve(-0.5) == 0.0
    assert curve(0.0)  == 0.0
    assert curve(1.0)  == 1.0
    assert curve(1.5)  == 1.0


@pytest.mark.parametrize('curve', [smoothstep, smootherstep])
def test_monotonic_in_unit_interval(curve):
    samples = [curve(i / 100.0) for i in range(101)]
    assert samples == sorted(samples)


def test_smootherstep_midpoint_is_half():
    assert smootherstep(0.5) == pytest.approx(0.5, abs=1e-9)


def test_trapezoid_zero_outside_window():
    assert trapezoid_ramp(-0.1, 5.0, 1.0) == 0.0
    assert trapezoid_ramp( 5.1, 5.0, 1.0) == 0.0


def test_trapezoid_cruise_full_in_middle():
    assert trapezoid_ramp(2.5, 5.0, 1.0) == pytest.approx(1.0)


def test_trapezoid_collapses_to_triangle_when_short():
    """No cruise phase when total < 2 * ramp; both edges still hit zero."""
    assert trapezoid_ramp(0.0,  1.0, 1.0) == 0.0
    assert trapezoid_ramp(0.5,  1.0, 1.0) == pytest.approx(1.0)
    assert trapezoid_ramp(1.0,  1.0, 1.0) == 0.0
