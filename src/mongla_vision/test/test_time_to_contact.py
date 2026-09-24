"""Time to contact for a vehicle that is itself moving.

The prototype solved the quadratic correctly and assumed a stationary
observer, which is never the case here. These tests pin the difference that
subtraction makes, and -- more importantly -- the refusals, because a TTC of
900 s and "not approaching at all" lead to different decisions and a
controller cannot tell them apart if both arrive as floats.
"""
import math

import pytest

from mongla_vision.time_to_contact import (INSIDE, NEVER, OK,
                                           time_to_contact as ttc)


def test_a_vehicle_closing_on_a_fixed_prop_has_a_finite_ttc():
    """⭐ THE WHOLE POINT. A stationary-observer solver returns 'never' here,
    because the PROP is not moving -- the closing motion is entirely ours."""
    c = ttc(rel_pos=(10.0, 0.0, 0.0), target_vel=(0.0, 0.0, 0.0),
            ego_vel=(1.0, 0.0, 0.0), radius_m=0.5)
    assert c.ok, c.reason
    assert c.seconds == pytest.approx(9.5, abs=0.01)


def test_without_ego_velocity_the_same_case_is_never():
    """The prototype's answer, kept as the control that makes the fix visible."""
    c = ttc(rel_pos=(10.0, 0.0, 0.0), target_vel=(0.0, 0.0, 0.0),
            ego_vel=None, radius_m=0.5)
    assert c.state == NEVER


def test_the_radius_is_the_surface_not_the_centre():
    """Contact is reaching the safety radius. Solving to the centre would
    report a collision later than it happens, which is the wrong direction to
    be wrong in."""
    a = ttc((10.0, 0, 0), (0, 0, 0), (1.0, 0, 0), radius_m=0.5).seconds
    b = ttc((10.0, 0, 0), (0, 0, 0), (1.0, 0, 0), radius_m=2.0).seconds
    assert b < a and b == pytest.approx(8.0, abs=0.01)


def test_a_receding_target_never_contacts():
    c = ttc((5.0, 0, 0), (1.0, 0, 0), (0.0, 0, 0), radius_m=0.5)
    assert c.state == NEVER and 'past' in c.reason or c.state == NEVER


def test_a_pass_alongside_is_not_an_approach():
    """⛔ THE CASE range/speed GETS WRONG. The vehicle closes fast but will
    pass 3 m to the side; a linear range-over-speed estimate calls that a
    contact in a few seconds."""
    c = ttc(rel_pos=(10.0, 3.0, 0.0), target_vel=(0.0, 0.0, 0.0),
            ego_vel=(1.0, 0.0, 0.0), radius_m=0.5)
    assert c.state == NEVER
    assert c.closest_m == pytest.approx(3.0, abs=0.01), (
        'closest approach is the number a standoff decision needs; a bare '
        '"never" throws it away')


def test_closest_approach_is_reported_even_when_contact_happens():
    c = ttc((10.0, 0.1, 0.0), (0, 0, 0), (1.0, 0, 0), radius_m=0.5)
    assert c.ok and math.isfinite(c.closest_m)


def test_already_inside_the_radius_is_zero_not_a_solve():
    c = ttc((0.2, 0.0, 0.0), (0, 0, 0), (1.0, 0, 0), radius_m=0.5)
    assert c.state == INSIDE and c.seconds == 0.0


# --------------------------------------------------------------------------- #
#  Refusals
# --------------------------------------------------------------------------- #
def test_below_the_noise_floor_it_refuses_rather_than_extrapolating():
    """A TTC of 900 s and 'not approaching' are different decisions."""
    c = ttc((10.0, 0, 0), (0, 0, 0), (0.001, 0, 0), radius_m=0.5)
    assert c.state == NEVER and 'noise floor' in c.reason


def test_a_non_finite_input_refuses(  ):
    for bad in ((float('nan'), 0, 0), (float('inf'), 0, 0)):
        assert ttc(bad, (0, 0, 0), (1.0, 0, 0)).state == NEVER


def test_no_radius_refuses():
    assert ttc((10.0, 0, 0), (0, 0, 0), (1.0, 0, 0), radius_m=0.0).state == NEVER


def test_every_answer_explains_itself():
    for args in (((10.0, 0, 0), (0, 0, 0), (1.0, 0, 0)),
                 ((10.0, 3.0, 0), (0, 0, 0), (1.0, 0, 0)),
                 ((0.2, 0, 0), (0, 0, 0), (1.0, 0, 0)),
                 ((10.0, 0, 0), (0, 0, 0), (0.0, 0, 0))):
        assert ttc(*args).reason.strip()


# --------------------------------------------------------------------------- #
#  The property that makes it usable for approach control
# --------------------------------------------------------------------------- #
def test_ttc_decreases_monotonically_on_a_straight_approach():
    """The falsifier from the plan: on an approach, TTC must fall, and its
    zero must land where contact actually is."""
    prev = None
    for r in (10.0, 8.0, 6.0, 4.0, 2.0, 1.0):
        c = ttc((r, 0, 0), (0, 0, 0), (1.0, 0, 0), radius_m=0.5)
        assert c.ok or c.state == INSIDE
        if c.ok:
            if prev is not None:
                assert c.seconds < prev, f'TTC rose at range {r}'
            prev = c.seconds
    final = ttc((0.5, 0, 0), (0, 0, 0), (1.0, 0, 0), radius_m=0.5)
    assert final.state == INSIDE, 'the zero does not land at the radius'


def test_a_moving_target_and_a_moving_vehicle_compose():
    """Both moving toward each other: closing speed is the sum."""
    c = ttc((10.0, 0, 0), (-0.5, 0, 0), (0.5, 0, 0), radius_m=0.0 + 0.5)
    assert c.ok and c.seconds == pytest.approx(9.5, abs=0.01)
