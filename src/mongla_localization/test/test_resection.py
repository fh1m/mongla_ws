"""A fix from props must refuse a geometry that cannot support one.

Two nearly parallel sight lines still cross, just a long way off and with
enormous sensitivity, and the observer on the circle through three props sees
the same angles from every point on that arc. Both produce a confident wrong
position and no error anywhere, so the tests are built around them.

Every case is constructed from a KNOWN observer position, so the fix is scored
against truth rather than against another estimator.
"""
import math

import pytest

from mongla_localization.resection import (
    Fix, circumcircle, fix_from_bearings, on_danger_circle,
)


def _bearing(frm, to):
    """Compass bearing from one pool point to another: +x is 0, +y is 90."""
    d = math.degrees(math.atan2(to[1] - frm[1], to[0] - frm[0]))
    return d + 360.0 if d < 0.0 else d


def _sight(truth, positions, names=None):
    """Exact bearings an observer at `truth` would measure."""
    return {n: _bearing(truth, positions[n])
            for n in (names or positions)}


# A plausible course: props spread around the pool.
PROPS = {
    'gate':    (0.0, 0.0),
    'slalom':  (8.0, 2.0),
    'bin':     (6.0, -5.0),
    'torpedo': (12.0, -1.0),
}


# --- it recovers a known position -------------------------------------------


def test_two_props_recover_the_observer():
    truth = (4.0, -2.0)
    got = fix_from_bearings(_sight(truth, PROPS, ['gate', 'slalom']), PROPS)
    assert got.ok
    assert got.x_m == pytest.approx(truth[0], abs=1e-6)
    assert got.y_m == pytest.approx(truth[1], abs=1e-6)
    assert got.used == 2


def test_four_props_still_recover_it_and_the_residual_is_zero():
    truth = (5.0, -1.0)
    got = fix_from_bearings(_sight(truth, PROPS), PROPS)
    assert got.ok and got.used == 4
    assert math.hypot(got.x_m - truth[0], got.y_m - truth[1]) < 1e-6
    assert got.residual_m < 1e-6


def test_a_bearing_error_shows_up_as_a_residual():
    # Exact bearings give a zero residual, so a non-zero one is the only signal
    # that the sightings disagree. It must not stay zero when they do.
    truth = (5.0, -1.0)
    s = _sight(truth, PROPS)
    s['bin'] += 3.0                                  # one bad sighting
    got = fix_from_bearings(s, PROPS)
    assert got.ok
    assert got.residual_m > 0.05, 'a disagreement must be visible'


# --- refusals ---------------------------------------------------------------


def test_one_prop_is_not_a_fix():
    got = fix_from_bearings({'gate': 10.0}, PROPS)
    assert not got.ok and 'need 2' in got.reason
    assert math.isnan(got.x_m)


def test_a_prop_with_no_known_position_does_not_count():
    got = fix_from_bearings({'gate': 10.0, 'octagon': 40.0}, PROPS)
    assert not got.ok, 'octagon is not in the map, so this is a single sighting'


def test_nearly_parallel_sightings_are_refused_not_reported():
    # Two props almost in line with the observer: the crossing slides a long
    # way for a small bearing error, and the solve would still succeed.
    props = {'a': (0.0, 0.0), 'b': (0.5, 0.0)}
    truth = (-40.0, 0.0)
    got = fix_from_bearings(_sight(truth, props), props)
    assert not got.ok
    assert 'deg' in got.reason and got.separation_deg < 12.0


def test_the_separation_is_reported_even_on_refusal():
    # Reported so an operator can see HOW thin the geometry was, and whether
    # moving a metre would fix it. Props offset slightly so the number is a
    # real small angle rather than an exact zero.
    props = {'a': (0.0, 0.0), 'b': (0.5, 0.4)}
    got = fix_from_bearings(_sight((-40.0, 0.0), props), props)
    assert not got.ok
    assert 0.0 < got.separation_deg < 12.0


def test_a_wide_geometry_is_accepted():
    props = {'a': (0.0, 0.0), 'b': (0.0, 8.0)}
    truth = (4.0, 4.0)
    got = fix_from_bearings(_sight(truth, props), props)
    assert got.ok and got.separation_deg > 80.0


# --- the danger circle ------------------------------------------------------


def test_the_circumcircle_of_three_points():
    centre, r = circumcircle((0.0, 0.0), (2.0, 0.0), (0.0, 2.0))
    assert centre == pytest.approx((1.0, 1.0))
    assert r == pytest.approx(math.sqrt(2.0))


def test_collinear_props_have_no_circumcircle():
    assert circumcircle((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)) is None


def test_an_observer_on_the_circle_is_flagged():
    a, b, c = (0.0, 0.0), (4.0, 0.0), (0.0, 4.0)
    (ux, uy), r = circumcircle(a, b, c)
    on = (ux + r * math.cos(2.2), uy + r * math.sin(2.2))
    assert on_danger_circle(on, a, b, c)


def test_an_observer_well_inside_the_circle_is_fine():
    a, b, c = (0.0, 0.0), (4.0, 0.0), (0.0, 4.0)
    (ux, uy), _ = circumcircle(a, b, c)
    assert not on_danger_circle((ux, uy), a, b, c)


def test_collinear_props_are_treated_as_dangerous():
    # No circle means no angles-only fix either; saying "safe" would be worse.
    assert on_danger_circle((1.0, 1.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0))
