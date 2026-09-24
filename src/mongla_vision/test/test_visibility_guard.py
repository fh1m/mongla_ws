"""The vehicle must not steer its own target out of frame — and must remember
where it went if it does.

⛔ THE GAP. Every rung, cue and re-identification built so far works in the
IMAGE PLANE, downstream of where the vehicle chose to point. There is no
field-of-view guard anywhere in mongla_control, so the perception stack can be
perfect and still lose the lock because the controller turned away.

⛔ AND THE PRIORITY IS NOT NEGOTIABLE. Visibility is a SOFT constraint. A
vehicle that holds a lock into a wall has optimised the wrong thing, which is
why the 2026 safety-critical result keeps collision hard and gives
field-of-view slack.
"""
import math

import pytest

from mongla_vision.tracking.visibility import (CRITICAL, LOST, NEAR_EDGE, OK,
                                               SLACK, Visibility, WorldTarget,
                                               assess, bearing_from_pixel)

W, H, FX, FY = 640.0, 480.0, 500.0, 500.0


def test_a_centred_target_is_safe():
    v = assess(W / 2, H / 2, W, H, FX, FY)
    assert v.state == OK and v.safe and v.scale == 1.0


def test_a_target_near_the_edge_is_flagged_before_it_leaves():
    """⭐ The point: warn while there is still time to act.

    ⛔ AND WHY THE BARRIER IS RECTANGULAR. Against the published CONICAL
    barrier this same box scored h = +0.061 -- comfortably "safe" -- because a
    cone only closes at the diagonal CORNER, so a target can walk out of the
    left edge with margin reported the whole way. A camera's image is a
    rectangle and the guard has to agree with it.
    """
    v = assess(W / 2 + 290, H / 2, W, H, FX, FY)
    assert v.state == NEAR_EDGE
    assert 0.0 < v.scale < 1.0
    edge = assess(W - 1, H / 2, W, H, FX, FY)
    assert edge.state in (CRITICAL, LOST) and edge.scale == 0.0


def test_a_target_outside_the_frame_is_lost():
    v = assess(W * 1.6, H / 2, W, H, FX, FY)
    assert v.state == LOST and v.scale == 0.0


def test_the_barrier_decreases_as_the_target_moves_off_axis():
    """Monotone in the distance to the nearest edge, by construction."""
    hs = [assess(W / 2 + d, H / 2, W, H, FX, FY).h for d in (0, 60, 120, 200)]
    assert all(b < a for a, b in zip(hs, hs[1:])), hs


def test_the_guard_needs_no_RANGE():
    """⭐ WHY THIS FORMULATION AND NOT ANOTHER. Range is the number we are
    least sure of on a monocular vehicle with no DVL; the barrier is computed
    from the box centre and intrinsics alone."""
    import inspect
    from mongla_vision.tracking import visibility
    sig = inspect.signature(visibility.assess)
    assert not any('range' in p or 'depth' in p for p in sig.parameters), (
        'assess() grew a range parameter -- the guard must work without one')


def test_damping_is_tapered_not_a_cliff():
    """A guard that slams from 1.0 to 0.0 makes the controller ring."""
    near = assess(W - 40, H / 2, W, H, FX, FY)
    if near.state == NEAR_EDGE:
        assert 0.0 < near.scale < 1.0


def test_visibility_is_SOFT():
    """⛔ Collision stays hard, field of view gets slack. A vehicle that holds
    a lock into a wall has optimised the wrong thing."""
    assert SLACK is True


def test_no_intrinsics_is_lost_not_a_guess():
    assert assess(10, 10, W, H, 0.0, 0.0).state == LOST


def test_bearing_is_a_unit_vector():
    bx, by, bz = bearing_from_pixel(W / 2, H / 2, W, H, FX, FY)
    assert math.isclose(math.sqrt(bx * bx + by * by + bz * bz), 1.0, abs_tol=1e-9)
    assert bz == pytest.approx(1.0)      # dead centre looks down the axis


# --------------------------------------------------------------------------- #
#  World-frame memory: leaving the frame is not forgetting
# --------------------------------------------------------------------------- #
def test_a_target_seen_ahead_is_remembered_in_the_world():
    t = WorldTarget()
    assert t.observe((0.0, 0.0), yaw_deg=0.0, bearing_deg=0.0,
                     range_m=5.0, now=0.0)
    assert t.xy == pytest.approx((5.0, 0.0))


def test_after_the_vehicle_turns_the_bearing_back_is_known():
    """⭐ THE WHOLE POINT. Image-plane tracking forgets the moment the box
    leaves; a world-frame memory turns that into a navigation problem."""
    t = WorldTarget()
    t.observe((0.0, 0.0), yaw_deg=0.0, bearing_deg=0.0, range_m=5.0, now=0.0)
    rel = t.bearing_from((0.0, 0.0), yaw_deg=90.0, now=1.0)
    assert rel == pytest.approx(-90.0, abs=1e-6)


def test_after_the_vehicle_moves_the_bearing_updates():
    t = WorldTarget()
    t.observe((0.0, 0.0), yaw_deg=0.0, bearing_deg=0.0, range_m=5.0, now=0.0)
    rel = t.bearing_from((5.0, -5.0), yaw_deg=0.0, now=1.0)
    assert rel == pytest.approx(90.0, abs=1e-6)


def test_a_stale_memory_answers_NOTHING():
    """⛔ A position from a minute ago is not knowledge, it is a rumour."""
    t = WorldTarget(max_age_s=10.0)
    t.observe((0.0, 0.0), yaw_deg=0.0, bearing_deg=0.0, range_m=5.0, now=0.0)
    assert t.bearing_from((0.0, 0.0), 0.0, now=5.0) is not None
    assert t.bearing_from((0.0, 0.0), 0.0, now=999.0) is None


def test_no_memory_answers_NOTHING():
    assert WorldTarget().bearing_from((0.0, 0.0), 0.0, now=0.0) is None


def test_a_bad_range_is_REFUSED_not_stored():
    """Range is the weakest number we have. A memory built from a guess sends
    the vehicle confidently to the wrong place."""
    t = WorldTarget()
    for bad in (0.0, -1.0, float('nan'), float('inf')):
        assert not t.observe((0.0, 0.0), 0.0, 0.0, bad, 0.0)
    assert t.xy is None


def test_the_memory_carries_no_confidence_that_looks_like_a_sighting():
    """⛔ It stores where the target WAS. Nothing downstream may read it as
    'the target is there now'."""
    t = WorldTarget()
    t.observe((0.0, 0.0), 0.0, 0.0, 5.0, 0.0)
    assert not hasattr(t, 'score') and not hasattr(t, 'confidence')
    assert not hasattr(t, 'detected')
