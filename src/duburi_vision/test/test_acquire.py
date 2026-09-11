"""The standoff must be arithmetic, and which term binds must be the right one.

A waypoint that stops at an arbitrary distance can park the hull where the prop
is present and undetectable: the mission then cannot tell "not there" from "too
far to see". These tests pin the two regimes -- pixels bind for a small prop,
water binds for a large one -- because getting that backwards puts a gate
approach 38 m out, looking at silt.
"""
import math

import pytest

from duburi_vision.acquire import (
    DETECT_FLOOR_PX, DETECT_MARGIN, Leg, VISIBILITY_M, detection_range_m,
    expanding_box, search_radius_m, standoff_for, total_path_m,
)

F_PX = 513.94          # the measured forward focal length at 640 wide


# --- the pinhole relation ---------------------------------------------------


def test_detection_range_is_the_pinhole_relation():
    # A 1 m prop at a 10 px floor is visible to f/10 metres.
    assert detection_range_m(1.0, 500.0, min_box_px=10.0) == pytest.approx(50.0)


def test_a_wider_prop_is_visible_from_further():
    assert (detection_range_m(3.0, F_PX) > detection_range_m(0.3, F_PX))


def test_nonsense_inputs_give_zero_not_infinity():
    assert detection_range_m(0.0, F_PX) == 0.0
    assert detection_range_m(1.0, 0.0) == 0.0
    assert detection_range_m(1.0, F_PX, min_box_px=0.0) == 0.0


# --- which constraint binds -------------------------------------------------


def test_pixels_bind_for_a_slalom_pipe():
    # 33.4 mm: detectable to 1.7 m and no further, whatever the water does.
    # No prior map can put a hull in detection range of one from across a pool.
    assert detection_range_m(0.0334, F_PX) == pytest.approx(1.7, abs=0.1)
    assert standoff_for(0.0334, F_PX) == pytest.approx(0.8, abs=0.01)


def test_water_binds_for_a_gate():
    # The pixel-limited range is 154 m. Nobody sees a gate at 154 m underwater.
    assert detection_range_m(3.0, F_PX) > 100.0
    assert standoff_for(3.0, F_PX, visibility_m=6.0) == pytest.approx(6.0)


def test_clearer_water_moves_the_gate_standoff_and_not_the_bin_one():
    # The bin is pixel-limited at 4.3 m, so better visibility changes nothing.
    assert standoff_for(3.0, F_PX, visibility_m=12.0) == pytest.approx(12.0)
    assert standoff_for(0.335, F_PX, visibility_m=12.0) == pytest.approx(
        standoff_for(0.335, F_PX, visibility_m=6.0))


def test_the_standoff_never_lands_on_the_hull():
    assert standoff_for(0.001, F_PX) >= 0.8


def test_the_margin_keeps_the_box_off_the_cliff():
    # The floor was MEASURED as a cliff at ~10 px with conf 0.21-0.28, so the
    # standoff aims at a box `DETECT_MARGIN` times bigger.
    far = detection_range_m(1.0, F_PX, min_box_px=DETECT_FLOOR_PX)
    aimed = standoff_for(1.0, F_PX, visibility_m=1e6)
    assert aimed == pytest.approx(far / DETECT_MARGIN)


# --- the search -------------------------------------------------------------


def test_the_search_is_sized_from_the_error_we_carry():
    # A 3 deg heading error over a 6 m leg throws the end 0.31 m off line, and
    # the fix already disagreed with itself by 0.2 m.
    r = search_radius_m(0.2, 6.0, heading_sigma_deg=3.0)
    assert r == pytest.approx(0.2 + 6.0 * math.tan(math.radians(3.0)), abs=1e-6)


def test_a_longer_leg_needs_a_wider_search():
    assert search_radius_m(0.2, 12.0) > search_radius_m(0.2, 3.0)


def test_the_search_never_collapses_to_nothing():
    assert search_radius_m(0.0, 0.0) >= 0.3


def test_the_box_expands_and_stays_inside_its_reach():
    legs = expanding_box(1.0, reach_m=3.0)
    assert [l.run_m for l in legs] == [1.0, 1.0, 2.0, 2.0, 3.0, 3.0]
    assert all(l.turn_deg == 90.0 for l in legs)
    assert max(l.run_m for l in legs) <= 3.0


def test_a_tight_reach_gives_a_short_pattern():
    assert len(expanding_box(1.0, reach_m=1.0)) == 2


def test_a_step_below_the_floor_is_raised_to_it():
    legs = expanding_box(0.01, reach_m=5.0)
    assert legs[0].run_m >= 0.5


def test_the_pattern_is_bounded_by_max_legs():
    assert len(expanding_box(0.5, reach_m=1000.0, max_legs=5)) == 5


def test_the_swim_distance_is_reported_because_it_is_a_budget():
    legs = expanding_box(1.0, reach_m=2.0)
    assert total_path_m(legs) == pytest.approx(6.0)
