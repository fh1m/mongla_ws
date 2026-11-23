"""A wrong heading zero is worse than no heading zero.

Every later turn inherits it, and the mission stops being able to tell it is
lost -- so these tests are mostly about the cases where the anchor must REFUSE.
The geometry cases are worked by hand in the module docstring and checked here
against those exact numbers, because a sign error drives the vehicle the wrong
way with total confidence.
"""
import math

import pytest

from mongla_localization.pose_cluster import RULE_VIEWPOINT
from mongla_localization.heading_anchor import (
    absolute_heading, anchor_from, apply_offset,
)
from mongla_localization.pose_cluster import Fused


def _fused(yaw, support=10, spread=1.0, decided=True, rule=RULE_VIEWPOINT):
    return Fused(decided=decided, yaw_deg=yaw, support=support,
                 spread_deg=spread, rule=rule)


# --- the geometry, on the two cases derived by hand -------------------------


def test_head_on_at_a_north_facing_board_means_facing_south():
    # The only way to see a north-facing face is from the north, looking south.
    assert absolute_heading(0.0, 0.0) == pytest.approx(180.0)


def test_a_normal_seen_to_the_right_means_the_hull_yawed_left():
    # Worked in the docstring: hull at 190, north-facing board, normal appears
    # 10 deg LEFT of the reversed optical axis, so yaw_deg = -10.
    assert absolute_heading(-10.0, 0.0) == pytest.approx(190.0)
    # And the mirror of that.
    assert absolute_heading(+10.0, 0.0) == pytest.approx(170.0)


def test_the_board_bearing_carries_straight_through():
    # An east-facing board (B=90) seen head-on puts the hull at 270.
    assert absolute_heading(0.0, 90.0) == pytest.approx(270.0)


def test_a_heading_is_a_compass_bearing_not_a_signed_offset():
    # Mixing those two conventions is its own recurring defect here.
    for yaw in (-170.0, -90.0, 0.0, 90.0, 170.0):
        h = absolute_heading(yaw, 0.0)
        assert 0.0 <= h < 360.0


# --- the offset -------------------------------------------------------------


def test_the_offset_converts_the_hull_s_relative_yaw_to_absolute():
    # Hull reads 30 relative; truth is 180. The offset must be +150.
    got = anchor_from(_fused(0.0), hull_relative_yaw_deg=30.0,
                      board_world_bearing_deg=0.0)
    assert got.ok
    assert got.absolute_deg == pytest.approx(180.0)
    assert got.offset_deg == pytest.approx(150.0)
    assert apply_offset(30.0, got.offset_deg) == pytest.approx(180.0)


def test_the_offset_takes_the_short_way_round():
    # A +350 offset and a -10 offset are the same rotation; reporting 350 to an
    # operator reads as a fault.
    got = anchor_from(_fused(0.0), hull_relative_yaw_deg=190.0,
                      board_world_bearing_deg=0.0)
    assert abs(got.offset_deg) <= 180.0


# --- refusals, which are the point ------------------------------------------


def test_an_undecided_pose_cannot_redefine_north():
    got = anchor_from(Fused(False, reason='largest cluster has 2 poses'),
                      hull_relative_yaw_deg=0.0, board_world_bearing_deg=0.0)
    assert not got.ok
    assert 'not decided' in got.reason
    assert math.isnan(got.offset_deg), 'a refused anchor must not hand back a number'


def test_a_thin_fuse_is_refused_even_though_it_was_decided():
    # Good enough to steer on for a second is not good enough to redefine north.
    got = anchor_from(_fused(10.0, support=4), hull_relative_yaw_deg=0.0,
                      board_world_bearing_deg=0.0)
    assert not got.ok and 'support 4' in got.reason


def test_a_wide_pose_is_refused():
    got = anchor_from(_fused(10.0, spread=20.0), hull_relative_yaw_deg=0.0,
                      board_world_bearing_deg=0.0)
    assert not got.ok and 'too wide' in got.reason


def test_a_nan_spread_is_refused_rather_than_compared_away():
    # NaN fails every comparison, so a naive `spread > max` test would ACCEPT it.
    got = anchor_from(_fused(10.0, spread=float('nan')),
                      hull_relative_yaw_deg=0.0, board_world_bearing_deg=0.0)
    assert not got.ok


def test_the_fuse_rule_is_carried_onto_the_anchor():
    got = anchor_from(_fused(0.0), hull_relative_yaw_deg=0.0,
                      board_world_bearing_deg=0.0)
    assert got.ok and got.rule == RULE_VIEWPOINT


def test_a_COUNTED_pose_never_redefines_a_heading():
    """Issue #55: 'support' is a vote, and the vote can be the mirror branch.
    Only a viewpoint resolution may move the heading zero."""
    got = anchor_from(_fused(0.0, rule='support'), hull_relative_yaw_deg=0.0,
                      board_world_bearing_deg=0.0)
    assert not got.ok and 'viewpoint' in got.reason


# --- the unanchored path must not change under anyone's feet ----------------


def test_never_anchored_passes_relative_headings_straight_through():
    assert apply_offset(90.0, None) == pytest.approx(90.0)
    assert apply_offset(90.0, float('nan')) == pytest.approx(90.0)


def test_apply_offset_returns_a_compass_bearing():
    assert apply_offset(350.0, 20.0) == pytest.approx(10.0)
    assert apply_offset(10.0, -20.0) == pytest.approx(350.0)


# --------------------------------------------------------------------------- #
#  snap_to_grid: a DRIFT BOUND, not a heading source
# --------------------------------------------------------------------------- #
def test_a_small_drift_is_pulled_back_onto_the_grid():
    """A free-running BNO drifts without bound. The grid's residual should be
    constant, so a moving residual IS the drift and can be removed."""
    from mongla_localization.heading_anchor import snap_to_grid

    # Hull truly on the grid at 30 deg; gyro has drifted 3 deg.
    assert snap_to_grid(33.0, 30.0) == pytest.approx(30.0)
    assert snap_to_grid(27.0, 30.0) == pytest.approx(30.0)


def test_it_does_not_move_a_heading_already_on_the_grid():
    from mongla_localization.heading_anchor import snap_to_grid

    assert snap_to_grid(30.0, 30.0) == pytest.approx(30.0)


def test_the_four_fold_ambiguity_is_respected_not_resolved():
    """A square grid looks identical from four directions, so the correction
    is to the NEAREST grid line -- never to 'the' grid line."""
    from mongla_localization.heading_anchor import snap_to_grid

    # Facing ~120 deg with the grid reading 30: 120 is already on a grid line
    # (30 + 90), so nothing should move.
    assert snap_to_grid(120.0, 30.0) == pytest.approx(120.0)
    assert snap_to_grid(122.0, 30.0) == pytest.approx(120.0)


def test_a_large_disagreement_is_REFUSED_not_applied():
    """The safety. A correction beyond the bound means the estimate and the
    floor disagree about WHICH grid line is which -- a four-fold aliasing
    error. Applying it snaps the hull 90 degrees onto the wrong branch. A
    drifting heading is still roughly right; a confidently wrong one is not."""
    from mongla_localization.heading_anchor import snap_to_grid

    assert snap_to_grid(30.0 + 40.0, 30.0, max_correction_deg=20.0) is None


def test_the_correction_never_exceeds_half_a_cell():
    """Whatever the inputs, the pull is bounded by 45 deg -- so a drifting yaw
    can never be moved further than half a grid cell."""
    from mongla_localization.heading_anchor import snap_to_grid

    for yaw in range(0, 360, 7):
        for grid_a in (0.0, 13.0, 44.0, 89.0):
            got = snap_to_grid(float(yaw), grid_a, max_correction_deg=90.0)
            assert got is not None
            assert abs(got - yaw) <= 45.0 + 1e-9


def test_nan_and_none_are_refused():
    from mongla_localization.heading_anchor import snap_to_grid

    assert snap_to_grid(float('nan'), 30.0) is None
    assert snap_to_grid(30.0, float('nan')) is None
    assert snap_to_grid(None, 30.0) is None


def test_snap_period_180_is_the_lane_lines_symmetry():
    """A line is identical from both ends, so its residual wraps at +/-90,
    not +/-45: a hull 60 deg off a lane is NOT on another branch."""
    from mongla_localization.heading_anchor import snap_to_grid
    assert snap_to_grid(3.0, 0.0, period_deg=180.0) == pytest.approx(0.0)
    assert snap_to_grid(183.0, 0.0, period_deg=180.0) == pytest.approx(180.0)
    # 60 off: the grid reads it as -30 onto another branch; the line refuses.
    assert snap_to_grid(60.0, 0.0, max_correction_deg=40.0) == pytest.approx(90.0)
    assert snap_to_grid(60.0, 0.0, max_correction_deg=40.0,
                        period_deg=180.0) is None


@pytest.mark.parametrize('hull, bearing', [(float('nan'), 30.0), (10.0, float('nan'))])
def test_no_finite_heading_is_no_anchor(hull, bearing):
    # head() returns NaN when the board gives no heading: that anchored ok=True
    # with a NaN offset, and the DSL logged "heading anchored" (audit 2026-11-24).
    got = anchor_from(_fused(20.0), hull, bearing)
    assert not got.ok and 'no finite heading' in got.reason
