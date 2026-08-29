"""Style points, and the clause that makes them hard.

Handbook p. 32: "For every 90 degree change in orientation, the AUV increases
the accumulated points. However, returning to the last previous orientation
won't count. I.e. an AUV that rolls 90 degrees and then back to 0 would not
get points."

So a roll to 90 and back to 0 is TWO 90-degree changes and scores ONE. Every
test here is that sentence.
"""

import pytest

from duburi_sim_bridge.scoring import AxisStyle, _quadrant


def _walk(axis, angles):
    for a in angles:
        axis.update(a)
    return axis.score


def test_quadrants_are_centred_on_the_cardinals():
    """0 and 359 are the same orientation; a naive floor(a/90) disagrees."""
    assert _quadrant(0.0) == _quadrant(359.0) == _quadrant(1.0)
    assert _quadrant(90.0) != _quadrant(0.0)
    assert _quadrant(180.0) != _quadrant(90.0)


def test_a_single_ninety_scores_once():
    axis = AxisStyle(2.0)
    assert _walk(axis, [0.0, 90.0]) == 2.0
    assert axis.changes == 1


def test_rolling_there_and_back_scores_only_the_first():
    """The handbook's own example, verbatim: 90 then back to 0 gets one."""
    axis = AxisStyle(2.0)
    score = _walk(axis, [0.0, 90.0, 0.0])
    assert score == 2.0, 'the return scored, but the rulebook says it must not'
    assert axis.rejected_returns == 1


def test_a_full_roll_scores_every_quarter():
    """0 -> 90 -> 180 -> 270 -> 0 never revisits the PREVIOUS orientation."""
    axis = AxisStyle(2.0)
    assert _walk(axis, [0.0, 90.0, 180.0, 270.0, 0.0]) == 8.0
    assert axis.changes == 4
    assert axis.rejected_returns == 0


def test_oscillating_cannot_farm_points():
    """The failure this rule exists to prevent."""
    axis = AxisStyle(2.0)
    score = _walk(axis, [0.0, 90.0] * 20)
    assert score == 2.0, f'oscillation farmed {score} points'


def test_roll_and_pitch_outscore_yaw():
    """"Roll and Pitch are worth more than Yaw." """
    from duburi_sim_bridge.scoring import (
        STYLE_POINTS_PITCH, STYLE_POINTS_ROLL, STYLE_POINTS_YAW,
    )
    assert STYLE_POINTS_ROLL > STYLE_POINTS_YAW
    assert STYLE_POINTS_PITCH > STYLE_POINTS_YAW


def test_hovering_on_a_boundary_does_not_ratchet():
    """A vehicle sitting near 45 deg must not score on sensor noise."""
    axis = AxisStyle(2.0)
    axis.update(0.0)
    for a in (44.0, 46.0) * 25:
        axis.update(a)
    assert axis.score == 0.0, f'boundary noise scored {axis.score}'


def test_partial_rotation_does_not_score():
    axis = AxisStyle(2.0)
    assert _walk(axis, [0.0, 30.0, 60.0, 30.0, 0.0]) == 0.0
