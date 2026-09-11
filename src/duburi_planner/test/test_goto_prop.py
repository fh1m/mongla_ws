"""A waypoint must stop short, and refuse everything it would otherwise guess.

A prior map is a rulebook and a tape measure. It can put the vehicle in
detection range and no closer, so the verb aims at a standoff and hands over.
Each precondition is refused by name rather than defaulted, and there is
deliberately no timed fallback: that fallback once drove 2.361 m for a 1.0 m
command and reported success.
"""
import math

from unittest.mock import MagicMock

import pytest

from duburi_planner.course_map import Course, Prop
from duburi_planner.duburi_dsl import DuburiMission
from duburi_vision.resection import Fix


def _course(**placed):
    return Course('t', {n: Prop(n, x_m=xy[0], y_m=xy[1], detect_class=n,
                                measured=True) for n, xy in placed.items()})


def _fake(course=None, fix=None, moved=True):
    m = MagicMock()
    m._course = course
    m.fix_position.return_value = fix if fix is not None else Fix(
        True, x_m=0.0, y_m=0.0, used=2, residual_m=0.0, separation_deg=60.0)
    m.absolute_to_relative.side_effect = lambda b: b
    m.move_forward_dist.return_value = moved
    return m


def test_it_turns_to_the_bearing_and_runs_to_the_standoff():
    fake = _fake(course=_course(gate=(10.0, 0.0)))
    got = DuburiMission.goto_prop(fake, 'gate', standoff_m=2.0)
    assert got.ok and got.branch == 'dead_reckon'
    assert fake.turn.call_args.args[0] == pytest.approx(0.0)
    assert fake.move_forward_dist.call_args.args[0] == pytest.approx(8.0)


def test_the_bearing_is_taken_from_the_fix_not_from_the_origin():
    # Standing at (4, 4), the prop at (10, 4) is due +x, not 45 degrees.
    fake = _fake(course=_course(gate=(10.0, 4.0)),
                 fix=Fix(True, x_m=4.0, y_m=4.0, used=2, separation_deg=60.0))
    DuburiMission.goto_prop(fake, 'gate', standoff_m=1.0)
    assert fake.turn.call_args.args[0] == pytest.approx(0.0)
    assert fake.move_forward_dist.call_args.args[0] == pytest.approx(5.0)


def test_already_inside_the_standoff_does_not_move():
    fake = _fake(course=_course(gate=(1.0, 0.0)))
    got = DuburiMission.goto_prop(fake, 'gate', standoff_m=2.0)
    assert got.ok and got.branch == 'already_there'
    fake.move_forward_dist.assert_not_called()
    fake.turn.assert_not_called()


def test_no_course_is_refused():
    got = DuburiMission.goto_prop(_fake(course=None), 'gate')
    assert not got.ok and 'use_course' in got.error


def test_an_unmeasured_prop_is_refused_not_reckoned_to_the_origin():
    course = Course('t', {'gate': Prop('gate')})
    fake = _fake(course=course)
    got = DuburiMission.goto_prop(fake, 'gate')
    assert not got.ok and 'unset' in got.error
    fake.move_forward_dist.assert_not_called()


def test_no_fix_means_nothing_to_reckon_from():
    fake = _fake(course=_course(gate=(10.0, 0.0)),
                 fix=Fix(False, reason='heading is not anchored'))
    got = DuburiMission.goto_prop(fake, 'gate')
    assert not got.ok and 'no position fix' in got.error
    fake.move_forward_dist.assert_not_called()


def test_an_over_long_leg_is_refused():
    # Dead reckoning that far accumulates more error than the standoff it aims
    # for, so the honest move is to search instead.
    fake = _fake(course=_course(gate=(40.0, 0.0)))
    got = DuburiMission.goto_prop(fake, 'gate', max_leg_m=12.0)
    assert not got.ok and 'exceeds max_leg_m' in got.error
    fake.move_forward_dist.assert_not_called()


def test_a_failed_distance_leg_is_reported_not_assumed():
    # THE LOAD-BEARING ONE. There is no timed fallback: a mission that believes
    # it arrived is worse off than one told to search.
    fake = _fake(course=_course(gate=(10.0, 0.0)), moved=False)
    got = DuburiMission.goto_prop(fake, 'gate')
    assert not got.ok
    assert fake.log.warning.called


def test_the_turn_goes_through_the_relative_conversion():
    # turn() speaks the hull's frame; the bearing is a world one.
    fake = _fake(course=_course(gate=(0.0, 10.0)))
    DuburiMission.goto_prop(fake, 'gate')
    fake.absolute_to_relative.assert_called_once()
    assert fake.absolute_to_relative.call_args.args[0] == pytest.approx(90.0)
