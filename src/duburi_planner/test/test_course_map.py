"""A course prior must refuse an unmeasured number, never default it.

Every value in a course file is a claim about a pool. A missing position that
silently became 0.0 would dead-reckon the vehicle to the pool origin with total
confidence, which is this stack's most expensive recurring defect wearing a new
hat. So the tests are mostly about refusals and about the shipped file being
honestly empty.
"""
import math
import pathlib

import pytest

from duburi_planner.course_map import (
    Course, Prop, bearing_to, load_course, range_to,
)


def _course():
    return Course('t', {
        'gate': Prop('gate', x_m=4.0, y_m=1.0, bearing_deg=90.0, measured=True),
        'bin':  Prop('bin'),                       # nothing measured
    })


def test_a_measured_prop_answers():
    assert _course().position_of('gate') == (4.0, 1.0)
    assert _course().bearing_of('gate') == pytest.approx(90.0)


def test_an_unset_position_refuses_instead_of_returning_zero():
    with pytest.raises(ValueError) as exc:
        _course().position_of('bin')
    assert 'unset' in str(exc.value)


def test_an_unset_bearing_refuses_because_a_guess_writes_a_wrong_zero():
    with pytest.raises(ValueError) as exc:
        _course().bearing_of('bin')
    assert 'wrong heading zero' in str(exc.value)


def test_an_unknown_prop_names_what_it_does_know():
    with pytest.raises(KeyError) as exc:
        _course().prop('torpedo')
    assert 'gate' in str(exc.value)


def test_unmeasured_props_are_listed_for_the_operator():
    assert _course().unmeasured() == ['bin']


# --- the frame convention, stated once and checked -------------------------


def test_plus_x_is_bearing_zero_and_plus_y_is_ninety():
    assert bearing_to((0.0, 0.0), (1.0, 0.0)) == pytest.approx(0.0)
    assert bearing_to((0.0, 0.0), (0.0, 1.0)) == pytest.approx(90.0)
    assert bearing_to((0.0, 0.0), (-1.0, 0.0)) == pytest.approx(180.0)
    assert bearing_to((0.0, 0.0), (0.0, -1.0)) == pytest.approx(270.0)


def test_a_bearing_is_a_compass_bearing():
    for to in ((1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)):
        assert 0.0 <= bearing_to((0.0, 0.0), to) < 360.0


def test_range_is_euclidean():
    assert range_to((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)


# --- the shipped file -------------------------------------------------------


def test_the_shipped_course_loads():
    c = load_course('robosub26')
    assert 'gate' in c.props and 'torpedo' in c.props


def test_the_shipped_course_ships_NO_invented_coordinates():
    # THE LOAD-BEARING ONE. We have not measured a RoboSub pool. A file with
    # plausible coordinates would send the vehicle somewhere nobody measured.
    c = load_course('robosub26')
    placed = [n for n, p in c.props.items() if p.has_position]
    assert not placed, (
        f'these props carry coordinates nobody measured: {placed}. '
        f'A course prior must ship empty and be filled at the venue.')


def test_every_shipped_prop_is_marked_unmeasured():
    c = load_course('robosub26')
    assert c.unmeasured() == sorted(c.props)


def test_a_missing_course_says_where_it_looked():
    with pytest.raises(FileNotFoundError) as exc:
        load_course('no_such_course')
    assert 'Looked in' in str(exc.value)


def test_an_unset_value_never_becomes_zero():
    from duburi_planner.course_map import _opt_float
    assert _opt_float(None) is None
    assert _opt_float('') is None
    assert _opt_float(float('nan')) is None
    assert _opt_float(0) == 0.0          # a real zero survives


# --- the DSL surface --------------------------------------------------------
#
# One number, one place: a prop's face bearing is what an absolute heading needs
# AND what approaching it from the front needs. Passing it by hand at each call
# site is how the two come to disagree.

from unittest.mock import MagicMock                                  # noqa: E402

from duburi_planner.duburi_dsl import DuburiMission                  # noqa: E402


def test_anchor_on_takes_the_bearing_from_the_loaded_course():
    fake = MagicMock()
    fake._course = Course('t', {'gate': Prop('gate', bearing_deg=270.0,
                                             measured=True)})
    DuburiMission.anchor_on(fake, 'gate')
    assert fake.anchor_heading.call_args.kwargs['bearing_deg'] == pytest.approx(270.0)


def test_anchor_on_without_a_course_says_which_call_is_missing():
    fake = MagicMock()
    fake._course = None
    with pytest.raises(RuntimeError) as exc:
        DuburiMission.anchor_on(fake, 'gate')
    assert 'use_course' in str(exc.value)


def test_anchor_on_refuses_an_unmeasured_bearing_rather_than_guessing():
    fake = MagicMock()
    fake._course = Course('t', {'gate': Prop('gate')})
    with pytest.raises(ValueError):
        DuburiMission.anchor_on(fake, 'gate')
    fake.anchor_heading.assert_not_called()
