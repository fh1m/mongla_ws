"""A pool fix must refuse a frame mix, not silently produce a rotated answer.

The bearings come from the camera; the prop positions come from the course
file. Those live in the same frame only once a landmark has anchored the
heading. Without that the fix is in a boot-relative frame that looks exactly
like a good one.
"""
import math

from unittest.mock import MagicMock

import pytest

from mongla_localization.course_map import Course, Prop
from mongla_planner.mongla_dsl import MonglaMission


def _course(**placed):
    props = {}
    for name, xy in placed.items():
        props[name] = Prop(name, x_m=xy[0], y_m=xy[1], detect_class=name,
                           measured=True)
    return Course('t', props)


def _fake(course=None, offset=150.0, heading=0.0, seen=None):
    m = MagicMock()
    m._course = course
    m._heading_offset = offset
    m.HFOV_WATER_DEG_BY_CAMERA = MonglaMission.HFOV_WATER_DEG_BY_CAMERA
    m.absolute_heading.return_value = heading
    seen = seen or {}
    m.where_offset.side_effect = lambda cls, camera=None: seen.get(cls)
    m.bearing_to.side_effect = lambda cls, camera=None: (
        None if seen.get(cls) is None
        else (heading + seen[cls] * MonglaMission.HFOV_WATER_DEG_BY_CAMERA['forward'] / 2.0) % 360.0)
    return m


def test_no_course_is_refused_by_name():
    got = MonglaMission.fix_position(_fake(course=None))
    assert not got.ok and 'use_course' in got.reason


def test_an_unanchored_heading_is_refused_because_the_frames_would_mix():
    # THE LOAD-BEARING ONE. Bearings relative to boot, positions in pool terms:
    # the solve succeeds and the answer is in a rotated frame.
    fake = _fake(course=_course(gate=(0.0, 0.0), slalom=(8.0, 2.0)), offset=None)
    got = MonglaMission.fix_position(fake)
    assert not got.ok and 'rotated frame' in got.reason


def test_a_prop_that_is_not_visible_contributes_nothing():
    fake = _fake(course=_course(gate=(0.0, 0.0), slalom=(8.0, 2.0)),
                 seen={'gate': 0.0})            # slalom unseen
    got = MonglaMission.fix_position(fake)
    assert not got.ok and 'need 2' in got.reason


def test_a_prop_without_a_measured_position_contributes_nothing():
    course = Course('t', {'gate': Prop('gate', x_m=0.0, y_m=0.0,
                                       detect_class='gate', measured=True),
                          'bin': Prop('bin', detect_class='bin')})
    fake = _fake(course=course, seen={'gate': 0.0, 'bin': 0.5})
    got = MonglaMission.fix_position(fake)
    assert not got.ok, 'bin has no position, so this is one usable sighting'


def test_two_visible_measured_props_produce_a_fix():
    # Observer at (4, -2); bearings computed from geometry so the fix is scored
    # against a known truth rather than against another estimator.
    props = {'gate': (0.0, 0.0), 'slalom': (8.0, 2.0)}
    truth = (4.0, -2.0)

    def bearing(to):
        d = math.degrees(math.atan2(to[1] - truth[1], to[0] - truth[0]))
        return d + 360.0 if d < 0.0 else d

    fake = _fake(course=_course(**props))
    fake.bearing_to.side_effect = lambda cls, camera=None: bearing(props[cls])
    got = MonglaMission.fix_position(fake)
    assert got.ok
    assert got.x_m == pytest.approx(truth[0], abs=1e-6)
    assert got.y_m == pytest.approx(truth[1], abs=1e-6)


def test_the_props_argument_restricts_which_are_used():
    props = {'gate': (0.0, 0.0), 'slalom': (8.0, 2.0), 'bin': (6.0, -5.0)}
    fake = _fake(course=_course(**props))
    fake.bearing_to.side_effect = lambda cls, camera=None: 0.0
    MonglaMission.fix_position(fake, props=['gate', 'slalom'])
    asked = [c.args[0] for c in fake.bearing_to.call_args_list]
    assert 'bin' not in asked


def test_bearing_to_returns_none_for_an_unseen_prop():
    fake = _fake(seen={})
    fake.bearing_to = MonglaMission.bearing_to.__get__(fake)
    assert fake.bearing_to('gate') is None


def test_bearing_to_uses_the_WATER_field_of_view_of_THAT_camera():
    # Water, not air (air would stretch every bearing by ~37 %), and per camera:
    # forward is the Fantech at 53.6 deg, downward the global shutter at 46.7.
    fake = _fake(seen={'gate': 1.0}, heading=0.0)
    fake.bearing_to = MonglaMission.bearing_to.__get__(fake)
    fake.where_offset.side_effect = lambda cls, camera=None: 1.0
    assert fake.bearing_to('gate', camera='forward') == pytest.approx(53.6 / 2.0)
    assert fake.bearing_to('gate', camera='downward') == pytest.approx(46.7 / 2.0)


# --------------------------------------------------------------------------- #
#  the single-prop fix: EXECUTE the success path, do not grep for it
# --------------------------------------------------------------------------- #
def test_the_success_Fix_can_actually_be_constructed():
    """`fix_from_prop` ends in a five-kwarg `Fix(...)` that no test had ever
    run. A grep for the function name passes whether or not that line raises,
    which is the grep-versus-execute gap this register already records.
    """
    from mongla_localization.resection import Fix

    got = Fix(True, x_m=1.25, y_m=-0.5, used=1, residual_m=0.0,
              separation_deg=0.0)
    assert got.ok is True
    assert got.x_m == 1.25
    assert got.y_m == -0.5
    assert got.used == 1


def test_a_refusal_Fix_carries_its_reason():
    from mongla_localization.resection import Fix

    got = Fix(False, reason='not visible')
    assert got.ok is False
    assert 'not visible' in got.reason
