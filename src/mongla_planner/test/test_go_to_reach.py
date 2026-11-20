"""`go_to` says, before any thrust, when a leg is longer than the estimate can fly.

The filter's position sigma grows with distance since the last fix (B-136),
and the verb stops LOST when it passes the tolerance. A long leg therefore
starts, and stops partway. The DSL warns up front; the verb still decides.
Also `go_to_prop`, which no test had called.
"""
import math
from types import SimpleNamespace as NS
from unittest.mock import MagicMock

import pytest

from mongla_localization.course_map import Course, Prop
from mongla_localization.inekf import FLOW_SCALE_SIGMA
from mongla_planner.dsl_position import reach_m
from mongla_planner.mongla_dsl import MonglaMission


def _state(x=0.0, y=0.0, sigma=0.15, usable=True):
    return NS(x_m=x, y_m=y, sigma_xy_m=sigma, usable=usable, reason='')


def _mission(state):
    m = MagicMock()
    m.localization = lambda timeout=1.0: state
    m.log = MagicMock()
    m.go_to = lambda *a, **k: MonglaMission.go_to(m, *a, **k)
    return m


def _warned(m):
    return [c.args[0] for c in m.log.warning.call_args_list]


def test_the_reach_from_a_fix_at_the_default_tolerance():
    assert reach_m(0.15, 0.5, FLOW_SCALE_SIGMA) == pytest.approx(13.25, abs=0.01)
    assert reach_m(0.15, 0.3, FLOW_SCALE_SIGMA) == pytest.approx(7.22, abs=0.01)
    assert reach_m(0.6, 0.5, FLOW_SCALE_SIGMA) == 0.0
    assert math.isnan(reach_m(float('nan'), 0.5, FLOW_SCALE_SIGMA))


@pytest.mark.parametrize('leg, warned', [(10.0, False), (16.0, True)])
def test_a_leg_past_the_reach_is_warned_and_still_sent(leg, warned):
    m = _mission(_state())
    MonglaMission.go_to(m, leg, 0.0)
    assert any('LOST' in w for w in _warned(m)) is warned
    m._send.assert_called_once()
    assert m._send.call_args.kwargs['x_m'] == leg


def test_no_estimate_no_warning_the_verb_refuses_it():
    for state in (None, _state(usable=False)):
        m = _mission(state)
        MonglaMission.go_to(m, 30.0, 0.0)
        assert _warned(m) == []
        m._send.assert_called_once()


def test_go_to_prop_drives_to_the_standoff_along_the_line():
    m = _mission(_state(x=0.0, y=0.0))
    m._course = Course(name='t', source='test', props={
        'gate': Prop(name='gate', x_m=6.0, y_m=8.0)})
    MonglaMission.go_to_prop(m, 'gate', standoff=1.0)
    kw = m._send.call_args.kwargs
    assert math.hypot(kw['x_m'], kw['y_m']) == pytest.approx(9.0)
    assert kw['x_m'] / kw['y_m'] == pytest.approx(6.0 / 8.0)


def test_go_to_prop_refuses_without_a_course_a_position_or_an_estimate():
    m = _mission(_state())
    m._course = None
    assert MonglaMission.go_to_prop(m, 'gate') is None
    m._course = Course(name='t', source='test', props={'gate': Prop(name='gate')})
    assert MonglaMission.go_to_prop(m, 'gate') is None
    m = _mission(_state(usable=False))
    m._course = Course(name='t', source='test', props={
        'gate': Prop(name='gate', x_m=1.0, y_m=0.0)})
    assert MonglaMission.go_to_prop(m, 'gate') is None
    m._send.assert_not_called()
