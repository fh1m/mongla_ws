"""The venue's floor comes from the course a mission names, not a launch line.

Optical flow turns pixels into metres with the height above the floor, and
without a DVL or a trusted barometer that height is the pool's depth minus
the vehicle's. The depth is a fact about the venue, so it lives in the course
file beside the props, and `use_course()` hands it to flow while the mission
runs. Each test builds the case where getting it wrong would be silent.
"""
import math
from unittest.mock import MagicMock

import pytest

from mongla_localization import course_map
from mongla_localization.course_map import load_course
from mongla_planner.mongla_dsl import MonglaMission


def _course_file(tmp_path, monkeypatch, pool: str):
    (tmp_path / 'venue.yaml').write_text(
        'course: venue\nprops:\n  gate:\n    detect_class: gate\n' + pool)
    monkeypatch.setenv(course_map._ENV, str(tmp_path))
    return load_course('venue')


def test_a_course_states_its_floor(tmp_path, monkeypatch):
    c = _course_file(tmp_path, monkeypatch,
                     'pool:\n  depth_m: 1.6\n  depth_min_m: 1.2\n  depth_max_m: 1.6\n')
    assert c.pool_depth() == (1.6, 1.2, 1.6)


def test_an_unset_floor_is_None_never_zero(tmp_path, monkeypatch):
    """A zero depth would put the floor at the surface and every speed at 0."""
    c = _course_file(tmp_path, monkeypatch, 'pool:\n  depth_m:\n')
    assert c.pool_depth() is None
    c = _course_file(tmp_path, monkeypatch, '')
    assert c.pool_depth() is None


def test_a_missing_range_is_NaN_not_the_depth(tmp_path, monkeypatch):
    """Flow sizes its uncertainty from the range; inventing one as the depth
    itself would claim a perfectly flat floor nobody measured."""
    c = _course_file(tmp_path, monkeypatch, 'pool:\n  depth_m: 2.0\n')
    d, lo, hi = c.pool_depth()
    assert d == 2.0 and math.isnan(lo) and math.isnan(hi)


def _mission():
    m = MagicMock()
    m.log = MagicMock()
    return m


def test_use_course_gives_flow_the_floor(tmp_path, monkeypatch):
    _course_file(tmp_path, monkeypatch,
                 'pool:\n  depth_m: 1.6\n  depth_min_m: 1.2\n  depth_max_m: 1.6\n')
    m = _mission()
    m._course_floor_to_flow = lambda c: MonglaMission._course_floor_to_flow(m, c)
    MonglaMission.use_course(m, 'venue')
    m.set_node.assert_called_once_with('flow', pool_depth_m=1.6,
                                       pool_depth_min_m=1.2, pool_depth_max_m=1.6)


def test_a_course_without_a_floor_leaves_flow_alone_and_says_so(tmp_path, monkeypatch):
    _course_file(tmp_path, monkeypatch, '')
    m = _mission()
    m._course_floor_to_flow = lambda c: MonglaMission._course_floor_to_flow(m, c)
    MonglaMission.use_course(m, 'venue')
    m.set_node.assert_not_called()
    assert any('no pool depth' in str(c) for c in m.log.warning.call_args_list)


def test_flow_down_is_a_warning_not_a_crash(tmp_path, monkeypatch):
    """The mission still has its course; the operator must hear that the
    floor never reached the velocity."""
    _course_file(tmp_path, monkeypatch, 'pool:\n  depth_m: 1.6\n')
    m = _mission()
    m.set_node.side_effect = RuntimeError('mongla_flow_velocity not running')
    m._course_floor_to_flow = lambda c: MonglaMission._course_floor_to_flow(m, c)
    MonglaMission.use_course(m, 'venue')
    assert any('NOT given to flow' in str(c) for c in m.log.warning.call_args_list)


def test_flow_takes_the_floor_live_and_refuses_a_bad_one():
    pytest.importorskip('rclpy')
    import rclpy
    from rclpy.parameter import Parameter
    from mongla_vision.flow.flow_node import FlowVelocityNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    try:
        n = FlowVelocityNode(parameter_overrides=[
            Parameter('pool_depth_m', value=1.0), Parameter('medium', value='air')])
        try:
            assert 'pool_depth_m' in n.LIVE_PARAMS
            ok = n.set_parameters([Parameter('pool_depth_m', value=1.6),
                                   Parameter('pool_depth_min_m', value=1.2),
                                   Parameter('pool_depth_max_m', value=1.6)])
            assert all(r.successful for r in ok)
            assert n._pool_depth == 1.6
            assert n._floor_sigma_m == pytest.approx(
                math.sqrt(((-0.4) ** 2 + (-0.4 * 0.0) + 0.0) / 3.0))
            bad = n.set_parameters([Parameter('pool_depth_m', value=0.0)])
            assert not bad[0].successful
            assert n._pool_depth == 1.6, 'a refused depth must not land'
        finally:
            n.destroy_node()
    finally:
        if started:
            rclpy.shutdown()
