"""Orchestration tests for the two-verb vision DSL (align / move).

These pin the recover-don't-fail contract:
  * neither verb raises on a miss -- TIMEOUT / LOST return VisionResult(ok=False);
  * ALIGNED returns ok=True;
  * on LOST with a fallback, the mission-authored fallback runs once and the
    loop re-enters vision within the same duration budget;
  * a 2-arg fallback receives a `should_stop` predicate.

The server action is mocked via a fake `_send`, so no ROS / MAVLink is needed.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from duburi_planner.vision_dsl import _VisionDSL, VisionResult
from duburi_planner.client import MoveFailed
from duburi_control.motion_vision import ALIGNED, LOST, TIMEOUT, NO_CAMERA


def _result(code, err=0.0):
    return SimpleNamespace(final_value=float(code), error_value=float(err),
                           success=True, message=str(code))


def _mission(send):
    """Fake DuburiMission exposing only what _VisionDSL touches."""
    m = MagicMock()
    m.camera = 'forward'
    m.target = 'gate'
    m._send = send
    m.detected.return_value = False
    m.log = MagicMock()
    return m


def _dsl(send):
    return _VisionDSL(_mission(send))


# --------------------------------------------------------------------------- #
#  align                                                                       #
# --------------------------------------------------------------------------- #
def test_align_ok_on_aligned():
    dsl = _dsl(MagicMock(return_value=_result(ALIGNED, 8.0)))
    res = dsl.align('gate', yaw=0, lat=0)
    assert isinstance(res, VisionResult)
    assert res.ok is True
    assert res.reason == 'ALIGNED'


def test_align_never_raises_on_timeout():
    dsl = _dsl(MagicMock(return_value=_result(TIMEOUT, 120.0)))
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert res.ok is False
    assert res.reason == 'TIMEOUT'


def test_align_no_camera_returns_false():
    dsl = _dsl(MagicMock(return_value=_result(NO_CAMERA)))
    res = dsl.align('gate', yaw=0)
    assert res.ok is False
    assert res.reason == 'NO_CAMERA'


def test_align_requires_an_axis():
    dsl = _dsl(MagicMock(return_value=_result(ALIGNED)))
    with pytest.raises(ValueError):
        dsl.align('gate')          # no lat/yaw/depth -> programming error


def test_align_runs_fallback_once_then_reenters():
    # First server call reports LOST, fallback runs, second call ALIGNED.
    send = MagicMock(side_effect=[_result(LOST, 50.0), _result(ALIGNED, 5.0)])
    dsl = _dsl(send)
    calls = []

    def creep(_duburi):
        calls.append('creep')

    res = dsl.align('gate', yaw=0, lat=0, fallback=creep, duration=10)
    assert res.ok is True
    assert calls == ['creep']       # fallback ran exactly once
    assert send.call_count == 2     # re-entered the vision loop after fallback


def test_align_lost_without_fallback_returns_false():
    dsl = _dsl(MagicMock(return_value=_result(LOST, 50.0)))
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert res.ok is False
    assert res.reason == 'LOST'


def test_align_never_dies_on_server_failure():
    # A server-side failure (bad camera, ALT_HOLD rejected, disarmed, abort)
    # raises MoveFailed inside _send. The verb MUST catch it and return a
    # non-fatal VisionResult so the mission moves on -- never propagate.
    dsl = _dsl(MagicMock(side_effect=MoveFailed(
        'Goal "vision_align" FAILED: ALT_HOLD rejected')))
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert isinstance(res, VisionResult)
    assert res.ok is False
    assert res.reason == 'FAILED'


def test_align_never_dies_on_unexpected_exception():
    # Even a totally unexpected error must not escape a vision verb.
    dsl = _dsl(MagicMock(side_effect=RuntimeError('rclpy hiccup')))
    res = dsl.align('gate', yaw=0, duration=0.5)
    assert res.ok is False
    assert res.reason == 'FAILED'


def test_align_two_arg_fallback_gets_should_stop():
    send = MagicMock(side_effect=[_result(LOST), _result(ALIGNED)])
    dsl = _dsl(send)
    seen = {}

    def sweep(_duburi, should_stop):
        seen['callable'] = callable(should_stop)

    dsl.align('gate', yaw=0, fallback=sweep, duration=10)
    assert seen.get('callable') is True


def test_align_fallback_exception_does_not_propagate():
    send = MagicMock(side_effect=[_result(LOST), _result(ALIGNED)])
    dsl = _dsl(send)

    def bad(_duburi):
        raise RuntimeError('search blew up')

    res = dsl.align('gate', yaw=0, fallback=bad, duration=10)
    assert res.ok is True           # bad fallback swallowed, loop re-entered


# --------------------------------------------------------------------------- #
#  move                                                                        #
# --------------------------------------------------------------------------- #
def test_move_ok_on_reach():
    dsl = _dsl(MagicMock(return_value=_result(ALIGNED, 0.85)))
    res = dsl.move('gate', fwd=80)
    assert res.ok is True
    assert res.fill == pytest.approx(0.85)


def test_move_never_raises_on_timeout():
    dsl = _dsl(MagicMock(return_value=_result(TIMEOUT, 0.4)))
    res = dsl.move('gate', fwd=80, duration=0.5)
    assert res.ok is False


def test_move_never_dies_on_server_failure():
    dsl = _dsl(MagicMock(side_effect=MoveFailed('boom')))
    res = dsl.move('gate', fwd=80, duration=0.5)
    assert res.ok is False
    assert res.reason == 'FAILED'


# --------------------------------------------------------------------------- #
#  detected() case-insensitive (control path lowercases both sides)           #
# --------------------------------------------------------------------------- #
def _rec(cls, cx=320.0, cy=240.0, w=40.0, h=40.0, conf=0.9):
    """Build a DetRecord tuple: (class_lower, cx, cy, w, h, conf)."""
    return (cls, cx, cy, w, h, conf)


def test_detected_is_case_insensitive(monkeypatch):
    import time as _t
    import duburi_planner.duburi_dsl as dd

    # detected() pumps the node; stub the pump out (no ROS in unit tests).
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    monkeypatch.setattr(m, '_pump_detections', lambda *a, **k: None)
    # Pretend we're subscribed and the detector just published 'gate'.
    m._det_subs['forward'] = object()
    m._det_cache['forward'] = (_t.monotonic(), [_rec('gate')])

    assert m.detected('gate') is True      # case-insensitive match
    assert m.detected('GATE') is True
    assert m.detected('flare') is False


# --------------------------------------------------------------------------- #
#  Pure cores: _eval_detected / _eval_where (no ROS)                          #
# --------------------------------------------------------------------------- #
def test_eval_detected_present_absent_caseinsensitive():
    import duburi_planner.duburi_dsl as dd
    recs = [_rec('gate'), _rec('flare')]
    assert dd._eval_detected(recs, 'gate') is True
    assert dd._eval_detected(recs, 'GATE') is True       # case-insensitive
    assert dd._eval_detected(recs, ' Flare ') is True    # trimmed
    assert dd._eval_detected(recs, 'hole') is False
    assert dd._eval_detected([], 'gate') is False


def test_eval_where_left_center_right():
    import duburi_planner.duburi_dsl as dd
    W = 640.0
    # cx=64 -> offset -0.8 -> left; cx=320 -> 0 -> center; cx=576 -> +0.8 -> right
    assert dd._eval_where([_rec('gate', cx=64.0)],  'gate', W, 0.15)[0] == 'left'
    assert dd._eval_where([_rec('gate', cx=320.0)], 'gate', W, 0.15)[0] == 'center'
    assert dd._eval_where([_rec('gate', cx=576.0)], 'gate', W, 0.15)[0] == 'right'


def test_eval_where_largest_area_wins():
    import duburi_planner.duburi_dsl as dd
    # Two gates: a tiny one on the left, a big one on the right -> right wins.
    recs = [_rec('gate', cx=50.0,  w=10.0, h=10.0),
            _rec('gate', cx=600.0, w=200.0, h=200.0)]
    label, offset = dd._eval_where(recs, 'gate', 640.0, 0.15)
    assert label == 'right'
    assert offset > 0.0


def test_eval_where_unknown_when_no_width_or_absent():
    import duburi_planner.duburi_dsl as dd
    assert dd._eval_where([_rec('gate')], 'gate', 0.0, 0.15) == ('unknown', None)
    assert dd._eval_where([], 'gate', 640.0, 0.15) == ('unknown', None)
    assert dd._eval_where([_rec('flare')], 'gate', 640.0, 0.15) == ('unknown', None)


def test_wait_for_returns_true_when_target_appears(monkeypatch):
    import time as _t
    import duburi_planner.duburi_dsl as dd
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    monkeypatch.setattr(m, '_pump_detections', lambda *a, **k: None)
    # Target absent for the first 2 polls, then appears.
    state = {'n': 0}
    def fake_records(cam, stale):
        state['n'] += 1
        return [_rec('gate')] if state['n'] >= 3 else []
    monkeypatch.setattr(m, '_records', fake_records)
    assert m.wait_for('gate', timeout=5.0) is True


def test_wait_for_returns_false_on_timeout(monkeypatch):
    import duburi_planner.duburi_dsl as dd
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    monkeypatch.setattr(m, '_pump_detections', lambda *a, **k: None)
    monkeypatch.setattr(m, '_records', lambda *a, **k: [])  # never appears
    assert m.wait_for('gate', timeout=0.2) is False


def test_where_returns_label_from_cache(monkeypatch):
    import time as _t
    import duburi_planner.duburi_dsl as dd
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    monkeypatch.setattr(m, '_pump_detections', lambda *a, **k: None)
    m._img_size['forward'] = (640.0, 480.0)
    m._det_cache['forward'] = (_t.monotonic(), [_rec('gate', cx=64.0)])
    assert m.where('gate') == 'left'
    assert m.where_offset('gate') < 0.0
    assert m.where('hole') == 'unknown'


def test_detector_node_derivation():
    # Single naming rule: /duburi_detector_<camera>, camera defaults to the
    # mission's sticky camera; explicit node always wins.
    import duburi_planner.duburi_dsl as dd
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    assert m._detector_node() == '/duburi_detector_forward'
    assert m._detector_node('downward') == '/duburi_detector_downward'
    assert m._detector_node(node='/duburi_detector_custom') == '/duburi_detector_custom'


def test_move_passes_fill_and_mode_to_server():
    send = MagicMock(return_value=_result(ALIGNED, 0.9))
    dsl = _dsl(send)
    dsl.move('red_pipe', fwd=60, mode='height')
    _, kwargs = send.call_args
    assert kwargs['fwd_fill'] == pytest.approx(60.0)
    assert kwargs['mode'] == 'height'


def test_move_passthrough_sends_negative_sentinel():
    # move('gate') with no fwd = pass-through. The DSL must wire fwd_fill=-1
    # (a value that survives the manager's 0.0==unset rule) so the control
    # loop selects pass-through instead of the 95% spec default.
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.move('gate')
    _, kwargs = send.call_args
    assert kwargs['fwd_fill'] < 0.0


def test_move_explicit_fill_is_not_sentinel():
    send = MagicMock(return_value=_result(ALIGNED, 0.8))
    dsl = _dsl(send)
    dsl.move('gate', fwd=80)
    _, kwargs = send.call_args
    assert kwargs['fwd_fill'] == pytest.approx(80.0)


# --------------------------------------------------------------------------- #
#  align -- 0-offset centring (the pool-day "yaw=0, lat=0" case)              #
# --------------------------------------------------------------------------- #
def test_align_zero_offsets_build_axes_and_center():
    # align('gate', yaw=0, lat=0) must activate both axes (0 = centre) and
    # send offset_*=0.0 -- never silently drop a zero-valued axis.
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    res = dsl.align('gate', yaw=0, lat=0)
    assert res.ok is True
    _, kwargs = send.call_args
    assert set(kwargs['axes'].split(',')) == {'lat', 'yaw'}
    assert kwargs['offset_lat'] == pytest.approx(0.0)
    assert kwargs['offset_yaw'] == pytest.approx(0.0)


def test_align_single_zero_axis_only_that_axis():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('gate', yaw=0)
    _, kwargs = send.call_args
    assert kwargs['axes'] == 'yaw'
    assert kwargs['offset_yaw'] == pytest.approx(0.0)


def test_align_nonzero_offset_passed_through():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('gate', lat=64, yaw=0)
    _, kwargs = send.call_args
    assert kwargs['offset_lat'] == pytest.approx(64.0)
    assert kwargs['offset_yaw'] == pytest.approx(0.0)
