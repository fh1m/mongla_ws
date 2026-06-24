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
def test_detected_is_case_insensitive(monkeypatch):
    import time as _t
    import duburi_planner.duburi_dsl as dd

    # detected() spins the node once; stub it out (no ROS in unit tests).
    monkeypatch.setattr(dd.rclpy, 'spin_once', lambda *a, **k: None)
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    # Pretend we're subscribed and the detector just published 'Gate'.
    m._det_subs['forward'] = object()
    m._det_cache['forward'] = (_t.monotonic(), {'Gate'})

    assert m.detected('gate') is True      # case-insensitive match
    assert m.detected('GATE') is True
    assert m.detected('flare') is False


def test_move_passes_fill_and_mode_to_server():
    send = MagicMock(return_value=_result(ALIGNED, 0.9))
    dsl = _dsl(send)
    dsl.move('red_pipe', fwd=60, mode='height')
    _, kwargs = send.call_args
    assert kwargs['fwd_fill'] == pytest.approx(60.0)
    assert kwargs['mode'] == 'height'
