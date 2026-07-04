"""DuburiClient bounded-deadline tests — a wedged/dead action server must never
hang the caller (or the mission's disarm backstop) forever.

rclpy spin + the ActionClient are mocked, so no ROS/MAVLink is needed.
"""

import pytest
from unittest.mock import MagicMock

from duburi_planner.client import (
    DuburiClient, MoveFailed, MoveRejected, MoveTimeout,
    _QUICK_DEADLINE_S, _RESULT_TIMEOUT_FLOOR_S, _RESULT_TIMEOUT_MARGIN_S,
)


class _Future:
    def __init__(self, done=True, result=None):
        self._done = done
        self._result = result

    def done(self):
        return self._done

    def result(self):
        return self._result


def _client(monkeypatch):
    monkeypatch.setattr('duburi_planner.client.ActionClient',
                        lambda *a, **k: MagicMock())
    # spin is a no-op -> futures keep whatever done-state the test set.
    monkeypatch.setattr('duburi_planner.client.rclpy.spin_until_future_complete',
                        lambda *a, **k: None)
    return DuburiClient(MagicMock())


# --------------------------------------------------------------------------- #
#  Deadline policy                                                             #
# --------------------------------------------------------------------------- #
def test_quick_cmd_gets_short_deadline(monkeypatch):
    c = _client(monkeypatch)
    g = MagicMock(); g.cmd = 'disarm'
    assert c._result_deadline(g) == _QUICK_DEADLINE_S


def test_timed_cmd_deadline_is_limit_plus_margin(monkeypatch):
    c = _client(monkeypatch)
    g = MagicMock(); g.cmd = 'move_forward'; g.duration = 5.0; g.timeout = 0.0
    assert c._result_deadline(g) == 5.0 + _RESULT_TIMEOUT_MARGIN_S


def test_no_time_field_falls_back_to_floor(monkeypatch):
    c = _client(monkeypatch)
    g = MagicMock(); g.cmd = 'some_verb'; g.duration = 0.0; g.timeout = 0.0
    assert c._result_deadline(g) == _RESULT_TIMEOUT_FLOOR_S


def test_move_timeout_is_a_move_failed():
    # so every existing `except (MoveFailed, MoveRejected)` treats a stall as a
    # bounded failure -- no handler needs updating.
    assert issubclass(MoveTimeout, MoveFailed)


# --------------------------------------------------------------------------- #
#  send() backstops                                                           #
# --------------------------------------------------------------------------- #
def test_result_never_completes_raises_timeout_and_cancels(monkeypatch):
    c = _client(monkeypatch)
    gh = MagicMock(); gh.accepted = True
    gh.get_result_async.return_value = _Future(done=False)      # server wedged
    gh.cancel_goal_async.return_value = _Future(done=True)
    c._client.send_goal_async.return_value = _Future(done=True, result=gh)

    with pytest.raises(MoveTimeout):
        c.send('move_forward', duration=5)
    gh.cancel_goal_async.assert_called()          # goal cancelled on stall
    assert c._active_goal_handle is None           # cleared in finally


def test_goal_never_accepted_raises_timeout(monkeypatch):
    c = _client(monkeypatch)
    c._client.send_goal_async.return_value = _Future(done=False)  # never acked
    with pytest.raises(MoveTimeout):
        c.send('arm')


def test_none_goal_handle_raises_move_failed(monkeypatch):
    c = _client(monkeypatch)
    c._client.send_goal_async.return_value = _Future(done=True, result=None)
    with pytest.raises(MoveFailed):
        c.send('arm')


def test_rejected_goal_raises(monkeypatch):
    c = _client(monkeypatch)
    gh = MagicMock(); gh.accepted = False
    c._client.send_goal_async.return_value = _Future(done=True, result=gh)
    with pytest.raises(MoveRejected):
        c.send('arm')


def test_success_result_returned(monkeypatch):
    c = _client(monkeypatch)
    gh = MagicMock(); gh.accepted = True
    res_msg = MagicMock(); res_msg.result.success = True
    gh.get_result_async.return_value = _Future(done=True, result=res_msg)
    c._client.send_goal_async.return_value = _Future(done=True, result=gh)
    out = c.send('arm')
    assert out is res_msg.result


def test_cancel_active_is_bounded_and_none_safe(monkeypatch):
    c = _client(monkeypatch)
    c._active_goal_handle = None
    c.cancel_active()                              # no-op, no raise
    gh = MagicMock(); gh.cancel_goal_async.return_value = _Future(done=True)
    c._active_goal_handle = gh
    c.cancel_active()
    gh.cancel_goal_async.assert_called()
