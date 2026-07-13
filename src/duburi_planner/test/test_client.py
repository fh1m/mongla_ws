"""DuburiClient bounded-deadline tests — a wedged/dead action server must never
hang the caller (or the mission's disarm backstop) forever.

rclpy spin + the ActionClient are mocked, so no ROS/MAVLink is needed.
"""

import pytest
from types import SimpleNamespace
from unittest.mock import MagicMock

from duburi_planner.client import (
    DuburiClient, MoveFailed, MoveRejected, MoveTimeout,
    _QUICK_DEADLINE_S, _RESULT_TIMEOUT_FLOOR_S, _RESULT_TIMEOUT_MARGIN_S,
)


def _goal(cmd, *, duration=0.0, timeout=0.0):
    """A Move.Goal-shaped stub with REAL float time fields (rosidl defaults 0.0);
    `send()` does not apply COMMANDS defaults, so the on-the-wire timeout is 0.0
    and _result_deadline must read the effective budget from the registry."""
    return SimpleNamespace(cmd=cmd, duration=duration, timeout=timeout)


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
def test_pure_safety_verb_gets_short_floor(monkeypatch):
    # stop/surface/unlock carry no timeout default -> the 12 s safety floor, so the
    # disarm backstop never waits on a healthy-server assumption.
    c = _client(monkeypatch)
    assert c._result_deadline(_goal('stop')) == _QUICK_DEADLINE_S


def test_arm_floor_covers_its_server_budget(monkeypatch):
    # The "doesn't arm, 12 s crossed" fix: arm's real budget is 3 s ACK + 15 s
    # poll; the goal wire-timeout is 0.0, so the registry default (15) drives the
    # deadline to 15+15=30 -- well past the old fixed 12 s that cut it off.
    c = _client(monkeypatch)
    assert c._result_deadline(_goal('arm')) == 15.0 + _RESULT_TIMEOUT_MARGIN_S
    assert c._result_deadline(_goal('arm')) > _QUICK_DEADLINE_S


def test_disarm_floor_covers_its_server_budget(monkeypatch):
    # disarm is still a quick/safety verb but its real budget (MANUAL + neutral +
    # 20 s poll) needs a floor that covers it, not the old fixed 12 s ceiling.
    c = _client(monkeypatch)
    assert c._result_deadline(_goal('disarm')) == 20.0 + _RESULT_TIMEOUT_MARGIN_S


def test_timed_cmd_deadline_is_limit_plus_margin(monkeypatch):
    c = _client(monkeypatch)
    assert c._result_deadline(_goal('move_forward', duration=5.0)) \
        == 5.0 + _RESULT_TIMEOUT_MARGIN_S


def test_no_time_field_falls_back_to_floor(monkeypatch):
    c = _client(monkeypatch)
    assert c._result_deadline(_goal('some_verb')) == _RESULT_TIMEOUT_FLOOR_S


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
