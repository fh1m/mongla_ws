"""`mongla.task(name, deadline_s=)`: cancel the goal in flight and abandon cleanly.

The client and rclpy spin are mocked exactly as `test_client.py` does, so the
deadline logic is executed, not read.
"""
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from mongla_planner.client import MonglaClient, TaskAbandoned, goal_uuid_hex
from mongla_planner.mongla_dsl import MonglaMission


class _Future:
    def __init__(self, done=True, result=None):
        self._done, self._result = done, result

    def done(self):
        return self._done

    def result(self):
        return self._result


def _client(monkeypatch):
    monkeypatch.setattr('mongla_planner.client.ActionClient', lambda *a, **k: MagicMock())
    monkeypatch.setattr('mongla_planner.client.rclpy.spin_until_future_complete',
                        lambda *a, **k: time.sleep(0.01))
    return MonglaClient(MagicMock())


def _hanging_goal(c):
    gh = MagicMock()
    gh.accepted = True
    gh.goal_id.uuid = list(range(16))
    gh.get_result_async.return_value = _Future(done=False)       # never finishes
    gh.cancel_goal_async.return_value = _Future(done=True)
    c._client.send_goal_async.return_value = _Future(done=True, result=gh)
    return gh


def test_a_passed_deadline_cancels_the_goal_in_flight(monkeypatch):
    c = _client(monkeypatch)
    gh = _hanging_goal(c)
    c._task_deadline = time.monotonic() + 0.05
    with pytest.raises(TaskAbandoned):
        c.send('move_forward', duration=30.0)
    gh.cancel_goal_async.assert_called()
    assert c._active_goal_handle is None


def test_no_new_work_starts_after_the_deadline_but_safety_verbs_do(monkeypatch):
    c = _client(monkeypatch)
    gh = _hanging_goal(c)
    gh.get_result_async.return_value = _Future(
        done=True, result=SimpleNamespace(result=SimpleNamespace(success=True, message='')))
    c._task_deadline = time.monotonic() - 1.0
    with pytest.raises(TaskAbandoned):
        c.send('move_forward', duration=5.0)
    c._client.send_goal_async.assert_not_called()
    assert c.send('surface').success                   # always sendable


def test_the_goal_uuid_is_recorded(monkeypatch):
    c = _client(monkeypatch)
    gh = _hanging_goal(c)
    gh.get_result_async.return_value = _Future(
        done=True, result=SimpleNamespace(result=SimpleNamespace(success=True, message='')))
    c.send('stop')
    assert c.last_goal_id == bytes(range(16)).hex()
    assert goal_uuid_hex(object()) == ''


def _dsl():
    m = MagicMock()
    m.client = SimpleNamespace(_task_deadline=None)
    m.__dict__['_budget'] = None
    m.__dict__['_scoreboard'] = []
    return m


def test_the_task_block_sets_restores_and_records():
    m = _dsl()
    with MonglaMission.task(m, 'gate', deadline_s=30.0):
        assert m.client._task_deadline is not None
    assert m.client._task_deadline is None
    assert m.__dict__['_scoreboard'][-1]['msg'] == 'done'


def test_an_abandoned_task_is_recorded_and_re_raised():
    m = _dsl()
    with pytest.raises(TaskAbandoned):
        with MonglaMission.task(m, 'torpedo', deadline_s=1.0):
            raise TaskAbandoned('deadline')
    row = m.__dict__['_scoreboard'][-1]
    assert (row['cmd'], row['msg'], row['success']) == ('task:torpedo', 'abandoned', False)


def test_a_nested_task_keeps_the_earlier_deadline():
    m = _dsl()
    with MonglaMission.task(m, 'outer', deadline_s=5.0):
        outer = m.client._task_deadline
        with MonglaMission.task(m, 'inner', deadline_s=60.0):
            assert m.client._task_deadline == outer
        assert m.client._task_deadline == outer


def test_no_deadline_and_no_budget_changes_nothing():
    m = _dsl()
    with MonglaMission.task(m, 'bins'):
        assert m.client._task_deadline is None
