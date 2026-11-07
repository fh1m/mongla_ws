"""A Move result says HOW it ended, not only whether.

Before this, `MoveResult.code` existed on the host -- the board's ACCEPTED,
CANCELLED, FAILED and DENIED, plus the host's TIMEOUT and ABORTED -- and
`_make_result` dropped it. A mission got `success=False` and a string, so a
board refusal, a deadline and an operator's cancel looked the same unless it
parsed English.
"""
from mongla_interfaces.action import Move

from mongla_control.fc import base
from mongla_control.fc.base import MoveResult
from mongla_planner.client import MoveFailed, MoveTimeout, _failed

NAMES = ('SUCCEEDED', 'PREEMPTED', 'FAILED', 'DENIED', 'TIMEOUT', 'ABORTED')


def test_the_wire_codes_are_the_host_codes():
    """Two copies of one table: the .action constants and fc/base.py."""
    for name in NAMES:
        assert getattr(Move.Result, name) == getattr(base, name), name


class _Stub:
    """Only what `_make_result` and `_board` touch."""

    def __init__(self, move_result):
        from mongla_control.mongla import Mongla
        self._make_result = Mongla._make_result.__get__(self)
        self._board = Mongla._board.__get__(self)
        self._current_depth = lambda: -0.5
        self._current_heading = lambda: 90.0
        self._abort_fn = lambda: False
        self._distance_axis = None
        from contextlib import nullcontext
        self._command_scope = lambda cmd: nullcontext()
        self.fc = type('FC', (), {'move': lambda s, *a, **k: move_result})()


def test_a_board_refusal_keeps_the_boards_code():
    for code in (base.DENIED, base.FAILED, base.TIMEOUT, base.PREEMPTED):
        res = _Stub(MoveResult(code, 'barometer unhealthy'))._board('move_forward')
        assert not res.success
        assert res.code == code, base._CODE_NAME[code]


def test_a_board_success_is_succeeded():
    res = _Stub(MoveResult(base.SUCCEEDED))._board('move_forward')
    assert res.success and res.code == Move.Result.SUCCEEDED


def test_a_host_verb_is_succeeded_or_failed_by_success():
    stub = _Stub(None)
    assert stub._make_result(True, 'ok').code == Move.Result.SUCCEEDED
    assert stub._make_result(False, 'no').code == Move.Result.FAILED
    assert stub._make_result(False, 'x', code=base.ABORTED).code == Move.Result.ABORTED


def test_the_mission_sees_the_code_on_the_exception():
    r = Move.Result()
    r.success, r.code, r.message = False, Move.Result.DENIED, 'baro refused'
    exc = _failed('move_forward', r)
    assert isinstance(exc, MoveFailed)
    assert exc.code == Move.Result.DENIED and exc.code_name == 'DENIED'
    assert 'DENIED' in str(exc) and 'baro refused' in str(exc)


def test_a_client_deadline_is_a_timeout():
    exc = MoveTimeout('stalled')
    assert isinstance(exc, MoveFailed) and exc.code == Move.Result.TIMEOUT


def test_a_failure_with_no_result_is_failed():
    assert MoveFailed('no goal handle').code == Move.Result.FAILED
