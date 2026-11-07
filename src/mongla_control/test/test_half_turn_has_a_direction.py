"""A relative turn of 180 degrees or more is sent as legs the board cannot reverse.

The board aims a relative TURN at `wrapPi(cur + angle)` and picks its direction
from the sign of `wrapPi(target - yaw)` (`movement.cpp`, issue 14 of the
2026-09-24 review). At exactly 180 degrees that error is +pi or -pi, and which
one comes down to float rounding and the yaw noise of the first tick:
simulated in float32 the way the ESP32 computes it, `yaw_right(180)` turned
clockwise 0.498 of the time with 0.2 deg of noise (measured-bars §68). A turn
past 180 goes the other way, and 360 completes at once without moving.

So the facade splits any relative turn wider than MAX_TURN_LEG_DEG into equal
legs: 180 is two 90s, 360 three 120s, each unambiguous.
"""
import math
from types import SimpleNamespace

import pytest

from mongla_control import mongla as M


class _Facade:
    """Just enough of `Mongla` to run `_relative_turn` against a recorder."""

    def __init__(self, fail_leg=None, abort_after=None):
        self.legs = []
        self._fail_leg = fail_leg
        self._abort_after = abort_after
        self._abort_event = SimpleNamespace(is_set=lambda: self._aborted())

    def _aborted(self):
        return self._abort_after is not None and len(self.legs) >= self._abort_after

    def _board(self, cmd, on_progress=None, **kw):
        self.legs.append((cmd, kw))
        if on_progress is not None:
            on_progress(1.0)
        ok = self._fail_leg is None or len(self.legs) != self._fail_leg
        return SimpleNamespace(success=ok, message='done' if ok else 'stalled')

    def _make_result(self, success, message, final_value=None, code=None):
        return SimpleNamespace(success=success, message=message)


def _turn(f, verb, target, **kw):
    return M.Mongla._relative_turn(f, verb, target, kw.get('timeout', 30.0),
                                   kw.get('settle', 1.0), kw.get('on_progress'))


@pytest.mark.parametrize('target, legs', [
    (90.0, [90.0]), (120.0, [120.0]), (180.0, [90.0, 90.0]),
    (270.0, [90.0, 90.0, 90.0]), (360.0, [120.0, 120.0, 120.0]),
])
def test_the_split(target, legs):
    f = _Facade()
    assert _turn(f, 'yaw_right', target).success
    assert [kw['target'] for _, kw in f.legs] == pytest.approx(legs)
    assert all(abs(kw['target']) <= M.MAX_TURN_LEG_DEG for _, kw in f.legs)


def test_every_leg_turns_the_same_way_and_only_the_last_settles():
    f = _Facade()
    _turn(f, 'yaw_right', 180.0, settle=2.0)
    assert {cmd for cmd, _ in f.legs} == {'yaw_right'}
    assert [kw['settle'] for _, kw in f.legs] == [0.0, 2.0]


def test_progress_runs_once_across_the_whole_turn():
    seen = []
    _turn(_Facade(), 'yaw_left', 180.0, on_progress=seen.append)
    assert seen == pytest.approx([0.5, 1.0])


def test_a_failed_leg_stops_the_turn_and_says_which():
    f = _Facade(fail_leg=1)
    res = _turn(f, 'yaw_right', 180.0)
    assert not res.success and len(f.legs) == 1
    assert 'leg 1/2' in res.message and 'stalled' in res.message


def test_an_abort_between_legs_is_not_cleared_by_the_next_leg():
    f = _Facade(abort_after=1)
    res = _turn(f, 'yaw_right', 180.0)
    assert not res.success and len(f.legs) == 1
    assert 'aborted' in res.message


def test_bad_targets_pass_through_to_the_verb_table():
    """NaN and more than one revolution are refused by `_build_params`; the
    split must not turn them into something else first."""
    for bad in (math.nan, 3600.0):
        f = _Facade()
        _turn(f, 'yaw_right', bad)
        assert len(f.legs) == 1
        assert f.legs[0][1]['target'] is bad or f.legs[0][1]['target'] == bad
