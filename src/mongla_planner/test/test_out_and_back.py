"""The control-only swim: its shape, executed against a recorder.

Checks the run against what it has to show: at least 20 s of swimming after
the dive, a CLOCKWISE half turn, a leg back, thrusters running when the kill
switch is turned, and `surface()` as the last motion. Nothing here proves it
flies; the durations are unmeasured.
"""
import pytest

from mongla_planner.missions import out_and_back as M

SWIM_VERBS = {'move_forward', 'move_back', 'move_left', 'move_right'}


class _Recorder:
    def __init__(self, fail_on=None):
        self.sent = []
        self._fail_on = fail_on

    def __getattr__(self, name):
        def _call(*a, **k):
            self.sent.append((name, a, k))
            if name == self._fail_on:
                raise RuntimeError(f'{name} failed')
            return True
        return _call

    def names(self):
        return [n for n, _, _ in self.sent]


def _run(monkeypatch, **overrides):
    for k, v in overrides.items():
        monkeypatch.setattr(M, k, v)
    r = _Recorder()
    M.run(r)
    return r


def test_countdown_then_arm_then_dive(monkeypatch):
    names = _run(monkeypatch).names()
    assert names[:4] == ['mission_reset', 'countdown', 'arm', 'set_depth']


def test_it_turns_clockwise_through_180(monkeypatch):
    for turn in ('yaw', 'square'):
        r = _run(monkeypatch, TURN=turn)
        turns = [(n, a) for n, a, _ in r.sent if n in ('yaw_left', 'yaw_right', 'turn')]
        assert turns and all(n == 'yaw_right' for n, _ in turns), (turn, turns)
        assert sum(a[0] for _, a in turns) == pytest.approx(180.0)


def test_swims_at_least_20_s_after_the_dive(monkeypatch):
    for turn in ('yaw', 'square'):
        r = _run(monkeypatch, TURN=turn)
        swim = sum(a[0] for n, a, _ in r.sent if n in SWIM_VERBS)
        assert swim >= 20.0, (turn, swim)


def test_thrusters_run_at_the_end_for_the_kill_switch(monkeypatch):
    names = _run(monkeypatch).names()
    assert names[-3] == 'move_forward' and M.HOLD_S >= 10.0


def test_surface_is_the_last_motion_even_on_failure(monkeypatch):
    for verb in ('set_depth', 'move_forward', 'yaw_right'):
        r = _Recorder(fail_on=verb)
        with pytest.raises(RuntimeError):
            M.run(r)
        assert r.names()[-2:] == ['surface', 'disarm'], verb


def test_an_unknown_turn_is_refused(monkeypatch):
    monkeypatch.setattr(M, 'TURN', 'circle')
    r = _Recorder()
    with pytest.raises(ValueError, match='circle'):
        M.run(r)
    assert r.names()[-2:] == ['surface', 'disarm']
