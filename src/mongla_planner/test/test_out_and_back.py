"""The control-only swim: its shape, executed against a recorder.

Checks the run's requested leg durations (not measured swimming distance or
wet motion), clockwise half turn, stationary kill-switch window, and that
SURFACE is the last requested motion. Nothing here proves it flies.
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


def test_requests_at_least_20_s_of_underwater_forward_legs(monkeypatch):
    for turn in ('yaw', 'square'):
        r = _run(monkeypatch, TURN=turn)
        swim = sum(a[0] for n, a, _ in r.sent if n in SWIM_VERBS)
        assert swim >= 20.0, (turn, swim)


def test_the_dive_and_clockwise_turn_have_real_board_holds(monkeypatch):
    for turn in ('yaw', 'square'):
        r = _run(monkeypatch, TURN=turn)
        calls = r.sent
        names = r.names()
        pauses = [(i, a[0]) for i, (n, a, _) in enumerate(calls) if n == 'pause']
        assert [t for _, t in pauses] == [
            M.DIVE_HOLD_S, M.TURN_HOLD_S, M.KILL_WINDOW_S
        ]
        # Explicit SROT_MOVE HOLDs, visible in the mission (settle= is the same HOLD, #86).
        assert pauses[0][0] == names.index('set_depth') + 1
        last_turn = max(i for i, n in enumerate(names) if n == 'yaw_right')
        assert pauses[1][0] == last_turn + 1
        assert calls[pauses[0][0] + 1][0] == 'move_forward'
        assert calls[pauses[1][0] + 1][0] == 'move_forward'


def test_kill_window_is_stationary_hold_not_an_unbounded_forward_leg(monkeypatch):
    r = _run(monkeypatch)
    names = r.names()
    assert names[-3:] == ['pause', 'surface', 'disarm']
    assert M.KILL_WINDOW_S > 0.0
    pause = r.sent[-3]
    assert pause[1] == (M.KILL_WINDOW_S,)
    # The kill window follows the back leg directly, and once it starts no
    # further translation can be commanded. (The FIRST pause is the dive hold,
    # which forward legs rightly follow.)
    kill = len(names) - 3
    assert names[kill - 1] == 'move_forward'
    assert not any(n in SWIM_VERBS for n in names[kill + 1:])


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


def test_disarm_is_attempted_even_if_surface_raises():
    r = _Recorder(fail_on='surface')
    with pytest.raises(RuntimeError, match='surface failed'):
        M.run(r)
    assert r.names()[-2:] == ['surface', 'disarm']


def test_failed_stationary_hold_attempts_surface_and_disarm():
    r = _Recorder(fail_on='pause')
    with pytest.raises(RuntimeError, match='pause failed'):
        M.run(r)
    assert r.names()[-3:] == ['pause', 'surface', 'disarm']


def test_every_turn_settles_before_the_hull_translates(monkeypatch):
    """In 'square' the side leg follows the first right angle directly, so
    without a settle the hull starts it still swinging."""
    for turn in ('yaw', 'square'):
        r = _run(monkeypatch, TURN=turn)
        turns = [k for n, _, k in r.sent if n == 'yaw_right']
        assert turns and all(k.get('settle', 0) > 0 for k in turns), (turn, turns)
