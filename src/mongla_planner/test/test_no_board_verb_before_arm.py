"""No mission sends a board verb before it arms (B-89).

The board refuses every SROT_MOVE while disarmed -- `mav_commands.cpp`,
"SROT_MOVE refused: arm first" -- and every move, `pause` and `stop` included,
is a SROT_MOVE. Five missions took their tether window as `pause(N)` before
`arm()`, a habit from the old backend, where `pause` was a host sleep. On this
board each of them would have failed at its first line, surfaced and disarmed.

The tether window is `countdown()`, which runs on the host.
"""
import ast
from pathlib import Path

import pytest

MISSIONS = Path(__file__).resolve().parents[1] / 'mongla_planner' / 'missions'
BOARD_VERBS = {'pause', 'move_forward', 'move_back', 'move_left', 'move_right',
               'yaw_left', 'yaw_right', 'turn', 'set_depth', 'stop', 'style_roll'}


def _board_verbs_before_arm(src: str):
    bad = []
    for fn in ast.walk(ast.parse(src)):
        if not isinstance(fn, ast.FunctionDef):
            continue
        calls = sorted((n.lineno, n.col_offset, n.func.attr) for n in ast.walk(fn)
                       if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute))
        names = [c[2] for c in calls]
        if 'arm' not in names:
            continue
        first_arm = names.index('arm')
        bad += [f'{fn.name}:{ln} {verb}()' for ln, _, verb in calls[:first_arm]
                if verb in BOARD_VERBS]
    return bad


def test_no_mission_moves_the_board_before_arming():
    files = sorted(MISSIONS.glob('*.py'))
    assert len(files) > 10, 'found too few missions -- the path broke, not the rule'
    bad = [f'{p.name}: {b}' for p in files for b in _board_verbs_before_arm(p.read_text())]
    assert not bad, ('the board refuses these while disarmed; take the tether window '
                     'with countdown():\n  ' + '\n  '.join(bad))


def test_the_check_catches_the_old_tether_pause():
    """Injection: the exact shape the missions had."""
    old = 'def run(m):\n    m.pause(10.0)\n    m.arm()\n    m.set_depth(-1)\n'
    assert _board_verbs_before_arm(old) == ['run:2 pause()']
    assert _board_verbs_before_arm(old.replace('pause', 'countdown')) == []


def _countdown(monkeypatch, env):
    from mongla_planner.dsl_report import ReportMixin
    for k in ('SSH_CONNECTION', 'TMUX', 'STY'):
        monkeypatch.delenv(k, raising=False)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    return ReportMixin()


def test_countdown_refuses_on_bare_ssh(monkeypatch):
    rm = _countdown(monkeypatch, {'SSH_CONNECTION': '1 2 3 4'})
    with pytest.raises(RuntimeError, match='tmux'):
        rm.countdown(1)


def test_countdown_runs_inside_tmux_on_deadlines(monkeypatch):
    """Each tick sleeps to its deadline, so slow redraws do not add up."""
    import mongla_planner.dsl_report as dr
    rm = _countdown(monkeypatch, {'SSH_CONNECTION': '1 2 3 4', 'TMUX': '/tmp/t,1,0'})
    clock = {'t': 100.0}
    slept = []

    def sleep(s):
        slept.append(s)
        clock['t'] += s + 0.05          # every tick overshoots by 50 ms
    # Replace the module's handle, not attributes of `time` itself: patching
    # time.sleep moved this fake clock whenever another test's thread slept.
    from types import SimpleNamespace
    monkeypatch.setattr(dr, '_time', SimpleNamespace(
        monotonic=lambda: clock['t'], sleep=sleep, strftime=dr._time.strftime))
    rm.countdown(5)
    assert len(slept) == 5
    assert clock['t'] - 100.0 == pytest.approx(5.05, abs=1e-9)   # one overshoot, not five
