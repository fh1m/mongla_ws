"""B42 -- Ctrl-C during a move did not command a brake on srot.

`_emergency_stop` is the Ctrl-C / SIGTERM path, and CLAUDE.md safety rule 1 makes
it non-negotiable: "Ctrl-C on the manager triggers Duburi.stop() + disarm()".

Its "stop the thrusters" step was `pixhawk.send_neutral()`. On srot that is a zero
MANUAL_CONTROL frame -- and a SROT_MOVE leaves the board in AUTO, where the
firmware overwrites every pilot axis from the movement primitive and the frame is
DISCARDED (B28). Ctrl-C arrives most often DURING a move, so the one step that
exists to halt the hull was a no-op in exactly the case it exists for, leaving
`disarm()` as the only thing stopping it: motors cut mid-leg instead of a
commanded brake, and nothing at all if the disarm is the step that fails.

MOVE_STOP is the board's own brake, honoured in AUTO, and from fw rev 2 it
decelerates on-board.
"""

import ast
import inspect
import pathlib
import sys

import pytest

import duburi_control.fc.srot_protocol as sp

# `_fc()` lives in duburi_control's test dir; this test spans both packages.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / 'duburi_control' / 'test'))
from test_srot_fc import _fc          # noqa: E402


def test_stop_motion_puts_MOVE_STOP_on_the_wire():
    fc = _fc()
    fc.stop_motion()
    stops = [x for x in fc.master.mav.sent
             if x[0] == 'cmd' and x[1] == sp.CMD_SROT_MOVE and x[2][0] == sp.MOVE_STOP]
    assert stops, 'stop_motion must command the board brake'


def test_send_neutral_is_only_a_MANUAL_CONTROL_frame():
    """Documents WHY send_neutral is not sufficient: it is the discarded kind."""
    fc = _fc()
    fc.send_neutral()
    kinds = {x[0] for x in fc.master.mav.sent}
    assert 'manual' in kinds
    assert not any(x[0] == 'cmd' and x[2][0] == sp.MOVE_STOP
                   for x in fc.master.mav.sent if x[0] == 'cmd'), \
        'send_neutral does not brake a running leg -- that is the point of B42'


def _emergency_stop_source():
    import duburi_manager.auv_manager_node as amn
    return inspect.getsource(amn._emergency_stop)


def _call_order():
    """Names of the vehicle actions, in EXECUTION order.

    Parsed, not text-searched: the explanatory comment above the fix mentions
    `send_neutral` before the code does, so a naive `src.index()` compares a
    comment against a call. My first version of this test did exactly that and
    failed on correct code.
    """
    tree = ast.parse(_emergency_stop_source().lstrip())
    seen = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
           and n.func.attr in ('stop_motion', 'send_neutral', 'disarm'):
            seen.append((n.lineno, n.func.attr))
    return [name for _, name in sorted(seen)]


def test_the_emergency_stop_brakes_before_it_goes_neutral():
    order = _call_order()
    assert 'stop_motion' in order, (
        'Ctrl-C during a move must command MOVE_STOP -- send_neutral is '
        'discarded while the board is in AUTO (B28/B42)')
    assert order.index('stop_motion') < order.index('send_neutral'), \
        'brake the running leg BEFORE releasing the sticks'
    assert order.index('stop_motion') < order.index('disarm'), \
        'brake before disarm, so the hull decelerates instead of coasting'


def test_the_brake_step_is_guarded_for_the_pixhawk_backend():
    """PixhawkFC has no stop_motion; an unguarded call would break that path."""
    from duburi_control.pixhawk import Pixhawk
    assert not hasattr(Pixhawk, 'stop_motion')
    assert "hasattr(node.fc, 'stop_motion')" in _emergency_stop_source(), \
        'the srot-only brake must be feature-detected, not assumed'


def test_every_step_stays_isolated():
    """A failing brake must not prevent the disarm below it."""
    src = _emergency_stop_source()
    tree = ast.parse(src.lstrip())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_emergency_stop')
    step = next(n for n in ast.walk(fn) if isinstance(n, ast.FunctionDef)
                and n.name == '_step')
    assert any(isinstance(h, ast.ExceptHandler) for h in ast.walk(step)), \
        '_step must swallow per-step failures so later steps still run'
