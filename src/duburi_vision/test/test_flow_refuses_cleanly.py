"""A refusal must publish NOTHING but its quality byte, and must stop the tick.

Measured on the vehicle with the downward camera fitted and no manager running:

    [FLOW ] REFUSING: no depth yet, so no height above the floor  (q=0, 5
            refused / 0 used)

quality 0, zero velocity messages, no fabricated number. That is the behaviour
that makes the flow path safe to leave running: without depth there is no
height above the floor, without height there are no metres per pixel, and a
velocity published anyway would be a plausible number with no fault anywhere --
this stack's most expensive recurring defect.

Two things are guarded. The behaviour: `_refuse` publishes quality and distance
and never a velocity. And the STRUCTURE: nothing executes after a `_refuse` in
its own block, because a refusal that falls through would go on to publish a
velocity computed from the very state it just rejected.
"""
from __future__ import annotations

import ast
import pathlib
from unittest.mock import MagicMock

import pytest

_NODE = (pathlib.Path(__file__).resolve().parents[1]
         / 'duburi_vision' / 'flow' / 'flow_node.py')


def test_refusing_publishes_quality_zero_and_no_velocity():
    from duburi_vision.flow.flow_node import FlowVelocityNode

    fake = MagicMock()
    fake._n_refused = 0
    fake._acc.distance_m = 1.25
    FlowVelocityNode._refuse(fake, 'no depth yet, so no height above the floor')

    assert fake._last_quality == 0
    fake._pub_quality.publish.assert_called_once()
    assert fake._pub_quality.publish.call_args.args[0].data == 0
    fake._pub_vel.publish.assert_not_called()


def test_refusing_records_the_reason_for_the_operator():
    from duburi_vision.flow.flow_node import FlowVelocityNode

    fake = MagicMock()
    fake._n_refused = 0
    FlowVelocityNode._refuse(fake, 'gyro stale')
    assert fake._last_reason == 'gyro stale'
    assert fake._n_refused == 1


def _refusals_not_followed_by_return() -> list[int]:
    """Line numbers of `self._refuse(...)` calls that do not stop the tick."""
    tree = ast.parse(_NODE.read_text())
    bad: list[int] = []
    for parent in ast.walk(tree):
        body = getattr(parent, 'body', None)
        for attr in ('body', 'orelse', 'finalbody'):
            block = getattr(parent, attr, None)
            if not isinstance(block, list):
                continue
            for i, stmt in enumerate(block):
                if not (isinstance(stmt, ast.Expr)
                        and isinstance(stmt.value, ast.Call)):
                    continue
                f = stmt.value.func
                if not (isinstance(f, ast.Attribute) and f.attr == '_refuse'):
                    continue
                # Last statement in its block is CORRECT: control leaves the
                # branch and whatever the enclosing block does applies. Two of
                # the real sites sit in an if/else whose caller returns right
                # after, and one ends its method -- demanding a literal
                # `return` next would flag all three and teach the reader that
                # the guard is noise. What is NOT correct is computing on after
                # a refusal in the SAME block.
                if i + 1 >= len(block):
                    continue
                nxt = block[i + 1]
                if not isinstance(nxt, (ast.Return, ast.Continue, ast.Break)):
                    bad.append(stmt.lineno)
    return bad


def test_every_refusal_stops_the_tick():
    bad = _refusals_not_followed_by_return()
    assert not bad, (
        f'flow_node.py: _refuse() at line(s) {bad} is not followed by '
        f'return/continue/break -- the tick would carry on and publish a '
        f'velocity built from the state it just refused')
