"""An unknown verb is refused before it can claim the command gate.

It used to be claimed in `goal_callback` and refused only in
`execute_callback`, which returned before the `try` whose `finally` releases
the gate. The claim then stood, unconfirmed, until the leak reaper took it
COMMAND_GATE_STALE_S later, and every real verb sent in that window was
rejected as "command already active". Found by the cloud audit, 2026-11-07.

INJECTION-VERIFIED: delete the `not in COMMANDS` check in goal_callback and
this fails.
"""
import types

import pytest

pytest.importorskip('rclpy')


def _node():
    import mongla_manager.auv_manager_node as amn
    from mongla_manager.dispatch_policy import _CommandGate
    log = types.SimpleNamespace(warn=lambda *a: None, info=lambda *a: None)
    node = types.SimpleNamespace(
        _gate=_CommandGate(),
        get_logger=lambda: log,
        mongla=types.SimpleNamespace(request_abort=lambda: None))
    return amn, node


def test_an_unknown_verb_is_rejected_and_claims_nothing():
    amn, node = _node()
    from rclpy.action import GoalResponse
    req = types.SimpleNamespace(cmd='move_sideways_fast')
    assert amn.AUVManagerNode.goal_callback(node, req) == GoalResponse.REJECT
    assert not node._gate.active, 'the gate must be free for the next real verb'


def test_a_real_verb_still_claims():
    amn, node = _node()
    from rclpy.action import GoalResponse
    req = types.SimpleNamespace(cmd='stop')
    assert amn.AUVManagerNode.goal_callback(node, req) == GoalResponse.ACCEPT
    assert node._gate.active
