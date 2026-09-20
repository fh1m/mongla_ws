"""Pure goal-acceptance policy for the /mongla/move action server.

Extracted from `auv_manager_node.goal_callback` so the safety-bypass rule is
unit-testable without spinning a live ROS2 node. The rule (CLAUDE.md §13 safety):
the **safety verbs** (`disarm` / `stop` / `surface`) bypass the `command_active`
gate so they always execute even while another command runs — and accepting one
while busy signals a cooperative abort to the running loop so it releases the lock.
"""
from __future__ import annotations

SAFETY_VERBS = frozenset({'disarm', 'stop', 'surface'})


def goal_acceptance(cmd, command_active, safety_verbs=SAFETY_VERBS):
    """Decide whether to accept an incoming goal. Returns ``(accept, signal_abort)``.

    - **idle** (``command_active`` False): accept any verb; no abort.
    - **busy + non-safety verb**: reject (one command at a time).
    - **busy + safety verb**: accept AND signal abort (pre-empt the running loop).
    """
    if not command_active:
        return True, False
    if cmd in safety_verbs:
        return True, True
    return False, False
