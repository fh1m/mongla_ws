"""Unit tests for the /mongla/move goal-acceptance policy.

The safety-bypass rule (CLAUDE.md §13) is competition-safety-critical: `disarm`,
`stop`, `surface` must ALWAYS be accepted even while another command runs, and
accepting one while busy must signal abort. Tested via the pure
`goal_acceptance` helper (no live ROS2 node).
"""

import pytest

from mongla_manager.dispatch_policy import goal_acceptance, SAFETY_VERBS


# --- idle: accept anything, never abort --------------------------------------

@pytest.mark.parametrize('cmd', ['move_forward', 'yaw_left', 'disarm', 'vision_align'])
def test_idle_accepts_any_verb_without_abort(cmd):
    accept, signal_abort = goal_acceptance(cmd, command_active=False)
    assert accept is True
    assert signal_abort is False


# --- busy: one command at a time, except safety verbs ------------------------

@pytest.mark.parametrize('cmd', ['move_forward', 'yaw_left', 'set_depth', 'vision_move'])
def test_busy_rejects_non_safety(cmd):
    accept, signal_abort = goal_acceptance(cmd, command_active=True)
    assert accept is False
    assert signal_abort is False


@pytest.mark.parametrize('cmd', ['disarm', 'stop', 'surface'])
def test_busy_accepts_safety_verb_and_signals_abort(cmd):
    accept, signal_abort = goal_acceptance(cmd, command_active=True)
    assert accept is True
    assert signal_abort is True          # pre-empts the running loop


def test_safety_set_is_exactly_the_three_verbs():
    assert SAFETY_VERBS == {'disarm', 'stop', 'surface'}


def test_custom_safety_set_is_honored():
    accept, signal_abort = goal_acceptance(
        'kill', command_active=True, safety_verbs={'kill'})
    assert accept is True and signal_abort is True
