"""The /mongla/move busy gate must be a test-and-SET, not a test then a set.

⛔ THE DEFECT THIS EXISTS FOR. `command_active` was a plain bool, READ in
`goal_callback` (the accept/reject decision) and WRITTEN in `execute_callback`
(the start of the motion loop). Those are two callbacks on a
`ReentrantCallbackGroup` under a `MultiThreadedExecutor`, and nothing held a lock
across the gap, so a second goal arriving inside it saw `command_active` still
False and was ACCEPTED. Two motion loops then commanded the board at the same
time, sharing one abort flag, and whichever finished first cleared the gate while
the other was still driving thrust.

MODELLED, and stated precisely: these tests do not spin an ActionServer (rclpy is
not a test dependency of this package). They drive the REAL `_CommandGate` through
the SAME two-phase sequence the node uses. rclpy decides the WIDTH of the window,
not whether it exists -- and the pre-fix reproduction hit it in 40/40 trials at
hand-off delays of 20 ms, 5 ms and 1 ms.

INJECTION-VERIFIED. Replace `_CommandGate.claim` with the old two-step form

    def claim(self, cmd, policy):
        accept, sa = policy(cmd, self.active)   # test
        if accept: self._cmd = cmd              # ...and set, separately
        return accept, sa

and move the assignment into a simulated `execute_callback`, and
`test_two_goals_cannot_both_claim` fails. Verified before this file was committed.
"""
import threading
import time

import pytest

from mongla_manager.dispatch_policy import (
    goal_acceptance, _CommandGate, SAFETY_VERBS, COMMAND_GATE_STALE_S)


def _run_goal(gate, cmd, hand_off_s, tally):
    """One goal, exactly as the executor drives it: claim, hand off, then run."""
    accept, _ = gate.claim(cmd, goal_acceptance)
    if not accept:
        return
    time.sleep(hand_off_s)              # ACCEPT -> EXECUTING, on another thread
    gate.confirm(cmd)
    with tally['lock']:
        tally['n'] += 1
        tally['max'] = max(tally['max'], tally['n'])
    try:
        time.sleep(0.03)                # the motion loop
    finally:
        with tally['lock']:
            tally['n'] -= 1
        gate.release()


@pytest.mark.parametrize('hand_off_s', [0.020, 0.005, 0.001])
def test_two_goals_cannot_both_claim(hand_off_s):
    """⛔ The regression. Two ordinary motion verbs, arriving back-to-back as a
    mission sequences them, must never both reach the motion loop."""
    for _ in range(40):
        gate = _CommandGate()
        tally = {'n': 0, 'max': 0, 'lock': threading.Lock()}
        threads = [threading.Thread(target=_run_goal,
                                    args=(gate, cmd, hand_off_s, tally))
                   for cmd in ('move_forward', 'move_left')]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert tally['max'] == 1, (
            f'{tally["max"]} motion loops ran concurrently at a {hand_off_s * 1e3:.0f} ms '
            f'hand-off -- the gate is not atomic')


def test_a_safety_verb_still_preempts_a_running_command():
    """The whole point of the gate's exception. Fixing the race must not close it."""
    gate = _CommandGate()
    assert gate.claim('move_forward', goal_acceptance) == (True, False)
    for verb in sorted(SAFETY_VERBS):
        g = _CommandGate()
        g.claim('move_forward', goal_acceptance)
        g.confirm('move_forward')
        accept, signal_abort = g.claim(verb, goal_acceptance)
        assert accept, f'{verb} must be accepted while busy'
        assert signal_abort, f'{verb} must signal a cooperative abort'


def test_an_ordinary_verb_is_still_rejected_while_busy():
    gate = _CommandGate()
    gate.claim('move_forward', goal_acceptance)
    gate.confirm('move_forward')
    assert gate.claim('move_left', goal_acceptance) == (False, False)
    gate.release()
    assert gate.claim('move_left', goal_acceptance) == (True, False)


def test_cancel_drains_rather_than_opening_the_gate():
    """A cancel used to free the gate for EVERY verb, so an ordinary `move_*`
    could start while the cancelled leg was still commanding thrust. DRAINING is
    busy for everything except a safety verb."""
    gate = _CommandGate()
    gate.claim('move_forward', goal_acceptance)
    gate.confirm('move_forward')
    gate.begin_drain()
    assert gate.active, 'a draining gate is still busy'
    assert gate.claim('move_left', goal_acceptance) == (False, False), \
        'an ordinary verb must NOT start while a cancelled leg winds down'
    accept, signal_abort = gate.claim('disarm', goal_acceptance)
    assert accept and signal_abort, 'disarm must still get through a drain'


def test_a_leaked_claim_is_reaped_but_a_long_command_is_not():
    """A claim that never reaches execute_callback would reject every later verb
    in silence. A CONFIRMED long-running command must never be reaped."""
    gate = _CommandGate(stale_s=0.05)
    gate.claim('move_forward', goal_acceptance)       # never confirmed
    assert gate.reap_stale() is None, 'not stale yet'
    time.sleep(0.06)
    assert gate.reap_stale() == 'move_forward'
    assert not gate.active
    assert gate.reap_stale() is None, 'reaping is not repeatable'

    gate = _CommandGate(stale_s=0.05)
    gate.claim('move_forward', goal_acceptance)
    gate.confirm('move_forward')                      # a real, running leg
    time.sleep(0.06)
    assert gate.reap_stale() is None, \
        'a confirmed command must never be timed out -- move_forward 120 is not a leak'
    assert gate.active


def test_the_policy_itself_is_untouched():
    """`goal_acceptance` stays the pure rule; the gate only adds atomicity."""
    assert goal_acceptance('move_forward', False) == (True, False)
    assert goal_acceptance('move_forward', True) == (False, False)
    assert goal_acceptance('disarm', True) == (True, True)
    assert COMMAND_GATE_STALE_S > 0.5
