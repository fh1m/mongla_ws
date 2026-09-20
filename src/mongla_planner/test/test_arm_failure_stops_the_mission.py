"""B36 -- a failed arm must stop the mission, not run it disarmed.

NASA JPL Power-of-10 rule 7 (Holzmann, *The Power of 10: Rules for Developing
Safety-Critical Code*, IEEE Computer 39(6), 2006): "the return value of non-void
functions must be checked by each calling function."

Measured before the fix: **16 of 16 missions called `mongla.arm()` and discarded
the result**, including `task_full_2026` -- the full competition run. `_send()`
does not raise; it logs, records `success=False` on the scoreboard, and returns.
So an arm that fails its pre-arm checks left the mission executing its whole
sequence while disarmed: the board refuses every move ("SROT_MOVE refused: arm
first"), the hull sits still, the run reports "complete", and the pool slot is
gone.

This is J02's shape at the top of the stack -- the one call a step exists to make,
with its failure swallowed -- and it is the single worst place for it, because
arm is the precondition for everything after it.
"""

from types import SimpleNamespace

import pytest

from mongla_planner.client import MoveFailed
from mongla_planner.mongla_dsl import MonglaMission


def _result(ok, msg='NOT_ARMED_AFTER_ACK: pre-arm check failed'):
    return SimpleNamespace(success=ok, message=msg, final_value=0.0, error_value=0.0)


class _Client:
    """Fails exactly the verbs named; everything else succeeds."""
    def __init__(self, fail=()):
        self.sent, self._fail = [], set(fail)

    def send(self, cmd, **kw):
        self.sent.append(cmd)
        return _result(cmd not in self._fail)


class _Log:
    def __init__(self): self.errors = []
    def info(self, m): pass
    def warn(self, m): pass
    def warning(self, m): pass
    def error(self, m): self.errors.append(m)


def _mission(fail=()):
    d = MonglaMission.__new__(MonglaMission)
    d.client, d.log, d._scoreboard = _Client(fail), _Log(), []
    d.camera, d.target = 'forward', ''
    return d


def test_a_failed_arm_raises_instead_of_returning():
    d = _mission(fail={'arm'})
    with pytest.raises(MoveFailed) as exc:
        d.arm()
    assert 'arm failed' in str(exc.value)
    assert 'pre-arm' in str(exc.value), 'the refusal reason must survive into the error'


def test_the_reason_is_logged_as_an_ERROR_not_swallowed():
    """A pool-side operator reads the log, not the traceback."""
    d = _mission(fail={'arm'})
    with pytest.raises(MoveFailed):
        d.arm()
    assert d.log.errors, 'a failed arm must log at ERROR'
    assert 'ARM FAILED' in d.log.errors[0]
    assert 'disarmed' in d.log.errors[0], 'say WHY aborting beats continuing'


def test_no_further_verb_is_sent_after_a_failed_arm():
    """The whole point: the sequence must not continue."""
    d = _mission(fail={'arm'})
    with pytest.raises(MoveFailed):
        d.arm()
    assert d.client.sent == ['arm'], f'verbs leaked after a failed arm: {d.client.sent[1:]}'


def test_a_successful_arm_is_unchanged():
    d = _mission()
    res = d.arm()
    assert res.success is True
    assert d.client.sent == ['arm']


def test_required_False_preserves_the_old_behaviour_for_diagnostics():
    """A diagnostic that wants to OBSERVE a refusal must still be able to."""
    d = _mission(fail={'arm'})
    res = d.arm(required=False)
    assert res.success is False


def test_disarm_is_deliberately_NOT_given_the_same_treatment():
    """disarm's failure is handled by mission.py's isolated _safe_shutdown steps.

    Raising there would abort the very cleanup that exists to run after a
    failure -- the opposite of the fix. Recorded so the asymmetry reads as a
    decision rather than an oversight.
    """
    d = _mission(fail={'disarm'})
    res = d.disarm()                      # must NOT raise
    assert res.success is False
