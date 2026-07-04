"""run_fsm -- run a YASMIN state machine with a GUARANTEED safe exit.

Every FSM launcher used to do ``outcome = sm(Blackboard())`` bare. That is
unsafe: a state can terminate ``ABORT`` *normally* (e.g. a SurfaceState whose
``disarm()`` raised → ``execute()`` catches it → returns ABORT), so the FSM
returns an outcome STRING rather than raising -- and the ``mission.py`` runner's
emergency disarm lives only in its ``except`` clause, which never fires on a
normal return. Result: an FSM mission could end with the vehicle ARMED and no
backstop.

``run_fsm`` closes that gap: it runs the machine and, in a ``finally``,
best-effort releases the heading lock and disarms -- so an FSM mission can never
exit armed regardless of how it terminated (SUCCEED / ABORT / TIMEOUT / raise).
Both cleanup steps are idempotent (release on no lock and disarm on an
already-disarmed hull are safe), and disarm rides the bounded action client
(MoveTimeout), so the safe exit itself can never hang.
"""

from __future__ import annotations

from yasmin import Blackboard


def run_fsm(duburi, sm, log=None) -> str:
    """Run ``sm`` and ALWAYS release-heading + disarm on the way out.

    Returns the FSM outcome string. ``log`` (if given) receives one completion
    line. The finally block is the single safety backstop for every FSM
    launcher -- individual launchers no longer need their own try/finally.
    """
    _log = log or (lambda *_a, **_k: None)
    try:
        outcome = sm(Blackboard())
        _log(f'[FSM] mission complete: {outcome}')
        return outcome
    finally:
        # Guaranteed safe exit -- covers a normal ABORT return (runner's
        # except-only backstop won't fire) AND an FSM that raised.
        try:
            duburi.release_heading()
        except Exception:   # noqa: BLE001 -- cleanup must never raise
            pass
        try:
            duburi.disarm()
        except Exception:   # noqa: BLE001 -- cleanup must never raise
            pass
