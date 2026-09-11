"""Make a task survive its own failures. The idiom, without the tree library.

Distilled from a census of BumblebeeAS's behaviour trees: 43 `Retry` wrappers
around service calls, 37 `FailureIsSuccess` on cleanup and on whole tasks, and
every task tree ending `Selector[precise_sequence, blind_fire]` -- *"fire both
torpedoes blindly so the score doesn't go to zero on a perception miss."*

⛔ WHY NOT py_trees. Our missions are imperative `detected()` scripts and the
FSM layer wraps those same verbs as states. Adopting a tree library to get
three guarantees would be a new layer for behaviour we can express in three
functions, and the guarantees are what matter: retry the flaky thing, never let
cleanup kill a run, and always leave a scoring attempt on the table.

⛔ AND THE ONE THING THAT MUST NOT BE WRAPPED. A safety verb -- disarm, stop,
surface -- is not a step that may fail quietly. `never_fails` is for cleanup
that would otherwise mask an exit, not for the abort path, and wrapping the
latter would turn a failed disarm into a clean-looking finish. That is why this
module takes callables and never verb names: it cannot be pointed at `disarm`
by configuration.

Pure Python. No ROS.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence


@dataclass
class Attempt:
    """What happened, for the scorecard and for the operator reading the log."""
    name: str
    ok: bool
    tries: int = 0
    elapsed_s: float = 0.0
    error: str = ''
    branch: str = ''          # which selector branch produced the outcome


def retry(fn: Callable[[], object], *, times: int = 3,
          settle_s: float = 0.0, log: Optional[Callable] = None,
          name: str = '') -> Attempt:
    """Run `fn` until it returns truthy, up to `times`.

    ⛔ RETRIES ARE FOR FLAKY, NOT FOR WRONG. Their 2026 gate cut retries from 3
    to 1 and lengthened the settle from 3 s to 15 s, which is the lesson: a
    step that fails because the hull has not stopped moving does not need
    another go, it needs time. So `settle_s` waits BETWEEN tries and the
    default count is small. Three attempts at a genuinely broken step is three
    times the delay and the same failure.

    An exception counts as a failed try, not a crash: a service that is not up
    yet raises, and that is exactly the case worth retrying.
    """
    t0 = time.monotonic()
    last = ''
    for i in range(1, max(1, int(times)) + 1):
        try:
            if fn():
                return Attempt(name, True, tries=i,
                               elapsed_s=time.monotonic() - t0)
            last = 'returned falsy'
        except Exception as exc:            # noqa: BLE001 -- a try may raise
            last = f'{type(exc).__name__}: {exc}'
        if log and i < times:
            log(f'[RETRY] {name or "step"} attempt {i}/{times} failed '
                f'({last}); settling {settle_s:.1f}s')
        if settle_s > 0.0 and i < times:
            time.sleep(settle_s)
    return Attempt(name, False, tries=max(1, int(times)),
                   elapsed_s=time.monotonic() - t0, error=last)


def never_fails(fn: Callable[[], object], *, log: Optional[Callable] = None,
                name: str = '') -> Attempt:
    """Run `fn`; a raise becomes a recorded failure, never a propagated one.

    For CLEANUP and for whole optional tasks. A camera restore that raises in a
    `finally` masks the real exit; a bin task that raises takes the torpedo
    task down with it. Neither is worth a run.

    ⚠ Never wrap an abort path in this. A failed disarm that reports success is
    the one failure this stack cannot tolerate.
    """
    t0 = time.monotonic()
    try:
        ok = bool(fn())
        return Attempt(name, ok, tries=1, elapsed_s=time.monotonic() - t0,
                       error='' if ok else 'returned falsy')
    except Exception as exc:                # noqa: BLE001 -- that is the point
        if log:
            log(f'[GUARD] {name or "step"} failed and was contained: '
                f'{type(exc).__name__}: {exc}')
        return Attempt(name, False, tries=1, elapsed_s=time.monotonic() - t0,
                       error=f'{type(exc).__name__}: {exc}')


def selector(branches: Sequence, *, log: Optional[Callable] = None,
             name: str = '') -> Attempt:
    """First branch that succeeds wins. `branches` is [(label, callable), ...].

    ⛔ THE LAST BRANCH IS THE POINT. Their every task tree ends
    `Selector[precise_sequence, blind_fire]`, and the reason is scoring, not
    elegance: a perception miss should cost the precision points, not all of
    them. Put the aimed shot first and the unaimed one last, and the run stops
    being all-or-nothing.

    A branch that RAISES is a failed branch, not a failed selector -- otherwise
    the fallback, which exists precisely for when things go wrong, would never
    be reached.
    """
    t0 = time.monotonic()
    last = ''
    for label, fn in branches:
        try:
            if fn():
                if log:
                    log(f'[SEL  ] {name or "task"}: {label} succeeded')
                return Attempt(name, True, tries=1,
                               elapsed_s=time.monotonic() - t0, branch=label)
            last = f'{label} returned falsy'
        except Exception as exc:            # noqa: BLE001
            last = f'{label} raised {type(exc).__name__}: {exc}'
        if log:
            log(f'[SEL  ] {name or "task"}: {last}; trying the next branch')
    return Attempt(name, False, tries=len(branches),
                   elapsed_s=time.monotonic() - t0, error=last)
