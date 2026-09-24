"""Apply every filter event at the instant it describes, not the instant it arrived.

⛔ THE DEFECT THIS FIXES. The invariant filter predicted on the IMU at its board
stamp but applied flow, depth, fixes and headings ON ARRIVAL. Flow is stamped at
the exposure midpoint and lands 30-60 ms later; a prop fix is computed from a
detection and lands later still. Applying either to the CURRENT state compares a
measurement of the past against a prediction of the present, and the motion in
between becomes innovation the filter believes is error.

THE METHOD, and why this shape. Standard retrodiction for delayed measurements:
restore the state as it was at the measurement's time, apply it, re-propagate
forward (FusionCore, arXiv 2605.25239; the Stone Soup OOSM example). The usual
form re-propagates buffered IMU only. That is NOT enough here: the node also
applies the board's attitude on every IMU sample, and ZUPT and depth between
them, so dropping those on replay would release attitude and diverge (measured on
the vehicle without the attitude update: 7.1e6 m in 35 s). So this buffers EVERY
event -- predicts and updates alike -- each with the filter snapshot taken just
before it ran, and a late event is inserted at its time and the tail replayed in
order.

The cost is proportional to how LATE a measurement is, not to the horizon: a flow
sample 50 ms late replays ~2-3 IMU steps and their attitude updates. The horizon
only bounds memory and how late a fix may still arrive and be used.

Pure Python + the filter. No ROS.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass
from typing import Callable, List


@dataclass
class _Event:
    t: float
    fn: Callable
    before: tuple          # filter snapshot taken just before `fn` ran


def snapshot(filt) -> tuple:
    """Everything an update can change, including the gate's own counters --
    a replay must not count one measurement's rejection twice."""
    return (filt.X.copy(), filt.P.copy(), filt.accepted, filt.rejected,
            dict(filt.reject_streak), filt.lockout_breaks)


def restore(filt, snap: tuple) -> None:
    X, P, acc, rej, streak, breaks = snap
    filt.X = X.copy()
    filt.P = P.copy()
    filt.accepted, filt.rejected = acc, rej
    # A COPY: the snapshot is restored on every replay across it, and handing
    # the live dict back would let the replay's rejections write into it.
    filt.reject_streak, filt.lockout_breaks = dict(streak), breaks


class Retrodictor:
    """Event log over `horizon_s` of filter time, with replay on a late event."""

    def __init__(self, filt, horizon_s: float = 2.0):
        self.filt = filt
        self.horizon_s = float(horizon_s)
        self._events: List[_Event] = []
        self.replayed = 0          # events re-run because something arrived late
        self.late = 0              # events inserted behind the newest
        self.refused = 0           # older than the horizon: not applied

    def run(self, t: float, fn: Callable) -> bool:
        """Apply `fn(filter)` as of time `t`. False if `t` is too old to reach."""
        ev = self._events
        if not ev or t >= ev[-1].t:
            self._exec_append(t, fn)
            self._prune()
            return True
        if t < ev[0].t:
            self.refused += 1
            return False
        i = bisect.bisect_right([e.t for e in ev], t)
        restore(self.filt, ev[i].before)
        tail = ev[i:]
        del ev[i:]
        self.late += 1
        self._exec_append(t, fn)
        for e in tail:
            self._exec_append(e.t, e.fn)
        self.replayed += len(tail)
        return True

    def newest_t(self):
        return self._events[-1].t if self._events else None

    def _exec_append(self, t: float, fn: Callable) -> None:
        before = snapshot(self.filt)
        fn(self.filt)
        self._events.append(_Event(t, fn, before))

    def _prune(self) -> None:
        ev = self._events
        cutoff = ev[-1].t - self.horizon_s
        k = 0
        while k < len(ev) - 1 and ev[k].t < cutoff:
            k += 1
        if k:
            del ev[:k]
