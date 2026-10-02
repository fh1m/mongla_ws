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
from typing import Callable, List, Optional


@dataclass
class _Event:
    t: float
    fn: Callable
    before: dict           # filter snapshot taken just before `fn` ran
    # A PROPAGATION over (t - span, t], and how to rebuild it over any other
    # duration -- so a late measurement can land INSIDE it (see `run`). None
    # for an update, which happens at an instant.
    span: float = 0.0
    make: Optional[Callable] = None


def snapshot(filt) -> dict:
    """Everything an update can change, including the gate's own counters --
    a replay must not count one measurement's rejection twice. The filter
    names the list (`RIEKF.REPLAY_STATE`), so it cannot drift from here."""
    return filt.replay_state()


def restore(filt, snap: dict) -> None:
    filt.set_replay_state(snap)


class Retrodictor:
    """Event log over `horizon_s` of filter time, with replay on a late event."""

    def __init__(self, filt, horizon_s: float = 2.0):
        self.filt = filt
        self.horizon_s = float(horizon_s)
        self._events: List[_Event] = []
        self.replayed = 0          # events re-run because something arrived late
        self.late = 0              # events inserted behind the newest
        self.refused = 0           # older than the horizon: not applied

    def run(self, t: float, fn: Callable, *, span: float = 0.0,
            make: Optional[Callable] = None) -> bool:
        """Apply `fn(filter)` as of time `t`. False if `t` is too old to reach.

        A propagation passes `span` (its duration, ending at `t`) and `make`
        (`make(duration) -> fn`), which is what lets a later, late measurement
        split it.
        """
        ev = self._events
        if not ev or t >= ev[-1].t:
            self._exec_append(t, fn, span, make)
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
        first = tail[0]
        if make is None and first.make is not None and first.t - first.span < t:
            # ⛔ THE LATE EVENT LANDS INSIDE A PROPAGATION (issue #31). Applying
            # it before that step evaluated it at the PREVIOUS IMU sample, up
            # to one step (20 ms at 50 Hz) earlier than it happened. Split the
            # step at `t`: propagate to `t`, apply, propagate the rest. The
            # IMU sample is held across both halves, exactly as the unsplit
            # step held it, so this introduces no new approximation.
            head = t - (first.t - first.span)
            self._exec_append(t, first.make(head), head, first.make)
            self._exec_append(t, fn, span, make)
            rest = first.t - t
            self._exec_append(first.t, first.make(rest), rest, first.make)
            tail = tail[1:]
            self.replayed += 1
        else:
            self._exec_append(t, fn, span, make)
        for e in tail:
            self._exec_append(e.t, e.fn, e.span, e.make)
        self.replayed += len(tail)
        return True

    def newest_t(self):
        return self._events[-1].t if self._events else None

    def _exec_append(self, t: float, fn: Callable, span: float = 0.0,
                     make: Optional[Callable] = None) -> None:
        before = snapshot(self.filt)
        fn(self.filt)
        self._events.append(_Event(t, fn, before, span, make))

    def _prune(self) -> None:
        ev = self._events
        cutoff = ev[-1].t - self.horizon_s
        k = 0
        while k < len(ev) - 1 and ev[k].t < cutoff:
            k += 1
        if k:
            del ev[:k]
