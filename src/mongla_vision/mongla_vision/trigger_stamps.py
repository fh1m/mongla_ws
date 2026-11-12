"""Which trigger made this frame: pairing camera frames with the board's pulses.

When the control board triggers the camera (upstream `pr-w`), it also says on
the wire when each pulse fired: `CAMERA_TRIGGER`, a sequence number and the
board's time. A frame reaches the host some milliseconds later, with no
number on it. This module answers the one question in between: which pulse
exposed this frame, so the frame can carry the instant it was taken instead
of the instant it arrived.

It works in one time base. The caller maps the board's trigger times onto the
host clock first (`flow_timing.ClockMap` today, `TIMESYNC` when the board
answers it) and passes host seconds for both triggers and arrivals.

The method, and its limit. A frame from pulse `s` arrives `L` after it, where
`L` (exposure, readout, USB, decode) is stable and can be bounded by
measurement: `latency_lo_s <= L <= latency_hi_s`. A pulse is this frame's
pulse when it lies in `[arrival - hi, arrival - lo]`. If that window is
narrower than the trigger period it holds at most one pulse, and the pairing
is exact with nothing learned and nothing to drift. If the window is as wide
as the period, times alone cannot tell pulse `s` from `s - 1`, and this
refuses every frame and says why: at 210 Hz the latency has to be known to
better than 4.8 ms, which is a measurement to make on the bench, not a
default to pick.

Pulses the wire did not deliver are filled from the ones it did: the trigger
is crystal-timed, so pulse `s` fired at `t(nearest heard) + (s - nearest) * P`.
That covers a lost message and a board that reports only every Nth pulse. It
stops `max_gap_s` after the last pulse heard, because a silent link may be a
board that stopped triggering, and a stamp invented for a pulse that never
fired is the plausible number this codebase keeps getting hurt by.

It never guesses: zero pulses in the window, or two, is `None`, counted under
its reason in `refused`.

What it cannot see. The bounds are a measurement the caller owns. A camera
slower than its stated bounds by a whole period lands the previous pulse in
the window, and that is paired, wrongly, with nothing here able to tell. So
the bounds are measured per camera and per mode on the bench (a strobe or an
LED the board fires on a known pulse), and re-measured when either changes.
"""
from __future__ import annotations

from collections import deque
from typing import NamedTuple, Optional


class Stamp(NamedTuple):
    seq: int               # the board's trigger count
    t_s: float             # when that pulse fired, host seconds
    interpolated: bool     # the pulse was not heard; its time is from the fit


class TriggerPairer:
    """Assign each arriving frame the trigger pulse that exposed it."""

    def __init__(self, latency_lo_s: float, latency_hi_s: float,
                 max_gap_s: float = 0.5, keep: int = 512):
        if not 0.0 <= latency_lo_s < latency_hi_s:
            raise ValueError(
                f'latency bounds must satisfy 0 <= lo < hi, got '
                f'({latency_lo_s}, {latency_hi_s})')
        self._lo = float(latency_lo_s)
        self._hi = float(latency_hi_s)
        self._max_gap = float(max_gap_s)
        self._heard: deque = deque(maxlen=int(keep))     # (seq, t_s), by seq
        self.refused = {'no_triggers': 0, 'window_not_narrower_than_period': 0,
                        'none_in_window': 0, 'two_in_window': 0,
                        'link_silent': 0}
        self.resets = 0

    def note_trigger(self, seq: int, t_s: float) -> None:
        """One `CAMERA_TRIGGER`, its time already on the host clock."""
        seq, t_s = int(seq), float(t_s)
        if self._heard:
            last_seq, last_t = self._heard[-1]
            if seq <= last_seq or t_s <= last_t:
                # The count went back or time did: the board rebooted, or the
                # clock map was refitted. Pulses from before are another
                # numbering, so nothing is paired across the break.
                self._heard.clear()
                self.resets += 1
        self._heard.append((seq, t_s))

    def period_s(self) -> Optional[float]:
        """The trigger period, from the first and last pulse heard."""
        if len(self._heard) < 2:
            return None
        (s0, t0), (s1, t1) = self._heard[0], self._heard[-1]
        return (t1 - t0) / (s1 - s0)

    def stamp(self, t_arrival_s: float) -> Optional[Stamp]:
        """The pulse that exposed the frame arriving now, or None."""
        period = self.period_s()
        if period is None:
            self.refused['no_triggers'] += 1
            return None
        if (self._hi - self._lo) >= period:
            self.refused['window_not_narrower_than_period'] += 1
            return None
        last_t = self._heard[-1][1]
        first_seq = self._heard[0][0]
        early, late = t_arrival_s - self._hi, t_arrival_s - self._lo
        if early - last_t > self._max_gap:
            self.refused['link_silent'] += 1
            return None
        # Candidate counts, from the line through the nearest pulse heard.
        anchor_seq, anchor_t = min(
            self._heard, key=lambda p: abs(p[1] - (early + late) / 2.0))
        centre = anchor_seq + ((early + late) / 2.0 - anchor_t) / period
        found = []
        for seq in range(int(centre) - 1, int(centre) + 3):
            if seq < first_seq:
                continue
            heard = self._time_heard(seq)
            t = heard if heard is not None else (
                anchor_t + (seq - anchor_seq) * period)
            if early <= t <= late:
                found.append(Stamp(seq, t, heard is None))
        if len(found) == 1:
            return found[0]
        self.refused['none_in_window' if not found else 'two_in_window'] += 1
        return None

    def _time_heard(self, seq: int) -> Optional[float]:
        for s, t in reversed(self._heard):
            if s == seq:
                return t
            if s < seq:
                break
        return None
