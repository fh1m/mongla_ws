#!/usr/bin/env python3
"""What the vision loops write to the board, and how they wait.

Every write is one MANUAL_CONTROL frame. A frame carries all four axes, so a
single-axis write (a brake kick) zeroes the others -- which is what a kick
wants, because its caller goes neutral immediately after. The board holds
attitude, heading and depth itself at 500 Hz; these frames add translation
and yaw on top.

Arguments are PERCENT of thrust, -100..100, the unit the loops compute in.
"""

import time
from dataclasses import dataclass
from typing import Callable

# The arrival brake: a reverse kick this long bleeds the hull's momentum.
REVERSE_KICK_SEC = 0.20


@dataclass
class Writers:
    forward: Callable[[float], None]
    lateral: Callable[[float], None]
    neutral: Callable[[], None]


def make_writers(fc) -> Writers:
    return Writers(
        forward=lambda pct: fc.manual(fwd=pct / 100.0, lat=0.0, up=0.0, yaw=0.0),
        lateral=lambda pct: fc.manual(fwd=0.0, lat=pct / 100.0, up=0.0, yaw=0.0),
        neutral=fc.send_neutral,
    )


def _interruptible_sleep(duration, abort_fn=None, slice_s=0.05):
    """Sleep up to `duration`, but return early if `abort_fn()` goes True.

    Polls the abort predicate every `slice_s` seconds so a settle or brake on
    the safety-stop path cannot block an abort for its full duration. Monotonic
    time, so an NTP step cannot extend the wait.
    """
    deadline = time.monotonic() + duration
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        if abort_fn and abort_fn():
            return
        time.sleep(min(slice_s, remaining))
