"""Tests for the abort-interruptible sleep on the safety-stop path.

A long settle must not swallow an abort for its whole duration — `_interruptible_sleep` polls the abort predicate and returns early.
These assert the early-exit so the safety change is actually exercised.
"""

import time

from mongla_control.motion_writers import _interruptible_sleep


def _elapsed(fn, *a, **k):
    t0 = time.monotonic()
    fn(*a, **k)
    return time.monotonic() - t0


def test_sleep_runs_full_duration_without_abort():
    dt = _elapsed(_interruptible_sleep, 0.3, None)
    assert dt >= 0.28          # ran ~the full 0.3 s


def test_sleep_returns_immediately_if_already_aborted():
    dt = _elapsed(_interruptible_sleep, 5.0, lambda: True)
    assert dt < 0.1            # never waited the 5 s


def test_sleep_wakes_when_abort_flips_midway():
    flip_at = time.monotonic() + 0.15
    dt = _elapsed(_interruptible_sleep, 5.0,
                  lambda: time.monotonic() >= flip_at)
    assert dt < 0.5            # woke shortly after the 0.15 s flip, not at 5 s
