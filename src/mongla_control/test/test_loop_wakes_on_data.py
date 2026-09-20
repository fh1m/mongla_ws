"""The control loop waits for an OBSERVATION, not for a clock.

WHY
---
A fixed-rate poll against an asynchronous producer waits, on average, half a
period for data that has already arrived. Measured on this vehicle:
detections land at ~77 Hz (13 ms apart) and the srot control loop ticked at
50 Hz (20 ms), so every command was computed from an observation up to 13 ms
older than the one available -- about a third of the entire detection age,
spent waiting for a timer.

It is the same defect that a fixed-rate timer caused in `camera_node`, where
it measured 8.4 ms, one layer further down the same chain. The vehicle's
reaction time is the SUM of these, so each one is worth removing on its own.

WHAT MUST NOT CHANGE
--------------------
The timeout is a FLOOR, not a rate. The loop's time-based work -- freshness
decay, hold timing, the arrival brake, the overall deadline -- has to keep
running when nothing is being detected, which is exactly the searching/lost
case. So the loop may only get FASTER, never slower, and with no vision state
at all it must behave precisely as it did.
"""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control import motion_vision as MV        # noqa: E402

_SROT_PERIOD = 1.0 / 50.0


class _Fc:
    """Enough of a flight controller for `_loop_hz` to pick the srot rate."""
    backend = 'srot'
    name = 'srot'


def _period(fc):
    return 1.0 / MV._loop_hz(fc)


class _State:
    def __init__(self, returns):
        self.returns = returns
        self.timeouts = []

    def wait_for_sample(self, timeout):
        self.timeouts.append(timeout)
        if self.returns:
            return True          # a detection was already waiting
        time.sleep(timeout)      # nothing arrived; behave like the old sleep
        return False


def test_a_waiting_detection_wakes_the_loop_immediately():
    """THE POINT. If a sample is ready the loop must not sit out the rest of
    its period -- that wait is the latency being removed."""
    st = _State(returns=True)
    t0 = time.monotonic()
    MV._tick(st, _Fc())
    took = time.monotonic() - t0
    assert took < 0.005, f'{took * 1000:.1f} ms -- still waiting on the clock'
    assert st.timeouts, 'the loop did not consult the vision state at all'


def test_no_detection_still_paces_at_the_tick():
    """The floor. With nothing arriving the loop must keep its old cadence,
    because freshness decay, the brake and the deadline all live on it."""
    st = _State(returns=False)
    t0 = time.monotonic()
    MV._tick(st, _Fc())
    took = time.monotonic() - t0
    assert took >= _period(_Fc()) * 0.8, f'{took * 1000:.1f} ms -- too fast'
    assert st.timeouts[0] == pytest.approx(_period(_Fc()), rel=0.01)


def test_without_a_vision_state_it_is_exactly_the_old_sleep():
    """Unit tests and the non-vision callers pass None. Nothing may depend on
    the wake-up existing."""
    t0 = time.monotonic()
    MV._tick(None, _Fc())
    took = time.monotonic() - t0
    assert took >= _period(_Fc()) * 0.8


def test_a_state_without_the_method_falls_back_rather_than_raising():
    """An older VisionState, a stub, a mock. A missing optional capability is
    not an error -- it is the pre-change behaviour."""
    t0 = time.monotonic()
    MV._tick(object(), _Fc())
    assert time.monotonic() - t0 >= _period(_Fc()) * 0.8


def test_no_fixed_rate_sleep_survives_in_the_loops():
    """All six sites, not five. A single missed one is a path that still pays
    the old wait -- and it would be the searching path, where reaction time
    matters most."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_control'
           / 'motion_vision.py').read_text()
    assert 'time.sleep(1.0 / _loop_hz(' not in src
    assert src.count('_tick(vision_state, pixhawk)') >= 6


def test_dt_is_measured_not_assumed():
    """`dt` feeds the continuity-lock gate and the lateral integral. Once the
    loop runs at detection rate instead of its nominal tick, a hard-coded
    period overstates both by up to 1.5x."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_control'
           / 'motion_vision.py').read_text()
    assert 'dt = min(max(now - _last_pass' in src, \
        'dt is not derived from the clock'
