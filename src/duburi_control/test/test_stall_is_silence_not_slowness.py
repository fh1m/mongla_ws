"""B39 -- a stall must mean SILENCE, not slowness.

MAVLink command protocol (https://mavlink.io/en/services/command.html):

    "The GCS should have a much increased timeout after receiving an ACK with
     MAV_RESULT_IN_PROGRESS."

`_relay_move_ack` used to arm ONE deadline before the loop and never extend it.
A move that was demonstrably alive -- the board reporting rising progress every
tick -- was declared stalled and BRAKED the moment it passed ~2x its predicted
time. Braking a healthy manoeuvre mid-leg is worse than waiting for it: the leg
is lost and the mission continues from somewhere unplanned.

The deadline is now a SILENCE window refreshed by genuine progress, under an
absolute ceiling (`_STALL_HARD_MULT`) so a board that reports progress for ever
still terminates.

⛔ THE STALE-ACK TRAP IS WHY THIS IS DELICATE. `_cache()` returns the SAME
COMMAND_ACK object on every poll until a new one lands. Refreshing the deadline on
every *sighting* of an IN_PROGRESS would let ONE stale ACK hold it open for ever,
converting the backstop into a hang. The refresh therefore lives inside the
`prog != last_prog` branch, and `test_a_single_stale_in_progress_still_stalls`
exists precisely to catch a future "simplification" that moves it out.
"""

import threading
import time

import pytest

import duburi_control.fc.srot_fc as srot_mod
import duburi_control.fc.srot_protocol as sp
from test_srot_fc import _ack, _fc


@pytest.fixture(autouse=True)
def _short_budget(monkeypatch):
    """~1.4 s budget for a 0.2 s move, so these run in seconds not minutes."""
    monkeypatch.setattr(srot_mod, '_ACK_MARGIN_S', 1.0)
    monkeypatch.setattr(srot_mod, '_ACK_MIN_BUDGET_S', 1.0)


def _move_while(fc, feeder):
    t = threading.Thread(target=feeder, args=(fc,), daemon=True)
    t.start()
    return fc.move('move_forward', duration=0.2, gain=50)


def test_a_silent_board_still_stalls():
    """The backstop must survive the fix: no ACKs at all is still a stall."""
    res = _move_while(_fc(), lambda fc: None)
    assert res.code == srot_mod.TIMEOUT


def test_a_single_stale_in_progress_still_stalls():
    """ONE IN_PROGRESS that never advances is a WEDGED board, not a working one."""
    def one_then_silence(fc):
        time.sleep(0.2)
        fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_IN_PROGRESS, progress=10)

    t0 = time.monotonic()
    res = _move_while(_fc(), one_then_silence)
    elapsed = time.monotonic() - t0

    assert res.code == srot_mod.TIMEOUT
    # ⛔ CHECKING THE CODE ALONE DOES NOT CATCH THIS, and the first version of this
    # test did exactly that. If the deadline is refreshed on every SIGHTING of the
    # cached ACK, it is held open for ever and the run is terminated by the HARD
    # CAP instead -- same TIMEOUT code, ~4x later. Verified: the broken version
    # passed a code-only assertion. The elapsed time is the only thing that tells
    # "stalled on silence" apart from "ran to the ceiling".
    budget_ish = srot_mod._ACK_MIN_BUDGET_S + srot_mod._ACK_MARGIN_S
    assert elapsed < budget_ish * (1.0 + srot_mod._STALL_HARD_MULT) / 2.0, (
        f'stalled after {elapsed:.1f}s -- that is the hard cap, not the silence '
        f'window. A stale cached ACK is refreshing the deadline.')


def test_a_slow_but_PROGRESSING_move_is_no_longer_braked():
    """The actual regression: alive for ~4 s against a ~1.4 s budget."""
    def slow_but_alive(fc):
        for p in range(10, 60, 10):
            time.sleep(0.8)
            fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_IN_PROGRESS, progress=p)
        fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_ACCEPTED)

    t0 = time.monotonic()
    res = _move_while(_fc(), slow_but_alive)
    elapsed = time.monotonic() - t0

    assert res.code == srot_mod.SUCCEEDED, \
        'a board reporting real progress must not be declared stalled'
    assert elapsed > 2.0, \
        'this test is only meaningful if it ran PAST the original budget'


def test_endless_progress_is_still_bounded_by_the_hard_cap():
    """A board that progresses for ever must not hold the action thread for ever."""
    def forever(fc):
        p = 0
        while True:
            p = (p + 7) % 100
            time.sleep(0.3)
            fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_IN_PROGRESS, progress=p)

    t0 = time.monotonic()
    res = _move_while(_fc(), forever)
    elapsed = time.monotonic() - t0

    assert res.code == srot_mod.TIMEOUT
    assert elapsed < 30.0, 'the absolute ceiling must bound the worst case'


def test_the_hard_cap_is_a_multiple_of_the_budget_not_a_magic_number():
    assert srot_mod._STALL_HARD_MULT > 1.0, \
        'the ceiling must leave room for at least one refresh, or the fix is inert'
