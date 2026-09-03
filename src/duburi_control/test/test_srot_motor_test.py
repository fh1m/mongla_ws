"""`DO_MOTOR_TEST` -- the one procedure that identifies a thruster by turning it.

Fully implemented on the board the whole time, with no driver method here, so
it was reachable only from Bondor. The interesting property is not the command,
it is the KEEP-ALIVE: the board expires the test when the resends stop, and
expiry auto-disarms. That is the safety mechanism, so the driver has to own the
cadence -- a `while` loop in a caller that wedges is exactly the shape that
holds a thruster spinning.
"""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.fc import srot_protocol as sp     # noqa: E402
from duburi_control.fc.srot_fc import SrotFC          # noqa: E402


class _Mav:
    """Answers ON SEND, the way the link does.

    The driver calls `_clear_ack()` before each send, so a fake that plants an
    ACK up front has it erased and never answers -- which is the staleness
    guard working correctly. (Third time this bit me in this codebase; the fix
    is always to model the reply arriving AFTER the request.)
    """

    def __init__(self, master):
        self.sent = []
        self._m = master

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)

        def _send(*a, **k):
            self.sent.append((time.monotonic(), name, a, k))
            if name == 'command_long_send' and self._m.ack is not None:
                self._m.messages['COMMAND_ACK'] = type(
                    'A', (), {'command': a[2], 'result': self._m.ack})()
                # The board's explanatory text arrives with the ACK, not before
                # it -- `_clear_ack()` pops STATUSTEXT too, because that slot is
                # what separates "busy, retry" from "arm first".
                if self._m.text is not None:
                    self._m.messages['STATUSTEXT'] = type(
                        'S', (), {'text': self._m.text})()
        return _send


class _Master:
    def __init__(self, armed=True, ack=None, text=None):
        self.ack = ack
        self.text = text
        self.mav = _Mav(self)
        self.messages = {}


def _fc(armed=True, ack=sp.ACK_ACCEPTED, text=None):
    fc = SrotFC(_Master(ack=ack, text=text), log=None)
    fc.is_armed = lambda: armed
    return fc


def _tests(fc):
    return [a for _, n, a, _ in fc.master.mav.sent
            if n == 'command_long_send' and a[2] == sp.CMD_DO_MOTOR_TEST]


# --------------------------------------------------------------------------- #
#  Preconditions
# --------------------------------------------------------------------------- #
def test_a_disarmed_board_is_refused_without_sending_anything():
    """The board refuses too ("Arm motors before testing motors."), but saying
    it here means the caller does not have to spin a motor to find out."""
    fc = _fc(armed=False)
    ok, why = fc.motor_test(1, 15.0, seconds=0.1)
    assert ok is False and 'ARMED' in why
    assert _tests(fc) == [], 'nothing should reach the wire while disarmed'


@pytest.mark.parametrize('n', [0, 9, -1])
def test_a_motor_outside_1_to_8_is_refused(n):
    """1-BASED, matching DO_SET_SERVO and SERVOn_ROLE. An off-by-one here spins
    the wrong thruster, which on a restrained hull is how you conclude the
    wiring is wrong when it is not."""
    ok, why = _fc().motor_test(n, 15.0, seconds=0.1)
    assert ok is False and 'range' in why


@pytest.mark.parametrize('pct', [101.0, -101.0, float('nan'), float('inf')])
def test_an_impossible_throttle_is_refused(pct):
    """A NaN on this wire reaches a 500 Hz loop."""
    ok, _ = _fc().motor_test(1, pct, seconds=0.1)
    assert ok is False


def test_a_refusal_from_the_board_is_reported_with_its_own_text():
    fc = _fc(ack=sp.ACK_FAILED, text=b'Arm motors before testing motors.')
    ok, why = fc.motor_test(1, 15.0, seconds=0.4)
    assert ok is False and 'FAILED' in why and 'Arm motors' in why


# --------------------------------------------------------------------------- #
#  The keep-alive -- the part that matters
# --------------------------------------------------------------------------- #
def test_it_resends_faster_than_the_board_expires():
    """The board's window is clamped to 600..3000 ms, so a resend slower than
    ~2 Hz lets the test expire mid-press -- which auto-disarms, and reads as
    the vehicle disarming on its own."""
    fc = _fc()
    fc.motor_test(3, 20.0, seconds=1.0)
    sends = _tests(fc)
    assert len(sends) >= 3, f'only {len(sends)} keep-alives in 1 s'
    assert sp.MOTOR_TEST_KEEPALIVE_HZ >= 2.0


def test_the_keep_alive_STOPS_when_the_call_returns():
    """THE test the whole design is for. The driver owns the cadence precisely
    so that a caller which wedges cannot hold a thruster spinning: when this
    returns, nothing further reaches the wire, and the board expires and
    disarms within one clamped window.

    The observation window is deliberately MUCH longer than the test
    itself. An earlier version ran a 0.5 s test and watched for 0.5 s,
    and a deliberately injected leaking thread passed it -- the leak
    simply finished while the call was still running, so there was
    nothing left to escape. A test that can only see a leak shorter
    than itself does not test for leaks.
    """
    fc = _fc()
    fc.motor_test(1, 15.0, seconds=0.25)
    n_at_return = len(_tests(fc))
    time.sleep(1.5)          # 6x the test duration -- see below
    assert len(_tests(fc)) == n_at_return, 'a keep-alive escaped the call'


def test_an_abort_stops_it_immediately():
    """A cancelled command must not keep a motor turning for its full duration."""
    fc = _fc()
    t0 = time.monotonic()
    ok, why = fc.motor_test(1, 15.0, seconds=10.0, abort_fn=lambda: True)
    assert ok is False and 'aborted' in why
    assert time.monotonic() - t0 < 1.0, 'abort did not take effect promptly'


def test_the_window_we_ask_for_is_inside_the_firmwares_clamp():
    """p4 is SECONDS and the firmware clamps it to 600..3000 ms whatever we
    say. Asking for something outside that is not an error -- it is silently
    changed, which is the PM1_VMULT shape. Pin the arithmetic instead."""
    fc = _fc()
    fc.motor_test(1, 15.0, seconds=0.4)
    p4_seconds = _tests(fc)[0][7]        # p1..p7 start at index 4
    ms = p4_seconds * 1000.0
    assert sp.MOTOR_TEST_WINDOW_MIN_MS <= ms <= sp.MOTOR_TEST_WINDOW_MAX_MS


def test_the_firmwares_clamp_values_are_pinned():
    assert (sp.MOTOR_TEST_WINDOW_MIN_MS, sp.MOTOR_TEST_WINDOW_MAX_MS) == (600, 3000)


# --------------------------------------------------------------------------- #
#  What goes on the wire
# --------------------------------------------------------------------------- #
def test_the_parameters_are_in_ArduPilot_order():
    """p1 = motor (1-based), p2 = throttle TYPE, p3 = throttle, p4 = window s.
    Getting p2 wrong is not an error: type 1 means raw PWM microseconds, so a
    percent value would be read as a pulse width near the bottom of the band."""
    fc = _fc()
    fc.motor_test(5, 25.0, seconds=0.4)
    a = _tests(fc)[0]
    assert a[2] == sp.CMD_DO_MOTOR_TEST
    # command_long_send(sysid, compid, command, confirmation, p1..p7)
    assert a[4] == pytest.approx(5.0)                              # p1 motor
    assert a[5] == pytest.approx(sp.MOTOR_TEST_THROTTLE_PERCENT)   # p2 type
    assert a[6] == pytest.approx(25.0)                             # p3 throttle


def test_the_success_message_says_the_board_will_disarm():
    """"The vehicle disarmed when I let go" is the DESIGNED end state (it
    mirrors ArduSub's verify_motor_test), not a fault -- so the message says so
    rather than leaving the operator to diagnose it."""
    ok, why = _fc().motor_test(2, 10.0, seconds=0.4)
    assert ok is True and 'disarm' in why.lower()


def test_nothing_is_sent_after_an_abort_either():
    """The abort path must stop the cadence for the same reason the normal exit
    does, and it is the path a cancelled mission takes."""
    fc = _fc()
    fc.motor_test(1, 15.0, seconds=5.0, abort_fn=lambda: True)
    n_at_return = len(_tests(fc))
    time.sleep(1.0)
    assert len(_tests(fc)) == n_at_return
