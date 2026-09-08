"""B30 -- every Writers callable must work on the backend it was built for.

WHAT WENT WRONG. `make_writers` closed over `send_rc_override` /
`send_rc_translation` unconditionally, and **SrotFC implements neither**. The
vision arrival brake (`motion_vision._brake_axis`) is the one path that calls
`writers.forward` / `writers.lateral` on srot, and it is ON BY DEFAULT
(`brake=not bool(brake_off)`, with `brake_off=False`). A plain
`vision_align(lat=0)` therefore raised

    AttributeError: 'SrotFC' object has no attribute 'send_rc_override'

on arrival, on the vehicle's primary vision verb.

⛔ WHY IT HID FOR SO LONG, AND WHY THESE TESTS ARE BEHAVIOURAL, NOT STRUCTURAL:

  * `writers.neutral` maps to `send_neutral`, which SrotFC DOES have -- so three
    of the four callables worked and the bundle looked fine.
  * `_brake_axis` returns early when `abs(ema_pct) < VISION_BRAKE_MIN_PCT`, so a
    gently converged lock never reaches the call. It failed on a FAST approach
    and passed on a slow one -- intermittent by momentum, which is the hardest
    shape to catch on a bench.
  * The facade drift test (`test_no_facade_mode_gate_is_reachable_on_srot` and
    its derived sibling) scans `duburi.py` / `vision_verbs.py` for
    `self.pixhawk.<attr>`. These calls live in `motion_writers`, behind a lambda,
    reached through a Writers field -- invisible to that scan.

So: BUILD the writers for each backend and CALL every one of them. A closure that
names a method the backend lacks cannot survive being invoked.
"""

from types import SimpleNamespace

import pytest

from duburi_control.fc.srot_fc import SrotFC
from duburi_control.motion_writers import make_writers, is_srot


class _FakeMaster:
    def __init__(self, sent):
        self.messages = {}
        self.mav = SimpleNamespace(
            manual_control_send=lambda t, x, y, z, r, b: sent.append(('manual', x, y, z, r)),
            heartbeat_send=lambda *a: None)


def _srot_fc(sent=None):
    return SrotFC(_FakeMaster(sent if sent is not None else []))


@pytest.mark.parametrize('release_yaw', [False, True])
@pytest.mark.parametrize('field', ['forward', 'lateral', 'neutral', 'depth_keepalive'])
def test_every_writer_is_callable_on_srot(field, release_yaw):
    """The bug was one field out of four. Exercise all of them, both lock states."""
    w = make_writers(_srot_fc(), release_yaw=release_yaw)
    fn = getattr(w, field)
    try:
        fn(1600) if field in ('forward', 'lateral') else fn()
    except AttributeError as exc:                       # the B30 signature
        pytest.fail(f'Writers.{field} calls something SrotFC lacks: {exc}')


def test_the_vision_arrival_brake_no_longer_raises_on_srot():
    """The exact reproduction: default brake, momentum above the deadband."""
    from duburi_control.motion_vision import _brake_axis

    sent = []
    w = make_writers(_srot_fc(sent), release_yaw=False)
    _brake_axis(w.lateral, ema_pct=40.0, brake_gain=1.0,
                abort_fn=lambda: False, log=None, label='VBRK')
    assert sent, 'the brake produced no frame at all'
    assert sent[-1][0] == 'manual', 'srot must brake through MANUAL_CONTROL'


def test_the_brake_kick_carries_the_RIGHT_value_not_merely_a_frame():
    """Not-crashing is not correct. Check the number against the cap.

    ema +40% with gain 1.0 is capped at VISION_BRAKE_CAP_PCT (30), giving a
    -30% kick -> pwm 1380 -> back to -30% -> -0.30 units -> mc -300. The pwm
    round trip must be exact, or the brake is quietly weaker or stronger than
    the ArduSub path it mirrors.
    """
    from duburi_control.motion_vision import _brake_axis, VISION_BRAKE_CAP_PCT

    sent = []
    w = make_writers(_srot_fc(sent), release_yaw=False)
    _brake_axis(w.lateral, ema_pct=40.0, brake_gain=1.0,
                abort_fn=lambda: False, log=None, label='VBRK')
    _, x, y, z, r = sent[-1]
    assert y == -int(VISION_BRAKE_CAP_PCT) * 10, f'lateral {y}, expected -300'
    assert x == 0 and r == 0, 'a lateral brake must not command surge or yaw'


def test_pwm_percent_round_trip_is_exact_across_the_band():
    """The srot writers invert percent_to_pwm; drift here silently rescales thrust."""
    from duburi_control.pixhawk import Pixhawk
    for pct in range(-100, 101, 5):
        pwm = Pixhawk.percent_to_pwm(pct)
        assert (pwm - 1500) / 4.0 == pytest.approx(pct, abs=0.26), \
            f'{pct}% -> {pwm} -> {(pwm - 1500) / 4.0}%'


def test_is_srot_has_exactly_one_definition():
    """Two copies of "which backend am I on" is how the split goes wrong in one
    file and not the other -- which is precisely what B30 was."""
    from duburi_control import motion_vision, motion_writers
    assert motion_vision._is_srot is motion_writers.is_srot
    assert is_srot(_srot_fc()) is True
    assert is_srot(SimpleNamespace(name='pixhawk')) is False
