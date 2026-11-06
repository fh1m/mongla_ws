"""B30 -- every Writers callable must work on the board, and carry the right value.

WHAT WENT WRONG. `make_writers` once closed over RC-override methods SrotFC never
implemented. The vision arrival brake (`motion_vision._brake_axis`) is the one
path that calls `writers.forward` / `writers.lateral`, and it is ON BY DEFAULT,
so a plain `vision_align(lat=0)` raised AttributeError on arrival -- but only on
a FAST approach (a gently converged lock is under the brake deadband), which is
the hardest shape to catch on a bench.

So these BUILD the writers on a real SrotFC and CALL every one of them, then
check the number that leaves, not merely that a frame did.
"""
from types import SimpleNamespace

import pytest

from mongla_control.fc.srot_fc import SrotFC
from mongla_control.motion_writers import make_writers


class _FakeMaster:
    def __init__(self, sent):
        self.messages = {}
        self.mav = SimpleNamespace(
            manual_control_send=lambda t, x, y, z, r, b: sent.append(('manual', x, y, z, r)),
            heartbeat_send=lambda *a: None)


def _srot_fc(sent=None):
    return SrotFC(_FakeMaster(sent if sent is not None else []))


@pytest.mark.parametrize('field', ['forward', 'lateral', 'neutral'])
def test_every_writer_is_callable_on_srot(field):
    w = make_writers(_srot_fc())
    fn = getattr(w, field)
    try:
        fn(25.0) if field in ('forward', 'lateral') else fn()
    except AttributeError as exc:                       # the B30 signature
        pytest.fail(f'Writers.{field} calls something SrotFC lacks: {exc}')


def test_the_vision_arrival_brake_does_not_raise():
    """The exact reproduction: default brake, momentum above the deadband."""
    from mongla_control.motion_vision import _brake_axis
    sent = []
    w = make_writers(_srot_fc(sent))
    _brake_axis(w.lateral, ema_pct=40.0, brake_gain=1.0,
                abort_fn=lambda: False, log=None, label='VBRK')
    assert sent, 'the brake produced no frame at all'
    assert sent[-1][0] == 'manual', 'the brake must leave as MANUAL_CONTROL'


def test_the_brake_kick_carries_the_RIGHT_value_not_merely_a_frame():
    """ema +40 % at gain 1.0 is capped at VISION_BRAKE_CAP_PCT (30): a -30 %
    kick, i.e. -0.30 of full scale, i.e. MANUAL_CONTROL y = -300."""
    from mongla_control.motion_vision import VISION_BRAKE_CAP_PCT, _brake_axis
    sent = []
    w = make_writers(_srot_fc(sent))
    _brake_axis(w.lateral, ema_pct=40.0, brake_gain=1.0,
                abort_fn=lambda: False, log=None, label='VBRK')
    kicks = [f for f in sent if f[0] == 'manual' and f[2] != 0]
    assert kicks, 'no lateral kick was sent'
    _, x, y, z, r = kicks[-1]
    assert y == -int(VISION_BRAKE_CAP_PCT) * 10, f'lateral {y}, expected -300'
    assert x == 0 and r == 0, 'a lateral brake must not command surge or yaw'


@pytest.mark.parametrize('pct', [-100, -33.3, -0.1, 0.1, 12.5, 100])
def test_percent_maps_to_the_unit_axis_without_rescaling(pct):
    """The loops compute in percent; the wire is -1..1. Drift here silently
    rescales every vision command."""
    seen = []
    fc = SimpleNamespace(manual=lambda **kw: seen.append(kw), send_neutral=lambda: None)
    make_writers(fc).forward(pct)
    assert seen[-1]['fwd'] == pytest.approx(pct / 100.0)
    assert seen[-1]['lat'] == seen[-1]['up'] == seen[-1]['yaw'] == 0.0
