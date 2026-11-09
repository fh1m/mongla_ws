"""`settle=N` is a board HOLD for N seconds after the move's ACK (issue #86).

The keyword was accepted by every board verb, forwarded by the DSL and the
facade, and dropped at the wire: `yaw_right(180, settle=1)` started its next
leg while the hull still turned. These drive `SrotFC.move` against a fake
board that ACKs each SROT_MOVE and heartbeats after it, and count what went
out on the wire -- not source text.
"""

import time

import pytest

from mongla_control.fc import srot_protocol as sp
from mongla_control.fc.base import DENIED, FAILED, PREEMPTED, SUCCEEDED

from test_srot_fc import _ack  # noqa: E402  (shared fakes)
from test_srot_move_respects_surface import (  # noqa: E402,F401
    _fast, _fc_with_board, _hb, _moves_sent)


def _board(fc, mode_after=lambda n: 'AUTO', on_send=None):
    """ACK every SROT_MOVE ACCEPTED, then heartbeat in `mode_after(n)`."""
    master = fc.master
    orig = master.mav.command_long_send

    def send(*a):
        orig(*a)
        n = len(_moves_sent(fc))
        if on_send is not None:
            on_send(n)
        master.messages['COMMAND_ACK'] = _ack(sp.ACK_ACCEPTED)
        master.messages['HEARTBEAT'] = _hb(mode_after(n), t=time.time() + 0.01)
    master.mav.command_long_send = send


def _types(fc):
    return [int(s[2][0]) for s in _moves_sent(fc)]


def test_settle_zero_sends_exactly_the_move():
    fc = _fc_with_board('AUTO')
    _board(fc)
    res = fc.move('move_forward', duration=2.0, gain=50.0, settle=0.0)
    assert res.code == SUCCEEDED
    assert _types(fc) == [sp.MOVE_FORWARD]
    assert 'held' not in res.reason


@pytest.mark.parametrize('verb,kw', [
    ('move_forward', dict(duration=2.0, gain=50.0)),
    ('yaw_right', dict(duration=1.0, gain=30.0)),
    ('set_depth', dict(target=-1.0)),
])
def test_settle_holds_on_the_board_for_n_seconds(verb, kw):
    fc = _fc_with_board('AUTO')
    _board(fc)
    res = fc.move(verb, settle=1.5, **kw)
    assert res.code == SUCCEEDED, res.reason
    sent = _moves_sent(fc)
    assert len(sent) == 2, 'the move, then one HOLD'
    hold = sent[1][2]
    assert int(hold[0]) == sp.MOVE_HOLD
    assert hold[1] == pytest.approx(1.5), 'the HOLD lasts the settle'
    assert 'held 1.5 s' in res.reason
    assert 'settled' not in res.reason, 'time held is not a measured stop'


def test_an_abort_after_the_move_sends_no_hold():
    fc = _fc_with_board('AUTO')
    _board(fc)
    flag = {'abort': False}
    orig = fc._relay_move_ack

    def relay(*a, **k):
        r = orig(*a, **k)
        flag['abort'] = True               # the operator aborts at the ACK
        return r
    fc._relay_move_ack = relay
    res = fc.move('move_forward', duration=2.0, gain=50.0, settle=3.0,
                  abort_fn=lambda: flag['abort'])
    assert res.code == PREEMPTED
    assert _types(fc) == [sp.MOVE_FORWARD], 'no HOLD after an abort'


def test_a_failsafe_surface_during_the_move_is_never_undone_by_the_hold():
    """The move is cut short by SURFACE: no HOLD goes out, because a SROT_MOVE
    would put the board back in AUTO and re-dive."""
    fc = _fc_with_board('AUTO')
    _board(fc, mode_after=lambda n: 'SURFACE')
    res = fc.move('set_depth', target=-1.0, settle=2.0)
    assert res.code == FAILED and 'SURFACE' in res.reason
    assert _types(fc) == [sp.MOVE_DIVE]


def test_a_hold_the_board_cuts_short_fails_the_verb():
    """SURFACE lands during the HOLD: the verb reports it, not 'held'."""
    fc = _fc_with_board('AUTO')
    _board(fc, mode_after=lambda n: 'AUTO' if n == 1 else 'SURFACE')
    res = fc.move('yaw_right', duration=1.0, gain=30.0, settle=2.0)
    assert res.code == FAILED
    assert 'did not complete' in res.reason and 'held' not in res.reason


@pytest.mark.parametrize('bad', [-1.0, float('nan'), float('inf'), 'soon'])
def test_a_bad_settle_is_refused_before_anything_is_sent(bad):
    fc = _fc_with_board('AUTO')
    _board(fc)
    res = fc.move('move_forward', duration=2.0, gain=50.0, settle=bad)
    assert res.code == DENIED
    assert not _moves_sent(fc)
