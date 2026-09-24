"""A motion verb must never undo SURFACE, and a move a failsafe cut short
must never be reported as completed.

Firmware facts these guard against (srot-control-board f1d3ba9):
  * mav_commands.cpp: the SROT_MOVE handler sets `mode = AUTO` for EVERY
    move type, STOP included -- so the next mission leg, or a plain `stop`,
    pulls the board out of a leak/battery/GCS failsafe SURFACE and re-dives.
  * mav_stream.cpp: a move whose AUTO was displaced (failsafe, baro loss,
    disarm) is latched done and ACKed ACCEPTED at 100 %.
"""

import time
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from mongla_control.fc import srot_protocol as sp
from mongla_control.fc.base import DENIED, FAILED, SUCCEEDED
from mongla_control.fc.srot_fc import SrotFC

from test_srot_fc import _FakeMaster, _ack  # noqa: E402  (shared fakes)

_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    import mongla_control.fc.srot_fc as mod
    monkeypatch.setattr(mod, '_ACK_MIN_BUDGET_S', 0.3)
    monkeypatch.setattr(mod, '_ACK_MARGIN_S', 0.2)
    monkeypatch.setattr(mod, '_POST_ACK_HB_WAIT_S', 0.1)


def _hb(mode, armed=True, t=None):
    return SimpleNamespace(custom_mode=sp.MODE_INTS[mode],
                           base_mode=_ARMED if armed else 0,
                           autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                           _timestamp=time.time() if t is None else t)


def _fc_with_board(mode, armed=True):
    m = _FakeMaster()
    m.messages['HEARTBEAT'] = _hb(mode, armed)
    return SrotFC(m)


def _moves_sent(fc):
    return [s for s in fc.master.mav.sent
            if s[0] == 'cmd' and s[1] == sp.CMD_SROT_MOVE]


@pytest.mark.parametrize('verb,kw', [
    ('move_forward', dict(duration=3.0, gain=50.0)),
    ('set_depth', dict(target=-1.0)),
    ('yaw_left', dict(duration=1.0, gain=30.0)),
])
def test_no_move_is_sent_into_surface(verb, kw):
    fc = _fc_with_board('SURFACE')
    fc.master.auto_ack = _ack(sp.ACK_ACCEPTED)
    res = fc.move(verb, **kw)
    assert res.code == DENIED and 'SURFACE' in res.reason
    assert not _moves_sent(fc), 'a SROT_MOVE would have switched the board to AUTO'


def test_stop_in_surface_is_a_no_op_not_a_re_dive():
    fc = _fc_with_board('SURFACE')
    res = fc.move('stop')
    assert res.code == SUCCEEDED
    assert not _moves_sent(fc)
    fc.stop_motion()
    assert not _moves_sent(fc)


def test_moves_still_go_out_in_normal_modes():
    fc = _fc_with_board('AUTO')
    fc.master.auto_ack = _ack(sp.ACK_ACCEPTED)
    res = fc.move('move_forward', duration=3.0, gain=50.0)
    assert res.code == SUCCEEDED
    assert _moves_sent(fc)


def test_a_move_cut_short_by_a_failsafe_surface_is_failed_not_completed():
    """The ACK says ACCEPTED 100 %, but the heartbeat after it says SURFACE."""
    fc = _fc_with_board('AUTO')
    master = fc.master
    orig = master.mav.command_long_send

    def send(*a):
        orig(*a)
        # The board: failsafe -> SURFACE, the move is cancelled and latched done.
        master.messages['COMMAND_ACK'] = _ack(sp.ACK_ACCEPTED)
        master.messages['HEARTBEAT'] = _hb('SURFACE', t=time.time() + 0.01)
    master.mav.command_long_send = send
    res = fc.move('move_forward', duration=3.0, gain=50.0)
    assert res.code == FAILED, res.reason
    assert 'SURFACE' in res.reason


def test_a_move_cut_short_by_a_disarm_is_failed():
    fc = _fc_with_board('AUTO')
    master = fc.master
    orig = master.mav.command_long_send

    def send(*a):
        orig(*a)
        master.messages['COMMAND_ACK'] = _ack(sp.ACK_ACCEPTED)
        master.messages['HEARTBEAT'] = _hb('AUTO', armed=False, t=time.time() + 0.01)
    master.mav.command_long_send = send
    res = fc.move('move_forward', duration=3.0, gain=50.0)
    assert res.code == FAILED and 'DISARM' in res.reason


def test_a_genuinely_completed_move_still_succeeds():
    fc = _fc_with_board('AUTO')
    master = fc.master
    orig = master.mav.command_long_send

    def send(*a):
        orig(*a)
        master.messages['COMMAND_ACK'] = _ack(sp.ACK_ACCEPTED)
        master.messages['HEARTBEAT'] = _hb('AUTO', t=time.time() + 0.01)
    master.mav.command_long_send = send
    assert fc.move('move_forward', duration=3.0, gain=50.0).code == SUCCEEDED


def test_the_manager_asks_for_a_fast_heartbeat():
    from mongla_manager.auv_manager_node import SROT_MESSAGE_RATES
    hz = SROT_MESSAGE_RATES.get(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)
    assert hz >= 10, 'at 1 Hz a failsafe SURFACE is invisible for up to a second'
