"""SrotFC backend tests against a FakeSrotMaster (no board, no ROS).

Covers the load-bearing logic that must be right before the first bench session:
the MANUAL_CONTROL mapping, the four-terminal ACK state machine (incl. the three
non-ACCEPTED terminals that would otherwise HANG an action), telemetry decode
(radians->deg, alt=-depth), and host-side NaN/verb rejection.
"""

import math
import time
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from duburi_control.fc.srot_fc import SrotFC, _build_params, MOVE_VERBS
from duburi_control.fc import srot_protocol as sp
from duburi_control.fc.base import (SUCCEEDED, PREEMPTED, FAILED, DENIED,
                                    TIMEOUT, ABORTED)

_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
_ARM_CMD = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM


# --------------------------------------------------------------------------- #
#  Fakes                                                                       #
# --------------------------------------------------------------------------- #
class _FakeMav:
    def __init__(self, master):
        self._m = master
        self.sent = []

    def command_long_send(self, sysid, comp, cmd, conf, *p):
        self.sent.append(('cmd', cmd, p))
        # Simulate the board installing a COMMAND_ACK for this command, if scripted.
        if self._m.auto_ack is not None and self._m.auto_ack.command == cmd:
            self._m.messages['COMMAND_ACK'] = self._m.auto_ack

    def manual_control_send(self, target, x, y, z, r, buttons):
        self.sent.append(('manual', x, y, z, r, buttons))

    def heartbeat_send(self, *a):
        self.sent.append(('heartbeat', a))

    def param_set_send(self, sysid, comp, pid, value, ptype):
        self.sent.append(('param_set', pid, value))
        self._m.messages['PARAM_VALUE'] = SimpleNamespace(
            param_id=b'JS_GAIN_DEFAULT', param_value=value)


class _FakeMaster:
    def __init__(self):
        self.messages = {}
        self.auto_ack = None       # SimpleNamespace(command=, result=, progress=)
        self.mav = _FakeMav(self)


def _fc():
    return SrotFC(_FakeMaster())


def _ack(result, command=sp.CMD_SROT_MOVE, progress=100):
    return SimpleNamespace(command=command, result=result, progress=progress,
                           _timestamp=time.time())


# --------------------------------------------------------------------------- #
#  MANUAL_CONTROL mapping                                                       #
# --------------------------------------------------------------------------- #
def test_manual_maps_axes_and_z_neutral():
    fc = _fc()
    fc.manual(fwd=1.0, lat=-1.0, up=0.0, yaw=0.5)
    kind, x, y, z, r, btn = fc.master.mav.sent[-1]
    assert kind == 'manual'
    assert (x, y, r) == (1000, -1000, 500)   # fwd/ lat/ yaw scaled to +/-1000
    assert z == 500                          # up=0 -> heave neutral 500
    assert btn == 0


def test_manual_up_is_high_z():
    fc = _fc()
    fc.manual(fwd=0, lat=0, up=1.0, yaw=0)     # full ascend
    assert fc.master.mav.sent[-1][3] == 1000
    fc.manual(fwd=0, lat=0, up=-1.0, yaw=0)    # full descend
    assert fc.master.mav.sent[-1][3] == 0


# --------------------------------------------------------------------------- #
#  move() -- the four-terminal ACK state machine                               #
# --------------------------------------------------------------------------- #
def test_move_accepted_succeeds_and_reports_progress():
    fc = _fc()
    fc.master.auto_ack = _ack(sp.ACK_ACCEPTED)
    seen = []
    res = fc.move('move_forward', duration=3.0, gain=50.0,
                  on_progress=seen.append)
    assert res.code == SUCCEEDED and res.ok
    assert seen and seen[-1] == 1.0            # progress driven to 1.0 on success
    # The forward SROT_MOVE was actually sent with the right type + speed.
    cmd = [s for s in fc.master.mav.sent if s[0] == 'cmd'][0]
    assert cmd[1] == sp.CMD_SROT_MOVE
    assert cmd[2][0] == sp.MOVE_FORWARD and cmd[2][2] == pytest.approx(0.5)


def test_move_cancelled_is_preempted_not_hang():
    fc = _fc()
    fc.master.auto_ack = _ack(sp.ACK_CANCELLED)
    res = fc.move('move_forward', duration=3.0, gain=50.0)
    assert res.code == PREEMPTED and not res.ok


def test_move_failed_is_failed_with_statustext():
    fc = _fc()
    fc.master.messages['STATUSTEXT'] = SimpleNamespace(text=b'No depth sensor')
    fc.master.auto_ack = _ack(sp.ACK_FAILED)
    res = fc.move('set_depth', target=-1.0)
    assert res.code == FAILED
    assert 'No depth sensor' in res.reason


def test_move_denied_on_unknown_verb():
    res = _fc().move('teleport')
    assert res.code == DENIED and 'no SROT_MOVE mapping' in res.reason


def test_move_denied_on_nonfinite_param():
    res = _fc().move('move_forward', duration=float('nan'), gain=50.0)
    assert res.code == DENIED and 'non-finite' in res.reason
    # A NaN goal must NOT have reached the wire.
    assert not [s for s in _fc().master.mav.sent if s[0] == 'cmd']


def test_move_abort_sends_stop_and_returns_aborted():
    fc = _fc()
    fc.master.auto_ack = _ack(sp.ACK_ACCEPTED)
    # abort trips before the terminal is read -> a STOP (type 6) brake is sent.
    res = fc.move('move_forward', duration=3.0, gain=50.0, abort_fn=lambda: True)
    assert res.code == ABORTED
    stops = [s for s in fc.master.mav.sent
             if s[0] == 'cmd' and s[1] == sp.CMD_SROT_MOVE and s[2][0] == sp.MOVE_STOP]
    assert stops, 'abort must brake the vehicle (SROT_MOVE stop)'


def test_move_timeout_when_no_terminal_ack():
    fc = _fc()   # no auto_ack scripted -> never resolves
    res = fc.move('move_forward', duration=0.0, gain=50.0, timeout=0.05)
    assert res.code == TIMEOUT


# --------------------------------------------------------------------------- #
#  arm / disarm / mode                                                          #
# --------------------------------------------------------------------------- #
def test_arm_succeeds_when_heartbeat_shows_armed():
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=_ARMED, custom_mode=sp.MODE_AUTO, _timestamp=time.time())
    ok, reason = fc.arm(timeout=1.0)
    assert ok and reason == 'ARMED'


def test_arm_rejected_surfaces_prearm_statustext():
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=sp.MODE_STABILIZE, _timestamp=time.time())
    fc.master.messages['STATUSTEXT'] = SimpleNamespace(text=b'PreArm: gyro cal')
    fc.master.auto_ack = _ack(sp.ACK_FAILED, command=_ARM_CMD)
    ok, reason = fc.arm(timeout=0.3)
    assert not ok and 'PreArm: gyro cal' in reason


def test_arm_abort_disarms_and_returns_aborted():
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=0, _timestamp=time.time())
    ok, reason = fc.arm(timeout=1.0, abort=lambda: True)
    assert not ok and reason == 'ABORTED'
    disarms = [s for s in fc.master.mav.sent
               if s[0] == 'cmd' and s[1] == _ARM_CMD and s[2][0] == 0.0]
    assert disarms, 'an aborted arm must command a disarm (never strand armed)'


def test_set_mode_confirms_via_heartbeat():
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=sp.MODE_DEPTH_HOLD, _timestamp=time.time())
    ok, reason = fc.set_mode('DEPTH_HOLD', timeout=1.0)
    assert ok and reason == 'DEPTH_HOLD'


def test_set_mode_unknown_name_fails_fast():
    ok, reason = _fc().set_mode('WARP', timeout=0.1)
    assert not ok and 'unknown SROT mode' in reason


# --------------------------------------------------------------------------- #
#  telemetry decode                                                             #
# --------------------------------------------------------------------------- #
def test_telemetry_decodes_attitude_depth_and_mode():
    fc = _fc()
    now = time.time()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=_ARMED, custom_mode=sp.MODE_AUTO, _timestamp=now)
    fc.master.messages['ATTITUDE'] = SimpleNamespace(
        yaw=math.radians(90.0), roll=0.0, pitch=0.0)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(alt=-2.5)   # alt = -depth
    fc.master.messages['BATTERY_STATUS'] = SimpleNamespace(voltages=[16000])
    t = fc.telemetry()
    assert t.armed is True and t.mode == 'AUTO'
    assert t.yaw_deg == pytest.approx(90.0)
    assert t.depth_m == pytest.approx(2.5)        # negated from alt
    assert t.battery_voltage == pytest.approx(16.0)
    assert t.link_alive is True


def test_telemetry_reads_esc_rpm_and_leak():
    fc = _fc()
    fc.master.messages['ESC_STATUS'] = SimpleNamespace(rpm=[1200, 1180, 1210, 1195])
    fc.master.messages['NAMED_VALUE_FLOAT'] = SimpleNamespace(name='LEAK', value=1.0)
    t = fc.telemetry()
    assert t.rpm == (1200, 1180, 1210, 1195)
    assert t.leak is True


# --------------------------------------------------------------------------- #
#  the verb table (pure, no master)                                             #
# --------------------------------------------------------------------------- #
def test_build_params_forward_speed_from_gain():
    p1, p2, p3, p4, p5 = _build_params('move_forward', {'duration': 3.0, 'gain': 60.0})
    assert p1 == sp.MOVE_FORWARD and p2 == 3.0 and p3 == pytest.approx(0.6)


def test_build_params_set_depth_negates_to_positive_dive():
    p1, p2, *_ = _build_params('set_depth', {'target': -1.6})
    assert p1 == sp.MOVE_DIVE and p2 == pytest.approx(1.6)


def test_build_params_set_depth_rejects_above_surface():
    with pytest.raises(ValueError):
        _build_params('set_depth', {'target': 0.5})    # positive = above surface


def test_build_params_turn_is_absolute():
    p1, p2, p3, p4, p5 = _build_params('turn', {'target': 90.0})
    assert p1 == sp.MOVE_TURN and p2 == 90.0 and p4 == float(sp.TURN_ABSOLUTE)


def test_build_params_yaw_left_is_relative_negative():
    p1, p2, p3, p4, p5 = _build_params('yaw_left', {'target': 30.0})
    assert p1 == sp.MOVE_TURN and p2 == -30.0 and p4 == float(sp.TURN_RELATIVE)


def test_move_verbs_membership():
    assert 'move_forward' in MOVE_VERBS and 'set_depth' in MOVE_VERBS
    assert 'arm' not in MOVE_VERBS and 'vision_align' not in MOVE_VERBS


def test_build_params_arc_uses_target_yaw_and_style_uses_flips():
    p1, p2, p3, p4, p5 = _build_params('arc', {'duration': 4.0, 'gain': 40.0,
                                               'target_yaw': 20.0})
    assert p1 == sp.MOVE_ARC and p4 == 20.0            # signed yaw rate from target_yaw
    s1, s2, *_ = _build_params('style_roll', {'flips': 2.0})
    assert s1 == sp.MOVE_STYLE and s2 == 2.0           # count from flips


# --------------------------------------------------------------------------- #
#  Pixhawk-compatible surface (lets the manager treat SrotFC as a drop-in)      #
# --------------------------------------------------------------------------- #
def test_get_attitude_returns_degrees_dict():
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(
        yaw=math.radians(45.0), roll=0.0, pitch=0.0)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(alt=-1.2)
    att = fc.get_attitude()
    assert att['yaw'] == pytest.approx(45.0) and att['depth'] == pytest.approx(1.2)


def test_get_attitude_none_without_attitude_msg():
    assert _fc().get_attitude() is None


def test_get_mode_and_battery():
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=sp.MODE_STABILIZE, _timestamp=time.time())
    fc.master.messages['BATTERY_STATUS'] = SimpleNamespace(voltages=[15200])
    assert fc.get_mode() == 'STABILIZE'
    assert fc.get_battery() == pytest.approx(15.2)


def test_noop_writes_do_not_raise_and_send_nothing():
    fc = _fc()
    fc.set_message_rate(30, 50)          # no-op on SROT (fixed rates)
    fc.send_att_pos_mocap(90.0)          # no-op (board fuses BNO on-board)
    assert fc.get_rc_channels() is None
    assert not fc.master.mav.sent        # neither reached the wire


def test_send_heartbeat_emits_gcs_heartbeat():
    fc = _fc()
    fc.send_heartbeat()
    assert any(s[0] == 'heartbeat' for s in fc.master.mav.sent)
