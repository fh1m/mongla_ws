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
from duburi_control.fc.base import (FIRE_FIRED, FIRE_REJECTED_ARM, FIRE_DISABLED,
                                    FIRE_DENIED, FIRE_NO_ACK, FIRE_NOT_READY,
                                    FIRE_BUSY)
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

    def param_request_read_send(self, sysid, comp, pid, index):
        # The payload role gate reads SERVOn_ROLE off the board. `roles` defaults to
        # "every channel is a SWITCH" so the actuation tests below exercise the PULSE
        # mechanics rather than the gate; the gate has its own tests, which set roles
        # explicitly. On the real vehicle 1-8 are SERVO and 9-16 are SWITCH.
        name = pid.decode() if isinstance(pid, bytes) else str(pid)
        self.sent.append(('param_read', name, None))   # 3-tuple: callers unpack k,c,p
        self._m.messages['PARAM_VALUE'] = SimpleNamespace(
            param_id=name.encode(), param_value=float(self._m.roles.get(name, 2)))


class _FakeMaster:
    def __init__(self):
        self.messages = {}
        self.auto_ack = None       # SimpleNamespace(command=, result=, progress=)
        self.roles = {}            # 'SERVOn_ROLE' -> int; default 2 (SWITCH)
        self.mav = _FakeMav(self)


def _fc():
    return SrotFC(_FakeMaster())


def _ack(result, command=sp.CMD_SROT_MOVE, progress=100):
    return SimpleNamespace(command=command, result=result, progress=progress,
                           _timestamp=time.time())


@pytest.fixture(autouse=True)
def _fast_deadlines(monkeypatch):
    """Shrink the ACK deadline floor and the fire pulse for the whole module.

    Both are real production values (an 8 s floor so a slow first ACK is not called
    a stall; a pulse long enough for a servo to travel) -- but a test that
    deliberately drives the stall path should not sit through them. Patched here
    rather than lowered in the source, so the shipped numbers stay honest.

    (The BRAKE_* patches are gone with the host-side brake -- fw behaviour rev 2
    brakes on-board.)
    """
    import duburi_control.fc.srot_fc as mod
    monkeypatch.setattr(mod, '_ACK_MIN_BUDGET_S', 0.3)
    monkeypatch.setattr(mod, '_ACK_MARGIN_S', 0.2)
    monkeypatch.setattr(mod, '_FIRE_PULSE_S', 0.01)
    monkeypatch.setattr(mod, '_FIRE_ACK_S', 0.01)


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
    # NEGATIVE below the surface -- the stack-wide convention (DuburiState.msg,
    # set_depth's input, the surface guards). alt already carries that sign, so
    # it passes straight through; this used to be negated a second time.
    assert t.depth_m == pytest.approx(-2.5)
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


def test_build_params_move_back_is_the_reverse_axis():
    """move_back was refused for a missing branch, not a missing wire verb --
    MOVE_BACK=1 has always existed and the old host brake already commanded it."""
    p1, p2, p3, *_ = _build_params('move_back', {'duration': 2.0, 'gain': 40.0})
    assert p1 == sp.MOVE_BACK and p2 == 2.0 and p3 == pytest.approx(0.4)


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
    # arc is excluded on SROT (heading-hold vs rate-arc mismatch).
    assert 'arc' not in MOVE_VERBS


def test_style_uses_flips_and_arc_has_no_mapping():
    s1, s2, *_ = _build_params('style_roll', {'flips': 2.0})
    assert s1 == sp.MOVE_STYLE and s2 == 2.0           # count from flips
    with pytest.raises(KeyError):                       # arc must not map on SROT
        _build_params('arc', {'duration': 4.0, 'gain': 40.0, 'target_yaw': 20.0})


def test_manual_coerces_nonfinite_axis_to_neutral():
    # A NaN from a vision loop must degrade to hold, not crash the 20 Hz stream.
    fc = _fc()
    fc.manual(fwd=float('nan'), lat=0.0, up=float('inf'), yaw=0.0)
    kind, x, y, z, r, btn = fc.master.mav.sent[-1]
    assert x == 0 and z == 500                          # nan->0, inf-up->neutral 500


# --------------------------------------------------------------------------- #
#  Pixhawk-compatible surface (lets the manager treat SrotFC as a drop-in)      #
# --------------------------------------------------------------------------- #
def test_get_attitude_returns_degrees_dict():
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(
        yaw=math.radians(45.0), roll=0.0, pitch=0.0)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(alt=-1.2)
    att = fc.get_attitude()
    assert att['yaw'] == pytest.approx(45.0)
    assert att['depth'] == pytest.approx(-1.2)   # negative below surface


def test_depth_sign_matches_pixhawk_convention():
    """Submerged reads NEGATIVE on the SROT backend, exactly as AHRS2 does on
    Pixhawk. This is the regression guard for the double-negation bug: every
    depth guard in the stack (STYLE_ROLL_SURFACE_GUARD_M, motion_vision's
    _MIN_DEPTH_M / max_depth_m / depth_ceiling_m, calibrate_depth's refusal)
    compares against a NEGATIVE constant, so a positive-down reading did not
    error -- it silently stopped every one of them from ever firing."""
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(yaw=0.0, roll=0.0, pitch=0.0)
    for board_depth_m in (0.5, 2.0, 8.0):
        # The board sends alt = -depth (fw mav_stream.cpp:210-218).
        fc.master.messages['VFR_HUD'] = SimpleNamespace(alt=-board_depth_m)
        assert fc.get_attitude()['depth'] < 0.0
        assert fc.telemetry().depth_m < 0.0


def test_get_attitude_none_without_attitude_msg():
    assert _fc().get_attitude() is None


def test_get_mode_and_battery():
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=sp.MODE_STABILIZE, _timestamp=time.time())
    fc.master.messages['BATTERY_STATUS'] = SimpleNamespace(voltages=[15200])
    assert fc.get_mode() == 'STABILIZE'
    assert fc.get_battery()['voltage'] == pytest.approx(15.2)   # dict, matches Pixhawk


def test_noop_writes_do_not_raise_and_send_nothing():
    fc = _fc()
    fc.send_att_pos_mocap(90.0)          # no-op (board fuses the IMU on-board)
    assert fc.get_rc_channels() is None  # no RC input path -- no radio on the vehicle
    assert not fc.master.mav.sent        # neither reached the wire


def test_set_message_rate_sends_511():
    """No longer a no-op: fw behaviour rev 2 implements SET_MESSAGE_INTERVAL, so the
    board's 10 Hz ATTITUDE is no longer the ceiling on every host loop."""
    fc = _fc()
    fc.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
    sent = [(c, p) for k, c, p in fc.master.mav.sent if k == 'cmd']
    assert len(sent) == 1
    cmd, p = sent[0]
    assert cmd == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
    assert p[0] == float(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
    assert p[1] == pytest.approx(20000.0)      # 50 Hz -> 20 000 us


def test_set_message_rate_zero_restores_the_board_default():
    fc = _fc()
    fc.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 0)
    _, _, p = [s for s in fc.master.mav.sent if s[0] == 'cmd'][0]
    assert p[1] == 0.0


def test_send_heartbeat_emits_gcs_heartbeat():
    fc = _fc()
    fc.send_heartbeat()
    assert any(s[0] == 'heartbeat' for s in fc.master.mav.sent)


def test_get_angular_rates_uses_rate_suffixed_keys():
    # The manager's _imu_rates_tick reads rates['pitch_rate'] etc. -- a key mismatch
    # crashes the 50 Hz timer (it did, on the first live connect).
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(
        yaw=0.0, roll=0.0, pitch=0.0, rollspeed=0.1, pitchspeed=0.2, yawspeed=0.3)
    r = fc.get_angular_rates()
    assert r['pitch_rate'] == pytest.approx(0.2)
    assert r['roll_rate'] == pytest.approx(0.1) and r['yaw_rate'] == pytest.approx(0.3)
    assert 'age_s' in r


def test_get_battery_is_a_dict_like_pixhawk():
    fc = _fc()
    fc.master.messages['BATTERY_STATUS'] = SimpleNamespace(
        voltages=[15200], current_battery=350)
    b = fc.get_battery()
    assert b['voltage'] == pytest.approx(15.2) and b['current'] == pytest.approx(3.5)
    assert _fc().get_battery() is None            # None when no BATTERY_STATUS


# --------------------------------------------------------------------------- #
#  Payload over MAVLink (DO_SET_SERVO / DO_SET_RELAY) + SrotPayload             #
# --------------------------------------------------------------------------- #
def test_set_servo_sends_do_set_servo():
    fc = _fc()
    fc.set_servo(1, 2000)                          # PCA ch 0 (1-based) -> 2000 µs
    cmd = [s for s in fc.master.mav.sent if s[0] == 'cmd'][-1]
    assert cmd[1] == sp.CMD_DO_SET_SERVO and cmd[2][0] == 1.0 and cmd[2][1] == 2000.0


def test_set_relay_sends_do_set_relay():
    fc = _fc()
    fc.set_relay(0, True)
    cmd = [s for s in fc.master.mav.sent if s[0] == 'cmd'][-1]
    assert cmd[1] == sp.CMD_DO_SET_RELAY and cmd[2][0] == 0.0 and cmd[2][1] == 1.0


def test_srot_payload_fire_is_a_bounded_pulse_on_the_board_channel():
    """fire(N) drives BOARD channel N -- no map, no translation -- and always
    returns it to rest. The de-energise is in a `finally`, so it happens even when
    the board never ACKs (this fake never does), which is the case that would
    otherwise leave a solenoid coil latched on with no board-side failsafe."""
    from duburi_control.fc.srot_fc import SrotPayload
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=sp.MODE_MANUAL, _timestamp=time.time())
    pay = SrotPayload(fc)
    # Seed the role cache: this test is about the WIRE pulse, not the role read
    # (which has its own tests above). Going through get_param here would just buy
    # a 2 s timeout against a fake board that answers no PARAM_VALUE.
    pay._roles[9] = sp.PCA_ROLE_SWITCH
    res = pay.fire(9)
    servo_cmds = [c for c in fc.master.mav.sent
                  if c[0] == 'cmd' and c[1] == sp.CMD_DO_SET_SERVO]
    # channel is passed straight through, and it is a pulse: ON then OFF.
    assert [(p[0], p[1]) for _, _, p in servo_cmds] == [
        (9.0, float(sp.PCA_SWITCH_ON_US)), (9.0, float(sp.PCA_SWITCH_OFF_US))]
    # This fake board never sends COMMAND_ACK -> the outcome is UNKNOWN, not FIRED.
    assert res.code == FIRE_NO_ACK and not res.ok


def test_fire_rejects_a_channel_outside_the_boards_range():
    from duburi_control.fc.srot_fc import SrotPayload
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=sp.MODE_MANUAL, _timestamp=time.time())
    pay = SrotPayload(fc)
    for bad in (0, -1, sp.PCA9685_NUM_CH + 1):
        res = pay.fire(bad)
        assert res.code == FIRE_DENIED, bad
    assert not fc.master.mav.sent, 'an out-of-range channel reached the wire'


def test_srot_payload_is_ready_tracks_link():
    from duburi_control.fc.srot_fc import SrotPayload
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, custom_mode=0, autopilot=0, _timestamp=time.time())
    assert SrotPayload(fc).is_ready is True        # link alive -> payload ready


def test_heartbeat_from_gcs_is_ignored_for_mode_and_armed():
    # A GCS / loopback heartbeat (autopilot INVALID) on a shared link must NOT be
    # read as the vehicle. First a real vehicle HB (autopilot GENERIC=0), then a
    # GCS HB overwrites the cache -> mode/armed must still reflect the vehicle.
    fc = _fc()
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=_ARMED, custom_mode=sp.MODE_AUTO, autopilot=0, _timestamp=time.time())
    assert fc.get_mode() == 'AUTO' and fc.is_armed() is True
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(   # a GCS on the link
        base_mode=0, custom_mode=sp.MODE_MANUAL,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID, _timestamp=time.time())
    assert fc.get_mode() == 'AUTO' and fc.is_armed() is True   # vehicle state held


# --------------------------------------------------------------------------- #
#  Tier-1 safety: the abort brake, the ACK deadline, the 5th ACK result         #
# --------------------------------------------------------------------------- #
def _moves(fc):
    """Every SROT_MOVE the fake saw, as (move_type, p2, p3, p5) tuples."""
    return [(int(p[0]), p[1], p[2], p[4])
            for kind, cmd, p in fc.master.mav.sent
            if kind == 'cmd' and cmd == sp.CMD_SROT_MOVE]


def test_abort_sends_a_bare_stop_and_the_board_brakes():
    """The host-side reverse-leg brake is GONE as of fw behaviour rev 2.

    It existed because the wire-reachable MOVE_STOP coasted: the firmware zeroed
    s_uf/s_ul/s_speed before the STOP case, so PH_BRAKE computed -0*gain*0 == 0.
    Rev 2 (fw AUDIT.md R36) restores the outgoing leg's axis and speed, so STOP
    brakes on-board -- and a host brake on top would kick the hull twice.

    This pins the ABSENCE of the reverse leg, so re-adding one fails here."""
    fc = _fc()
    fc.master.auto_ack = None                     # never terminates -> abort path
    res = fc.move('move_forward', duration=3.0, gain=50.0,
                  abort_fn=lambda: True)
    assert res.code == ABORTED
    sent = _moves(fc)
    assert sent[0][0] == sp.MOVE_FORWARD          # the leg
    assert sent[-1][0] == sp.MOVE_STOP            # straight to STOP
    assert sp.MOVE_BACK not in [m[0] for m in sent], (
        'a reverse brake leg reappeared -- the board already brakes on MOVE_STOP '
        '(fw rev 2), so this would double-kick the hull')


def test_no_verb_emits_a_reverse_brake_leg():
    """Same invariant across the axes that used to be braked host-side."""
    for verb, kw, opposite in (
            ('move_left', {'duration': 2.0, 'gain': 40.0}, sp.MOVE_STRAFE_R),
            ('move_forward', {'duration': 2.0, 'gain': 40.0}, sp.MOVE_BACK)):
        fc = _fc()
        fc.master.auto_ack = None
        fc.move(verb, abort_fn=lambda: True, **kw)
        types = [m[0] for m in _moves(fc)]
        assert types[-1] == sp.MOVE_STOP
        assert opposite not in types


def test_stop_motion_does_not_block():
    """The old brake slept its full duration inside stop_motion -- and the safety
    `surface` verb calls it, so an emergency ascent waited up to BRAKE_MAX_S
    before the mode change was even sent. Nothing sleeps here now."""
    import time as _t
    fc = _fc()
    fc.master.auto_ack = None
    t0 = _t.monotonic()
    fc.stop_motion()
    assert _t.monotonic() - t0 < 0.05


def test_temporarily_rejected_is_terminal_and_says_it_is_retryable():
    """The board returns TEMPORARILY_REJECTED on a state-mutex miss at dispatch
    (fw mav_commands.cpp:287/376/422). It is NOT in JETSON_COMMS.md's four-result
    table, so it used to fall through as non-terminal and burn the whole deadline
    before reporting a bogus TIMEOUT."""
    fc = _fc()
    fc.master.auto_ack = _ack(sp.ACK_TEMPORARILY_REJECTED, progress=0)
    res = fc.move('move_forward', duration=3.0, gain=50.0)
    assert res.code == FAILED
    assert 'retry' in res.reason.lower()


def test_ack_deadline_tracks_the_leg_not_the_60s_board_default():
    """p5 is 0 for every verb whose commands.py row has no `timeout` field, so a
    3 s move used to hold the action thread 65 s on a stall. That matters because
    the host deadline is the ONLY terminator: a failsafe mid-leg takes the board
    out of AUTO and it streams IN_PROGRESS forever (fw task_control_loop.cpp:731)."""
    import duburi_control.fc.srot_fc as mod
    from duburi_control.fc.srot_fc import _ack_budget_s
    floor = mod._ACK_MIN_BUDGET_S          # the autouse fixture shrinks this
    short = _ack_budget_s('move_forward', sp.MOVE_FORWARD, 3.0, 0.0)
    assert short < 20.0
    # A long legitimate leg still gets room, and an explicit p5 is honoured.
    assert _ack_budget_s('move_forward', sp.MOVE_FORWARD, 60.0, 0.0) > 60.0
    assert _ack_budget_s('move_forward', sp.MOVE_FORWARD, 1.0, 45.0) > 45.0
    # Turn/dive budget from their own rates, not from a duration.
    assert _ack_budget_s('turn', sp.MOVE_TURN, 180.0, 0.0) > 180.0 / sp.MOVE_YAW_RATE
    assert _ack_budget_s('set_depth', sp.MOVE_DIVE, 2.0, 0.0) > 2.0 / sp.MOVE_DEPTH_RATE
    # Never below the floor, so a tiny leg still tolerates a slow first ACK.
    assert _ack_budget_s('stop', sp.MOVE_STOP, 0.0, 0.0) >= floor


def test_shipped_ack_floor_is_generous_enough_for_a_slow_board():
    """The shipped floor (not the shrunk test one) must tolerate a slow first ACK.

    Guards the fixture from hiding a regression: if someone drops the real floor to
    something like 0.5 s, a healthy-but-busy board would be reported as a stall and
    the vehicle braked mid-leg for no reason."""
    import duburi_control.fc.srot_fc as mod
    import importlib
    fresh = importlib.reload(mod)          # fixture patches are not applied here
    try:
        assert fresh._ACK_MIN_BUDGET_S >= 5.0
        assert fresh._ACK_MARGIN_S >= 2.0
    finally:
        importlib.reload(mod)


def test_stop_verb_is_a_single_bare_stop():
    """`duburi stop` is now ONE MOVE_STOP -- the board brakes it (fw rev 2).

    It used to emit a reverse leg first, because the board's end-of-leg PH_BRAKE
    computed zero thrust and a completed leg left the hull coasting. Rev 2 fixed
    that at the source, so the extra leg would now be a second kick.
    """
    fc = _fc()
    fc.master.auto_ack = _ack(sp.ACK_ACCEPTED)
    fc.move('move_forward', duration=5.0, gain=60.0)      # completes
    fc.master.mav.sent.clear()
    fc.move('stop')
    assert [m[0] for m in _moves(fc)] == [sp.MOVE_STOP]


def test_repeated_stop_stays_a_single_stop():
    """A second stop (or an abort right after one) must not add anything."""
    fc = _fc()
    fc.master.auto_ack = _ack(sp.ACK_ACCEPTED)
    fc.move('move_forward', duration=5.0, gain=60.0)
    fc.move('stop')
    fc.master.mav.sent.clear()
    fc.move('stop')
    assert [m[0] for m in _moves(fc)] == [sp.MOVE_STOP]


# --------------------------------------------------------------------------- #
#  The unsupported-verb contract                                                #
# --------------------------------------------------------------------------- #
def test_unsupported_verbs_are_disjoint_from_move_verbs():
    from duburi_control.fc.srot_fc import UNSUPPORTED_VERBS
    assert not (UNSUPPORTED_VERBS & MOVE_VERBS)
    # The ones that still lie or crash rather than refuse.
    for verb in ('lock_heading', 'arc', 'vision_align'):
        assert verb in UNSUPPORTED_VERBS
    # move_back came OFF the list: MOVE_BACK=1 was always on the wire and the verb
    # was refused only for a missing _build_params branch. It is a board verb now.
    assert 'move_back' in MOVE_VERBS and 'move_back' not in UNSUPPORTED_VERBS


def test_no_facade_mode_gate_is_reachable_on_srot():
    """INVARIANT: every facade verb that engages an ArduSub-only mode gate must be
    either collapsed to the board (MOVE_VERBS) or refused (UNSUPPORTED_VERBS).

    This is what makes the ALT_HOLD->DEPTH_HOLD alias safe. The alias exists so a
    literal `set_mode('ALT_HOLD')` in a mission works, but it also means
    `_ensure_alt_hold` / `_ensure_yaw_capable_mode` would now SUCCEED on srot --
    and the code right after each of them calls `set_target_depth` or
    `send_rc_override`, which SrotFC does not implement. Today none of those verbs
    can reach the facade, so the gates are dead code on this backend.

    If someone later removes a verb from UNSUPPORTED_VERBS without porting its
    body, this test fails instead of the vehicle finding out.
    """
    from duburi_control.fc.srot_fc import UNSUPPORTED_VERBS
    gated = {
        'set_depth', 'surface',                       # _ensure_alt_hold
        'vision_align', 'vision_move',                # _ensure_alt_hold
        'arc', 'style_roll', 'style_yaw',             # _ensure_yaw_capable_mode
        'turn', 'yaw_left', 'yaw_right',              # _ensure_yaw_capable_mode
    }
    # 'surface' is intercepted in the manager (_run_srot_surface) rather than
    # collapsed or refused -- it is the one deliberate exception.
    handled = MOVE_VERBS | UNSUPPORTED_VERBS | {'surface'}
    unreachable = gated - handled
    assert not unreachable, (
        f'{sorted(unreachable)} would reach a facade mode gate on srot, then call '
        f'a Pixhawk-only primitive. Port them or add them to UNSUPPORTED_VERBS.')


# --------------------------------------------------------------------------- #
#  Firmware behaviour revision -- the RUNTIME interlock for the removed brake   #
# --------------------------------------------------------------------------- #
# The drift test that checks this same number reads the firmware repo off disk and
# SKIPS when that repo is not checked out beside the workspace -- i.e. it skips on
# the vehicle, the one place the answer matters. These tests pin the check that
# actually runs there.

def _fc_reporting_rev(rev):
    """SrotFC whose fake board answers AUTOPILOT_VERSION with `rev`, or None for a
    board that never answers at all."""
    fc = _fc()
    if rev is not None:
        fc.master.messages['AUTOPILOT_VERSION'] = SimpleNamespace(
            middleware_sw_version=rev, flight_sw_version=131072)
    return fc


def test_behaviour_rev_is_read_from_autopilot_version():
    fc = _fc_reporting_rev(2)
    assert fc.read_behaviour_rev() == 2
    # and it requested the message rather than waiting for an unsolicited one
    assert any(kind == 'cmd' and cmd == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
               and p[0] == sp.MSG_ID_AUTOPILOT_VERSION
               for kind, cmd, p in fc.master.mav.sent)


def test_arm_is_refused_on_firmware_older_than_required():
    """THE regression this whole interlock exists for.

    Rev 1 COASTS on MOVE_STOP and we deleted `_brake_last_leg`, so `stop` and every
    abort would apply zero braking thrust to 20 kg of hull with nothing in the log.
    Arming is the last gate before anything can move, so it is where a known-bad
    board has to be turned away.
    """
    fc = _fc_reporting_rev(1)
    ok, reason = fc.arm(timeout=0.2)
    assert ok is False
    assert 'FW_BEHAVIOUR_REV_TOO_OLD' in reason
    # It must not have even attempted to arm.
    assert not any(kind == 'cmd' and cmd == _ARM_CMD
                   for kind, cmd, _p in fc.master.mav.sent)


def test_rev_zero_is_treated_as_too_old_not_as_unknown():
    """0 is what pre-2026-08-01 firmware reports -- that build never populated the
    field. Reading it as 'unknown, proceed' would let exactly the bad case through."""
    fc = _fc_reporting_rev(0)
    ok, reason = fc.arm(timeout=0.2)
    assert ok is False and 'FW_BEHAVIOUR_REV_TOO_OLD' in reason


def test_current_firmware_rev_arms_normally():
    fc = _fc_reporting_rev(sp.FW_BEHAVIOUR_REV_REQUIRED)
    fc.master.auto_ack = SimpleNamespace(
        command=_ARM_CMD, result=sp.ACK_ACCEPTED, progress=0)
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=_ARMED, autopilot=0, custom_mode=0, _timestamp=time.time())
    ok, _reason = fc.arm(timeout=1.0)
    assert ok is True


def test_a_silent_board_warns_but_does_not_brick_the_vehicle():
    """Asymmetry on purpose: 'board says rev 1' is a definite statement that stop
    will coast, and refusing is right. 'board said nothing' is far more likely a
    dropped frame, and refusing to arm on a comms hiccup is its own hazard."""
    fc = _fc_reporting_rev(None)
    warnings = []
    fc._log = SimpleNamespace(info=lambda m: None, warning=warnings.append)
    fc.master.auto_ack = SimpleNamespace(
        command=_ARM_CMD, result=sp.ACK_ACCEPTED, progress=0)
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=_ARMED, autopilot=0, custom_mode=0, _timestamp=time.time())
    ok, _reason = fc.arm(timeout=1.0)
    assert ok is True
    assert any('BEHAVIOUR_REV' in w for w in warnings), \
        'a silent board must still say so loudly'


def test_the_override_is_opt_in_and_still_shouts():
    fc = _fc_reporting_rev(1)
    warnings = []
    fc._log = SimpleNamespace(info=lambda m: None, warning=warnings.append)
    fc.allow_fw_behaviour_mismatch = True
    ok, reason = fc.check_behaviour_rev()
    assert ok is True and 'OVERRIDDEN' in reason
    assert any('OVERRIDDEN' in w for w in warnings)


# --------------------------------------------------------------------------- #
#  NAMED_VALUE_FLOAT de-multiplexing -- LEAK must not be a lottery             #
# --------------------------------------------------------------------------- #

def _nvf(name, value):
    return SimpleNamespace(name=name, value=value)


def test_named_value_survives_other_names_landing_after_it():
    """THE regression. SROT rides ~15 names on NAMED_VALUE_FLOAT and pymavlink keeps one
    message per msgid, so the old `read the slot, compare .name` returned None unless the
    value you wanted happened to be the most recent to arrive. Against LEAK that is a
    lottery on whether a flooding hull is ever observed."""
    fc = _fc()
    fc.master.messages['NAMED_VALUE_FLOAT'] = _nvf('LEAK', 1.0)
    assert fc._named_value('LEAK') == 1.0
    # ...now fourteen other names arrive and overwrite the single slot.
    for other in ('WTEMP', 'GAIN', 'CURR', 'KILL', 'HEAP', 'STK_CTL'):
        fc.master.messages['NAMED_VALUE_FLOAT'] = _nvf(other, 1.0)
        fc._drain_named()
    assert fc._named_value('LEAK') == 1.0, \
        'LEAK was lost the moment any other NAMED_VALUE_FLOAT arrived'
    assert fc._named_value('WTEMP') == 1.0


def test_named_value_ages_out():
    """A value that stopped arriving must read None, not its value from minutes ago --
    the same freshness rule every other external input in this stack follows."""
    fc = _fc()
    fc.note_named_value(_nvf('LEAK', 1.0))
    assert fc._named_value('LEAK', max_age_s=100.0) == 1.0
    fc._named_cache['LEAK'] = (1.0, time.time() - 60.0)
    assert fc._named_value('LEAK', max_age_s=3.0) is None


def test_reader_hook_is_lossless():
    """note_named_value() is the hook for the manager's reader thread, which sees EVERY
    NAMED_VALUE_FLOAT rather than only whichever last landed in pymavlink's slot."""
    fc = _fc()
    for name, val in (('LEAK', 1.0), ('WTEMP', 21.5), ('GAIN', 0.5)):
        fc.note_named_value(_nvf(name, val))
    assert fc._named_value('LEAK') == 1.0
    assert fc._named_value('WTEMP') == 21.5
    assert fc._named_value('GAIN') == 0.5


def test_sys_status_extended_health_is_not_readable_on_this_pymavlink():
    """Pins WHY we still read LEAK from NAMED_VALUE_FLOAT even though the board now also
    publishes it on the SYS_STATUS extended health bits (verified on the wire:
    present_extended = health_extended = 0x02).

    pymavlink 2.4.49's SYS_STATUS has 13 fields and no extensions, so the board's 40 bytes
    are parsed against a 31-byte schema and the rest is discarded. Same class of trap as
    ESC_STATUS(291): the data is on the wire and the library cannot see it.

    When this test FAILS, pymavlink gained the fields -- flip
    srot_protocol.SYS_STATUS_HAS_EXTENDED_HEALTH and read the bit directly.
    """
    from pymavlink import mavutil
    fields = mavutil.mavlink.MAVLink_sys_status_message.fieldnames
    has_ext = 'onboard_control_sensors_health_extended' in fields
    assert has_ext == sp.SYS_STATUS_HAS_EXTENDED_HEALTH, (
        f'pymavlink SYS_STATUS extended-health support changed (now {has_ext}); '
        f'update srot_protocol.SYS_STATUS_HAS_EXTENDED_HEALTH and read LEAK from the bit')


def test_behaviour_rev_required_still_matches_what_we_assume():
    """FW_BEHAVIOUR_REV tracks the board; FW_BEHAVIOUR_REV_REQUIRED is what THIS code
    needs, and the gap between them is deliberate.

    Every rev so far has been ADDITIVE for this host: rev 3 made WTEMP/SCALED_PRESSURE2
    absentable (we already treat absence as absence), rev 4 made yaw absolute (we read
    whatever the board reports), rev 5 added the baro jitter gate and a BARO_P2P value we
    simply display, rev 6 fixed MOTOR_DETECT (which this host never runs), rev 7 added a
    success statustext to PREFLIGHT_STORAGE, rev 8 SUPPRESSED DEPTH_ERR/DEPTH_OUT while
    the loop is not running (we moved check_depth_loop_settled onto DEPTH_CMD for it,
    which is a host change but not a REQUIREMENT change -- a rev-2 board still streams
    DEPTH_CMD), rev 9 published YAW_REF (new capability, nothing breaks without it). None of them
    break a rev-2 board, so the requirement stays at 2 -- raising it would strand a
    working vehicle for no safety gain.

    Rev 7 is the one to be careful about, and the care belongs HERE rather than in the
    requirement: it also shipped FRAME_REVERSE, which inverts every axis. That is a PARAM
    (default 0, set to 1 on our hull), not a revision property -- so raising the floor
    would neither catch a rev-7 board with it left at 0 nor help a board that has it set.
    A rev compare cannot express "check a stored value"; a param read can.

    The two numbers are asserted separately on purpose: bumping the tracker is routine
    bookkeeping, raising the requirement is a decision to refuse hardware.

    *** THE "EVERY REV IS ADDITIVE" CLAIM ABOVE EXPIRED AT REV 10. ***  It held for
    3..9 and the reasoning is kept because it is still the right test to apply. Two
    later revisions fail it:

      rev 10  YAW SENSE WAS INVERTED AND IS NOW CORRECT. An improper axis transform
              left the frame left-handed, so on a rev <= 9 board turning right makes
              yaw DECREASE -- measured in water 2026-08-07. MANUAL is unaffected
              (open loop); STABILIZE and DEPTH_HOLD span the hull. Any absolute
              heading, and any host loop that closes on yaw, has the wrong sign.
      rev 13  SROT_MOVE REQUIRES ARMED. Below it, a move sent while disarmed runs its
              whole profile with the thrusters silent and answers ACCEPTED at 100 %.
              The firmware's own note names duburi_ws as the consumer that would
              advance an entire mission on a dead hull.

    So the requirement is RAISED TO 10, and rev 10 is the reason on its own. This host
    is about to close a vision loop on yaw through STABILIZE; on a rev <= 9 board that
    loop diverges instead of converging, and it does so silently because roll and pitch
    read correctly. That is the "refuse hardware" case this docstring reserves the
    decision for -- not stranding a working vehicle, but declining one whose heading
    sign is wrong. Rev 13 alone would not justify it (an over-eager ACK is caught by
    our own arm check); rev 10 does."""
    assert sp.FW_BEHAVIOUR_REV == 14
    assert sp.FW_BEHAVIOUR_REV_REQUIRED == 10


def test_a_dead_link_ages_out_instead_of_being_restamped_forever():
    """`_drain_named()` re-folds whatever is in pymavlink's single slot -- and pymavlink
    NEVER clears that slot. So folding it unconditionally re-stamps a dead value as fresh
    on every call, and `max_age_s` can never fire: the one thing it exists to do.

    Fold each message object once. (`test_named_value_ages_out` misses this because it
    injects via note_named_value() and hand-backdates the table, going AROUND _drain_named.)
    """
    fc = _fc()
    fc.master.messages['NAMED_VALUE_FLOAT'] = _nvf('GAIN', 0.5)
    assert fc._named_value('GAIN') == 0.5
    # Link dies here. The slot still holds that same message object.
    fc._named_cache['GAIN'] = (0.5, time.time() - 60.0)
    assert fc._named_value('GAIN', max_age_s=3.0) is None, \
        'a value from a minute ago read as current -- the slot was re-stamped'


def test_the_burst_is_why_the_reader_hook_is_not_optional():
    """The board sends LEAK, WTEMP, STUNT_PRG, ATUNE, KILL, CURR, GAIN back-to-back in ONE
    500 ms tick (fw mav_stream.cpp `iv_nvf`). The reader drains the whole burst, so the slot
    settles on the LAST name -- GAIN -- and holds it until the next burst.

    Sampling therefore does not degrade to "1 call in 15" evenly; it degrades to GAIN almost
    always and LEAK almost NEVER. Only the reader hook sees the names in between.
    """
    fc = _fc()
    burst = ('LEAK', 'WTEMP', 'STUNT_PRG', 'ATUNE', 'KILL', 'CURR', 'GAIN')
    for name in burst:                       # what the reader thread sees
        fc.note_named_value(_nvf(name, 1.0))
    fc.master.messages['NAMED_VALUE_FLOAT'] = _nvf('GAIN', 1.0)   # what the slot keeps
    assert fc._named_value('LEAK') == 1.0, 'leak is unobservable without the reader hook'


# --------------------------------------------------------------------------- #
#  Two batteries -- BATTERY_STATUS is instanced, pymavlink's cache is not      #
# --------------------------------------------------------------------------- #

def _batt(bid, millivolts, centiamps=-1):
    return SimpleNamespace(id=bid, voltages=[millivolts, 65535, 65535, 65535],
                           current_battery=centiamps)


def test_get_battery_is_pinned_to_the_main_pack_not_whatever_arrived_last():
    """MEASURED ON THE VEHICLE: the board streams id 0 (PM1 electronics, 1.35 V) and
    id 1 (PM2 thruster pack, 14.74 V) at 2 Hz EACH, and pymavlink caches per MSGID.
    Sampling that slot alternates between two voltages an order of magnitude apart --
    so /duburi/state's battery reading was a coin flip on which battery it meant."""
    fc = _fc()
    fc.note_battery(_batt(0, 1350))
    fc.note_battery(_batt(1, 14740))     # arrives last; must NOT become "the" battery
    assert fc.get_battery()['voltage'] == pytest.approx(1.35)
    both = fc.get_batteries()
    assert both[sp.BATTERY_ID_MAIN]['voltage'] == pytest.approx(1.35)
    assert both[sp.BATTERY_ID_THRUSTER]['voltage'] == pytest.approx(14.74)


def test_an_unheard_battery_is_absent_not_zero():
    """The thruster pack reaches the board over ESP-NOW and is simply gone when that
    link drops. Reporting 0.0 V would read as a flat pack and trip a low-battery
    reaction; absence must stay absence."""
    fc = _fc()
    fc.note_battery(_batt(0, 1350))
    assert sp.BATTERY_ID_THRUSTER not in fc.get_batteries()


def test_a_stale_battery_ages_out_rather_than_being_restamped():
    fc = _fc()
    fc.master.messages['BATTERY_STATUS'] = _batt(1, 14740)
    assert sp.BATTERY_ID_THRUSTER in fc.get_batteries()
    fc._battery_cache[sp.BATTERY_ID_THRUSTER] = (14.74, math.nan, time.time() - 60.0)
    assert fc.get_batteries(max_age_s=5.0) == {}, 'dead link re-stamped as fresh'


# --------------------------------------------------------------------------- #
#  Arm guard -- a saturated depth loop is full vertical thrust on arm          #
# --------------------------------------------------------------------------- #

def test_arm_is_refused_while_the_barometer_is_implausible():
    """THE blocker, reproduced against the CURRENT signal. Observed disarmed on the
    bench 2026-08-02: a phantom baro reading -3.1 m at the surface. DEPTH_CMD is
    clamp(DEPTH_P * (depth - 0.10)) so that pins at -1.00. The mixer throttle column is
    -1 on all four verticals and 0 on all four horizontals, so arming turns that into
    full vertical thrust with the horizontals idling -- exactly the reported symptom."""
    fc = _fc()
    fc.note_named_value(_nvf('DEPTH_CMD', -1.0))
    ok, reason = fc.check_depth_loop_settled()
    assert ok is False
    assert 'IMPLAUSIBLE' in reason and 'SATURATED' in reason, \
        'a refusal must quote what it refused on, and name the clamp as a clamp'
    assert 'calibrate_depth' in reason, \
        'a large zero offset is commoner than a dead Bar30 -- name the cheap fix'


def test_the_guard_reads_depth_cmd_because_depth_out_is_absent_while_disarmed():
    """REGRESSION, and the reason this guard moved signals.

    fw rev 8 SUPPRESSES DEPTH_OUT while the controller is not running -- the honest-
    absence fix we asked for. But arm() is the only caller and only runs while
    disarmed, when the loop never runs, so DEPTH_OUT is ALWAYS absent here: the old
    code took its None -> "not reported" -> PASS branch every single time and the
    guard was structurally dead while still printing a reassuring line.

    A saturated DEPTH_CMD must refuse even with DEPTH_OUT absent, which is the only
    state a rev >= 8 board is ever in at this point."""
    fc = _fc()
    fc.note_named_value(_nvf('DEPTH_CMD', -1.0))      # no DEPTH_OUT at all
    assert fc.check_depth_loop_settled()[0] is False


def test_the_threshold_follows_depth_p_instead_of_a_hardcoded_output():
    """DEPTH_CMD is a CLAMPED OUTPUT, not an error, so a fixed 0.90 silently means a
    different physical depth the moment anyone retunes the gain. The guard divides
    DEPTH_P back out and compares metres.

    Same DEPTH_CMD, two gains, opposite verdicts -- that is the whole point."""
    lo = _fc(); lo.depth_p = 1.0     # 0.5 / 1.0 = 0.50 m  -> implausible
    lo.note_named_value(_nvf('DEPTH_CMD', 0.5))
    assert lo.check_depth_loop_settled()[0] is False

    hi = _fc(); hi.depth_p = 10.0    # 0.5 / 10.0 = 0.05 m -> fine
    hi.note_named_value(_nvf('DEPTH_CMD', 0.5))
    assert hi.check_depth_loop_settled()[0] is True


def _cmd(depth_m, gain=None):
    """The DEPTH_CMD a board at `depth_m` would emit: clamp(DEPTH_P * (depth - 0.10)).

    Written in METRES on purpose. Hand-typed wire values encode whatever DEPTH_P
    was current when they were written, and fw f533bd2 retuned it 3.0 -> 0.5 after
    the 2026-08-07 water test -- at which point every such fixture in this repo
    silently described a different physical depth.
    """
    g = sp.DEPTH_P_DEFAULT if gain is None else gain
    return max(-1.0, min(1.0, g * (depth_m - 0.10)))


def test_a_healthy_surface_reading_arms_normally():
    """~0.03 m at the surface -> DEPTH_CMD ~ -0.035 at DEPTH_P=0.5. Comfortably clear;
    if this ever fails the guard has become a nuisance that gets overridden by habit.

    The fixture MOVED with the firmware, and that is the point of writing it in the
    board's units. `DEPTH_CMD = clamp(DEPTH_P * (depth - 0.10))`, and fw f533bd2 took
    DEF_DEPTH_P from 3.0 to 0.5 after the water test tuned it. The same physical surface
    reading therefore emits -0.035 now where it emitted -0.21 before. Feeding the old
    -0.22 to a host that assumes 0.5 claims 0.44 m of error and REFUSES to arm a healthy
    board."""
    fc = _fc()
    fc.note_named_value(_nvf('DEPTH_CMD', _cmd(0.03)))     # ~3 cm at the surface
    assert fc.check_depth_loop_settled()[0] is True


def test_a_board_that_never_reports_the_depth_loop_is_not_blocked():
    """rev < 3 has no DEPTH_OUT. Silence must not become a permanent arm refusal --
    that would strand every older board for a check it cannot answer."""
    assert _fc().check_depth_loop_settled()[0] is True


def test_the_depth_guard_is_overridable_but_shouts():
    fc = _fc()
    fc.note_named_value(_nvf('DEPTH_CMD', 1.0))
    fc.allow_saturated_depth_arm = True
    ok, reason = fc.check_depth_loop_settled()
    assert ok is True and 'OVERRIDDEN' in reason


def test_telemetry_battery_voltage_is_the_main_pack_not_the_last_instance_seen():
    """REGRESSION, caught by a live smoke test against the board. telemetry() read the
    raw `_cache('BATTERY_STATUS')` slot behind a comment claiming "id 0 = electronics
    pack" -- so it reported the THRUSTER pack whenever that instance happened to land
    last. On the vehicle it showed battery_voltage == thruster_voltage == 14.63 V while
    PM1 actually reads ~1.35 V. Fixing get_battery() alone was not enough; this was a
    second call site."""
    fc = _fc()
    fc.note_battery(_batt(0, 1350))
    fc.note_battery(_batt(1, 14630))     # thruster pack lands last
    tel = fc.telemetry()
    assert tel.battery_voltage == pytest.approx(1.35), 'reported the wrong battery'
    assert tel.thruster_voltage == pytest.approx(14.63)


def test_esc_temperatures_survive_a_suppressed_water_temperature():
    """ESC temps must NOT be gated on WTEMP. The board suppresses WTEMP exactly when
    the barometer is unhealthy, so nesting them would drop thruster temperatures in
    the one situation where you most want them -- a vehicle that is already sick."""
    fc = _fc()
    fc.master.messages['ESC_TELEMETRY_1_TO_4'] = SimpleNamespace(
        rpm=[10, 20, 30, 40], temperature=[31, 32, 33, 34])
    tel = fc.telemetry()                       # no WTEMP ever noted
    assert math.isnan(tel.water_temp_c)
    assert tel.esc_temp_c == (31, 32, 33, 34)


# --------------------------------------------------------------------------- #
#  Payload: fire(N) is the BOARD channel, and the board's role decides           #
# --------------------------------------------------------------------------- #

class _RoleFC:
    """SrotFC stand-in that answers get_param for SERVOn_ROLE and ACKs commands."""
    def __init__(self, roles, ack=sp.ACK_ACCEPTED, funcs=None):
        self.roles = roles
        self.funcs = funcs or {}
        self.servo_calls = []
        self.param_reads = 0
        self._ack = ack

    def get_param(self, name, timeout=2.0):
        self.param_reads += 1
        for ch, role in self.roles.items():
            if name == f'SERVO{ch}_ROLE':
                return float(role)
        for ch, fn in self.funcs.items():
            if name == f'SERVO{ch}_FUNCTION':
                return float(fn)
        return None

    def set_servo(self, ch, us):
        self.servo_calls.append((ch, us))

    def set_servo_acked(self, ch, us, timeout):
        self.servo_calls.append((ch, us))
        return self._ack(us) if callable(self._ack) else self._ack

    def link_alive(self):
        return True


def _payload(roles, ack=sp.ACK_ACCEPTED, names=None, funcs=None):
    from duburi_control.fc.srot_fc import SrotPayload
    return SrotPayload(_RoleFC(roles, ack, funcs), names=names)


def test_fire_addresses_the_board_channel_with_no_translation():
    """The whole point of the redesign: the number a mission writes is the number
    that goes on the wire, and the same n as SERVO{n}_ROLE. Nothing is looked up
    to decide WHERE the shot goes -- only whether it may go at all."""
    pl = _payload({11: sp.PCA_ROLE_SWITCH})
    res = pl.fire(11)
    assert res.ok and res.code == FIRE_FIRED and res.channel == 11
    assert pl._fc.servo_calls == [(11, sp.PCA_SWITCH_ON_US),
                                  (11, sp.PCA_SWITCH_OFF_US)]


def test_firing_a_pwm_channel_is_refused_because_it_is_the_arm():
    """The firmware does NOT protect us here: DO_SET_SERVO on a role-1 channel
    writes servo_us and returns ACCEPTED (fw mav_commands.cpp:475-481), i.e. it
    moves the manipulator arm and reports success. Until the board gains a
    role-enforcing payload command this refusal is the only thing in the way."""
    pl = _payload({3: sp.PCA_ROLE_SERVO})
    res = pl.fire(3)
    assert res.code == FIRE_REJECTED_ARM and not res.ok
    assert pl._fc.servo_calls == [], 'a PWM/arm channel was actuated'


def test_firing_a_disabled_channel_is_refused_not_silently_dropped():
    """Role 0 would be a silent no-op on the board -- ACCEPTED, nothing actuated.
    Reporting that as a fire is the failure this whole result type exists for."""
    pl = _payload({5: sp.PCA_ROLE_DISABLED})
    res = pl.fire(5)
    assert res.code == FIRE_DISABLED and pl._fc.servo_calls == []


def test_an_unreadable_role_fails_closed():
    """A param read that times out is not evidence the channel is safe to drive."""
    pl = _payload({})
    res = pl.fire(9)
    assert res.code == FIRE_NOT_READY and pl._fc.servo_calls == []


def test_the_role_is_cached_so_fire_does_not_pay_a_param_roundtrip():
    """One read per channel, not one per fire: fire() is called at the moment a
    mission is glued to a target, and a 2 s param read there is not affordable."""
    pl = _payload({9: sp.PCA_ROLE_SWITCH})
    pl.fire(9); pl.fire(9)
    assert pl._fc.param_reads == 1, 'role re-read on every fire'


# ---- the ACK is the only feedback the wire offers ------------------------- #

def test_a_denied_ack_is_reported_as_denied():
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, ack=sp.ACK_DENIED)
    res = pl.fire(9)
    assert res.code == FIRE_DENIED and not res.ok


def test_a_temporarily_rejected_ack_is_busy_and_says_it_is_retryable():
    """The board missed its state mutex (fw :464). Distinct from DENIED because
    'retry' is the right response here and wrong for every other failure."""
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, ack=sp.ACK_TEMPORARILY_REJECTED)
    res = pl.fire(9)
    assert res.code == FIRE_BUSY and 'retry' in res.reason


def test_no_ack_is_unknown_not_success():
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, ack=None)
    res = pl.fire(9)
    assert res.code == FIRE_NO_ACK and not res.ok


def test_the_channel_is_de_energised_on_every_outcome():
    """A MOSFET held on burns the solenoid coil, and NO board-side failsafe clears
    it -- leak, disarm and GCS-loss all leave an energised channel energised."""
    for ack in (sp.ACK_ACCEPTED, sp.ACK_DENIED, sp.ACK_TEMPORARILY_REJECTED, None):
        pl = _payload({9: sp.PCA_ROLE_SWITCH}, ack=ack)
        pl.fire(9)
        assert pl._fc.servo_calls[-1] == (9, sp.PCA_SWITCH_OFF_US), ack


def test_a_second_concurrent_fire_is_refused_rather_than_queued():
    """Overlapping fires would let B's ON land inside A's pulse and A's finally
    de-energise while B still believed it was firing -- a truncated shot, from a
    race, with both callers reporting success. A queued shot is worse than a
    refused one: it fires after the hull has moved off target."""
    import threading
    from duburi_control.fc.srot_fc import SrotPayload
    import duburi_control.fc.srot_fc as mod
    pl = SrotPayload(_RoleFC({9: sp.PCA_ROLE_SWITCH}))
    mod_pulse = mod._FIRE_PULSE_S
    mod._FIRE_PULSE_S = 0.25
    try:
        out = []
        t = threading.Thread(target=lambda: out.append(pl.fire(9)))
        t.start()
        time.sleep(0.05)                      # land inside the first pulse
        second = pl.fire(9)
        t.join()
        assert second.code == FIRE_BUSY, 'a concurrent fire was allowed through'
        assert out[0].ok, 'the first fire was disturbed by the second'
    finally:
        mod._FIRE_PULSE_S = mod_pulse


def test_names_are_labels_only_and_never_route():
    """A stale label can mislabel a log line; it must not be able to send a shot
    anywhere. fire(N) is N whatever the name table says."""
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, names={9: 'torpedo_1', 11: 'dropper_1'})
    pl.fire(9)
    assert pl._fc.servo_calls[0][0] == 9
    assert pl.label(9) == 'torpedo_1' and pl.label(12) == ''


def test_preflight_reads_every_channel_and_reports_what_the_board_actually_has():
    """Defaults happen to be 1-8 arm / 9-16 switch, but that is a DEFAULT, not a
    rule -- any channel is re-rolable from Bondor. Preflight prints measured truth
    so nobody has to trust the folklore."""
    roles = {1: sp.PCA_ROLE_SERVO, 9: sp.PCA_ROLE_SWITCH, 10: sp.PCA_ROLE_SWITCH}
    pl = _payload(roles)
    report = pl.preflight_roles()
    assert report[1] == sp.PCA_ROLE_SERVO
    assert report[9] == sp.PCA_ROLE_SWITCH
    assert report[7] is None                     # unreadable, reported as such
    assert len(report) == sp.PCA9685_NUM_CH      # all 16, no assumed grouping


def test_a_rerolled_channel_is_honoured_in_both_directions():
    """The anti-folklore test: a board with channel 2 as a SWITCH and channel 10 as
    the ARM must fire 2 and refuse 10 -- the exact inverse of the defaults."""
    pl = _payload({2: sp.PCA_ROLE_SWITCH, 10: sp.PCA_ROLE_SERVO})
    assert pl.fire(2).ok
    assert pl.fire(10).code == FIRE_REJECTED_ARM


# --------------------------------------------------------------------------- #
#  Depth is gated on baro health -- VFR_HUD is NOT suppressed by the firmware  #
# --------------------------------------------------------------------------- #

def _sys_status(baro_healthy: bool):
    bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    return SimpleNamespace(onboard_control_sensors_health=(bit if baro_healthy else 0),
                           onboard_control_sensors_present=bit)


def test_depth_is_suppressed_when_the_board_calls_the_barometer_unhealthy():
    """MEASURED on a bare board, 2026-08-03: with NO Bar30 fitted the firmware still
    streams VFR_HUD.alt = -0.000 forever -- VFR_HUD is sent unconditionally, unlike
    SCALED_PRESSURE2/WTEMP which ARE suppressed (fw mav_stream.cpp:258). Passing that
    through published a confident 0.00 m depth on /duburi/state and into every depth
    guard, all of which compare against negative constants and would just stop firing."""
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(alt=-0.0, heading=160)
    fc.master.messages['SYS_STATUS'] = _sys_status(False)
    assert math.isnan(fc.get_attitude()['depth'])
    assert math.isnan(fc.telemetry().depth_m)


def test_depth_passes_through_when_the_barometer_is_healthy():
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(alt=-1.5, heading=160)
    fc.master.messages['SYS_STATUS'] = _sys_status(True)
    assert fc.get_attitude()['depth'] == pytest.approx(-1.5)


def test_depth_is_not_blanked_before_the_first_sys_status_arrives():
    """Tri-state on purpose: "the board says the baro is bad" and "we have not heard a
    SYS_STATUS yet" are different. Treating the second as unhealthy would blank depth
    for the first half-second of every connection."""
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(alt=-2.0, heading=160)
    assert fc.get_attitude()['depth'] == pytest.approx(-2.0)


def test_the_heading_matches_the_boards_own_0_360_convention():
    """The board's OLED and VFR_HUD.heading both wrap to 0..360, and so must we --
    a signed -162 beside the board's 197 is the same angle read two ways."""
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(
        roll=0.0, pitch=0.0, yaw=math.radians(-162.23))
    assert fc.get_attitude()['yaw'] == pytest.approx(197.77, abs=0.01)


# --------------------------------------------------------------------------- #
#  SERVOn_FUNCTION -- payload IDENTITY, read from the board                     #
# --------------------------------------------------------------------------- #
# The point of putting identity on the board is that it travels with the hull. A
# host-side table is wrong the moment someone re-wires a channel, and the failure
# mode of a wrong payload map is firing the manipulator arm during a drop.

def test_the_board_supplies_the_payload_name():
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, funcs={9: sp.PCA_FUNC_TORPEDO})
    pl.preflight_roles(channels=[9])
    assert pl.label(9) == 'torpedo'


def test_an_unassigned_function_is_not_a_name():
    """0 is the param default, so it means "nobody has said" -- rendering it as a
    device would invent a payload out of a factory default."""
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, funcs={9: sp.PCA_FUNC_NONE})
    pl.preflight_roles(channels=[9])
    assert pl.label(9) == ''


def test_the_host_override_wins_over_the_board():
    """The board's enum cannot express "torpedo_1 vs torpedo_2" -- two tubes share
    one function -- so payload_channels stays available for per-instance names."""
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, names={9: 'torpedo_2'},
                  funcs={9: sp.PCA_FUNC_TORPEDO})
    pl.preflight_roles(channels=[9])
    assert pl.label(9) == 'torpedo_2'


def test_function_never_makes_a_channel_fireable():
    """The whole safety argument: identity must not become an actuation decision.
    A PWM/arm channel labelled 'torpedo' is still the arm."""
    pl = _payload({3: sp.PCA_ROLE_SERVO}, funcs={3: sp.PCA_FUNC_TORPEDO})
    res = pl.fire(3)
    assert res.code == FIRE_REJECTED_ARM
    assert pl._fc.servo_calls == [], 'a labelled arm channel was actuated'


def test_a_disagreement_between_host_and_board_is_reported():
    """The stale-copy detector. Someone re-wired the harness and told one side only;
    neither value is trustworthy, so both get named."""
    class _Log:
        def __init__(self): self.warns = []
        def info(self, m): pass
        def warn(self, m): self.warns.append(m)
        def warning(self, m): self.warns.append(m)
        def error(self, m): pass
    log = _Log()
    from duburi_control.fc.srot_fc import SrotPayload
    pl = SrotPayload(_RoleFC({9: sp.PCA_ROLE_SWITCH}, funcs={9: sp.PCA_FUNC_DROPPER}),
                     log=log, names={9: 'torpedo_1'})
    pl.preflight_roles(channels=[9])
    assert any('torpedo_1' in w and 'dropper' in w for w in log.warns), log.warns


def test_an_unreadable_function_is_not_fatal():
    """A dropped PARAM_VALUE must cost a name, not the preflight -- and never the
    role, which is what actually gates firing."""
    pl = _payload({9: sp.PCA_ROLE_SWITCH}, funcs={})
    report = pl.preflight_roles(channels=[9])
    assert report[9] == sp.PCA_ROLE_SWITCH
    assert pl.label(9) == ''
    assert pl.fire(9).ok


# --------------------------------------------------------------------------- #
# YAW_REF -- absolute vs boot-relative heading (fw rev 9, Round 8 §8.8)
# --------------------------------------------------------------------------- #
def test_only_locked_means_the_heading_is_absolute():
    fc = _fc()
    fc.note_named_value(_nvf('YAW_REF', float(sp.YAW_REF_LOCKED)))
    ok, reason = fc.check_yaw_reference()
    assert ok is True and 'LOCKED' in reason


def test_a_refused_yaw_reference_is_reported_with_its_cause():
    """Each refusal has a different fix -- REFUSED_FIELD is hard iron near the board,
    REFUSED_NOISE is the vehicle moving during alignment, REFUSED_CAL is calibration.
    Collapsing them to "not locked" sends the operator looking in the wrong place."""
    for state in (sp.YAW_REF_REFUSED_CAL, sp.YAW_REF_REFUSED_FIELD,
                  sp.YAW_REF_REFUSED_NOISE, sp.YAW_REF_IDLE, sp.YAW_REF_SAMPLING):
        fc = _fc()
        fc.note_named_value(_nvf('YAW_REF', float(state)))
        ok, reason = fc.check_yaw_reference()
        assert ok is False
        assert sp.YAW_REF_NAMES[state].split()[0] in reason, \
            f'state {state} must name its own cause, got: {reason}'


def test_magacc_is_never_treated_as_a_yaw_reference_proxy():
    """The trap this whole check exists to close. We read MAGACC 2 on 2026-08-07 and
    concluded nothing about the lock, correctly: the board's alignment is protected by
    the |B| band and a sample-agreement test, neither of which depends on the sensor's
    self-assessment, and with a stored calibration need_acc drops to 0 so accuracy
    stops correlating with the outcome entirely. A perfect MAGACC with no YAW_REF must
    still report UNKNOWN."""
    fc = _fc()
    fc.note_named_value(_nvf('MAGACC', 3.0))
    ok, reason = fc.check_yaw_reference()
    assert ok is False and 'UNKNOWN' in reason


def test_an_older_board_reports_unknown_rather_than_assuming_either_way():
    """fw < 9 has no YAW_REF. Guessing 'locked' risks a meaningless absolute turn;
    guessing 'refused' would strand a board whose heading is in fact fine."""
    ok, reason = _fc().check_yaw_reference()
    assert ok is False and 'UNKNOWN' in reason and 'relative' in reason


def test_the_guard_subtracts_the_previews_own_target():
    """DEPTH_CMD is preview(depth, 0.10), so the 0.10 m target is baked into it and a
    perfectly-zeroed barometer in air still reads -0.10 m of raw "error". Judging the
    raw number spends a third of the 0.30 m budget before the sensor says anything --
    measured on the vehicle, a healthy board at 0.15 m of offset sat 0.05 m from a
    refusal it did not deserve. Subtracting the target recovers the board's own depth,
    which is the quantity actually being judged."""
    fc = _fc()
    # Values are DEPTH_P-dependent by construction; these are DEPTH_P=0.5 (fw rev 14,
    # water-tuned). 0.5 * (0.00 - 0.10) = -0.05.
    fc.note_named_value(_nvf('DEPTH_CMD', _cmd(0.00)))    # exactly 0.00 m
    ok, reason = fc.check_depth_loop_settled()
    assert ok is True and '+0.00 m' in reason

    healthy = _fc()
    # 0.5 * (-0.15 - 0.10) = -0.125
    healthy.note_named_value(_nvf('DEPTH_CMD', _cmd(-0.15)))  # the real -0.15 m reading
    assert healthy.check_depth_loop_settled()[0] is True
