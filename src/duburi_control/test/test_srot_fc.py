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


def test_srot_payload_fire_servo_is_a_bounded_pulse():
    from duburi_control.fc.srot_fc import SrotPayload
    import duburi_control.fc.srot_fc as m
    orig = m._FIRE_PULSE_S
    m._FIRE_PULSE_S = 0.0                          # keep the test fast
    try:
        fc = _fc()
        pay = SrotPayload(fc, fire_map={1: ('servo', 1, 2000, 1000)})
        assert pay.fire(1) is True
        servo_cmds = [s for s in fc.master.mav.sent
                      if s[0] == 'cmd' and s[1] == sp.CMD_DO_SET_SERVO]
        # A pulse = fire µs then a return-to-rest µs (never left at the end-stop).
        assert servo_cmds[0][2][1] == 2000.0 and servo_cmds[-1][2][1] == 1000.0
        assert pay.fire(9) is False               # unmapped channel -> no-op
    finally:
        m._FIRE_PULSE_S = orig


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
#  Payload: refuse rather than guess                                            #
# --------------------------------------------------------------------------- #
def test_fire_refuses_when_the_map_is_not_configured():
    """Which PCA channel each payload is on, and servo-vs-MOSFET, are hardware
    facts. Guessing them fires the wrong actuator on a live vehicle."""
    from duburi_control.fc.srot_fc import SrotPayload, _FIRE_MAP
    assert _FIRE_MAP == {}, 'the shipped fire map must stay empty'
    fc = _fc()
    payload = SrotPayload(fc)
    assert payload.fire(1) is False
    assert not _moves(fc) and not fc.master.mav.sent   # nothing actuated at all


def test_fire_uses_a_configured_map():
    from duburi_control.fc.srot_fc import SrotPayload
    fc = _fc()
    payload = SrotPayload(fc, fire_map={2: ('relay', 1)})
    assert payload.fire(2) is True
    relays = [(p[0], p[1]) for k, c, p in fc.master.mav.sent
              if k == 'cmd' and c == sp.CMD_DO_SET_RELAY]
    assert relays == [(1.0, 1.0), (1.0, 0.0)]   # energise then de-energise


# --------------------------------------------------------------------------- #
#  payload_fire_map parsing -- the param that makes fire() possible at all      #
# --------------------------------------------------------------------------- #
def test_parse_fire_map_relay_and_servo():
    from duburi_control.fc.srot_fc import parse_fire_map
    m = parse_fire_map('1:relay:0, 2:relay:1, 3:servo:3, 4:servo:5:1900:1100')
    assert m[1] == ('relay', 0)
    assert m[2] == ('relay', 1)
    assert m[3] == ('servo', 3, sp.SERVO_MAX_US, sp.SERVO_MIN_US)   # us default
    assert m[4] == ('servo', 5, 1900, 1100)                          # us explicit


def test_parse_fire_map_empty_is_empty_not_a_guess():
    """Blank must stay empty so fire() keeps refusing. A default here would mean
    firing an unknown actuator on a live vehicle."""
    from duburi_control.fc.srot_fc import parse_fire_map
    assert parse_fire_map('') == {}
    assert parse_fire_map(None) == {}


def test_parse_fire_map_skips_bad_entries_without_losing_good_ones():
    """A typo in one channel must not silently disarm the other three."""
    from duburi_control.fc.srot_fc import parse_fire_map
    m = parse_fire_map('1:relay:0, 2:banana:1, oops, 3:relay:99, 4:servo:3')
    assert set(m) == {1, 4}          # the two well-formed, in-range entries
    assert m[1] == ('relay', 0)


def test_parse_fire_map_output_drives_fire():
    """End-to-end: the parsed shape is what SrotPayload.fire() consumes."""
    from duburi_control.fc.srot_fc import SrotPayload, parse_fire_map
    fc = _fc()
    payload = SrotPayload(fc, fire_map=parse_fire_map('2:relay:1'))
    assert payload.fire(2) is True
    assert payload.fire(1) is False   # unmapped channel still refuses


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
    needs. Rev 3 changes are additive for us (WTEMP/SCALED_PRESSURE2 can be absent, which
    we already treat as absent), so the requirement stays at 2 -- a rev-2 board still runs
    this host correctly. Raising it would strand a working vehicle."""
    assert sp.FW_BEHAVIOUR_REV == 4
    assert sp.FW_BEHAVIOUR_REV_REQUIRED == 2


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
