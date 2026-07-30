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
    """Shrink the ACK deadline floor and the brake pulse for the whole module.

    Both are real production values (an 8 s floor so a slow first ACK is not called
    a stall; a brake long enough to actually null momentum) -- but a test that
    deliberately drives the stall path should not sit through them. Patched here
    rather than lowered in the source, so the shipped numbers stay honest.
    """
    import duburi_control.fc.srot_fc as mod
    monkeypatch.setattr(mod, '_ACK_MIN_BUDGET_S', 0.3)
    monkeypatch.setattr(mod, '_ACK_MARGIN_S', 0.2)
    monkeypatch.setattr(sp, 'BRAKE_MAX_S', 0.05)
    monkeypatch.setattr(sp, 'BRAKE_K', 0.02)
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
    fc.set_message_rate(30, 50)          # no-op on SROT (fixed rates)
    fc.send_att_pos_mocap(90.0)          # no-op (board fuses BNO on-board)
    assert fc.get_rc_channels() is None
    assert not fc.master.mav.sent        # neither reached the wire


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


def test_abort_brakes_with_a_reverse_leg_not_a_bare_stop():
    """MOVE_STOP alone is a COAST: the firmware zeroes s_uf/s_ul/s_speed before
    the STOP case, so PH_BRAKE computes -0*gain*0 == 0 (fw movement.cpp:56,90,152).
    An abort must therefore reverse the axis itself, or a 20 kg hull keeps going
    on a board that has no position estimate."""
    fc = _fc()
    fc.master.auto_ack = None                     # never terminates -> abort path
    res = fc.move('move_forward', duration=3.0, gain=50.0,
                  abort_fn=lambda: True)
    assert res.code == ABORTED
    sent = _moves(fc)
    assert sent[0][0] == sp.MOVE_FORWARD          # the leg
    assert sent[1][0] == sp.MOVE_BACK             # the brake -- reversed axis
    assert sent[1][2] > 0.0                       # ...with real thrust
    assert 0.0 < sent[1][1] <= sp.BRAKE_MAX_S     # ...and bounded duration
    assert sent[1][3] == pytest.approx(sent[1][1])  # p5 caps it board-side too
    assert sent[-1][0] == sp.MOVE_STOP            # then settle/cancel the seq


def test_brake_reverses_the_correct_lateral_axis():
    fc = _fc()
    fc.master.auto_ack = None
    fc.move('move_left', duration=2.0, gain=40.0, abort_fn=lambda: True)
    assert [m[0] for m in _moves(fc)][:2] == [sp.MOVE_STRAFE_L, sp.MOVE_STRAFE_R]


def test_yaw_and_depth_legs_are_not_braked():
    """Yaw is a rate the board bleeds; depth is a hold. Reversing either fights
    the controller -- same rule as motion_vision's inertial brake."""
    for verb, kw in (('yaw_right', {'target': 90.0}), ('set_depth', {'target': -1.5})):
        fc = _fc()
        fc.master.auto_ack = None
        fc.move(verb, abort_fn=lambda: True, **kw)
        types = [m[0] for m in _moves(fc)]
        assert types[-1] == sp.MOVE_STOP
        assert sp.MOVE_BACK not in types and sp.MOVE_FORWARD not in types


def test_slow_leg_is_not_braked():
    """No momentum worth nulling below BRAKE_MIN_SPEED -- don't kick the hull."""
    fc = _fc()
    fc.master.auto_ack = None
    fc.move('move_forward', duration=1.0, gain=1.0, abort_fn=lambda: True)
    assert [m[0] for m in _moves(fc)] == [sp.MOVE_FORWARD, sp.MOVE_STOP]


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


def test_stop_verb_brakes_the_previous_leg():
    """`duburi stop` must actually stop, not coast.

    A *completed* leg still leaves the hull coasting, because the board's own
    end-of-leg PH_BRAKE is the zero-thrust one -- so the following `stop` is
    exactly where the momentum has to be nulled.
    """
    fc = _fc()
    fc.master.auto_ack = _ack(sp.ACK_ACCEPTED)
    fc.move('move_forward', duration=5.0, gain=60.0)      # completes; leg recorded
    fc.master.mav.sent.clear()
    fc.move('stop')
    assert [m[0] for m in _moves(fc)] == [sp.MOVE_BACK, sp.MOVE_STOP]


def test_stop_does_not_double_brake():
    """Once stop_motion() has braked, the leg is consumed -- a second stop (or an
    abort right after a timeout that already braked) must not kick the hull again."""
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
#  The unsupported-verb contract                                                #
# --------------------------------------------------------------------------- #
def test_unsupported_verbs_are_disjoint_from_move_verbs():
    from duburi_control.fc.srot_fc import UNSUPPORTED_VERBS
    assert not (UNSUPPORTED_VERBS & MOVE_VERBS)
    # The ones that used to lie or crash rather than refuse.
    for verb in ('lock_heading', 'move_back', 'arc', 'vision_align'):
        assert verb in UNSUPPORTED_VERBS


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
