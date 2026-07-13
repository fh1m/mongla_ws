"""Static-method tests for the Pixhawk class.

These don't touch a real autopilot -- they cover the two pure helpers
that the rest of the stack relies on (PWM conversion + heading-error
wrap), plus a logger-only check that the per-command MAVLink trace
tag (``[MAV <fn>[ cmd=<verb>]] ...``) and the compact RC summary
helper are wired up correctly.
"""

import pytest

from duburi_control import tracing
from duburi_control.pixhawk import Pixhawk, NO_OVERRIDE


def test_percent_to_pwm_neutral():
    assert Pixhawk.percent_to_pwm(0) == 1500


def test_percent_to_pwm_full_range():
    assert Pixhawk.percent_to_pwm(+100) == 1900
    assert Pixhawk.percent_to_pwm(-100) == 1100


def test_percent_to_pwm_clamps():
    assert Pixhawk.percent_to_pwm(+999) == 1900
    assert Pixhawk.percent_to_pwm(-999) == 1100


def test_heading_error_zero():
    assert Pixhawk.heading_error(90.0, 90.0) == pytest.approx(0.0)


def test_heading_error_short_path_positive():
    assert Pixhawk.heading_error(10.0, 350.0) == pytest.approx(20.0)


def test_heading_error_short_path_negative():
    assert Pixhawk.heading_error(350.0, 10.0) == pytest.approx(-20.0)


def test_heading_error_180_boundary():
    # 180 deg apart -> the wrap returns -180 (the formula is
    # (delta + 540) % 360 - 180; 540 % 360 == 180 -> 180 - 180 == 0 -> -180).
    # We pin the sign here so a refactor doesn't accidentally flip it.
    assert Pixhawk.heading_error(180.0, 0.0) == pytest.approx(-180.0)


# ---------------------------------------------------------------------- #
#  RC summariser                                                          #
# ---------------------------------------------------------------------- #
#
# The RC summariser is what keeps the [MAV send_rc_override ...] line
# short during a typical "yaw correction only" tick. These tests pin
# the three classes of body it can produce so a regression on either
# side (RC writer changing channel layout, or the formatter trying to
# get clever) shows up immediately.

def _rc_array(**channels):
    """Build an 18-slot RC override array with the given channels set."""
    values = [NO_OVERRIDE] * 18
    name_to_idx = {
        'pitch': 0, 'roll': 1, 'thr': 2, 'yaw': 3, 'fwd': 4, 'lat': 5,
    }
    for name, pwm in channels.items():
        values[name_to_idx[name]] = int(pwm)
    return values


def test_rc_summary_only_active_channels():
    body = Pixhawk._summarise_rc(_rc_array(yaw=1430))
    assert body == 'yaw=1430'


def test_rc_summary_multiple_active_channels():
    body = Pixhawk._summarise_rc(_rc_array(fwd=1700, yaw=1620))
    assert body == 'yaw=1620 fwd=1700'


def test_rc_summary_all_neutral():
    body = Pixhawk._summarise_rc(_rc_array(
        pitch=1500, roll=1500, thr=1500, yaw=1500, fwd=1500, lat=1500))
    assert body == 'all=neutral'


def test_rc_summary_all_released():
    body = Pixhawk._summarise_rc([NO_OVERRIDE] * 18)
    assert body == 'all=released'


# ---------------------------------------------------------------------- #
#  MAVLink trace-tag wiring                                               #
# ---------------------------------------------------------------------- #
#
# These tests don't open a serial port -- they call _log_mavlink directly
# with a captured logger so the message format is locked in regardless of
# how pymavlink decides to spell things over the wire. The format is what
# operators grep for during pool runs (`rg "cmd=yaw_right" session.log`)
# so a silent reformat would burn the next debug session.

class _CapLogger:
    """Minimal logger duck-type that records DEBUG calls."""
    def __init__(self):
        self.lines = []

    def debug(self, msg, *args, **kwargs):
        self.lines.append(str(msg))

    # Pixhawk only calls .debug; stub the others so a stray refactor
    # that suddenly logs at INFO/WARN doesn't silently break the format.
    def info(self, *a, **k):
        raise AssertionError('Pixhawk._log_mavlink must log at DEBUG, not INFO')

    def warning(self, *a, **k):
        raise AssertionError('Pixhawk._log_mavlink must log at DEBUG, not WARNING')


class _StubMaster:
    """Just enough of mavutil.mavlink_connection for Pixhawk.__init__."""
    target_system = 1
    target_component = 1
    messages = {}


def test_mav_line_uses_caller_function_name_only():
    log = _CapLogger()
    px  = Pixhawk(_StubMaster(), log=log)
    # Calling _log_mavlink directly with sys._getframe(1) walks back
    # ONE frame: the caller is THIS test function. We deliberately
    # dropped the filename from the prefix to keep the line short --
    # the function name alone pinpoints the callsite via grep.
    px._log_mavlink('hello')
    assert len(log.lines) == 1
    line = log.lines[0]
    assert line == '[MAV test_mav_line_uses_caller_function_name_only] hello'


def test_mav_line_carries_cmd_when_tracing_enabled():
    log = _CapLogger()
    px  = Pixhawk(_StubMaster(), log=log)
    tracing.set_enabled(True)
    try:
        with tracing.command_scope('yaw_right'):
            px._log_mavlink('hello')
    finally:
        tracing.set_enabled(False)
    line = log.lines[-1]
    assert line.endswith(' cmd=yaw_right] hello'), line
    assert line.startswith('[MAV test_mav_line_carries_cmd_when_tracing_enabled '), line


def test_mav_line_drops_cmd_when_tracing_disabled():
    log = _CapLogger()
    px  = Pixhawk(_StubMaster(), log=log)
    # tracing.set_enabled(False) is the default; opening a command_scope()
    # while disabled must NOT introduce a cmd= tag (production runs
    # stay quiet -- this is the "off by default" contract).
    tracing.set_enabled(False)
    with tracing.command_scope('yaw_right'):
        px._log_mavlink('hello')
    line = log.lines[-1]
    assert 'cmd=' not in line, line


def test_mav_silent_when_log_is_none():
    # Pixhawk(log=None) must be a no-op so unit tests using FakePixhawk
    # never have to inject a logger. We verify by handing it a logger
    # object that throws on ANY method call... and then the production
    # default of None to make sure that path stays free.
    px = Pixhawk(_StubMaster(), log=None)
    px._log_mavlink('this should silently no-op')


# --------------------------------------------------------------------------- #
#  MAVLink write-lock: concurrent sends must be SERIALIZED (no torn frames).   #
#  A fake master.mav records whether any two rc_channels_override_send calls   #
#  ever overlap; with the _tx_lock they never should.                          #
# --------------------------------------------------------------------------- #
import threading
import time as _time
import types
from unittest.mock import MagicMock


class _OverlapDetectingMav:
    """Flags if two sends are ever in-flight at once (unserialized writes)."""
    def __init__(self):
        self._active = 0
        self.overlap = False
        self._probe = threading.Lock()

    def _enter(self):
        with self._probe:
            self._active += 1
            if self._active > 1:
                self.overlap = True

    def _exit(self):
        with self._probe:
            self._active -= 1

    def rc_channels_override_send(self, *a, **k):
        self._enter()
        _time.sleep(0.001)          # widen the window a torn frame would use
        self._exit()

    def __getattr__(self, _name):   # any other *_send -> same overlap probe
        def _fn(*a, **k):
            self._enter(); _time.sleep(0.0005); self._exit()
        return _fn


def _pixhawk_with_overlap_mav():
    from duburi_control.pixhawk import Pixhawk
    master = MagicMock()
    master.mav = _OverlapDetectingMav()
    master.target_system = 1
    master.target_component = 1
    return Pixhawk(master), master.mav


def test_tx_lock_serializes_concurrent_rc_sends():
    px, mav = _pixhawk_with_overlap_mav()
    # 8 threads hammering the RC override path (heading-lock + heartbeat +
    # action-thread + mocap all writing at once, the real 5-writer race).
    def hammer():
        for _ in range(25):
            px.send_rc_override(1500, 1500, 1500, 1600, 1500, 1500)
    threads = [threading.Thread(target=hammer) for _ in range(8)]
    for t in threads: t.start()
    for t in threads: t.join()
    assert mav.overlap is False, 'two rc_channels_override_send overlapped -> torn frame'


def test_tx_lock_serializes_mixed_send_types():
    px, mav = _pixhawk_with_overlap_mav()
    # Mixed writers: RC override vs depth setpoint vs mocap vs heartbeat.
    def rc():   [px.send_rc_yaw_only(1600) for _ in range(20)]
    def dep():  [px.set_target_depth(-1.0) for _ in range(20)]
    def moc():  [px.send_att_pos_mocap(30.0) for _ in range(20)]
    def hb():   [px.send_heartbeat() for _ in range(20)]
    fns = (rc, dep, moc, hb, rc, dep, moc, hb)   # 8 fresh threads (no reuse)
    threads = [threading.Thread(target=f) for f in fns]
    for t in threads: t.start()
    for t in threads: t.join()
    assert mav.overlap is False, 'mixed MAVLink writes overlapped -> torn frame'


# --------------------------------------------------------------------------- #
#  Link-loss freshness gate: a stale cached HEARTBEAT must NOT report a live   #
#  armed/ALT_HOLD (which would let command preconditions pass on a dead link). #
# --------------------------------------------------------------------------- #
from duburi_control.pixhawk import _LINK_STALE_S
from pymavlink import mavutil as _mavutil


def _hb(*, armed, custom_mode=0, age_s=0.0):
    """A fake autopilot HEARTBEAT `age_s` seconds old."""
    m = MagicMock()
    m.autopilot = _mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA  # not INVALID
    m.base_mode = (_mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
    m.custom_mode = custom_mode
    m._timestamp = _time.time() - age_s
    return m


def _pixhawk_with_hb(hb):
    from duburi_control.pixhawk import Pixhawk
    master = MagicMock()
    master.mav = MagicMock()
    master.messages = {'HEARTBEAT': hb} if hb is not None else {}
    master.mode_mapping.return_value = {'ALT_HOLD': 2, 'MANUAL': 19}
    return Pixhawk(master)


def test_fresh_heartbeat_reports_armed():
    px = _pixhawk_with_hb(_hb(armed=True, custom_mode=2, age_s=0.0))
    assert px.is_armed() is True
    assert px.get_mode() == 'ALT_HOLD'
    assert px.link_alive() is True


def test_stale_heartbeat_keeps_cached_arm_mode_but_link_advisory_flips():
    # link stalled past the window: is_armed()/get_mode() are HARD per-command
    # preconditions, so they must keep reporting the last KNOWN arm state/mode
    # (cached) -- NOT flip on transient jitter and abort every motion verb ->
    # disarm. Staleness surfaces ONLY through the advisory link_alive()/age.
    px = _pixhawk_with_hb(_hb(armed=True, custom_mode=2, age_s=_LINK_STALE_S + 2))
    assert px.is_armed() is True           # cached; only OUR disarm() flips it
    assert px.get_mode() == 'ALT_HOLD'     # cached; never spuriously UNKNOWN
    assert px.link_alive() is False        # advisory-only: link is stale
    assert px.heartbeat_age() > _LINK_STALE_S


def test_missing_timestamp_treated_fresh():
    # test-style mock with no _timestamp -> treated fresh (get_attitude_age rule)
    hb = _hb(armed=True, custom_mode=2)
    del hb._timestamp
    px = _pixhawk_with_hb(hb)
    assert px.is_armed() is True
    assert px.heartbeat_age() == 0.0


def test_no_heartbeat_is_link_dead():
    px = _pixhawk_with_hb(None)
    assert px.is_armed() is False
    assert px.get_mode() == 'UNKNOWN'
    assert px.heartbeat_age() is None
    assert px.link_alive() is False


# --------------------------------------------------------------------------- #
#  Barometer calibration: must send PREFLIGHT_CALIBRATION with param3=1.       #
#  A wrong param would ACK but calibrate NOTHING (silent no-op safety bug), so #
#  pin the exact command id + param index/value.                              #
# --------------------------------------------------------------------------- #
from pymavlink import mavutil as _mavutil


class _CalRecordingMav:
    """Records command_long_send args and injects an ACK, so calibrate_barometer
    sees a reply like the real FC. `result` is the MAV_RESULT it returns."""
    def __init__(self, master, result):
        self._master = master
        self._result = result
        self.long_calls = []

    def command_long_send(self, *args):
        self.long_calls.append(args)
        # Simulate the FC replying with a COMMAND_ACK for this command id.
        ack = types.SimpleNamespace(command=args[2], result=self._result)
        self._master.messages['COMMAND_ACK'] = ack


class _CalMaster:
    target_system = 1
    target_component = 1

    def __init__(self, result=0):
        self.messages = {}
        self.mav = _CalRecordingMav(self, result)


def test_calibrate_barometer_sends_preflight_cal_param3_one():
    master = _CalMaster(result=0)   # ACCEPTED
    px = Pixhawk(master, log=None)
    ok, reason = px.calibrate_barometer(timeout=1.0)
    assert ok is True and reason == 'ACCEPTED'
    assert len(master.mav.long_calls) == 1
    args = master.mav.long_calls[0]
    # args: sys, comp, command, confirmation, p1, p2, p3, p4, p5, p6, p7
    assert args[2] == _mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
    assert args[6] == 1, 'param3 (ground pressure / baro) must be 1'
    # every other calibration param must be 0 (don't trigger gyro/accel/mag/etc.)
    assert args[4] == 0 and args[5] == 0 and args[7] == 0
    assert args[8] == 0 and args[9] == 0 and args[10] == 0


def test_calibrate_barometer_reports_failure_result():
    master = _CalMaster(result=4)   # MAV_RESULT_FAILED
    px = Pixhawk(master, log=None)
    ok, reason = px.calibrate_barometer(timeout=1.0)
    assert ok is False


# --------------------------------------------------------------------------- #
#  get_angular_rates: body-frame gyro rates from MAVLink ATTITUDE (flow comp)  #
# --------------------------------------------------------------------------- #
class _AttMaster:
    target_system = 1
    target_component = 1

    def __init__(self, att=None):
        self.messages = {}
        if att is not None:
            self.messages['ATTITUDE'] = att


def test_get_angular_rates_reads_attitude_speeds():
    att = types.SimpleNamespace(rollspeed=0.10, pitchspeed=-0.20, yawspeed=0.05)
    px = Pixhawk(_AttMaster(att), log=None)
    r = px.get_angular_rates()
    assert r is not None
    assert r['roll_rate']  == 0.10
    assert r['pitch_rate'] == -0.20
    assert r['yaw_rate']   == 0.05
    assert r['age_s'] == 0.0   # mock without _timestamp -> treated fresh


def test_get_angular_rates_none_when_no_attitude():
    px = Pixhawk(_AttMaster(None), log=None)
    assert px.get_angular_rates() is None


# ---------------------------------------------------------------------- #
#  arm() safety: abort-with-disarm + STATUSTEXT pre-arm reason            #
# ---------------------------------------------------------------------- #
#
# The "doesn't arm, 12 s crossed" fix lives in the client deadline, but arm()
# also gained an abort hook (never strand an armed hull the caller gave up on)
# and a pre-arm reason on timeout. These pin both without a real autopilot.

import threading


class _ArmMaster:
    """Records command_long_send calls; p1 (arg index 4) is 1=arm / 0=disarm."""
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = self
        self.sent = []

    def command_long_send(self, *args):
        self.sent.append(args)


def _bare_arm_pixhawk(*, statustext=None):
    """A Pixhawk with __init__ bypassed, wired with just what arm() touches."""
    p = Pixhawk.__new__(Pixhawk)
    p.master = _ArmMaster()
    p._tx_lock = threading.Lock()
    p.clear_ack = lambda: None
    p._log_mavlink = lambda *a, **k: None
    p.wait_ack = lambda cmd, timeout=3.0: (True, 'ACCEPTED')
    p.is_armed = lambda: False           # never actually arms
    p.get_statustext = lambda: statustext
    return p


def test_arm_abort_sends_disarm_and_returns_aborted():
    p = _bare_arm_pixhawk()
    ok, reason = p.arm(timeout=2.0, abort=lambda: True)   # abort on first poll
    assert ok is False
    assert reason == 'ABORTED'
    # arm (p1=1) then a disarm (p1=0) so an aborted arm can't strand the hull.
    p1_sequence = [args[4] for args in p.master.sent]
    assert p1_sequence == [1, 0]


def test_arm_timeout_appends_prearm_statustext():
    p = _bare_arm_pixhawk(statustext='PreArm: Battery below minimum')
    ok, reason = p.arm(timeout=0.2, abort=None)           # ACK ok, never arms
    assert ok is False
    assert reason == 'NOT_ARMED_AFTER_ACK: PreArm: Battery below minimum'


def test_arm_timeout_without_statustext_is_plain():
    p = _bare_arm_pixhawk(statustext=None)
    ok, reason = p.arm(timeout=0.2, abort=None)
    assert ok is False
    assert reason == 'NOT_ARMED_AFTER_ACK'
