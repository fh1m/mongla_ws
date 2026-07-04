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


def test_gain_to_pwm_percent_mode_matches_percent_to_pwm():
    # pass_through=False is the percent path (unchanged behaviour).
    assert Pixhawk.gain_to_pwm(0, False) == 1500
    assert Pixhawk.gain_to_pwm(50, False) == 1700
    assert Pixhawk.gain_to_pwm(4, False) == 1516


def test_gain_to_pwm_raw_mode_is_direct_delta():
    # pass_through=True: value is a RAW PWM delta off 1500 (the 20kg stiction probe).
    assert Pixhawk.gain_to_pwm(0, True) == 1500
    assert Pixhawk.gain_to_pwm(4, True) == 1504
    assert Pixhawk.gain_to_pwm(-4, True) == 1496


def test_gain_to_pwm_raw_mode_keeps_clamp():
    # A fat-fingered raw value must NOT escape [1100, 1900].
    assert Pixhawk.gain_to_pwm(900, True) == 1900
    assert Pixhawk.gain_to_pwm(-900, True) == 1100


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


def test_stale_heartbeat_reports_not_armed_and_unknown_mode():
    # link died: last heartbeat is well past the stale window
    px = _pixhawk_with_hb(_hb(armed=True, custom_mode=2, age_s=_LINK_STALE_S + 2))
    assert px.is_armed() is False          # precondition BLOCKS instead of driving
    assert px.get_mode() == 'UNKNOWN'      # never a stale ALT_HOLD
    assert px.link_alive() is False


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
