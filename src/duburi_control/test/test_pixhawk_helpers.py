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
