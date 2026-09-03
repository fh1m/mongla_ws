"""What the manager does when the flight controller restarts under a mission.

`SrotFC.check_for_reboot()` was fully implemented, documented, unit tested --
and called by NOTHING outside its own test. That is the same defect
`telemetry()` had on this backend and the same shape as a helper nothing
calls: the code reads as finished, the behaviour is absent, and no test of the
detector itself can tell the difference.

It matters here more than most places. Opening the serial port reboots this
board, so an FC restart mid-session is reachable by any stray second process
-- and afterwards the board is DISARMED, in its boot mode, with every setpoint
cleared and stream rates back to compiled defaults. A mission that keeps
issuing verbs is steering a vehicle it no longer configured, and each verb
fails in its own way without naming the cause.
"""
import pytest

from duburi_manager import auv_manager_node as amn


class _Log:
    def __init__(self):
        self.errors = []

    def error(self, m):
        self.errors.append(m)

    def debug(self, m):
        pass

    def warning(self, m):
        pass

    def info(self, m):
        pass


class _Tel:
    rpm = []
    leak = False


class _FC:
    def __init__(self, rebooted=False):
        self.rebooted = rebooted
        self.checks = 0

    def check_for_reboot(self):
        self.checks += 1
        return self.rebooted

    def telemetry(self):
        return _Tel()


class _Duburi:
    def __init__(self):
        self.aborts = 0

    def request_abort(self):
        self.aborts += 1


class _Node:
    def __init__(self, rebooted=False, command_active=True, is_srot=True):
        self.fc = _FC(rebooted)
        self.duburi = _Duburi()
        self.command_active = command_active
        self._is_srot = is_srot
        self._leak_latched = False
        self.esc_rpm_publisher = None
        self._log = _Log()

    def get_logger(self):
        return self._log

    def _maybe_print_srot_block(self, tel):
        pass


def _tick(**kw):
    n = _Node(**kw)
    amn.AUVManagerNode._publish_srot_telemetry(n)
    return n


def test_the_detector_is_actually_called():
    """The whole finding in one line. It passed its own unit tests for a round
    while running zero times in production."""
    assert _tick().fc.checks == 1


def test_a_restart_under_a_running_command_aborts_it():
    """Fail-safe default: a mission step that continues here is commanding a
    disarmed board through a configuration that no longer exists. Stopping and
    surfacing the cause beats N verbs failing for N unrelated-looking reasons."""
    n = _tick(rebooted=True, command_active=True)
    assert n.duburi.aborts == 1
    assert any('restarted' in m for m in n._log.errors)


def test_a_quiet_link_is_not_an_abort():
    """The negative control. A detector that fires without a reboot would abort
    missions at 2 Hz, which is a worse failure than the one being fixed."""
    n = _tick(rebooted=False, command_active=True)
    assert n.duburi.aborts == 0
    assert n._log.errors == []


def test_a_restart_with_nothing_running_is_not_an_abort():
    """There is no command to abort between mission steps -- and the driver has
    already logged the restart itself, so nothing is lost by staying quiet."""
    n = _tick(rebooted=True, command_active=False)
    assert n.duburi.aborts == 0


def test_the_pixhawk_backend_is_left_alone():
    """`check_for_reboot` is a SROT method. This branch also runs on
    flight_controller:=pixhawk, where calling it would raise inside a 2 Hz
    timer -- the tick is gated on the backend for that reason."""
    n = _tick(is_srot=False)
    assert n.fc.checks == 0
