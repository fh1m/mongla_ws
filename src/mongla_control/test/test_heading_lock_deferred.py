"""Deferred-activation heading lock tests.

The operator can call ``lock_heading`` BEFORE arming: the heading is
captured immediately, but yaw correction stays suspended (no Ch4 writes,
no thruster kick) until the AUV is armed AND the first actuating
control/vision command runs. This keeps surface holders from fighting a
correction during arm + descent and makes runs reproducible.

These tests drive the real Mongla facade against a fake autopilot that
records every Ch4 write (``send_rc_yaw_only``) and a heartbeat whose
hold-count we can inspect for ref-count balance.
"""

import logging
import threading
import time

import pytest

from mongla_control.mongla import Mongla


class ThrottleLogger:
    """stdlib Logger that tolerates rclpy's ``throttle_duration_sec`` kwarg."""

    def __init__(self, inner):
        self._inner = inner

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakeHeartbeat:
    """Reentrant pause/resume counter -- mirrors the real Heartbeat surface."""

    def __init__(self):
        self.hold_count = 0

    def pause(self):
        self.hold_count += 1

    def resume(self):
        if self.hold_count > 0:
            self.hold_count -= 1


class FakePixhawk:
    """Records Ch4 writes; arm state is togglable so we can model pre-arm."""

    def __init__(self, *, yaw=42.0, armed=False):
        self._attitude = {'yaw': yaw, 'pitch': 0.0, 'roll': 0.0, 'depth': -0.5}
        self._mode  = 'ALT_HOLD'
        self._armed = armed
        self.yaw_only_calls = 0
        self.calls = []

    # --- read surface ---
    def get_attitude(self):
        return dict(self._attitude)

    def get_mode(self):
        return self._mode

    def is_armed(self):
        return self._armed

    # --- write surface ---
    def send_neutral(self):
        self.calls.append('send_neutral')

    def send_rc_override(self, **kw):
        self.calls.append(('send_rc_override', kw))

    def send_rc_translation(self, **kw):
        self.calls.append(('send_rc_translation', kw))

    def send_rc_yaw_only(self, pwm):
        self.yaw_only_calls += 1

    def set_target_depth(self, depth):
        self._attitude['depth'] = depth

    def arm(self, timeout=15.0, abort=None):
        if abort is not None:
            abort()                 # mirror real contract; guards facade wiring
        self._armed = True
        return True, 'ACCEPTED'

    def disarm(self, timeout=20.0):
        self._armed = False
        return True, 'ACCEPTED'

    def set_mode(self, name, timeout=8.0):
        self._mode = name
        return True, 'ACCEPTED'


def _make(armed=False):
    pixhawk = FakePixhawk(armed=armed)
    hb      = FakeHeartbeat()
    log     = ThrottleLogger(logging.getLogger('test.deferred'))
    mongla  = Mongla(pixhawk, log, heartbeat=hb)
    return mongla, pixhawk, hb


@pytest.fixture(autouse=True)
def _no_sleep(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)


def test_lock_heading_runs_while_disarmed_and_defers():
    """Disarmed call: captures heading, suspends correction, no Ch4 writes."""
    mongla, pixhawk, hb = _make(armed=False)

    result = mongla.lock_heading(target=0.0, timeout=10.0)

    assert result.success is True
    # Captured the live heading (fixture yaw=42.0).
    assert abs(result.final_value - 42.0) < 0.1
    assert mongla._heading_lock is not None
    assert mongla._lock_deferred is True
    assert mongla._heading_lock.is_suspended is True
    # _lock_active() drives lock-aware writers -- must be False while deferred.
    assert mongla._lock_active() is False
    # The whole point: zero thruster correction at capture time.
    time.sleep(0)  # yield; thread parked before start, so still zero
    assert pixhawk.yaw_only_calls == 0
    # Heartbeat NOT held for a deferred lock (neutral stream keeps running).
    assert hb.hold_count == 0
    assert mongla._lock_holds_heartbeat is False

    mongla.unlock_heading()


def test_passive_verb_while_armed_does_not_activate():
    """Arming + a passive verb (head) must NOT start correction."""
    mongla, pixhawk, hb = _make(armed=False)
    mongla.lock_heading(target=0.0, timeout=10.0)

    mongla.arm()
    assert mongla._lock_deferred is True          # arm alone never activates

    mongla.head()                                 # passive verb
    assert mongla._lock_deferred is True
    assert mongla._heading_lock.is_suspended is True
    assert hb.hold_count == 0

    mongla.unlock_heading()


def test_first_armed_actuating_command_activates():
    """Arm, then an actuating verb resumes correction exactly once."""
    mongla, pixhawk, hb = _make(armed=False)
    mongla.lock_heading(target=0.0, timeout=10.0)
    mongla.arm()

    mongla.move_forward(duration=0.05, gain=50.0)   # actuating

    assert mongla._lock_deferred is False
    assert mongla._heading_lock is not None
    assert mongla._heading_lock.is_suspended is False
    assert mongla._lock_active() is True
    # Heartbeat now held for the active lock -- exactly one outstanding hold.
    assert hb.hold_count == 1
    assert mongla._lock_holds_heartbeat is True

    mongla.unlock_heading()
    assert hb.hold_count == 0                        # balanced release


def test_lock_heading_while_armed_activates_immediately():
    """Regression: locking while already armed keeps legacy behaviour."""
    mongla, pixhawk, hb = _make(armed=True)

    mongla.lock_heading(target=90.0, timeout=10.0)

    assert mongla._lock_deferred is False
    assert mongla._heading_lock.is_suspended is False
    assert hb.hold_count == 1
    assert mongla._lock_holds_heartbeat is True

    mongla.unlock_heading()
    assert hb.hold_count == 0


def test_unlock_after_deferred_never_underflows_heartbeat():
    """Capture disarmed, never activate, unlock -> heartbeat count stays 0."""
    mongla, pixhawk, hb = _make(armed=False)
    mongla.lock_heading(target=0.0, timeout=10.0)

    mongla.unlock_heading()

    assert hb.hold_count == 0                        # no underflow below zero
    assert mongla._heading_lock is None
    assert mongla._lock_deferred is False


def test_lock_heading_never_changes_flight_mode():
    """Heading lock must NOT force ALT_HOLD (would engage depth hold)."""
    mongla, pixhawk, hb = _make(armed=True)
    pixhawk._mode = 'MANUAL'

    mongla.lock_heading(target=0.0, timeout=10.0)

    assert pixhawk.get_mode() == 'MANUAL', (
        'lock_heading must never switch flight mode -- depth hold is the '
        "operator's responsibility via set_depth")

    mongla.unlock_heading()


# ── heartbeat pause is a SELF-CORRECTING bool (CTRL-6 counter reverted) ────────
def test_hold_release_bool_balances_and_self_corrects():
    # A single release always fully resumes (fail-open), and a release with no
    # matching hold is a safe no-op -- the bool can never strand PAUSED (which
    # would stop the 5 Hz neutral stream -> FS_PILOT_INPUT disarm).
    mongla, _pix, hb = _make()

    mongla._hold_heartbeat_for_lock()
    assert hb.hold_count == 1 and mongla._lock_holds_heartbeat is True
    mongla._hold_heartbeat_for_lock()               # idempotent: no double-pause
    assert hb.hold_count == 1
    mongla._release_heartbeat_for_lock()            # single release fully resumes
    assert hb.hold_count == 0 and mongla._lock_holds_heartbeat is False

    mongla._release_heartbeat_for_lock()            # release with no hold: no-op
    assert hb.hold_count == 0 and mongla._lock_holds_heartbeat is False
