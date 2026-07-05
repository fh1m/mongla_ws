"""Integration-ish tests for the Duburi facade.

We don't talk to a real autopilot -- a FakePixhawk records the calls
and reports a fixed attitude. Everything else (lock, return type, side
effects) is exercised through the real Duburi class.
"""

import logging
import math
import threading
import time

import pytest

from duburi_interfaces.action import Move

from duburi_control.duburi import Duburi
from duburi_control.errors import NotArmedError


class ThrottleLogger:
    """Stdlib logging.Logger but tolerates rclpy's `throttle_duration_sec`.

    The motion_* modules call `log.info(msg, throttle_duration_sec=...)`
    which is rclpy-only. In tests we use a stdlib Logger so we have to
    drop the unknown kwarg before forwarding.
    """

    def __init__(self, inner):
        self._inner = inner

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    """Records calls; reports a fixed attitude.

    Keeps the surface intentionally narrow so tests fail loudly if
    Duburi reaches for a method we forgot to implement here.
    """

    def __init__(self, *, depth=-0.5, yaw=10.0, mode='ALT_HOLD', armed=True,
                 cal_ack=(True, 'ACCEPTED'), cal_result_depth=0.0):
        self._attitude = {'yaw': yaw, 'pitch': 0.0, 'roll': 0.0, 'depth': depth}
        self._mode     = mode
        self._armed    = armed
        # Baro calibration behaviour (test-controllable): the ack the FC returns,
        # and the depth AHRS2 reports AFTER the re-zero (None = leave depth as-is,
        # simulating a cal that ACKed but didn't actually zero).
        self._cal_ack          = cal_ack
        self._cal_result_depth = cal_result_depth
        self.calls     = []
        self.lock      = threading.Lock()

    def get_attitude(self):
        return dict(self._attitude)

    def get_attitude_age(self):
        return 0.0   # always fresh in tests (matches the _fresh_depth precedent)

    def calibrate_barometer(self, timeout=6.0):
        self.calls.append(('calibrate_barometer',))
        if self._cal_result_depth is not None:
            self._attitude['depth'] = self._cal_result_depth
        return self._cal_ack

    def get_mode(self):
        return self._mode

    def is_armed(self):
        return self._armed

    def send_neutral(self):
        self.calls.append(('send_neutral',))

    def release_rc_override(self):
        self.calls.append(('release_rc_override',))

    def send_rc_override(self, **kw):
        self.calls.append(('send_rc_override', kw))

    def send_rc_translation(self, **kw):
        self.calls.append(('send_rc_translation', kw))

    def set_target_depth(self, depth):
        self.calls.append(('set_target_depth', depth))
        self._attitude['depth'] = depth

    def arm(self, timeout=15.0):
        self._armed = True
        return True, 'ACCEPTED'

    def disarm(self, timeout=20.0):
        self._armed = False
        return True, 'ACCEPTED'

    def set_mode(self, name, timeout=8.0):
        self._mode = name
        return True, 'ACCEPTED'


@pytest.fixture
def duburi():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger(logging.getLogger('test.duburi'))
    return Duburi(pixhawk, log)


def test_arm_returns_move_result(duburi):
    result = duburi.arm()
    assert isinstance(result, Move.Result)
    assert result.success is True
    assert result.message.startswith('arm:')


def test_set_mode_returns_move_result(duburi):
    result = duburi.set_mode('MANUAL')
    assert isinstance(result, Move.Result)
    assert result.success is True


def test_stop_sends_neutral_and_returns_result(duburi):
    result = duburi.stop(settle_time=0)
    assert ('send_neutral',) in duburi.pixhawk.calls
    assert result.success is True


def test_pause_releases_then_neutralises(duburi):
    result = duburi.pause(duration=0.05)
    kinds = [call[0] for call in duburi.pixhawk.calls]
    assert 'release_rc_override' in kinds
    # We always settle back to active hold afterwards so the next
    # command starts from a known state.
    assert kinds[-1] == 'send_neutral'
    assert result.success is True


def test_move_forward_short_burst_returns_depth(duburi, monkeypatch):
    # Hot loop: shrink any sleep so the test doesn't actually wait.
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    result = duburi.move_forward(duration=0.05, gain=50.0)
    assert isinstance(result, Move.Result)
    assert result.success is True
    assert math.isclose(result.final_value, -0.5, abs_tol=0.01)


def test_arc_dispatches_to_motion_forward(duburi, monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    result = duburi.arc(duration=0.05, gain=50.0, target_yaw=30.0)
    assert isinstance(result, Move.Result)
    assert result.success is True
    # arc emits packets with both forward + yaw -- this is the
    # single-packet contract that makes curved motion possible.
    arc_packets = [
        c for c in duburi.pixhawk.calls
        if c[0] == 'send_rc_override' and 'forward' in c[1] and 'yaw' in c[1]]
    assert arc_packets, 'arc must emit Ch5 + Ch4 in the same packet'


def test_lock_heading_starts_thread_and_unlock_stops(duburi, monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    assert duburi._heading_lock is None
    duburi.lock_heading(target=90.0, timeout=2.0)
    assert duburi._heading_lock is not None
    duburi.unlock_heading()
    assert duburi._heading_lock is None


def test_lock_heading_target_zero_locks_current_heading(duburi, monkeypatch):
    """target=0.0 (rosidl unset) means "lock at current heading"."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    result = duburi.lock_heading(target=0.0, timeout=2.0)
    # Fixture FakePixhawk reports yaw=10.0, so locked target should be 10.0.
    assert math.isclose(result.final_value, 10.0, abs_tol=0.1)
    duburi.unlock_heading()


def test_surface_does_not_deadlock(duburi, monkeypatch):
    """surface() runs inside its own command scope and then calls set_depth()
    (another scope) on the SAME thread. self.lock must be reentrant (RLock)
    or this self-deadlocks forever -- the original P0 bug."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    done = threading.Event()

    def _run():
        duburi.surface()
        done.set()

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    assert done.wait(timeout=5.0), (
        'surface() deadlocked -- self.lock must be reentrant (RLock)')


def test_disarm_stops_active_heading_lock(duburi, monkeypatch):
    """disarm() must stop a running heading lock (its background Ch4 stream
    must not outlive MANUAL) and clear the handle so no zombie lingers."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    duburi.lock_heading(target=0.0, timeout=10.0)
    assert duburi._heading_lock is not None
    duburi.disarm()
    assert duburi._heading_lock is None
    assert duburi.pixhawk.is_armed() is False


def test_set_depth_engages_alt_hold(duburi, monkeypatch):
    """set_depth must guarantee ALT_HOLD is engaged before driving."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    duburi.pixhawk._mode = 'MANUAL'
    duburi.set_depth(target=-1.0, timeout=2.0)
    assert duburi.pixhawk._mode == 'ALT_HOLD', (
        'set_depth must auto-engage ALT_HOLD; otherwise ArduSub silently '
        'drops the depth setpoint')


def test_quick_settle_skips_pause_on_same_axes(monkeypatch):
    """quick_settle=True drops the inter-command 0.6 s pause when the
    next command writes the same axis set as the previous one and no
    lock is active. The pause is *only* for guarding against axis
    cross-talk; same-axis chains have no cross-talk to guard against.
    """
    sleeps = []
    monkeypatch.setattr(time, 'sleep', lambda d: sleeps.append(float(d)))

    pixhawk = FakePixhawk()
    log     = ThrottleLogger(logging.getLogger('test.duburi.quick'))
    d       = Duburi(pixhawk, log, quick_settle=True)

    # First move primes _last_axes; settle pause is honoured.
    d.move_forward(duration=0.05, gain=50.0)
    first_pauses = sleeps.count(0.6)
    sleeps.clear()

    # Second move on the same axis should SKIP the 0.6 s pre-flight pause.
    d.move_forward(duration=0.05, gain=50.0)
    second_pauses = sleeps.count(0.6)

    assert first_pauses >= 1, (
        'first command must still settle (no prior axis state to compare)')
    assert second_pauses == 0, (
        'quick_settle=True must skip the 0.6 s pre-flight pause when the '
        'next command writes the same axes as the previous one')


def test_quick_settle_off_by_default_keeps_pause(monkeypatch):
    """Default behaviour: every command pre-flight always settles."""
    sleeps = []
    monkeypatch.setattr(time, 'sleep', lambda d: sleeps.append(float(d)))

    pixhawk = FakePixhawk()
    log     = ThrottleLogger(logging.getLogger('test.duburi.classic'))
    d       = Duburi(pixhawk, log)        # quick_settle defaults to False

    d.move_forward(duration=0.05, gain=50.0)
    sleeps.clear()
    d.move_forward(duration=0.05, gain=50.0)

    assert 0.6 in sleeps, (
        'default Duburi() must always settle 0.6 s before each command')


def test_quick_settle_pauses_when_axes_differ(monkeypatch):
    """move_forward -> move_left differs in axis set; pause must fire."""
    sleeps = []
    monkeypatch.setattr(time, 'sleep', lambda d: sleeps.append(float(d)))

    pixhawk = FakePixhawk()
    log     = ThrottleLogger(logging.getLogger('test.duburi.qs.diff'))
    d       = Duburi(pixhawk, log, quick_settle=True)

    d.move_forward(duration=0.05, gain=50.0)
    sleeps.clear()
    d.move_left(duration=0.05, gain=50.0)

    assert 0.6 in sleeps, (
        'quick_settle must still settle when the next command writes a '
        'different axis set')


def test_motion_uses_translation_when_lock_active(duburi, monkeypatch):
    """When heading-lock is engaged, motion commands MUST stay off Ch4.

    With the rate-based HeadingLock the LOCK thread now writes Ch4
    itself (``send_rc_override(yaw=...)``). Translation commands
    (``move_forward``, ``move_lateral``) must keep using
    ``send_rc_translation`` which touches Ch5/Ch6/Ch3 only; any Ch4
    write from the motion path would fight the lock.
    """
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    duburi.lock_heading(target=0.0, timeout=2.0)
    duburi.pixhawk.calls.clear()
    duburi.move_forward(duration=0.05, gain=50.0)
    # Ignore packets the lock thread itself is emitting in the background
    # -- those are *expected* Ch4 writes. We only care about packets that
    # touch forward/lateral/throttle here, i.e. translation packets.
    translation_calls = [c for c in duburi.pixhawk.calls
                         if c[0] == 'send_rc_translation']
    motion_yaw_overrides = [
        c for c in duburi.pixhawk.calls
        if c[0] == 'send_rc_override'
        and ('forward' in c[1] or 'lateral' in c[1] or 'throttle' in c[1])]
    duburi.unlock_heading()
    assert not motion_yaw_overrides, (
        'with heading-lock active, translation commands must NOT call '
        'send_rc_override for fwd/lat/throttle (they would clobber Ch4)')
    assert translation_calls, 'move_forward must emit translation packets'


# ================================================================== #
#  Arm-state guard                                                    #
# ================================================================== #

def test_motion_verb_raises_not_armed_when_disarmed(monkeypatch):
    """Motion commands must fail loudly (NotArmedError) when disarmed.

    ArduSub silently drops all RC override frames while disarmed, so a
    silent success would be a lie. The action server catches the error
    and returns success=False to the caller.
    """
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    pixhawk = FakePixhawk(armed=False)
    log     = ThrottleLogger(logging.getLogger('test.duburi.unarm'))
    d       = Duburi(pixhawk, log)

    with pytest.raises(NotArmedError):
        d.move_forward(duration=0.05, gain=50.0)


def test_unarm_safe_verbs_succeed_when_disarmed(monkeypatch):
    """stop, pause, and unlock_heading must work regardless of arm state.

    These are the 'safe' verbs that need to operate even when the sub
    is disarmed (e.g. releasing an RC override after an emergency stop).
    """
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    pixhawk = FakePixhawk(armed=False)
    log     = ThrottleLogger(logging.getLogger('test.duburi.unarm.safe'))
    d       = Duburi(pixhawk, log)

    assert d.stop(settle_time=0).success is True
    assert d.pause(duration=0.0).success is True
    assert d.unlock_heading().success is True   # no-op when no lock active


def test_motion_succeeds_after_arm(monkeypatch):
    """Motion command must execute normally once arm() is called."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    pixhawk = FakePixhawk(armed=False)
    log     = ThrottleLogger(logging.getLogger('test.duburi.arm.then.move'))
    d       = Duburi(pixhawk, log)

    d.arm()
    result = d.move_forward(duration=0.05, gain=50.0)
    assert result.success is True


# --------------------------------------------------------------------------- #
#  Vision telemetry slot + rich _make_result (live feedback + end-state)       #
# --------------------------------------------------------------------------- #
def test_report_vision_then_telemetry_fresh(duburi):
    duburi.report_vision(-42.0, 8.0)
    assert duburi.vision_telemetry() == (-42.0, 8.0)


def test_vision_telemetry_stale_returns_none(duburi):
    duburi.report_vision(10.0, 20.0)
    # A tiny freshness window must elapse -> stale -> None (no live verb).
    time.sleep(0.05)
    assert duburi.vision_telemetry(fresh_s=0.001) is None


def test_vision_telemetry_none_before_any_report(duburi):
    assert duburi.vision_telemetry() is None


def test_make_result_sets_vision_fields(duburi):
    r = duburi._make_result(True, 'x', final_value=0.0, error_value=12.0,
                            end_x_px=-42.0, end_y_px=8.0,
                            fill_frac=0.6, elapsed_s=3.1)
    assert r.end_x_px == pytest.approx(-42.0)
    assert r.end_y_px == pytest.approx(8.0)
    assert r.fill_frac == pytest.approx(0.6)
    assert r.elapsed_s == pytest.approx(3.1)


def test_make_result_defaults_nan_for_non_vision(duburi):
    r = duburi._make_result(True, 'arm', final_value=-0.5)
    assert math.isnan(r.end_x_px) and math.isnan(r.end_y_px)
    assert r.fill_frac == 0.0 and r.elapsed_s == 0.0


# ── depth calibration (barometer re-zero before a dive) ──────────────────────
def test_calibrate_depth_success_when_surfaced_and_disarmed(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    # disarmed, slight drift (-0.07m); the fake zeroes depth on calibrate.
    pixhawk = FakePixhawk(armed=False, depth=-0.07, cal_result_depth=0.0)
    duburi  = Duburi(pixhawk, ThrottleLogger(logging.getLogger('test.baro')))
    r = duburi.calibrate_depth(settle_s=0.0)
    assert r.success is True
    assert ('calibrate_barometer',) in pixhawk.calls
    assert abs(r.final_value) < 0.05          # verified post-cal depth ~0


def test_calibrate_depth_skips_when_armed():
    # NEVER calibrate a diving hull -- arm-state is the surface proxy.
    pixhawk = FakePixhawk(armed=True, depth=-1.5)
    duburi  = Duburi(pixhawk, ThrottleLogger(logging.getLogger('test.baro')))
    r = duburi.calibrate_depth(settle_s=0.0)
    assert r.success is False                  # standalone verb: skip -> not-success
    assert ('calibrate_barometer',) not in pixhawk.calls   # never sent the command


def test_calibrate_depth_refuses_when_not_at_surface(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    # disarmed but 0.5m deep (> 0.30 bound) => not surfaced / baro fault => refuse.
    pixhawk = FakePixhawk(armed=False, depth=-0.5)
    duburi  = Duburi(pixhawk, ThrottleLogger(logging.getLogger('test.baro')))
    r = duburi.calibrate_depth(settle_s=0.0)
    assert r.success is False
    assert ('calibrate_barometer',) not in pixhawk.calls   # refused before sending


def test_calibrate_depth_fails_when_post_not_zero(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    # cal ACKs but depth stays off zero (e.g. wrong CMD param) => verify FAILS.
    pixhawk = FakePixhawk(armed=False, depth=-0.08, cal_result_depth=-0.20)
    duburi  = Duburi(pixhawk, ThrottleLogger(logging.getLogger('test.baro')))
    r = duburi.calibrate_depth(settle_s=0.0)
    assert r.success is False
    assert ('calibrate_barometer',) in pixhawk.calls       # it DID try


def test_mission_reset_calibrates_when_disarmed(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    pixhawk = FakePixhawk(armed=False, depth=-0.06, cal_result_depth=0.0)
    duburi  = Duburi(pixhawk, ThrottleLogger(logging.getLogger('test.baro')))
    r = duburi.mission_reset()
    assert r.success is True                                # reset always succeeds
    assert ('calibrate_barometer',) in pixhawk.calls        # auto re-zero fired


def test_mission_reset_skips_calibration_when_armed(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    pixhawk = FakePixhawk(armed=True, depth=-1.0)
    duburi  = Duburi(pixhawk, ThrottleLogger(logging.getLogger('test.baro')))
    r = duburi.mission_reset()
    assert r.success is True                                # reset duties still done
    assert ('calibrate_barometer',) not in pixhawk.calls    # armed => auto-skip


def test_mission_reset_survives_calibration_error(monkeypatch):
    # A raising calibration must NEVER break mission_reset's safety duties.
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    pixhawk = FakePixhawk(armed=False, depth=-0.05)
    def _boom(*a, **k):
        raise RuntimeError('cal exploded')
    pixhawk.calibrate_barometer = _boom
    duburi  = Duburi(pixhawk, ThrottleLogger(logging.getLogger('test.baro')))
    r = duburi.mission_reset()
    assert r.success is True                                # reset completed anyway
