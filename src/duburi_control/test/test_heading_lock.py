"""Tests for heading_lock (background Ch4-rate heading controller).

The lock runs a proportional yaw-rate loop against the configured
YawSource and writes Ch4 via `send_rc_yaw_only` (Ch4-only override that
leaves Ch5/Ch6 at NO_OVERRIDE so concurrent DVL translation moves are
not interrupted). Tests check the contract: rate packets are emitted,
retarget takes effect, suspend freezes output, and the AHRS fallback
still works when yaw_source is None.
"""

import logging
import math
import time

import pytest

from duburi_control.heading_lock import (
    HeadingLock,
    _lock_floor, _lock_command,
    LOCK_DEADBAND_DEG, LOCK_APPROACH_BAND_DEG,
    LOCK_SPEED_MIN_PCT, LOCK_PCT_MAX, LOCK_KP_PCT_PER_DEG,
)
from duburi_control.pixhawk import Pixhawk


class ThrottleLogger:
    def __init__(self):
        self._inner = logging.getLogger('test.heading_lock')

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        self._inner.warning(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    def __init__(self, yaw=10.0):
        self.rc_overrides = []
        self._yaw = yaw

    def send_rc_override(self, **kw):
        self.rc_overrides.append(kw)

    def send_rc_yaw_only(self, yaw):
        # Mirrors Pixhawk.send_rc_yaw_only: Ch4-only override. Recorded
        # in the same list shape (`{'yaw': pwm}`) that _yaw_packets reads.
        self.rc_overrides.append({'yaw': int(yaw)})

    def get_attitude(self):
        return {'yaw': self._yaw, 'depth': -0.5,
                'roll': 0.0, 'pitch': 0.0}


class FakeYawSource:
    """Drop-in YawSource. `name` is what HeadingLock prints in logs."""
    name = 'fake_source'

    def __init__(self, value=42.0):
        self._v = value

    def read_yaw(self):
        return self._v


def _yaw_packets(pixhawk):
    return [p for p in pixhawk.rc_overrides if 'yaw' in p]


def test_heading_lock_on_target_parks_ch4_at_neutral():
    """When current yaw == target, error is inside the deadband and the
    loop must write Ch4 = 1500 us (zero yaw rate)."""
    pixhawk = FakePixhawk(yaw=90.0)
    log     = ThrottleLogger()
    src     = FakeYawSource(value=90.0)

    lock = HeadingLock(pixhawk, target_deg=90.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.2)
    lock.stop()

    packets = _yaw_packets(pixhawk)
    assert packets, 'lock thread must emit Ch4 override packets'
    assert all(p['yaw'] == 1500 for p in packets), (
        'on-target lock must park Ch4 at 1500 us (deadband)')


def test_heading_lock_off_target_commands_proportional_yaw_rate():
    """When current yaw is far from target, the loop must drive Ch4 in
    the direction of the shortest heading error."""
    # Source reads 0; target is 90 -> error ~ +90 -> yaw_pct positive ->
    # PWM > 1500 (convention: percent_to_pwm clamps symmetric around 1500).
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource(value=0.0)

    lock = HeadingLock(pixhawk, target_deg=90.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    lock.stop()

    packets = _yaw_packets(pixhawk)
    assert packets
    assert any(p['yaw'] > 1500 for p in packets), (
        'positive heading error should push Ch4 above 1500 us')


def test_heading_lock_retarget_swaps_value():
    # Source reads 0.0. First we target 10 (small err -> may deadband or
    # small positive pct), then retarget to 180 (big negative err ->
    # Ch4 decidedly below 1500). We check that the direction flips.
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource(value=0.0)

    lock = HeadingLock(pixhawk, target_deg=10.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    first_batch = list(_yaw_packets(pixhawk))
    lock.retarget(180.0)
    time.sleep(0.2)
    lock.stop()

    second_batch = _yaw_packets(pixhawk)[len(first_batch):]
    assert second_batch, 'packets should continue after retarget'
    # heading_error(180, 0) == +180 OR -180; either way |yaw_pct| should
    # be near LOCK_PCT_MAX, i.e. Ch4 noticeably off 1500.
    assert any(abs(p['yaw'] - 1500) >= 50 for p in second_batch), (
        'retarget to 180 deg should produce a large Ch4 deflection')


def test_heading_lock_suspend_pauses_streaming():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource()

    lock = HeadingLock(pixhawk, target_deg=0.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    pre_count = len(_yaw_packets(pixhawk))
    lock.suspend()
    time.sleep(0.25)
    post_count = len(_yaw_packets(pixhawk))
    lock.resume()
    time.sleep(0.15)
    final_count = len(_yaw_packets(pixhawk))
    # Allow stop() to emit its own safe-stop packet.
    lock.stop()

    # Suspended -> at most 1-2 in-flight packets after suspend.
    assert post_count - pre_count <= 2, (
        f'suspend should stop streaming, but got '
        f'{post_count - pre_count} extra packets')
    assert final_count > post_count


def test_heading_lock_timeout_fires_on_exit_callback():
    """On its own timeout the lock must notify the owner via on_exit so a
    timed-out lock gets cleared instead of lingering as a zombie that still
    looks 'active' (keeping the heartbeat paused)."""
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource()
    fired = []

    lock = HeadingLock(pixhawk, target_deg=0.0, yaw_source=src, log=log,
                       timeout=0.1, on_exit=lambda lk: fired.append(lk))
    lock.start()
    time.sleep(0.4)            # well past the 0.1 s auto-release
    assert fired == [lock], (
        'on_exit must fire exactly once with the lock instance on timeout')
    lock.stop()               # idempotent; thread already exited


def test_heading_lock_explicit_stop_does_not_fire_on_exit():
    """on_exit is only the self-timeout escape hatch; an explicit stop()
    (the caller already cleans up) must NOT invoke it."""
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource()
    fired = []

    lock = HeadingLock(pixhawk, target_deg=0.0, yaw_source=src, log=log,
                       timeout=5.0, on_exit=lambda lk: fired.append(lk))
    lock.start()
    time.sleep(0.15)
    lock.stop()
    time.sleep(0.1)
    assert fired == [], 'explicit stop() must not fire on_exit'


def test_heading_lock_works_with_no_yaw_source():
    """yaw_source=None -> falls back to AHRS via pixhawk.get_attitude.

    Verifies the source-agnostic plug: same lock object works in
    Gazebo/SITL with `mavlink_ahrs` (no external IMU).
    """
    pixhawk = FakePixhawk(yaw=42.0)
    log     = ThrottleLogger()

    lock = HeadingLock(pixhawk, target_deg=42.0, yaw_source=None, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    lock.stop()

    assert _yaw_packets(pixhawk), (
        'lock must emit Ch4 packets even without an external yaw_source')


# =========================================================================== #
#  Tapered stiction floor (2026-06 align-jitter fix)                           #
#                                                                              #
#  The HARD min-PWM floor was a relay on the Ch4 yaw RATE: holding a still hull #
#  it barely fired (error in the deadband), but rejecting the continuous yaw    #
#  moment a lat-only vision_align induces it kicked >=5% every tick -> overshoot #
#  -> sign flip -> limit-cycle = the align-yaw jitter. The floor now tapers to 0 #
#  at the deadband edge (mirror of the motion_yaw `ab2014f` fix). `_lock_floor`  #
#  / `_lock_command` are pure, so this needs no thread.                         #
# =========================================================================== #
def _approx(x, tol=1e-6):
    class _A:
        def __eq__(_s, other): return abs(other - x) <= tol
        def __repr__(_s): return f"~{x}"
    return _A()


def test_floor_full_outside_band():
    assert _lock_floor(LOCK_APPROACH_BAND_DEG) == LOCK_SPEED_MIN_PCT
    assert _lock_floor(LOCK_APPROACH_BAND_DEG + 45.0) == LOCK_SPEED_MIN_PCT


def test_floor_zero_at_deadband_edge():
    # The fix: at the deadband edge the floor is ~0 so the command decays to a
    # stop instead of relay-bouncing at the hard 5% floor.
    assert _lock_floor(LOCK_DEADBAND_DEG) == 0.0


def test_floor_monotonic_linear_in_band():
    mid = 0.5 * (LOCK_DEADBAND_DEG + LOCK_APPROACH_BAND_DEG)
    f_mid = _lock_floor(mid)
    assert 0.0 < f_mid < LOCK_SPEED_MIN_PCT
    assert f_mid == _approx(LOCK_SPEED_MIN_PCT * 0.5)


def test_command_inside_deadband_is_zero():
    assert _lock_command(0.0) == 0.0
    assert _lock_command(LOCK_DEADBAND_DEG - 0.01) == 0.0
    assert _lock_command(-(LOCK_DEADBAND_DEG - 0.01)) == 0.0


def test_command_near_edge_decays_not_pinned_to_floor():
    # THE regression guard. Just outside the deadband the command must be SMALL
    # (P with ~0 floor), NOT pinned at the old hard 5% floor.
    out = abs(_lock_command(LOCK_DEADBAND_DEG + 0.2))
    assert out < LOCK_SPEED_MIN_PCT, (
        f"near-edge command {out:.2f}% must decay below the {LOCK_SPEED_MIN_PCT}% "
        "floor, not be pinned to it (the relay limit-cycle)")


def test_command_large_error_floored_and_capped():
    assert _lock_command(180.0) == _approx(LOCK_PCT_MAX)
    assert abs(_lock_command(LOCK_APPROACH_BAND_DEG + 1.0)) >= LOCK_SPEED_MIN_PCT - 1e-9


def test_command_sign_follows_error():
    assert _lock_command(30.0) > 0
    assert _lock_command(-30.0) < 0


# ---- Closed-loop toy plant: taper rejects disturbance with a small dither --- #
def _simulate(command_fn, *, start=4.0, target=0.0, disturbance_dps=2.0,
              steps=800, dt=0.05, gain_dps_per_pct=4.0, latency=1):
    """Kinematic yaw plant under a CONSTANT yaw disturbance (the lateral-thrust
    moment a strafing align injects). Heading integrates commanded rate + the
    disturbance with one tick of latency. Returns the heading-error history.
    """
    heading = start
    pending = [0.0] * max(1, latency)
    hist = []
    for _ in range(steps):
        err = Pixhawk.heading_error(target, heading)
        cmd = command_fn(err)
        applied = pending.pop(0)
        pending.append(cmd)
        heading = (heading
                   + (applied * gain_dps_per_pct + disturbance_dps) * dt) % 360.0
        hist.append(Pixhawk.heading_error(target, heading))
    return hist


def _peak_to_peak(hist, window=200):
    tail = hist[-window:]
    return max(tail) - min(tail)


def _hard_floor_command(error):
    """The OLD hard-floor law (pins the fix to the mechanism)."""
    if abs(error) <= LOCK_DEADBAND_DEG:
        return 0.0
    speed = max(LOCK_SPEED_MIN_PCT,
                min(LOCK_PCT_MAX, abs(error) * LOCK_KP_PCT_PER_DEG))
    return math.copysign(speed, error)


def test_tapered_law_holds_within_small_band():
    hist = _simulate(_lock_command)
    worst = max(abs(e) for e in hist[-200:])
    assert worst < 4.0, (
        f"tapered law should hold heading within a small band under a steady "
        f"disturbance; last200 max|err|={worst:.2f}")


def test_tapered_law_dithers_less_than_hard_floor():
    soft = _peak_to_peak(_simulate(_lock_command))
    hard = _peak_to_peak(_simulate(_hard_floor_command))
    assert hard > soft + 0.5, (
        f"hard-floor peak-to-peak {hard:.2f} must exceed tapered {soft:.2f} by a "
        "clear margin -- the relay limit-cycle the taper removes (if not, the toy "
        "plant no longer reproduces the bug)")


# --------------------------------------------------------------------------- #
#  Fire-window quiet mode: a widened deadband holds a steady launcher heading  #
# --------------------------------------------------------------------------- #
def test_hold_mode_deadband_widens_the_no_command_zone():
    from duburi_control.heading_lock import LOCK_HOLD_DEADBAND_DEG
    # A 2 deg error corrects in normal mode (default deadband 1 deg) but is INSIDE
    # the widened quiet-mode deadband -> commands 0 (holds steady, no micro-correct).
    err = 0.5 * (LOCK_DEADBAND_DEG + LOCK_HOLD_DEADBAND_DEG)   # between the two
    assert abs(_lock_command(err)) > 0.0, 'normal mode must correct this error'
    assert _lock_command(err, LOCK_HOLD_DEADBAND_DEG) == 0.0, \
        'quiet-mode (wide deadband) must hold steady inside it'
    # Well outside even the wide deadband, quiet mode still corrects.
    big = LOCK_HOLD_DEADBAND_DEG + 2.0
    assert abs(_lock_command(big, LOCK_HOLD_DEADBAND_DEG)) > 0.0, \
        'quiet mode must still correct a real drift beyond the wide deadband'


def test_set_hold_mode_toggles_active_deadband():
    from duburi_control.heading_lock import (
        HeadingLock, LOCK_HOLD_DEADBAND_DEG)
    lock = HeadingLock(pixhawk=None, target_deg=0.0, yaw_source=None,
                       log=logging.getLogger('t'))
    assert lock._deadband_deg == LOCK_DEADBAND_DEG
    lock.set_hold_mode(True)
    assert lock._deadband_deg == LOCK_HOLD_DEADBAND_DEG
    lock.set_hold_mode(False)
    assert lock._deadband_deg == LOCK_DEADBAND_DEG
