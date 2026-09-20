"""Behaviour tests for the yaw settle law (motion_yaw).

The 2026-06 regression: a HARD min-speed floor (`max(7.5%, |raw|)`) pinned the
Ch4 yaw-rate command at >=7.5% for any error past a 1deg tolerance, so a heavy
hull overshot the band and limit-cycled forever -> every yaw command TIMEOUTed.

These tests pin the FIX's law shape (the floor now tapers to 0 at the tolerance,
unmasking the integral) plus a closed-loop toy-plant proof that the tapered law
SETTLES where the old hard-floor law WOBBLES. No ROS / MAVLink -- `_YawPID` and
`_yaw_floor` are pure, and `Pixhawk.heading_error` is a static.
"""

import math

from mongla_control.motion_yaw import (
    _YawPID, _yaw_floor,
    YAW_TOL_DEG, YAW_APPROACH_BAND_DEG,
    YAW_SPEED_MIN_PCT, YAW_SPEED_MAX_PCT,
)
from mongla_control.pixhawk import Pixhawk


# --------------------------------------------------------------------------- #
#  _yaw_floor -- tapered stiction floor                                        #
# --------------------------------------------------------------------------- #
def test_floor_full_outside_band():
    assert _yaw_floor(YAW_APPROACH_BAND_DEG) == YAW_SPEED_MIN_PCT
    assert _yaw_floor(YAW_APPROACH_BAND_DEG + 30.0) == YAW_SPEED_MIN_PCT


def test_floor_zero_at_tolerance_edge():
    # The whole point: at the lock tolerance the floor is ~0 so the command can
    # decay to a stop instead of being pinned at 7.5% (the limit-cycle driver).
    assert _yaw_floor(YAW_TOL_DEG) == 0.0


def test_floor_monotonic_linear_in_band():
    mid = 0.5 * (YAW_TOL_DEG + YAW_APPROACH_BAND_DEG)
    f_mid = _yaw_floor(mid)
    assert 0.0 < f_mid < YAW_SPEED_MIN_PCT
    # linear: midpoint floor is ~half the full floor
    assert f_mid == _approx(YAW_SPEED_MIN_PCT * 0.5)


def _approx(x, tol=1e-6):
    class _A:
        def __eq__(_s, other): return abs(other - x) <= tol
        def __repr__(_s): return f"~{x}"
    return _A()


# --------------------------------------------------------------------------- #
#  _YawPID.update -- decay near target, floor far, sign, integral unmask       #
# --------------------------------------------------------------------------- #
def test_inside_tolerance_commands_zero_and_resets_integral():
    pid = _YawPID()
    pid._i_acc = 5.0
    assert pid.update(0.5) == 0.0
    assert pid._i_acc == 0.0


def test_near_target_output_decays_not_pinned_to_floor():
    # THE regression guard. Just outside tol the command must be SMALL (P/D with
    # a ~0 floor), NOT pinned at the old hard 7.5% floor.
    pid = _YawPID()
    out = abs(pid.update(YAW_TOL_DEG + 0.3))   # ~2.3 deg error
    assert out < YAW_SPEED_MIN_PCT, (
        f"near-target command {out:.2f}% should decay below the {YAW_SPEED_MIN_PCT}% "
        "floor, not be pinned to it (the limit-cycle bug)")


def test_large_error_is_floored_and_capped():
    pid = _YawPID()
    out = pid.update(120.0)
    assert out == _approx(YAW_SPEED_MAX_PCT)   # capped
    pid2 = _YawPID()
    # An error well outside the band but with tiny raw still gets the full floor.
    assert abs(pid2.update(YAW_APPROACH_BAND_DEG + 1.0)) >= YAW_SPEED_MIN_PCT - 1e-9


def test_sign_follows_error():
    assert _YawPID().update(40.0) > 0
    assert _YawPID().update(-40.0) < 0


def test_integral_unmasked_and_grows_when_stalled_in_band():
    # Hold a fixed small in-band error (P/D below the tapered floor): the integral
    # must accumulate tick over tick (it was masked by the old hard max()).
    pid = _YawPID()
    err = YAW_TOL_DEG + 0.5
    pid.update(err)
    i1 = pid._i_acc
    for _ in range(5):
        pid.update(err)
    assert pid._i_acc > i1 > 0.0


# --------------------------------------------------------------------------- #
#  Closed-loop toy plant -- tapered law SETTLES, hard-floor law WOBBLES        #
# --------------------------------------------------------------------------- #
def _simulate(command_fn, *, start=40.0, target=0.0, steps=600,
              dt=0.1, gain_dps_per_pct=4.0, latency=1):
    """Kinematic yaw plant: heading integrates a rate ~ commanded %, with one
    tick of command latency (a heavy hull + sensor lag). Returns the heading
    history. `command_fn(error)` -> signed % yaw command.
    """
    heading = start
    pending = [0.0] * max(1, latency)
    hist = []
    for _ in range(steps):
        err = Pixhawk.heading_error(target, heading)
        cmd = command_fn(err)
        applied = pending.pop(0)
        pending.append(cmd)
        heading = (heading + applied * gain_dps_per_pct * dt) % 360.0
        hist.append(Pixhawk.heading_error(target, heading))
    return hist


def _settled(hist, tol, dwell=5):
    """True if the last `dwell` samples are all within tol (declares 'locked')."""
    return all(abs(e) <= tol for e in hist[-dwell:])


def test_tapered_law_settles():
    pid = _YawPID()
    hist = _simulate(pid.update)
    assert _settled(hist, YAW_TOL_DEG), (
        f"tapered law must settle within {YAW_TOL_DEG}deg; "
        f"final={hist[-1]:.2f} last5={[round(e,2) for e in hist[-5:]]}")


def test_hard_floor_law_limit_cycles():
    # Reconstruct the OLD hard-floor law and show it never holds in-band -- the
    # exact wobble the fix removes. (Pins the fix to the mechanism.)
    def hard_floor(error):
        if abs(error) <= YAW_TOL_DEG:
            return 0.0
        speed = max(YAW_SPEED_MIN_PCT, min(YAW_SPEED_MAX_PCT, abs(1.2 * error)))
        return math.copysign(speed, error)

    hist = _simulate(hard_floor)
    assert not _settled(hist, YAW_TOL_DEG), (
        "hard-floor law should limit-cycle (never hold in tol) -- if this holds, "
        "the toy plant no longer reproduces the bug and the settle test is weak")
