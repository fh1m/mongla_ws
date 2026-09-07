"""The navigation filter, against constructed truth.

The properties under test are the ones that make it safe to ACT on, not that a
number comes out:

  * a measured velocity is tracked, and its uncertainty falls;
  * position drift is REPORTED, and grows without a fix;
  * a bad fix is rejected rather than integrated;
  * `position_trustworthy` refuses when it has never had a fix -- the state
    where every number is small and none of it is earned.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_manager.estimator.nav_estimator import (      # noqa: E402
    MAX_SPEED_MS, NavEstimator, NavState, stationary_from_imu)

DT = 0.033


def _run(est, secs, yaw=0.0, fix=None, sigma=0.02, t0=0.0):
    """Drive the filter for `secs`, optionally feeding a velocity each step."""
    t = t0
    n = int(secs / DT)
    for _ in range(n):
        t += DT
        est.predict(t, yaw)
        if fix is not None:
            est.update_velocity(fix[0], fix[1], sigma)
    return t


# --------------------------------------------------------------------------- #
#  Velocity: measured, so it converges
# --------------------------------------------------------------------------- #
def test_a_measured_velocity_is_tracked():
    est = NavEstimator()
    _run(est, 3.0, fix=(0.40, 0.0))
    s = est.state()
    assert s.vx == pytest.approx(0.40, abs=0.02)
    assert abs(s.vy) < 0.02
    assert s.vel_sigma < 0.05, 'uncertainty did not fall with fixes'


def test_position_integrates_velocity_through_YAW():
    """The one place the frames couple. A hull heading east at 0.5 m/s must put
    its displacement on the EAST axis, not the body one -- getting this wrong
    is a navigation error that looks perfectly healthy in the velocity."""
    est = NavEstimator()
    _run(est, 4.0, yaw=math.radians(90.0), fix=(0.50, 0.0))
    s = est.state()
    # body-forward at yaw 90 deg -> +y in the local frame
    assert s.py == pytest.approx(2.0, rel=0.1)
    assert abs(s.px) < 0.2


# --------------------------------------------------------------------------- #
#  Position: dead-reckoned, and it SAYS so
# --------------------------------------------------------------------------- #
def test_position_uncertainty_GROWS_without_a_fix():
    """The honest part. Position is unobservable with this sensor set, so the
    only safe output is one whose sigma keeps growing until a fix arrives."""
    est = NavEstimator()
    _run(est, 2.0, fix=(0.3, 0.0))
    fixed = est.state().pos_sigma
    _run(est, 5.0, t0=2.0)                     # no fixes at all
    blind = est.state().pos_sigma
    assert blind > fixed * 2.0, 'drift was not reported'
    assert est.state().since_fix_s > 4.0


def test_a_filter_with_NO_FIX_is_never_trustworthy():
    """Before any measurement the state is all zeros with a small sigma, which
    reads as a confident origin. It is not: it is an untested guess."""
    est = NavEstimator()
    _run(est, 1.0)
    assert est.state().n_fixes == 0
    assert not est.state().position_trustworthy(10.0)


def test_position_trustworthy_tracks_the_growing_sigma():
    est = NavEstimator()
    _run(est, 2.0, fix=(0.3, 0.0))
    assert est.state().position_trustworthy(1.0)
    _run(est, 20.0, t0=2.0)                    # long blind stretch
    assert not est.state().position_trustworthy(0.1), \
        'a 20 s blind drift still claimed 10 cm'


def test_reset_position_makes_HERE_exact():
    est = NavEstimator()
    _run(est, 3.0, fix=(0.5, 0.0))
    est.reset_position()
    s = est.state()
    assert s.px == 0.0 and s.py == 0.0
    assert s.pos_sigma == pytest.approx(0.0, abs=1e-9), \
        'a deliberate re-origin kept drift that no longer exists'


# --------------------------------------------------------------------------- #
#  Rejecting what should not be fused
# --------------------------------------------------------------------------- #
def test_an_ABSURD_speed_is_rejected():
    est = NavEstimator()
    assert not est.update_velocity(MAX_SPEED_MS + 1.0, 0.0, 0.02)
    assert est.state().n_rejected == 1


def test_a_WILD_fix_is_gated_out_once_the_filter_is_confident():
    """Mahalanobis gating. After a run of consistent fixes, a single reversal
    is far more likely to be a bad flow interval than real -- and one such fix
    poisons the velocity the position then integrates."""
    est = NavEstimator()
    _run(est, 4.0, fix=(0.40, 0.0), sigma=0.01)
    before = est.state().vx
    ok = est.update_velocity(-1.5, 0.0, 0.01)
    assert not ok, 'a 1.9 m/s reversal was fused'
    assert est.state().vx == pytest.approx(before, abs=1e-6)


def test_NaN_and_bad_sigma_are_refused():
    est = NavEstimator()
    assert not est.update_velocity(float('nan'), 0.0, 0.02)
    assert not est.update_velocity(0.1, float('inf'), 0.02)
    assert not est.update_velocity(0.1, 0.0, 0.0)


def test_an_OUT_OF_ORDER_sample_does_not_roll_the_state_back():
    """A late message must be ignored, not integrated with negative dt --
    that is how a filter acquires a velocity it never had."""
    est = NavEstimator()
    t = _run(est, 2.0, fix=(0.3, 0.0))
    before = est.state()
    est.predict(t - 1.0, 0.0)
    after = est.state()
    assert after.px == pytest.approx(before.px) and after.vx == pytest.approx(before.vx)


# --------------------------------------------------------------------------- #
#  ZUPT, and the trap it exists to avoid
# --------------------------------------------------------------------------- #
def test_a_ZUPT_pulls_velocity_to_zero_and_STOPS_the_drift():
    est = NavEstimator()
    _run(est, 3.0, fix=(0.5, 0.0))
    for i in range(60):
        est.predict(3.0 + i * DT, 0.0)
        est.update_zero_velocity()
    s = est.state()
    assert abs(s.vx) < 0.05 and s.vel_sigma < 0.05


def test_stationarity_needs_ALL_THREE_signals():
    """Each one alone has a failure mode that looks exactly like stillness: a
    hull held motionless by opposing thrust is inertially quiet, and one
    drifting on the current under no thrust is thrust-quiet and moving."""
    assert stationary_from_imu(0.001, 0.001, 0.001)
    assert not stationary_from_imu(0.50, 0.001, 0.001)    # rotating
    assert not stationary_from_imu(0.001, 0.50, 0.001)    # accelerating
    assert not stationary_from_imu(0.001, 0.001, 0.50)    # thrusting


def test_the_gyro_threshold_sits_above_THIS_BOARDS_noise():
    """Measured static: gyro sd 8-15 mrad/s. A threshold at or below that
    would call a moving vehicle still whenever the gyro happened to be quiet."""
    from duburi_manager.estimator import nav_estimator as N
    import inspect
    sig = inspect.signature(N.stationary_from_imu)
    assert sig.parameters['gyro_max'].default > 0.015
