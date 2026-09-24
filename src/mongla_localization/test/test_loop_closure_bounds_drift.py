"""Does a recognised place actually BOUND the filter's drift -- and only then?

`mongla_vision/test/test_loop_closure_refuses.py` covers when a closure is
ALLOWED. This covers what one DOES to the filter, which is the half that
matters: `update_position` shrinks the covariance, so a closure that should not
have fired is not merely wrong, it is BELIEVED.

⛔ THE FIXTURE AIDS VELOCITY, AND THAT IS LOAD-BEARING. An earlier version
aided only depth and yaw. Those do not observe x or y -- which is true, and is
exactly why loop closure is worth having -- but the resulting velocity sigma
reached 21 m/s, and in that regime a position fix spends most of its
innovation ROTATING ATTITUDE: `_inject` applies the correction on the left, so
`p <- dR p + dp`, and a large `dtheta` swings the position instead of moving
it. Measured there: a fix at the origin moved a 1.50 m error to 2.91 m, on the
far side. `update_position` warns about this coupling at 0.18 rad; unaided, it
is far past that.

That is a real property of the filter and not a defect, but it is not the
vehicle's regime: the downward camera feeds `update_body_velocity_xy` (verified
to 1.09 cm over 30 cm). With velocity aided, the same fix takes a 4.55 m error
to 0.000 m and leaves yaw at 0.000 deg. So the fixture aids velocity, drifts
through a BIASED velocity -- which is what actually moves a flow-aided vehicle
off truth -- and `test_the_fixture_is_the_vehicles_regime` fails if that stops
being so.
"""
import numpy as np
import pytest

from mongla_localization.inekf import RIEKF

GYRO = np.zeros(3)
ACCEL = np.array([0.0, 0.0, 9.80665])
DT = 0.02
VX = 0.30                 # true speed, m/s
FLOW_BIAS = 0.05          # the flow's error: what actually drifts position
SECONDS = 20.0
TRUTH_X = VX * SECONDS
FLOW_VAR = 0.0025         # (0.05 m/s)^2


def _drifted(seconds: float = SECONDS, flow_bias: float = FLOW_BIAS):
    """A flow-aided vehicle whose flow is slightly wrong -- the real failure.

    Depth, yaw and body velocity at 10 Hz, which is what the vehicle supplies.
    """
    f = RIEKF()
    f.X.v = np.array([VX, 0.0, 0.0])
    for i in range(int(seconds / DT)):
        f.predict(GYRO, ACCEL, DT)
        if i % 5 == 0:
            f.update_depth(0.0, sigma=0.05)
            f.update_yaw(0.0, sigma_deg=2.0)
            f.update_body_velocity_xy(VX + flow_bias, 0.0, FLOW_VAR, FLOW_VAR)
    return f


def _err(f):
    return abs(float(f.X.p[0]) - TRUTH_X)


def _pos_var(f):
    return float(f.P[6, 6] + f.P[7, 7])


# --------------------------------------------------------------------------- #
#  The fixture must be a real drift, in the vehicle's actual regime
# --------------------------------------------------------------------------- #
def test_the_setup_actually_drifts():
    assert _err(_drifted()) > 1.0, 'the fixture never left the truth'


def test_the_fixture_is_the_vehicles_regime():
    """Velocity must be OBSERVED, or the position update rotates attitude
    instead of moving position (see the module docstring)."""
    f = _drifted()
    assert f._velocity_is_observed(), (
        'velocity is unobserved, so a position fix will swing attitude and '
        'every number below describes a regime the vehicle is never in')


def test_flow_aiding_still_leaves_position_unbounded():
    """The case for loop closure: velocity aiding slows the drift, it does not
    bound it. A biased velocity integrates into position without limit."""
    near, far = _drifted(10.0), _drifted(40.0)
    assert _err(far) > _err(near) * 1.5, (
        'position error stopped growing with time -- if something now bounds '
        'it, the case for loop closure has changed and should be re-argued')


# --------------------------------------------------------------------------- #
#  What a closure does
# --------------------------------------------------------------------------- #
def test_a_closure_removes_the_drift():
    f = _drifted()
    before = _err(f)
    assert f.update_position((TRUTH_X, 0.0), sigma=0.30)
    assert _err(f) < 0.01, f'{before:.3f} m -> {_err(f):.3f} m'


def test_a_closure_does_not_swing_the_attitude():
    """The failure mode of the unaided regime, pinned as a guard."""
    f = _drifted()
    f.update_position((TRUTH_X, 0.0), sigma=0.30)
    assert abs(f.X.yaw_deg()) < 1.0, (
        f'yaw moved to {f.X.yaw_deg():.3f} deg on a POSITION fix -- the '
        f'correction is going into attitude through P[0:3,6:9]')


def test_a_closure_shrinks_position_covariance():
    f = _drifted()
    before = _pos_var(f)
    f.update_position((TRUTH_X, 0.0), sigma=0.30)
    assert _pos_var(f) < before


def test_a_looser_closure_never_corrects_more_than_a_tighter_one():
    """Sigma must not be decoration: composing it in the closure layer only
    means something if the filter orders corrections by it.

    ⚠ Weakly, here. With position sigma around 20 m, every sigma from 0.05 to
    1.0 lands on the truth; only 5.0 leaves a visible residual. So this
    asserts MONOTONICITY, which is what is actually true, rather than a
    separation the measurement does not support.
    """
    res = []
    for s in (0.05, 0.3, 1.0, 5.0):
        f = _drifted()
        f.update_position((TRUTH_X, 0.0), sigma=s)
        res.append(_err(f))
    assert all(b >= a - 1e-9 for a, b in zip(res, res[1:])), (
        f'residual did not grow with sigma: {res}')
    assert res[-1] > res[0], f'sigma changed nothing at all: {res}'


def test_repeating_one_closure_does_not_collapse_the_covariance():
    """One stored place re-offered every frame is ONE observation, not fifty.

    Measured: the 2nd through 200th identical fix leave the position variance
    unchanged to four figures. That the filter already resists this is worth
    pinning -- but it is NOT why the age and travel gates exist. A real
    self-closure does not repeat a fixed number; it hands back the vehicle's
    own drifting estimate, which this cannot simulate and the gates prevent.
    """
    f = _drifted()
    f.update_position((TRUTH_X, 0.0), sigma=0.30)
    once = _pos_var(f)
    for _ in range(200):
        f.update_position((TRUTH_X, 0.0), sigma=0.30)
    many = _pos_var(f)
    assert many <= once * 1.01
    assert many > once * 0.5, (
        f'variance fell {once:.3e} -> {many:.3e} on ONE repeated observation; '
        f'the filter is counting it as new information')


def test_a_wrong_closure_is_believed_which_is_why_the_gates_exist():
    """Not a bug report -- the justification for every refusal in
    `mongla_vision/anchor/loop_closure.py`, pinned so it cannot be argued away.

    A fix at a place the vehicle never was moves the estimate there AND makes
    it more certain. Nothing downstream recovers from that, so the only
    defence is not emitting it.
    """
    f = _drifted()
    wrong = np.array([TRUTH_X + 40.0, -30.0])
    before = _pos_var(f)
    f.update_position(tuple(wrong), sigma=0.10)
    assert np.hypot(*(f.X.p[:2] - wrong)) < 1.0, (
        'the filter did not accept the wrong fix -- if it now rejects one, '
        'the innovation gate is doing work the closure layer is credited for')
    assert _pos_var(f) < before, 'the filter did not become MORE certain'


@pytest.mark.parametrize('sigma', [0.0, -1.0, float('nan')])
def test_a_nonsense_sigma_must_not_produce_a_non_finite_state(sigma):
    """The closure layer never emits one; this records what it would cost.

    Zero sigma is infinite confidence, and the filter cannot tell a number
    that was missing from one that is merely tiny -- the same shape as
    rendering an absent measurement as 0.0 instead of `--`.
    """
    f = _drifted()
    try:
        f.update_position((TRUTH_X, 0.0), sigma=sigma)
    except Exception:
        return                      # refusing outright is a fine answer
    assert np.all(np.isfinite(f.X.p)), f'sigma={sigma} produced a broken state'
