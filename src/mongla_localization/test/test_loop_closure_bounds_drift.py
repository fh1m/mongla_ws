"""Does a recognised place actually BOUND the filter's drift -- and only then?

`mongla_vision/test/test_loop_closure_refuses.py` covers when a closure is
ALLOWED. This covers what one DOES to the filter, which is the half that
matters: `update_position` shrinks the covariance, so a closure that should not
have fired is not merely wrong, it is BELIEVED.

⛔ AND EVERY NUMBER IN THIS FILE WAS ONCE MEASURED ON A FILTER IN FREE FALL.
`ACCEL` read `+9.80665` until 2026-09-30. `predict` computes
`a_world = R @ a + GRAVITY` with `GRAVITY = +z` (world z is DOWN), so a level
stationary hull reports `-9.80665`; `+9.80665` is **2 g downward**. Measured:
z reached **3 914 m at 391 m/s** in 20 s -- analytically 0.5*19.6*400 = 3 921 --
and **depth was rejected 198 of 200 times**, for ever, because the innovation was
kilometres. `test_the_setup_actually_drifts` passed throughout, because it checks
only x. So the file's own docstring numbers, and two of its assertions, described
a vehicle 3.9 km under the pool.

⚠ Fixing the sign changed two things this file BELIEVED, and both are now
measured rather than asserted -- see
`test_a_wrong_closure_within_the_gate_is_believed` and
`test_repeating_one_closure_DOES_shrink_the_covariance`. It also removed the
only evidence for a narrowing of the NIS lockout break that was shipped and then
reverted on the same day (B-77).

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
ACCEL = np.array([0.0, 0.0, -9.80665])   # see the GRAVITY note below
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


def _err(f, seconds: float = SECONDS):
    """Along-track error against the truth for THIS run's duration.

    ⚠ It used to compare every run against `TRUTH_X`, the 20 s truth, so a 10 s
    run scored 2.5 m of "drift" that was simply 3 m of unflown distance. With
    the duration passed in, the number is the flow bias integrated:
    0.05 m/s * t, which is what actually moves a flow-aided hull off truth.
    """
    return abs(float(f.X.p[0]) - VX * seconds)


def _pos_var(f):
    return float(f.P[6, 6] + f.P[7, 7])


# --------------------------------------------------------------------------- #
#  The fixture must be a real drift, in the vehicle's actual regime
# --------------------------------------------------------------------------- #
def test_the_setup_actually_drifts():
    """And it drifts by the flow bias, not by a broken accelerometer.

    Measured: 0.99999 m over 20 s, against FLOW_BIAS * SECONDS = 1.0 m exactly.
    Pinned to the analytic value rather than to `> 1.0`, which is what let a
    2 g fixture through -- it drifted 4.55 m in x while falling 3 914 m in z,
    and `> 1.0` called that a pass.
    """
    f = _drifted()
    assert _err(f) == pytest.approx(FLOW_BIAS * SECONDS, abs=0.01), (
        f'{_err(f):.4f} m of along-track drift, expected '
        f'{FLOW_BIAS * SECONDS:.4f} m from the flow bias alone')
    assert abs(float(f.X.p[2])) < 0.01, (
        f'z is {f.X.p[2]:.3f} m: the hull is not level and stationary in depth, '
        f'so nothing below this describes the vehicle. Check the ACCEL sign.')


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
    assert _err(far, 40.0) > _err(near, 10.0) * 1.5, (
        'position error stopped growing with time -- if something now bounds '
        'it, the case for loop closure has changed and should be re-argued')


# --------------------------------------------------------------------------- #
#  What a closure does
# --------------------------------------------------------------------------- #
def test_a_closure_removes_the_drift():
    f = _drifted()
    before = _err(f)
    assert f.update_position((TRUTH_X, 0.0), sigma=0.30)
    # ⚠ NOT zero, and it should not be. sigma 0.30 against a position variance
    # of 2.04 leaves the measurement about 87 % of the weight, so 1.000 m of
    # drift becomes 0.082 m of residual -- measured. The old bar was < 0.01,
    # which the free-falling fixture met only because its P was enormous and
    # the fix therefore took essentially all the weight.
    assert _err(f) < 0.10, f'{before:.3f} m -> {_err(f):.3f} m'


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


def test_repeating_one_closure_DOES_shrink_the_covariance():
    """⛔ THE OPPOSITE OF WHAT THIS FILE USED TO CLAIM, and the correction
    matters, because the claim was used to argue the gates were belt-and-braces.

    It asserted that the 2nd through 200th identical fix "leave the position
    variance unchanged to four figures", and the filter obliged -- while falling
    at 391 m/s, where P was so large that one 0.30 m fix could not move it.

    On a level hull, measured:

        after   1 identical fix    pos_var 0.170803
        after   2                          0.091136
        after   5                          0.039978
        after  10                          0.022317
        after  50                          0.007954
        after 200                          0.005235

    **A 33x collapse from re-offering ONE stored place.** The filter has no way
    to know the fix is the same observation twice; a Kalman update is
    unconditionally information. So the age and travel gates in
    `mongla_vision/anchor/loop_closure.py` are LOAD-BEARING, not defensive
    decoration -- they are the only thing standing between one recognised place
    and a filter that is 33x more certain than its evidence.
    """
    f = _drifted()
    f.update_position((TRUTH_X, 0.0), sigma=0.30)
    once = _pos_var(f)
    for _ in range(200):
        f.update_position((TRUTH_X, 0.0), sigma=0.30)
    many = _pos_var(f)
    assert many < once * 0.05, (
        f'variance {once:.6f} -> {many:.6f}: if a repeated identical fix has '
        f'stopped adding certainty, the filter gained a duplicate-observation '
        f'defence and the closure gates can be re-argued -- until then they '
        f'are the defence')


def test_a_wrong_closure_within_the_gate_is_believed():
    """Not a bug report -- the justification for every refusal in
    `mongla_vision/anchor/loop_closure.py`, pinned so it cannot be argued away.

    ⚠ CORRECTED 2026-09-30. This used to offer a fix 50 m away and assert the
    filter took it. On a level hull the chi-square gate REJECTS that outright,
    which would have read as "the innovation gate protects us" -- and it does
    not, because a place-recognition false positive does not land 50 m away.
    Measured, position variance 2.04, fix sigma 0.10:

        offset 0.5 .. 4.0 m   ACCEPTED, and pos_var 2.0365 -> 0.0256  (80x)
        offset 5.0 m and out  rejected

    ⛔ So the gate catches the absurd closure and waves through the PLAUSIBLE
    one -- the only kind a real false positive produces. A fix at a place the
    vehicle never was moves the estimate there AND makes it 80x more certain.
    Nothing downstream recovers from that, so the only defence is not emitting
    it.
    """
    f = _drifted()
    wrong = np.array([TRUTH_X + 4.0, 0.0])          # inside the gate, measured
    before = _pos_var(f)
    assert f.update_position(tuple(wrong), sigma=0.10), (
        'a 4 m closure is no longer accepted -- re-measure the gate reach '
        'before trusting the numbers in this docstring')
    assert np.hypot(*(f.X.p[:2] - wrong)) < 0.05, (
        f'the filter moved to {f.X.p[:2]}, not to the wrong fix at {wrong}')
    assert _pos_var(f) < before * 0.05, (
        f'pos_var {before:.4f} -> {_pos_var(f):.4f}: the filter did not become '
        f'dramatically MORE certain, which is the whole hazard')


def test_the_gate_rejects_an_absurd_closure_but_that_is_not_the_defence():
    """The other half of the pair above, so neither can be quoted alone."""
    f = _drifted()
    assert f.update_position((TRUTH_X + 40.0, -30.0), sigma=0.10) is False
    assert _err(f) == pytest.approx(FLOW_BIAS * SECONDS, abs=0.01), \
        'a rejected fix must leave the estimate exactly where it was'


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
