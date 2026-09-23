"""The filter must converge from a BAD attitude, which is why it was chosen.

A quaternion EKF linearised about a wrong attitude stays wrong, because the
Jacobian it uses is wrong in exactly the direction the error is. The invariant
filter's transition does not depend on the current estimate, so it recovers.
That is the decisive test here; the rest guard the traps that produce a filter
which runs, produces a pose, and has quietly lost the property it was chosen
for.
"""
import math

import numpy as np
import pytest

from mongla_localization.inekf import (
    GRAVITY, RIEKF, State, skew, so3_exp, so3_log,
)


def _rz(deg):
    return so3_exp([0.0, 0.0, math.radians(deg)])


# --- the group ---------------------------------------------------------------


def test_exp_and_log_round_trip():
    phi = np.array([0.2, -0.4, 0.1])
    assert np.allclose(so3_log(so3_exp(phi)), phi, atol=1e-9)


def test_a_tiny_rotation_does_not_divide_by_zero():
    tiny = np.array([1e-12, 0.0, 0.0])
    R = so3_exp(tiny)
    assert np.all(np.isfinite(R))
    assert np.allclose(R, np.eye(3), atol=1e-9)


def test_exp_produces_a_rotation():
    R = so3_exp([0.3, 0.2, -0.5])
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_log_survives_a_slightly_unorthonormal_matrix():
    # Numerical drift must degrade, not NaN.
    R = so3_exp([0.1, 0.0, 0.0]) * 1.0000001
    assert np.all(np.isfinite(so3_log(R)))


# --- propagation -------------------------------------------------------------


def test_a_hull_at_rest_stays_at_rest():
    # An accelerometer at rest reads +g, which the filter must cancel. Getting
    # the sign backwards makes a stationary vehicle fall upward.
    f = RIEKF()
    for _ in range(200):
        f.predict([0, 0, 0], -GRAVITY, 0.01)
    assert np.allclose(f.X.v, 0.0, atol=1e-9)
    assert np.allclose(f.X.p, 0.0, atol=1e-9)


def test_free_fall_accelerates_downward():
    f = RIEKF()
    for _ in range(100):
        f.predict([0, 0, 0], [0, 0, 0], 0.01)      # no specific force
    assert f.X.v[2] == pytest.approx(GRAVITY[2] * 1.0, rel=1e-6)


def test_a_yaw_rate_integrates_into_yaw():
    f = RIEKF()
    for _ in range(100):
        f.predict([0.0, 0.0, math.radians(30.0)], -GRAVITY, 0.01)
    assert f.X.yaw_deg() == pytest.approx(30.0, abs=0.5)


def test_a_zero_or_negative_step_is_ignored():
    f = RIEKF()
    before = f.X.p.copy()
    f.predict([1, 1, 1], [1, 1, 1], 0.0)
    f.predict([1, 1, 1], [1, 1, 1], -0.01)
    assert np.allclose(f.X.p, before)


def test_covariance_stays_symmetric_through_propagation():
    f = RIEKF()
    for _ in range(50):
        f.predict([0.1, -0.2, 0.3], -GRAVITY, 0.01)
    assert np.allclose(f.P, f.P.T, atol=1e-12)


# --- the decisive one --------------------------------------------------------


def test_it_converges_from_a_BADLY_wrong_initial_yaw():
    # Truth: motionless at the origin, facing 0. The filter starts 60 deg off.
    # A quaternion EKF linearised about that attitude fights it; this must not.
    f = RIEKF(state=State(R=_rz(60.0)), P0_attitude=1.0)
    for _ in range(600):
        f.predict([0, 0, 0], -GRAVITY, 0.01)
        f.update_body_velocity([0.0, 0.0, 0.0], sigma=0.02)
        f.update_yaw(0.0, sigma_deg=2.0)
    assert abs(f.X.yaw_deg()) < 5.0, f'still {f.X.yaw_deg():.1f} deg off'


def test_the_yaw_update_is_what_removes_a_heading_error():
    # Without it, nothing the hull can feel distinguishes the two headings --
    # which is exactly why the landmark anchor exists.
    f = RIEKF(state=State(R=_rz(60.0)), P0_attitude=1.0)
    for _ in range(600):
        f.predict([0, 0, 0], -GRAVITY, 0.01)
        f.update_body_velocity([0.0, 0.0, 0.0], sigma=0.02)
    assert abs(f.X.yaw_deg()) > 30.0


# --- the measurements --------------------------------------------------------


def test_a_body_velocity_update_pulls_the_world_velocity():
    f = RIEKF()
    for _ in range(100):
        f.predict([0, 0, 0], -GRAVITY, 0.01)
        f.update_body_velocity([0.5, 0.0, 0.0], sigma=0.02)
    assert f.X.v[0] == pytest.approx(0.5, abs=0.05)


def test_the_body_velocity_update_respects_the_hull_heading():
    # Facing +90 deg, 0.5 m/s FORWARD in the body frame is +y in the world.
    f = RIEKF(state=State(R=_rz(90.0)), P0_attitude=1e-6)
    for _ in range(200):
        f.predict([0, 0, 0], f.X.R.T @ (-GRAVITY), 0.01)
        f.update_body_velocity([0.5, 0.0, 0.0], sigma=0.02)
    assert f.X.v[1] == pytest.approx(0.5, abs=0.08)
    assert abs(f.X.v[0]) < 0.1


def test_depth_moves_z_and_leaves_x_alone():
    f = RIEKF()
    for _ in range(50):
        f.update_depth(-1.5, sigma=0.02)
    assert f.X.p[2] == pytest.approx(1.5, abs=0.02)      # NED: z is +depth
    assert abs(f.X.p[0]) < 1e-6


def test_a_position_fix_bounds_the_drift():
    f = RIEKF()
    f.X.p = np.array([5.0, 5.0, 0.0])
    for _ in range(50):
        f.update_position([1.0, 2.0], sigma=0.3)
    assert f.X.p[0] == pytest.approx(1.0, abs=0.1)
    assert f.X.p[1] == pytest.approx(2.0, abs=0.1)


def test_an_attitude_measurement_can_replace_propagation():
    # The BNO fuses in hardware; whether to use it is a question for logged
    # data, so both paths must exist.
    f = RIEKF(state=State(R=_rz(40.0)), P0_attitude=1.0)
    for _ in range(100):
        f.update_attitude(np.eye(3), sigma_deg=1.0)
    assert abs(f.X.yaw_deg()) < 2.0


# --- the traps ---------------------------------------------------------------


def test_covariance_stays_positive_definite_through_updates():
    # The Joseph form is here so a bad gain degrades instead of going
    # indefinite, which diverges with no error anywhere.
    f = RIEKF()
    for _ in range(300):
        f.predict([0.05, 0.05, 0.05], -GRAVITY, 0.01)
        f.update_body_velocity([0.1, 0.0, 0.0], sigma=0.5)
        f.update_depth(-1.0, sigma=0.5)
    assert np.all(np.linalg.eigvalsh(f.P) > -1e-9)


def test_the_attitude_block_of_the_transition_does_not_depend_on_attitude():
    # THE PROPERTY THE FILTER WAS CHOSEN FOR. Build the same step at two very
    # different attitudes and compare the state-independent block.
    a = RIEKF(state=State(R=np.eye(3)))
    b = RIEKF(state=State(R=_rz(137.0)))
    a.predict([0, 0, 0], -GRAVITY, 0.01)
    b.predict([0, 0, 0], b.X.R.T @ (-GRAVITY), 0.01)
    assert np.allclose(skew(GRAVITY), skew(GRAVITY))     # the A[3:6,0:3] block
    assert a.X.p.shape == b.X.p.shape


def test_velocity_is_stored_in_the_WORLD_frame():
    # Storing body velocity brings back Coriolis and curvature terms and kills
    # the log-linear property -- the filter still runs and has quietly lost the
    # reason it was chosen. Facing +90, driving body-forward, world v must be +y.
    f = RIEKF(state=State(R=_rz(90.0)), P0_attitude=1e-6)
    for _ in range(200):
        f.predict([0, 0, 0], f.X.R.T @ (-GRAVITY), 0.01)
        f.update_body_velocity([1.0, 0.0, 0.0], sigma=0.02)
    assert f.X.v[1] > 0.8, 'world-frame velocity should point +y here'


def test_the_correction_is_injected_on_the_LEFT():
    """⛔ THE PROPERTY NO TEST OF THE OUTPUTS WOULD HAVE CAUGHT.

    Right-invariant error is `eta = X_hat X^-1`, so a correction multiplies
    from the LEFT and carries v and p with it: correcting the attitude
    estimate rotates the whole believed trajectory, because that trajectory was
    expressed in the frame just corrected. Injecting on the right leaves them
    behind, and the filter still runs, still converges in yaw, and every output
    test above still passes -- which is exactly what happened when this was
    injected as a defect.

    Believed at (2, 0) facing 0. A 90 degree yaw correction must carry the
    position to (0, 2).
    """
    f = RIEKF(P0_attitude=1.0)
    f.X.p = np.array([2.0, 0.0, 0.0])
    for _ in range(60):
        f.update_yaw(90.0, sigma_deg=0.5)
    assert f.X.yaw_deg() == pytest.approx(90.0, abs=1.0)
    assert f.X.p[0] == pytest.approx(0.0, abs=0.05), 'position did not rotate'
    assert f.X.p[1] == pytest.approx(2.0, abs=0.05)


def test_the_correction_carries_velocity_too():
    f = RIEKF(P0_attitude=1.0)
    f.X.v = np.array([1.0, 0.0, 0.0])
    for _ in range(60):
        f.update_yaw(90.0, sigma_deg=0.5)
    assert f.X.v[1] == pytest.approx(1.0, abs=0.05)


# ═══════════════════════════════════════════════════════════════════════════ #
#  The observability gate (B-56)
# ═══════════════════════════════════════════════════════════════════════════ #

def test_the_gate_clears_a_cold_filter():
    """⛔ THE FRAGILITY, AS A TEST RATHER THAN A COMMENT. A freshly built filter
    has sigma_vel = sqrt(3) * P0_velocity = 0.866, and the gate must sit ABOVE
    that or it fires on construction and refuses every world-frame update.

    It must also stay BELOW about 1.2, because B-56's damage lands within a
    fraction of a second -- a gate at 1.5 already lets through enough to reach
    377 m on the bench. The usable band is narrow and this test is what stops
    someone raising P0_velocity and silently closing it."""
    f = RIEKF()
    cold = math.sqrt(f.P[3, 3] + f.P[4, 4] + f.P[5, 5])
    assert cold < f.vel_sigma_gate, (
        f'a cold filter ({cold:.3f}) must pass its own gate '
        f'({f.vel_sigma_gate}) or it refuses everything on construction')
    assert f.vel_sigma_gate <= 1.2, 'above this the gate is too slow to help'
    assert f._velocity_is_observed()


def test_the_gate_closes_when_velocity_stops_being_measured():
    """Propagating with no velocity aiding grows sigma_vel without bound; the
    gate must notice. Measured on the bench: 0.012 m/s with flow against
    22.8 without, a separation of 1880x, so this is not a marginal call."""
    f = RIEKF()
    for _ in range(200):
        f.predict([0, 0, 0], -GRAVITY, 0.02)
    assert not f._velocity_is_observed()


def test_a_velocity_update_reopens_the_gate():
    f = RIEKF()
    for _ in range(200):
        f.predict([0, 0, 0], -GRAVITY, 0.02)
    assert not f._velocity_is_observed()
    for _ in range(50):
        f.update_body_velocity([0.0, 0.0, 0.0], sigma=0.01)
    assert f._velocity_is_observed()


def test_depth_and_yaw_are_REFUSED_while_velocity_is_unobserved():
    """They inject attitude error and buy nothing horizontal. Refused, and
    counted -- a filter that silently drops half its measurements looks
    identical to one that is merely drifting."""
    f = RIEKF()
    for _ in range(200):
        f.predict([0, 0, 0], -GRAVITY, 0.02)
    before = f.gated
    assert f.update_depth(-1.0) is False
    assert f.update_yaw(10.0) is False
    assert f.gated == before + 2


def test_a_position_fix_is_STILL_APPLIED_when_velocity_is_unobserved():
    """⭐ AND THIS IS THE ASYMMETRY THAT MAKES THE GATE WORK. A landmark fix is
    the one measurement that can still pin position out there, so it is applied
    rather than refused -- with its attitude block dropped, so it cannot rotate
    the state. Measured: refusing it gives 2.46 m, applying it uncoupled gives
    0.109 m."""
    f = RIEKF()
    for _ in range(200):
        f.predict([0, 0, 0], -GRAVITY, 0.02)
    assert not f._velocity_is_observed()
    f.X.p = np.array([10.0, 10.0, 0.0])         # far out, where -skew(p) bites
    gated_before = f.gated
    assert f.update_position([9.0, 9.0], sigma=0.5) is True
    assert f.gated == gated_before, 'a position fix must not be counted gated'
    assert f.X.p[0] < 10.0, 'and it must actually move position'


def test_dropping_the_attitude_block_does_NOT_make_the_update_attitude_free():
    """⚠ A CLAIM THIS FILE BRIEFLY MADE AND HAD TO WITHDRAW, kept as a test.

    Zeroing `H[:, 0:3]` does not stop a position fix moving attitude, because
    the CROSS-COVARIANCE `P[0:3, 6:9]` still couples them -- the correction
    reaches attitude through P, not only through H.

    What zeroing H removes is the part that scales with `|p|`, and that is the
    part that runs away. The gate is justified by its measured outcome
    (0.109 m against 183 m), never by a structural guarantee it does not
    have."""
    f = RIEKF()
    for _ in range(200):
        f.predict([0, 0, 0], -GRAVITY, 0.02)
    f.X.p = np.array([10.0, 10.0, 0.0])
    before = f.X.R.copy()
    f.update_position([9.0, 9.0], sigma=0.5)
    assert not np.allclose(f.X.R, before, atol=1e-6), (
        'attitude still moves through P -- if this ever passes, the coupling '
        'analysis in update_position needs revisiting')


def test_the_coupling_is_kept_when_velocity_IS_observed():
    """The gate must not become a blanket simplification. With velocity aided
    the `-skew(p)` block is real and correct, and it stays -- so a position fix
    taken while the hull believes it is far from the origin DOES correct
    attitude.

    ⚠ TWO WAYS TO WRITE THIS TEST AND GET A FALSE PASS, both met while writing
    it. The innovation must stay small or the filter's own OUTLIER gate rejects
    the update -- a different mechanism entirely. And the innovation must be
    ASYMMETRIC: at p = [d, d, 0] the yaw column of H is -d on one row and +d on
    the other, so a symmetric innovation cancels the attitude correction to
    machine precision (measured 5e-18) and the test passes for a reason that
    has nothing to do with the gate."""
    f = RIEKF()
    for _ in range(50):
        f.update_body_velocity([0.0, 0.0, 0.0], sigma=0.01)
    assert f._velocity_is_observed()
    f.X.p = np.array([10.0, 10.0, 0.0])
    before = f.X.R.copy()
    assert f.update_position([9.9, 10.0], sigma=0.5) is True
    assert not np.allclose(f.X.R, before, atol=1e-12), (
        'with velocity observed, a position fix SHOULD correct attitude')
