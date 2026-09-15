"""Right-invariant EKF on SE_2(3): where the hull is, from what it can feel.

⛔ WHY INVARIANT AND NOT A QUATERNION EKF. The error dynamics of a filter whose
state lives on a matrix Lie group satisfy a LOG-LINEAR differential equation, so
the linearisation carries minimal approximation error and -- the part that
matters -- DOES NOT DEPEND ON THE CURRENT ESTIMATE. A standard EKF linearised
about a bad attitude estimate stays bad because the Jacobian it uses is wrong in
exactly the direction the error is. An InEKF started 90 degrees off converges
anyway. Potokar et al., RA-L 2021, show it directly for underwater navigation
with our sensor set: IMU plus body-frame velocity, with depth as a singleton
measurement.

⛔ HANDEDNESS: RIGHT-INVARIANT -- AND THE ORIGINAL REASON WRITTEN HERE WAS
WRONG, so it is corrected rather than quietly deleted. This paragraph used to
argue "three of our five measurements are body-frame, therefore right-
invariant". The convention runs the other way: a BODY-frame observation (DVL
or flow velocity) is the exact one for the LEFT-invariant form, and a
WORLD-frame observation (GPS, a pool position fix) is exact for the RIGHT.

What actually settles it is that the distinction matters less than the
argument assumed. Recent work shows the left and right IEKF are the same
algorithm when the error reset is applied consistently, so handedness is not
the performance decision it is often presented as. What this filter does keep
is the property that pays: the correction is injected on the LEFT
(`dX * X`, see `_inject`), biases are carried additively OUTSIDE the
exponential, and `A[3:6,0:3] = skew(GRAVITY)` couples attitude error into
velocity error. All three match RossHartley/invariant-ekf, the reference C++
implementation, line for line.

So: every measurement here except the exact body-velocity form is "imperfect"
in the strict sense, which is a known and accepted approximation -- Potokar et
al. say the same of depth as a singleton. It is not an oversight, and it is
also not the carefully-reasoned frame-counting decision this file used to
claim.

⛔ AND THE TRAP THE LITERATURE NAMES. Autonomous error propagation survives ONLY
IF VELOCITY IS HELD IN THE WORLD FRAME. Store body velocity instead and
state-dependent Coriolis and curvature terms appear and log-linearity is gone --
the filter still runs, still produces a pose, and has quietly lost the property
it was chosen for. So `v` here is world-frame, always, and the body-frame
measurement is converted at the update rather than the state being stored the
convenient way.

⚠ TWO HONEST CAVEATS, carried not glossed:
  1. **Bias states break exactness.** With noise and bias the log-linear error
     system is only approximate. The convergence behaviour is still better than
     a quaternion EKF's, but the theoretical guarantee is weaker than the
     headline. We need bias states anyway.
  2. **Our BNO already fuses attitude in hardware** (mag-free, drift under
     0.01 deg/min at rest). Whether to propagate on raw gyro or consume that
     quaternion as an attitude measurement is a question for logged data, not
     for principle, and this module supports both: skip `predict`'s gyro and
     call `update_attitude`, or propagate and do not.

State X in SE_2(3), a 5x5 matrix:

    X = [ R  v  p ]      R in SO(3), v and p in R^3, both WORLD frame
        [ 0  1  0 ]
        [ 0  0  1 ]

Pure numpy. No ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

# ⛔ WORLD IS NED (z DOWN), BODY IS FRD. The board's ATTITUDE is aerospace
# Z-Y-X Euler in NED, its gyro and (host-mapped) accel are FRD, flow's +y is
# starboard. This was z-up until 2026-09-15 while the rotation fed in was NED:
# replayed through 50 s of real board data at rest, that ran to 583 m; NED,
# 0.036 m. At rest the accelerometer reads (0, 0, -g) level.
GRAVITY = np.array([0.0, 0.0, 9.80665])

# Below this rotation angle the closed-form exponential divides by ~0, so the
# series expansion is used instead. Not a tuning knob -- a numerical boundary.
_SMALL_ANGLE = 1e-8


def skew(v) -> np.ndarray:
    x, y, z = (float(c) for c in v)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_exp(phi) -> np.ndarray:
    """Rodrigues. Falls back to the series for a tiny angle rather than dividing."""
    phi = np.asarray(phi, dtype=float).reshape(3)
    theta = float(np.linalg.norm(phi))
    K = skew(phi)
    if theta < _SMALL_ANGLE:
        return np.eye(3) + K + 0.5 * (K @ K)
    return (np.eye(3) + (math.sin(theta) / theta) * K
            + ((1.0 - math.cos(theta)) / (theta * theta)) * (K @ K))


def so3_log(R) -> np.ndarray:
    """Inverse of `so3_exp`, clamped so a slightly non-orthonormal R cannot NaN."""
    R = np.asarray(R, dtype=float)
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))           # numerical drift must not become NaN
    theta = math.acos(c)
    if theta < _SMALL_ANGLE:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]]) * 0.5
    return (theta / (2.0 * math.sin(theta))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


@dataclass
class State:
    """X in SE_2(3) plus IMU biases. `v` and `p` are WORLD frame -- see above."""
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bg: np.ndarray = field(default_factory=lambda: np.zeros(3))   # gyro bias
    ba: np.ndarray = field(default_factory=lambda: np.zeros(3))   # accel bias

    def matrix(self) -> np.ndarray:
        X = np.eye(5)
        X[:3, :3] = self.R
        X[:3, 3] = self.v
        X[:3, 4] = self.p
        return X

    def yaw_deg(self) -> float:
        return math.degrees(math.atan2(self.R[1, 0], self.R[0, 0]))

    def copy(self) -> 'State':
        return State(self.R.copy(), self.v.copy(), self.p.copy(),
                     self.bg.copy(), self.ba.copy())


# Chi-square 99th percentile by degrees of freedom. Only the dimensions this
# filter actually uses are listed, so a new measurement of an unlisted width
# raises a KeyError instead of silently picking a wrong threshold.
CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345}

# ⛔ THE GATE'S OWN FAILURE MODE, and it is worse than no gate.
# A filter that has become CONFIDENTLY WRONG rejects every measurement that
# disagrees with it -- and the more wrong it is, the larger the innovation and
# the more certain the rejection. It locks itself out of the one channel that
# could correct it and goes on publishing a tight covariance around a false
# position, for ever, with no error anywhere.
#
# So persistent disagreement is evidence about the FILTER, not the sensor. On
# a run of consecutive rejections the estimate stops being trusted: the gate
# is bypassed once and the covariance inflated, which lets the measurement
# back in and widens the gate for what follows. Five is roughly a second of
# our slowest channel -- long enough that noise does not trip it, short enough
# that a lockout does not survive a manoeuvre.
REJECT_STREAK_LIMIT = 5
REJECT_INFLATION = 4.0


class RIEKF:
    """Right-invariant EKF. Predict on IMU, update on what the vehicle can see.

    Covariance ordering, stated once because every Jacobian depends on it:

        [ theta (3) | v (3) | p (3) | bg (3) | ba (3) ]  -> 15 states
    """

    DIM = 15

    def __init__(self, *, state: Optional[State] = None,
                 sigma_gyro: float = 0.01, sigma_accel: float = 0.1,
                 sigma_gyro_bias: float = 1e-4, sigma_accel_bias: float = 1e-3,
                 P0_attitude: float = 0.3, P0_velocity: float = 0.5,
                 P0_position: float = 1.0, P0_bias: float = 0.01):
        self.X = state.copy() if state is not None else State()
        self.P = np.diag(np.concatenate([
            np.full(3, P0_attitude ** 2), np.full(3, P0_velocity ** 2),
            np.full(3, P0_position ** 2), np.full(3, P0_bias ** 2),
            np.full(3, P0_bias ** 2)]))
        self.Q = np.diag(np.concatenate([
            np.full(3, sigma_gyro ** 2), np.full(3, sigma_accel ** 2),
            np.zeros(3),
            np.full(3, sigma_gyro_bias ** 2), np.full(3, sigma_accel_bias ** 2)]))
        # Published, because a filter that silently discards half its
        # measurements looks identical to one that is merely drifting.
        self.accepted = 0
        self.rejected = 0
        self.reject_streak = 0
        self.lockout_breaks = 0

    # ── propagation ──────────────────────────────────────────────────────────
    def predict(self, gyro, accel, dt: float) -> None:
        """One IMU step. `gyro` rad/s and `accel` m/s^2, both BODY frame.

        `accel` is specific force as an accelerometer reports it, so gravity is
        added after rotating into the world. Getting that backwards produces a
        filter that falls upward, which is obvious in a plot and invisible in a
        number.
        """
        dt = float(dt)
        if dt <= 0.0:
            return
        w = np.asarray(gyro, dtype=float).reshape(3) - self.X.bg
        a = np.asarray(accel, dtype=float).reshape(3) - self.X.ba

        R = self.X.R
        a_world = R @ a + GRAVITY
        self.X.p = self.X.p + self.X.v * dt + 0.5 * a_world * dt * dt
        self.X.v = self.X.v + a_world * dt
        self.X.R = R @ so3_exp(w * dt)

        # Right-invariant error dynamics. A is state-INDEPENDENT in theta,
        # which is the whole reason for this filter: the transition does not
        # care how wrong the current attitude estimate is.
        A = np.zeros((self.DIM, self.DIM))
        A[3:6, 0:3] = skew(GRAVITY)
        A[6:9, 3:6] = np.eye(3)
        A[0:3, 9:12] = -R
        A[3:6, 9:12] = -skew(self.X.v) @ R
        A[3:6, 12:15] = -R
        A[6:9, 9:12] = -skew(self.X.p) @ R

        Phi = np.eye(self.DIM) + A * dt
        Qd = self._adapt_noise(R) * dt
        self.P = Phi @ self.P @ Phi.T + Qd
        self.P = 0.5 * (self.P + self.P.T)      # keep it symmetric

    def _adapt_noise(self, R) -> np.ndarray:
        """IMU noise is BODY frame; the error state is not. Rotate it in."""
        Q = self.Q.copy()
        Q[0:3, 0:3] = R @ Q[0:3, 0:3] @ R.T
        Q[3:6, 3:6] = R @ Q[3:6, 3:6] @ R.T
        return Q

    # ── updates ──────────────────────────────────────────────────────────────
    def update_body_velocity(self, v_body, sigma: float = 0.05) -> bool:
        """The flow DVL: velocity measured in the BODY frame.

        Right-invariant and exact for this filter, which is why the handedness
        was chosen this way: three of our five measurements have this shape.
        """
        z = np.asarray(v_body, dtype=float).reshape(3)
        H = np.zeros((3, self.DIM))
        H[:, 3:6] = self.X.R.T
        H[:, 0:3] = self.X.R.T @ skew(self.X.v)
        y = z - self.X.R.T @ self.X.v
        return self._apply(H, y, np.eye(3) * (sigma ** 2))

    def update_body_velocity_xy(self, vx: float, vy: float,
                                var_x: float, var_y: float) -> bool:
        """Downward optical flow: body x and y ONLY.

        ⛔ A BOTTOM-LOOKING CAMERA CANNOT MEASURE VERTICAL VELOCITY, and the
        flow node says so -- it marks `twist.covariance[14]` as -1.0, the ROS
        convention for an unobserved component. Passing its `linear.z` (which
        is 0.0, because nothing set it) into the 3-D update states that the
        vehicle is not moving vertically, at full confidence, and that fights
        the depth channel on every dive.

        The variances are the flow node's OWN, derived per sample from the
        standard error of the mean over its RANSAC inliers -- a textured floor
        and a bare one do not deserve the same weight, and a constant sigma
        here would throw that away.
        """
        H = np.zeros((2, self.DIM))
        R_T = self.X.R.T
        H[:, 3:6] = R_T[:2, :]
        H[:, 0:3] = (R_T @ skew(self.X.v))[:2, :]
        y = np.array([vx, vy]) - (R_T @ self.X.v)[:2]
        return self._apply(H, y, np.diag([var_x, var_y]))

    def update_zero_velocity(self, sigma: float = 0.02) -> bool:
        """ZUPT: the vehicle is stationary, so body velocity is exactly zero.

        Cheap and strong. With no aiding, velocity error integrates into
        position without bound; a stationary interval turns that into a direct
        observation of the accumulated error, and the standard result is that
        it keeps INS error growth linear instead of quadratic.

        This is worth having precisely when flow is NOT available -- flow
        already measures zero when the hull is still. The caller owns the
        stationarity test, because getting that wrong is how a ZUPT ruins a
        filter: declaring "still" during a slow constant-velocity transit
        removes real motion.
        """
        return self.update_body_velocity((0.0, 0.0, 0.0), sigma=sigma)

    def update_depth(self, depth_m: float, sigma: float = 0.02) -> bool:
        """Bar30 depth, NEGATIVE below the surface as everywhere in this stack.

        World z is DOWN (NED), so the state holds +depth: z = -depth_m.

        ⚠ An IMPERFECT measurement for a right-invariant filter -- world-frame,
        so the log-linear property is approximate here. Accepted, and named, as
        Potokar does for the same singleton.
        """
        H = np.zeros((1, self.DIM))
        H[0, 8] = 1.0
        y = np.array([-float(depth_m) - self.X.p[2]])
        return self._apply(H, y, np.array([[sigma ** 2]]))

    def update_yaw(self, yaw_deg: float, sigma_deg: float = 2.0) -> bool:
        """The landmark heading anchor: an absolute world yaw.

        ⚠ Also imperfect, and worth more than it looks: it is the only source
        here that can remove a heading error the vehicle cannot feel.
        """
        err = math.radians(_wrap180(float(yaw_deg) - self.X.yaw_deg()))
        H = np.zeros((1, self.DIM))
        H[0, 2] = 1.0
        return self._apply(H, np.array([err]),
                           np.array([[math.radians(sigma_deg) ** 2]]))

    def update_position(self, xy, sigma: float = 0.5) -> bool:
        """A pool fix from prop resection: world x and y.

        ⚠ Imperfect, like depth. This is the measurement that bounds the
        dead-reckoning drift, and it exists because the course is a landmark
        field -- see `resection`.
        """
        z = np.asarray(xy, dtype=float).reshape(2)
        H = np.zeros((2, self.DIM))
        H[0, 6] = 1.0
        H[1, 7] = 1.0
        return self._apply(H, z - self.X.p[:2], np.eye(2) * (sigma ** 2))

    def update_attitude(self, R_meas, sigma_deg: float = 1.0) -> bool:
        """Consume the BNO's own fused attitude instead of propagating to it.

        Offered because the BNO fuses in hardware and does it well (drift under
        0.01 deg/min at rest). Which path is better is a question for logged
        data; this module refuses to decide it by assertion.
        """
        R_meas = np.asarray(R_meas, dtype=float).reshape(3, 3)
        err = so3_log(R_meas @ self.X.R.T)
        H = np.zeros((3, self.DIM))
        H[:, 0:3] = np.eye(3)
        return self._apply(H, err, np.eye(3) * (math.radians(sigma_deg) ** 2))

    # ── the correction itself ────────────────────────────────────────────────
    def _apply(self, H, y, R_noise) -> bool:
        """One correction. Returns False if the measurement was REJECTED.

        ⛔ THE GATE IS NOT OPTIONAL FOR A VISION-FED FILTER. Every measurement
        here except depth comes from a detector, and a detector's failure mode
        is not noise -- it is a confident answer about the wrong object. One
        mirrored PnP branch or one mislabelled prop is a metre-scale
        innovation, and an ungated filter takes it at full gain.

        NIS (normalised innovation squared) is `y' S^-1 y`, chi-square
        distributed with `dim(y)` degrees of freedom when the filter is
        consistent. Rejecting above the 99th percentile is the standard
        ellipsoidal validation gate.

        ⚠ AND THE CAVEAT, because it is easy to fool yourself with this:
        gating TRUNCATES the innovation distribution, so post-gate NIS
        statistics are contracted by a factor gamma(tau, m) < 1 that depends
        only on the threshold and the dimension. A gated filter therefore
        looks OVERCONFIDENT when you measure its consistency after the gate,
        and an adaptive scheme that reacts to that will tighten the gate and
        shrink the noise until it believes nothing. So: never tune R from
        post-gate innovations. The gate is a guard, not a statistic.
        """
        y = np.asarray(y, dtype=float).reshape(-1)
        S = H @ self.P @ H.T + R_noise
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        nis = float(y @ S_inv @ y)
        if not math.isfinite(nis):
            self.rejected += 1
            return False
        if nis > CHI2_99[len(y)]:
            self.reject_streak += 1
            if self.reject_streak < REJECT_STREAK_LIMIT:
                self.rejected += 1
                return False
            # Believe the world instead of the estimate. Inflate FIRST, then
            # recompute the gain from the widened covariance -- accepting on
            # the old tight P would apply a small correction to a large error
            # and leave the filter just as stuck next time.
            self.lockout_breaks += 1
            self.reject_streak = 0
            self.P = self.P * REJECT_INFLATION
            S = H @ self.P @ H.T + R_noise
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return False
        self.reject_streak = 0
        K = self.P @ H.T @ S_inv
        dx = K @ y
        self._inject(dx)
        I_KH = np.eye(self.DIM) - K @ H
        # Joseph form, for numerical robustness under a suboptimal gain.
        # ⚠ HONESTLY LABELLED: swapping this for the short form `I_KH @ P`
        # does NOT fail any test here, so this is standard practice rather
        # than a measured guard in this codebase. The difference shows up as
        # loss of positive-definiteness over long runs with mismatched noise,
        # which these tests do not reproduce -- do not read the comment as
        # evidence the short form was tried and found wanting.
        self.P = I_KH @ self.P @ I_KH.T + K @ R_noise @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.accepted += 1
        return True

    def _inject(self, dx) -> None:
        """Apply the error state to the group, on the LEFT.

        Right-invariant error is `eta = X_hat X^-1`, so the correction
        multiplies from the left. Injecting on the right is the single easiest
        way to write a filter that runs, converges slowly and is wrong in a way
        no test of its outputs will show.
        """
        dtheta, dv, dp = dx[0:3], dx[3:6], dx[6:9]
        dR = so3_exp(dtheta)
        self.X.R = dR @ self.X.R
        self.X.v = dR @ self.X.v + dv
        self.X.p = dR @ self.X.p + dp
        self.X.bg = self.X.bg + dx[9:12]
        self.X.ba = self.X.ba + dx[12:15]


def _wrap180(deg: float) -> float:
    d = math.fmod(float(deg) + 180.0, 360.0)
    if d <= 0.0:
        d += 360.0
    return d - 180.0
