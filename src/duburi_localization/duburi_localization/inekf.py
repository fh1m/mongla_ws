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

⛔ THE ONE DESIGN DECISION, TAKEN DELIBERATELY: RIGHT-INVARIANT. The invariance
must match the measurement model. Of our five sources, three are body-frame --
flow velocity, learned velocity, camera bearings -- and two are world-frame:
depth and the landmark heading anchor. Three of five body-frame means
right-invariant, and depth and heading are then "imperfect" measurements whose
log-linearity is approximate. That is a known, accepted approximation, not an
oversight.

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

GRAVITY = np.array([0.0, 0.0, -9.80665])

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
    def update_body_velocity(self, v_body, sigma: float = 0.05) -> None:
        """The flow DVL: velocity measured in the BODY frame.

        Right-invariant and exact for this filter, which is why the handedness
        was chosen this way: three of our five measurements have this shape.
        """
        z = np.asarray(v_body, dtype=float).reshape(3)
        H = np.zeros((3, self.DIM))
        H[:, 3:6] = self.X.R.T
        H[:, 0:3] = self.X.R.T @ skew(self.X.v)
        y = z - self.X.R.T @ self.X.v
        self._apply(H, y, np.eye(3) * (sigma ** 2))

    def update_depth(self, depth_m: float, sigma: float = 0.02) -> None:
        """Bar30 depth: world z, NEGATIVE below the surface in this stack.

        ⚠ An IMPERFECT measurement for a right-invariant filter -- world-frame,
        so the log-linear property is approximate here. Accepted, and named, as
        Potokar does for the same singleton.
        """
        H = np.zeros((1, self.DIM))
        H[0, 8] = 1.0
        y = np.array([float(depth_m) - self.X.p[2]])
        self._apply(H, y, np.array([[sigma ** 2]]))

    def update_yaw(self, yaw_deg: float, sigma_deg: float = 2.0) -> None:
        """The landmark heading anchor: an absolute world yaw.

        ⚠ Also imperfect, and worth more than it looks: it is the only source
        here that can remove a heading error the vehicle cannot feel.
        """
        err = math.radians(_wrap180(float(yaw_deg) - self.X.yaw_deg()))
        H = np.zeros((1, self.DIM))
        H[0, 2] = 1.0
        self._apply(H, np.array([err]),
                    np.array([[math.radians(sigma_deg) ** 2]]))

    def update_position(self, xy, sigma: float = 0.5) -> None:
        """A pool fix from prop resection: world x and y.

        ⚠ Imperfect, like depth. This is the measurement that bounds the
        dead-reckoning drift, and it exists because the course is a landmark
        field -- see `resection`.
        """
        z = np.asarray(xy, dtype=float).reshape(2)
        H = np.zeros((2, self.DIM))
        H[0, 6] = 1.0
        H[1, 7] = 1.0
        self._apply(H, z - self.X.p[:2], np.eye(2) * (sigma ** 2))

    def update_attitude(self, R_meas, sigma_deg: float = 1.0) -> None:
        """Consume the BNO's own fused attitude instead of propagating to it.

        Offered because the BNO fuses in hardware and does it well (drift under
        0.01 deg/min at rest). Which path is better is a question for logged
        data; this module refuses to decide it by assertion.
        """
        R_meas = np.asarray(R_meas, dtype=float).reshape(3, 3)
        err = so3_log(R_meas @ self.X.R.T)
        H = np.zeros((3, self.DIM))
        H[:, 0:3] = np.eye(3)
        self._apply(H, err, np.eye(3) * (math.radians(sigma_deg) ** 2))

    # ── the correction itself ────────────────────────────────────────────────
    def _apply(self, H, y, R_noise) -> None:
        S = H @ self.P @ H.T + R_noise
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ np.asarray(y, dtype=float).reshape(-1)
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
