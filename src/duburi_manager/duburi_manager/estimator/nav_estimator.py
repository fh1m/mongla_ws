"""Horizontal navigation without a DVL: velocity OBSERVED, position dead-reckoned.

⛔ THE ONE DECISION THIS FILE IS BUILT ON: **we do not integrate the
accelerometer.** Measured on our own board, flat and still, over 250 samples:

    xacc sits at  -17 mg  =  0.167 m/s^2 of bias
      integrated  1 s  ->  0.17 m/s of phantom velocity,   0.08 m of position
      integrated 10 s  ->  1.67 m/s,                       8.34 m
      integrated 30 s  ->  5.00 m/s,                      75.05 m

A 20 kg hull moving at 0.65 m/s would be buried by its own accelerometer inside
five seconds. This is not a tuning problem; it is what a consumer MEMS bias
does when integrated twice, and it is why the literature is unanimous that
underwater velocity must be **observed** rather than propagated
(RD-VIO, Applied Ocean Research 2023; DeepVL, ICRA 2025).

So the inertial sensor is used for what it is good at -- ROTATION -- and the
velocity comes from optical flow over the floor.

WHAT IS OBSERVED, WHAT IS NOT. Stated up front because the difference is the
whole safety story:

    depth        MEASURED, absolute          Bar30, no drift
    attitude     MEASURED, absolute-ish      BNO085 fusion on the board
    velocity     MEASURED when there is      flow x range / focal length
                 bottom lock and texture
    position     DEAD-RECKONED               drifts without bound. Always.

**Horizontal position is not observable** with this sensor set -- that is a
property of the physics, not of the filter, and no amount of tuning changes it.
The filter's job is to make the drift as slow and as HONEST as possible: every
position carries a sigma that only ever grows between fixes, so a consumer can
tell a metre of confidence from a metre of hope.

WHY SCALE SURVIVES CONSTANT-VELOCITY CRUISE. Monocular scale is normally
unobservable without accelerometer excitation, and "travelling at constant
velocity is the most efficient trajectory for most robotics applications" --
exactly what an AUV transit is. Range-VIO (Delaune & Bayard, RA-L 2021, the
navigation approach flown on NASA's Ingenuity) showed that adding a RANGE
measurement removes scale from the observability nullspace, so scale is
recoverable under zero or constant acceleration. Our range is the altitude
above the floor, and that is what makes a cruising velocity estimate possible
at all here.

WHY THE PREDICTION USES THRUST, NOT ACCELERATION. Between flow updates the
state has to go somewhere. The eight signed ESC RPMs plus battery voltage are
the same input set DeepVL feeds its network, they are already on our wire at
5 Hz, and unlike the accelerometer their error does not integrate into a ramp.
A caller with no thrust model simply gets a decaying-velocity prediction with
honest growth in sigma, which is the correct answer for "I do not know".
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

# Chi-square 99 % gate for a 2-DOF innovation. A measurement further than this
# from the prediction is rejected rather than fused: the flow module already
# refuses what it cannot stand behind, and this catches what survives that.
CHI2_2DOF_99 = 9.21

# Velocity process noise (m/s per sqrt(s)) when NO thrust model is supplied:
# how fast an unmodelled acceleration is allowed to move the velocity between
# fixes. It sets the whole dead-reckoning budget, so here is what it buys --
# seconds of BLIND flight before position sigma passes each bound, measured
# from a converged filter (velocity already fixed to ~0.02 m/s):
#
#     q_vel     10 cm     25 cm     50 cm     100 cm
#     0.35       0.5 s     0.9 s     1.5 s      2.3 s   <- default
#     0.15       0.9       1.6       2.5        4.1
#     0.05       1.8       3.3       5.3        8.4
#
# 0.35 is deliberately pessimistic: a thrusting 20 kg hull can change velocity
# fast, and a filter that under-states that will report a confident position it
# has not earned. **It should be FITTED IN WATER from NIS** -- a bench cannot
# produce the accelerations that decide it, and lowering it without that
# measurement buys coast time by asserting the hull is calmer than it is.
DEFAULT_VEL_PROCESS = 0.35

# The hull cannot exceed this; used only to reject absurd fixes, never to clamp
# a plausible one.
MAX_SPEED_MS = 2.0


@dataclass
class NavState:
    """What the estimator believes, and how much of it is earned."""
    vx: float = 0.0            # body-frame forward velocity, m/s
    vy: float = 0.0            # body-frame right velocity, m/s
    px: float = 0.0            # local-frame east/x position, m  (DEAD RECKONED)
    py: float = 0.0            # local-frame north/y position, m (DEAD RECKONED)
    vel_sigma: float = 1.0     # 1-sigma horizontal speed uncertainty, m/s
    pos_sigma: float = 0.0     # 1-sigma horizontal position uncertainty, m
    since_fix_s: float = float('inf')   # since the last velocity MEASUREMENT
    stationary: bool = False
    n_fixes: int = 0
    n_rejected: int = 0

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def position_trustworthy(self, within_m: float) -> bool:
        """Is the dead-reckoned position good to `within_m`?

        THE QUESTION A MISSION SHOULD ASK, instead of reading `px`/`py` and
        hoping. Position drifts without bound, so the only honest test is
        against the filter's own growing sigma -- and a filter that has never
        had a fix answers False however small its numbers look.
        """
        return self.n_fixes > 0 and self.pos_sigma <= float(within_m)


class NavEstimator:
    """Body velocity + dead-reckoned horizontal position.

    Holds no ROS: it takes measurements and returns a state, so the node, a
    replay and a bench test all exercise one implementation.

    Frames: velocity is BODY (x forward, y right); position is LOCAL, rotated
    by the yaw the caller supplies. Yaw is taken as KNOWN rather than estimated
    -- the board's BNO085 fusion is better than anything this filter could
    produce from a 1 mg-quantised accelerometer, and estimating it here would
    add an unobservable direction for no gain.
    """

    def __init__(self, *, vel_process: float = DEFAULT_VEL_PROCESS,
                 chi2_gate: float = CHI2_2DOF_99):
        self._x = np.zeros(4)                       # vx, vy, px, py
        self._P = np.diag([1.0, 1.0, 0.0, 0.0])     # position starts EXACT: it
        #                                             defines the local origin
        self._q_vel = float(vel_process)
        self._gate = float(chi2_gate)
        self._t: Optional[float] = None
        self._since_fix = float('inf')
        self._n_fix = 0
        self._n_rej = 0
        self._stationary = False

    # -- prediction --------------------------------------------------------- #
    def predict(self, t: float, yaw_rad: float, *,
                accel_body: Optional[Tuple[float, float]] = None) -> None:
        """Advance to time `t`. `accel_body` is a THRUST-MODEL acceleration.

        Explicitly NOT the accelerometer -- see the module docstring. Passing
        None gives a constant-velocity prediction with growing uncertainty,
        which is the honest answer when the thrust model is unavailable.
        """
        if self._t is None:
            self._t = float(t)
            return
        dt = float(t) - self._t
        if dt <= 0.0:
            # Out-of-order or duplicate sample. Ignoring it is right: rolling
            # the state backwards on a late message is how a filter acquires a
            # velocity it never had.
            return
        self._t = float(t)
        self._since_fix += dt

        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        F = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0],
                      [c * dt, -s * dt, 1.0, 0.0],
                      [s * dt, c * dt, 0.0, 1.0]])
        self._x = F @ self._x
        if accel_body is not None:
            self._x[0] += float(accel_body[0]) * dt
            self._x[1] += float(accel_body[1]) * dt

        # Process noise. Velocity is a random walk; position inherits it
        # through the same dt.
        #
        # HONEST NOTE ON THIS TERM: the position rows of G are the exact
        # discretisation, and they contribute **0.06-0.19 %** of the position
        # sigma -- measured by deleting them. Position drift is dominated
        # almost entirely by velocity uncertainty propagating through F. They
        # are kept because they are the correct form and cost nothing, NOT
        # because they are load-bearing, and no test here can separate them:
        # an injected defect that zeroed them changed nothing any assertion
        # could see. Recorded so the next person does not go looking for a
        # guard that cannot exist.
        q = (self._q_vel ** 2) * dt
        G = np.array([[1.0, 0.0], [0.0, 1.0],
                      [c * dt, -s * dt], [s * dt, c * dt]])
        self._P = F @ self._P @ F.T + G @ (q * np.eye(2)) @ G.T

    # -- updates ------------------------------------------------------------ #
    def update_velocity(self, vx: float, vy: float, sigma: float) -> bool:
        """Fuse a measured BODY velocity. Returns False if the fix was rejected.

        `sigma` is the measurement's own 1-sigma, in m/s -- the flow module can
        derive it from height and the pixel noise floor, so a fix taken 3 m up
        is correctly trusted less than one taken at 0.5 m.
        """
        if not (math.isfinite(vx) and math.isfinite(vy) and sigma > 0.0):
            return False
        if math.hypot(vx, vy) > MAX_SPEED_MS:
            self._n_rej += 1
            return False
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        R = (sigma ** 2) * np.eye(2)
        y = np.array([vx, vy]) - H @ self._x
        S = H @ self._P @ H.T + R
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        # MAHALANOBIS GATE. A fix that disagrees with the prediction by more
        # than chance is far more likely to be a bad flow interval than a real
        # 2 m/s reversal, and one such fix poisons the velocity the position
        # then integrates.
        if float(y @ Sinv @ y) > self._gate:
            self._n_rej += 1
            return False
        K = self._P @ H.T @ Sinv
        self._x = self._x + K @ y
        # Joseph form: it stays symmetric positive-definite under rounding,
        # which the textbook (I-KH)P does not over a long mission.
        IKH = np.eye(4) - K @ H
        self._P = IKH @ self._P @ IKH.T + K @ R @ K.T
        self._since_fix = 0.0
        self._n_fix += 1
        return True

    def update_zero_velocity(self, sigma: float = 0.01) -> bool:
        """A ZUPT: assert the vehicle is not moving.

        ⛔ THE CALLER MUST ESTABLISH STATIONARITY INDEPENDENTLY. It is NOT
        enough that the flow module returned nothing -- a low-texture floor, a
        dark frame and a genuinely motionless hull are the same silence, and
        treating "I cannot see motion" as "there is no motion" injects a
        confident zero into a filter that will believe it. Use the inertial
        signal and the thrust command, which fail for different reasons than
        the camera does. `stationary_from_imu()` is the reference test.
        """
        return self.update_velocity(0.0, 0.0, sigma)

    def note_stationary(self, stationary: bool) -> None:
        self._stationary = bool(stationary)

    # -- output ------------------------------------------------------------- #
    def state(self) -> NavState:
        p = self._P
        return NavState(
            vx=float(self._x[0]), vy=float(self._x[1]),
            px=float(self._x[2]), py=float(self._x[3]),
            vel_sigma=float(math.sqrt(max(p[0, 0] + p[1, 1], 0.0))),
            pos_sigma=float(math.sqrt(max(p[2, 2] + p[3, 3], 0.0))),
            since_fix_s=self._since_fix, stationary=self._stationary,
            n_fixes=self._n_fix, n_rejected=self._n_rej)

    def reset_position(self) -> None:
        """Make here the origin. Zeroes position AND its covariance -- the
        origin is exact by definition, and carrying the old sigma across a
        deliberate re-origin would report drift that no longer exists."""
        self._x[2] = self._x[3] = 0.0
        self._P[2:, :] = 0.0
        self._P[:, 2:] = 0.0


def stationary_from_imu(gyro_rms: float, accel_rms: float, thrust_norm: float,
                        *, gyro_max: float = 0.02, accel_max: float = 0.05,
                        thrust_max: float = 0.02) -> bool:
    """Is the vehicle at rest, judged WITHOUT the camera?

    Defaults come from this board's own static noise, measured over 250
    samples: gyro sd 8-15 mrad/s (0.5-0.9 deg/s) and accel quantised at 1 mg.
    The gyro threshold sits just above that floor, so real motion trips it and
    sensor noise does not.

    All three must agree. Any one of them alone has a failure mode that looks
    exactly like stillness: a hull can be held motionless by opposing thrust
    (inertial quiet, thrust loud), and it can drift with the current under no
    thrust at all (thrust quiet, and genuinely moving).
    """
    return (gyro_rms < gyro_max and accel_rms < accel_max
            and thrust_norm < thrust_max)
