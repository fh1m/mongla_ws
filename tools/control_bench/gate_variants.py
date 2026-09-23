"""Three candidate fixes for B-56, measured against each other on the bench.

⛔ NONE OF THESE IS IN THE FILTER YET. They are subclasses so the bench can rank
them before anything ships, which is the whole point of having a plant.

    A  OBSERVABILITY GATE -- refuse a world-frame update while velocity is
       unobserved. Measured separation is 1880x (sigma_vel 0.012 m/s with flow
       against 22.8 without), so the signal is not marginal.

    B  DROP THE ATTITUDE BLOCK -- keep the update, but zero `H[:, 0:3]`, so a
       world-frame measurement corrects position and NOTHING ELSE. Throws away
       a real coupling; the question is whether the coupling was worth more
       than the damage it does.

    C  INFLATE R WITH |p| -- keep everything, but acknowledge that the
       linearisation error grows with distance from the origin by trusting the
       measurement less out there.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'mongla_localization'))
from mongla_localization.inekf import RIEKF, skew          # noqa: E402

# With flow the filter holds sigma_vel at 0.012 m/s; without it, it passes 2 m/s
# within one second. Anything in 0.1 .. 1.0 separates them cleanly. 0.5 m/s is
# chosen as a PHYSICAL statement -- half the hull's cruise speed of uncertainty
# means the velocity is not known at all -- rather than as a fitted number.
VEL_SIGMA_GATE = 0.5


class GateOnObservability(RIEKF):
    """A -- refuse world-frame updates while velocity is unobserved."""

    def __init__(self, *a, gate=VEL_SIGMA_GATE, **k):
        super().__init__(*a, **k)
        self.gate = gate
        self.gated = 0

    def _velocity_is_observed(self) -> bool:
        return math.sqrt(self.P[3, 3] + self.P[4, 4] + self.P[5, 5]) < self.gate

    def update_depth(self, depth_m, sigma=0.02):
        if not self._velocity_is_observed():
            self.gated += 1
            return False
        return super().update_depth(depth_m, sigma)

    def update_position(self, xy, sigma=0.5):
        if not self._velocity_is_observed():
            self.gated += 1
            return False
        return super().update_position(xy, sigma)

    def update_yaw(self, yaw_deg, sigma_deg=2.0):
        if not self._velocity_is_observed():
            self.gated += 1
            return False
        return super().update_yaw(yaw_deg, sigma_deg)


class DropAttitudeBlock(RIEKF):
    """B -- a world-frame measurement corrects position and nothing else."""

    def update_depth(self, depth_m, sigma=0.02):
        H = np.zeros((1, self.DIM))
        H[0, 8] = 1.0                      # NO -skew(p) term
        y = np.array([-float(depth_m) - self.X.p[2]])
        return self._apply(H, y, np.array([[sigma ** 2]]))

    def update_position(self, xy, sigma=0.5):
        z = np.asarray(xy, dtype=float).reshape(2)
        H = np.zeros((2, self.DIM))
        H[0, 6] = 1.0
        H[1, 7] = 1.0                      # NO -skew(p) term
        return self._apply(H, z - self.X.p[:2], np.eye(2) * (sigma ** 2))


class InflateWithDistance(RIEKF):
    """C -- trust a world-frame measurement less the further out we believe we are."""

    def update_depth(self, depth_m, sigma=0.02):
        s = sigma * (1.0 + float(np.linalg.norm(self.X.p)))
        return super().update_depth(depth_m, s)

    def update_position(self, xy, sigma=0.5):
        s = sigma * (1.0 + float(np.linalg.norm(self.X.p)))
        return super().update_position(xy, s)


VARIANTS = {
    'baseline (as shipped)': RIEKF,
    'A  observability gate': GateOnObservability,
    'B  drop attitude block': DropAttitudeBlock,
    'C  inflate R with |p|': InflateWithDistance,
}


class Hybrid(RIEKF):
    """⭐ D -- A's TRIGGER with B's ACTION, and the position fix let through.

    The ranking made the tension explicit. Gating the whole update (A) is the
    only thing that helps when no fix is available, because it stops depth and
    yaw injecting attitude. Dropping the attitude block (B) is the only thing
    that helps when a fix IS available, because it lets the fix pin position
    without the coupling. Each is catastrophic in the other's regime.

    So: while velocity is unobserved,
      * DEPTH and YAW are refused outright -- they inject attitude and buy
        nothing horizontal;
      * a POSITION FIX is still applied, but with `H[:, 0:3]` zeroed, so it
        corrects position and cannot rotate the state.
    Once velocity is observed again, everything reverts to the shipped
    behaviour exactly.
    """

    def __init__(self, *a, gate=VEL_SIGMA_GATE, **k):
        super().__init__(*a, **k)
        self.gate = gate
        self.gated = 0

    def _velocity_is_observed(self) -> bool:
        return math.sqrt(self.P[3, 3] + self.P[4, 4] + self.P[5, 5]) < self.gate

    def update_depth(self, depth_m, sigma=0.02):
        if not self._velocity_is_observed():
            self.gated += 1
            return False
        return super().update_depth(depth_m, sigma)

    def update_yaw(self, yaw_deg, sigma_deg=2.0):
        if not self._velocity_is_observed():
            self.gated += 1
            return False
        return super().update_yaw(yaw_deg, sigma_deg)

    def update_position(self, xy, sigma=0.5):
        if self._velocity_is_observed():
            return super().update_position(xy, sigma)
        # Unobserved: pin position, touch nothing else.
        z = np.asarray(xy, dtype=float).reshape(2)
        H = np.zeros((2, self.DIM))
        H[0, 6] = 1.0
        H[1, 7] = 1.0
        return self._apply(H, z - self.X.p[:2], np.eye(2) * (sigma ** 2))


VARIANTS['D  hybrid'] = Hybrid


class StalenessGate(RIEKF):
    """⭐ E -- gate on TIME SINCE the last velocity update, not on covariance.

    ⛔ WHY THE COVARIANCE THRESHOLD WAS THE WRONG SIGNAL. It has to sit below a
    freshly constructed filter's own sigma_vel (sqrt(3) * P0_velocity = 0.866)
    or the gate fires on a cold start and refuses everything. But it also has to
    fire within a fraction of a second of losing velocity, because that is how
    fast the damage lands -- a gate at 1.5 already lets through enough to reach
    377 m. That leaves a usable band of 0.87 .. 1.0, a 15 % margin, which is not
    a thing to ship.

    Staleness has no such conflict. A cold filter is not stale, so existing
    behaviour is untouched; a filter that has lost flow is stale within one
    timeout regardless of what its covariance happens to be; and the threshold
    is a physical quantity an operator can reason about rather than a tuned one.

    ⚠ The action is still D's: depth and yaw are REFUSED while stale, because
    they inject attitude and buy nothing horizontal; a position fix is still
    APPLIED, with `H[:, 0:3]` zeroed so it pins position without rotating the
    state.
    """

    def __init__(self, *a, vel_timeout_s: float = 0.5, **k):
        super().__init__(*a, **k)
        self.vel_timeout_s = vel_timeout_s
        self._vel_stale_s = 0.0
        self.gated = 0

    def predict(self, gyro, accel, dt):
        super().predict(gyro, accel, dt)
        if dt > 0:
            self._vel_stale_s += float(dt)

    def update_body_velocity(self, v_body, sigma=0.05):
        ok = super().update_body_velocity(v_body, sigma)
        if ok:
            self._vel_stale_s = 0.0
        return ok

    def update_body_velocity_xy(self, vx, vy, sigma=0.05):
        ok = super().update_body_velocity_xy(vx, vy, sigma)
        if ok:
            self._vel_stale_s = 0.0
        return ok

    def update_zero_velocity(self, sigma=0.02):
        ok = super().update_zero_velocity(sigma)
        if ok:
            self._vel_stale_s = 0.0
        return ok

    def _stale(self) -> bool:
        return self._vel_stale_s > self.vel_timeout_s

    def update_depth(self, depth_m, sigma=0.02):
        if self._stale():
            self.gated += 1
            return False
        return super().update_depth(depth_m, sigma)

    def update_yaw(self, yaw_deg, sigma_deg=2.0):
        if self._stale():
            self.gated += 1
            return False
        return super().update_yaw(yaw_deg, sigma_deg)

    def update_position(self, xy, sigma=0.5):
        if not self._stale():
            return super().update_position(xy, sigma)
        z = np.asarray(xy, dtype=float).reshape(2)
        H = np.zeros((2, self.DIM))
        H[0, 6] = 1.0
        H[1, 7] = 1.0
        return self._apply(H, z - self.X.p[:2], np.eye(2) * (sigma ** 2))


VARIANTS['E  staleness gate'] = StalenessGate
