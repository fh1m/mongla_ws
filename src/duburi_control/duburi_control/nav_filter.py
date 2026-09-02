"""Alpha-beta filters for navigation signals. One core, three wrappers.

WHY ALPHA-BETA AND NOT AN EMA, once, here
------------------------------------------
Measured on this vehicle's own bearing stream (see `bearing.BearingFilter`),
against real detector noise plus a 30 deg/s slew:

    filter            jitter    reduction   lag
    raw                3.570           0%   +0.302 deg
    ema a=0.15         0.496          86%   +3.564      <- unusable
    ema a=0.30         0.897          75%   +1.627
    median 9           1.147          68%   +2.508
    alpha-beta .25/.02 0.772          78%   +0.231      <- chosen

An EMA trades smoothness for lag one for one, because it has no model of
motion. Alpha-beta carries a VELOCITY state, so on steady motion it
extrapolates through the sampling and pipeline delay and comes out with LESS
lag than the raw signal. Every navigation signal on this vehicle is a smoothly
moving physical quantity sampled with noise, which is exactly the case that
holds for.

THREE THINGS EVERY ONE OF THESE NEEDS, and each has bitten something
--------------------------------------------------------------------
1. A RESET GATE. A velocity state keeps extrapolating across a discontinuity,
   so a jump makes the filter fly past the new value. Measured on the bearing:
   a 12 deg step overshoots 1.51 deg and takes 21 frames to settle; treating a
   large residual as a NEW measurement removes it entirely.
2. GAP HANDLING. Extrapolating a velocity across a real dropout walks the
   estimate somewhere it was never measured, while reporting confidently.
3. HONEST ABSENCE. `update(None)` clears the state and returns None rather
   than holding the last value. What absence MEANS is the caller's decision --
   grace windows, coast, the uplink's send-nothing contract -- and a filter
   that invents continuity takes that decision away from it.

WHAT NOT TO FILTER
------------------
A range/standoff measure. Lag on the axis that closes distance is a collision,
and the forward axis is one-sided by design. `BearingFilter` deliberately
passes `size_x`/`size_y` through untouched.
"""
from __future__ import annotations

import math
from typing import Optional


class AlphaBeta:
    """Scalar alpha-beta tracker with a reset gate and gap handling.

    `alpha` weights the position correction, `beta` the velocity correction.
    0.25 / 0.02 is the pair measured on this vehicle; smaller is smoother and
    slower to respond.
    """

    def __init__(self, alpha: float = 0.25, beta: float = 0.02,
                 gate: float = math.inf, max_gap_s: float = 0.30):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.gate = float(gate)
        self.max_gap_s = float(max_gap_s)
        self.reset()

    def reset(self) -> None:
        self.x: Optional[float] = None
        self.v = 0.0
        self._t: Optional[float] = None
        self.resets = 0

    # Subclasses override these two to change the metric (see AngleAlphaBeta).
    def _diff(self, a: float, b: float) -> float:
        return a - b

    def _wrap(self, x: float) -> float:
        return x

    def update(self, z: Optional[float], now: float) -> Optional[float]:
        if z is None or not math.isfinite(z):
            self.reset()
            return None
        if self.x is None or self._t is None or (now - self._t) > self.max_gap_s:
            self.x, self.v, self._t = float(z), 0.0, now
            return self.x
        dt = max(1e-3, now - self._t)
        self._t = now
        pred = self._wrap(self.x + self.v * dt)
        resid = self._diff(z, pred)
        if abs(resid) > self.gate:
            self.x, self.v = float(z), 0.0
            self.resets += 1
        else:
            self.x = self._wrap(pred + self.alpha * resid)
            self.v += (self.beta / dt) * resid
        return self.x


class AngleAlphaBeta(AlphaBeta):
    """The same, on a circle. For headings, which wrap.

    The wrap is not cosmetic. A hull crossing north goes 359 -> 1, and a filter
    that subtracts naively sees a -358 degree error and slams the estimate the
    long way round -- or, with a gate, snaps every single time it crosses. It
    would work perfectly through 358 degrees of the compass and fail at the one
    heading a returning AUV is most likely to be on.

    Velocity stays a signed rate, so a steady turn through the wrap is tracked
    without a discontinuity.
    """

    def _diff(self, a: float, b: float) -> float:
        return (a - b + 180.0) % 360.0 - 180.0

    def _wrap(self, x: float) -> float:
        return x % 360.0


class DepthFilter(AlphaBeta):
    """Depth, in metres, negative below the surface.

    Tuned looser than the bearing (gate 0.5 m rather than 4 sigma of noise)
    because the Bar30's own jitter is small -- the board reports BARO_P2P around
    5.8 mbar, roughly 6 cm of water -- while a genuine depth STEP is what a
    calibration or a bad read looks like, and those must not be smoothed into
    the estimate.
    """

    def __init__(self):
        super().__init__(alpha=0.35, beta=0.05, gate=0.5, max_gap_s=0.5)


class HeadingFilter(AngleAlphaBeta):
    """Heading in degrees, 0-360.

    Gate 25 deg: comfortably above real yaw rates seen between samples at these
    rates (45 deg/s at 10 Hz is 4.5 deg) and below a genuine reference jump.
    """

    def __init__(self):
        super().__init__(alpha=0.35, beta=0.05, gate=25.0, max_gap_s=0.5)
