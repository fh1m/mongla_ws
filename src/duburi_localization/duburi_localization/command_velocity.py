"""Velocity from what we COMMANDED, learned while the floor was readable.

⛔ THE HOLE THIS FILLS. Every velocity input the filter has comes from the
downward camera, and the floor instruments this round added (tile grating,
lane line) need texture too. On a bare, untextured or turbid floor flow
refuses, and the filter is left integrating an accelerometer -- quadratic
growth, metres in tens of seconds. The RPM thrust model (duburi_manager
/estimator) was meant to cover this
and is blocked: every ESC RPM ever recorded on this wire is exactly 0.

Model-aided INS (Hegrenaes & Hallingstad, IEEE JOE 2011; flown on HUGIN)
needs no RPM. It uses the COMMANDED actuation through a vehicle model, and it
estimates the model's error rather than trusting a constant. DeepVL (ICRA
2025) takes the same input. This is the smallest honest form of that idea:

    v_ss = g * u + b        per body axis, u = demand in [-1, 1]
    v    = first-order lag of v_ss with time constant tau

LEARNED, NEVER ASSUMED. `g` and `b` are fitted by recursive least squares
against flow velocity WHILE FLOW IS HEALTHY -- the easy regime calibrates the
hard one. Until an axis has enough excited samples and a small enough
residual it predicts NOTHING. A plausible default gain would be the
`pool_depth_m = 4.0` hazard: a number nobody measured, scaling every
position downstream.

WHY LINEAR IN u. On srot the demand maps linearly to DShot; thrust goes as
RPM^2 and quadratic drag as v^2, so steady speed goes roughly as RPM, i.e. as
u. The residual gate decides whether that holds on the day -- if it does
not, the axis never becomes ready.

WHAT `b` ABSORBS, AND WHAT IT CANNOT. A steady current biases water-relative
speed against the floor-relative speed flow measures; `b` soaks up its body-
frame component. The body frame rotates with the hull and the current does
not, so after a turn `b` is wrong until relearned. Forgetting (`lam`) bounds
how long, and the sigma handed on is the fitted residual, not a guess.

Only MANUAL_CONTROL demand is modelled. An on-board SROT_MOVE primitive ramps
its own demand and reports none, so the host does not know it -- and a model
fed an unknown input must predict nothing, not zero.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

# An axis must be MOVED to learn: near-zero demand says nothing about the gain
# and everything about noise.
MIN_EXCITATION = 0.05
MIN_SAMPLES = 100          # excited samples per axis before it may predict
MAX_RESIDUAL_MPS = 0.10    # predictive RMS above this: the model does not fit
SIGMA_FLOOR_MPS = 0.05     # never claim better than the flow it learned from
UNREADY_VAR = 1.0e4        # variance handed on for an unready axis


class _Axis:
    """RLS with exponential forgetting on theta = [g, b]."""

    def __init__(self, lam: float):
        self.lam = float(lam)
        self.theta = np.zeros(2)
        self.P = np.eye(2) * 100.0
        self.n = 0
        self.sq = 0.0          # EMA of squared a-priori residual

    def learn(self, u: float, v: float) -> None:
        if not (math.isfinite(u) and math.isfinite(v)) or abs(u) < MIN_EXCITATION:
            return
        phi = np.array([u, 1.0])
        resid = v - float(phi @ self.theta)
        Pphi = self.P @ phi
        k = Pphi / (self.lam + float(phi @ Pphi))
        self.theta = self.theta + k * resid
        self.P = (self.P - np.outer(k, Pphi)) / self.lam
        self.n += 1
        # Residual BEFORE the update: predictive. The post-update residual
        # flatters a model that merely memorises the last sample.
        a = 0.05 if self.n > 20 else 1.0 / self.n
        self.sq += a * (resid * resid - self.sq)

    @property
    def rms(self) -> float:
        return math.sqrt(max(self.sq, 0.0))

    @property
    def ready(self) -> bool:
        return self.n >= MIN_SAMPLES and self.rms <= MAX_RESIDUAL_MPS

    def v_ss(self, u: float) -> float:
        return float(self.theta[0] * u + self.theta[1])


class CommandVelocityModel:
    """Body-frame (x forward, y starboard) velocity from commanded demand."""

    def __init__(self, *, tau_s: float = 1.0, lam: float = 0.995):
        if not tau_s > 0.0:
            raise ValueError(f'tau_s must be positive, got {tau_s!r}')
        self.tau_s = float(tau_s)
        self.x = _Axis(lam)
        self.y = _Axis(lam)
        self._w = [0.0, 0.0]   # lagged demand
        self._known = False
        self._settle = 0.0

    def step(self, u_fwd: Optional[float], u_lat: Optional[float],
             dt: float) -> None:
        """Advance the lag. None = demand UNKNOWN (a board primitive ran, or the
        stream went stale): what the hull did meanwhile is unknown, so forget."""
        if u_fwd is None or u_lat is None:
            self._known = False
            self._w = [0.0, 0.0]
            return
        dt = max(0.0, float(dt))
        if not self._known:
            # The hull is not at the lag's zero after an unknown stretch, so
            # neither learning nor prediction until one full tau has passed.
            self._known = True
            self._settle = self.tau_s
        a = min(1.0, dt / self.tau_s)
        self._w[0] += a * (float(u_fwd) - self._w[0])
        self._w[1] += a * (float(u_lat) - self._w[1])
        self._settle = max(0.0, self._settle - dt)

    def learn(self, vx: float, vy: float) -> None:
        """One healthy flow sample against the current lagged demand."""
        if not self._known or self._settle > 0.0:
            return
        self.x.learn(self._w[0], vx)
        self.y.learn(self._w[1], vy)

    def predict(self) -> Optional[Tuple[float, float, float, float]]:
        """(vx, vy, var_x, var_y), or None when nothing honest can be said."""
        if not self._known or self._settle > 0.0:
            return None
        if not (self.x.ready or self.y.ready):
            return None
        out = []
        for axis, w in ((self.x, self._w[0]), (self.y, self._w[1])):
            if axis.ready:
                s = max(axis.rms, SIGMA_FLOOR_MPS)
                out.append((axis.v_ss(w), s * s))
            else:
                out.append((0.0, UNREADY_VAR))
        return out[0][0], out[1][0], out[0][1], out[1][1]
