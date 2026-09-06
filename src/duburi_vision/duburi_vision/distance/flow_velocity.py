"""Optical flow -> BODY VELOCITY. The observation an estimator can actually use.

WHY VELOCITY AND NOT DISTANCE. `distance_estimation_node` accumulates metres
travelled, which is an INTEGRAL: its error grows without bound and it cannot be
fused as a measurement, because a filter needs a quantity with a stationary
error, not a running sum of one. Velocity is that quantity. This module
produces it; the accumulator stays where it is for the operator readout it was
written for.

⛔ THE GEOMETRY, AND THE ONE FACT THE WHOLE THING TURNS ON.

A camera looking at a plane a distance `h` away sees two kinds of image motion,
and they scale differently:

    TRANSLATION   dx_px = f * (v / h) * dt        <- depends on h
    ROTATION      dx_px = f * omega * dt          <- DOES NOT depend on h

That asymmetry is the entire design. It means:

  * rotation must be subtracted BEFORE scaling by `h`, never after, because
    scaling a rotation term by range is meaningless;
  * you can separate the two at all only because you measure `omega`
    independently -- a gyro is not an optional refinement here, it is the only
    thing that makes the translation term recoverable;
  * an error in `h` scales the ANSWER proportionally but leaves the rotation
    subtraction untouched. A 10 % height error is a 10 % velocity error and
    nothing worse, which is a well-behaved failure and worth knowing.

So:  v = h * (flow_px/dt - f*omega) / f

⛔ AND THE FAILURE MODE THAT DOES NOT ANNOUNCE ITSELF. When the vehicle rotates
much more than it translates, `flow_px/dt` and `f*omega` are two large numbers
whose DIFFERENCE is the small quantity we want. Ordinary noise on either then
dominates the result, and the answer stays plausible while meaning nothing --
same shape as the planar-pose flip. `rot_fraction` reports exactly this ratio,
and a consumer must gate on it rather than on the velocity looking reasonable.

WHAT THIS DOES NOT DO. It does not integrate, does not filter, and does not
guess `h`. A missing height is a refusal, not a default: a default height turns
an unknown scale into a confident wrong speed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# Above this share of the flow being rotational, the translation is a small
# difference of large numbers. Measured against the model in the tests: at 0.9
# a 1 px flow error already moves the velocity by more than the velocity.
ROT_FRACTION_MAX = 0.80

# Below this many pixels of NET flow over the interval, the measurement is
# indistinguishable from the matcher's own noise (~1.5 px on our footage).
MIN_NET_FLOW_PX = 0.5

# COHERENCE. Maximum ratio of per-point flow DISPERSION to flow MAGNITUDE.
#
# Over a flat surface at constant range, a translation moves every point by
# nearly the same vector -- that is what a translation IS in this geometry. So
# points that DISAGREE as much as they move are not observing a translation,
# whatever their median says.
#
# Measured on a camera 0.77 m above a floor, over 690 intervals:
#
#   669 quiet intervals   dispersion median 0.127 px
#    21 with large flow   dispersion median 7.712 px, and a
#                         dispersion/magnitude ratio of 1.03 median, 0.84 MIN
#
# ⛔ HONEST CAVEAT ON THAT RUN: the operator reported afterwards that the
# camera was knocked during it, so those 21 intervals are NOT established as
# pure noise -- some may be real motion. What the ratio does establish is that
# they were INCOHERENT: the points disagreed as much as they moved, which a
# translation over a flat floor cannot produce. The gate is justified on that
# geometry, not on the assumption that the camera was still.
#
# A subsequent genuinely static run -- 727 intervals, peak flow 0.069 px --
# reported ZERO velocities, which is the correct answer and the baseline that
# matters. Note it never exercised this gate: nothing came near the 0.5 px
# floor. The coherence gate is verified by construction and by the ratio
# separation, and it still owes a POSITIVE test on real motion.
MAX_DISPERSION_RATIO = 0.5


@dataclass
class FlowVelocity:
    """Body-frame velocity from one flow interval."""
    ok: bool
    vx: float = float('nan')        # m/s, + = body forward (image +y sense)
    vy: float = float('nan')        # m/s, + = body right
    rot_fraction: float = 0.0       # |rotational flow| / |total flow|
    dispersion_ratio: float = 0.0   # per-point disagreement / flow magnitude
    net_flow_px: float = 0.0        # what is left after de-rotation
    height_m: float = 0.0
    reason: str = ''

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy) if self.ok else float('nan')


def flow_velocity(dx_px: float, dy_px: float, dt: float, *,
                  f_px: float,
                  height_m: Optional[float],
                  pitch_rate: float = 0.0,
                  roll_rate: float = 0.0,
                  dispersion_px: Optional[float] = None,
                  rot_fraction_max: float = ROT_FRACTION_MAX,
                  min_net_flow_px: float = MIN_NET_FLOW_PX,
                  max_dispersion_ratio: float = MAX_DISPERSION_RATIO
                  ) -> FlowVelocity:
    """Body velocity from a de-rotated flow measurement.

    `dx_px`/`dy_px` are the TOTAL image shift over `dt`, `pitch_rate`/
    `roll_rate` the body rates over the same interval (rad/s), and `height_m`
    the distance to the plane being observed -- the altimeter's altitude for a
    downward camera, or a known-size target's range for a forward one. They are
    the same quantity in this equation.

    Refuses rather than returning a number it cannot stand behind.
    """
    if dt <= 0.0:
        return FlowVelocity(ok=False, reason='non-positive dt')
    if f_px <= 0.0:
        return FlowVelocity(ok=False, reason='no focal length (uncalibrated)')
    if height_m is None or height_m <= 0.0:
        # A default height would turn an unknown SCALE into a confident wrong
        # speed -- the error would be a clean multiplier, invisible in every
        # plot, and it would be integrated by whatever consumes this.
        return FlowVelocity(ok=False, reason='no height/range')

    # Rotation-induced shift over the SAME interval. Range-independent, which
    # is why it is removed here rather than after the scaling.
    rot_x = f_px * roll_rate * dt
    rot_y = f_px * pitch_rate * dt
    net_x, net_y = dx_px - rot_x, dy_px - rot_y

    total = math.hypot(dx_px, dy_px)
    rot = math.hypot(rot_x, rot_y)
    net = math.hypot(net_x, net_y)
    frac = (rot / total) if total > 1e-9 else 0.0

    if frac > rot_fraction_max:
        return FlowVelocity(ok=False, rot_fraction=frac, net_flow_px=net,
                            height_m=height_m,
                            reason=f'rotation-dominated ({frac:.2f} of the flow)')
    if net < min_net_flow_px:
        # Not "stationary" -- UNMEASURABLE. Reporting 0 m/s here would feed a
        # filter a confident zero, which is a measurement, not an absence.
        return FlowVelocity(ok=False, rot_fraction=frac, net_flow_px=net,
                            height_m=height_m,
                            reason=f'net flow {net:.2f}px below the noise floor')

    # COHERENCE, and it is the gate the magnitude floor cannot supply. See
    # MAX_DISPERSION_RATIO: on a static camera the spurious intervals were
    # large AND incoherent, which is the only thing distinguishing them.
    ratio = 0.0
    if dispersion_px is not None:
        ratio = float(dispersion_px) / max(net, 1e-9)
        if ratio > max_dispersion_ratio:
            return FlowVelocity(ok=False, rot_fraction=frac, net_flow_px=net,
                                dispersion_ratio=ratio, height_m=height_m,
                                reason=f'incoherent flow (dispersion '
                                       f'{ratio:.2f}x the motion)')

    scale = height_m / (f_px * dt)
    return FlowVelocity(ok=True, vx=net_y * scale, vy=net_x * scale,
                        rot_fraction=frac, net_flow_px=net,
                        dispersion_ratio=ratio, height_m=height_m)
