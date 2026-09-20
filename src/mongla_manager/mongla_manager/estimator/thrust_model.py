"""Eight signed ESC RPMs -> a body-frame acceleration for the estimator.

Its consumer is the RIEKF's velocity aiding in `mongla_localization` (the
private `NavEstimator` that once took it as `accel_body` is gone), and nothing
has ever supplied it. It is a PURE function of numbers already on our wire,
so all of it is testable on a bench with no thrusters attached -- which is the
only reason it exists this round.

⛔ NOTHING CALLS THIS YET, AND THAT IS DELIBERATE. Measured on this vehicle: 958
CRC-valid ESC_STATUS frames across two recorded sessions, every rpm exactly 0.
No non-zero RPM has ever crossed this wire. Wiring a thrust model to a signal
that has only ever been zero would add a confident zero to the estimator's
prediction, which is worse than adding nothing.

────────────────────────────────────────────────────────────────────────────
THE BIAS HAS A SIGN, AND IT IS THE FLATTERING ONE
────────────────────────────────────────────────────────────────────────────
`thrust ~ RPM^2` is a BOLLARD-PULL relation -- true at zero forward speed. The
thrust coefficient KT falls roughly linearly with advance ratio J = V/(nD), so
at cruise a propeller turning at a given RPM makes LESS thrust than this model
says. The error therefore:

    overestimates thrust -> overestimates acceleration -> the dead-reckoned
    position runs AHEAD of truth

i.e. it flatters us, at exactly the operating point transits live in. That is
the direction that does not announce itself. Until a pool run measures the
correction, a consumer must widen its velocity process noise to cover the bias
rather than treat this as a calibrated input.

The firmware's own claim -- thrust/RPM^2 invariant to ~3 % from 12 V to 20 V --
is a VOLTAGE invariance, not a SPEED one, and does not rescue this.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

# Mongla_others/srot-control-board/src/control/mixer.cpp:25, columns 4 (fwd)
# and 5 (lat). Motors 1-4 are the horizontal ring; 5-8 are vertical and
# contribute nothing to surge or sway, which is why they are absent here.
_FWD_SIGN = (-1.0, -1.0, +1.0, +1.0)
_LAT_SIGN = (+1.0, -1.0, +1.0, -1.0)

# ⛔ THE MIXER MATRIX IS A DEMAND MIX, NOT A GEOMETRY. Its entries are +/-1
# because it distributes a normalised demand; the thrusters themselves sit at
# 45 deg in the horizontal plane, so a real FORCE sum takes the cos(45) =
# 0.7071 component. Using the matrix's own +/-1 for a force sum overestimates
# surge and sway by sqrt(2) -- 41 % -- and looks entirely plausible.
# Same trap as "a mesh gives an axis LINE, not a push direction".
_HORIZ_COS = math.cos(math.radians(45.0))

# A T200 makes only ~75-79 % of its forward thrust in reverse (thrust_trim.h:23).
REVERSE_EFFICIENCY = 0.77


def body_accel_from_rpm(
    rpm: Optional[Sequence[Optional[int]]],
    *,
    k_n_per_rpm2: Optional[float],
    mass_kg: float,
) -> Optional[Tuple[float, float]]:
    """(a_fwd, a_lat) in m/s^2, or None when the answer is not known.

    `k_n_per_rpm2` is Newtons of forward thrust per RPM^2 for ONE thruster.

    ⛔ IT HAS NO DEFAULT ON PURPOSE. Neither our stack nor the firmware carries
    an absolute N-per-RPM^2 figure -- the board uses RPM only as a RELATIVE
    proxy, learning per-motor gain ratios and never a Newton. A plausible
    constant here would silently multiply every acceleration the estimator ever
    sees, which is the `pool_depth_m = 4.0` hazard exactly: a parameter nobody
    set deliberately, scaling everything downstream. So an unset k returns None,
    and the caller predicts without a thrust input rather than with a guessed one.

    To measure it: restrain the hull on a load cell, command one horizontal
    thruster, and record (thrust_N, rpm) at several duties. k is the slope of
    thrust against rpm^2 through the origin. That is a GATE-0-shaped procedure
    and needs thrusters on the vehicle.

    Returns None -- never zeros -- when rpm is absent, k is unset, or fewer than
    all four horizontal thrusters are reporting. A partial ring cannot be summed
    into a body force: the missing motor's contribution is unknown, not zero.
    """
    if k_n_per_rpm2 is None or not (k_n_per_rpm2 > 0.0):
        return None
    if not (mass_kg > 0.0):
        return None
    if rpm is None:
        return None
    horiz = list(rpm)[:4]
    if len(horiz) < 4 or any(v is None for v in horiz):
        return None

    f_fwd = 0.0
    f_lat = 0.0
    for i, v in enumerate(horiz):
        n = float(v)
        # Signed RPM is the whole reason ESC_STATUS is worth CRC-checking by
        # hand: a prop turning backwards must SUBTRACT, and unsigned magnitude
        # cannot express that. Reverse is also weaker, hence the asymmetry.
        thrust = k_n_per_rpm2 * n * n
        if n < 0.0:
            thrust = -thrust * REVERSE_EFFICIENCY
        f_fwd += thrust * _FWD_SIGN[i] * _HORIZ_COS
        f_lat += thrust * _LAT_SIGN[i] * _HORIZ_COS

    return (f_fwd / mass_kg, f_lat / mass_kg)
