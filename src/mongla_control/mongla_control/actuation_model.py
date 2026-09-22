"""What the board actually does with a demand, and how to undo it.

⛔ WHY THIS EXISTS. Between a host demand and a thruster there are four shaping
stages, and until 2026-09-22 the host modelled none of them. The visible symptom
is `VISION_YAW_MIN_PCT = 5.0` in `motion_vision.py`, which the code itself
labels "a hardware spin-up assumption, NOT a measured value" -- a floor invented
to cross a deadband nobody had measured.

There is no deadband. There is a FLOOR, and it is three times higher.

THE CHAIN, transcribed from the firmware (srot-control-board, Hengla):

    d                                    the host's demand, -1..1
      -> PILOT_EXPO                      task_control_loop.cpp:161
         t = (1-pe)*d + pe*d^3           pe = PILOT_EXPO, scaled by PILOT_SPEED
      -> the mixer, entries +-1          mixer.cpp:26   M[8][6]
         FRAME_REVERSE negates all six   (1 on this hull)
         per-group saturation scaling    mixer.cpp:56   horiz and vert separately
      -> inverse thrust curve            mixer.cpp:15   thstExpo(t, MOT_THST_EXPO)
      -> lift into [SPIN_MIN, 1]         mixer.cpp:127
         shaped = sm + (1-sm)*thr
      -> DShot 3D band                   mixer.cpp:136
         forward 1049..2047, reverse 48..1047

VERIFIED AGAINST THE BOARD on 2026-09-22, nothing attached to any ESC, armed in
STABILIZE, eleven demand levels read off the RP2350 console: worst error
**3 counts of 999 (0.30 %)**, and that one is at full scale where the band
saturates. Raw capture: `.claude/context/workbench/data/mixer_ladder.json`.

⛔ THE FINDING THAT MATTERS MORE THAN THE GAIN. `MOT_SPIN_MIN` lifts every
non-zero demand into [spin_min, 1] -- the firmware comment says "so the smallest
command crosses the prop deadband instead of doing nothing then lurching". The
consequence is that **the smallest output this vehicle can command is 15 % of
full scale.** A demand of 0.001 and a demand of 0.02 both produce roughly that.
No host-side gain changes it: it is a floor, not a slope. Precision alignment
has a minimum actuation step, and it is large.

⚠ WHAT THIS DOES NOT GIVE YOU. Newtons. Every number here is a DShot command
fraction. `k_n_per_rpm2` still has no value, so nothing here converts to force,
and a column of the mixer is not a thrust. Do not use this to predict motion.
"""
from __future__ import annotations

import math

# Firmware defaults (include/config.h). Read the live values off the board with
# `get_param` before trusting these -- a params reset restores defaults silently
# and this module cannot tell.
DEF_PILOT_EXPO = 0.30        # config.h:537
DEF_MOT_THST_EXPO = 0.65     # config.h:464
DEF_MOT_SPIN_MIN = 0.15      # config.h:474
DEF_MOT_SPIN_ARM = 0.0       # config.h:475  0 = props stopped at neutral

# DShot 3D, both bands counting UPWARD from their own floor. There is no single
# neutral: 48 and 1048 are each "no output" for their direction.
DSHOT_FWD_MIN, DSHOT_FWD_MAX = 1049, 2047
DSHOT_REV_MIN, DSHOT_REV_MAX = 48, 1047
DSHOT_NEUTRAL = 1048
DSHOT_SPAN = 999

# Below this the firmware treats the stick as centred and returns neutral
# outright (mixer.cpp:102), so the floor does not apply.
CENTRE_EPS = 0.005


def pilot_expo(d: float, expo: float = DEF_PILOT_EXPO) -> float:
    """Stick shaping. `task_control_loop.cpp:161`, `attitude_control.cpp:22`."""
    expo = min(max(expo, 0.0), 1.0)
    return (1.0 - expo) * d + expo * d * d * d


def thrust_expo(t: float, expo: float = DEF_MOT_THST_EXPO) -> float:
    """Demand -> throttle ratio. `mixer.cpp:15`.

    Propeller thrust goes as RPM^2 and throttle -> RPM is roughly linear, so
    thrust ~ (1-e)*thr + e*thr^2. This solves that quadratic for thr, which is
    why it is called an INVERSE thrust curve: it linearises command -> thrust.
    """
    expo = min(max(expo, 0.0), 1.0)
    if expo < 0.001:
        return t
    return ((expo - 1.0)
            + math.sqrt((1.0 - expo) ** 2 + 4.0 * expo * t)) / (2.0 * expo)


def inverse_thrust_expo(thr: float, expo: float = DEF_MOT_THST_EXPO) -> float:
    """Undo `thrust_expo`. This is the ORIGINAL quadratic, not a numeric solve:
    `thrust_expo` is the root of `t = (1-e)*thr + e*thr^2`, so evaluating that
    polynomial inverts it exactly."""
    expo = min(max(expo, 0.0), 1.0)
    return (1.0 - expo) * thr + expo * thr * thr


def _solve_pilot_expo(t: float, expo: float = DEF_PILOT_EXPO) -> float:
    """The d with `pilot_expo(d) == t`, for t in [0, 1].

    pe*d^3 + (1-pe)*d is strictly increasing on [0, 1] for pe in [0, 1], so the
    root is unique. Bisection rather than Newton: sixty iterations is cheaper
    than the branch that decides whether Newton converged, and this is not in a
    500 Hz loop.
    """
    expo = min(max(expo, 0.0), 1.0)
    if expo < 1e-9:
        return min(max(t, 0.0), 1.0)
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if pilot_expo(mid, expo) < t:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def demand_to_fraction(d: float, *, pilot: float = DEF_PILOT_EXPO,
                       thst: float = DEF_MOT_THST_EXPO,
                       spin_min: float = DEF_MOT_SPIN_MIN) -> float:
    """A single-axis demand in -1..1 -> signed output fraction of full scale.

    Single axis only: with two axes active the mixer's per-group saturation
    scaling (mixer.cpp:56) couples them, and that is not modelled here.
    """
    if not math.isfinite(d):
        return 0.0
    mag = min(abs(d), 1.0)
    if mag < CENTRE_EPS:
        return 0.0                       # mixer.cpp:102 -- centred means stopped
    shaped = spin_min + (1.0 - spin_min) * thrust_expo(pilot_expo(mag), thst)
    return math.copysign(min(shaped, 1.0), d)


def fraction_to_demand(f: float, *, pilot: float = DEF_PILOT_EXPO,
                       thst: float = DEF_MOT_THST_EXPO,
                       spin_min: float = DEF_MOT_SPIN_MIN) -> float:
    """The demand that produces output fraction `f`. The inverse of
    `demand_to_fraction`, for a caller that wants a known output.

    ⛔ RETURNS 0.0 FOR ANYTHING BELOW THE FLOOR, rather than the smallest
    non-zero demand. Asking for 5 % when the vehicle cannot produce less than
    15 % has no answer, and returning a demand that silently delivers three
    times what was asked is the failure this module exists to stop. A caller
    that wants "as little as possible" should ask `min_fraction()` and decide
    knowingly.
    """
    if not math.isfinite(f) or f == 0.0:
        return 0.0
    mag = min(abs(f), 1.0)
    if mag < spin_min:
        return 0.0
    thr = (mag - spin_min) / (1.0 - spin_min)
    t = inverse_thrust_expo(thr, thst)
    return math.copysign(_solve_pilot_expo(t, pilot), f)


def min_fraction(spin_min: float = DEF_MOT_SPIN_MIN) -> float:
    """The smallest non-zero output this vehicle can command, as a fraction of
    full scale. Not a deadband to push through -- a floor.

    Measured 2026-09-22: a demand of 0.02, the smallest tried, already produced
    18.1 % of full scale.
    """
    return spin_min


def dshot_command(fraction: float) -> int:
    """Signed output fraction -> the DShot value the console reports.

    Both bands run low->high within themselves; the reverse band is NOT
    mirrored around the neutral gap (mixer.cpp:131). Mapping it backwards makes
    the smallest reverse demand nearly full reverse -- a judder that never
    spins up.
    """
    if not math.isfinite(fraction) or fraction == 0.0:
        return DSHOT_NEUTRAL
    mag = min(abs(fraction), 1.0)
    if fraction > 0:
        return int(DSHOT_FWD_MIN + mag * (DSHOT_FWD_MAX - DSHOT_FWD_MIN))
    return int(DSHOT_REV_MIN + mag * (DSHOT_REV_MAX - DSHOT_REV_MIN))


def dshot_signed(value: int) -> int:
    """A console value -> signed magnitude in [-999, 999]. 48 and 1048 are 0."""
    if value >= DSHOT_NEUTRAL:
        return int(value - DSHOT_NEUTRAL)
    return int(-(value - DSHOT_REV_MIN))
