"""Turning a vision demand into one MANUAL_CONTROL frame the board can act on.

Three jobs, all about the board rather than the camera: the smallest yaw the
board produces, whether a demand will saturate its mixer, and the frame itself,
fitted yaw-first so the axis the tool points with keeps what it asked for.
"""

from __future__ import annotations

from .. import actuation_model


# Yaw is THE essential axis for micro-aligning + holding against a small
# target (the torpedo 'hole'). Pure-proportional yaw falls below what the
# board actuates near centre, so a small residual error commands nothing and
# the hole-lock stalls just outside err_px. So: zero inside the err_px
# deadband, and close in, a floor at the smallest command that moves the hull.
#
# That smallest command is MEASURED, not guessed (the first version copied a
# 5.0 % spin-up assumption from an older autopilot). `attitude_control.cpp:87`
# gates the yaw stick at a hardcoded 0.02, AFTER PILOT_EXPO. Below it the board is not taking a rate command at all -- it is
# Holding heading. Above it the output jumps straight to 17.4 % of full scale,
# because `MOT_SPIN_MIN` lifts every non-zero demand into [0.15, 1].
#
# Measured on `tools/control_bench` -- the board's own attitude_control.cpp and
# mixer.cpp compiled natively -- under the LIVE board's parameters
# (`workbench/data/board_params_20260923.json`, read off the vehicle 2026-09-23):
#
#     stick 2.80 %  ->  yaw torque 0.000000  ->   0.00 % output
#     stick 2.86 %  ->  yaw torque 0.010369  ->  17.42 % output
#     stick 5.00 %  ->  yaw torque 0.018140  ->  19.12 % output   (the old guess)
#     stick 100  %  ->  yaw torque 0.506651  ->  70.67 % output
#
# Two consequences, and the second is the one that changes behaviour.
#  1. The old 5.0 was 1.75x higher than the smallest command that moves the hull.
#  2. A taper cannot taper. A floor ramped from 0 across an approach band (the
#     `_vision_yaw_floor` this file carried until 2026-11-06) produces EXACTLY
#     ZERO output for every value in (0, 2.856) and 17.4-19.1 % above it: a step
#     function wearing a ramp's clothes. Once the snap below existed the ramp
#     changed nothing -- 1 050 cases, identical output -- except at kp_yaw=0,
#     where it commanded yaw the caller had switched off. So it was deleted.
# So the close-in floor SNAPS UP to the smallest value that actuates:
# inside the fill gate a non-zero correction becomes at least 2.8561 % (17.4 % of
# full scale), capped by the caller's g_yaw. Rounding DOWN was tried and is
# wrong -- it stalls the hole-lock just outside err_px, which is the exact
# failure this floor was written for.
#
# And one claim we had to retract. It is NOT true that a sub-gate demand
# disturbs the board's heading hold while a true zero does not. Measured
# 2026-09-23 with a 5 deg heading error: sticks of 0.0, 0.5, 1.0, 2.0 and 2.8 %
# all produce the same hold torque, -0.097075. They land in the same branch of
# `attitude::stabilize`. The reason to act on a sub-gate demand is that it
# produces no thrust, full stop.
SROT_YAW_MIN_PCT = 100.0 * actuation_model.yaw_rate_command_floor()   # 2.8561 %


def _mixer_saturated(lat_pct: float, yaw_pct: float, fwd_pct: float) -> bool:
    """True when the board will scale this demand down before it reaches water.

    Percent-thrust in, normalised axes out -- the mixer works in [-1, 1] and
    this loop in percent, and getting that conversion wrong would make the
    check fire either never or always.

    Best-effort: a failure here must never stop a vision verb, so it degrades
    to "not saturated", which is exactly the behaviour that existed before
    this function and therefore cannot be a regression.
    """
    try:
        from mongla_control.allocation import allocate
        return allocate(yaw=yaw_pct / 100.0, forward=fwd_pct / 100.0,
                        lateral=lat_pct / 100.0).saturated
    except Exception:                       # noqa: BLE001
        return False


def _srot_drive(fc, *, fwd_pct: float, lat_pct: float, yaw_pct: float,
                prioritise: bool = True) -> None:
    """Write one MANUAL_CONTROL frame for the srot board.

    Prioritised before it leaves. The board scales a saturated horizontal
    group UNIFORMLY, so a large lateral correction steals yaw in the same
    proportion -- measured on the mixer at fwd 0.9 / lat 0.9 / yaw 0.3, yaw
    arrives at 0.143. `prioritise` fits the demand host-side, yaw first, so the
    axis the tool points with keeps what it asked for. Below saturation it
    returns the demand unchanged, so an unsaturated frame is identical to
    before. A failure degrades to the raw demand, which is the old behaviour.
    """
    fwd, lat, yaw = fwd_pct / 100.0, lat_pct / 100.0, yaw_pct / 100.0
    if prioritise:                          # vision.mixer_aware; False = raw demand
        try:
            from mongla_control.allocation import prioritise as _fit
            fit = _fit({'forward': fwd, 'lateral': lat, 'yaw': yaw})
            fwd, lat, yaw = fit['forward'], fit['lateral'], fit['yaw']
        except Exception:                   # noqa: BLE001
            pass
    fc.manual(fwd=fwd, lat=lat, up=0.0, yaw=yaw)


def _srot_yaw_expressible(mag_pct: float, cap_pct: float) -> float:
    """Round a NON-ZERO yaw magnitude (%) UP to something the board can produce.

    Why up and not down. Between 0 and `SROT_YAW_MIN_PCT` the board produces
    no thrust at all, so a demand in that range is a correction that does not
    happen. The old code reached it through a taper and stalled just outside
    the deadband -- which is the exact failure the yaw floor was written for.
    Rounding up costs a relay; rounding down costs convergence, and a verb that
    times out having never moved is worse than one that wobbles in.

    The relay is unavoidable here, not a choice we are making. `MOT_SPIN_MIN`
    puts the smallest non-zero output at ~17.4 % of full scale, so there is no
    gentler value to pick. The real fix is a heading-hold SETPOINT we can move,
    which makes small yaw a continuous function of heading error instead of a
    stick demand -- upstream ask N §2. Until that lands this is the best the
    axis can do, and it is 17.4 % rather than the 19.1 % the guessed constant
    was producing.

    An exact zero stays zero. Only a correction the loop actually wants is
    lifted; `mag_pct == 0` means the error is inside the deadband.

    `cap_pct` is the caller's speed cap (`g_yaw`): we never exceed it, so a
    mission that asked for a gentle yaw does not get a lurch because of this.

    NOT a claim about heading hold. Measured 2026-09-23: with a 5 deg heading
    error, sticks of 0.0, 0.5, 1.0, 2.0 and 2.8 % ALL produce the same hold
    torque (-0.097075). A sub-gate demand and a true zero land in the same
    branch of `attitude::stabilize`, so neither disturbs the hold. The reason to
    act here is the stall, not the hold.

    Pure.
    """
    if mag_pct <= 0.0:
        return 0.0
    return min(max(mag_pct, SROT_YAW_MIN_PCT), max(cap_pct, 0.0))
