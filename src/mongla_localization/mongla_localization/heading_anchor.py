"""Recover an ABSOLUTE heading from a prop whose world bearing is known.

The problem. Our BNO is deliberately magnetometer-free, so its yaw is
relative to wherever the board happened to boot: a one-shot offset, and an
arbitrary zero. Measured drift is under 0.01 deg/min at rest, which is
excellent and does not help at all with the zero being unknown. Every mission
that says "turn to 90" means 90 from boot, so a hull powered on at a different
angle flies a different course from the same mission file.

Why this, and not a magnetometer. A compass inside an aluminium hull sitting
on eight thrusters is the sensor this stack already decided not to trust, and
adding one is a hardware change we cannot test in water this week. A prop whose
orientation the rulebook fixes is a heading reference that is already in the
pool, costs nothing, and is read by a camera we already run.

And why it must consume a fused pose, never a raw one. The planar flip puts
the mirrored branch at roughly -yaw, so re-anchoring on a flipped frame writes
an absolute heading that is wrong by TWICE the off-axis angle -- and then every
later turn inherits it. A wrong heading zero is worse than no heading zero,
because the mission stops being able to tell it is lost. `pose_cluster` exists
to answer that question first; this refuses anything it has not answered.

BumblebeeAS ship exactly this and it is one of the few things they ship rather
than describe: after the second torpedo shot they compute
`(odom_yaw + torp_yaw - board_yaw) % 2pi` into a global `zero_yaw`, making the
board a heading reference for every later task. The published literature calls
the family rotation-aiding visual landmarks for an AHRS.

Pure math. No ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# An anchor taken from a pose this wide is not a heading reference, it is a
# guess with a decimal point. Degrees of median absolute deviation across the
# fused cluster.
MAX_SPREAD_DEG = 6.0

# Below this many agreeing frames the fuse itself is not trusted, and neither
# is a heading derived from it. Kept separate from the fuser's own floor: a
# pose good enough to steer on for one second is not necessarily good enough to
# redefine north with.
MIN_SUPPORT = 6


@dataclass(frozen=True)
class Anchor:
    ok: bool
    absolute_deg: float = float('nan')   # the hull's TRUE world heading now
    offset_deg: float = float('nan')     # add to relative yaw to get absolute
    support: int = 0
    spread_deg: float = float('nan')
    rule: str = ''                       # which fuse rule decided the pose
    reason: str = ''


def _wrap180(deg: float) -> float:
    """Fold an angle into (-180, 180]. The package's one copy: the filter,
    resection and pose clustering import it from here."""
    d = math.fmod(float(deg) + 180.0, 360.0)
    if d <= 0.0:
        d += 360.0
    return d - 180.0


def _wrap360(deg: float) -> float:
    d = math.fmod(float(deg), 360.0)
    return d + 360.0 if d < 0.0 else d


def absolute_heading(board_yaw_deg: float, board_world_bearing_deg: float) -> float:
    """The hull's world heading, from one look at a prop of known bearing.

    DERIVATION, because a sign error here is a mission that drives the wrong
    way with total confidence.

    `board_world_bearing_deg` (B) is the compass bearing the board's FACE
    points along -- the direction its outward normal points in the world.
    `board_yaw_deg` (θ) is this stack's `TargetPose.yaw_deg`: positive when the
    board's normal points RIGHT of the optical axis.

    To see a board's face you must be in front of it, looking back along its
    normal, so head-on the hull's heading is B - 180. A normal that appears θ
    to the right means the hull has yawed θ to the LEFT of head-on. Hence

        H = B - 180 - θ

    Two checks, both worked by hand:
      * A board facing north (B=0) seen head-on (θ=0) -> H = -180 = 180. The
        hull faces south, which is the only way to look at a north-facing face.
      * From there, yaw the hull right to H=190. The normal still points north,
        the optical axis now points 190, and the normal appears 10 deg to the
        LEFT, so θ = -10 and H = 0 - 180 + 10 = -170 = 190. It closes.

    Returned in [0, 360) because a heading is a compass bearing, while
    `TargetPose.yaw_deg` is a signed offset. Mixing those two conventions is
    its own recurring defect, so the types are kept visibly different.
    """
    return _wrap360(float(board_world_bearing_deg) - 180.0 - float(board_yaw_deg))


def anchor_from(fused, hull_relative_yaw_deg: float,
                board_world_bearing_deg: float, *,
                min_support: int = MIN_SUPPORT,
                max_spread_deg: float = MAX_SPREAD_DEG) -> Anchor:
    """Turn a FUSED pose into a heading offset, or say why not.

    `fused` is a `pose_cluster.Fused`. Everything about believing the pose is
    already decided there -- which branch, on what evidence, how tightly the
    frames agreed -- and this adds only the two gates that are specific to
    redefining a heading zero rather than steering on a pose.
    """
    if not getattr(fused, 'decided', False):
        return Anchor(False, reason=f'pose not decided: '
                                    f'{getattr(fused, "reason", "") or "no fuse"}')
    support = int(getattr(fused, 'support', 0))
    spread = float(getattr(fused, 'spread_deg', float('nan')))
    rule = str(getattr(fused, 'rule', ''))
    # Only a geometric resolution redefines a heading (issue #55). A pose
    # decided by counting can be the mirror branch: a 7/3 wrong-branch majority
    # was accepted and anchored 54 deg against a true 9. The mirror is wrong by
    # twice the off-axis angle, and every later turn would inherit it.
    from mongla_localization.pose_cluster import RULE_VIEWPOINT
    if rule != RULE_VIEWPOINT:
        return Anchor(False, support=support, spread_deg=spread, rule=rule,
                      reason=f'pose decided by {rule or "an unknown rule"!r}, '
                             f'not by a change of viewpoint: a heading anchor '
                             f'needs the mirror branch ruled out by geometry')
    if support < min_support:
        return Anchor(False, support=support, spread_deg=spread, rule=rule,
                      reason=f'support {support} < {min_support} for a heading '
                             f'anchor')
    if not (spread <= max_spread_deg):        # NaN-safe: NaN fails this
        return Anchor(False, support=support, spread_deg=spread, rule=rule,
                      reason=f'pose spread {spread:.1f} deg is too wide to '
                             f'redefine a heading')
    absolute = absolute_heading(float(fused.yaw_deg), board_world_bearing_deg)
    offset = _wrap180(absolute - float(hull_relative_yaw_deg))
    # A hull heading the board did not give (head() returns NaN) is not a
    # zero: an anchor built on it is ok=True with a NaN offset, and every pool
    # fix after it is built from boot-relative bearings.
    if not (math.isfinite(absolute) and math.isfinite(offset)):
        return Anchor(False, support=support, spread_deg=spread, rule=rule,
                      reason=f'no finite heading to anchor (hull yaw '
                             f'{hull_relative_yaw_deg!r}, prop bearing '
                             f'{board_world_bearing_deg!r})')
    return Anchor(True, absolute_deg=absolute, offset_deg=offset,
                  support=support, spread_deg=spread, rule=rule)


def apply_offset(relative_deg: float, offset_deg: Optional[float]) -> float:
    """Relative heading -> absolute, or straight through when never anchored.

    Passing through unanchored is deliberate: a mission written against boot-
    relative headings keeps working exactly as before, and an anchor is
    something it opts into. Silently changing what `turn(90)` means for every
    existing mission would be the more dangerous choice.
    """
    if offset_deg is None or offset_deg != offset_deg:
        return _wrap360(relative_deg)
    return _wrap360(float(relative_deg) + float(offset_deg))


# A floor feature bounds heading DRIFT. `flow_floor` measures the grid's or the
# lane line's angle in the image (`mongla_vision.flow.tile_grating`,
# `pool_lines`); the filter applies it through this.
def snap_to_grid(current_yaw_deg: float, grid_angle_deg: float,
                 *, max_correction_deg: float = 20.0,
                 period_deg: float = 90.0) -> Optional[float]:
    """Pull a drifting yaw back onto the floor's grid. None if it is too far.

    This is the half that matters, and it is not a heading source -- it is a
    Drift bound. The grid gives orientation modulo 90 degrees, so it cannot
    tell you which way you are facing; a square floor looks identical from four
    directions. What it CAN do is say that whatever your heading is, its
    residual against the grid should be constant -- so any accumulated gyro
    drift shows up as that residual moving, and can be removed.

    A free-running BNO drifts without bound. With this, it cannot drift further
    than half a grid cell (45 degrees) before the correction is unambiguous,
    and in practice `max_correction_deg` keeps it far tighter than that. No
    magnetometer, which our hull deliberately does not fuse because of the
    thrusters; no prop in view; no detection.

    `max_correction_deg` is the safety. A correction larger than this means the
    estimate and the floor disagree about which grid line is which -- a
    four-fold aliasing error -- and applying it would snap the hull 90 degrees
    onto the wrong branch. Refusing is the only safe answer: a wrong heading is
    worse than a drifting one, because a drifting one is still roughly right.

    `period_deg` is the feature's symmetry: 90 for a square grid, 180 for a
    lane line (identical from both ends). Same bound, half the aliasing.
    """
    if grid_angle_deg is None or current_yaw_deg is None:
        return None
    if not (math.isfinite(current_yaw_deg) and math.isfinite(grid_angle_deg)):
        return None
    period = float(period_deg)
    # Residual of the current estimate against the feature, wrapped to +/-P/2.
    resid = (float(current_yaw_deg) - float(grid_angle_deg)) % period
    if resid > period / 2.0:
        resid -= period
    if abs(resid) > float(max_correction_deg):
        return None
    return float(current_yaw_deg) - resid
