"""Recover an ABSOLUTE heading from a prop whose world bearing is known.

⛔ THE PROBLEM. Our BNO is deliberately magnetometer-free, so its yaw is
relative to wherever the board happened to boot: a one-shot offset, and an
arbitrary zero. Measured drift is under 0.01 deg/min at rest, which is
excellent and does not help at all with the zero being unknown. Every mission
that says "turn to 90" means 90 from boot, so a hull powered on at a different
angle flies a different course from the same mission file.

⛔ WHY THIS, AND NOT A MAGNETOMETER. A compass inside an aluminium hull sitting
on eight thrusters is the sensor this stack already decided not to trust, and
adding one is a hardware change we cannot test in water this week. A prop whose
orientation the rulebook fixes is a heading reference that is already in the
pool, costs nothing, and is read by a camera we already run.

⛔ AND WHY IT MUST CONSUME A FUSED POSE, NEVER A RAW ONE. The planar flip puts
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
