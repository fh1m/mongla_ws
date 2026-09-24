"""When may a recognised place become a position fix?

A dead-reckoning filter with no GPS and no DVL cannot bound its own drift. A
recognised place can -- `inekf.update_position` takes exactly the (x, y) the
bank stores with each checkpoint. That makes this the highest-value thing the
checkpoint bank can do, and also the most dangerous: ⛔ a wrong hard constraint
does not merely add error, it makes the filter CONFIDENT about it, and a
confident wrong position is worse than an honest drifting one.

So this module is all refusal. It is pure -- no ROS, no cv2, no I/O -- so every
gate can be tested without a vehicle, which matters because the vehicle cannot
test this at all today (one camera, and a barometer that reports "not
initialised").

WHAT IS CLAIMED. Proximity, not displacement: "the vehicle is at the stored
place, give or take the apparent offset". Resolving the homography's offset
into world x and y would need YAW as well as altitude, and a yaw error rotates
the fix into a confidently wrong place. Claiming proximity needs only scale,
and the offset magnitude is folded into sigma instead of into the position.

THE BAR. Section 24 measured the downward camera separating same-place from
different-place by an order of magnitude: near p50 76-192 inliers, far p50
9-16. `MIN_INLIERS` for tracking is 15, which sits INSIDE the far
distribution -- ample to follow a target, useless to assert a position. The
closure bar is 100: above the far distribution entirely, and near the middle
of the near one.
"""
from dataclasses import dataclass
from typing import Optional

# --------------------------------------------------------------------------- #
#  Bars. Each is a refusal threshold, and each has a reason.
# --------------------------------------------------------------------------- #

# Section 24: far p50 is 9-16, near p50 is 76-192. 100 clears the far
# distribution entirely. 6.7x the 15-inlier tracking bar, deliberately.
CLOSURE_INLIERS = 100

# ⛔ SELF-CLOSURE. Seconds after enrolment the current frame matches the
# reference it was just made from, at hundreds of inliers. The filter would
# receive its OWN recent estimate back as a low-sigma measurement and shrink
# its covariance on zero new information -- the textbook way to make an EKF
# confident and wrong. A closure must be OLD and must be DISTANT.
MIN_REF_AGE_S = 30.0
MIN_TRAVEL_M = 1.5
EXCLUDE_NEWEST = 2          # belt and braces: never close on the newest refs

# Beyond this the "proximity" claim stops meaning anything: sigma would swamp
# the fix and the filter is better off with its own dead reckoning.
MAX_OFFSET_M = 1.0

# One pixel of homography residual, converted by the same scale as the offset.
# Not a guess at the homography's accuracy: a floor under it.
OFFSET_ERR_PX = 3.0

# A place must be labelled as one. A bank can hold props, checkpoints and
# places; a prop is not a position, and today's forward bank is full of crops
# of a PERSON who walks about.
PLACE_PREFIX = 'place:'


@dataclass(frozen=True)
class Closure:
    """The answer, and -- always -- why.

    `reason` is populated on acceptance too. A closure that fires needs to be
    explainable after the fact as much as one that refuses.
    """
    ok: bool
    reason: str
    xy: Optional[tuple] = None
    sigma: float = float('nan')
    index: Optional[int] = None
    inliers: int = 0
    offset_m: float = float('nan')


# The backend's own pixel grid (xfeat_onnx runs at 320x240), whose CENTRE is
# the point the offset is measured at.
BACKEND_CENTRE = (160.0, 120.0)


def offset_metres(pose, m_per_px: float, centre=BACKEND_CENTRE) -> float:
    """How far the stored view has moved, in metres, measured at the CENTRE.

    ⛔ NOT H[0,2]/H[1,2]. Those are the translation only for a pure shift
    about the origin; under any rotation, scale or perspective they are not
    the offset at all, and every prototype in this lineage read them anyway.

    ⛔ AND NOT THE WARP OF THE ORIGIN EITHER, which is the same mistake
    wearing a hat: `H @ [0, 0, 1]` IS the third column of H. This function had
    that bug, and `test_the_offset_is_read_from_the_warp...` caught it. The
    point warped has to be one the homography's rotation and perspective terms
    actually act on, so it is the frame centre -- and the offset is how far
    that centre MOVED, not where it landed.
    """
    if pose is None or getattr(pose, 'H', None) is None:
        return float('nan')
    H = pose.H
    if H is None or H.shape != (3, 3):
        return float('nan')
    import numpy as np
    cx, cy = float(centre[0]), float(centre[1])
    w = H @ np.array([cx, cy, 1.0])
    if abs(w[2]) < 1e-9:
        return float('nan')
    d = w[:2] / w[2]
    return float(np.hypot(d[0] - cx, d[1] - cy) * m_per_px)


def consider(pose, *, ref_age_s: float, travel_m: float, ref_sigma_m: float,
             m_per_px: float, bank_size: int,
             min_inliers: int = CLOSURE_INLIERS,
             min_age_s: float = MIN_REF_AGE_S,
             min_travel_m: float = MIN_TRAVEL_M,
             exclude_newest: int = EXCLUDE_NEWEST,
             max_offset_m: float = MAX_OFFSET_M,
             centre=BACKEND_CENTRE) -> Closure:
    """Decide whether a bank match may be handed to the filter as a position.

    Every refusal is named, because a loop closure that silently never fires
    is indistinguishable from one that is not wired up -- which is exactly the
    state this module was written into: nothing ever passed `position=` to
    `enrol`, so every reference carried `ref_position=None` and no closure
    could ever have fired.
    """
    if pose is None or not getattr(pose, 'ok', False):
        return Closure(False, 'no match')
    if not str(pose.label or '').startswith(PLACE_PREFIX):
        return Closure(False, f'not a place ({pose.label!r})')
    if pose.ref_position is None:
        return Closure(False, 'reference has no position -- nothing passed '
                              'position= at enrol')
    if pose.inliers < min_inliers:
        return Closure(False, f'{pose.inliers} inliers < {min_inliers}',
                       inliers=pose.inliers)

    # Self-closure guards. Age and travel are separate on purpose: a vehicle
    # holding station is old but has not moved, and one that dashed away and
    # back has moved but may be seconds old.
    if not (ref_age_s >= min_age_s):
        return Closure(False, f'reference is {ref_age_s:.1f}s old '
                              f'< {min_age_s:.0f}s -- would close on itself',
                       inliers=pose.inliers)
    if not (travel_m >= min_travel_m):
        return Closure(False, f'only {travel_m:.2f} m travelled since enrol '
                              f'< {min_travel_m:.2f} m', inliers=pose.inliers)
    if (pose.index is not None and exclude_newest > 0
            and pose.index >= bank_size - exclude_newest):
        return Closure(False, f'reference {pose.index} is among the newest '
                              f'{exclude_newest}', inliers=pose.inliers)

    # ⛔ SCALE. Pixels become metres only with a height above the floor, and
    # there is no height without a working barometer. Substituting a constant
    # here would put a plausible number where a measurement is missing, which
    # is this codebase's recurring defect.
    if not (m_per_px > 0.0) or m_per_px != m_per_px:      # NaN-safe
        return Closure(False, 'no altitude -- cannot turn pixels into metres',
                       inliers=pose.inliers)

    off = offset_metres(pose.pose, m_per_px, centre)
    if off != off:
        return Closure(False, 'no homography to measure the offset from',
                       inliers=pose.inliers)
    if off > max_offset_m:
        return Closure(False, f'offset {off:.2f} m > {max_offset_m:.2f} m -- '
                              f'too far from the stored place to call it one',
                       inliers=pose.inliers, offset_m=off)

    # ⭐ SIGMA IS COMPOSED, NEVER CONSTANT. A closure is only as good as the
    # position the reference itself carries, so a closure onto a
    # badly-localised checkpoint must inherit that badness. Closures compose,
    # and a chain of them onto one drifting anchor is how a filter convinces
    # itself of a wrong place.
    err = OFFSET_ERR_PX * m_per_px
    sigma = ((ref_sigma_m ** 2) + (off ** 2) + (err ** 2)) ** 0.5
    if not (sigma > 0.0) or sigma != sigma:
        return Closure(False, 'sigma is not finite', inliers=pose.inliers)

    return Closure(True, f'{pose.inliers} inliers, {off:.2f} m off, '
                         f'ref {ref_age_s:.0f}s / {travel_m:.1f} m ago',
                   xy=tuple(pose.ref_position), sigma=sigma,
                   index=pose.index, inliers=pose.inliers, offset_m=off)
