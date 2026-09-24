"""Seconds until contact, for a MOVING observer.

The prototype this comes from (`person_tracker.py`) solves the right equation
-- it closes |rel_pos + t*rel_vel|^2 = r^2 exactly, a real quadratic rather
than a linear range/speed estimate -- and then assumes the observer is
stationary, which its own comment says.

⭐ WE ARE ONE SUBTRACTION FROM THE MOVING-VEHICLE FORM, because we have an
ego-velocity the prototype did not: the downward camera, verified to 1.09 cm
over 30 cm. Subtract the vehicle's own velocity from the relative velocity and
the answer is correct for a vehicle that is itself moving, which is the only
case that ever occurs.

WHY A QUADRATIC AND NOT range/speed. `range/speed` is the time to reach the
target's CENTRE along the current line of sight. It is wrong whenever the
closing motion is not head-on -- a pass alongside reads as an approach -- and
it cannot express "closest approach never reaches the safety radius", which is
the answer a standoff manoeuvre actually needs.

⛔ AND IT REFUSES RATHER THAN EXTRAPOLATING. Below a noise floor on the
closing rate the honest answer is "never", not a large finite number: a TTC of
900 s and a TTC of "not approaching" lead to different decisions, and a
controller cannot tell them apart if both arrive as floats.
"""
from dataclasses import dataclass
from typing import Optional, Sequence
import math

# Below this closing speed the estimate is noise. The downward camera's own
# verified error is ~1 cm over 30 cm; this is deliberately well above it.
MIN_CLOSING_MS = 0.02

NEVER = 'never'
INSIDE = 'inside'
OK = 'ok'


@dataclass(frozen=True)
class Contact:
    """Time to contact, and -- always -- whether it means anything."""
    state: str
    reason: str
    seconds: float = float('nan')
    closest_m: float = float('nan')

    @property
    def ok(self) -> bool:
        return self.state == OK


def _v(x) -> Optional[tuple]:
    if x is None:
        return None
    t = tuple(float(v) for v in x)
    return t if all(math.isfinite(v) for v in t) else None


def time_to_contact(rel_pos: Sequence[float],
                    target_vel: Sequence[float],
                    ego_vel: Optional[Sequence[float]] = None,
                    radius_m: float = 0.5,
                    min_closing: float = MIN_CLOSING_MS) -> Contact:
    """Solve |rel_pos + t*rel_vel|^2 = radius^2 for the smallest t >= 0.

    `rel_pos`   target position relative to the vehicle, metres, any frame
                in which both velocities are also expressed.
    `target_vel` the target's velocity in that frame (zero for a fixed prop).
    `ego_vel`   the VEHICLE's own velocity. ⭐ This is the term the prototype
                lacked; without it a vehicle closing on a stationary prop
                computes a TTC of infinity.
    """
    p = _v(rel_pos)
    tv = _v(target_vel) or (0.0, 0.0, 0.0)
    ev = _v(ego_vel) or (0.0, 0.0, 0.0)
    if p is None:
        return Contact(NEVER, 'no relative position')
    if radius_m <= 0 or not math.isfinite(radius_m):
        return Contact(NEVER, 'no safety radius')

    n = max(len(p), len(tv), len(ev))
    p = tuple(list(p) + [0.0] * (n - len(p)))
    rv = tuple((tv + (0.0,) * n)[i] - (ev + (0.0,) * n)[i] for i in range(n))

    pp = sum(a * a for a in p)
    vv = sum(a * a for a in rv)
    pv = sum(a * b for a, b in zip(p, rv))

    if pp <= radius_m * radius_m:
        return Contact(INSIDE, 'already within the safety radius',
                       seconds=0.0, closest_m=math.sqrt(pp))

    speed = math.sqrt(vv)
    if speed < min_closing:
        return Contact(NEVER, f'closing at {speed:.4f} m/s, below the '
                              f'{min_closing:.3f} m/s noise floor',
                       closest_m=math.sqrt(pp))

    # Closest approach on the straight-line model. Reported even when contact
    # never happens, because "it will pass 0.4 m away" is the number a
    # standoff decision needs and a bare "never" throws it away.
    t_star = -pv / vv if vv > 0 else 0.0
    if t_star <= 0.0:
        closest = math.sqrt(pp)
    else:
        closest = math.sqrt(max(0.0, pp + 2 * pv * t_star + vv * t_star * t_star))

    disc = pv * pv - vv * (pp - radius_m * radius_m)
    if disc < 0.0:
        return Contact(NEVER, f'closest approach {closest:.2f} m stays '
                              f'outside {radius_m:.2f} m', closest_m=closest)
    root = math.sqrt(disc)
    t1 = (-pv - root) / vv
    t2 = (-pv + root) / vv
    ts = [t for t in (t1, t2) if t >= 0.0]
    if not ts:
        return Contact(NEVER, 'contact is in the past, not ahead',
                       closest_m=closest)
    return Contact(OK, f'closing at {speed:.2f} m/s', seconds=min(ts),
                   closest_m=closest)
