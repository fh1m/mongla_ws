"""Keep the target in frame — and know where it went if it leaves anyway.

The gap this closes, and it is the largest one left. Every rung of the
ladder, every tracker, every anchor match — all of it operates
in the IMAGE PLANE, and all of it is downstream of a decision the vehicle makes
about where to point. Grepped: there is **no field-of-view guard anywhere in
`mongla_control`**. Nothing stops the controller yawing the target out of frame,
and no tracker survives that. The perception stack can be perfect and still lose
the lock because the vehicle turned away.

The state of the art says the same thing twice.
A 2026 safety-critical visual-servoing result observes that occlusion of the
camera–target line "cause[s] visual-servoing failure **even when the robot
remains physically safe**", and that maintaining visual contact "can conflict
with navigation progress and collision avoidance" — so it makes collision
constraints HARD and field-of-view constraints SOFT, with slack. Perception-
aware planning literature encodes the same idea as a cost: keep the feature
inside the cone, and keep its image-plane velocity small.

The barrier itself is simple and, critically, **needs bearing rather than
range**:

    h = beta . (R e_c)  -  cos(psi_F)

`beta` the unit bearing to the target, `R e_c` the optical axis, `psi_F` the
half-aperture. h > 0 inside, h = 0 at the rim, h < 0 gone. A published
splitting strategy makes it "robust to bounded distance estimation errors",
which matters here because we are monocular with no DVL and range is the thing
we are least sure of.

A target that has left the frame is a navigation problem, and the stack
answers it with the localiser and the course map, not with a memory here: a
world-frame target memory was built in this module, written by the downward
lock on every sighting, never read by anything, and deleted (2026-11-18).
"""
from dataclasses import dataclass
from typing import Optional, Tuple
import math

# Margin, in units of h, below which the guard starts pushing back. Above this
# the controller is unconstrained -- a guard that is always active is a guard
# nobody will leave switched on.
# Fraction of the half-frame remaining. 0.18 means the guard wakes when
# the target is inside the outer ~18 % of the frame on either axis --
# far enough out to act, not so early it is always on.
MARGIN_ENTER = 0.18

# Below this the target is about to leave and yaw away from it is refused
# outright rather than merely damped.
MARGIN_CRITICAL = 0.06

# Soft, not hard. Visibility yields to collision avoidance, always. The
# 2026 result is explicit: collision constraints stay hard, field-of-view gets
# slack. A vehicle that holds a lock into a wall has optimised the wrong thing.
SLACK = True

OK = 'ok'
NEAR_EDGE = 'near_edge'
CRITICAL = 'critical'
LOST = 'lost'


@dataclass(frozen=True)
class Visibility:
    """How much field of view is left before the target is gone."""
    state: str
    h: float
    reason: str
    bearing_deg: float = float('nan')

    @property
    def safe(self) -> bool:
        return self.state == OK

    @property
    def scale(self) -> float:
        """Multiplier the controller should apply to motion that makes the
        bearing WORSE. 1.0 free, 0.0 refused, tapered between.

        Only ever applied to the component that increases bearing. Damping
        everything would stop the vehicle approaching a target that is dead
        centre, which is the opposite of the intent.
        """
        if self.state == OK:
            return 1.0
        if self.state in (CRITICAL, LOST):
            return 0.0
        span = MARGIN_ENTER - MARGIN_CRITICAL
        if span <= 0:
            return 0.0
        return max(0.0, min(1.0, (self.h - MARGIN_CRITICAL) / span))


def bearing_from_pixel(cx: float, cy: float, w: float, h: float,
                       fx: float, fy: float) -> Tuple[float, float, float]:
    """Unit bearing to a pixel, in CAMERA frame (optical axis = +z).

    Use the RECTIFIED intrinsics underwater. The air focal length gives a
    clean 1/n bearing error, which is the same defect `optics.py` exists to
    prevent everywhere else.
    """
    if not (fx > 0 and fy > 0):
        return float('nan'), float('nan'), float('nan')
    x = (float(cx) - w * 0.5) / fx
    y = (float(cy) - h * 0.5) / fy
    n = math.sqrt(x * x + y * y + 1.0)
    return x / n, y / n, 1.0 / n


def assess(cx: float, cy: float, w: float, h: float, fx: float, fy: float,
           *, enter: float = MARGIN_ENTER,
           critical: float = MARGIN_CRITICAL) -> Visibility:
    """How close is this box to leaving the frame? **No range required.**

    The barrier is rectangular, not conical, and that is a deliberate
    Departure from the paper. The published CBF uses
    `h = beta . (R e_c) - cos(psi_F)`, a cone, because it models a sensor with
    a conical field of view. A camera's image is a RECTANGLE, and the two
    disagree exactly where it matters: measured on our own geometry, a target
    sitting on the right-hand edge of a 640x480 frame at fx=500 scores
    **h = +0.061 against a conical barrier** -- comfortably "safe" -- while
    being one pixel from gone. The cone only closes at the diagonal CORNER,
    so a target can walk out of the left edge with the guard reporting margin
    the whole way.

    So the margin is the fraction of the half-width and half-height remaining,
    whichever is smaller: 1.0 dead centre, 0.0 on any edge, monotone between.
    The bearing angle is still reported, because it is what a controller wants
    to act on -- but it is not what the barrier is made of.
    """
    if not (fx > 0 and fy > 0) or not (w > 0 and h > 0):
        return Visibility(LOST, float('nan'), 'no intrinsics')

    hw, hh = w * 0.5, h * 0.5
    mx = 1.0 - abs(float(cx) - hw) / hw
    my = 1.0 - abs(float(cy) - hh) / hh
    hv = min(mx, my)

    bx, by, bz = bearing_from_pixel(cx, cy, w, h, fx, fy)
    ang = (math.degrees(math.acos(max(-1.0, min(1.0, bz))))
           if bz == bz else float('nan'))

    if hv <= 0.0:
        return Visibility(LOST, hv, f'{ang:.1f} deg off axis: outside the '
                                    f'frame', ang)
    if hv < critical:
        return Visibility(CRITICAL, hv, f'{ang:.1f} deg off axis, '
                                        f'{100 * hv:.0f} % margin: about to '
                                        f'leave frame', ang)
    if hv < enter:
        return Visibility(NEAR_EDGE, hv, f'{ang:.1f} deg off axis, '
                                         f'{100 * hv:.0f} % margin: near the '
                                         f'edge', ang)
    return Visibility(OK, hv, f'{ang:.1f} deg off axis, '
                              f'{100 * hv:.0f} % margin', ang)


# --------------------------------------------------------------------------- #
#  PRIOR-GUIDED DETECTION -- vision stops fighting alone
# --------------------------------------------------------------------------- #
# The detector currently searches the whole frame at one threshold, and so
# treats a weak blob in the corner exactly like a weak blob where the target
# was a moment ago. It has no idea where the vehicle is, where the target was,
# or which way either of them has moved -- although the stack knows all three.
#
# The state of the art does two things with that knowledge.
# Pose priors from inertial prediction "initialize feature search by projecting
# 3D landmarks to the current image, shrinking the search region around the
# expected pixel location", improving robustness under fast motion and blur.
# And detection-tracking feedback lowers the bar inside that region: "when
# search regions fall within predicted regions, the detector reduces the
# threshold to a lower value to enhance the detection rate in that region".
# ByteTrack makes the same argument from the other end -- similarity with
# existing tracklets is what lets a LOW-SCORE box be recovered rather than
# discarded.
#
# The prior used is the lock's own last centre (`lock_frames.py`): the place
# the target was a moment ago, in the same image, with no world model needed.
#
# And the bar is never lowered below what the detector can produce. Our HEF
# bakes an NMS floor of 0.200; nothing at runtime brings back what the chip
# already discarded. So this RAISES the bar outside the predicted region rather
# than lowering it inside -- identical in effect, and honest about the floor.

# How far the predicted pixel may be wrong before the prior is worthless, as a
# fraction of frame width. Generous: the point is to exclude the far corners,
# not to demand precision from a memory.
PRIOR_RADIUS_FRAC = 0.25


def region_conf_bar(px: Optional[Tuple[float, float]],
                    cx: float, cy: float, w: float,
                    *, inside_bar: float, outside_bar: float,
                    radius_frac: float = PRIOR_RADIUS_FRAC) -> float:
    """The confidence a detection at (cx, cy) must clear.

    Inside the predicted region the detector's own floor is enough; outside it
    the usual bar applies. With no prediction, EVERYTHING gets the strict
    bar -- an absent prior must never be read as a permissive one.
    """
    if px is None:
        return outside_bar
    r = radius_frac * float(w)
    if math.hypot(float(cx) - px[0], float(cy) - px[1]) <= r:
        return inside_bar
    return outside_bar
