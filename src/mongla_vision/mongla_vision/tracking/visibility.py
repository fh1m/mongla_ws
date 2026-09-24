"""Keep the target in frame — and know where it went if it leaves anyway.

⛔ THE GAP THIS CLOSES, AND IT IS THE LARGEST ONE LEFT. Every rung of the
ladder, every cue in the cascade, every re-identification — all of it operates
in the IMAGE PLANE, and all of it is downstream of a decision the vehicle makes
about where to point. Grepped: there is **no field-of-view guard anywhere in
`mongla_control`**. Nothing stops the controller yawing the target out of frame,
and no tracker survives that. The perception stack can be perfect and still lose
the lock because the vehicle turned away.

⭐ THE STATE OF THE ART SAYS THE SAME THING TWICE.
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

⭐⭐ AND THE SECOND HALF, FOR WHEN THE GUARD IS NOT ENOUGH.
A target that leaves the frame is forgotten by anything that tracks in pixels.
The fix is old and is stated most plainly in a robot-behaviour patent: store
target position "**not on a sensor coordinate system but on a world coordinate
system**", so it "remains identical from the behaviour control perspective"
however the vehicle moves. Then losing sight stops being a perception failure
and becomes a navigation problem — the vehicle still knows the bearing to turn
back to.

⛔ WHAT THIS IS NOT. It never invents a detection. `WorldTarget` holds where
the target WAS and how stale that is; it refuses to answer once the estimate is
older than its own horizon, and it carries no confidence that could be mistaken
for a sighting. The ladder's rule is unchanged: no rung fabricates.
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

# ⛔ SOFT, NOT HARD. Visibility yields to collision avoidance, always. The
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

        ⚠ Only ever applied to the component that increases bearing. Damping
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

    ⚠ Use the RECTIFIED intrinsics underwater. The air focal length gives a
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
           *, half_fov_deg: Optional[float] = None,
           enter: float = MARGIN_ENTER,
           critical: float = MARGIN_CRITICAL) -> Visibility:
    """How close is this box to leaving the frame? **No range required.**

    ⛔ THE BARRIER IS RECTANGULAR, NOT CONICAL, AND THAT IS A DELIBERATE
    DEPARTURE FROM THE PAPER. The published CBF uses
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


class WorldTarget:
    """Where the target is in the WORLD, so leaving frame is not forgetting.

    ⭐ Image-plane tracking forgets the instant the box leaves. Holding the
    target in world coordinates turns that from a perception failure into a
    navigation problem: the vehicle still knows which way to turn back.

    ⛔ It stores a MEMORY, never a sighting. `bearing_from` answers only while
    the estimate is fresher than `max_age_s`, and there is no confidence field
    that downstream could mistake for a detection.
    """

    def __init__(self, max_age_s: float = 20.0):
        self.max_age_s = float(max_age_s)
        self._xy: Optional[Tuple[float, float]] = None
        self._t: float = 0.0
        self.updates = 0

    def observe(self, vehicle_xy, yaw_deg: float, bearing_deg: float,
                range_m: float, now: float) -> bool:
        """Project a sighting into the world and remember it.

        ⛔ Range is the weakest number we have, so a non-finite or absurd one
        is refused rather than stored. A remembered position built from a
        guessed range would send the vehicle confidently to the wrong place.
        """
        # ⛔ `> 0.0` is True for inf. A memory built from an infinite range
        # is a point at infinity, and the bearing to it is meaningless.
        if vehicle_xy is None or not math.isfinite(range_m) or range_m <= 0.0:
            return False
        if not math.isfinite(yaw_deg) or not math.isfinite(bearing_deg):
            return False
        th = math.radians(yaw_deg + bearing_deg)
        self._xy = (float(vehicle_xy[0]) + range_m * math.cos(th),
                    float(vehicle_xy[1]) + range_m * math.sin(th))
        self._t = float(now)
        self.updates += 1
        return True

    def age(self, now: float) -> float:
        return float('inf') if self._xy is None else (now - self._t)

    def bearing_from(self, vehicle_xy, yaw_deg: float,
                     now: float) -> Optional[float]:
        """Relative bearing to turn to, degrees, or None.

        None means "I do not know", and it is returned for a stale memory as
        readily as for no memory at all -- because a position from a minute
        ago is not knowledge, it is a rumour.
        """
        if self._xy is None or vehicle_xy is None:
            return None
        if (now - self._t) > self.max_age_s:
            return None
        dx = self._xy[0] - float(vehicle_xy[0])
        dy = self._xy[1] - float(vehicle_xy[1])
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None
        world = math.degrees(math.atan2(dy, dx))
        rel = (world - float(yaw_deg) + 180.0) % 360.0 - 180.0
        return rel

    @property
    def xy(self) -> Optional[Tuple[float, float]]:
        return self._xy


# --------------------------------------------------------------------------- #
#  ⭐ PRIOR-GUIDED DETECTION -- vision stops fighting alone
# --------------------------------------------------------------------------- #
# ⛔ THE DETECTOR CURRENTLY SEARCHES THE WHOLE FRAME AT ONE THRESHOLD, and so
# treats a weak blob in the corner exactly like a weak blob where the target
# was a moment ago. It has no idea where the vehicle is, where the target was,
# or which way either of them has moved -- although the stack knows all three.
#
# ⭐ THE STATE OF THE ART DOES TWO THINGS WITH THAT KNOWLEDGE.
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
# ⭐⭐ WE ALREADY HOLD EVERY INPUT. `WorldTarget` has the target in pool
# coordinates, the localiser has the vehicle pose, and `optics` has the
# intrinsics. Projecting one through the others gives the predicted pixel --
# which is the sonar-substitute of section 4, built from geometry instead of a
# second sensor.
#
# ⛔ AND THE BAR IS NEVER LOWERED BELOW WHAT THE DETECTOR CAN PRODUCE. Our HEF
# bakes an NMS floor of 0.200; nothing at runtime brings back what the chip
# already discarded. So this RAISES the bar outside the predicted region rather
# than lowering it inside -- identical in effect, and honest about the floor.

# How far the predicted pixel may be wrong before the prior is worthless, as a
# fraction of frame width. Generous: the point is to exclude the far corners,
# not to demand precision from a memory.
PRIOR_RADIUS_FRAC = 0.25


def project_world_target(target_xy, vehicle_xy, yaw_deg: float,
                         w: float, h: float, fx: float,
                         ) -> Optional[Tuple[float, float]]:
    """Where a remembered world position should appear in the image, in px.

    Bearing only -- no range is used, because the horizontal pixel of a target
    depends on its BEARING and not on how far away it is. Returns None when the
    target is behind the vehicle, where no pixel exists and a projection would
    silently fold it back into the frame.
    """
    if target_xy is None or vehicle_xy is None or not (fx > 0 and w > 0):
        return None
    dx = float(target_xy[0]) - float(vehicle_xy[0])
    dy = float(target_xy[1]) - float(vehicle_xy[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return None
    rel = math.radians(
        (math.degrees(math.atan2(dy, dx)) - float(yaw_deg) + 180.0) % 360.0
        - 180.0)
    if abs(rel) >= math.radians(89.0):
        return None                      # abeam or behind: no pixel
    return (w * 0.5 + fx * math.tan(rel), h * 0.5)


def region_conf_bar(px: Optional[Tuple[float, float]],
                    cx: float, cy: float, w: float,
                    *, inside_bar: float, outside_bar: float,
                    radius_frac: float = PRIOR_RADIUS_FRAC) -> float:
    """The confidence a detection at (cx, cy) must clear.

    Inside the predicted region the detector's own floor is enough; outside it
    the usual bar applies. ⛔ With no prediction, EVERYTHING gets the strict
    bar -- an absent prior must never be read as a permissive one.
    """
    if px is None:
        return outside_bar
    r = radius_frac * float(w)
    if math.hypot(float(cx) - px[0], float(cy) - px[1]) <= r:
        return inside_bar
    return outside_bar
