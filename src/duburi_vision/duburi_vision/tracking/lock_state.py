"""One target position, from whichever rung can still supply it.

THE GOLDEN RULE this serves: the vehicle should always have a target position,
and a detection is only the BEST of several ways to get one. Measured gaps on
real competition footage are p50 0.155 s / p90 0.651 s / p99 2.418 s, so a stack
with one source spends a real fraction of every run with nothing at all.

    rung        source                    horizon      failure mode
    DETECTION   the detector              -            semantic: domain, blur
    FOLLOW      sparse LK in the last box ~1 gap       drifts, unbounded
    ANCHOR      XFeat homography          long         needs texture
    LOST        nothing                   -            -

The rungs fail for DECORRELATED reasons, which is the entire argument for
having more than one: the detector fails on appearance, the follower on texture
motion, the anchor on texture loss.

TWO RULES, AND THEY ARE WHAT MAKE IT SAFE TO ACT ON

1. **Authority decays on its own schedule, whatever rung is talking.** A lower
   rung buys TIME, never certainty. `age_s` is measured from the last real
   DETECTION -- not from the last rung output -- so a follower and an anchor
   cannot between them keep authority at 1.0 indefinitely while the vehicle
   drives on a position nothing has confirmed in ten seconds.

2. **Nothing is ever fabricated.** Each rung reports its own confidence and may
   REFUSE; a refusal drops to the next rung and eventually to LOST. There is no
   path here that invents a position, which is why the control loop can treat
   `LockState.confidence` the way it treats a detection score.

Deliberately holds no ROS and no cv2: it takes rung outputs and returns a
decision, so the node, a bag replay and the archive bench all exercise one
implementation.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class Rung(Enum):
    DETECTION = 'detection'
    FOLLOW = 'follow'
    ANCHOR = 'anchor'
    LOST = 'lost'


# How long after the last real DETECTION the lock keeps full authority, and
# when it reaches zero. Derived from the measured gap distribution: the p90 gap
# is 0.651 s, so a vehicle that gave up before that would abandon nine gaps in
# ten that the lower rungs can genuinely cover. The zero point sits past the
# p99 (2.418 s), beyond which a "lock" is a story about the past.
# 0.70 not 0.65: the measured p90 is 0.651 s and a rounded-down 0.65 does not
# actually cover it. Caught by the test that asserts the constant against the
# measurement it cites -- a one-millisecond gap, and exactly the kind that turns
# a justification into decoration.
FULL_AUTHORITY_S = 0.70
ZERO_AUTHORITY_S = 2.50

# A rung's own confidence below this is not worth acting on even if the rung
# says ok -- it is the same idea as `vision.ctrl_conf` on the detector.
MIN_RUNG_CONF = 0.10


@dataclass
class LockState:
    rung: Rung = Rung.LOST
    xyxy: Optional[Tuple[float, float, float, float]] = None
    rung_conf: float = 0.0        # how much THIS rung trusts itself
    authority: float = 0.0        # how much the loop should act, 0..1
    age_s: float = float('inf')   # since the last real DETECTION

    @property
    def have_target(self) -> bool:
        return self.rung is not Rung.LOST and self.xyxy is not None

    @property
    def confidence(self) -> float:
        """What the control loop should weigh. Rung trust TIMES time decay --
        a confident anchor on a ten-second-old lock is still an old lock."""
        return float(self.rung_conf * self.authority)

    @property
    def centre(self) -> Optional[Tuple[float, float]]:
        if self.xyxy is None:
            return None
        x1, y1, x2, y2 = self.xyxy
        return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)


def authority_for(age_s: float,
                  full_s: float = FULL_AUTHORITY_S,
                  zero_s: float = ZERO_AUTHORITY_S) -> float:
    """Linear ramp from 1.0 to 0.0 across [full_s, zero_s].

    Linear rather than a cliff because a cliff makes the hull lurch at the
    moment the lock ages out -- the same relay behaviour that made
    `heading_lock` limit-cycle until its floor was tapered.
    """
    if age_s != age_s:                       # NaN -- never seen
        return 0.0
    if age_s <= full_s:
        return 1.0
    if age_s >= zero_s:
        return 0.0
    return float((zero_s - age_s) / (zero_s - full_s))


def arbitrate(*, now: float, last_detection_t: float,
              detection: Optional[Tuple] = None, detection_conf: float = 0.0,
              follow: Optional[Tuple] = None, follow_conf: float = 0.0,
              anchor: Optional[Tuple] = None, anchor_conf: float = 0.0,
              full_s: float = FULL_AUTHORITY_S,
              zero_s: float = ZERO_AUTHORITY_S,
              min_rung_conf: float = MIN_RUNG_CONF) -> LockState:
    """Pick the highest rung that will still stand behind an answer.

    Strict priority rather than blending. Blending two positions that disagree
    produces a third that neither rung believes and that sits between a real
    target and a drifted one -- and the disagreement is exactly the case worth
    getting right. The rungs already express their doubt through confidence and
    through refusing; that is the place for softness, not the position itself.
    """
    age = float('inf') if last_detection_t <= 0 else max(0.0, now - last_detection_t)

    if detection is not None and detection_conf >= min_rung_conf:
        # A live detection resets the clock: it IS the confirmation the decay
        # is counting time since.
        return LockState(rung=Rung.DETECTION, xyxy=tuple(detection),
                         rung_conf=float(detection_conf), authority=1.0,
                         age_s=0.0)

    auth = authority_for(age, full_s, zero_s)
    if auth <= 0.0:
        # Past the horizon there is no rung worth listening to. Reporting LOST
        # here rather than passing a confident anchor through is the whole
        # point: recovery races the decay, it never delays it.
        return LockState(rung=Rung.LOST, age_s=age)

    if follow is not None and follow_conf >= min_rung_conf:
        return LockState(rung=Rung.FOLLOW, xyxy=tuple(follow),
                         rung_conf=float(follow_conf), authority=auth, age_s=age)
    if anchor is not None and anchor_conf >= min_rung_conf:
        return LockState(rung=Rung.ANCHOR, xyxy=tuple(anchor),
                         rung_conf=float(anchor_conf), authority=auth, age_s=age)
    return LockState(rung=Rung.LOST, age_s=age)
