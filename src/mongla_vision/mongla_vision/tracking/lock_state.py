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

import math
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

# ⛔ HOW OLD A DETECTION MAY BE AND STILL OUTRANK A RUNG COMPUTED ON THIS FRAME
# (issue #33). Inside `FULL_AUTHORITY_S` a detection used to win outright and
# report `age_s=0.0`: a 0.69 s-old box beat a follower tracking the target in
# the current frame, and said it was fresh. During a 0.3 rad/s turn that is
# ~170 px of image motion steered on at full authority.
#
# A detection is LIVE while it is no older than the pipeline makes every
# detection: the capture->arrival latency (48 ms p95, measured on the vehicle;
# 32 ms median) plus one detector interval, during which the next one is not
# yet due. Older than that, at least one detection has been MISSED, and a rung
# that ran on the newest frame knows more. `lock_node` passes the interval it
# observes; the default is the 20 Hz design rate the ladder was sized at.
DETECTION_LATENCY_P95_S = 0.048
DESIGN_DETECTION_HZ = 20.0
DETECTION_LIVE_S = DETECTION_LATENCY_P95_S + 1.0 / DESIGN_DETECTION_HZ


def live_window(detection_interval_s: Optional[float]) -> float:
    """The live window for an observed detection interval, or the design one.

    Never past `FULL_AUTHORITY_S`: a detector slower than that has no live
    detections to speak of, and the ladder's own decay governs instead.
    """
    if not detection_interval_s or detection_interval_s <= 0.0:
        return DETECTION_LIVE_S
    return min(FULL_AUTHORITY_S, DETECTION_LATENCY_P95_S + float(detection_interval_s))


# A rung's own confidence below this is not worth acting on even if the rung
# says ok -- it is the same idea as `vision.ctrl_conf` on the detector.
MIN_RUNG_CONF = 0.10


# ⛔ HOW FAR TWO INDEPENDENT ESTIMATES MAY SEPARATE BEFORE THE LADDER STOPS
# TRUSTING EITHER. 64 px on a 640-wide frame is a tenth of the image: wide
# enough that ordinary parallax between a frame-to-frame and a
# frame-to-reference fit does not trip it, tight enough that a follower which
# has slid onto the background beside a prop does.
#
# ⚠ NOT MEASURED ON WATER YET. It is a geometric argument, not a number off a
# clip, and it is deliberately generous -- the cost of tripping late is a
# slower demotion, while the cost of tripping early is a hull that hesitates
# whenever a fish crosses the frame.
DISAGREE_PX = 64.0

# Two independent rungs landing within this FRACTION of the disagreement bar
# are treated as corroborating rather than merely not-conflicting.
_AGREE_FRAC = 0.25

# ⚠ 1.15, not the 1.3 of the R&D code. There the boost was applied to a
# DISPLAY confidence; here it scales AUTHORITY, which is how hard the hull
# pushes. A rung that is right deserves a little more reach, not a third more.
_AGREE_BOOST = 1.15


@dataclass
class LockState:
    rung: Rung = Rung.LOST
    xyxy: Optional[Tuple[float, float, float, float]] = None
    rung_conf: float = 0.0        # how much THIS rung trusts itself
    authority: float = 0.0        # how much the loop should act, 0..1
    age_s: float = float('inf')   # since the last real DETECTION
    # ⭐ HOW FAR APART THE TWO INDEPENDENT ESTIMATES ARE, in px of centre
    # separation, or NaN when only one rung answered. See `arbitrate`.
    disagreement_px: float = float('nan')

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
              disagree_px: float = DISAGREE_PX,
              live_s: float = DETECTION_LIVE_S,
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

    usable_det = (detection is not None and detection_conf >= min_rung_conf
                  and age <= full_s)
    if usable_det and age <= live_s:
        # A live detection is the confirmation the decay counts time since, so
        # it holds full authority. Its age is REPORTED, never zeroed: absence
        # is not zero, and neither is 40 ms (issue #33).
        #
        # ⛔ `age <= full_s` IS LOad-BEARING, and its absence was measured on
        # the vehicle. The caller hands in its LAST detection box, and it only
        # clears that cache when a message arrives carrying no match --
        # `lock_node._on_det` cannot run at all during a TOTAL detector
        # outage, which is the exact case this ladder exists for. Without the
        # age test this branch short-circuits before `age` is ever consulted,
        # so a silent detector produced `rung=DETECTION, authority=1.0,
        # age_s=0.0` INDEFINITELY: measured on the Pi, the label stayed
        # `detection` through a 16 s pause while /lock kept publishing at
        # 53.6 Hz.
        #
        # That is the module's own stated failure -- "the vehicle has a
        # position" becoming "the vehicle saw the target" -- reached from the
        # inside. With `vision.lock_s > 0` the control loop would have steered
        # on a frozen box at full authority forever, and nothing would log a
        # fault.
        #
        # Past `full_s` the box is not discarded: it falls through to the
        # decay path below, where FOLLOW and ANCHOR outrank a stale detection
        # and authority declines to LOST.
        return LockState(rung=Rung.DETECTION, xyxy=tuple(detection),
                         rung_conf=float(detection_conf), authority=1.0,
                         age_s=age)

    auth = authority_for(age, full_s, zero_s)
    if auth <= 0.0:
        # Past the horizon there is no rung worth listening to. Reporting LOST
        # here rather than passing a confident anchor through is the whole
        # point: recovery races the decay, it never delays it.
        return LockState(rung=Rung.LOST, age_s=age)

    # ⭐⭐⭐ THE CROSS-CHECK. Until this, `follow` won outright whenever its
    # own confidence cleared the bar and the anchor box was never looked at --
    # two independent estimates of one target, in one function, never
    # compared.
    #
    # ⛔ WHY THAT MATTERS, AND WHY THE FOLLOWER CANNOT CATCH IT ALONE. Its two
    # quality signals -- forward-backward error and point survival -- are both
    # SELF-REFERENTIAL: they measure whether LK tracked ITS OWN POINTS
    # consistently, not whether those points are on the target. A follower
    # that has smoothly latched onto a passing fish, or slid onto the textured
    # background beside the prop, has PERFECT fb error and PERFECT survival.
    # The tracking literature names exactly this: similarity-based checks
    # "struggle to detect slow, cumulative drift, or cases where the tracker
    # settles on a visually similar but incorrect background patch".
    #
    # The anchor is the independent witness. It is fitted frame-to-REFERENCE
    # against a stored patch, so its errors are uncorrelated with LK's -- it
    # cannot drift with the follower, because it never integrates.
    #
    # ⚠ DISAGREEMENT DEMOTES, IT DOES NOT SWAP. When the two centres separate
    # by more than `disagree_px`, we know one of them is wrong and NOT which:
    # the follower may have drifted, or the anchor may have matched a repeated
    # texture. Preferring the anchor would be a guess. So the ladder keeps the
    # follower's box -- it is the fresher of the two -- and cuts AUTHORITY,
    # which is the one response that is correct under either explanation.
    disagree = float('nan')
    if follow is not None and anchor is not None:
        fcx = 0.5 * (float(follow[0]) + float(follow[2]))
        fcy = 0.5 * (float(follow[1]) + float(follow[3]))
        acx = 0.5 * (float(anchor[0]) + float(anchor[2]))
        acy = 0.5 * (float(anchor[1]) + float(anchor[3]))
        disagree = math.hypot(fcx - acx, fcy - acy)

    if follow is not None and follow_conf >= min_rung_conf:
        a = auth
        # ⭐ AGREEMENT IS EVIDENCE TOO, not merely the absence of a fault.
        #
        # From the operator's own R&D fusion code (`r_&_d/features_v9.py`,
        # `fuse_visual_imu_guidance`), which had this and the shipped ladder
        # did not:
        #
        #     if dx_agree and dy_agree and ekf_vel_magnitude > 0.1:
        #         fused_confidence = min(100, visual_conf * 1.3)
        #
        # Two estimates that are INDEPENDENT -- LK integrating frame to frame,
        # XFeat fitting frame to a stored reference -- landing on the same
        # centre is a much stronger statement than either making its own
        # claim. The first version of this cross-check only punished
        # disagreement, which threw half the information away.
        #
        # ⚠ THE GUARD IS THE OPERATOR'S TOO, AND IT IS THE SUBTLE PART.
        # `abs(ekf_dx) > 5` in that code refuses to read agreement out of a
        # near-zero signal: two estimators both saying "about here" agree on
        # nothing. Here the same idea is a MINIMUM SEPARATION SCALE -- the
        # agreement bonus needs both boxes to be real and close, not two
        # degenerate boxes sitting on top of each other.
        #
        # ⛔ CAPPED AT 1.0. Authority is a fraction of what the CONTROL LOOP
        # may do, and boosting past full would let a carried rung outrank a
        # fresh detection.
        if (disagree == disagree and disagree <= disagree_px * _AGREE_FRAC
                and anchor_conf >= min_rung_conf):
            a = min(1.0, auth * _AGREE_BOOST)
        if disagree == disagree and disagree > disagree_px:
            # Scale down smoothly rather than dropping to zero: a target
            # crossing in front of clutter produces a brief spike, and a cliff
            # would make the hull stutter on something that resolves itself.
            a = auth * max(0.0, min(1.0, disagree_px / max(disagree, 1e-6)))
        return LockState(rung=Rung.FOLLOW, xyxy=tuple(follow),
                         rung_conf=float(follow_conf), authority=a, age_s=age,
                         disagreement_px=disagree)
    if usable_det:
        # A detection past its live window but inside `full_s`: a follower on
        # the current frame outranked it above, but it still beats the anchor,
        # which is fitted on frames up to a whole anchor period old and was
        # never the detector's equal. Carried with its TRUE age.
        return LockState(rung=Rung.DETECTION, xyxy=tuple(detection),
                         rung_conf=float(detection_conf), authority=auth,
                         age_s=age, disagreement_px=disagree)
    if anchor is not None and anchor_conf >= min_rung_conf:
        return LockState(rung=Rung.ANCHOR, xyxy=tuple(anchor),
                         rung_conf=float(anchor_conf), authority=auth,
                         age_s=age, disagreement_px=disagree)
    return LockState(rung=Rung.LOST, age_s=age)
