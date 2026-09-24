"""Stitch a track's identity back together after a gap. The 179-switch fix.

⛔ THE MEASUREMENT THAT FORCED THIS. On a 253 s vehicle recording the tracker
reported **179 identity switches** -- the same person, walking in and out of
one room, assigned a new id 179 times. That number read 0 until the id was
read from the right field (`Detection2D.id`, not the non-existent
`tracking_id`), so the fragmentation had been invisible rather than absent.

WHY IT HAPPENS. ByteTrack and OC-SORT associate on MOTION: IoU plus a Kalman
prediction. Across a real gap the prediction decays and the overlap is gone,
so a returning target cannot be matched to the track it came from and a new id
is minted. Nothing in a motion-only tracker can recognise a thing it has seen
before -- that is not a bug in ByteTrack, it is the boundary of what motion
can tell you.

⭐ THE STATE OF THE ART AGREES, AND NAMES THE PRICE. BoT-SORT unifies motion
prediction with appearance modelling and explicit camera-motion compensation
"to maintain stable object identities"; McByte++ (2026) reports up to
**+6.1 IDF1** from adding online re-identification to a tracking-by-detection
pipeline. The standard objection is cost: a dedicated Re-ID embedding network
runs **15-25 ms per frame** on top of detection, which an embedded AUV budget
does not have.

⭐⭐ WE DO NOT PAY THAT COST. XFeat is ALREADY loaded and already running for
the anchor rung -- measured at 701 FPS on the Hailo-8 (section 38) and sharing
the chip with the detector for about 10 % of its throughput (section 32). The
descriptor that re-identifies a place can re-identify a target. This is the
one piece of the SOTA recipe we can afford precisely because we built it for
something else.

⛔ AND THE FAILURE MODE IS ASYMMETRIC, SO THE GATES ARE TOO. A missed
re-identification costs one extra id -- an inconvenience. A WRONG one welds
two different objects into one identity, and every consumer downstream then
believes the target teleported. So this refuses unless the evidence is strong,
and it says why.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math

# ⭐ NOT A NEW CONSTANT. The identity bar measured in section 25 on
# ROI-cropped references -- the same configuration a track crop produces.
# 15 is the TRACKING bar and sits inside the far distribution; asserting
# identity needs more evidence than following a box does.
IDENTITY_INLIERS = 40

# A retired identity older than this is not coming back, and keeping it alive
# only creates opportunities to match the wrong thing.
MAX_AGE_S = 30.0

# Cap on remembered identities. Small on purpose: this is a short-term memory
# for a target that just left, not the checkpoint bank.
CAPACITY = 16

# ⛔ A re-identification must beat the RUNNER-UP by this factor. Two stored
# identities that both match moderately well is exactly the ambiguous case
# where a wrong merge happens, and an absolute bar alone cannot see it.
MARGIN = 1.5

MATCHED = 'matched'
REFUSED = 'refused'


@dataclass
class Retired:
    """One identity that has left, and what is needed to know it again."""
    track_id: int
    label: str
    t_last: float
    keypoints: object = None          # backend keypoints of the last good crop
    descriptors: object = None
    best_inliers: int = 0


@dataclass(frozen=True)
class ReidResult:
    state: str
    reason: str
    track_id: Optional[int] = None
    inliers: int = 0
    runner_up: int = 0


class TrackReid:
    """Short-term appearance memory for identities that just disappeared.

    Pure: no ROS, no cv2 at the decision layer. The caller supplies a matcher
    so the same logic runs against ONNX, the Hailo, or a stub in a test.
    """

    def __init__(self, matcher, *, min_inliers: int = IDENTITY_INLIERS,
                 max_age_s: float = MAX_AGE_S, capacity: int = CAPACITY,
                 margin: float = MARGIN):
        # `matcher(kp_a, desc_a, kp_b, desc_b) -> inliers`
        self._match = matcher
        self.min_inliers = int(min_inliers)
        self.max_age_s = float(max_age_s)
        self.capacity = int(capacity)
        self.margin = float(margin)
        self._retired: List[Retired] = []
        self.matched = 0
        self.refusals: Dict[str, int] = {}

    # ---- memory ---------------------------------------------------------
    def retire(self, track_id: int, label: str, t: float,
               keypoints, descriptors) -> bool:
        """Remember an identity that has just been lost.

        ⛔ An empty descriptor set is NOT stored. It can never match, and it
        would occupy a slot that a usable identity needs -- the same rule the
        checkpoint bank applies to an empty reference.
        """
        if descriptors is None or len(descriptors) < 8:
            return False
        self._retired = [r for r in self._retired if r.track_id != track_id]
        self._retired.append(Retired(int(track_id), str(label), float(t),
                                     keypoints, descriptors))
        if len(self._retired) > self.capacity:
            # Drop the OLDEST, not the weakest: a recent identity is the one a
            # returning target is most likely to be.
            self._retired.sort(key=lambda r: r.t_last)
            self._retired = self._retired[-self.capacity:]
        return True

    def forget(self, track_id: int) -> None:
        self._retired = [r for r in self._retired if r.track_id != track_id]

    def _live(self, now: float) -> List[Retired]:
        return [r for r in self._retired if (now - r.t_last) <= self.max_age_s]

    @property
    def size(self) -> int:
        return len(self._retired)

    # ---- the decision ---------------------------------------------------
    def identify(self, keypoints, descriptors, *, label: str,
                 now: float) -> ReidResult:
        """Is this new track something we have already seen?

        Returns the OLD id to reuse, or a refusal naming the reason. Every
        refusal is counted, because a re-identifier that silently never fires
        is indistinguishable from one that is switched off -- the defect this
        repo has produced more than any other.
        """
        def refuse(why: str, **kw) -> ReidResult:
            self.refusals[why] = self.refusals.get(why, 0) + 1
            return ReidResult(REFUSED, why, **kw)

        if descriptors is None or len(descriptors) < 8:
            return refuse('too few descriptors on the new track')
        live = [r for r in self._live(now) if r.label == label]
        if not live:
            return refuse('nothing retired under this label')

        scored: List[Tuple[int, Retired]] = []
        for r in live:
            n = int(self._match(r.keypoints, r.descriptors,
                                keypoints, descriptors) or 0)
            scored.append((n, r))
        scored.sort(key=lambda x: -x[0])
        best_n, best = scored[0]
        second = scored[1][0] if len(scored) > 1 else 0

        if best_n < self.min_inliers:
            return refuse(f'{best_n} inliers < {self.min_inliers}',
                          inliers=best_n, runner_up=second)
        # ⛔ THE MARGIN. Two identities that both match moderately is the
        # ambiguous case, and merging the wrong pair is unrecoverable.
        if second > 0 and best_n < self.margin * second:
            return refuse(f'ambiguous: {best_n} vs {second} runner-up',
                          inliers=best_n, runner_up=second)

        self.matched += 1
        best.best_inliers = max(best.best_inliers, best_n)
        return ReidResult(MATCHED,
                          f'{best_n} inliers, runner-up {second}',
                          track_id=best.track_id, inliers=best_n,
                          runner_up=second)
