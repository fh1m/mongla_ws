"""Several references instead of one, and a policy for when to add another.

⛔ THE DEFECT THIS FIXES, AND ITS MEASUREMENT. `lock_node.py` snapped the anchor
reference ONCE -- the guard was literally `not self._anchor.has_reference` --
and never refreshed it while a reference existed. Measured 2026-09-24 on the
archive clips, through the shipped backend at 320x240, matching each frame
against that one frozen reference:

    clip                +1 s   +3 s   +5 s   +8 s
    mirpur_torpedo       189     65    134     24
    mirpur_torpedo_1      69     43     51     12   <- below the 15 bar
    mirpur_gate          195     36    237     31
    octagon              182    115     73     18
    torpedo (clear)      355    199    121     88

Every clip's minimum is its +8 s column, and the murkiest clip falls UNDER the
trust bar. A reference does not fail suddenly; it decays with elapsed time. The
same frames against a FRESH reference are the +1 s column, so this is not a
limit of the descriptor -- it is a limit of never taking a new photograph.

WHAT THIS IS, IN THE LITERATURE'S TERMS. Long-term tracking (LTMU, Dai et al.,
CVPR 2020) is a local tracker, a re-detector, a VERIFIER and a META-UPDATER --
the last answering "is the tracker ready for updating in this frame?". Our
ladder had the first two: the LK follower and the Hailo detector. This module is
the other two, and it gets the verifier nearly free, because a MAGSAC inlier
count against a real reference already IS a geometric verification.

WHY IT COMPOSES `Anchor` RATHER THAN REPLACING IT. Every reference here is an
ordinary `Anchor` over a shared backend. The ROI scaling, the homography, the
pose, the refusal below `min_inliers` -- all of it is code already measured on
footage, and a second implementation would be a second thing to keep true. The
bank adds exactly what a plain list cannot: which reference answered, which to
throw away, and when to take a new one.

THE ENROLMENT POLICY IS PORTED, INCLUDING ITS BUG. The shape comes from a
prototype gallery (`dino_idea_v12.py:409-467`): cap the bank, enrol only from a
confident detection that ALSO already agrees with the bank, score best-of-bank
rather than mean. ⛔ But that implementation can never fill: its match returns
0.0 for an empty bank while enrolment demands a score above 0.7, so the first
entry is impossible and the gallery stays empty forever -- its author left a
warning at `:995` ("No templates learned yet") which is that deadlock firing.
`enrol()` therefore bootstraps the first reference UNCONDITIONALLY, and
`test_the_bootstrap_deadlock_is_injection_verified` restores the broken rule to
prove the guard is real.

EVICTION IS BY YIELD, NOT AGE. The table above says what decays is usefulness;
age is only a proxy for it. A reference that still carries the most inliers is
the one to keep however old it is, and the newest is not automatically better.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .anchor import Anchor, AnchorPose, MIN_COSSIM, MIN_INLIERS

# How many references to hold. 5 is the prototype gallery's cap and a starting
# point, not a measured constant: the falsifier is the +8 s column above, which
# a bank must beat clip-for-clip. Raising it costs one match per reference per
# evaluation, which is why `locate()` stays linear and small.
CAPACITY = 5

# A detection must be at least this confident before its view is worth storing.
# Deliberately a FLOOR and not a threshold to act on -- the detector's own
# control-confidence gate lives elsewhere, and this one exists only to keep a
# doubtful frame out of long-term memory.
CONF_FLOOR = 0.60

# Ask for a refresh while the match is still comfortably above the trust bar.
# 40 against MIN_INLIERS=15: the point is to take the new photograph BEFORE the
# old one stops working, and the measured decay is steep enough (189 -> 24 over
# 8 s) that waiting for 15 means waiting until the rung has already failed.
REFRESH_INLIERS = 40


@dataclass
class EnrolResult:
    """Why a view was or was not taken into the bank.

    A bare bool would make every refusal look the same, and the reasons are
    operationally different: 'confidence' means the detector was unsure,
    'disagrees' means this view may not be the target at all, and 'empty' means
    the backend found nothing worth storing.
    """
    accepted: bool
    reason: str = ''
    index: Optional[int] = None
    keypoints: int = 0


@dataclass
class BankPose:
    """An `AnchorPose` plus WHICH reference produced it.

    The index is not decoration: an operator watching the lock needs to know
    the bank answered from the reference snapped ten seconds ago rather than
    the one from two seconds ago, and eviction needs the same information.
    """
    ok: bool
    inliers: int = 0
    index: Optional[int] = None
    pose: Optional[AnchorPose] = None

    @property
    def confidence(self) -> float:
        return 0.0 if self.pose is None else float(self.pose.confidence)


class CheckpointBank:
    """Up to `capacity` references over one backend, plus the update policy."""

    def __init__(self, backend, *, capacity: int = CAPACITY,
                 min_inliers: int = MIN_INLIERS,
                 min_cossim: float = MIN_COSSIM,
                 conf_floor: float = CONF_FLOOR,
                 refresh_inliers: int = REFRESH_INLIERS,
                 k=None):
        self._be = backend
        self._cap = int(max(1, capacity))
        self._min_inliers = int(min_inliers)
        self._min_cossim = float(min_cossim)
        self._conf_floor = float(conf_floor)
        self._refresh = int(refresh_inliers)
        self._k = k
        self._refs: list[Anchor] = []
        # Best inlier count each reference has produced. This is the eviction
        # key, and it starts at the snap's keypoint count so a brand-new
        # reference is not evicted before it has ever been asked a question.
        self._yields: list[int] = []
        # Injection hook for the ported deadlock. Production never sets it;
        # the test does, to watch the bank stay empty forever.
        self._require_agreement_always = False

    # -- state -------------------------------------------------------------- #
    @property
    def size(self) -> int:
        return len(self._refs)

    @property
    def yields(self) -> list[int]:
        return list(self._yields)

    @property
    def has_reference(self) -> bool:
        return bool(self._refs)

    def clear(self) -> None:
        self._refs.clear()
        self._yields.clear()

    def reference_image(self, i: int):
        """The frame reference `i` was snapped from, for display. A number of
        inliers says a match is good and cannot say good *at what*."""
        return self._refs[i].reference_image if 0 <= i < len(self._refs) else None

    @property
    def reference_roi(self):
        """The newest reference's ROI, in BACKEND pixels.

        The newest rather than the best: this is what drawing and the pose
        message use to say where the target was last SEEN, and the most recent
        checkpoint is the honest answer to that. `locate()` already reports
        which reference actually answered when that is the question.
        """
        return self._refs[-1].reference_roi if self._refs else None

    # -- enrolment (the meta-updater) --------------------------------------- #
    def enrol(self, gray: np.ndarray, roi=None, det_conf: float = 0.0,
              *, force: bool = False) -> EnrolResult:
        """Consider storing this view as a reference.

        `force` skips the POLICY and is for a caller that already knows it wants
        a checkpoint -- a deliberate operator snap, or a test. It does not skip
        the capacity cap or the empty-reference check, because those protect the
        bank's invariants rather than its policy.
        """
        bootstrap = not self._refs and not self._require_agreement_always

        if not (force or bootstrap):
            if float(det_conf) < self._conf_floor:
                return EnrolResult(False, 'confidence')
            # The view must already agree with something we hold. This is what
            # stops the bank quietly learning a distractor the detector
            # happened to be confident about.
            if self.locate(gray).inliers < self._min_inliers:
                return EnrolResult(False, 'disagrees')

        if self._require_agreement_always and not self._refs:
            # The ported rule, restored only by the injection test: agreement
            # is demanded of a bank that cannot supply it.
            return EnrolResult(False, 'disagrees')

        a = Anchor(self._be, min_inliers=self._min_inliers,
                   min_cossim=self._min_cossim, k=self._k)
        n = a.snap(gray, roi=roi)
        if n <= 0:
            # An empty reference is worse than none: it occupies a slot and can
            # never answer. `Anchor.snap` returns the count precisely so this is
            # visible here rather than at the first silent non-engagement.
            return EnrolResult(False, 'empty', keypoints=0)

        if len(self._refs) >= self._cap:
            self._evict()
        self._refs.append(a)
        self._yields.append(int(n))
        return EnrolResult(True, 'ok', index=len(self._refs) - 1, keypoints=n)

    def _evict(self) -> None:
        """Drop the least useful reference, which is not the oldest one."""
        i = int(np.argmin(self._yields))
        self._refs.pop(i)
        self._yields.pop(i)

    def wants_refresh(self, gray: np.ndarray) -> bool:
        """Is the best reference decaying toward uselessness?

        Asked against `refresh_inliers`, not `min_inliers`: a bank that waits
        for the trust bar has waited until the rung already failed. An empty
        bank always wants a reference.
        """
        if not self._refs:
            return True
        return self.locate(gray).inliers < self._refresh

    # -- use ---------------------------------------------------------------- #
    def locate(self, gray: np.ndarray) -> BankPose:
        """Best-of-bank, and say which one.

        Best-of rather than a vote or a mean: the references are views of one
        target from different instants, so the one with the most inliers is the
        one whose viewpoint this frame actually resembles. Averaging their poses
        would produce a position none of them claims -- the same argument
        `lock_state.arbitrate` makes for refusing to blend rungs.
        """
        best = BankPose(ok=False)
        for i, a in enumerate(self._refs):
            p = a.locate(gray)
            n = int(getattr(p, 'inliers', 0) or 0)
            if n > best.inliers:
                best = BankPose(ok=bool(p.ok), inliers=n, index=i, pose=p)
            if n > self._yields[i]:
                self._yields[i] = n
        return best

    def verify(self, gray: np.ndarray) -> int:
        """The LTMU verifier: how much evidence says this is still the target.

        Returns an inlier count, and **0 when the bank cannot stand behind an
        answer** -- an empty bank, a refused match, a backend that found
        nothing. A caller comparing against `min_inliers` then gets the same
        semantics everywhere, and no path here invents support.
        """
        p = self.locate(gray)
        return int(p.inliers) if p.ok else 0
