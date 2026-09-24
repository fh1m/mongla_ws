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

import json
import os

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .anchor import Anchor, AnchorPose, MIN_COSSIM, MIN_INLIERS

# ⭐ HOW MANY REFERENCES, AND WHY IT IS NO LONGER A CONSTANT.
#
# This shipped at 5, copied from the prototype gallery. Measured 2026-09-24
# (`measured-bars.md` §20), matching is LINEAR in bank size -- one
# 1024x1024x64 similarity matrix plus a mutual-NN pass per reference:
#
#                 dev box        Pi 5
#     1 ref        14.4 ms      31.4 ms
#     5 refs       56.8 ms     156.7 ms
#    10 refs      122.9 ms     314.0 ms   <- at the 3 Hz budget already
#   100 refs      853.2 ms    3146.5 ms
#
# ⛔ Batching the bank into one GEMM does NOT fix it: 1.5-1.7x on the dev box
# and NOTHING on the Pi (3146 -> 3136 ms), because at this shape the matmul is
# already bound by something a bigger call does not change. That optimisation
# was measured and dropped.
#
# What does fix it is not matching everything. Each reference carries a 64-D
# signature -- the L2-normalised mean of its own descriptors, already computed,
# no second network -- and only the top-k by signature are matched. That is
# FLAT: 100 references cost what 5 do, plus a 100x64 dot product.
#
# Memory never was the constraint: a reference is 1024x64 float32 = 256 kB, so
# 64 of them is 16 MB.
CAPACITY = 64

# How wide the shortlist may get. The accuracy cost of a narrow one, measured
# on the archive clips over 120 queries (`measured-bars.md` §20.2) as the
# inlier yield lost against always matching every reference:
#
#     k=1  17.6 %     k=2  9.3 %     k=3  6.3 %     k=5  1.9 %     k=8  0.7 %
#
# ⚠ Recall@1 is only 50.8 %, and that is not the number to steer by: several
# references are nearly as good, so a "wrong" shortlist costs yield rather than
# the lock. k=5 is the knee. 8 is the ceiling this constant sets, not the
# default -- the default is whatever the time budget affords, below.
MAX_SHORTLIST = 8

# The fraction of one evaluation period the bank may spend matching. The rest
# belongs to the detector, the follower and the camera pumps; the anchor is the
# rung that can afford to be late, which is exactly why it must not be greedy.
BUDGET_FRACTION = 0.45

# A detection must be at least this confident before its view is worth storing.
#
# ⛔ WAS 0.60, AND IT STARVED THE BANK. Measured on Mirpur torpedo footage
# through our own YOLO graph: the detector's confidences are p50 **0.49**
# (min 0.22, max 0.85), so a 0.60 floor refused **10 of 13** candidate views
# and the bank ended a whole clip holding ONE reference -- which then asserted
# identity on 0 % of later frames, where a 14-reference bank asserted on 3 of 8.
#
# ⚠ And a fixed absolute floor is the wrong SHAPE here, not just the wrong
# number: §6b records our own detector's recall swinging 29.2 / 72.7 / 68.3 %
# across venues, so a confidence that means different things in different water
# cannot have a threshold set once.
#
# 0.25 because the checks that actually discriminate are elsewhere and are
# measured: `enrol` already requires the candidate to AGREE geometrically with
# the bank (≥ MIN_INLIERS), eviction is by inlier yield so a reference that
# never wins is discarded, and `recognise()` needs 40 inliers plus the
# detector's class before any identity is asserted. This floor's only job is to
# keep obvious noise out of long-term memory.
CONF_FLOOR = 0.25

# ⛔ HOW MANY REFERENCES BEFORE THE BANK MAY JUDGE A CANDIDATE.
#
# `enrol` requires a candidate to AGREE with the bank, which stops it learning
# a distractor. But a bank of one reference from one viewpoint cannot judge:
# measured on Mirpur torpedo footage, 10 of 13 candidates were refused as
# 'disagrees' and the bank ended the clip holding ONE reference. A second
# bootstrap deadlock, in the same family as the first.
#
# The prototype gallery this policy came from had it right and the port missed
# it: `dino_idea_v12.py:458` rejects only `if template_score < THRESH and
# len(self.pipe_templates) > 2` -- the gate switches on once there are three
# templates. Below that the bank is accumulating evidence, not adjudicating it.
AGREEMENT_MIN = 3

# Ask for a new checkpoint while the best one is still working.
#
# ⚠ RE-DERIVED TWICE, AND BOTH EARLIER VALUES WERE WRONG FOR PRODUCTION.
# The first (40) was reasoned from a decay that §21.1 then withdrew -- the
# curves oscillate rather than decay, so a threshold near the typical match
# fires on troughs and churns the bank. The second problem is the one §22.2
# found: every bar here was swept on WHOLE-FRAME references (1024 keypoints),
# while `lock_node` enrols with `roi=det_box` and those carry a median of 376.
#
# Measured on production-shaped references -- cropped to a real detector box,
# queried against whole live frames:
#
#     same run                p50  52
#     same prop, other run    p50  40
#     other prop              p50  25
#
# 25 sits below what a working match returns and at what a wrong one does, so
# it fires when the bank is genuinely running out of usable viewpoints rather
# than on every trough.
REFRESH_INLIERS = 25

# ⭐ THE SCALE-COVERAGE TRIGGER, and it is the meta-updater's PRIMARY criterion.
#
# §21.1 withdrew the decay story: references oscillate rather than decay, so a
# threshold on inlier count fires on troughs. §21.2 replaced it -- what the bank
# buys is VIEWPOINT DIVERSITY. This is that, made operational.
#
# Seen on real footage (`tools/pipeline_visual_check.py`, Mirpur torpedo): the
# bank enrolled where the detector was confident, which is mid-range, and then
# refused every close-range frame -- 7 inliers against a board filling the
# screen. The number said "no match"; the picture said "the prop is
# unmistakable and we had nothing at that scale to compare it to". That is the
# partial-view case a torpedo run actually fires from.
#
# A view earns a slot when its apparent SIZE is outside the band the bank
# already covers by this factor. 1.6 is roughly half an octave: close enough
# that references overlap, far enough that five of them span a useful range.
SCALE_COVERAGE = 1.6

# ⛔ THE BAR FOR ASSERTING AN IDENTITY, which is NOT the bar for tracking.
#
# `MIN_INLIERS = 15` answers "is this the same scene as the reference" -- asked
# of a reference a live detection just supplied, so identity is already known.
# It is not an identity test, and it was measured failing as one
# (`measured-bars.md` §22): 8 whole-frame references from a torpedo run cleared
# 15 inliers on 100 % of GATE frames from the same venue, and said 'torpedo'.
#
# The references had `roi=None`, so they encoded Mirpur's water and pool edge
# rather than the prop -- features every Mirpur clip contains. A genuinely
# different venue (octagon) scored 0 %, which is the same fact from the other
# side: the bank discriminates VENUE well and PROP-within-venue not at all.
#
# Swept on same-prop-other-run vs other-prop-same-venue:
#
#     bar 15: keeps 100 % of true, admits 100 % of false
#     bar 40: keeps  64 %,          admits  14 %
#     bar 60: keeps  57 %,          admits   0 %
#
# ⛔ THAT SWEEP USED WHOLE-FRAME REFERENCES AND PRODUCTION DOES NOT.
# `lock_node` enrols with `roi=det_box`, so a shipped reference holds only the
# keypoints inside the box -- measured median 376 against 1024. Re-swept on
# production-shaped references (`tools/anchor_roi_bars.py`):
#
#     bar 15: keeps  86 % of true, admits  93 % of false
#     bar 30: keeps  64 %,          admits  21 %
#     bar 40: keeps  64 %,          admits   7 %          <- this
#     bar 60: keeps  14 %,          admits   0 %          <- was shipped
#
# ⛔ At 60 a true re-acquisition fires on 14 % of frames, i.e. essentially
# never, and nothing would log a fault. That is the failure direction that
# hides: a capability that silently does not work.
#
# ⚠ 40 admits 7 % of other-prop frames rather than 0 %, and that is deliberate:
# `recognise()` does not decide on geometry alone. The detector must also agree
# on the class, and its failure mode is missing things rather than confusing
# them -- so a 7 % geometric false rate meets a semantic check that does not
# correlate with it.
IDENTITY_INLIERS = 40

# How stale a detector hypothesis may be and still corroborate an identity.
# ⭐ THE DETECTOR IS THE IDENTITY AUTHORITY, not the bank. Its failure mode is
# semantic -- it MISSES things -- rather than confusing one prop for another, so
# even a hypothesis far below the control-confidence gate is strong evidence of
# WHAT is in frame while being useless as a position. That is exactly the
# division this uses: identity from the detector, position from the bank.
IDENTITY_DET_AGE_S = 3.0

# How far the vehicle's attitude may differ from the attitude a checkpoint was
# enrolled at before the match is implausible. A third opinion that fails for
# reasons decorrelated from both vision rungs: turbidity blinds the detector and
# the matcher together, and moves the IMU not at all.
IDENTITY_ATT_DEG = 45.0


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
    # WHAT was recognised, not merely where. One bank can hold a prop, a
    # checkpoint and a place; without this the caller gets a position and no
    # idea what it belongs to.
    label: str = ''
    searched: int = 0          # how many references were actually matched
    # Where the vehicle was when the winning checkpoint was taken, (x, y) m, or
    # None. This is what a loop closure is FOR: `inekf.update_position` takes
    # exactly this, once the match is trusted enough to hand it over.
    ref_position: Optional[tuple] = None
    # Points annotated ON THE REFERENCE, carried into THIS frame through the
    # homography. See `enrol(annotations=...)`.
    points: Optional[dict] = None
    # Set only by `recognise()`. `locate()` leaves it False, because geometry
    # alone was measured unable to tell a torpedo from a gate in one venue.
    identity_ok: bool = False
    identity_why: str = ''
    ref_attitude: Optional[tuple] = None
    ref_depth_m: float = float('nan')

    @property
    def confidence(self) -> float:
        return 0.0 if self.pose is None else float(self.pose.confidence)


class CheckpointBank:
    """Up to `capacity` references over one backend, plus the update policy."""

    def __init__(self, backend, *, capacity: int = CAPACITY,
                 min_inliers: int = MIN_INLIERS,
                 min_cossim: float = MIN_COSSIM,
                 conf_floor: float = CONF_FLOOR,
                 agreement_min: int = AGREEMENT_MIN,
                 refresh_inliers: int = REFRESH_INLIERS,
                 scale_coverage: float = SCALE_COVERAGE,
                 identity_inliers: int = IDENTITY_INLIERS,
                 identity_det_age_s: float = IDENTITY_DET_AGE_S,
                 identity_att_deg: float = IDENTITY_ATT_DEG,
                 period_s: float = 0.0,
                 max_shortlist: int = MAX_SHORTLIST,
                 budget_fraction: float = BUDGET_FRACTION,
                 k=None):
        self._be = backend
        self._cap = int(max(1, capacity))
        self._min_inliers = int(min_inliers)
        self._min_cossim = float(min_cossim)
        self._conf_floor = float(conf_floor)
        self._agree_min = int(agreement_min)
        self._refresh = int(refresh_inliers)
        self._cover = float(scale_coverage)
        self._id_inliers = int(identity_inliers)
        self._id_det_age = float(identity_det_age_s)
        self._id_att_deg = float(identity_att_deg)
        self._k = k
        self._period = float(max(0.0, period_s))
        self._max_k = int(max(1, max_shortlist))
        self._budget = float(budget_fraction)
        self._refs: list[Anchor] = []
        # One 64-D signature per reference, and what each reference IS.
        self._sigs: list[np.ndarray] = []
        self._labels: list[str] = []
        # Where the vehicle WAS when each checkpoint was taken: (roll, pitch,
        # yaw) in degrees and depth in metres, or None/NaN when unknown. Stored
        # rather than required, because a bank built from stills has neither.
        self._att: list[Optional[tuple]] = []
        self._depth: list[float] = []
        self._ann: list[Optional[dict]] = []
        # Where the vehicle was in the WORLD when each checkpoint was taken,
        # (x, y) metres, or None. Supplied by the localiser at enrol time. See
        # `locate(near=...)` for what it buys.
        self._pos: list[Optional[tuple]] = []
        # Apparent size of each reference's target, as sqrt(ROI area) in
        # backend pixels, or NaN when the reference is whole-frame. This is the
        # axis `SCALE_COVERAGE` spreads the bank along.
        self._scale: list[float] = []
        # Measured cost of ONE match, in seconds, as a decaying average. This
        # is what makes the shortlist width adapt instead of being guessed:
        # the same code picks 5 on a dev box and 3 on a Pi because it has
        # measured both. 0.0 means "not yet timed" -- the first evaluation runs
        # the full width and learns the number.
        self._match_s = 0.0
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
        self._sigs.clear()
        self._labels.clear()
        self._att.clear()
        self._depth.clear()
        self._ann.clear()
        self._pos.clear()
        self._scale.clear()

    @property
    def labels(self) -> list[str]:
        return list(self._labels)

    @property
    def match_ms(self) -> float:
        """Measured cost of one match on THIS machine, or 0.0 if never timed."""
        return self._match_s * 1e3

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
    @staticmethod
    def _signature(desc: np.ndarray) -> np.ndarray:
        """One 64-D vector per reference: the L2-normalised mean descriptor.

        Not a learned global descriptor -- it is what the backend already
        produced, so retrieval costs a dot product and no second network. The
        claim it encodes is weak but real and measured: two views of one scene
        share keypoints, so their mean descriptors sit closer together than two
        views of different scenes. Recall@1 is only 50.8 % on the archive, and
        that is acceptable because being wrong costs 1.9 % of inlier yield at
        k=5, not the lock.
        """
        if desc is None or len(desc) == 0:
            return np.zeros(64, np.float32)
        v = np.asarray(desc, np.float32).mean(axis=0)
        n = float(np.linalg.norm(v))
        return (v / n) if n > 0.0 else v

    def shortlist_k(self) -> int:
        """How many references this box can afford to match, right now.

        ⭐ Derived, not configured. The same code picks a wider shortlist on a
        dev box than on a Pi because it has MEASURED one match on the machine
        it is running on -- 14.4 ms against 31.4 ms for the identical work.
        A constant here would be wrong on one of the two by a factor of two.

        Before any match has been timed, and when no period was given, it
        returns the ceiling: the first evaluation is what supplies the number,
        and refusing to search until then would be a worse failure than being
        briefly slow.
        """
        if self._period <= 0.0 or self._match_s <= 0.0:
            return self._max_k
        affordable = int((self._period * self._budget) / self._match_s)
        return int(min(self._max_k, max(1, affordable)))

    def enrol(self, gray: np.ndarray, roi=None, det_conf: float = 0.0,
              *, label: str = '', attitude=None, depth_m: float = float('nan'),
              annotations: Optional[dict] = None,
              position=None,
              force: bool = False) -> EnrolResult:
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
            # The view must already agree with something we hold -- ONCE there
            # is enough held to judge with. This is what stops the bank quietly
            # learning a distractor the detector happened to be confident
            # about, without it also stopping the bank from ever reaching two.
            if (len(self._refs) >= self._agree_min
                    and self.locate(gray).inliers < self._min_inliers):
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
        self._sigs.append(self._signature(a._ref_desc))
        self._labels.append(str(label))
        self._att.append(None if attitude is None
                         else tuple(float(v) for v in attitude))
        self._depth.append(float(depth_m))
        # ⭐ ANNOTATIONS: named points marked once on the reference -- a hole
        # centre, an aim point, the spot a marker must be dropped on. `locate()`
        # warps them into the live frame through the SAME homography the pose
        # comes from, so they are found without a detector that was ever
        # trained to find them, and they survive occlusion of the point itself
        # because the transform is fitted to the whole reference.
        #
        # This is the half of the published feature-matching approach we did
        # not have: correspondences -> PnP was already here (`anchor/pose.py`),
        # but nothing carried MEANING attached to the reference.
        #
        # ⚠ In FULL-FRAME pixels of the enrolled image, like `roi`, and scaled
        # to backend pixels here for the same reason `snap` scales the ROI: the
        # homography is fitted in backend pixels and mixing the two silently
        # moves every annotation.
        ann = None
        if annotations:
            fh, fw = gray.shape[:2]
            sx, sy = self._be.w / float(fw), self._be.h / float(fh)
            ann = {str(k): (float(v[0]) * sx, float(v[1]) * sy)
                   for k, v in annotations.items()}
        self._ann.append(ann)
        self._pos.append(None if position is None
                         else (float(position[0]), float(position[1])))
        self._scale.append(self._roi_scale(gray, roi))
        return EnrolResult(True, 'ok', index=len(self._refs) - 1, keypoints=n)

    def _evict(self) -> None:
        """Drop the least useful reference, which is not the oldest one."""
        i = int(np.argmin(self._yields))
        self._refs.pop(i)
        self._yields.pop(i)
        self._sigs.pop(i)
        self._labels.pop(i)
        self._att.pop(i)
        self._depth.pop(i)
        self._ann.pop(i)
        self._pos.pop(i)
        self._scale.pop(i)

    def _roi_scale(self, gray, roi) -> float:
        """sqrt(ROI area) in BACKEND pixels. NaN for a whole-frame reference.

        NaN rather than the frame diagonal, because a whole-frame reference
        covers no particular scale and pretending it covers the largest one
        would suppress every close-range enrolment.
        """
        if roi is None:
            return float('nan')
        fh, fw = gray.shape[:2]
        sx, sy = self._be.w / float(fw), self._be.h / float(fh)
        x1, y1, x2, y2 = (float(v) for v in roi)
        return float(np.sqrt(max(1.0, abs(x2 - x1) * sx * abs(y2 - y1) * sy)))

    def covers_scale(self, gray, roi) -> bool:
        """Is a target this size already represented in the bank?

        False means this view would ADD coverage -- the caller should enrol it
        even though the existing references are matching fine, because they
        will stop matching when the vehicle closes in.
        """
        if roi is None:
            return True                       # no scale to reason about
        s = self._roi_scale(gray, roi)
        known = [v for v in self._scale if v == v and v > 0]
        if not known or s <= 0:
            return False
        return any(max(s, k) / min(s, k) < self._cover for k in known)

    @property
    def scales(self) -> list[float]:
        return list(self._scale)

    def wants_refresh(self, gray: np.ndarray) -> bool:
        """Is the best reference decaying toward uselessness?

        Asked against `refresh_inliers`, not `min_inliers`: a bank that waits
        for the trust bar has waited until the rung already failed. An empty
        bank always wants a reference.
        """
        if not self._refs:
            return True
        return self.locate(gray).inliers < self._refresh

    # -- persistence: the bank can be built before the run ------------------ #
    #
    # ⭐ WHY THIS EXISTS. On the run there may be no confident detection to
    # enrol from at the moment the anchor is needed -- which is precisely when
    # the detector is failing, i.e. the case the rung exists for. A bank
    # prepared on practice footage does not have that dependency.
    #
    # Measured 2026-09-24 (`measured-bars.md` §21.3): references snapped on one
    # run clear the trust bar on 92 % and 100 % of frames of a DIFFERENT run of
    # the same prop. ⛔ And on a generic structural view they clear it on 8 %.
    # So preloading is a per-target capability, not a per-vehicle one.
    #
    # ⛔ A PRELOADED REFERENCE IS TRUSTED EXACTLY LIKE A LIVE ONE: it answers
    # through `MIN_INLIERS` or it does not answer. Preloading changes where a
    # checkpoint comes from, never what it has to prove -- which is what keeps
    # the octagon result a visible refusal instead of a confident wrong lock.

    SAVE_VERSION = 1

    def save(self, path: str) -> int:
        """Write the bank to a .npz. Returns the number of references saved."""
        if not self._refs:
            raise ValueError('refusing to save an empty bank')
        out = {'version': np.array([self.SAVE_VERSION]),
               'count': np.array([len(self._refs)]),
               'backend_wh': np.array([self._be.w, self._be.h]),
               'labels': np.array(self._labels, dtype=object),
               'yields': np.array(self._yields, np.int32),
               'att': np.array([(np.nan, np.nan, np.nan) if a is None else a
                                for a in self._att], np.float32),
               'depth': np.array(self._depth, np.float32),
               'pos': np.array([(np.nan, np.nan) if q is None else q
                                for q in self._pos], np.float32),
               # JSON per reference: a dict of named points is not an array,
               # and a ragged object array would not survive allow_pickle=False
               # for anyone who loads this more carefully than we do.
               'ann': np.array([json.dumps(a or {}) for a in self._ann],
                               dtype=object)}
        for i, a in enumerate(self._refs):
            out[f'k{i}'] = np.asarray(a._ref_kpts, np.float32)
            out[f'd{i}'] = np.asarray(a._ref_desc, np.float32)
            out[f's{i}'] = np.asarray(self._sigs[i], np.float32)
            roi = a.reference_roi
            out[f'r{i}'] = (np.array([np.nan] * 4, np.float32) if roi is None
                            else np.asarray(roi, np.float32))
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
        np.savez_compressed(path, **out)
        return len(self._refs)

    def load(self, path: str, *, append: bool = False) -> int:
        """Read a bank back. Returns how many references were added.

        ⛔ REFUSES A BANK BUILT AT ANOTHER RESOLUTION. Keypoints are stored in
        BACKEND pixels, so loading 640x480 references into a 320x240 backend
        would silently halve every coordinate -- no error, a plausible
        homography, and every pose wrong by a factor of two. That is the exact
        class of failure this codebase keeps finding, so it is a hard refusal
        rather than a rescale.
        """
        z = np.load(path, allow_pickle=True)
        ver = int(z['version'][0])
        if ver != self.SAVE_VERSION:
            raise ValueError(f'bank version {ver} != {self.SAVE_VERSION}')
        w, h = (int(v) for v in z['backend_wh'])
        if (w, h) != (self._be.w, self._be.h):
            raise ValueError(
                f'bank was built at {w}x{h}, backend is '
                f'{self._be.w}x{self._be.h} -- keypoints are in backend pixels, '
                f'so loading it would scale every pose silently')
        if not append:
            self.clear()
        labels = [str(v) for v in z['labels']]
        yields = [int(v) for v in z['yields']]
        n = int(z['count'][0])
        added = 0
        for i in range(n):
            if len(self._refs) >= self._cap:
                self._evict()
            a = Anchor(self._be, min_inliers=self._min_inliers,
                       min_cossim=self._min_cossim, k=self._k)
            a._ref_kpts = z[f'k{i}']
            a._ref_desc = z[f'd{i}']
            a._ref_shape = (self._be.h, self._be.w)
            roi = z[f'r{i}']
            a._ref_roi = None if not np.isfinite(roi).all() else tuple(
                float(v) for v in roi)
            # No stored image: a preloaded reference can still MATCH, it simply
            # cannot be displayed. Better than refusing to load, and better
            # than inventing a picture.
            a._ref_gray = None
            self._refs.append(a)
            self._yields.append(yields[i] if i < len(yields) else len(a._ref_kpts))
            self._sigs.append(z[f's{i}'])
            self._labels.append(labels[i] if i < len(labels) else '')
            att = z['att'][i] if 'att' in z.files and i < len(z['att']) else None
            self._att.append(None if att is None or not np.isfinite(att).all()
                             else tuple(float(v) for v in att))
            self._depth.append(float(z['depth'][i])
                               if 'depth' in z.files and i < len(z['depth'])
                               else float('nan'))
            q = z['pos'][i] if 'pos' in z.files and i < len(z['pos']) else None
            self._pos.append(None if q is None or not np.isfinite(q).all()
                             else (float(q[0]), float(q[1])))
            raw = (json.loads(str(z['ann'][i]))
                   if 'ann' in z.files and i < len(z['ann']) else {})
            # Already in BACKEND pixels when saved, so no rescale here -- and
            # `load` refuses a foreign resolution outright, which is what makes
            # that safe.
            self._ann.append({k: (float(v[0]), float(v[1]))
                              for k, v in raw.items()} or None)
            added += 1
        return added

    # -- use ---------------------------------------------------------------- #
    def locate(self, gray: np.ndarray, *, label: Optional[str] = None,
               near=None, radius_m: float = 0.0,
               shortlist: Optional[int] = None) -> BankPose:
        """Best-of-shortlist, and say which reference and what it is.

        Best-of rather than a vote or a mean: the references are views of one
        thing from different instants, so the one with the most inliers is the
        one whose viewpoint this frame actually resembles. Averaging their poses
        would produce a position none of them claims -- the same argument
        `lock_state.arbitrate` makes for refusing to blend rungs.

        `label` restricts the search to references of one kind, which is what
        makes a single bank able to hold a prop, a checkpoint and a place at
        once. `None` searches everything, which is the re-identification
        question: not "is this the gate" but "what is this".

        ⭐ ONLY THE TOP-k BY SIGNATURE ARE MATCHED, and k is measured rather
        than configured -- see `shortlist_k()`. This is what makes the bank's
        cost independent of its size: 100 references cost what 5 do.
        """
        import time

        pool = [i for i in range(len(self._refs))
                if label is None or self._labels[i] == label]

        # ⭐ THE LOCALISER AS A PRIOR, and it is the cheapest false-positive
        # filter available. §24 measured the downward camera separating same
        # place from different place -- but its far tail still clears the
        # tracking bar on 17-55 % of pairs, which is why loop closure needs a
        # bar near 100. A vehicle that knows where it is to within a few metres
        # does not have to ask whether this is a place 20 m away: it can refuse
        # to consider it at all.
        #
        # `radius_m` should come from the FILTER'S OWN UNCERTAINTY, not from a
        # constant -- a confident filter gates hard, a drifting one gates
        # loosely and eventually not at all. ⛔ Checkpoints with no stored
        # position are KEPT, never silently dropped: absent is not far away.
        if near is not None and radius_m > 0.0:
            x, y = float(near[0]), float(near[1])
            pool = [i for i in pool
                    if self._pos[i] is None
                    or ((self._pos[i][0] - x) ** 2
                        + (self._pos[i][1] - y) ** 2) <= radius_m ** 2]

        if not pool:
            return BankPose(ok=False)

        # ⭐ DETECT ONCE. Every reference is asked about the SAME frame, so the
        # backbone runs here and the features are handed to each one --
        # `Anchor.locate_features`. Previously each reference detected for
        # itself and the signature detected again, so a 4-wide shortlist ran
        # the backbone five times on one image.
        kq, dq = self._be.detect(gray)

        k = int(shortlist) if shortlist else self.shortlist_k()
        if len(pool) > k:
            q = self._signature(dq)
            # One (N,64) @ (64,) dot product. At 100 references this is
            # microseconds against tens of milliseconds per match, which is the
            # whole reason the shortlist is worth having.
            score = np.stack([self._sigs[i] for i in pool]) @ q
            pool = [pool[j] for j in np.argsort(-score)[:k]]

        best = BankPose(ok=False, searched=len(pool))
        t0 = time.perf_counter()
        for i in pool:
            p = self._refs[i].locate_features(kq, dq)
            n = int(getattr(p, 'inliers', 0) or 0)
            if n > best.inliers:
                best = BankPose(ok=bool(p.ok), inliers=n, index=i, pose=p,
                                label=self._labels[i], searched=len(pool),
                                ref_attitude=self._att[i],
                                ref_depth_m=self._depth[i],
                                ref_position=self._pos[i],
                                points=self._warp(self._ann[i], p))
            if n > self._yields[i]:
                self._yields[i] = n
        # Decaying average of the per-match cost, which is what `shortlist_k`
        # spends. Measured here rather than configured because the same code
        # runs on a dev box and on the Pi and the two differ by 2.2x.
        if pool:
            per = (time.perf_counter() - t0) / len(pool)
            self._match_s = per if self._match_s <= 0.0 else (
                0.8 * self._match_s + 0.2 * per)
        return best

    @staticmethod
    def _att_gap_deg(a, b) -> float:
        """Largest per-axis attitude difference, wrapped. NaN if either is unknown.

        Per-axis rather than a single rotation angle: the axes fail for
        different reasons -- roll is unactuated on this hull, yaw is where the
        board's rev-10 inversion lived -- and a combined magnitude would hide
        which one disagrees.
        """
        if a is None or b is None:
            return float('nan')
        worst = 0.0
        for x, y in zip(a, b):
            if not (np.isfinite(x) and np.isfinite(y)):
                return float('nan')
            d = abs((float(x) - float(y) + 180.0) % 360.0 - 180.0)
            worst = max(worst, d)
        return worst

    def recognise(self, gray: np.ndarray, *, label: Optional[str] = None,
                  det_class: Optional[str] = None,
                  det_age_s: float = float('inf'),
                  attitude=None) -> BankPose:
        """Locate, then ask whether the bank may ASSERT this is that thing.

        ⛔ WHY THIS IS SEPARATE FROM `locate()`. Geometry alone was measured
        unable to tell one prop from another inside one venue: whole-frame
        references from a torpedo run cleared `MIN_INLIERS` on 100 % of GATE
        frames and said 'torpedo' (`measured-bars.md` §22). `locate()` answers
        "where", which is all a rung with a detector-supplied reference needs.
        Asserting "what" needs more, and this is the more.

        THREE CHECKS THAT FAIL FOR UNRELATED REASONS, which is the whole point
        -- turbidity blinds the detector and the matcher together and moves the
        IMU not at all:

          geometry    inliers >= `identity_inliers` (60, measured to admit 0 %
                      of the other-prop frames while keeping 57 % of true ones)
          semantics   a detector hypothesis of the SAME class, recently. ⭐ The
                      detector is the identity authority: it misses things, it
                      does not confuse a gate for a torpedo, so a hypothesis far
                      below the control-confidence gate is still strong evidence
                      of WHAT is in frame while being useless as a position
          kinematics  the vehicle's attitude is within `identity_att_deg` of
                      where the checkpoint was taken

        UNKNOWN IS NOT AGREEMENT. A check whose input is absent -- no detector
        hypothesis passed, no attitude stored -- is skipped and SAID SO in
        `identity_why`, never counted as a pass. A caller that wants a hard
        corroboration requirement can read `identity_why` and refuse; one that
        cannot supply a detector at all still gets the geometry bar.
        """
        p = self.locate(gray, label=label)
        why = []
        ok = True

        if p.inliers < self._id_inliers:
            ok = False
            why.append(f'inliers {p.inliers}<{self._id_inliers}')
        else:
            why.append(f'inliers {p.inliers}')

        if det_class is None:
            why.append('no detector opinion')
        elif det_age_s > self._id_det_age:
            why.append(f'detector {det_age_s:.1f}s stale')
        elif p.label and det_class != p.label:
            ok = False
            why.append(f'detector says {det_class!r}, bank says {p.label!r}')
        else:
            why.append(f'detector agrees ({det_class})')

        gap = self._att_gap_deg(attitude, p.ref_attitude)
        if gap != gap:                                   # NaN -- unknown
            why.append('no attitude')
        elif gap > self._id_att_deg:
            ok = False
            why.append(f'attitude off {gap:.0f}deg')
        else:
            why.append(f'attitude within {gap:.0f}deg')

        p.identity_ok = bool(ok and p.ok)
        p.identity_why = '; '.join(why)
        return p

    @staticmethod
    def _warp(ann: Optional[dict], pose) -> Optional[dict]:
        """Carry the reference's annotated points into the live frame.

        ⭐ THIS IS WHAT MAKES THE BANK A TRAININGLESS DETECTOR. A point marked
        once on a reference photograph -- a hole centre, an aim point -- lands
        in the live frame through the SAME homography the pose came from. No
        model was ever trained to find that point, and it is located even when
        it is itself occluded, because the transform is fitted to the whole
        reference rather than to the point.

        Returns None rather than an empty dict when there is nothing to warp or
        no homography to warp it through: absent and "found none" are different
        answers and a caller must be able to tell them apart.
        """
        if not ann or pose is None or not getattr(pose, 'ok', False):
            return None
        H = getattr(pose, 'H', None)
        if H is None:
            return None
        import cv2
        names = list(ann)
        src = np.array([[ann[n]] for n in names], np.float32)
        dst = cv2.perspectiveTransform(src, np.asarray(H, np.float64))
        return {n: (float(dst[i, 0, 0]), float(dst[i, 0, 1]))
                for i, n in enumerate(names)}

    def corroborate(self, gray: np.ndarray, xyxy, *,
                    label: Optional[str] = None,
                    overlap: float = 0.3) -> BankPose:
        """Does the bank agree that THIS BOX is the thing it remembers?

        ⭐ THE TWO-WAY USE, and it is one mechanism. A detector confidence is a
        semantic opinion with no geometry behind it; the bank's inliers are
        geometry with no semantics. Asking the bank about a specific box gets
        both, and answers two opposite questions at once:

          a WEAK detection the bank corroborates is worth acting on -- the
            prop is really there and the detector was merely unsure, which on
            our own footage is the common case (p50 confidence 0.49)
          a CONFIDENT detection the bank contradicts is a distractor -- the
            failure that steals the vehicle, and the one a confidence
            threshold alone cannot catch at any setting

        ⚠ ITS DISCRIMINATION DEPENDS ENTIRELY ON THE DETECTOR -- §30, §31.
        Scored with a model never trained for the footage, the distractor clip
        beat the true clip (26 vs 20 median inliers) and no threshold
        separated them. With the per-prop model for that footage, the
        distractor drops to a median of **0** against 10 -- a 9.5x separation
        from the same code.

        ⛔ So this is only as good as the box it is handed, and it must not be
        wired into a control path until it has been scored with the model that
        will actually run beside it. Mask-based enrolment was proposed as the
        fix in §30 and MEASURED WORSE (§31): a tight prop box leaves too little
        to segment.

        ⛔ IT DOES NOT INVENT A FUSED SCORE. There is no principled way to
        combine "0.31 confident" with "62 inliers" into one number, and a
        fabricated one would be acted on as though it meant something. The
        caller gets the inlier count, the overlap, and a verdict, and decides.

        `overlap` is the fraction of the bank's matched quad that must fall
        inside the box for them to be talking about the same object. Without
        it, a match anywhere in frame would corroborate a box anywhere else.
        """
        import cv2
        p = self.locate(gray, label=label)
        p.identity_why = ''
        if not p.ok or p.pose is None or p.pose.corners is None:
            p.identity_ok = False
            p.identity_why = f'bank has no match (inliers {p.inliers})'
            return p

        # The box arrives in FULL-FRAME pixels; the quad is in backend pixels.
        fh, fw = gray.shape[:2]
        sx, sy = self._be.w / float(fw), self._be.h / float(fh)
        x1, y1, x2, y2 = (float(v) for v in xyxy)
        bx = (min(x1, x2) * sx, min(y1, y2) * sy,
              max(x1, x2) * sx, max(y1, y2) * sy)
        quad = np.asarray(p.pose.corners, np.float32).reshape(-1, 2)
        inside = ((quad[:, 0] >= bx[0]) & (quad[:, 0] <= bx[2])
                  & (quad[:, 1] >= bx[1]) & (quad[:, 1] <= bx[3]))
        frac = float(inside.mean()) if len(quad) else 0.0

        ok = p.inliers >= self._id_inliers and frac >= overlap
        p.identity_ok = bool(ok)
        p.identity_why = (f'inliers {p.inliers}'
                          f'{"" if p.inliers >= self._id_inliers else f"<{self._id_inliers}"}; '
                          f'quad {frac:.0%} inside the box'
                          f'{"" if frac >= overlap else f" <{overlap:.0%}"}')
        return p

    def verify(self, gray: np.ndarray, *,
               label: Optional[str] = None) -> int:
        """The LTMU verifier: how much evidence says this is still the target.

        Returns an inlier count, and **0 when the bank cannot stand behind an
        answer** -- an empty bank, a refused match, a backend that found
        nothing. A caller comparing against `min_inliers` then gets the same
        semantics everywhere, and no path here invents support.
        """
        p = self.locate(gray, label=label)
        return int(p.inliers) if p.ok else 0
