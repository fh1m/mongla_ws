"""Asking the bank where the vehicle is: locate, recognise, corroborate, verify.

Each answers from the enrolled references and returns a `BankPose` or None.

Mixed into `CheckpointBank`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .anchor import AnchorPose


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


class LookupMixin:
    """Methods of `CheckpointBank`; see `bank.py` for the node."""

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

        ONLY THE TOP-k BY SIGNATURE ARE MATCHED, and k is measured rather
        than configured -- see `shortlist_k()`. This is what makes the bank's
        cost independent of its size: 100 references cost what 5 do.
        """
        import time

        pool = [i for i in range(len(self._refs))
                if label is None or self._labels[i] == label]

        # The localiser as a prior, and it is the cheapest false-positive
        # filter available. §24 measured the downward camera separating same
        # place from different place -- but its far tail still clears the
        # tracking bar on 17-55 % of pairs, which is why loop closure needs a
        # bar near 100. A vehicle that knows where it is to within a few metres
        # does not have to ask whether this is a place 20 m away: it can refuse
        # to consider it at all.
        #
        # `radius_m` should come from the FILTER'S OWN UNCERTAINTY, not from a
        # constant -- a confident filter gates hard, a drifting one gates
        # loosely and eventually not at all. Checkpoints with no stored
        # position are KEPT, never silently dropped: absent is not far away.
        if near is not None and radius_m > 0.0:
            x, y = float(near[0]), float(near[1])
            pool = [i for i in pool
                    if self._pos[i] is None
                    or ((self._pos[i][0] - x) ** 2
                        + (self._pos[i][1] - y) ** 2) <= radius_m ** 2]

        if not pool:
            return BankPose(ok=False)

        # Detect once. Every reference is asked about the SAME frame, so the
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
        # Kept, not maxed away. The per-reference yields of ONE lookup are
        # what separates a fouled lens from a changed world: a dirty port
        # collapses every reference at once, a moved target collapses only the
        # one that used to win. `best` cannot carry that -- it is a statement
        # about the whole shortlist -- and the numbers were being computed and
        # discarded on every lookup. See `anchor/health.py`.
        self._last_scores = {}
        for i in pool:
            p = self._refs[i].locate_features(kq, dq)
            n = int(getattr(p, 'inliers', 0) or 0)
            self._last_scores[i] = n
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

    def last_lookup_scores(self):
        """(current, best) per reference from the last `locate()`.

        Returned as two aligned lists over the references that were ACTUALLY
        searched, because the shortlist means most lookups touch a handful.
        Judging a reference that was never asked would count a silence as a
        collapse, which is how a healthy camera gets reported as fouled.
        """
        idx = sorted(self._last_scores)
        return ([self._last_scores[i] for i in idx],
                [self._yields[i] for i in idx if i < len(self._yields)])

    @staticmethod
    def _att_gap_deg(a, b) -> float:
        """Largest per-axis attitude difference, wrapped. NaN if either is unknown.

        Per-axis rather than a single rotation angle: the axes fail for
        different reasons -- roll is unactuated on the five-thruster CAD, yaw is where the
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

        Why this is separate from `locate()`. Geometry alone was measured
        unable to tell one prop from another inside one venue: whole-frame
        references from a torpedo run cleared `MIN_INLIERS` on 100 % of GATE
        frames and said 'torpedo' (`measured-bars.md` §22). `locate()` answers
        "where", which is all a rung with a detector-supplied reference needs.
        Asserting "what" needs more, and this is the more.

        Three checks that fail for unrelated reasons, which is the whole point
        -- turbidity blinds the detector and the matcher together and moves the
        IMU not at all:

          geometry    inliers >= `identity_inliers` (60, measured to admit 0 %
                      of the other-prop frames while keeping 57 % of true ones)
          semantics   a detector hypothesis of the SAME class, recently. The
                      detector is the identity authority: it misses things, it
                      does not confuse a gate for a torpedo, so a hypothesis far
                      below the control-confidence gate is still strong evidence
                      of WHAT is in frame while being useless as a position
          kinematics  the vehicle's attitude is within `identity_att_deg` of
                      where the checkpoint was taken

        Unknown is not agreement. A check whose input is absent -- no detector
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

        This is what makes the bank a trainingless detector. A point marked
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

        The two-way use, and it is one mechanism. A detector confidence is a
        semantic opinion with no geometry behind it; the bank's inliers are
        geometry with no semantics. Asking the bank about a specific box gets
        both, and answers two opposite questions at once:

          a WEAK detection the bank corroborates is worth acting on -- the
            prop is really there and the detector was merely unsure, which on
            our own footage is the common case (p50 confidence 0.49)
          a CONFIDENT detection the bank contradicts is a distractor -- the
            failure that steals the vehicle, and the one a confidence
            threshold alone cannot catch at any setting

        Its discrimination depends entirely on the detector -- §30, §31.
        Scored with a model never trained for the footage, the distractor clip
        beat the true clip (26 vs 20 median inliers) and no threshold
        separated them. With the per-prop model for that footage, the
        distractor drops to a median of **0** against 10 -- a 9.5x separation
        from the same code.

        So this is only as good as the box it is handed, and it must not be
        wired into a control path until it has been scored with the model that
        will actually run beside it. Mask-based enrolment was proposed as the
        fix in §30 and MEASURED WORSE (§31): a tight prop box leaves too little
        to segment.

        It does not invent a fused score. There is no principled way to
        combine "0.31 confident" with "62 inliers" into one number, and a
        fabricated one would be acted on as though it meant something. The
        caller gets the inlier count, the overlap, and a verdict, and decides.

        `overlap` is the fraction of the bank's matched quad that must fall
        inside the box for them to be talking about the same object. Without
        it, a match anywhere in frame would corroborate a box anywhere else.
        """
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
