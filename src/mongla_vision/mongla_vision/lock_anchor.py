"""The anchor rung: enrol a reference or locate against the bank, and box what it found.

Runs on the ladder thread when detection has lost the target.

Mixed into `LockNode`.
"""
import numpy as np

from mongla_vision.anchor import health as _health


# The bar for healing the follower from the anchor. Deliberately ABOVE the
# anchor's own trust bar (100): a box good enough to report is not
# automatically good enough to restart the fast rung on, because a reseeded
# follower then carries FULL authority for the rest of the gap. Set at the
# identity bar the re-ID work uses for "this is the same object", which is the
# strictest claim any rung in this package makes.
_ANCHOR_RESEED_INLIERS = 120


class AnchorMixin:
    """Methods of `LockNode`; see `lock_node.py` for the node."""

    def _enrol_or_locate(self, f) -> bool:
        """The anchor bank: enrol a checkpoint, or locate against the bank.

        Returns True when a locate ran this tick (the pose is fresh)."""
        gray, header, now = f.gray, f.header, f.now
        det_header, det_box, det_conf, det_cls = (f.det_header, f.det_box,
                                                  f.det_conf, f.det_cls)
        anchor_ran = False
        # The defect this replaced. The guard here used to be
        # `not self._anchor.has_reference`, so the reference was
        # snapped ONCE and never refreshed. Measured 2026-09-24 on
        # the archive clips, inliers against that frozen reference:
        #
        #   clip              +1 s  +3 s  +5 s  +8 s
        #   mirpur_torpedo     189    65   134    24
        #   mirpur_torpedo_1    69    43    51    12  <- under the bar
        #   mirpur_gate        195    36   237    31
        #
        # Every clip's minimum is its +8 s column. A reference decays,
        # and the same frames against a FRESH reference are the +1 s
        # column -- so this was never a limit of the descriptor.
        #
        # The bank takes a new checkpoint when the best stored one has
        # decayed toward the trust bar, judged on the inlier count the
        # 3 Hz `locate()` below already produced. Asking
        # `wants_refresh()` here instead would run a SECOND full match
        # per frame to learn what we just measured.
        stale = (self._anchor.has_reference
                 and self._anchor_inliers < self._anchor._refresh)
        # AND the criterion that matters more, per §21.2: the bank
        # buys VIEWPOINT DIVERSITY, not freshness. A target whose
        # apparent size is outside the band the bank covers earns a
        # slot even while the existing references are matching fine
        # -- because they will stop matching when the vehicle closes
        # in, and that is the frame a torpedo actually fires from.
        # Seen on footage: a bank enrolled at mid-range refused every
        # close-range frame at 7 inliers against a board filling the
        # screen.
        uncovered = (det_box is not None
                     and not self._anchor.covers_scale(gray, det_box))
        # And the reason the bank was staying empty.
        # `stale` only fires when the match is BAD -- but when it is
        # bad the target may already be lost, and when it is good
        # nothing is ever added, so the bank never grows. Measured on
        # Mirpur torpedo footage: the stale-only policy enrolled ONE
        # reference and asserted identity on 0 % of later frames,
        # while a bank of 14 asserted on 3 of 8.
        #
        # Hoarding is free here and we built the machinery that makes
        # it so: §20 measured search cost FLAT in bank size (100
        # references cost what 10 do, via the signature shortlist) and
        # 64 references is 16 MB. Eviction by inlier yield keeps the
        # quality bar. So while there is room, every confident
        # detection earns a slot -- diversity then emerges from the
        # run instead of being predicted by a heuristic.
        room = self._anchor.size < self._anchor._cap
        # Throttled, and not for tidiness. `enrol()` runs a full
        # `locate()` to decide whether a candidate agrees with the
        # bank, and a candidate that keeps being REFUSED leaves
        # `stale` true -- so an unthrottled path retries that match on
        # every detection frame. Measured cost of one match on the Pi
        # is 31.4 ms and the shortlist is several, which lands
        # ~124 ms of matching inside the loop that publishes /lock,
        # against a measured 53.6 Hz floor. The refusal case is
        # exactly when the vehicle can least afford it.
        #
        # One attempt per anchor period: enrolling faster than the
        # bank is evaluated cannot help, because nothing has changed
        # its mind in between.
        may_try = now >= self._anchor_enrol_next
        # A falling trend is a reason to enrol, alongside stale /
        # uncovered / room. It is the only one of the four that is
        # PREDICTIVE: the others describe the bank, this describes
        # where the view is heading. It licenses nothing else --
        # it never claims the target is gone.
        falling = False
        if len(self._anchor_trend) >= _health.TREND_MIN_SAMPLES:
            tr = _health.trend(self._anchor_trend)
            if tr.prepare:
                falling = True
                self._prepared += 1
                if self._prepared == 1 or self._prepared % 20 == 0:
                    self.get_logger().info(
                        f'[LOCK ] anticipating a loss: {tr.reason}')
        if det_box is not None and may_try and (
                falling or
                not self._anchor.has_reference or stale or uncovered
                or room):
            self._anchor_enrol_next = now + self._anchor_period
            # The frame the box belongs to, not merely the newest.
            # This patch becomes the object model, so pairing it with
            # the wrong frame scales every range that follows.
            snap_gray = self._frame_for(det_header)
            if snap_gray is None:
                # The matching frame has aged out of the ring, or a
                # stamp is missing. Snapping anyway would bake in the
                # skew silently; waiting costs one detection, because
                # the next one arrives with its frame still in hand.
                if not self._snap_skew_warned:
                    self._snap_skew_warned = True
                    self.get_logger().warn(
                        '[LOCK ] anchor snap deferred: no frame '
                        'matching the detection stamp. Check '
                        'use_sim_time and the detector latency -- '
                        'the reference patch defines the object '
                        'model, so it is not snapped against a '
                        'frame it does not belong to.')
            else:
                r = self._anchor.enrol(snap_gray, roi=det_box,
                                       det_conf=det_conf,
                                       label=det_cls)
                if r.accepted:
                    # A fresh checkpoint resets the staleness measure:
                    # the number that triggered this refresh described
                    # the reference we have just replaced.
                    self._anchor_inliers = self._anchor._refresh
                    self._anchor_enrolled += 1
                    self._anchor_label = det_cls
                    # Logged because a bank that silently stops
                    # enrolling looks exactly like one that never
                    # needed to, and those are opposite faults.
                    self.get_logger().info(
                        f'[LOCK ] checkpoint {self._anchor_enrolled}: '
                        f'{r.keypoints} kp, bank {self._anchor.size}/'
                        f'{self._anchor._cap}, '
                        f'{"scale" if uncovered else ("room" if room else "quality")}')
                elif r.reason != 'confidence':
                    self.get_logger().debug(
                        f'[LOCK ] checkpoint refused: {r.reason}')
        elif (self._anchor.has_reference
              and now >= self._anchor_next):
            self._anchor_next = now + self._anchor_period
            bp = self._anchor.locate(
                gray, label=self._anchor_label or None)
            # The bank answers best-of-bank; `pose` is the ordinary
            # AnchorPose the rest of this node already consumes, so
            # nothing downstream learns that there is now more than
            # one reference.
            # The per-reference yields, which used to be thrown
            # AWAY. A fouled port collapses the match against EVERY
            # reference at once; a target that left collapses only
            # the one that used to win. Both arrive at the ladder as
            # "the anchor is not matching" and need OPPOSITE
            # responses -- stop trusting vision, or go and search.
            # `bp` alone cannot carry that: it is a statement about
            # the whole shortlist.
            self._note_anchor_health()
            # ANTICIPATE, do not merely react. A falling match
            # quality predicts the next seconds will be worse, and
            # the reference worth having is the one from BEFORE the
            # detection is lost. Snapping then is cheap and
            # reversible; snapping after costs the very view it
            # needed.
            self._anchor_trend.append(int(bp.inliers))
            self._anchor_pose = bp.pose
            self._anchor_inliers = int(bp.inliers)
            self._anchor_best = bp.index
            anchor_ran = True
            # The frame this pose was fitted to. The anchor runs at
            # 3 Hz while this loop runs at frame rate, so between
            # evaluations the pose below is REUSED -- up to 333 ms old
            # -- and publishing it under the current frame's stamp
            # would report a third of a second of staleness as ~18 ms.
            # That defeats the freshness machinery precisely on the
            # rung it exists to protect, and only on that rung, so it
            # looks correct whenever a detection is present.
            self._anchor_header = header
        return anchor_ran

    def _anchor_box(self, gray, anchor_ran: bool):
        """The anchor's box and confidence, and the follower reseeded from it."""
        ab = ac = None
        p = self._anchor_pose
        # EVERY fit publishes its decomposition, including the
        # ones with no usable corner quad -- theta and scale are
        # fitted from the same inliers and are valid whether or not
        # the warped quad is. Gating them on `corners` would discard
        # the signal exactly when the box is least trustworthy.
        if anchor_ran:
            self._publish_decomposition(p, self._anchor_header)
        if p is not None and p.ok and p.corners is not None:
            q = np.asarray(p.corners, np.float32)
            ab = (float(q[:, 0].min()), float(q[:, 1].min()),
                  float(q[:, 0].max()), float(q[:, 1].max()))
            ac = p.confidence
            # The ladder heals upward, not only downward.
            #
            # Until this, `_follower.reset()` was called on an
            # accepted DETECTION and nowhere else. So when LK's
            # points died mid-gap -- which is exactly what the
            # survival and forward-backward gates exist to detect --
            # the fast ~50 Hz rung stayed dead until a detection
            # came back, even though the anchor was still finding
            # the target every 222 ms. The ladder had a one-way
            # ratchet: every rung could fall, none could climb.
            #
            # The anchor's box is a legitimate seed. It is fitted
            # frame-to-REFERENCE, so unlike a follower box it has
            # not drifted, and `goodFeaturesToTrack` inside it finds
            # the same corners LK wanted. Reseeding costs one GFTT
            # call (measured 8.0 ms for the whole LK step) and buys
            # back the 50 Hz rung for the rest of the gap.
            #
            # Only when the follower is dead and the anchor is
            # CONFIDENT. Reseeding a LIVE follower would replace a
            # frame-fresh box with one up to 222 ms old, and
            # reseeding from a weak fit would restart the fast rung
            # on a bad box and give it full authority -- exactly the
            # failure the acting bar exists to prevent, arriving by
            # another door.
            if (self._follower is not None
                    and not self._follower.active
                    and anchor_ran
                    and int(self._anchor_inliers)
                    >= _ANCHOR_RESEED_INLIERS):
                n = self._follower.reset(gray, ab)
                if n:
                    self._reseeds += 1
                    if self._reseeds in (1, 10, 100):
                        self.get_logger().info(
                            f'[LOCK ] follower reseeded from the '
                            f'anchor ({n} pts, {self._anchor_inliers} '
                            f'inliers) -- reseed #{self._reseeds}')
        return ab, ac
