"""LockNode's image and detection inputs: by topic, or by reference when composed.

The frame and the detection are what the ladder ticks on, so they are kept
together: `_on_img` and `submit_frame` hand a frame in, `_on_det` takes the
detector's answer and pairs it with the frame it was computed on.
"""
import cv2

from mongla_vision.stamps import capture_monotonic
from mongla_vision import image_msg


def _stamp_key(header):
    """A hashable identity for a header stamp, or None when it carries none.

    Compared as (sec, nanosec) integers rather than a float: a float seconds
    value loses nanosecond resolution at ROS epoch magnitudes, so two different
    frames can compare equal -- which would silently pair the wrong ones, the
    exact failure this exists to prevent.
    """
    if header is None:
        return None
    st = header.stamp
    if st.sec == 0 and st.nanosec == 0:
        return None
    return (int(st.sec), int(st.nanosec))



# Strong detections a lock needs before a weak box may hold it. One false
# strong box was what HELD extended on gate-free water (measured-bars section
# 72); two in one lock is a target that was seen twice.
HELD_MIN_CONFIRMS = 2

class LockFramesMixin:
    """Methods of `LockNode`; see `lock_node.py` for the node."""

    def _on_img(self, msg):
        try:
            g = cv2.cvtColor(image_msg.to_array(msg, 'bgr8'),
                             cv2.COLOR_BGR2GRAY)
        except Exception as exc:                          # noqa: BLE001
            # Counted and said, never swallowed: a camera publishing an
            # encoding this cannot convert starves the whole ladder, and the
            # only symptom was a lock that never formed. (Cloud audit, 2026-11-07.)
            self._n_bad_frames = getattr(self, '_n_bad_frames', 0) + 1
            if self._n_bad_frames in (1, 10, 100) or self._n_bad_frames % 1000 == 0:
                self.get_logger().error(
                    f'[LOCK ] frame {self._n_bad_frames} undecodable '
                    f'({msg.encoding!r}: {exc}) -- the ladder gets no image')
            return
        with self._lock:
            self._gray = g
            self._header = msg.header
            # A short history, so a detection can be paired with the frame it
            # was actually computed on. See `_frame_for`. Four frames at the
            # anchor backend's resolution is ~300 kB; the detector's own
            # capture->arrival latency is 32 ms median / 48 p95, which is one
            # to two frames, so four is the p95 with headroom and no more.
            self._recent.append((_stamp_key(msg.header), g))
        self._fresh.set()

    # -- composed intake (`direct_feed`) ------------------------------------ #
    def wants_frame(self) -> bool:
        """Every frame: the stamp history must hold the frame a detection was
        computed on, or `_frame_for` refuses the snap. The camera decodes
        every frame for its own publish slot anyway."""
        return True

    def submit_frame(self, frame_bgr, header) -> None:
        """A decoded BGR frame from the camera in this process, by reference.

        No copy and no grey conversion here: this runs on the camera's capture
        thread, at camera rate. The ladder converts on its own thread, once,
        only for the frame it actually uses; `_frame_for` converts a history
        entry only when a snap needs it. The camera decodes a NEW array per
        frame, so holding the reference is safe.
        """
        with self._lock:
            self._pending = frame_bgr
            self._header = header
            self._recent.append((_stamp_key(header), frame_bgr))
        self._fresh.set()

    def _gray_now(self):
        """The newest frame in grey, converting a pending one on demand.

        For readers outside the ladder's tick (timers). The ladder converts
        only on ticks that read the frame, so `self._gray` alone can be a
        frame or more old.
        """
        with self._lock:
            pending, self._pending = self._pending, None
        if pending is not None:
            self._gray = cv2.cvtColor(pending, cv2.COLOR_BGR2GRAY)
        return self._gray

    def submit_info(self, info) -> None:
        """CameraInfo straight from the camera; `_on_info` drops repeats."""
        self._on_info(info)

    def _frame_for(self, det_header):
        """The frame a detection was computed on, not merely the newest one.

        What this fixes. `snap` defines the anchor's reference patch, and
        that patch IS the object model: its width is taken to be the target's
        true width, so every later range is scaled by it. It was snapped from
        the CURRENT frame using the LAST detection's box, with nothing
        comparing the two stamps -- and the detector's box is 32 ms median /
        48 p95 behind the frame in hand. On a moving hull that box no longer
        bounds the target in the frame it is applied to, so the reference patch
        clips the target or swallows background, and the error is baked into
        the model for the whole lock.

        Matching the stamp removes the error rather than bounding it, which is
        affordable HERE and only here: snapping happens once, so waiting for
        the right frame costs nothing, while the steering rungs must act on
        whatever they have and use freshness decay instead.
        """
        key = _stamp_key(det_header)
        if key is None:
            return None
        for k, g in self._recent:
            if k == key:
                # Composed intake stores BGR; it turns grey only here, for
                # the one frame a snap needs.
                return g if g.ndim == 2 else cv2.cvtColor(
                    g, cv2.COLOR_BGR2GRAY)
        return None

    # A floor height older than this is not this vehicle's altitude any more.
    _FLOOR_H_MAX_AGE_S = 2.0

    def _on_det(self, msg):
        best = None
        best_name = ''
        weak = []
        for d in msg.detections:
            if not d.results:
                continue
            h = d.results[0]
            score = float(h.hypothesis.score if hasattr(h, 'hypothesis')
                          else h.score)
            name = str(h.hypothesis.class_id if hasattr(h, 'hypothesis')
                       else h.id)
            if self._cls and name != self._cls:
                continue
            # An acting bar, not merely "not zero" (B-59). This accepted the
            # highest-scoring box with score > 0 as THE target, at full
            # authority. Measured on real Mirpur footage, the now-RETIRED
            # `gate_rescue_repair` graph (not `gate_sharks`, which replaced
            # it on 2026-09-28 and against which this bar has NOT been
            # re-derived -- see measured-bars section 59)
            # claims a gate on 90.8 % of GATE-FREE frames at conf 0.15 --
            # half-frame boxes over empty turquoise at 0.44-0.56, confirmed by
            # rendering them. Separation between gate-present and gate-absent
            # footage is +0.0 points at 0.15 and at 0.30, and +74.0 at 0.60.
            #
            # With no bar here the ladder locked onto open water: the follower
            # seeded on it, the anchor enrolled against it, and the vision
            # verbs drove the hull at it -- section 8.6's failure exactly.
            #
            # The bar lives HERE and not only in the detector because the
            # detector's `conf` is deliberately permissive: the tracker needs
            # low-scoring boxes to maintain association across a gap. What may
            # be ASSOCIATED and what may be ACTED ON are different questions,
            # and conflating them turned a 0.15 detector bar into a 0.15
            # control bar.
            if score < self._act_conf:
                weak.append((score, d.bbox))
                continue
            if best is None or score > best[0]:
                best_name = name
                b = d.bbox
                hw, hh = b.size_x * 0.5, b.size_y * 0.5
                best = (score, (b.center.position.x - hw,
                                b.center.position.y - hh,
                                b.center.position.x + hw,
                                b.center.position.y + hh))
        # The decay clock. `_det_t` is what `arbitrate` measures authority
        # against, so it must be when the frame was SEEN, not when the message
        # landed here. Arrival time under-reports every gap by the whole
        # capture->inference->transport chain -- measured at 32 ms median,
        # 48 p95 -- always in the flattering direction, so the ladder holds
        # full authority slightly past the point the measurement justifies.
        t, why = capture_monotonic(msg.header)
        if why and not self._stamp_warned:
            self._stamp_warned = True
            self.get_logger().warn(
                f'[LOCK ] detection {why} -- the authority decay is measuring '
                f'age since ARRIVAL, not since capture. Check use_sim_time.')
        if best is not None:
            # Counted per lock; `_publish` zeroes it when the lock is lost.
            self._confirms = getattr(self, '_confirms', 0) + 1
        held = None if best is not None else self._held_from(weak)
        with self._lock:
            self._held_box, self._held_conf = (None, 0.0) if held is None else held
            if held is not None:
                self._held_t, self._held_header = t, msg.header
            if 0.0 < self._det_msg_t < t:
                dt = t - self._det_msg_t
                # EMA over ~10 messages: one dropped frame should not swing
                # the window, a model swap to a slower detector should.
                self._det_dt = (dt if self._det_dt is None
                                else 0.9 * self._det_dt + 0.1 * dt)
            self._det_msg_t = t
            if best is None:
                self._det_box, self._det_conf = None, 0.0
            else:
                self._det_conf, self._det_box = best
                self._det_cls = best_name
                self._det_t = t
                self._det_header = msg.header

    def _held_from(self, weak):
        """The weak box nearest the lock, if one is near enough, as (box, score).

        A box under the acting bar may keep a target and never start one, so
        it is judged against where the lock IS: inside the prior region the
        detector's own floor is enough, outside it the acting bar still
        applies (`visibility.region_conf_bar`, with the lock's own centre as
        the prior). The region is a quarter of the frame width -- 160 px at
        640, the radius at which 84 % of real weak boxes and 28 % of random
        points fell near the last sighting (measured-bars section 56). With
        no lock, every box gets the acting bar.
        """
        # getattr: `_on_det` can run before __init__ has set the ladder's
        # state (the subscription exists first; see lock_wiring).
        prior = getattr(self, '_lock_centre', None)
        gray = getattr(self, '_gray', None)
        if (prior is None or gray is None or not weak
                or getattr(self, '_confirms', 0) < HELD_MIN_CONFIRMS):
            return None
        from .tracking.visibility import region_conf_bar
        w = float(gray.shape[1])
        best = None
        for score, b in weak:
            cx, cy = b.center.position.x, b.center.position.y
            bar = region_conf_bar(prior, cx, cy, w, inside_bar=0.0,
                                  outside_bar=self._act_conf)
            if score < bar:
                continue
            dist = (cx - prior[0]) ** 2 + (cy - prior[1]) ** 2
            if best is None or dist < best[0]:
                hw, hh = b.size_x * 0.5, b.size_y * 0.5
                best = (dist, (cx - hw, cy - hh, cx + hw, cy + hh), score)
        return None if best is None else (best[1], best[2])

    # -- the ladder --------------------------------------------------------- #
