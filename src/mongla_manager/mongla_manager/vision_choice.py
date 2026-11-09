"""Which box the control loop steers on, and the Sample it is turned into.

The choice in one place: the largest matching box, or with the continuity
lock the one nearest the last aim; with coasting, the tracker's prediction
through a detection gap; with the lock ladder, its box when nothing live is.
`VisionState` (vision_state.py) caches the topics; this reads that cache.
"""
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from vision_msgs.msg import Detection2D


@dataclass
class Sample:
    """One snapshot of where the largest target sits in the frame.

    All values are normalized to [-1, +1] for ex/ey, [0, 1] for h_frac,
    so the controller math is camera-resolution-agnostic. The control
    loop reads /detections directly (the same topic the HUD shows), so a
    box visible on screen is a box the controller acts on.
    """
    ex:       float    # horizontal error: -1=left edge, 0=centre, +1=right edge
    ey:       float    # vertical error:   -1=top  edge, 0=centre, +1=bottom edge
    h_frac:   float    # bbox height as fraction of image height (0..1)
    w_frac:   float    # bbox width  as fraction of image width  (0..1)
    age_s:    float    # time since the last REAL detection of this target (s)
    class_id: str
    score:     float
    vis_range: float = 0.0         # monocular depth estimate from depth_estimation_node (0=far, 1=close)
    track_id:  int   = -1          # tracker id of this target (-1 = unknown / coast off)
    coasted:   bool  = False       # True = tracker-predicted box during a detection gap (no live box)


class ChoiceMixin:
    """Methods of `VisionState`; see vision_state.py for the subscriptions."""

    def _lock_sample(self, image_width: float, image_height: float,
                     class_name: str = ''):
        """A Sample from the ladder, or None.

        Identity first (#74). The box's hypothesis names the RUNG (detection,
        follow, anchor); the physical class it follows rides in
        `Detection2D.id`, set by `lock_node`. A loop that asked for one class
        never steers on a ladder holding another, and a box that does not say
        what it is (an older lock_node, an empty id) is refused for the same
        reason: an unverifiable identity is not a match.

        The ladder has ALREADY applied its own decay -- `score` is rung trust
        times time-since-the-last-real-detection -- so this must not decay it
        again. Double-decaying would make the fallback die roughly twice as
        fast as designed, which is the same arithmetic error that made the
        uplink's coasted target expire inside 0.4 s.
        """
        with self._lock:
            arr = self._lock_array
            stamp = self._lock_stamp
        if arr is None or not arr.detections:
            return None
        d = arr.detections[0]
        if not d.results:
            return None
        if class_name and d.id != class_name:
            return None
        score = _hypothesis_score(d)
        if score <= 0.0:
            return None
        b = d.bbox
        cx, cy = b.center.position.x, b.center.position.y
        return Sample(
            ex=(cx - image_width * 0.5) / (image_width * 0.5),
            ey=(cy - image_height * 0.5) / (image_height * 0.5),
            h_frac=float(b.size_y) / max(image_height, 1.0),
            w_frac=float(b.size_x) / max(image_width, 1.0),
            age_s=max(0.0, time.monotonic() - stamp) if stamp else 0.0,
            class_id=d.id or _hypothesis_class_id(d), score=score,
            vis_range=0.0, track_id=-1, coasted=True)

    def bbox_error(self, class_name: str = '', *,
                   near: Optional[Tuple[float, float]] = None,
                   gate_norm: float = 0.0,
                   min_score: float = 0.0,
                   locked_id: int = -1,
                   coast_s: float = 0.0,
                   lock_s: float = 0.0) -> Optional[Sample]:
        """Pick a matching detection and return a normalized Sample.

        Default (``near=None`` or ``gate_norm<=0``): the LARGEST-area matching
        box, exactly as before. With ``near=(ex,ey)`` and ``gate_norm>0``
        (the continuity lock): among matching boxes within ``gate_norm`` of
        ``near`` in normalized centre space, the one NEAREST ``near`` -- so a
        second hole / spurious box can't steal the aim once a target is locked;
        if none are inside the gate, returns None (a transient loss the control
        loop rides on its grace timer). ``min_score`` drops boxes below that
        detection score from consideration (control-side conf floor).

        Coast layer (OPT-IN, ``coast_s>0``): when NO live ``/detections`` box
        matches, fall back to the tracker's coasted (Kalman-predicted) box of
        ``locked_id`` from ``/tracks`` -- but ONLY that id, and ONLY while the
        gap is shorter than ``coast_s``. The returned Sample carries
        ``coasted=True`` and ``age_s`` = the TRUE time since the last real
        detection (not message age), so the control loop's freshness/coast
        decay reduces its authority and the grace timer still fires LOST. A
        live detection ALWAYS wins (this method tries it first); coast never
        gates out or overrides a real box. ``coast_s=0`` (default) ⇒ behaviour
        is byte-identical to the no-coast path. See the prior-bug note in
        BUGS.md (predicted boxes must not be conf-gated as the locked id).

        Returns None when no qualifying detection is cached, or before the
        first CameraInfo arrives (image size still (0,0)) so the control
        loop never steers on a mis-scaled pixel error.
        """
        with self._lock:
            detections_array     = self._latest_array
            image_width, image_height = self._image_size
            sampled_at_monotonic = self._latest_stamp
            vis_range_vals       = self._vis_range_vals
            tracks_array         = self._latest_tracks

        if detections_array is None:
            return None
        if image_width <= 0 or image_height <= 0:
            return None

        half_w = image_width  * 0.5
        half_h = image_height * 0.5
        use_near = near is not None and gate_norm > 0.0

        # One pass: per candidate compute a selection metric -- nearest-to-`near`
        # (continuity lock) or largest-area (default). Track the winner's index
        # for the parallel vis_range lookup.
        best_detection: Optional[Detection2D] = None
        best_metric:    Optional[float] = None
        best_index:     int   = 0
        for idx, det in enumerate(detections_array.detections):
            if class_name and not _hypothesis_matches(det, class_name):
                continue
            if min_score > 0.0 and _hypothesis_score(det) < min_score:
                continue
            if use_near:
                cx, cy = _bbox_center(det.bbox)
                ex = (cx - half_w) / half_w
                ey = (cy - half_h) / half_h
                dist = math.hypot(ex - near[0], ey - near[1])
                if dist > gate_norm:
                    continue
                metric = -dist                      # nearest wins
            else:
                metric = float(det.bbox.size_x) * float(det.bbox.size_y)  # largest
            if best_metric is None or metric > best_metric:
                best_metric    = metric
                best_detection = det
                best_index     = idx
        if best_detection is None:
            # No live detection this tick. Coast the locked target's predicted
            # box (opt-in) before declaring a loss -- the gap-bridging path.
            if coast_s > 0.0 and locked_id >= 0:
                cs = self._coast_sample(class_name, locked_id, coast_s,
                                        tracks_array, image_width, image_height)
                if cs is not None:
                    return cs
            # LAST rung before declaring nothing: the ladder (follower /
            # anchor). Opt-in, and it CANNOT fabricate -- `lock_node` publishes
            # nothing once its own authority reaches zero, so an absent message
            # is the loss being declared on schedule rather than hidden.
            # `lock_s` is a switch, not a duration, and its name says
            # otherwise. Any positive value enables the ladder; the magnitude
            # is never compared to anything, so `lock_s:=2.5` and `lock_s:=0.1`
            # behave identically.
            #
            # That is deliberate and correct: the ladder OWNS the horizon. It
            # decays its own authority from the last real detection and stops
            # publishing at zero, so a second numeric horizon here would be the
            # same quantity in two places -- and the shorter of the two would
            # silently win. The name is the defect, not the behaviour.
            if lock_s > 0.0:
                return self._lock_sample(image_width, image_height, class_name)
            return None

        vis_range = (float(vis_range_vals[best_index])
                     if best_index < len(vis_range_vals) else 0.0)

        detection = best_detection
        center_x, center_y = _bbox_center(detection.bbox)
        bbox_height_frac   = float(detection.bbox.size_y) / float(image_height)
        bbox_width_frac    = float(detection.bbox.size_x) / float(image_width)

        # Normalize to [-1, +1]. center_y > height/2 (target lower in image)
        # -> positive vertical_error, consistent with image-coordinates
        # (y grows downward).
        horizontal_error = (center_x - image_width  * 0.5) / (image_width  * 0.5)
        vertical_error   = (center_y - image_height * 0.5) / (image_height * 0.5)

        # Clamp to [-1.5, +1.5] so a bbox that drifts outside the frame
        # doesn't spike the controller. (Real bboxes can extend slightly
        # past the image edge after NMS.)
        horizontal_error = max(-1.5, min(1.5, horizontal_error))
        vertical_error   = max(-1.5, min(1.5, vertical_error))

        class_id  = _hypothesis_class_id(detection)
        score     = _hypothesis_score(detection)

        # Coast bookkeeping (only when enabled): tag this live box with its
        # tracker id (matched from /tracks by centre) and record the real-sighting
        # time so a later coast knows the true gap. Pure no-op when coast_s=0.
        track_id = -1
        if coast_s > 0.0:
            track_id = self._match_track_id(
                horizontal_error, vertical_error, tracks_array,
                image_width, image_height)
            if track_id >= 0:
                with self._lock:
                    # `sampled_at_monotonic`, NOT `time.monotonic()`. This is
                    # the sighting time a later coast measures its gap from --
                    # the same authority machinery as the ladder's decay -- so
                    # it must be when the frame was CAPTURED, not when the
                    # control loop got round to asking. Query time is short by
                    # the pipeline latency plus up to a loop period, always in
                    # the direction that makes a coast look younger than it is.
                    # Two lines below, `age_s` already uses the right value.
                    self._last_real[track_id] = (sampled_at_monotonic, score)
                    self._evict_last_real()

        return Sample(ex=horizontal_error, ey=vertical_error,
                      h_frac=bbox_height_frac, w_frac=bbox_width_frac,
                      age_s=time.monotonic() - sampled_at_monotonic,
                      class_id=class_id, score=score, vis_range=vis_range,
                      track_id=track_id, coasted=False)

    # ------------------------------------------------------------------ #
    #  Coast layer helpers (used only when bbox_error(coast_s>0))         #
    # ------------------------------------------------------------------ #
    # Bounds on `_last_real`. An entry only needs to outlive the longest
    # coast a caller can ask for, and `_present` rejects anything past
    # `_STALE_LIMIT_S` (1.0 s) anyway -- 30 s is enormous margin and exists
    # purely to bound memory.
    _LAST_REAL_HORIZON_S = 30.0
    _LAST_REAL_MAX = 256

    def _evict_last_real(self) -> None:
        """Drop sightings too old to start a coast. CALLER HOLDS THE LOCK.

        `_last_real` was written and never pruned. Two consequences, and the
        second is the dangerous one: it grew without bound over a mission, and
        a RECYCLED tracker id inherited the previous object's sighting
        timestamp -- so a coast could begin from a sighting that belonged to
        something else, at that object's score.

        Both tracker backends prune their own registries against exactly this
        hazard (`roboflow_tracker.py:202-211`, `bytetrack.py:138-148`); this
        dict did not.
        """
        if len(self._last_real) <= self._LAST_REAL_MAX:
            return
        cutoff = time.monotonic() - self._LAST_REAL_HORIZON_S
        for tid in [k for k, (t, _s) in self._last_real.items() if t < cutoff]:
            self._last_real.pop(tid, None)
        # Still oversized means many LIVE ids, not stale ones. Keep the newest
        # and SAY SO, rather than growing in silence.
        if len(self._last_real) > self._LAST_REAL_MAX:
            newest = sorted(self._last_real.items(),
                            key=lambda kv: kv[1][0], reverse=True)
            self._last_real = dict(newest[:self._LAST_REAL_MAX])
            if not self._evict_warned:
                self._evict_warned = True
                self._log.warn(
                    f'[VST  ] {self._LAST_REAL_MAX}+ live track ids -- the '
                    f'coast registry is being trimmed. Expect id churn.')

    _MATCH_GATE_NORM = 0.20   # max normalized centre distance to call a /tracks box "the same"

    def _match_track_id(self, ex: float, ey: float, tracks_array,
                        image_width: int, image_height: int) -> int:
        """Tracker id of the REAL /tracks box nearest the live detection at
        (ex, ey) normalized centre, within a small gate. -1 if none / no tracks."""
        if tracks_array is None or image_width <= 0 or image_height <= 0:
            return -1
        half_w, half_h = image_width * 0.5, image_height * 0.5
        best_id, best_dist = -1, self._MATCH_GATE_NORM
        for det in tracks_array.detections:
            if _track_is_predicted(det):
                continue                      # match against real boxes only
            tid = _track_id_of(det)
            if tid < 0:
                continue
            cx, cy = _bbox_center(det.bbox)
            d = math.hypot((cx - half_w) / half_w - ex, (cy - half_h) / half_h - ey)
            if d < best_dist:
                best_dist, best_id = d, tid
        return best_id

    def _coast_sample(self, class_name: str, locked_id: int, coast_s: float,
                      tracks_array, image_width: int,
                      image_height: int) -> Optional[Sample]:
        """Build a Sample from the coasted (predicted) /tracks box of locked_id.

        Returns None if: no tracks, the locked id has no predicted box this
        tick, the id was never seen as a real detection, or the gap already
        exceeds coast_s (-> caller treats as a loss). The Sample is conf-exempt
        by construction (it is not run through min_score) and carries the TRUE
        detection-age so downstream authority decays."""
        if tracks_array is None or image_width <= 0 or image_height <= 0:
            return None
        with self._lock:
            last = self._last_real.get(locked_id)
        if last is None:
            return None                        # never had a real sighting -> don't invent one
        last_t, last_score = last
        age = time.monotonic() - last_t
        if age > coast_s:
            return None                        # coast window elapsed -> loss

        for det in tracks_array.detections:
            if _track_id_of(det) != locked_id or not _track_is_predicted(det):
                continue
            if class_name and not _hypothesis_matches(det, class_name) \
                    and _hypothesis_class_id(det):
                continue
            cx, cy = _bbox_center(det.bbox)
            ex = max(-1.5, min(1.5, (cx - image_width * 0.5) / (image_width * 0.5)))
            ey = max(-1.5, min(1.5, (cy - image_height * 0.5) / (image_height * 0.5)))
            return Sample(
                ex=ex, ey=ey,
                h_frac=float(det.bbox.size_y) / float(image_height),
                w_frac=float(det.bbox.size_x) / float(image_width),
                age_s=age,                     # true time since last real detection
                class_id=class_name or _hypothesis_class_id(det),
                score=last_score,              # last real score (conf-exempt for the locked id)
                vis_range=0.0, track_id=locked_id, coasted=True)
        return None


def _bbox_center(bbox):
    centre = bbox.center
    if hasattr(centre, 'position'):       # Iron+: Pose2D w/ Point2D
        return float(centre.position.x), float(centre.position.y)
    return float(centre.x), float(centre.y)   # Humble: flat Pose2D


def _hypothesis_class_id(det: Detection2D) -> str:
    if not det.results:
        return ''
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'class_id'):
        return str(hyp.hypothesis.class_id)
    if hasattr(hyp, 'id'):
        return str(hyp.id)
    return ''


def _hypothesis_score(det: Detection2D) -> float:
    if not det.results:
        return 0.0
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'score'):
        return float(hyp.hypothesis.score)
    if hasattr(hyp, 'score'):
        return float(hyp.score)
    return 0.0


def _hypothesis_matches(det: Detection2D, class_name: str) -> bool:
    return _hypothesis_class_id(det).strip().lower() == class_name.strip().lower()


def _track_id_of(det: Detection2D) -> int:
    """tracker id carried on a /tracks Detection2D (`det.id` = str(track_id)).
    -1 for a raw /detections box (no id set) or a malformed value."""
    raw = getattr(det, 'id', '')
    try:
        return int(raw)
    except (TypeError, ValueError):
        return -1


def _track_is_predicted(det: Detection2D) -> bool:
    """True for a coasted (Kalman-predicted) /tracks box -- the tracker forces
    its hypothesis score to 0.0; a real box keeps the detector confidence."""
    return _hypothesis_score(det) <= 0.0
