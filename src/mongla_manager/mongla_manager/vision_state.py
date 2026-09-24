"""VisionState -- per-camera subscriber that the vision motion loop reads.

One instance per camera. Owns three topic subscriptions and a small
thread-safe snapshot the closed-loop controller in
`mongla_control.motion_vision` reads each tick.

Why this lives in `mongla_manager`, not `mongla_vision`:
  * It's a manager-process resource (single MAVLink owner, single ROS
    node). Putting it here keeps `mongla_vision` free of any motion /
    control coupling.
  * `Mongla` consumes it via dependency injection (`vision_state_provider`)
    just like it consumes `yaw_source`, so the control package never
    imports rclpy directly.

Topics consumed (one camera example, `camera='laptop'`):
  /mongla/vision/laptop/detections   vision_msgs/Detection2DArray
  /mongla/vision/laptop/tracks       vision_msgs/Detection2DArray  (coast only)
  /mongla/vision/laptop/camera_info  sensor_msgs/CameraInfo
  /mongla/vision/laptop/vis_range    std_msgs/Float32MultiArray

NOT consumed: `image_raw`. The control host does not decode pixels, and a
subscription costs a full-frame deserialisation per message on the machine
that has to answer a 50 Hz loop. Anything that needs frames -- the HUD, the
console, a recorder -- subscribes on its own.

Public surface (called from the control loop, never spinning):
  largest(class_name)        -> Detection2D | None
  bbox_error(class_name)     -> Sample | None  (ex, ey, h_frac, age_s)
  image_size()               -> (W, H) ints
  is_fresh(stale_after_s)    -> bool
  list_classes()             -> sorted list of class_id strings seen
  close()                    -> tear down subscriptions

Threading model:
  rclpy callbacks fire in the executor thread; the control loop runs
  inside the action callback (a different thread under
  MultiThreadedExecutor). Every read takes `_lock` for the brief moment
  it copies the latest msg pointer.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo

from mongla_vision import qos
from mongla_vision.stamps import capture_monotonic
from vision_msgs.msg import Detection2D, Detection2DArray


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




class VisionState:
    """One camera's worth of subscribed-and-cached vision state.

    The control loop always reads ``/mongla/vision/<cam>/detections`` --
    the raw detector output, the exact topic the operator HUD overlays.
    The tracker node still runs for the display but is no longer in the
    control path, so a detection on screen is one the controller sees.
    """

    def __init__(self, node: Node, *, camera: str = 'laptop',
                 default_image_size: tuple = (0, 0),
                 logger=None):
        self._node    = node
        self._camera  = camera
        self._log     = logger or node.get_logger()

        self._lock          = threading.Lock()
        self._latest_array: Optional[Detection2DArray] = None
        self._latest_stamp: float = 0.0           # monotonic seconds
        self._evict_warned: bool  = False
        self._stamp_warned: bool  = False        # one-shot, see _capture_monotonic
        # Set on every detections message. A control loop waits on this
        # instead of sleeping a fixed tick, so it acts the moment a new
        # observation exists rather than at the next scheduled poll.
        self._new_sample = threading.Event()
        self._image_size:  tuple  = default_image_size
        # CameraInfo K/D, kept rather than discarded -- see _on_info.
        self._K = None
        self._D = None
        self._vis_range_vals: list = []            # parallel to _latest_array.detections
        self._info_seen:   bool   = False
        # Detection message counter. This used to count `image_raw`, which
        # meant the CONTROL HOST subscribed to the full 691 kB frame stream
        # to maintain a diagnostic integer -- and it did not even work: the
        # subscription was RELIABLE against a BEST_EFFORT publisher, so it
        # received NOTHING and the counter sat at 0 for ever. See the class
        # docstring; `preflight.wait_vision_state_ready` gated on it.
        self._det_msgs:    int    = 0
        # Coast layer (opt-in, used only when bbox_error(coast_s>0)): the /tracks
        # topic is the coast SOURCE; /detections stays the authoritative primary.
        self._latest_tracks: Optional[Detection2DArray] = None
        self._last_real: dict = {}                # track_id -> (monotonic_t, score) of last REAL detection

        ns = f'/mongla/vision/{camera}'
        # From mongla_vision.qos -- the same objects the PUBLISHERS use, so the
        # two ends of each link cannot drift apart. They already did once: this
        # class asked for RELIABLE on a BEST_EFFORT image topic and received
        # nothing for ever, silently.
        self._lock_array = None
        self._lock_stamp = 0.0
        self._pose = None            # latest TargetPose msg
        self._pose_stamp = 0.0       # its CAPTURE instant
        self._sub_det   = node.create_subscription(
            Detection2DArray, f'{ns}/detections',   self._on_detections,
            qos.DETECTIONS)
        # Coast source. Cheap to subscribe; only CONSULTED when coast_s>0, so a
        # mission that never sets vision.coast_s behaves exactly as before. If
        # the tracker node isn't running, this simply never delivers and coast
        # silently never engages (degrades to raw-/detections behaviour).
        self._sub_trk   = node.create_subscription(
            Detection2DArray, f'{ns}/tracks',        self._on_tracks,
            qos.DETECTIONS)
        # LADDER source. Same shape as the coast subscription above: cheap to
        # subscribe, only CONSULTED when `lock_s > 0`, and if `lock_node` is not
        # running it simply never delivers and the ladder never engages. A
        # mission that does not opt in behaves exactly as before.
        self._sub_lock  = node.create_subscription(
            Detection2DArray, f'{ns}/lock',          self._on_lock,
            qos.DETECTIONS)
        # 6-DoF target pose. Cheap to subscribe, only consulted by a caller
        # that asks -- an absent lock_node simply never delivers and every
        # squareness gate then refuses, which is the safe direction.
        try:
            from mongla_interfaces.msg import TargetPose as _TargetPose
            self._sub_pose = node.create_subscription(
                _TargetPose, f'{ns}/target_pose', self._on_target_pose,
                qos.DETECTIONS)
        except ImportError:
            self._sub_pose = None
        self._sub_info  = node.create_subscription(
            CameraInfo,       f'{ns}/camera_info',   self._on_info,
            qos.CAMERA_INFO)
        self._sub_vr    = node.create_subscription(
            Float32MultiArray, f'{ns}/vis_range',    self._on_vis_range,  10)

        self._log.info(
            f"[VST  ] subscribed camera={camera!r} -> "
            f"{ns}/detections (+tracks, +camera_info, +vis_range). "
            f"NOT image_raw -- the control host has no use for pixels.")

    # ------------------------------------------------------------------ #
    #  Subscriber callbacks                                              #
    # ------------------------------------------------------------------ #
    def _on_detections(self, msg: Detection2DArray) -> None:
        stamp = self._capture_monotonic(msg)
        with self._lock:
            self._latest_array = msg
            self._latest_stamp = stamp
            self._det_msgs += 1
        # Outside the lock: waking a waiter must not make it block on the very
        # lock it is about to need.
        self._new_sample.set()

    def _capture_monotonic(self, msg) -> float:
        """The instant the FRAME WAS CAPTURED, on the monotonic clock.

        This used to be `time.monotonic()` at message arrival, and everything
        downstream that believes it reads FRAME AGE was in fact reading AGE
        SINCE THE MESSAGE LANDED:

          _freshness       decays the lateral command by it
          the coast ladder 0.10 / 0.40 / 0.80 / 1.00 s are measured in it
          is_new_frame     gates the mid-hold torpedo FIRE on it
          align_stable_frames counts distinct values of it

        None of them could see the capture->inference->transport chain, so the
        whole measured latency -- ~32 ms median, 48 p95 -- was invisible to the
        loop that exists to react to it. `motion_vision` even says "sampled_at
        is the frame's arrival time" in a comment; the consequence was never
        drawn. Same defect as `camera_node` stamping `now()` at publish, one
        layer downstream: a carried truth resampled against a local clock.

        The conversion and its fail-safe live in `mongla_vision.stamps`, so the
        detector, the ladder and this all read one implementation -- a second
        copy of the bound is the hazard, not the arithmetic.
        """
        # `getattr`, not `msg.header`: a publisher with no header at all must
        # not take down the subscription callback. Caught by the test for it.
        t, why = capture_monotonic(getattr(msg, 'header', None))
        if why and not self._stamp_warned:
            self._stamp_warned = True
            self._log.warn(
                f'[VST  ] detection {why} -- falling back to arrival time, so '
                f'freshness and the coast ladder measure age since arrival '
                f'(the pre-fix behaviour) rather than a wrong number. Check '
                f'use_sim_time and the clock on the vision host.')
        return t

    def _on_tracks(self, msg: Detection2DArray) -> None:
        with self._lock:
            self._latest_tracks = msg

    def _on_info(self, msg: CameraInfo) -> None:
        if msg.width and msg.height:
            with self._lock:
                self._image_size = (int(msg.width), int(msg.height))
                self._info_seen  = True
                # K and D were being received and thrown away. They are what
                # turns a pixel error into a BEARING -- i.e. what gives a
                # control gain units of thrust-per-radian instead of
                # thrust-per-whatever-this-camera-happens-to-be. camera_node
                # rescales K to the streamed resolution before publishing, so
                # this is already correct for the frames the detector saw.
                self._K = list(msg.k) if len(msg.k) >= 9 else None
                self._D = list(msg.d) if msg.d is not None else None

    def calibration(self):
        """(K, D) as published, or (None, None).

        `CameraInfo.k` is all zeros until a calibration file is loaded, so a
        caller must test fx > 0 rather than `k is not None`. `bearing.py` does
        exactly that, falls back to an FOV, and reports which it used.
        """
        with self._lock:
            return (list(self._K) if self._K else None,
                    list(self._D) if self._D else None)

    def px_per_rad(self):
        """Pixels per radian of ray angle at frame centre, in the vehicle's
        medium -- fx * n -- or None when either is unknown.

        The calibration is taken in AIR, and a flat port makes the water ray
        angle 1/n of the air one near the centre, so bare fx under-corrects by
        25 % under water. The index comes from the manager's one medium
        parameter (`_uplink_n`), the same the board uplink uses, so the host
        loop and the board cannot disagree about the optics. No medium source
        means no correction, not a guess of air.
        """
        K, _D = self.calibration()
        n_fn = getattr(self._node, '_uplink_n', None)
        if not K or not (K[0] > 0.0) or not callable(n_fn):
            return None
        return float(K[0]) * float(n_fn())

    def _on_vis_range(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._vis_range_vals = list(msg.data)

    # ------------------------------------------------------------------ #
    #  Read API used by the control loop                                 #
    # ------------------------------------------------------------------ #
    def image_size(self) -> tuple:
        with self._lock:
            return self._image_size

    def info_seen(self) -> bool:
        with self._lock:
            return self._info_seen

    def wait_for_sample(self, timeout: float) -> bool:
        """Block until a new detections message lands, or `timeout` elapses.

        THIS REPLACES A FIXED-RATE SLEEP IN THE CONTROL LOOP, and the reason
        is the same one that moved `camera_node` off a timer: a fixed-rate
        poll against an asynchronous producer waits, on average, half a period
        for data that had already arrived.

        Measured here: detections land at ~77 Hz (13 ms apart) and the srot
        control loop ticked at 50 Hz (20 ms). Every command was therefore
        computed from an observation up to 13 ms older than the one available,
        ~6.5 ms on average -- a third of the whole detection age, spent
        waiting for a clock.

        Returns True if woken by a new sample, False on timeout. The timeout
        is what keeps the loop's TIME-based work alive -- freshness decay,
        hold timing, the arrival brake, the overall deadline -- when no
        detections are arriving at all, so the loop's floor rate is unchanged
        and only its ceiling moves.
        """
        got = self._new_sample.wait(timeout)
        if got:
            self._new_sample.clear()
        return got

    def is_fresh(self, stale_after: float) -> bool:
        with self._lock:
            if self._latest_array is None:
                return False
            return (time.monotonic() - self._latest_stamp) <= stale_after

    def largest(self, class_name: str = '') -> Optional[Detection2D]:
        """Return the largest-area detection matching `class_name`, or None.

        Empty `class_name` matches anything (handy for ad-hoc CLI checks).
        """
        with self._lock:
            detections_array = self._latest_array
        if detections_array is None:
            return None

        best_detection: Optional[Detection2D] = None
        best_area:      float = 0.0
        for detection in detections_array.detections:
            if class_name and not _hypothesis_matches(detection, class_name):
                continue
            area = float(detection.bbox.size_x) * float(detection.bbox.size_y)
            if area > best_area:
                best_area      = area
                best_detection = detection
        return best_detection

    def _on_lock(self, msg) -> None:
        """Latest ladder output. Kept whole; the rung name and confidence ride
        in `class_id`/`score`, so nothing here needs to know how many rungs
        exist."""
        with self._lock:
            self._lock_array = msg
            self._lock_stamp = self._capture_monotonic(msg)

    def _on_target_pose(self, msg) -> None:
        with self._lock:
            self._pose = msg
            self._pose_stamp = self._capture_monotonic(msg)

    def target_pose(self, max_age_s: float = 1.0):
        """The latest 6-DoF target pose, or None if absent/stale/refused."""
        with self._lock:
            m, t = self._pose, self._pose_stamp
        if m is None or not m.ok:
            return None
        if max_age_s > 0.0 and (time.monotonic() - t) > max_age_s:
            return None
        return m

    def square_within(self, tol_deg: float, max_age_s: float = 1.0) -> bool:
        """Is the target square to us within `tol_deg`, ALLOWING for the flip?

        THE GATE. Both flip branches are legitimate answers, so squareness only
        counts if the WORSE of them is inside tolerance -- reading the point
        estimate alone fires on the lucky branch, which is the exact failure
        this whole path exists to prevent.

        NO POSE MEANS NOT SQUARE. An absent `lock_node`, an uncalibrated
        camera, an unset `target_width_m` or a refused decomposition all return
        False. For a firing gate the fail-safe direction is "do not fire", and
        this is the only place that choice is made.
        """
        m = self.target_pose(max_age_s)
        if m is None:
            return False
        worst = float(m.off_axis_deg) + max(float(m.yaw_spread_deg),
                                            float(m.pitch_spread_deg))
        return worst <= float(tol_deg)

    def tool_offset_px(self, tool: str, max_age_s: float = 1.0):
        """Signed (du, dv) px that puts the target on TOOL's axis, or None.

        ⛔ THE MISS EQUALS THE OFFSET AT EVERY RANGE. An align centres the
        target on the CAMERA axis; a tool mounted elsewhere acts along a
        PARALLEL axis, so the error does not shrink as the hull closes in. A
        10 cm offset misses a 4.75 cm-radius torpedo opening from 1 m and from
        3 m alike -- with a perfectly centred box and a clean fire gate, which
        is why nothing ever reported it.

        The correction is the opposite of a constant: `du = fx*x/Z`, LARGE up
        close, so it needs a live range. That comes from `target_pose`, which
        is metric only since the flat-port refraction fix.

        None when it cannot be computed -- no such tool, no calibration, or no
        pose. The caller then aims the camera exactly as before and warns.
        Guessing a correction would move the aim point with false confidence,
        which is worse than a known-absent one.

        This lives here, not in `mongla_control`, because the geometry table is
        in `mongla_vision` and control must not depend on it -- the same rule
        that keeps the refractive index out of `bearing.py`.
        """
        try:
            from mongla_vision.tool_geometry import pixel_offset
        except ImportError:
            return None
        K, _D = self.calibration()
        if not K or len(K) < 6 or not (K[0] > 0.0 and K[4] > 0.0):
            return None
        m = self.target_pose(max_age_s)
        if m is None or not getattr(m, 'ok', False):
            return None
        return pixel_offset(tool, fx=float(K[0]), fy=float(K[4]),
                            range_m=float(m.range_m))

    def obliquity_deg(self, max_age_s: float = 1.0):
        """Worst-case angle between the target's face and our axis, or None.

        The honest primitive under `square_within`, which collapses "no pose"
        and "not square" into one False. That collapse is correct for a FIRING
        gate -- both mean do-not-fire -- and WRONG for anything whose fail-safe
        runs the other way, so callers that need to tell them apart get the
        number and decide for themselves.

        WORST of the flip branches, for the same reason `square_within` uses it:
        both are legitimate answers to a planar pose, and reading the point
        estimate alone believes the lucky one.
        """
        m = self.target_pose(max_age_s)
        if m is None:
            return None
        return float(m.off_axis_deg) + max(float(m.yaw_spread_deg),
                                           float(m.pitch_spread_deg))

    def _lock_sample(self, image_width: float, image_height: float):
        """A Sample from the ladder, or None.

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
            class_id=_hypothesis_class_id(d), score=score,
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
            # ⚠ `lock_s` IS A SWITCH, NOT A DURATION, and its name says
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
                return self._lock_sample(image_width, image_height)
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

    def list_classes(self) -> List[str]:
        """Sorted list of distinct class_id strings in the latest array."""
        with self._lock:
            detections_array = self._latest_array
        if detections_array is None:
            return []
        seen_class_ids = set()
        for detection in detections_array.detections:
            class_id = _hypothesis_class_id(detection)
            if class_id:
                seen_class_ids.add(class_id)
        return sorted(seen_class_ids)

    def diagnostics(self) -> dict:
        """Snapshot for the [STATE] / [VST  ] log line."""
        with self._lock:
            return {
                'camera':       self._camera,
                'image_size':   self._image_size,
                'info_seen':    self._info_seen,
                'det_msgs':     self._det_msgs,
                'last_age_s':   (time.monotonic() - self._latest_stamp
                                 if self._latest_array is not None
                                 else float('inf')),
            }

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def close(self) -> None:
        """Tear down every subscription, INDEPENDENTLY.

        One `try` around all five meant the first failure skipped the rest --
        and it was failing every time: `_sub_img` is a leftover from when this
        class subscribed to `image_raw`, the attribute no longer exists, and
        the AttributeError landed in the bare `except`. So `_sub_vr` was never
        destroyed and nothing said so.
        """
        for name in ('_sub_det', '_sub_trk', '_sub_info', '_sub_vr'):
            sub = getattr(self, name, None)
            if sub is None:
                continue
            try:
                self._node.destroy_subscription(sub)
            except Exception as exc:
                self._log.debug(f"[VST  ] close() {name}: {exc!r}")


# ---------------------------------------------------------------------- #
#  vision_msgs layout helpers (Humble vs Iron+)                          #
# ---------------------------------------------------------------------- #
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
