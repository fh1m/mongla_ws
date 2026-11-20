#!/usr/bin/env python3
"""tracker_node -- subscribe to detections, publish stable tracks.

Sits between detector_node and everything downstream. Takes raw per-frame
Detection2DArray, runs ByteTrack + optional Kalman smoothing, and publishes
a Detection2DArray with a stable id per track in `Detection2D.id`.
The field is `id`. vision_msgs/Detection2D has no `tracking_id`;
this docstring said otherwise and `check_tracker.py` believed it,
raising AttributeError on the first track of every run.

Topics:
  in    /mongla/vision/<cam>/candidates    vision_msgs/Detection2DArray  (raw)
  out   /mongla/vision/<cam>/tracks        vision_msgs/Detection2DArray  (tracked)

Every tracker setting is a ROS parameter, read ONCE when the node starts (the
tracker is built in __init__), so a change needs a restart. On the vehicle,
set them under `nodes:` in ~/.mongla/bringup.yaml, by node name:

    nodes:
      mongla_tracker_forward:
        track_buffer: 150

The defaults in __init__ are what runs; there is no separate preset file.

Examples
--------
# Default: subscribe to laptop camera
ros2 run mongla_vision tracker_node --ros-args -p camera:=laptop

# A shorter lost-track buffer: 60 = 2.0 s (Roboflow counts it in 30 Hz frames)
ros2 run mongla_vision tracker_node --ros-args -p camera:=laptop \\
    -p track_buffer:=60
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import math
import sys
import time

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray

from mongla_vision import qos
from mongla_vision.detection.messages import array_to_detections
from mongla_vision.tracking.bytetrack       import ByteTrackWrapper
from mongla_vision.tracking.roboflow_tracker import RoboflowTracker
from mongla_vision.tracking.kalman    import TrackKalmanSmoother
from mongla_vision.tracking.tracker   import TrackedDetection


def _build_tracker(tracker_type, *, track_buffer, min_hits,
                   iou_threshold, track_activation_threshold,
                   high_conf_det_threshold, detector_conf=0.0, log=None):
    """Build the tracker backend by name.

    'ocsort'/'bytetrack' use the Roboflow `trackers` library;
    'legacy_bytetrack' falls back to the supervision ByteTrack wrapper. If the
    Roboflow lib fails to import (bad install), fall back to legacy with
    a loud WARN rather than crash the node -- tracking degraded, not dead.
    """
    ttype = (tracker_type or 'ocsort').strip().lower()
    if ttype == 'legacy_bytetrack':
        return ByteTrackWrapper(
            track_buffer=track_buffer, min_hits=min_hits,
            iou_threshold=iou_threshold,
            track_activation_threshold=track_activation_threshold)
    try:
        return RoboflowTracker(
            tracker_type=ttype, track_buffer=track_buffer,
            min_hits=min_hits, iou_threshold=iou_threshold,
            track_activation_threshold=track_activation_threshold,
            high_conf_det_threshold=high_conf_det_threshold,
            detector_conf=detector_conf)
    except ImportError as exc:
        if log is not None:
            log.warn(f"[TRK  ] roboflow trackers unavailable ({exc}); "
                     f"falling back to legacy_bytetrack")
        return ByteTrackWrapper(
            track_buffer=track_buffer, min_hits=min_hits,
            iou_threshold=iou_threshold,
            track_activation_threshold=track_activation_threshold)


def capture_time(header) -> float:
    """The instant the frame was taken, in seconds, from the camera's stamp.

    Not the arrival time. The Kalman's transition and process noise are built
    from the interval between two calls, and at the Hailo path's ~99 Hz a
    frame interval is 10 ms -- so a few milliseconds of DDS delivery jitter
    was a 30 % error in the interval, and the same error in every velocity
    the filter carried. The detector copies the camera's capture stamp
    (kernel start-of-exposure) into the header, so the interval between two
    headers is the interval between two exposures. A header with no stamp
    (a hand-built test message) falls back to arrival time.
    """
    t = header.stamp.sec + header.stamp.nanosec * 1e-9
    return t if t > 0.0 else time.monotonic()


def kalman_pass(kalman, tracked, frame_t):
    """Smooth every track's centre, reviving any filter a real box returns to.

    Extracted so the test can drive it. The suite already had a test that
    composed the tracker and the smoother and then reimplemented this loop in
    a local helper (`test_roboflow_tracker.py:116`) -- so it tested a copy,
    and the copy did not contain the bug below. A test that reimplements the
    thing it guards is the `_srot_drive` grep in another costume.

    The bug. This used to be:

        if kalman.is_expired(td.track_id):
            continue

    `is_expired()` reads `predict_s`, which resets ONLY inside `smooth()`
    (`kalman.py:83`) -- which the `continue` skipped. And `prune()` only drops
    filters whose id is ABSENT from `tracked`. So once a filter expired while
    the backend still held the id, a returning target with that id was
    suppressed **for ever**: it could never reach the one call that would
    clear the condition suppressing it.

    Reachable at the shipped config -- the Kalman expires at 30 frames (1.5 s)
    and the Roboflow backend holds an id for 100 (`int(20/30*150)`), so frames
    30-100 were a dead zone where a reacquired target was silently missing
    from `/tracks`. The control coast follows `locked_id`, so it saw nothing,
    in the exact code path whose job is to survive a gap. Measured the same
    day on real competition footage: the gate approach has 53 gaps in 57 s,
    four of them past 1.5 s.

    A still-COASTING track past the horizon stays suppressed -- that guard is
    why expiry exists, and removing it would trade one bug for a worse one.
    """
    kalman.prune({t.track_id for t in tracked})
    smoothed = []
    for td in tracked:
        if kalman.is_expired(td.track_id):
            if td.predicted:
                continue              # still coasting: stay suppressed
            # A real measurement. Drop the stale filter so `smooth()` rebuilds
            # it from THIS observation rather than extrapolating from a track
            # that went cold.
            kalman.drop(td.track_id)
        # A COASTED box carries score 0.0 from both backends, which is a
        # marker, not evidence about this frame. Fed to the confidence model it
        # would be learned as a real, very low detector score and drag the
        # model's p10 down while the target is absent, so predicted boxes pass
        # NaN -- which the smoother reads as "no confidence information" and
        # leaves R alone.
        cx_hat, cy_hat = kalman.smooth(
            td.track_id, td.cx, td.cy, frame_t, td.predicted,
            conf=(float('nan') if td.predicted else float(td.score)))
        half_w, half_h = td.width * 0.5, td.height * 0.5
        smoothed.append(TrackedDetection(
            class_id=td.class_id, class_name=td.class_name, score=td.score,
            xyxy=(cx_hat - half_w, cy_hat - half_h,
                  cx_hat + half_w, cy_hat + half_h),
            track_id=td.track_id, predicted=td.predicted))
    return smoothed


# Box SIZE smoothing, on capture time. The retention per 30 Hz frame is 0.7,
# what it always was at that rate; as a time constant it no longer stretches
# with a slower camera (the Fantech's 10 fps in dim light made it 0.4 s) or
# shrinks with a faster one -- the frame-count defect B-114 removed from the
# coast windows, in its last place.
SIZE_TAU_S = -(1.0 / 30.0) / math.log(0.7)


def smooth_sizes(ema: dict, tracked, frame_t: float) -> list:
    """Each track's box with its width and height smoothed; `ema` is updated.

    `ema` maps track id -> (width, height, capture time). Ids not in this
    frame are dropped, so a returning id starts from its own box.
    """
    active = {t.track_id for t in tracked}
    for k in [k for k in ema if k not in active]:
        del ema[k]
    out = []
    for td in tracked:
        sw, sh, t0 = ema.get(td.track_id, (td.width, td.height, frame_t))
        keep = math.exp(-max(0.0, frame_t - t0) / SIZE_TAU_S)
        sw = keep * sw + (1.0 - keep) * td.width
        sh = keep * sh + (1.0 - keep) * td.height
        ema[td.track_id] = (sw, sh, frame_t)
        out.append(TrackedDetection(
            class_id=td.class_id, class_name=td.class_name, score=td.score,
            xyxy=(td.cx - sw * 0.5, td.cy - sh * 0.5,
                  td.cx + sw * 0.5, td.cy + sh * 0.5),
            track_id=td.track_id, predicted=td.predicted))
    return out


class TrackerNode(Node):
    def __init__(self, **node_kw):
        super().__init__('mongla_tracker', **node_kw)

        self.declare_parameter('camera',                      'laptop')
        # tracker engine: ocsort (Roboflow, default — best dropout recovery) |
        # bytetrack (Roboflow two-stage) | legacy_bytetrack (supervision fallback)
        self.declare_parameter('tracker_type',                'ocsort')
        # Seconds a lost track survives, written as 30 Hz frames because that
        # is the library's unit: 150 -> 5.0 s. The wrapper enforces it on
        # capture time, so it is 5.0 s at any detection rate. Bar: >= the p99
        # detection gap, 2.418 s (measured-bars.md, the lock ladder), so a
        # re-acquisition keeps its id.
        self.declare_parameter('track_buffer',                150)
        self.declare_parameter('min_hits',                    1)
        self.declare_parameter('iou_threshold',               0.2)
        self.declare_parameter('track_activation_threshold',  0.40)
        # Unset (NaN) means the detector's floor, which the backend clamps to
        # anyway; only an operator's value above the floor is worth a warning.
        # The library's 0.6 applies when the detector's floor is not told.
        self.declare_parameter('high_conf_det_threshold',     float('nan'))
        # The conf the DETECTOR publishes at. The tracker's gates are clamped
        # to it, because a box the detector chose to emit must be allowed to
        # start a track. 0.0 = not told (old behaviour).
        self.declare_parameter('detector_conf',               0.0)
        self.declare_parameter('classes',                     '')
        self.declare_parameter('enable_kalman',               True)
        self.declare_parameter('kalman_process_noise',        0.1)
        self.declare_parameter('kalman_measurement_noise',    1.0)
        # NSA (GIAOTracker/StrongSORT): scale R by the detection score, so a
        # doubtful box moves the filter less than a confident one. ON by
        # default -- measured on 5 competition clips it follows a
        # high-confidence box ~2x more closely AND still rejects low-confidence
        # ones, and on two of those clips the fixed-R filter was following BAD
        # boxes more closely than good ones (selectivity 0.96 and 0.64).
        self.declare_parameter('kalman_adaptive_noise',       True)
        # Seconds of capture time a track may coast before the smoother drops
        # it: the outermost rung of the coast ladder
        # (_freshness < vision.coast_s < lost_grace_s < THIS). Both tracker
        # windows are seconds on the frames' own stamps, so no detection rate
        # can shorten one below an inner rung (BUGS.md B-114).
        self.declare_parameter('max_predict_s',               1.5)
        cam              = str(self.get_parameter('camera').value).strip() or 'cam'
        tracker_type     = str(self.get_parameter('tracker_type').value).strip().lower()
        track_buffer     = int(self.get_parameter('track_buffer').value)
        min_hits         = int(self.get_parameter('min_hits').value)
        iou_threshold    = float(self.get_parameter('iou_threshold').value)
        act_thresh       = float(self.get_parameter('track_activation_threshold').value)
        high_conf        = float(self.get_parameter('high_conf_det_threshold').value)
        self._enable_kal = bool(self.get_parameter('enable_kalman').value)
        proc_noise       = float(self.get_parameter('kalman_process_noise').value)
        meas_noise       = float(self.get_parameter('kalman_measurement_noise').value)
        adaptive_noise   = bool(self.get_parameter('kalman_adaptive_noise').value)
        max_pred_s       = float(self.get_parameter('max_predict_s').value)
        self._last_classes = str(self.get_parameter('classes').value)

        # The DETECTOR's floor, so the tracker's confidence gates can be
        # clamped to it. They were 4x above it and the tracker emitted nothing
        # on real footage -- see `RoboflowTracker.__init__`. Read from the
        # parameter the launch sets on the detector; 0 means "not told", which
        # leaves the old behaviour rather than guessing.
        detector_conf = float(self.get_parameter('detector_conf').value)
        asked_high = not math.isnan(high_conf)
        if not asked_high:
            high_conf = detector_conf if detector_conf > 0.0 else 0.6
        self._tracker = _build_tracker(
            tracker_type, track_buffer=track_buffer,
            min_hits=min_hits, iou_threshold=iou_threshold,
            track_activation_threshold=act_thresh, high_conf_det_threshold=high_conf,
            detector_conf=detector_conf, log=self.get_logger())
        if asked_high and detector_conf > 0.0 and high_conf > detector_conf:
            self.get_logger().warn(
                f'[TRK  ] high_conf_det_threshold {high_conf:.2f} is above the '
                f'detector floor {detector_conf:.2f} -- clamped. Above it, NO '
                f'track is ever created and /tracks stays empty while looking '
                f'healthy.')
        self._kalman = TrackKalmanSmoother(
            adaptive_noise=adaptive_noise,
            process_noise=proc_noise,
            measurement_noise=meas_noise,
            max_predict_s=max_pred_s,
        ) if self._enable_kal else None
        if self._enable_kal and adaptive_noise:
            self.get_logger().info(
                '[TRK  ] adaptive measurement noise ON -- R scales with the '
                'detector score, normalised against this detector\'s own '
                'p10..p90. Measured on 5 competition clips: follows a '
                'high-confidence box ~2x more closely while still rejecting '
                'low-confidence ones.')

        ns  = f'/mongla/vision/{cam}'
        # `candidates`: every box down to the detector's association floor.
        # Creation stays at `detector_conf` (the acting bar, clamped below);
        # the boxes under it can only extend a track the bar started, which is
        # ByteTrack's second stage. The same boxes as `detections` while the
        # detector's `assoc_conf` is off.
        self._sub_det  = self.create_subscription(
            Detection2DArray, f'{ns}/candidates',  self._on_detections,
            qos.DETECTIONS)
        # `tracks` mirrors `detections`: same shape, same consumers, and the
        # coast layer treats them as one ladder.
        self._pub      = self.create_publisher(
            Detection2DArray, f'{ns}/tracks', qos.DETECTIONS)

        self._size_ema: dict = {}   # track_id -> (width, height, capture time)

        # Diagnostics
        self._frames          = 0
        self._active_tracks   = 0
        self._last_log        = time.monotonic()
        self.create_timer(2.0, self._log_health)

        # Reset tracker when class filter changes (old IDs would linger otherwise)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"[TRK  ] subscribed {ns}/candidates  engine={self._tracker.name} "
            f"track_buffer={track_buffer / 30.0:.1f}s min_hits={min_hits} "
            f"coast={max_pred_s:.1f}s "
            f"act_thresh={act_thresh:.2f} "
            f"kalman={'on' if self._enable_kal else 'off'}")

    LIVE_PARAMS = ('classes',)

    def _on_param_change(self, params):
        """`classes` applies live; everything else is read once at startup.

        So a live set of anything else is REFUSED with the reason. It used to
        be accepted: `ros2 param set ... track_buffer 150` printed "Set
        parameter successful" on the deck and changed nothing.
        """
        from rcl_interfaces.msg import SetParametersResult
        fixed = [p.name for p in params if p.name != 'classes']
        if fixed:
            return SetParametersResult(
                successful=False,
                reason=f'{", ".join(fixed)}: read once at startup -- set it under '
                       f'`nodes:` in bringup.yaml and restart')
        for p in params:
            if p.name == 'classes' and str(p.value) != self._last_classes:
                self._last_classes = str(p.value)
                self._tracker.reset()
                if self._kalman is not None:
                    self._kalman.reset()
                self._size_ema.clear()
                self.get_logger().info(
                    f"[TRK  ] classes changed → reset tracker (new classes='{p.value}')")
        return SetParametersResult(successful=True)

    def _on_detections(self, msg: Detection2DArray) -> None:
        frame_t    = capture_time(msg.header)
        detections = array_to_detections(msg)

        try:
            tracked = self._tracker.update(detections, frame_t)
        except Exception as exc:
            self.get_logger().error(f"[TRK  ] tracker.update failed: {exc!r}")
            return

        # Kalman smoothing pass. The logic lives in `kalman_pass` so the
        # test suite can drive the SHIPPING code instead of a copy of it --
        # see the note on that function.
        if self._kalman is not None:
            tracked = kalman_pass(self._kalman, tracked, frame_t)

        # Size smoothing: the Kalman smooths only the centre, and raw box
        # size jitters every frame. On capture time; see `smooth_sizes`.
        tracked = smooth_sizes(self._size_ema, tracked, frame_t)

        out = self._build_array(tracked, msg.header)
        self._pub.publish(out)

        self._frames        += 1
        self._active_tracks  = len(tracked)

    def _build_array(self, tracks, header) -> Detection2DArray:
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        from mongla_vision.detection.messages import _set_center, _set_hypothesis

        arr = Detection2DArray()
        arr.header = header
        for td in tracks:
            d2 = Detection2D()
            d2.header = header
            _set_center(d2.bbox.center, td.cx, td.cy)
            d2.bbox.size_x = float(td.width)
            d2.bbox.size_y = float(td.height)
            d2.id = str(td.track_id)
            hypo = ObjectHypothesisWithPose()
            _set_hypothesis(hypo, td.class_name, td.score)
            d2.results.append(hypo)
            arr.detections.append(d2)
        return arr

    def _log_health(self) -> None:
        now     = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        in_hz   = self._frames / elapsed
        self.get_logger().info(
            f"[TRK  ] in_hz={in_hz:5.1f}  active_tracks={self._active_tracks}  "
            f"total_frames={self._frames}")
        self._frames    = 0
        self._last_log  = now


def main():
    from mongla_runtime.orphan import die_with_parent
    die_with_parent()        # an orphan holds devices and duplicates topics
    rclpy.init()
    node = TrackerNode()
    from mongla_runtime.params_file import refuse_unknown
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    try:
        from mongla_runtime.spin import spin
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        from mongla_runtime.spin import deaf_to_ctrl_c
        deaf_to_ctrl_c()      # a second Ctrl-C must not interrupt the teardown
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)
