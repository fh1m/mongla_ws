#!/usr/bin/env python3
"""tracker_node -- subscribe to detections, publish stable tracks.

Sits between detector_node and everything downstream. Takes raw per-frame
Detection2DArray, runs ByteTrack + optional Kalman smoothing, and publishes
a Detection2DArray with stable `tracking_id` fields.

Topics:
  in    /duburi/vision/<cam>/detections    vision_msgs/Detection2DArray  (raw)
  in    /duburi/vision/<cam>/camera_info   sensor_msgs/CameraInfo
  out   /duburi/vision/<cam>/tracks        vision_msgs/Detection2DArray  (tracked)

All tracker params are declared as ROS params so they can be tuned live:
  ros2 param set /duburi_tracker track_buffer 60
  ros2 param set /duburi_tracker enable_kalman false

A YAML preset lives at config/tracker.yaml:
  ros2 run duburi_vision tracker_node --ros-args \\
      --params-file src/duburi_vision/config/tracker.yaml

Examples
--------
# Default: subscribe to laptop camera
ros2 run duburi_vision tracker_node --ros-args -p camera:=laptop

# With custom buffer (60 frames = 3 s at 20 Hz)
ros2 run duburi_vision tracker_node --ros-args -p camera:=laptop \\
    -p track_buffer:=60
"""

import math
import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

# Detections to average the arrival rate over before judging it. Long enough
# that a startup hiccup or one slow frame does not decide it, short enough that
# the warning arrives while the operator is still watching the launch.
_RATE_SAMPLES = 60

# Detections to THROW AWAY before measuring. The first live run warned that a
# 55 Hz camera was arriving at 15 -- and it was, for the first two seconds,
# while both detectors were still contending for the chip's activation and the
# cameras were still ramping. Measuring across that transient gave a number
# that was true of the startup and false of the run. This project has the same
# lesson recorded on the loop sweep ("a mean folds in the startup transient");
# it applies to any rate measured from t=0.
_RATE_WARMUP = 120

# And take the MEDIAN inter-arrival, not the mean over the span. One long gap
# -- a detector pause, a model switch, a scheduler hiccup -- drags a span-based
# mean far more than it shifts the rate the coast windows actually see.
_RATE_MIN_HZ = 0.1

from duburi_vision.detection.messages import array_to_detections
from duburi_vision.tracking.bytetrack       import ByteTrackWrapper
from duburi_vision.tracking.roboflow_tracker import RoboflowTracker
from duburi_vision.tracking.kalman    import TrackKalmanSmoother
from duburi_vision.tracking.tracker   import TrackedDetection


def _build_tracker(tracker_type, *, track_buffer, frame_rate, min_hits,
                   iou_threshold, track_activation_threshold,
                   high_conf_det_threshold, log=None):
    """Build the tracker backend by name.

    'ocsort'/'bytetrack' use the Roboflow `trackers` library;
    'legacy_bytetrack' falls back to the supervision ByteTrack wrapper. If the
    Roboflow lib fails to import (bad Jetson install), fall back to legacy with
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
            tracker_type=ttype, track_buffer=track_buffer, frame_rate=frame_rate,
            min_hits=min_hits, iou_threshold=iou_threshold,
            track_activation_threshold=track_activation_threshold,
            high_conf_det_threshold=high_conf_det_threshold)
    except ImportError as exc:
        if log is not None:
            log.warn(f"[TRK  ] roboflow trackers unavailable ({exc}); "
                     f"falling back to legacy_bytetrack")
        return ByteTrackWrapper(
            track_buffer=track_buffer, min_hits=min_hits,
            iou_threshold=iou_threshold,
            track_activation_threshold=track_activation_threshold)


class TrackerNode(Node):
    def __init__(self):
        super().__init__('duburi_tracker')

        self.declare_parameter('camera',                      'laptop')
        # tracker engine: ocsort (Roboflow, default — best dropout recovery) |
        # bytetrack (Roboflow two-stage) | legacy_bytetrack (supervision fallback)
        self.declare_parameter('tracker_type',                'ocsort')
        self.declare_parameter('frame_rate',                  20.0)
        self.declare_parameter('track_buffer',                60)
        self.declare_parameter('min_hits',                    1)
        self.declare_parameter('iou_threshold',               0.2)
        self.declare_parameter('track_activation_threshold',  0.40)
        self.declare_parameter('high_conf_det_threshold',     0.6)
        self.declare_parameter('classes',                     '')
        self.declare_parameter('enable_kalman',               True)
        self.declare_parameter('kalman_process_noise',        0.1)
        self.declare_parameter('kalman_measurement_noise',    1.0)
        # SECONDS, not frames. It was `max_predict_frames: 30`, and 30 frames
        # is 1.5 s at the 20 Hz this was tuned at and 0.31 s at the Hailo
        # path's 98 Hz -- BELOW the control loop's typical `vision.coast_s` of
        # 0.8 s, so a coasted target vanished off /tracks before the coast
        # elapsed and the coast truncated early. That is the exact failure the
        # constant's own comment warned about, made live by perception getting
        # 5x faster, and nothing raised anything: the rung is the 4th of the
        # coast ladder (_freshness < coast_s < lost_grace_s < THIS) and it is
        # the only one that was expressed in frames.
        #
        # Note the asymmetry that hid it: Roboflow's own buffer IS frame-rate
        # aware (`int(frame_rate/30 * lost_track_buffer)`, verified in the
        # installed library), so the config's frame_rate scaled one rung and
        # not the other.
        self.declare_parameter('max_predict_s',               1.5)
        # How far the measured detection rate may drift from `frame_rate`
        # before saying so. Both rungs are sized from `frame_rate`, so a wrong
        # value makes the whole ladder wrong -- silently, since a tracker with
        # a truncated coast still publishes tracks and looks healthy.
        self.declare_parameter('frame_rate_warn_ratio',       1.5)

        cam              = str(self.get_parameter('camera').value).strip() or 'cam'
        tracker_type     = str(self.get_parameter('tracker_type').value).strip().lower()
        frame_rate       = float(self.get_parameter('frame_rate').value)
        track_buffer     = int(self.get_parameter('track_buffer').value)
        min_hits         = int(self.get_parameter('min_hits').value)
        iou_threshold    = float(self.get_parameter('iou_threshold').value)
        act_thresh       = float(self.get_parameter('track_activation_threshold').value)
        high_conf        = float(self.get_parameter('high_conf_det_threshold').value)
        self._enable_kal = bool(self.get_parameter('enable_kalman').value)
        proc_noise       = float(self.get_parameter('kalman_process_noise').value)
        meas_noise       = float(self.get_parameter('kalman_measurement_noise').value)
        max_pred_s       = float(self.get_parameter('max_predict_s').value)
        self._rate_tol   = float(self.get_parameter('frame_rate_warn_ratio').value)
        self._frame_rate = frame_rate
        # ceil, so a sub-frame remainder rounds UP: rounding a coast window
        # down is the failure being fixed.
        max_pred         = max(1, math.ceil(max_pred_s * frame_rate))
        self._last_classes = str(self.get_parameter('classes').value)

        self._tracker = _build_tracker(
            tracker_type, track_buffer=track_buffer, frame_rate=frame_rate,
            min_hits=min_hits, iou_threshold=iou_threshold,
            track_activation_threshold=act_thresh, high_conf_det_threshold=high_conf,
            log=self.get_logger())
        self._kalman = TrackKalmanSmoother(
            process_noise=proc_noise,
            measurement_noise=meas_noise,
            max_predict_frames=max_pred,
        ) if self._enable_kal else None

        ns  = f'/duburi/vision/{cam}'
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._sub_det  = self.create_subscription(
            Detection2DArray, f'{ns}/detections',  self._on_detections, qos)
        self._sub_info = self.create_subscription(
            CameraInfo,       f'{ns}/camera_info', self._on_info,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE))
        self._pub      = self.create_publisher(
            Detection2DArray, f'{ns}/tracks', qos)

        self._image_size = (640, 480)
        self._info_seen  = False
        self._size_ema: dict = {}   # track_id -> (smooth_w, smooth_h)

        # Rate check -- see _check_rate.
        self._rate_t: list = []
        self._rate_seen    = 0
        self._rate_warned  = False
        self._kal_frames   = max_pred

        # Diagnostics
        self._frames          = 0
        self._active_tracks   = 0
        self._last_log        = time.monotonic()
        self.create_timer(2.0, self._log_health)

        # Reset tracker when class filter changes (old IDs would linger otherwise)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"[TRK  ] subscribed {ns}/detections  engine={self._tracker.name} "
            f"track_buffer={track_buffer}@{frame_rate:.0f}Hz min_hits={min_hits} "
            f"coast={max_pred_s:.1f}s({max_pred}f) "
            f"act_thresh={act_thresh:.2f} "
            f"kalman={'on' if self._enable_kal else 'off'}")

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
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

    def _on_info(self, msg: CameraInfo) -> None:
        if msg.width and msg.height:
            self._image_size = (int(msg.width), int(msg.height))
            self._info_seen  = True

    def _check_rate(self, frame_t: float) -> None:
        """Is the detector running at the rate every coast window was sized for?

        Measured over a window rather than asserted, because the answer is a
        property of the machine: the same launch gives 98 Hz on the Pi's Hailo
        path, 20-30 Hz on the Jetson's TensorRT one and 3-4 Hz on a raw .pt --
        and the vehicle's two cameras do not even match each other (55 Hz
        forward against 15 Hz downward, measured). One `frame_rate` cannot be
        right for all of them, so the node says which one it is actually
        seeing instead of trusting the parameter.

        Warns ONCE. A rate mismatch is a configuration fact, not an event, and
        repeating it every window buries the log it is competing with.
        """
        if self._rate_warned:
            return
        self._rate_seen += 1
        if self._rate_seen <= _RATE_WARMUP:
            return
        self._rate_t.append(frame_t)
        if len(self._rate_t) < _RATE_SAMPLES:
            return
        gaps = sorted(b - a for a, b in zip(self._rate_t, self._rate_t[1:]))
        self._rate_t = []
        mid = gaps[len(gaps) // 2]
        if mid <= 0.0:
            return
        measured = 1.0 / mid
        if measured < _RATE_MIN_HZ:
            return
        self._rate_warned = True
        lo, hi = self._frame_rate / self._rate_tol, self._frame_rate * self._rate_tol
        coast_s = self._kal_frames / measured if measured else float('inf')
        if lo <= measured <= hi:
            # Report it EVEN WHEN IT IS RIGHT. Silence here is indistinguishable
            # from the check not running, and I could not tell those apart on
            # the vehicle: no warning appeared at a rate I had measured as out
            # of band with `ros2 topic hz` -- whose own subscriber load was
            # depressing the reading. The number the coast windows are actually
            # sized from is worth one line whatever it says.
            self.get_logger().warning(
                f"[TRK  ] detections {measured:.0f} Hz (frame_rate "
                f"{self._frame_rate:.0f}), coast {coast_s:.2f}s")
            return
        self.get_logger().warning(
            f"[TRK  ] detections arrive at {measured:.0f} Hz but frame_rate is "
            f"{self._frame_rate:.0f} -- every coast window is sized from that "
            f"number, so the Kalman coast is really {coast_s:.2f}s, not "
            f"{self._kal_frames / self._frame_rate:.2f}s. If that is under the "
            f"control loop's vision.coast_s the coast truncates early and a "
            f"target is dropped mid-lock. Set frame_rate:={measured:.0f}.")

    def _on_detections(self, msg: Detection2DArray) -> None:
        frame_t    = time.monotonic()
        self._check_rate(frame_t)
        detections = array_to_detections(msg)

        try:
            tracked = self._tracker.update(detections, frame_t)
        except Exception as exc:
            self.get_logger().error(f"[TRK  ] tracker.update failed: {exc!r}")
            return

        # Kalman smoothing pass.
        if self._kalman is not None:
            smoothed = []
            active_ids = {t.track_id for t in tracked}
            self._kalman.prune(active_ids)

            for td in tracked:
                if self._kalman.is_expired(td.track_id):
                    continue
                cx_hat, cy_hat = self._kalman.smooth(
                    td.track_id, td.cx, td.cy, frame_t, td.predicted)
                # Rebuild xyxy with smoothed centre, preserving original size.
                half_w = td.width  * 0.5
                half_h = td.height * 0.5
                smoothed_xyxy = (
                    cx_hat - half_w, cy_hat - half_h,
                    cx_hat + half_w, cy_hat + half_h,
                )
                smoothed.append(TrackedDetection(
                    class_id=td.class_id, class_name=td.class_name,
                    score=td.score, xyxy=smoothed_xyxy,
                    track_id=td.track_id, predicted=td.predicted,
                ))
            tracked = smoothed

        # Size EMA pass: smooth width/height per track_id independently.
        # Kalman only smoothed centre (cx, cy); raw ByteTrack size jitters every
        # frame and causes visible box shaking. EMA alpha=0.7 damps jitter in
        # ~4 frames while still following real size changes.
        active_ids = {t.track_id for t in tracked}
        self._size_ema = {k: v for k, v in self._size_ema.items() if k in active_ids}
        size_smoothed = []
        for td in tracked:
            sw, sh = self._size_ema.get(td.track_id, (td.width, td.height))
            sw = 0.7 * sw + 0.3 * td.width
            sh = 0.7 * sh + 0.3 * td.height
            self._size_ema[td.track_id] = (sw, sh)
            half_w = sw * 0.5
            half_h = sh * 0.5
            size_smoothed.append(TrackedDetection(
                class_id=td.class_id, class_name=td.class_name,
                score=td.score,
                xyxy=(td.cx - half_w, td.cy - half_h, td.cx + half_w, td.cy + half_h),
                track_id=td.track_id, predicted=td.predicted,
            ))
        tracked = size_smoothed

        out = self._build_array(tracked, msg.header)
        self._pub.publish(out)

        self._frames        += 1
        self._active_tracks  = len(tracked)

    def _build_array(self, tracks, header) -> Detection2DArray:
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        from duburi_vision.detection.messages import _set_center, _set_hypothesis

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
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)
