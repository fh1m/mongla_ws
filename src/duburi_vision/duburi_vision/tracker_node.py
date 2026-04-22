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

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from duburi_vision.detection.messages import array_to_detections
from duburi_vision.tracking.bytetrack import ByteTrackWrapper
from duburi_vision.tracking.kalman    import TrackKalmanSmoother
from duburi_vision.tracking.tracker   import TrackedDetection


class TrackerNode(Node):
    def __init__(self):
        super().__init__('duburi_tracker')

        self.declare_parameter('camera',                 'laptop')
        self.declare_parameter('track_buffer',           30)
        self.declare_parameter('min_hits',               3)
        self.declare_parameter('iou_threshold',          0.3)
        self.declare_parameter('enable_kalman',          True)
        self.declare_parameter('kalman_process_noise',   0.1)
        self.declare_parameter('kalman_measurement_noise', 1.0)
        self.declare_parameter('max_predict_frames',     5)

        cam              = str(self.get_parameter('camera').value).strip() or 'cam'
        track_buffer     = int(self.get_parameter('track_buffer').value)
        min_hits         = int(self.get_parameter('min_hits').value)
        iou_threshold    = float(self.get_parameter('iou_threshold').value)
        self._enable_kal = bool(self.get_parameter('enable_kalman').value)
        proc_noise       = float(self.get_parameter('kalman_process_noise').value)
        meas_noise       = float(self.get_parameter('kalman_measurement_noise').value)
        max_pred         = int(self.get_parameter('max_predict_frames').value)

        self._tracker = ByteTrackWrapper(
            track_buffer=track_buffer,
            min_hits=min_hits,
            iou_threshold=iou_threshold,
        )
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

        # Diagnostics
        self._frames          = 0
        self._active_tracks   = 0
        self._last_log        = time.monotonic()
        self.create_timer(2.0, self._log_health)

        self.get_logger().info(
            f"[TRK  ] subscribed {ns}/detections  "
            f"track_buffer={track_buffer} min_hits={min_hits} "
            f"kalman={'on' if self._enable_kal else 'off'}")

    def _on_info(self, msg: CameraInfo) -> None:
        if msg.width and msg.height:
            self._image_size = (int(msg.width), int(msg.height))
            self._info_seen  = True

    def _on_detections(self, msg: Detection2DArray) -> None:
        frame_t    = time.monotonic()
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
            d2.tracking_id = str(td.track_id)
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
