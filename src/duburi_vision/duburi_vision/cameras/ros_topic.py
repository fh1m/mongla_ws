"""RosTopicCamera — subscribes to a sensor_msgs/Image topic.

Used for Gazebo, BlueOS image_transport, or any other already-publishing
source. We hold the latest frame in a tiny single-slot buffer; reads are
non-blocking and just hand back whatever the subscriber thread last
received. `fresh=True` only when read() consumes a frame that was newer
than the previous read (so the detector can decide whether to re-infer).

Construction needs an rclpy.Node so we can attach the subscription —
the factory passes one in. If you build this outside a node (e.g. in a
test), pass node=None and the source raises on construction with a
helpful message.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import numpy as np

from .camera import Camera, FrameMeta


class RosTopicCamera(Camera):
    source_kind = 'ros_topic'

    def __init__(self, *, node, topic, frame_id='ros_cam', name='ros_topic',
                 expected_width=0, expected_height=0, expected_fps=30.0,
                 logger=None):
        if node is None:
            raise ValueError(
                "ros_topic camera needs node=<rclpy.node.Node>. "
                "Use this source from a ROS node (camera_node), not from "
                "plain Python scripts.")

        from sensor_msgs.msg import Image
        from cv_bridge        import CvBridge

        self.name      = str(name)
        self._topic    = str(topic)
        self._frame_id = str(frame_id)
        self._node     = node
        self._log      = logger
        self._bridge   = CvBridge()

        self._lock        = threading.Lock()
        self._latest      = None     # (np.ndarray, t_mono, w, h)
        self._consumed_at = -1.0     # t_mono of the last frame we returned
        self._idx         = 0
        self._exp_fps     = float(expected_fps)
        self._exp_w       = int(expected_width)
        self._exp_h       = int(expected_height)

        self._sub = node.create_subscription(Image, self._topic, self._on_image, 10)

        if self._log:
            self._log.info(f"[CAM ] subscribed to ros image topic {self._topic!r}")

    def _on_image(self, msg) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            if self._log:
                self._log.warning(f"[CAM ] cv_bridge convert failed: {exc!r}")
            return
        h, w = frame.shape[:2]
        with self._lock:
            self._latest = (frame, time.monotonic(), w, h)

    def read(self) -> Tuple[Optional[np.ndarray], FrameMeta]:
        with self._lock:
            snap = self._latest
            consumed_at = self._consumed_at
        if snap is None:
            return None, FrameMeta(frame_index=self._idx, fresh=False)
        frame, t_mono, w, h = snap
        fresh = t_mono > consumed_at
        if fresh:
            self._idx += 1
            with self._lock:
                self._consumed_at = t_mono
        meta = FrameMeta(
            frame_index=self._idx,
            stamp_monotonic=t_mono,
            stamp_wall=time.time(),
            width=w,
            height=h,
            fresh=fresh,
        )
        return frame, meta

    def is_healthy(self) -> bool:
        with self._lock:
            snap = self._latest
        if snap is None:
            return False
        return (time.monotonic() - snap[1]) < 2.0

    def info(self) -> dict:
        with self._lock:
            snap = self._latest
        w, h = (self._exp_w, self._exp_h) if snap is None else (snap[2], snap[3])
        return {
            'name':        self.name,
            'source_kind': self.source_kind,
            'topic':       self._topic,
            'width':       w,
            'height':      h,
            'fps':         self._exp_fps,
            'frame_id':    self._frame_id,
        }

    def close(self) -> None:
        if self._sub is not None:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
