#!/usr/bin/env python3
"""camera_node -- read from a Camera and publish standard ROS image topics.

One camera per node instance. Pick the source via the `profile` ROS param
(matched against `CAMERA_PROFILES`) OR by passing `source` + source-specific
overrides. The node publishes:

  /duburi/vision/<cam>/image_raw     sensor_msgs/Image      (bgr8)
  /duburi/vision/<cam>/camera_info   sensor_msgs/CameraInfo (size only; K/D
                                                              empty until we
                                                              ship a calib file)

Examples
--------
# Laptop webcam by named profile
ros2 run duburi_vision camera_node --ros-args -p profile:=laptop

# Same thing, fully explicit
ros2 run duburi_vision camera_node --ros-args \\
    -p source:=webcam -p device:=0 -p width:=640 -p height:=480 -p fps:=30 -p name:=laptop

# Subscribe to a Gazebo camera topic and re-publish under our namespace
ros2 run duburi_vision camera_node --ros-args \\
    -p source:=ros_topic -p topic:=/duburi/sim/front_camera/image_raw -p name:=sim_front
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import queue as _queue
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg     import Float32, Int32
from std_srvs.srv     import SetBool
from cv_bridge        import CvBridge

from duburi_vision import (
    CAMERA_PROFILES,
    make_camera, make_camera_from_profile,
    get_profile,
)
from duburi_vision.cameras.discover import discover_cameras


class CameraNode(Node):
    def __init__(self):
        super().__init__('duburi_camera')

        self.declare_parameter('profile',         '')        # e.g. 'laptop' / 'sim_front'
        self.declare_parameter('source',          '')        # explicit override
        self.declare_parameter('name',            '')
        self.declare_parameter('topic',           '')        # for source=ros_topic
        self.declare_parameter('device',          -1)        # -1 = use profile default; ≥0 overrides
        # PORT-STABLE identity for identical cameras (same VID/PID): a /dev/v4l/by-path/…
        # symlink pins the camera to a physical USB PORT, so 'forward'/'downward' never
        # swap on reboot/re-enumeration (unlike /dev/videoN indices). Non-empty wins over
        # `device`. See .claude/context/dual-camera-setup.md.
        self.declare_parameter('device_path',     '')        # e.g. /dev/v4l/by-path/...-video-index0
        self.declare_parameter('width',           640)
        self.declare_parameter('height',          480)
        # fps / publish_rate_hz declared as int so launch ints pass through
        # cleanly; we cast to float at the timer site.
        self.declare_parameter('fps',             30)
        self.declare_parameter('frame_id',        '')
        self.declare_parameter('publish_rate_hz', 30)
        self.declare_parameter('path',            '')        # for source=video_file
        self.declare_parameter('loop',            True)      # for source=video_file
        self.declare_parameter('discover_on_start', False)   # log USB camera table

        if self.get_parameter('discover_on_start').get_parameter_value().bool_value:
            cams = discover_cameras()
            if cams:
                self.get_logger().info('[CAM  ] USB cameras detected:')
                for c in cams:
                    self.get_logger().info(
                        f"[CAM  ]   [{c['index']}] {c['path']}  ({c['name']})")
            else:
                self.get_logger().warn('[CAM  ] no USB cameras found via discover')

        self._cam     = self._build_camera_with_retry()
        self._info    = self._cam.info()
        self._cam_name = str(self._info.get('name') or 'cam')
        self._frame_id = str(self._info.get('frame_id') or self._cam_name)

        ns = f'/duburi/vision/{self._cam_name}'
        self._pub_img  = self.create_publisher(Image,      f'{ns}/image_raw',   10)
        self._pub_info = self.create_publisher(CameraInfo, f'{ns}/camera_info', 10)
        self._bridge   = CvBridge()

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self._sent    = 0
        self._dropped = 0
        self._last_log = time.monotonic()
        self.create_timer(2.0, self._log_health)

        # Background capture thread — keeps cap.read() off the ROS executor.
        self._frame_q: _queue.SimpleQueue = _queue.SimpleQueue()
        threading.Thread(target=self._capture_loop, daemon=True).start()

        # Video file playback controls (only active when source supports pause/seek).
        if hasattr(self._cam, 'pause'):
            self.create_service(SetBool, f'{ns}/video_pause',       self._handle_video_pause)
            self.create_subscription(Float32, f'{ns}/video_seek_rel',   self._handle_seek,       10)
            self.create_subscription(Int32,   f'{ns}/video_seek_frame', self._handle_seek_frame, 10)
            self.create_subscription(Float32, f'{ns}/video_speed',      self._handle_speed,      10)
            self.get_logger().info(
                f'[CAM  ] video controls: {ns}/video_pause  '
                f'{ns}/video_seek_rel  {ns}/video_seek_frame  {ns}/video_speed')

        self.get_logger().info(
            f"[CAM  ] {self._cam_name!r} ({self._info.get('source_kind')}) -> "
            f"{ns}/image_raw  @ {rate:.1f} Hz")

    # A transient USB-enumeration blip at startup shouldn't kill the camera for
    # the whole session -- retry a few times before giving up. __init__ hasn't
    # started the ROS timers yet, so a short blocking backoff here is safe.
    _OPEN_RETRIES   = 5
    _OPEN_BACKOFF_S = 1.0

    def _build_camera_with_retry(self):
        last_exc = None
        for attempt in range(1, self._OPEN_RETRIES + 1):
            try:
                return self._build_camera()
            except Exception as exc:   # noqa: BLE001 -- retry any open failure
                last_exc = exc
                if attempt < self._OPEN_RETRIES:
                    self.get_logger().warn(
                        f'[CAM  ] camera open failed (attempt {attempt}/'
                        f'{self._OPEN_RETRIES}): {exc} — retrying in '
                        f'{self._OPEN_BACKOFF_S:.1f}s')
                    time.sleep(self._OPEN_BACKOFF_S)
        # A truly absent device after every retry is a real error -- still raise.
        self.get_logger().error(
            f'[CAM  ] camera open FAILED after {self._OPEN_RETRIES} attempts: {last_exc}')
        raise last_exc

    def _build_camera(self):
        # A by-path symlink (port-stable) wins over the int device index everywhere.
        device_path = str(self.get_parameter('device_path').value).strip()
        profile_name = str(self.get_parameter('profile').value).strip()
        if profile_name:
            profile = get_profile(profile_name)
            if not str(self.get_parameter('name').value).strip():
                profile.setdefault('name', profile_name)
            else:
                profile['name'] = str(self.get_parameter('name').value).strip()
            # device_path (by-path symlink) > int device override > profile default.
            if device_path:
                profile['device'] = device_path
                self.get_logger().info(f'[CAM  ] device_path (port-stable) → {device_path}')
            else:
                dev_param = self.get_parameter('device').value
                try:
                    dev_int = int(dev_param)
                except (TypeError, ValueError):
                    dev_int = -1
                if dev_int >= 0:
                    profile['device'] = dev_int
            return make_camera_from_profile(
                profile, node=self, logger=self.get_logger())

        source = str(self.get_parameter('source').value).strip()
        if not source:
            raise ValueError(
                f"camera_node: must set either 'profile' (one of "
                f"{sorted(CAMERA_PROFILES)}) or 'source' (one of webcam/ros_topic/...)")

        kwargs = {
            'name':       str(self.get_parameter('name').value).strip() or source,
            'frame_id':   str(self.get_parameter('frame_id').value).strip()
                          or str(self.get_parameter('name').value).strip()
                          or source,
        }
        if source == 'webcam':
            if device_path:
                dev = device_path   # by-path symlink (port-stable) wins
                self.get_logger().info(f'[CAM  ] device_path (port-stable) → {device_path}')
            else:
                dev = self.get_parameter('device').value
                try:
                    dev = int(dev)
                except (TypeError, ValueError):
                    pass
                if isinstance(dev, int) and dev < 0:
                    dev = 0  # no explicit override → first available device
            kwargs.update(
                device=dev,
                width=int(self.get_parameter('width').value),
                height=int(self.get_parameter('height').value),
                fps=int(self.get_parameter('fps').value),
            )
        elif source == 'ros_topic':
            kwargs.update(
                node=self,
                topic=str(self.get_parameter('topic').value).strip(),
                expected_width=int(self.get_parameter('width').value),
                expected_height=int(self.get_parameter('height').value),
                expected_fps=float(self.get_parameter('fps').value),
            )
        elif source == 'video_file':
            loop_val = self.get_parameter('loop').value
            kwargs.update(
                path=str(self.get_parameter('path').value).strip(),
                loop=bool(loop_val) if not isinstance(loop_val, str) else loop_val.lower() != 'false',
                width=int(self.get_parameter('width').value),
                height=int(self.get_parameter('height').value),
                fps=int(self.get_parameter('fps').value),
            )

        return make_camera(source, logger=self.get_logger(), **kwargs)

    def _capture_loop(self):
        """Daemon thread: continuously read frames and put latest into queue."""
        try:
            while rclpy.ok():
                frame, meta = self._cam.read()
                if frame is None:
                    time.sleep(0.02)
                    continue
                if not meta.fresh:
                    # Paused / EOF — avoid tight spin; display keeps its last rendered frame.
                    time.sleep(0.02)
                    continue
                # Single-slot: drop stale frame, keep only latest.
                while not self._frame_q.empty():
                    try:
                        self._frame_q.get_nowait()
                    except _queue.Empty:
                        break
                self._frame_q.put_nowait((frame, meta))
        except Exception:
            pass  # camera released during shutdown

    def _tick(self):
        try:
            frame, meta = self._frame_q.get_nowait()
        except _queue.Empty:
            return

        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f"[CAM  ] cv_bridge encode failed: {exc!r}")
            self._dropped += 1
            return

        stamp = self.get_clock().now().to_msg()
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = self._frame_id

        info = CameraInfo()
        info.header = img_msg.header
        info.width  = int(meta.width  or img_msg.width)
        info.height = int(meta.height or img_msg.height)

        self._pub_img.publish(img_msg)
        self._pub_info.publish(info)
        self._sent += 1

    def _handle_video_pause(self, req: SetBool.Request,
                             resp: SetBool.Response) -> SetBool.Response:
        if req.data:
            self._cam.pause()   # type: ignore[attr-defined]
            resp.message = 'paused'
        else:
            self._cam.resume()  # type: ignore[attr-defined]
            resp.message = 'resumed'
        resp.success = True
        return resp

    def _handle_seek(self, msg: Float32) -> None:
        self._cam.seek_rel(msg.data)  # type: ignore[attr-defined]

    def _handle_seek_frame(self, msg: Int32) -> None:
        if hasattr(self._cam, 'seek_frames'):
            self._cam.seek_frames(int(msg.data))  # type: ignore[attr-defined]

    def _handle_speed(self, msg: Float32) -> None:
        if hasattr(self._cam, 'set_speed'):
            new_speed = self._cam.set_speed(float(msg.data))  # type: ignore[attr-defined]
            self.get_logger().info(f'[CAM  ] playback speed → {new_speed:.2f}×')

    def _log_health(self):
        now = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        hz = self._sent / elapsed
        healthy = self._cam.is_healthy()
        marker = 'OK ' if healthy else 'BAD'
        extra = ''
        if hasattr(self._cam, 'position'):
            cur, total = self._cam.position
            pct = 100 * cur / total if total else 0.0
            paused = 'paused' if self._cam.is_paused else 'playing'  # type: ignore[attr-defined]
            spd = self._cam.speed if hasattr(self._cam, 'speed') else 1.0  # type: ignore[attr-defined]
            extra = f'  pos={cur}/{total} ({pct:.0f}%)  {paused}  {spd:.2f}×'
        self.get_logger().info(
            f"[CAM  ] {marker}  {self._cam_name}  pub={hz:5.1f}Hz  "
            f"sent={self._sent}  dropped={self._dropped}{extra}")
        self._sent = 0
        self._dropped = 0
        self._last_log = now

    def shutdown(self):
        try:
            self._cam.close()
        except Exception as exc:
            self.get_logger().debug(f"camera close ignored: {exc!r}")


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)
