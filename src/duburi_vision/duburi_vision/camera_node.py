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
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time

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
        # 0 = "the profile decides", which is what the comment in
        # _build_camera has claimed since round 26 while this line said 30.
        # The non-profile branch already falls back to 30 for a 0, so the
        # sentinel never reaches a device call; the profile branch now honours
        # a POSITIVE value as an override, the same precedence device_path has
        # over device. Declaring 30 here is what let a launch default silently
        # beat every profile in round 26 -- fixed there by removing the launch
        # default, which left the node's own default able to do it again.
        self.declare_parameter('fps',             0)
        self.declare_parameter('frame_id',        '')
        # Path to a calibration.json from tools/fov_calibrate.py. Empty (the
        # default) keeps the historical behaviour EXACTLY: size-only CameraInfo
        # with K and D left zero. Nothing changes until a real calibration
        # exists for this camera.
        self.declare_parameter('calibration',     '')
        # 0 = follow the camera's own fps (see the timer below). A positive
        # value pins the publish rate regardless of what the camera delivers.
        self.declare_parameter('publish_rate_hz', 0)
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
        self._log_rate_source = True
        self._info    = self._cam.info()
        self._cam_name = str(self._info.get('name') or 'cam')
        self._frame_id = str(self._info.get('frame_id') or self._cam_name)

        ns = f'/duburi/vision/{self._cam_name}'
        # A MAILBOX, NOT A QUEUE -- the DDS half of the same problem the
        # capture side has. An int depth here means KEEP_LAST at that depth
        # with RELIABLE reliability, so a slow subscriber makes the middleware
        # HOLD frames and retransmit them: the consumer then works through a
        # backlog of old pictures, which is precisely the failure the mailbox
        # capture path exists to prevent, reintroduced one layer up.
        #
        # Note `qos_profile_sensor_data` is NOT this: it is BEST_EFFORT but
        # KEEP_LAST **depth 5**, i.e. still a five-deep queue. For a stream
        # where only the newest frame has any value, one is the right number.
        self._img_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._pub_img  = self.create_publisher(Image, f'{ns}/image_raw',
                                               self._img_qos)
        # CameraInfo stays RELIABLE and TRANSIENT_LOCAL: it is ~1 kB of
        # calibration that changes never, and a subscriber that joins late must
        # still receive it. Dropping it would leave `CameraInfo.k` empty and
        # every pixel->bearing conversion falling back to a guessed FOV.
        self._pub_info = self.create_publisher(
            CameraInfo, f'{ns}/camera_info',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._calib = self._load_calibration()
        self._bridge   = CvBridge()

        # publish_rate_hz <= 0 means "follow the camera", and that is now the
        # default. It used to default to 30 and vision.launch.py passed its own
        # `fps` arg (also defaulting to 30) into it, so a NAMED PROFILE's fps was
        # silently overridden by a launch default: `camera:=pi_forward` asks for
        # 210 and published 29.999 Hz. Nothing was wrong anywhere -- the camera
        # ran at 210, the detector could do 70, and the node ticked at 30 in
        # between. Measured: fixing it took detections 30.0 -> 51.4 Hz and
        # image_raw 30.0 -> 71.2 Hz on the same hardware.
        rate = float(self.get_parameter('publish_rate_hz').value)
        pinned = rate > 0
        if rate <= 0:
            rate = float(self._info.get('fps') or 30.0)
            if self._log_rate_source:
                self.get_logger().info(
                    f'[CAM  ] publish rate follows the camera: {rate:.1f} Hz '
                    f'(set publish_rate_hz to pin it)')
        # A positive publish_rate_hz THROTTLES; it no longer paces. The
        # capture thread publishes on arrival, so this is a ceiling for the
        # cases that genuinely want one (a remote viewer, a bandwidth cap) and
        # 0 -- the default -- means "as fast as the camera produces".
        self._min_period = (1.0 / rate) if pinned else 0.0
        self._last_pub = 0.0

        self._sent    = 0
        self._dropped = 0
        self._last_log = time.monotonic()
        self.create_timer(2.0, self._log_health)

        # Background capture thread — keeps read() off the ROS executor AND
        # owns the publish, so a frame goes out the moment it exists.
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
            # An explicit positive fps overrides the profile, 0 leaves it
            # alone. This is the only way to cap a camera without editing a
            # checked-in config, and capping matters here: the forward camera
            # delivers 68 Hz raw against a detector that consumes ~50, and the
            # surplus starves the OTHER camera through USB/CPU contention.
            fps_override = int(self.get_parameter('fps').value)
            if fps_override > 0:
                profile['fps'] = fps_override
                # WARN, not info. The launch files pin the process default to
                # `warn`, so an info line here is invisible -- and a silently
                # capped camera is the exact round-26 defect, which cost a
                # round precisely because nothing said the rate had been
                # changed. It sits beside the camera's own "delivering X
                # against Y requested" warning, which is the same class of
                # notice from the other direction.
                self.get_logger().warning(
                    f'[CAM  ] fps override → {fps_override} '
                    f"(profile asked {get_profile(profile_name).get('fps')}). "
                    f'Deliberate: a camera outrunning its detector spends CPU '
                    f'decoding frames the detector drops.')
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

        # `fps` now defaults to 0, meaning "the profile decides". This branch has
        # NO profile to decide, so 0 would ask the driver for zero frames per
        # second. Fall back to the old default here rather than let a sentinel
        # leak into a device call.
        fps_param = int(self.get_parameter('fps').value)
        fps_explicit = fps_param if fps_param > 0 else 30

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
                fps=fps_explicit,
            )
        elif source == 'ros_topic':
            kwargs.update(
                node=self,
                topic=str(self.get_parameter('topic').value).strip(),
                expected_width=int(self.get_parameter('width').value),
                expected_height=int(self.get_parameter('height').value),
                expected_fps=float(fps_explicit),
            )
        elif source == 'video_file':
            loop_val = self.get_parameter('loop').value
            kwargs.update(
                path=str(self.get_parameter('path').value).strip(),
                loop=bool(loop_val) if not isinstance(loop_val, str) else loop_val.lower() != 'false',
                width=int(self.get_parameter('width').value),
                height=int(self.get_parameter('height').value),
                fps=fps_explicit,
            )

        return make_camera(source, logger=self.get_logger(), **kwargs)

    # How long to wait after a source reports "nothing new". A source that
    # already owns a keep-up thread (the v4l2 mailbox) answers instantly and
    # truthfully, so polling it costs a lock and this wait sets the extra
    # latency: at the old 20 ms it added up to a frame period of pure sleep on
    # top of a pipeline built to remove exactly that. One millisecond is short
    # against a 16 ms frame and long enough not to spin a core.
    #
    # It cannot be zero: `WebcamCamera` and the file source BLOCK inside
    # read(), so for them this is a backstop, not the pacer.
    _IDLE_WAIT_S = 0.001

    def _capture_loop(self):
        """Daemon thread: read the newest frame, hand it to the timer.

        NOT a buffer. The queue below is one deep and drained before every put,
        so it is a handoff between this thread and the ROS timer -- the same
        mailbox discipline the v4l2 source uses against the driver. A frame
        that arrives while the timer is busy REPLACES the one waiting; it never
        queues behind it.
        """
        try:
            while rclpy.ok():
                frame, meta = self._cam.read()
                if frame is None or not meta.fresh:
                    # `fresh=False` is the honest answer to "asked faster than
                    # the camera produces", not an error and not a pause.
                    time.sleep(self._IDLE_WAIT_S)
                    continue
                self._publish(frame, meta)
        except Exception:
            pass  # camera released during shutdown

    def _publish(self, frame, meta):
        """Publish the frame that just arrived, on the thread that read it.

        NOT on a timer. A fixed-rate timer draining a one-deep slot samples it
        at an arbitrary phase, so a frame waits on average half a timer period
        for no reason -- measured on this hardware as 15.4 ms of age at the
        slot against 23.8 ms after a 60 Hz timer, i.e. 8.4 ms of pure sitting
        still, on a pipeline built to remove exactly that.

        It also removes the queue between the two: a slot plus a timer IS a
        queue, just a shallow one. The publish is now driven by the event that
        matters (a frame exists) rather than by a clock that has no way to know
        when that happened.

        The rate is therefore the camera's, which is what `publish_rate_hz<=0`
        already meant. A positive `publish_rate_hz` still throttles, because
        deliberately publishing slower than the camera is a real request (a
        remote viewer, a bandwidth cap) -- it just no longer sets the floor.
        """
        if self._min_period > 0.0:
            now = time.monotonic()
            if now - self._last_pub < self._min_period:
                return
            self._last_pub = now

        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f"[CAM  ] cv_bridge encode failed: {exc!r}")
            self._dropped += 1
            return

        # THE CAPTURE TIME, NOT NOW(). This stamped `now()` at PUBLISH time,
        # which makes the header say the frame is 0 ms old at the instant it
        # leaves the node -- no matter how long it sat in the driver's queue or
        # this node's own. Everything downstream that reasons about age reads
        # this field: `_freshness` decays the lateral command by it, the coast
        # ladder is measured in it, `is_new_frame` gates the mid-hold fire on
        # it. All of them were therefore measuring age-since-publish and were
        # structurally blind to the queueing latency, which is the part that
        # actually grows when the vehicle is busy.
        #
        # `meta.stamp_wall` is the capture instant on the wall clock, derived
        # by the source from the kernel's monotonic capture stamp. Sources that
        # cannot know better (a video file, a ROS topic) set it at read time,
        # which is the best available answer there and no worse than before.
        img_msg.header.stamp    = Time(
            seconds=int(meta.stamp_wall),
            nanoseconds=int((meta.stamp_wall % 1.0) * 1e9)).to_msg()
        img_msg.header.frame_id = self._frame_id

        info = CameraInfo()
        info.header = img_msg.header
        info.width  = int(meta.width  or img_msg.width)
        info.height = int(meta.height or img_msg.height)
        self._fill_calibration(info)

        self._pub_img.publish(img_msg)
        self._pub_info.publish(info)
        self._sent += 1

    def _load_calibration(self) -> dict | None:
        """Load intrinsics measured by tools/fov_calibrate.py, if any.

        Returns None when unset or unreadable -- a missing calibration must
        never stop the camera. It is published as a WARN, not an exception,
        because a vehicle that will not stream video is worse than one that
        streams video without K.
        """
        path = str(self.get_parameter('calibration').value or '').strip()
        if not path:
            return None
        try:
            import json
            with open(os.path.expanduser(path)) as fh:
                c = json.load(fh)
            K = c['camera_matrix']; D = c['distortion_coefficients']
            cal = {
                'K': [float(v) for row in K for v in row],
                'D': [float(v) for v in D],
                'w': int(c['image_width']), 'h': int(c['image_height']),
                'hfov': float(c.get('hfov_deg_air', 0.0)),
            }
            self.get_logger().info(
                f"[CAM  ] calibration {os.path.basename(path)}: "
                f"fx={cal['K'][0]:.1f} fy={cal['K'][4]:.1f} "
                f"HFOV={cal['hfov']:.1f} deg (measured at {cal['w']}x{cal['h']})")
            return cal
        except Exception as exc:
            self.get_logger().warn(
                f"[CAM  ] calibration {path!r} not usable ({exc}); "
                "publishing size-only CameraInfo")
            return None

    def _fill_calibration(self, info: CameraInfo) -> None:
        """Populate K/D/P, rescaled if streaming at a different resolution.

        Intrinsics scale LINEARLY with resolution, so a calibration taken at
        1280x720 is valid at 640x360 -- but only after scaling. Publishing the
        unscaled matrix would put the principal point off the image and every
        derived angle would be wrong by 2x, silently.
        """
        cal = self._calib
        if cal is None:
            return
        sx = info.width / cal['w']
        sy = info.height / cal['h']
        K = list(cal['K'])
        K[0] *= sx; K[2] *= sx        # fx, cx
        K[4] *= sy; K[5] *= sy        # fy, cy
        info.k = K
        info.d = list(cal['D'])
        info.distortion_model = 'plumb_bob'
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0], 0.0, K[2], 0.0,
                  0.0, K[4], K[5], 0.0,
                  0.0, 0.0, 1.0, 0.0]

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
