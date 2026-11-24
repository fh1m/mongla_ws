#!/usr/bin/env python3
"""camera_node -- read from a Camera and publish standard ROS image topics.

One camera per node instance. Pick the source via the `profile` ROS param
(matched against `CAMERA_PROFILES`) OR by passing `source` + source-specific
overrides. The node publishes:

  /mongla/vision/<cam>/image_raw     sensor_msgs/Image      (bgr8)
  /mongla/vision/<cam>/camera_info   sensor_msgs/CameraInfo (K/D from the
                                                              calibration bound
                                                              to the profile;
                                                              size only without)
  /mongla/vision/<cam>/stream/compressed   sensor_msgs/CompressedImage  (the
  /mongla/vision/<cam>/preview/compressed  camera's own JPEG, 30 and 5 Hz;
                                           encoded only for a source with none)

Examples
--------
# Laptop webcam by named profile
ros2 run mongla_vision camera_node --ros-args -p profile:=laptop

# Same thing, fully explicit
ros2 run mongla_vision camera_node --ros-args \\
    -p source:=webcam -p device:=0 -p width:=640 -p height:=480 -p fps:=30 -p name:=laptop

# Subscribe to any Image topic (e.g. a bag replay) and re-publish under our namespace
ros2 run mongla_vision camera_node --ros-args \\
    -p source:=ros_topic -p topic:=/replay/forward/image_raw -p name:=forward
"""

import math
import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header
from std_msgs.msg     import Float32, Int32
from std_srvs.srv     import SetBool
from mongla_vision import image_msg

from mongla_vision import qos
from mongla_vision import (
    CAMERA_PROFILES,
    make_camera, make_camera_from_profile,
    get_profile,
)
from mongla_vision.calibration.binding import calibration_for_profile
from mongla_vision.cameras.discover import discover_cameras
from mongla_vision.cameras.reopen import ABSENT, GAVE_UP, REOPEN, CameraWatch
from mongla_runtime.tether import changes


# The browser's quality for a frame we encode ourselves; a camera's own JPEG
# is never re-encoded.
STREAM_JPEG_QUALITY = 80


def _encode_jpeg(frame) -> bytes:
    """A decoded frame as JPEG for the video topics, or b'' if it will not encode."""
    import cv2
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    return buf.tobytes() if ok else b''


def advance_slot(last: float, period: float, now: float) -> float:
    """The slot a publish at `now` fills, for a gate that is due at `last + period`.

    Kept on a fixed schedule. Stamping the slot with the publish time instead
    makes every period one frame-wait longer than asked, because a frame only
    arrives some time after the slot opens: a 30 Hz cap on a 210 Hz camera
    gave 27. A gate more than a period behind (the first frame, a stall)
    restarts from now rather than bursting to catch up.
    """
    return now if now - last >= 2.0 * period else last + period


class CameraNode(Node):
    def __init__(self, node_name: str = 'mongla_camera', *,
                 parameter_overrides=None, frame_sink=None):
        """`node_name` / `parameter_overrides` mirror `DetectorNode`.

        Launch implements `name=` and `parameters=` as PROCESS-WIDE
        remappings (`__node:=`, `__params:=`), so they cannot address two
        nodes in one process. A composed launcher therefore holds prefixed
        parameters and hands them down here, exactly as `detector_dual_node`
        already does for the detectors.

        `frame_sink` is the composed consumer -- an object with
        `wants_frame()` and `submit_frame(frame, header)`. When present the
        decoded frame is handed to it directly, skipping the serialise, the
        transport and the second decode. The image topic is still published,
        because the HUD, Batighor and any recorder read it; composition takes
        the topic out of the CONTROL path, it does not delete it.
        """
        super().__init__(node_name,
                         parameter_overrides=list(parameter_overrides or []))
        self._sink = frame_sink

        self.declare_parameter('profile',         '')        # e.g. 'forward' / 'laptop'
        self.declare_parameter('source',          '')        # explicit override
        self.declare_parameter('name',            '')
        self.declare_parameter('topic',           '')        # for source=ros_topic
        self.declare_parameter('device',          -1)        # -1 = use profile default; ≥0 overrides
        # PORT-STABLE identity for identical cameras (same VID/PID): a /dev/v4l/by-path/…
        # symlink pins the camera to a physical USB PORT, so 'forward'/'downward' never
        # swap on reboot/re-enumeration (unlike /dev/videoN indices). Non-empty wins over
        # `device`. See .claude/context/perception/dual-camera-setup.md.
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
        # Path to a calibration.json from calibration/solver.py. Empty (the
        # default) keeps the historical behaviour EXACTLY: size-only CameraInfo
        # with K and D left zero. Nothing changes until a real calibration
        # exists for this camera.
        self.declare_parameter('calibration',     '')
        # 0 = follow the camera's own fps (see the timer below). A positive
        # value pins the publish rate regardless of what the camera delivers.
        self.declare_parameter('publish_rate_hz', 0)
        # The operator's video: the camera's own JPEG, republished unchanged
        # on two topics, one a link can carry at full motion and one any link
        # can (Batighor's layouts pick by link). A browser cannot show more
        # than its display rate, and the downward camera makes 210 frames a
        # second, so both are capped. 0 turns that topic off.
        self.declare_parameter('stream_rate_hz',  30.0)
        self.declare_parameter('preview_rate_hz', 5.0)
        self.declare_parameter('path',            '')        # for source=video_file
        self.declare_parameter('loop',            True)      # for source=video_file
        self.declare_parameter('discover_on_start', False)   # log USB camera table
        # The two settings a mission or the deck changes while frames flow.
        # `exposure_us` is a shutter in MICROSECONDS (0 = the camera's own
        # auto exposure); `controls` is 'name=value, ...' in the names
        # `v4l2-ctl --list-ctrls` prints, on top of the profile's own. Both
        # apply live on the V4L2 source and are read back; see _on_set.
        self.declare_parameter('exposure_us',     0)
        self.declare_parameter('controls',        '')

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
        self.add_on_set_parameters_callback(self._on_set)
        self._log_rate_source = True
        self._info    = self._cam.info()
        self._cam_name = str(self._info.get('name') or 'cam')
        self._frame_id = str(self._info.get('frame_id') or self._cam_name)

        ns = f'/mongla/vision/{self._cam_name}'
        # A mailbox, not a queue -- the DDS half of the same problem the
        # capture side has. An int depth here means KEEP_LAST at that depth
        # with RELIABLE reliability, so a slow subscriber makes the middleware
        # HOLD frames and retransmit them: the consumer then works through a
        # backlog of old pictures, which is precisely the failure the mailbox
        # capture path exists to prevent, reintroduced one layer up.
        #
        # Both policies, and the reasoning behind them, live in `qos.py` --
        # which every subscriber imports too, so the two ends of a link cannot
        # disagree about a topic they both name.
        self._img_qos = qos.IMAGE
        self._pub_img  = self.create_publisher(Image, f'{ns}/image_raw',
                                               self._img_qos)
        self._pub_info = self.create_publisher(
            CameraInfo, f'{ns}/camera_info', qos.CAMERA_INFO)
        self._jpeg_out = [
            [self.create_publisher(CompressedImage, topic, qos.IMAGE),
             1.0 / rate if rate > 0 else None, -1e9]
            for topic, rate in (
                (f'{ns}/stream/compressed',
                 float(self.get_parameter('stream_rate_hz').value)),
                (f'{ns}/preview/compressed',
                 float(self.get_parameter('preview_rate_hz').value)))]
        self._calib = self._load_calibration()

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
        if pinned:
            rate_txt = f'{rate:.1f} Hz (pinned)'
        else:
            # What the driver REPORTS, which a Fantech asked for 60 answers
            # while delivering 15 (config.py): the health line measures it.
            reported = float(self._info.get('fps') or 0.0)
            rate_txt = (f'the camera\'s rate (driver reports {reported:.1f} Hz; '
                        f'measured on the health line)' if reported > 0
                        else 'the camera\'s rate (unreported; measured on the health line)')
            if self._log_rate_source:
                self.get_logger().info(
                    f'[CAM  ] publish rate follows {rate_txt} '
                    f'(set publish_rate_hz to pin it)')
        # A positive publish_rate_hz THROTTLES; it never paces. The
        # capture thread publishes on arrival, so this is a ceiling for the
        # cases that genuinely want one (a remote viewer, a bandwidth cap) and
        # 0 -- the default -- means "as fast as the camera produces".
        self._min_period = (1.0 / rate) if pinned else 0.0
        self._last_pub = 0.0

        # CUMULATIVE, never reset. Written ONLY by the capture thread; the
        # health timer keeps its own high-water marks below (B41).
        self._sent    = 0
        self._dropped = 0
        self._last_sent    = 0
        self._last_dropped = 0
        self._last_log = time.monotonic()
        self.create_timer(2.0, self._log_health)

        # Background capture thread — keeps read() off the ROS executor AND
        # owns the publish, so a frame goes out the moment it exists.
        # Held by the capture thread while it swaps a lost camera, and by a
        # control set from the executor: a set mid-reopen would ioctl a
        # closed (or reused) descriptor (vl_6 F5).
        self._cam_lock = threading.Lock()
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
            f"{ns}/image_raw  @ {rate_txt}")

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
            if profile.get('source') in ('v4l2', 'webcam'):
                profile.update(self._asked_controls(profile.get('controls')))
            if fps_override > 0:
                profile['fps'] = fps_override
                # Only when there is an override: a warning with none would
                # report a rate change that did not happen. WARN, not info. The launch files pin the process default to
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
            # A profile's device_path names a udev symlink that exists on ONE
            # host. `pi_forward` is the Pi box's Fantech; `forward` is a
            # generic USB role. Running the wrong one for the host
            # otherwise fails inside the V4L2 open with no mention of the
            # profile, which is the single most likely way to lose a pool
            # session to a launch argument.
            probe = device_path or profile.get('device')
            if isinstance(probe, str) and probe.startswith('/dev/') \
                    and not os.path.exists(probe):
                self.get_logger().error(
                    f'[CAM  ] profile {profile_name!r} wants {probe}, which does '
                    f'not exist on this host. That device is a udev symlink '
                    f'created per vehicle -- a Pi profile (pi_*) on a dev box, '
                    f'or the reverse, gives exactly this. Pass the profile for '
                    f'the host you are on: bringup.launch.py camera_profile:=...')
            # device_path (by-path symlink) > int device override > profile default.
            #
            # BOTH KEYS, and the bug this fixes was exactly one of them. A
            # profile carries its own `device_path` (`config.py`: pi_forward is
            # `/dev/mongla_cam_forward`). Overriding only `device` left that
            # entry untouched, so the v4l2 mailbox -- which reads `device_path`
            # -- kept opening the profile's udev symlink while this very line
            # logged the override as applied. Measured on the vehicle:
            #
            #   [CAM ] device_path (port-stable) -> /dev/video0
            #   [CAM ] v4l2 mailbox failed on '/dev/mongla_cam_forward'
            #
            # A parameter that is read, logged and then ignored is worse than
            # one that is unsupported: the log says it worked.
            if device_path:
                profile['device'] = device_path
                profile['device_path'] = device_path
                self.get_logger().info(f'[CAM  ] device_path (port-stable) → {device_path}')
            else:
                dev_param = self.get_parameter('device').value
                try:
                    dev_int = int(dev_param)
                except (TypeError, ValueError):
                    dev_int = -1
                if dev_int >= 0:
                    # Both keys again: the profile's own device_path would
                    # otherwise win in the factory over the index asked for.
                    profile['device'] = dev_int
                    profile.pop('device_path', None)
                    self.get_logger().info(f'[CAM  ] device override → index {dev_int}')
            # width/height/frame_id belong to the non-profile path; a size
            # asked for that the profile does not have is said, not dropped.
            asked = (int(self.get_parameter('width').value),
                     int(self.get_parameter('height').value))
            has = (profile.get('width'), profile.get('height'))
            if asked != (640, 480) and None not in has and asked != has:
                self.get_logger().warning(
                    f'[CAM  ] width/height {asked[0]}x{asked[1]} are not read '
                    f'with profile {profile_name!r}, which runs at '
                    f'{has[0]}x{has[1]}: edit the profile to change it')
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
        # `v4l2` is the mailbox with the camera's own JPEG, at any size; the
        # calibration launch uses it at the size a calibration is taken at.
        if source in ('webcam', 'v4l2'):
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
                **self._asked_controls(None),
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

    def _asked_controls(self, base, exposure_us=None, controls=None):
        """The profile's controls with this node's parameters on top, as the
        builder's `controls` and `exposure_us` keywords."""
        from .cameras.v4l2_mailbox import parse_controls
        text = self.get_parameter('controls').value if controls is None else controls
        us = self.get_parameter('exposure_us').value if exposure_us is None else exposure_us
        return {'controls': {**dict(base or {}), **parse_controls(text)},
                'exposure_us': int(us)}

    # Applied while streaming: the two the V4L2 camera can change without a
    # reopen. Everything else is read once, and a live set of it is refused
    # with the reason, because `ros2 param set` and `set_node` report success
    # for any declared name and the camera would carry on unchanged.
    LIVE_PARAMS = ('exposure_us', 'controls', 'stream_rate_hz', 'preview_rate_hz')
    # The viewers' JPEG topics, in `_jpeg_out` order. A rate is only a period
    # on the publisher, so it applies on any source; Batighor's helm lowers
    # `stream` while a browser's link lags (mongla_batighor/link.py).
    JPEG_RATES = ('stream_rate_hz', 'preview_rate_hz')

    def _on_set(self, params):
        from rcl_interfaces.msg import SetParametersResult
        from .cameras.v4l2_mailbox import exposure_controls
        params = changes(self, params)     # a declaration is not a change (B-145)
        if not params:
            return SetParametersResult(successful=True)
        fixed = [p.name for p in params if p.name not in self.LIVE_PARAMS]
        if fixed:
            return SetParametersResult(successful=False, reason=(
                f'{", ".join(fixed)}: read once when the camera opens -- set it '
                f'under `nodes:` in bringup.yaml and restart'))
        new = {p.name: p.value for p in params}
        periods = {}
        for name in self.JPEG_RATES:
            if name not in new:
                continue
            v = new.pop(name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not math.isfinite(v) or v < 0:
                return SetParametersResult(successful=False, reason=(
                    f'{name} must be a rate in Hz, 0 for off (got {v!r})'))
            periods[self.JPEG_RATES.index(name)] = 1.0 / v if v > 0 else None
        if not new:
            for i, period in periods.items():
                self._jpeg_out[i][1] = period
            return SetParametersResult(successful=True)
        if not hasattr(self._cam, 'set_controls'):
            return SetParametersResult(successful=False, reason=(
                f'the {self._cam.source_kind} source cannot change camera '
                f'controls while streaming; only the V4L2 source can'))
        try:
            asked = self._asked_controls(None, new.get('exposure_us'), new.get('controls'))
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        want = {**asked['controls'],
                **exposure_controls(asked['exposure_us'], self._cam.auto_mode)}
        if not self._cam_lock.acquire(blocking=False):
            return SetParametersResult(successful=False, reason=(
                'the camera is being reopened; set it again in a moment'))
        try:
            got = self._cam.set_controls(want)
        finally:
            self._cam_lock.release()
        wrong = {k: (v, got.get(k)) for k, v in want.items() if got.get(k) != v}
        if wrong:
            return SetParametersResult(successful=False, reason=(
                'the camera did not take ' + ', '.join(
                    f'{k}={a} (reads {b})' for k, (a, b) in wrong.items())))
        for i, period in periods.items():
            self._jpeg_out[i][1] = period
        return SetParametersResult(successful=True)

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
        # The source says what it is; a fake without a kind is never reopened.
        watch = CameraWatch(str(getattr(self._cam, 'source_kind', '')))
        next_watch = 0.0
        try:
            while rclpy.ok():
                now = time.monotonic()
                if now >= next_watch:
                    next_watch = now + self._WATCH_PERIOD_S
                    if not self._watch_camera(watch, now):
                        return
                # The rate gate comes before the read, because `read()` is
                # where the JPEG is DECODED.
                #
                # It used to sit inside `_publish`, i.e. AFTER the decode, so
                # at 210 fps capture and a 40 Hz publish this thread decoded
                # 210 frames a second and discarded 170 of them -- measured at
                # 55 % of a core for a node whose useful work is ~10 %. The
                # round-31 note claiming "the mailbox decouples capture from
                # DECODE" was true of the SOURCE and false of this loop, which
                # was the only reader and read at capture rate.
                #
                # Skipping the read is safe and is the whole point of a
                # mailbox: an unread frame is simply replaced, so waiting for
                # the slot and then reading yields the NEWEST frame rather than
                # the one that happened to finish decoding.
                # TWO reasons to decode a frame, and the first one is why
                # composing the two nodes is worth anything:
                #
                #   the SINK is idle    -> it will consume this frame NOW, so
                #                          it gets the newest picture the
                #                          instant inference goes free, rather
                #                          than whichever one the publisher's
                #                          clock happened to hand over
                #   the publish slot is due -> the topic's viewers want one
                #
                # When neither holds there is nothing to decode for, and a
                # decode nobody reads is 2.45 ms of a core.
                wanted = self._sink is not None and self._sink.wants_frame()
                wait = 0.0 if wanted else self._time_to_next_slot()
                if wait > 0.0:
                    # Capped so shutdown stays responsive; a long publish
                    # period must not make Ctrl-C wait for it.
                    time.sleep(min(wait, self._IDLE_WAIT_S * 10))
                    continue
                frame, meta = self._cam.read()
                if frame is None or not meta.fresh:
                    # `fresh=False` is the honest answer to "asked faster than
                    # the camera produces", not an error and not a pause.
                    # Woken by the next frame where the source can say when
                    # one lands (a 1 ms poll here measured as a GIL convoy
                    # once a consumer wanted every frame); capped so Ctrl-C
                    # stays responsive.
                    wait_new = getattr(self._cam, 'wait_new', None)
                    if wait_new is not None:
                        wait_new(self._IDLE_WAIT_S * 50)
                    else:
                        time.sleep(self._IDLE_WAIT_S)
                    continue
                if wanted:
                    self._sink.submit_frame(frame, self._make_header(meta))
                if self._time_to_next_slot() <= 0.0:
                    self._publish(frame, meta)
        except Exception as exc:
            # NOT a bare pass. This used to swallow everything as "camera
            # released during shutdown", so a programming error in the loop --
            # a missing attribute, a bad call -- killed the capture thread and
            # the node went on looking healthy while publishing nothing. Found
            # exactly that way: a test fixture omitted `_sink` and the loop
            # died silently instead of raising.
            if rclpy.ok():
                self.get_logger().error(
                    f'[CAM  ] capture loop died: {exc!r} -- this node is now '
                    f'publishing NOTHING')

    # How often the capture thread asks whether its camera is lost.
    _WATCH_PERIOD_S = 0.5

    def _watch_camera(self, watch: CameraWatch, now: float) -> bool:
        """Reopen a lost device camera (`cameras/reopen.py`). False: stop the loop.

        Runs on the capture thread, which owns `self._cam`, so the swap never
        races a `read()`; the swap holds `_cam_lock`, so it never races a
        control set from the executor either.
        """
        if not watch.enabled:          # not a device: nothing to reopen
            return True
        kind = self._cam.source_kind
        device = str(self._cam.info().get('device') or '')
        # A path that is gone means the lead is out: not an attempt. An int
        # index cannot be checked, so it counts as present.
        present = not device.startswith('/dev/') or os.path.exists(device)
        verdict = watch.step(self._cam.is_healthy(), now, present=present)
        if verdict == ABSENT and not getattr(self, '_absent_logged', False):
            self._absent_logged = True
            self.get_logger().error(
                f'[CAM  ] ERROR: {self._cam_name}: {device} is GONE (lead out, or '
                f'the hub re-enumerating). Publishing nothing; it reopens when '
                f'the device comes back')
        elif verdict != ABSENT:
            self._absent_logged = False
        if verdict == REOPEN:
            self.get_logger().warning(
                f'[CAM  ] WARNING: {self._cam_name} delivered nothing for '
                f'{watch.lost_after_s:.0f} s past its health limit; reopening '
                f'({watch.reopens}/{watch.max_reopens})')
            with self._cam_lock:
                return self._reopen(kind)
        elif verdict == GAVE_UP:
            self.get_logger().error(
                f'[CAM  ] ERROR: {self._cam_name} LOST: {watch.max_reopens} '
                f'reopens failed; it stays down for this run and publishes NOTHING')
            return False
        return True

    def _reopen(self, kind) -> bool:
        """Close the lost camera and swap in a fresh one of the same kind."""
        try:
            self._cam.close()
        except Exception as exc:   # noqa: BLE001 -- the device is already gone
            self.get_logger().debug(f'[CAM  ] close of the lost camera: {exc!r}')
        try:
            fresh = self._build_camera()
        except Exception as exc:   # noqa: BLE001 -- counted as a failed attempt
            self.get_logger().error(f'[CAM  ] reopen failed: {exc}')
            return True
        if fresh.source_kind != kind:
            # The factory falls back to the OpenCV source when the mailbox
            # will not open -- right on a dev box, wrong here: 396 ms stale
            # against 17 ms (measured-bars §3), for the rest of the run.
            self.get_logger().error(
                f'[CAM  ] reopen gave a {fresh.source_kind} source, not the '
                f'{kind} it replaces; refused, counted as a failed attempt')
            try:
                fresh.close()
            except Exception:  # noqa: BLE001
                pass
            return True
        self._cam = fresh
        self.get_logger().warning(
            f'[CAM  ] {self._cam_name} reopened ({kind}); frames resume')
        return True

    def _time_to_next_slot(self) -> float:
        """Seconds until the next publish is due; <=0 means now.

        0.0 whenever no rate is pinned (`publish_rate_hz<=0` = follow the
        camera), so the uncapped path is exactly as it was.
        """
        if self._min_period <= 0.0:
            return 0.0
        return self._min_period - (time.monotonic() - self._last_pub)

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
        # The gate itself now lives in `_capture_loop`, ahead of the decode.
        # Only the bookkeeping remains here, on the path that actually
        # publishes.
        if self._min_period > 0.0:
            self._last_pub = advance_slot(self._last_pub, self._min_period,
                                          time.monotonic())

        # The raw image only for a reader. The composed consumers take frames
        # by reference, so on the vehicle `image_raw` is usually unread, and
        # building a 691 kB message per frame for nobody was paid at camera
        # rate (210 Hz downward).
        img_msg = None
        if self._pub_img.get_subscription_count() > 0:
            try:
                img_msg = image_msg.to_msg(frame, 'bgr8')
            except Exception as exc:
                self.get_logger().warning(f"[CAM  ] image encode failed: {exc!r}")
                self._dropped += 1
                return

        # The capture time, not now(). This stamped `now()` at PUBLISH time,
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
        # cannot know better set it when they read or receive the frame: a video
        # file at read time, a ROS topic at arrival.
        header = self._make_header(meta)

        info = CameraInfo()
        info.header = header
        info.width  = int(meta.width  or frame.shape[1])
        info.height = int(meta.height or frame.shape[0])
        self._fill_calibration(info)

        if img_msg is not None:
            img_msg.header = header
            self._pub_img.publish(img_msg)
        self._pub_info.publish(info)
        self._publish_jpeg(meta, header, frame=frame)
        # A composed consumer that wants intrinsics takes them by reference
        # too (the lock ladder, `submit_info`), rather than subscribing to a
        # topic this same process publishes at camera rate.
        submit_info = getattr(self._sink, 'submit_info', None)
        if submit_info is not None:
            submit_info(info)
        self._sent += 1

    def _publish_jpeg(self, meta, header, now=None, frame=None):
        """The camera's JPEG, unchanged, to whichever video topic is due.

        Nothing is encoded when the camera sent a JPEG: the bytes are the ones
        the sensor sent, so the cost is a message and a copy. A topic nobody
        subscribes to costs nothing at all.

        A source that hands over decoded pixels (a recorded video, a ROS
        topic, OpenCV's fallback capture) has no JPEG, and once published
        nothing: Batighor showed a bench's footage as an empty panel, and a
        vehicle camera fallen back to OpenCV would have shown the operator
        the same. Its frame is encoded here, once, and only when a topic is
        due and watched.
        """
        if not meta.jpeg and frame is None:
            return
        now = time.monotonic() if now is None else now
        msg = None
        for out in self._jpeg_out:
            pub, period, last = out
            if period is None or now - last < period:
                continue
            if pub.get_subscription_count() == 0:
                continue
            out[2] = advance_slot(last, period, now)
            if msg is None:
                data = meta.jpeg or _encode_jpeg(frame)
                if not data:
                    return
                msg = CompressedImage(header=header, format='jpeg', data=data)
            pub.publish(msg)

    def _make_header(self, meta):
        """The header for one captured frame, built in ONE place.

        The composed path and the topic path must not diverge on this: the
        stamp is the capture instant, and every freshness gate downstream
        reads it. Two constructions of the same header is how they would.
        """
        h = Header()
        h.stamp = Time(
            seconds=int(meta.stamp_wall),
            nanoseconds=int((meta.stamp_wall % 1.0) * 1e9)).to_msg()
        h.frame_id = self._frame_id
        return h

    def _calibration_pairing(self, cal):
        """('match'|'different'|'unverified'|'unknown', connected USB id)."""
        from mongla_vision.calibration.binding import device_usb_id, pairing
        source = str(self.get_parameter('source').value or '').strip()
        if source in ('video_file', 'ros_topic'):
            return 'unknown', ('', '')
        dev = str(self.get_parameter('device_path').value or '').strip()
        if not dev:
            profile = str(self.get_parameter('profile').value or '').strip()
            try:
                dev = str((get_profile(profile) if profile else {}).get('device_path') or '')
            except Exception:
                dev = ''
        if not dev:
            dev = str(self.get_parameter('device').value)
        ident = device_usb_id(dev)
        return pairing(cal, ident), ident

    def _load_calibration(self) -> dict | None:
        """Load intrinsics measured by calibration/solver.py, if any.

        Returns None when unset or unreadable -- a missing calibration must
        never stop the camera. It is published as a WARN, not an exception,
        because a vehicle that will not stream video is worse than one that
        streams video without K.
        """
        path = str(self.get_parameter('calibration').value or '').strip()
        if not path:
            # Nothing passed: resolve from the profile, via each calibration's
            # own `applies_to`. This is what makes `bringup.launch.py` (which
            # goes through vision.launch.py and passes no calibration) publish
            # a real K instead of zeros -- without a camera-to-file map in
            # every launch file. See calibration/binding.py.
            profile = str(self.get_parameter('profile').value or '').strip()
            path = calibration_for_profile(profile)
            if path:
                self.get_logger().info(
                    f"[CAM  ] calibration auto-bound from profile "
                    f"{profile!r}: {os.path.basename(path)}")
            else:
                if profile:
                    self.get_logger().warn(
                        f"[CAM  ] no calibration declares applies_to "
                        f"{profile!r}; publishing size-only CameraInfo (k=0). "
                        f"Bearings fall back to an FOV or are refused.")
                return None
        try:
            import json
            with open(os.path.expanduser(path)) as fh:
                c = json.load(fh)
            # A calibration names the profiles it describes. A launch default
            # passes the Fantech's file for the forward slot whatever camera
            # sits there, so a global-shutter unit on the forward port would
            # otherwise publish the Fantech's focal: confident bearings, wrong
            # by a fixed factor (the round-38 defect, binding.py).
            applies = c.get('applies_to') or []
            profile = str(self.get_parameter('profile').value or '').strip()
            if applies and profile and profile not in applies:
                own = calibration_for_profile(profile)
                self.get_logger().warn(
                    f"[CAM  ] calibration {os.path.basename(path)} applies to "
                    f"{applies}, not {profile!r}; "
                    + (f"using {os.path.basename(own)}, which does" if own else
                       "publishing size-only CameraInfo (k=0) rather than "
                       "another camera's lens"))
                if not own:
                    return None
                with open(own) as fh:
                    c = json.load(fh)
                path = own
            if c.get('medium', 'air') != 'air':
                # CameraInfo carries the in-AIR lens; flow, the lock and
                # pnp_node apply the flat port themselves. A fit made with the
                # board in water already contains the port and would be
                # refracted twice: every range off by a factor of n.
                self.get_logger().warn(
                    f"[CAM  ] calibration {os.path.basename(path)} was fitted "
                    f"in {c.get('medium')!r}; CameraInfo carries the air lens, "
                    f"so it is not used. Publishing size-only CameraInfo (k=0)")
                return None
            # Only the unit it was measured on: a lens fitted on another
            # camera gives confident bearings wrong by a fixed factor, which
            # is worse than none (vl_5 F6, vl_6 F9). Forward is the Fantech
            # C30 and downward the Sonix global shutter (owner, 2026-11-24).
            verdict, dev_id = self._calibration_pairing(c)
            if verdict == 'different':
                self.get_logger().warn(
                    f"[CAM  ] calibration {os.path.basename(path)} was measured "
                    f"on USB {c.get('usb_vid')}:{c.get('usb_pid')}, and this "
                    f"camera is {dev_id[0]}:{dev_id[1]}. Not used: publishing "
                    f"size-only CameraInfo (k=0). Calibrate this unit.")
                return None
            if verdict == 'unverified':
                self.get_logger().warn(
                    f"[CAM  ] calibration {os.path.basename(path)} recorded no "
                    f"USB id, so it cannot be proved to be this unit's "
                    f"({dev_id[0]}:{dev_id[1]}). Used; recalibrate to bind it.")
            K = c['camera_matrix']; D = c['distortion_coefficients']
            cal = {
                'K': [float(v) for row in K for v in row],
                'D': [float(v) for v in D],
                'w': int(c['image_width']), 'h': int(c['image_height']),
                'hfov': float(c.get('hfov_deg_air', 0.0)),
            }
            ere, bar = c.get('max_ere_px'), c.get('max_ere_bar_px')
            if isinstance(ere, (int, float)) and isinstance(bar, (int, float)) \
                    and ere > bar:
                # Published, because the lock has no better K, but never quietly.
                self.get_logger().warn(
                    f"[CAM  ] calibration {os.path.basename(path)} did not pass "
                    f"its own bar: max ERE {ere:.1f} px against {bar:.1f} px. "
                    f"Ranges from this K carry that error; recalibrate.")
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
        # Scaling holds only at the calibration's own aspect ratio. At another
        # (4:3 from a 16:9 calibration) a sensor crops rather than squashes,
        # so a scaled K would be a confident, wrong lens: publish size only.
        if abs(sx - sy) > 0.01 * max(sx, sy):
            if not getattr(self, '_aspect_warned', False):
                self._aspect_warned = True
                self.get_logger().warning(
                    f"[CAM  ] streaming {info.width}x{info.height} against a "
                    f"calibration taken at {cal['w']}x{cal['h']}: another aspect "
                    f"ratio, so the lens cannot be scaled to it; publishing "
                    f"size-only CameraInfo (k=0)")
            return
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
        # Snapshot the cumulative counters ONCE, then work from the snapshot: two
        # reads of a live counter can straddle an increment and disagree.
        tot_sent, tot_dropped = self._sent, self._dropped
        sent = tot_sent - self._last_sent
        dropped = tot_dropped - self._last_dropped
        hz = sent / elapsed
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
            f"sent={sent}  encode_dropped={dropped}{extra}")
        # Delta, not reset (B41). `_sent`/`_dropped` are incremented by the
        # CAPTURE THREAD and this runs on a ROS timer thread. Read-then-zero is
        # two operations: an increment landing between them was silently lost, so
        # this line under-reported drops -- on the one readout an operator uses to
        # decide whether the pipeline is healthy.
        #
        # Fixed by structure rather than by a lock: the counters are now
        # CUMULATIVE and have exactly ONE writer (the capture thread), while this
        # reader keeps its own high-water marks. Single-writer + private reader
        # state needs no mutual exclusion at all, and costs nothing in the
        # publish path. (A lock would also have worked; this is smaller, and it
        # stays correct on a free-threaded build where `+=` is not atomic either.)
        self._last_sent, self._last_dropped = tot_sent, tot_dropped
        self._last_log = now

    def shutdown(self):
        try:
            self._cam.close()
        except Exception as exc:
            self.get_logger().debug(f"camera close ignored: {exc!r}")


def main():
    from mongla_runtime.orphan import die_with_parent
    die_with_parent()        # holds the camera device
    rclpy.init()
    node = CameraNode()
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
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)
