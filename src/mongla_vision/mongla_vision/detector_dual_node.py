"""The whole vision stack in ONE process: two cameras, two detectors, no hop.

Both detectors must share a process because the chip allows one VDevice per
process (below). Once they do, putting the CAMERAS in the same process costs
nothing and removes the last soft target in the frame path.

WHY THIS EXECUTABLE EXISTS
--------------------------
`vision_dual.launch.py` starts two `detector_node` PROCESSES. On the Jetson
that is fine -- CUDA is happy with two contexts. On the Pi + AI HAT+ the second
one dies with `HAILO_OUT_OF_PHYSICAL_DEVICES (74)`, so the two-camera path has
never started on this hardware. Measured, along with the two alternatives that
look like they should work and do not:

    two processes, multi_process_service + scheduler -> InvalidOperation
        (the `hailort_service` daemon it needs is not installed)
    one process, ROUND_ROBIN scheduler, two graphs   -> SIGSEGV
    one process, one VDevice, both graphs, taking turns -> 98.2 Hz

So: one process, two `DetectorNode` objects, the shared device and activation
arbiter in `detection/hailo.py`.

WHY THE CAMERAS JOINED THEM
---------------------------
With them in the same process the detector no longer receives frames on a
topic. It gets the decoded array handed straight across, which removes:

    cv_bridge encode   0.845 ms
    serialise          1.469 ms
    transport          1.760 ms
    imgmsg_to_cv2      a second full-frame copy

...and, worth more than all of it, THE PUBLISHER'S CLOCK. On the topic path
the detector acts on whichever frame the 40 Hz publish throttle handed over;
here `CameraNode` reads `DetectorNode.wants_frame()` and decodes at the moment
inference goes idle, so the picture is the newest one that exists.

`ComposableNodeContainer` would buy NONE of this: rclpy has no intra-process
comms (that is rclcpp), so composed Python nodes still traverse rmw. The gain
comes from the direct Python reference, not from ROS composition.

The image topic is still published, throttled, for the HUD, the console,
`web_video_server` and recorders. Composition takes the topic out of the
CONTROL path; it does not delete it.

WHAT IS DELIBERATELY UNCHANGED, PART TWO
----------------------------------------
The camera nodes keep their names (`mongla_camera_forward` / `_downward`) and
their topics, so `vision_display`, `mission_web` and `preflight` see exactly
what they saw before.

WHAT IS DELIBERATELY UNCHANGED
------------------------------
The node NAMES (`mongla_detector_forward` / `mongla_detector_downward`), every
topic, and every parameter. Missions, `ClassRef`, `vision_state`, the mission
console's `SetParameters` calls and the `active_camera` latch all address the
detector by node name, so collapsing the two into one node would have been a
much larger change for no benefit. Two nodes in one process is the smaller
edit and the invisible one.

Launch's `name=` and `parameters=` are PROCESS-WIDE remappings (`__node:=`,
`__params:=`) and cannot address two nodes in one process -- which is why the
parameters arrive here `fwd_`/`dwn_`-prefixed on this launcher and are handed
down as `parameter_overrides` rather than declared per node by launch.

A MultiThreadedExecutor, because the two image callbacks must not serialise
behind each other: at ~35 Hz per camera under contention a single-threaded
executor makes the second camera wait a whole inference for the first.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from .camera_node   import CameraNode
from .detector_node import DetectorNode

# Parameters that are per-camera. Anything not here is shared by both, which is
# right for the ones that describe the machine (device, imgsz, half) and for the
# ones an operator sets globally (max_det, debug_image_hz, paused).
_PER_CAMERA = ('model_path', 'models', 'active_model', 'classes', 'conf',
               'model_conf', 'image_topic')
_SHARED = ('device', 'half', 'iou', 'imgsz', 'max_det', 'publish_contours',
           'publish_debug_image',
           'debug_image_hz', 'alignment_deadband', 'paused',
           'preprocess', 'preprocess_clip', 'vision_profile',
           'range_crop')

# The camera half. Same prefixing scheme, same reason: launch cannot address
# two nodes in one process.
#
# `device` IS IN THIS LIST BECAUSE OF A NAME COLLISION, not because anyone
# needs to set it. Launch parameters are PROCESS-wide, and `device` means two
# different things to the two kinds of node in this process: the detector's
# backend (`'cpu'`, a string) and the camera's V4L2 index (`-1`, an integer).
# Without an explicit per-camera override the process-wide `device: 'cpu'`
# reaches `CameraNode.declare_parameter('device', -1)` and the WHOLE PROCESS
# dies at startup with InvalidParameterTypeException. Prefer `device_path`
# for identity regardless -- a raw index has already swapped the two cameras
# across a reboot once.
_CAM_PER_CAMERA = ('profile', 'device', 'device_path', 'calibration', 'fps',
                   'publish_rate_hz', 'width', 'height')
_CAM_DEFAULTS = {
    'profile': '', 'device': -1, 'device_path': '', 'calibration': '',
    'fps': 0,
    # A composed detector is fed on demand, so this rate now governs only the
    # topic's viewers -- not the control path, which is what it used to gate.
    'publish_rate_hz': 0, 'width': 640, 'height': 480,
}

_DEFAULTS = {
    'model_path': 'yolov11n', 'models': '', 'active_model': '', 'classes': '',
    # 0.15, NOT the 0.35 every other launch path ships. The HEFs are baked at
    # 0.05 exactly so this point is reachable, and INT8 costs ~0.08 of score
    # without moving the box -- so the CUDA number is ~3x too tight here and is
    # the most likely reason a detection is missed. `hailo.py` warns if it is
    # raised back.
    'conf': 0.15, 'model_conf': '', 'image_topic': '',
    'device': 'cuda:0', 'half': True, 'iou': 0.5, 'imgsz': 640,
    'max_det': 100, 'publish_contours': True,
    'publish_debug_image': True, 'debug_image_hz': 5.0,
    'alignment_deadband': 0.05, 'paused': False,
    # Underwater contrast enhancement -- see detection/preprocess.py.
    # Off by default: 3.78 ms on the Pi is a real trade.
    'preprocess': 'off', 'preprocess_clip': 3.0,
    # One word for the water -- see detection/profiles.py.
    # -1 = not set (an INT, so a profile can turn it on -- a bool has no
    # third state). The type must match `detector_node.declare_parameter`
    # exactly or rclpy raises InvalidParameterTypeException and the whole
    # composed process dies at startup, which is how this was found.
    'vision_profile': '', 'range_crop': -1,
}


class _Launcher(Node):
    """Holds the prefixed parameters. Not a detector; it owns no topics."""

    def __init__(self):
        super().__init__('mongla_detector_dual')
        for cam in ('fwd', 'dwn'):
            for key in _PER_CAMERA:
                self.declare_parameter(f'{cam}_{key}', _DEFAULTS[key])
        for key in _SHARED:
            self.declare_parameter(key, _DEFAULTS[key])
        for cam in ('fwd', 'dwn'):
            for key in _CAM_PER_CAMERA:
                self.declare_parameter(f'{cam}_cam_{key}', _CAM_DEFAULTS[key])

    def camera_overrides(self, cam: str, camera: str):
        out = [Parameter('name', value=camera),
               Parameter('frame_id', value=camera)]
        for key in _CAM_PER_CAMERA:
            out.append(Parameter(
                key, value=self.get_parameter(f'{cam}_cam_{key}').value))
        return out

    def overrides(self, cam: str, camera: str):
        # `direct_feed`: frames arrive by reference from the camera in this
        # process, so the detector must NOT also subscribe -- it would decode
        # and infer the same picture twice, and the topic copy is the SLOWER
        # of the two, so it would be the one acted on half the time.
        out = [Parameter('camera', value=camera),
               Parameter('direct_feed', value=True)]
        for key in _PER_CAMERA:
            out.append(Parameter(
                key, value=self.get_parameter(f'{cam}_{key}').value))
        for key in _SHARED:
            out.append(Parameter(key, value=self.get_parameter(key).value))
        return out


def main():
    rclpy.init()
    launcher = _Launcher()
    nodes = [launcher]
    live: list[str] = []
    try:
        for cam, camera in (('fwd', 'forward'), ('dwn', 'downward')):
            # Built SEQUENTIALLY on purpose. The second one configures a second
            # network group on the device the first created; doing that from a
            # thread pool is how the registry path already trips over itself.
            det = DetectorNode(
                f'mongla_detector_{camera}',
                parameter_overrides=launcher.overrides(cam, camera))
            # The detector FIRST, so the camera never submits to a half-built
            # sink. `frame_sink=det` is the whole composition -- one Python
            # reference where a topic used to be.
            #
            # ⛔ ONE ABSENT CAMERA USED TO KILL BOTH. CameraNode raises after
            # its five open retries, and the raise walked straight out of this
            # loop: an unplugged downward USB took the FORWARD detector down
            # with it, and the process exited 1. Measured on the vehicle --
            # `/dev/mongla_cam_downward` gone after a reboot, and the whole
            # vision stack refused to start on a hull whose forward camera was
            # working perfectly.
            #
            # A missing camera is a DEGRADED vehicle, not a broken one. Keep
            # the eye that works; the detector for the dead one is dropped too,
            # so nothing subscribes to a topic that will never carry frames.
            try:
                cam_node = CameraNode(
                    f'mongla_camera_{camera}',
                    parameter_overrides=launcher.camera_overrides(cam, camera),
                    frame_sink=det)
            except Exception as exc:      # noqa: BLE001 -- any open failure
                det.destroy_node()
                launcher.get_logger().error(
                    f'[COMP ] {camera} camera did NOT come up: {exc}')
                launcher.get_logger().error(
                    f'[COMP ] running WITHOUT {camera}. No '
                    f'/mongla/vision/{camera}/* topics this session -- a '
                    f'mission that steers on {camera} will find nothing.')
                continue
            nodes.append(det)
            nodes.append(cam_node)
            live.append(camera)
            launcher.get_logger().info(
                f'[COMP ] {camera}: camera -> detector DIRECT '
                f'(no topic in the control path)')
        # Every camera absent is a real failure: there is no vehicle to run.
        # Degrading to zero eyes silently is how a dead stack looks healthy.
        if not live:
            launcher.get_logger().error(
                '[COMP ] NO camera came up. Nothing to detect on -- exiting '
                'rather than idling as a healthy-looking node.')
            raise SystemExit(1)
        if len(live) == 1:
            launcher.get_logger().warning(
                f'[COMP ] DEGRADED: {live[0]} only. The chip is uncontended, '
                f'so this camera runs at full rate.')
        ex = MultiThreadedExecutor()
        for n in nodes:
            ex.add_node(n)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)
