"""The whole vision stack in ONE process: two cameras, two detectors, no hop.

Both detectors must share a process because the chip allows one VDevice per
process (below). Once they do, putting the CAMERAS in the same process costs
nothing and removes the last soft target in the frame path.

Why this executable exists
--------------------------
`vision_dual.launch.py` starts two `detector_node` PROCESSES. On a CUDA dev
box that is fine. On the Pi + AI HAT+ the second
one dies with `HAILO_OUT_OF_PHYSICAL_DEVICES (74)`, so the two-camera path has
never started on this hardware. Measured, along with the two alternatives that
look like they should work and do not:

    two processes, multi_process_service + scheduler -> works since the
        service was built and installed (kutum/setup.sh), but each inference
        pays its ~6 ms round trip (measured-bars 86)
    one process, one VDevice, ROUND_ROBIN scheduler    -> the default: the
        SIGSEGV once blamed on it was interpreter teardown (hailo_device.py)

So: one process, two `DetectorNode` objects, one VDevice opened directly
(`detection/hailo_device.py`). `cameras` selects a subset for the split
launch (`split:=true`), one process per camera through the service.

Why the cameras joined them
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

The image topic is still published, throttled, for the HUD, Batighor and
recorders. Composition takes the topic out of the
CONTROL path; it does not delete it.

What is deliberately unchanged, part two
----------------------------------------
The camera nodes keep their names (`mongla_camera_forward` / `_downward`) and
their topics, so `vision_display`, Batighor and `preflight` see exactly
what they saw before.

What is deliberately unchanged
------------------------------
The node NAMES (`mongla_detector_forward` / `mongla_detector_downward`), every
topic, and every parameter. Missions, `ClassRef`, `vision_state`, the helm's
`SetParameters` calls and the `active_camera` latch all address the
detector by node name, so collapsing the two into one node would have been a
much larger change for no benefit. Two nodes in one process is the smaller
edit and the invisible one.

Launch's `name=` and `parameters=` are PROCESS-WIDE remappings (`__node:=`,
`__params:=`) and cannot address two nodes in one process -- which is why the
parameters arrive here `fwd_`/`dwn_`-prefixed on this launcher and are handed
down as `parameter_overrides` rather than declared per node by launch.

The executor comes from `mongla_runtime.spin`. Live, each camera hands
its frames to its detector on the camera's own capture thread, so the executor
carries only timers, services and small topics, and the single-threaded events
executor is enough. In replay the frames arrive as image callbacks, and
`MONGLA_EXECUTOR=classic` gives the multi-threaded executor back, so one
camera's inference does not make the other wait.
"""
from __future__ import annotations

import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from .camera_node   import CameraNode
from .detector_node import DetectorNode
from .detector_params import CONF, NMS_FLOOR

# Parameters that are per-camera. Anything not here is shared by both, which is
# right for the ones that describe the machine (device, imgsz, half) and for the
# ones an operator sets globally (max_det, debug_image_hz, paused).
_PER_CAMERA = ('model_path', 'models', 'active_model', 'classes', 'conf',
               'assoc_conf', 'model_conf', 'image_topic')
_SHARED = ('device', 'half', 'iou', 'nms_floor', 'nms_iou', 'imgsz', 'max_det',
           'publish_contours',
           'publish_debug_image',
           'debug_image_hz', 'alignment_deadband', 'paused', 'range_crop')

# The camera half. Same prefixing scheme, same reason: launch cannot address
# two nodes in one process.
#
# `device` is in this list because of a name collision, not because anyone
# needs to set it. Launch parameters are PROCESS-wide, and `device` means two
# different things to the two kinds of node in this process: the detector's
# backend (`'cpu'`, a string) and the camera's V4L2 index (`-1`, an integer).
# Without an explicit per-camera override the process-wide `device: 'cpu'`
# reaches `CameraNode.declare_parameter('device', -1)` and the WHOLE PROCESS
# dies at startup with InvalidParameterTypeException. Prefer `device_path`
# for identity regardless -- a raw index has already swapped the two cameras
# across a reboot once.
_CAM_PER_CAMERA = ('profile', 'device', 'device_path', 'calibration', 'fps',
                   'publish_rate_hz', 'width', 'height', 'exposure_us',
                   'controls')
_CAM_DEFAULTS = {
    'profile': '', 'device': -1, 'device_path': '', 'calibration': '',
    'fps': 0, 'exposure_us': 0, 'controls': '',
    # A composed detector is fed on demand, so this rate now governs only the
    # topic's viewers -- not the control path, which is what it used to gate.
    'publish_rate_hz': 0, 'width': 640, 'height': 480,
}

_DEFAULTS = {
    'model_path': 'yolov11n', 'models': '', 'active_model': '', 'classes': '',
    # The node's own defaults (detector_params), so `ros2 run` and the
    # composed process start the same detector.
    'conf': CONF, 'model_conf': '', 'image_topic': '',
    # 0.0 here, so a direct `ros2 run` is the single-node behaviour; the
    # vehicle launch turns it on (vision_pi.launch.py, `assoc_conf`).
    'assoc_conf': 0.0,
    'device': 'auto', 'half': True, 'iou': 0.5, 'imgsz': 640,
    'nms_floor': NMS_FLOOR, 'nms_iou': 0.0,
    'max_det': 100, 'publish_contours': True,
    'publish_debug_image': True, 'debug_image_hz': 5.0,
    'alignment_deadband': 0.05, 'paused': False,
    # The type must match `detector_setup.declare_parameter` exactly or rclpy
    # raises InvalidParameterTypeException and the whole composed process
    # dies at startup.
    'range_crop': False,
}


class _Launcher(Node):
    """Holds the prefixed parameters. Not a detector; it owns no topics."""

    def __init__(self):
        super().__init__('mongla_detector_dual')
        # `replay`: consume frames from a RECORDED bag instead of a camera.
        # Without this there is no way to run the real graph on recorded
        # input -- CameraNode raises when the device is absent and this
        # launcher then drops that camera's detector, so a bag full of
        # image_raw has nothing left subscribed to play into. That made every
        # pipeline check require a live camera and a person standing in front
        # of it, which is not a thing a regression test can do.
        self.declare_parameter('replay', False)
        # Which cameras THIS process holds. Both, or one when the launch runs
        # a process per camera through the HailoRT service: each then has its
        # own interpreter and GIL, and the chip is shared by the service
        # rather than by threads (measured-bars 80, 86).
        self.declare_parameter('cameras', 'forward,downward')
        for cam in ('fwd', 'dwn'):
            for key in _PER_CAMERA:
                self.declare_parameter(f'{cam}_{key}', _DEFAULTS[key])
        for key in _SHARED:
            self.declare_parameter(key, _DEFAULTS[key])
        for cam in ('fwd', 'dwn'):
            for key in _CAM_PER_CAMERA:
                self.declare_parameter(f'{cam}_cam_{key}', _CAM_DEFAULTS[key])
        # The lock ladder, composed (2026-10-03). `lock` builds a LockNode
        # beside each live camera's detector, in THIS process -- the only
        # place XFeat can run on the Hailo, because the chip allows one
        # VDevice per process, shared by HailoRT's ROUND_ROBIN scheduler. A
        # separate lock_node raced the detector for the chip and, when it
        # won, left the vehicle with no detection at all. Frames reach the
        # ladder by reference (`_Tee`, `direct_feed`): with topic intake the
        # composition cost both detectors 71 -> 21.6 Hz (measured-bars 28.6).
        # Verify on real footage: tools/studies/composed_lock_harness.py.
        #
        # Only the values the launch SETS are forwarded (`fwd_lock_*`,
        # `dwn_lock_*`, shared `lock_*`); the LockNode keeps its own defaults,
        # so there is no third copy of them here to drift.
        self.declare_parameter('lock', False)
        for name, p in list(self._parameter_overrides.items()):
            if name.startswith(('fwd_lock_', 'dwn_lock_', 'lock_')):
                self.declare_parameter(name, p.value)
        # Optical flow on the downward camera, in this process for the same
        # reason as the lock: as its own node it took ~210 raw frames a second
        # off DDS, 44 % of its samples (measured-bars 87). `flow_*` values are
        # forwarded with the prefix stripped; flow_node keeps its own defaults.
        self.declare_parameter('flow', False)
        for name, p in list(self._parameter_overrides.items()):
            if name.startswith('flow_'):
                self.declare_parameter(name, p.value)

    def lock_overrides(self, cam: str, camera: str):
        # Generic `lock_X` first, then this camera's `fwd_lock_X`: rclpy keeps
        # the LAST override of a name, so the per-camera value wins.
        out = [Parameter('camera', value=camera)]
        for prefix in ('lock_', f'{cam}_lock_'):
            for name in sorted(self._parameters):
                if name.startswith(prefix):
                    out.append(Parameter(name[len(prefix):],
                                         value=self.get_parameter(name).value))
        return out

    def flow_overrides(self, direct: bool):
        out = [Parameter('camera', value='downward'),
               Parameter('direct_feed', value=direct)]
        for name in sorted(self._parameters):
            if name.startswith('flow_'):
                out.append(Parameter(name[len('flow_'):],
                                     value=self.get_parameter(name).value))
        return out

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
        # Under replay there IS no in-process camera, so the detector has to
        # take the topic path it normally refuses -- `direct_feed` is exactly
        # the switch between the two, and it must be False or the node will
        # sit waiting for a frame_sink that nothing will ever call.
        direct = not bool(self.get_parameter('replay').value)
        out = [Parameter('camera', value=camera),
               Parameter('direct_feed', value=direct)]
        for key in _PER_CAMERA:
            out.append(Parameter(
                key, value=self.get_parameter(f'{cam}_{key}').value))
        for key in _SHARED:
            out.append(Parameter(key, value=self.get_parameter(key).value))
        return out


def _make_failure_hook(launcher, live: list, failed: list, state: dict):
    """What a composed detector does when its model will not load.

    One dead model used to kill both (2026-10-03, on the vehicle): the
    forward detector was live on gate_sharks.hef, the downward model had no
    .hef, and the downward load's exit took the forward detector with it. Same
    rule as a missing camera: keep the eye that works, say LOUDLY which one is
    blind, and exit only when none is left -- a process with no detector at
    all would idle looking healthy. The camera stays up: its frames still feed
    flow and the recorders.
    """
    def hook(det, exc):
        camera = getattr(det, '_cam_name', None) or det.get_name()
        failed.append(camera)
        launcher.get_logger().error(
            f'[COMP ] {camera} DETECTOR did NOT come up ({exc}). Running '
            f'WITHOUT {camera} detections -- a mission that steers on it will '
            f'find nothing. The camera itself stays up.')
        # Loads are asynchronous: one can fail before the other camera is even
        # built, so nothing is decided until composition is finished.
        if state.get('built') and all(c in failed for c in live):
            _no_detector_left(launcher)
    return hook


class _Tee:
    """One camera, several in-process consumers: the detector, and the lock
    and flow when they run.

    `CameraNode` hands frames to ONE sink. The lock takes every decoded frame
    (its stamp history must hold the frame each detection was computed on),
    so the camera decodes every frame -- and every one is also OFFERED to the
    detector, whose slot is newest-wins: an idle worker takes it at once, a
    busy one finds the newest waiting when it frees up.

    MEASURED, 2026-10-03: handing the detector a frame only "if it wants
    one right now" starved it. The frame decoded for the lock drained the
    mailbox, and a detector freeing up 1 ms later waited for the NEXT
    capture -- py-spy put the workers 18-39 % of wall time there. `spare=True`
    keeps the detector's decoded-but-never-inferred counter honest: these
    frames were decoded for the lock, not wasted on the detector.
    """

    def __init__(self, detector, *others):
        self._det, self._others = detector, others

    def wants_frame(self) -> bool:
        return True                 # the lock and flow want every frame

    def submit_frame(self, frame_bgr, header) -> None:
        self._det.submit_frame(frame_bgr, header, spare=True)
        for o in self._others:
            o.submit_frame(frame_bgr, header)

    def submit_info(self, info) -> None:
        for o in self._others:
            if hasattr(o, 'submit_info'):
                o.submit_info(info)


def _compose_lock(launcher, nodes: list, cam: str, camera: str,
                  direct: bool):
    """Build this camera's lock ladder in-process, when `lock` is set.

    `direct`: frames and intrinsics arrive by reference from the camera in
    this process (see `_Tee`), so the ladder subscribes to neither topic.
    Under replay there is no camera here and it subscribes as usual.
    """
    if not bool(launcher.get_parameter('lock').value):
        return None
    from mongla_vision.lock_node import LockNode
    node = LockNode(f'mongla_lock_{camera}',
                    parameter_overrides=list(
                        launcher.lock_overrides(cam, camera))
                    + [Parameter('direct_feed', value=direct)])
    nodes.append(node)
    launcher.get_logger().info(
        f'[COMP ] {camera}: lock ladder COMPOSED -- one process, one Hailo '
        f'VDevice shared with the detector'
        + (', frames by reference' if direct else ''))
    return node


def _compose_flow(launcher, nodes: list, camera: str, direct: bool):
    """Build optical flow in-process on the downward camera, when `flow` is set.

    `direct` as for the lock: by reference from the camera here, or off the
    topic under replay.
    """
    if camera != 'downward' or not bool(launcher.get_parameter('flow').value):
        return None
    from mongla_vision.flow.flow_node import FlowVelocityNode
    node = FlowVelocityNode(parameter_overrides=launcher.flow_overrides(direct))
    nodes.append(node)
    launcher.get_logger().info(
        '[COMP ] downward: optical flow COMPOSED'
        + (', frames by reference' if direct else ''))
    return node


def _no_detector_left(launcher):
    launcher.get_logger().fatal(
        '[COMP ] NO detector came up. Exiting rather than idling as a '
        'healthy-looking node.')
    os._exit(2)


def main():
    from mongla_runtime.orphan import die_with_parent
    die_with_parent()        # holds the cameras and the Hailo VDevice
    rclpy.init()
    launcher = _Launcher()
    nodes = [launcher]
    live: list[str] = []
    failed: list[str] = []
    state = {'built': False}
    hook = _make_failure_hook(launcher, live, failed, state)
    try:
        replay = bool(launcher.get_parameter('replay').value)
        wanted = [c.strip() for c in
                  str(launcher.get_parameter('cameras').value).split(',') if c.strip()]
        unknown = sorted(set(wanted) - {'forward', 'downward'})
        if unknown or not wanted:
            raise ValueError(f"cameras={wanted}: name forward and/or downward")
        for cam, camera in (('fwd', 'forward'), ('dwn', 'downward')):
            if camera not in wanted:
                continue
            # Built SEQUENTIALLY on purpose. The second one configures a second
            # network group on the device the first created; doing that from a
            # thread pool is how the registry path already trips over itself.
            det = DetectorNode(
                f'mongla_detector_{camera}',
                parameter_overrides=launcher.overrides(cam, camera),
                init_failure_fn=hook)
            det._cam_name = camera
            # The detector FIRST, so the camera never submits to a half-built
            # sink. `frame_sink=det` is the whole composition -- one Python
            # reference where a topic used to be.
            #
            # One absent camera used to kill both. CameraNode raises after
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
            if replay:
                # Keep the detector, build no camera. The frames come from
                # `ros2 bag play`, so this is a LIVE graph on RECORDED input:
                # the detector, tracker, lock ladder and checkpoint bank all
                # run exactly as they do on the vehicle.
                nodes.append(det)
                live.append(camera)
                launcher.get_logger().info(
                    f'[COMP ] {camera}: REPLAY -- no camera; consuming '
                    f'/mongla/vision/{camera}/image_raw from a bag.')
                _compose_lock(launcher, nodes, cam, camera, direct=False)
                _compose_flow(launcher, nodes, camera, direct=False)
                continue
            # The lock BEFORE the camera, so the camera's first frame already
            # has both consumers to go to.
            lock = _compose_lock(launcher, nodes, cam, camera, direct=True)
            flow = _compose_flow(launcher, nodes, camera, direct=True)
            extra = [n for n in (lock, flow) if n is not None]
            try:
                cam_node = CameraNode(
                    f'mongla_camera_{camera}',
                    parameter_overrides=launcher.camera_overrides(cam, camera),
                    frame_sink=_Tee(det, *extra) if extra else det)
            except Exception as exc:      # noqa: BLE001 -- any open failure
                det.destroy_node()
                for n in extra:
                    nodes.remove(n)
                    n.destroy_node()
                launcher.get_logger().error(
                    f'[COMP ] {camera} camera did NOT come up: {exc}')
                launcher.get_logger().error(
                    f'[COMP ] running WITHOUT {camera}. No '
                    f'/mongla/vision/{camera}/* topics this session -- a '
                    f'mission that steers on {camera} will find nothing.')
                continue
            # Said, not inferred: the detector used to conclude "no camera
            # here" from 2 s without a frame, and a detector that starts
            # paused (the launch default) asks for none -- so every run
            # moved to the topic path for good (2026-11-18).
            det.attach_camera()
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
        state['built'] = True
        if live and all(c in failed for c in live):
            _no_detector_left(launcher)
        if len(live) == 1:
            launcher.get_logger().warning(
                f'[COMP ] DEGRADED: {live[0]} only. The chip is uncontended, '
                f'so this camera runs at full rate.')
        from mongla_runtime.params_file import refuse_unknown
        for n in nodes:
            refuse_unknown(n)     # bringup.yaml `nodes:` typos stop here
        from mongla_runtime.spin import make_executor
        ex, which = make_executor(launcher.get_logger(), classic=MultiThreadedExecutor)
        launcher.get_logger().info(f'[COMP ] spinning on the {which} executor')
        for n in nodes:
            ex.add_node(n)
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C (Jazzy raises the second) is a stop: an exit 1 made the
        # launch report this process died and restart it on a normal stop.
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
