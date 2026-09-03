"""Both detectors in ONE process, because the chip allows one VDevice per process.

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

WHAT IS DELIBERATELY UNCHANGED
------------------------------
The node NAMES (`duburi_detector_forward` / `duburi_detector_downward`), every
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

from .detector_node import DetectorNode

# Parameters that are per-camera. Anything not here is shared by both, which is
# right for the ones that describe the machine (device, imgsz, half) and for the
# ones an operator sets globally (max_det, debug_image_hz, paused).
_PER_CAMERA = ('model_path', 'models', 'active_model', 'classes', 'conf',
               'model_conf', 'image_topic')
_SHARED = ('device', 'half', 'iou', 'imgsz', 'max_det', 'publish_debug_image',
           'debug_image_hz', 'alignment_deadband', 'paused')

_DEFAULTS = {
    'model_path': 'yolov11n', 'models': '', 'active_model': '', 'classes': '',
    # 0.15, NOT the 0.35 every other launch path ships. The HEFs are baked at
    # 0.05 exactly so this point is reachable, and INT8 costs ~0.08 of score
    # without moving the box -- so the CUDA number is ~3x too tight here and is
    # the most likely reason a detection is missed. `hailo.py` warns if it is
    # raised back.
    'conf': 0.15, 'model_conf': '', 'image_topic': '',
    'device': 'cuda:0', 'half': True, 'iou': 0.5, 'imgsz': 640,
    'max_det': 100, 'publish_debug_image': True, 'debug_image_hz': 5.0,
    'alignment_deadband': 0.05, 'paused': False,
}


class _Launcher(Node):
    """Holds the prefixed parameters. Not a detector; it owns no topics."""

    def __init__(self):
        super().__init__('duburi_detector_dual')
        for cam in ('fwd', 'dwn'):
            for key in _PER_CAMERA:
                self.declare_parameter(f'{cam}_{key}', _DEFAULTS[key])
        for key in _SHARED:
            self.declare_parameter(key, _DEFAULTS[key])

    def overrides(self, cam: str, camera: str):
        out = [Parameter('camera', value=camera)]
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
    try:
        for cam, camera in (('fwd', 'forward'), ('dwn', 'downward')):
            # Built SEQUENTIALLY on purpose. The second one configures a second
            # network group on the device the first created; doing that from a
            # thread pool is how the registry path already trips over itself.
            nodes.append(DetectorNode(
                f'duburi_detector_{camera}',
                parameter_overrides=launcher.overrides(cam, camera)))
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
