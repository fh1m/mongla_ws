"""mongla_vision -- perception for mongla.

Public exports:
  Camera                    abstract base every camera source implements
  make_camera               factory: source-name -> Camera
  make_camera_from_profile  factory: CAMERA_PROFILES dict -> Camera
  CAMERA_PROFILES           named profile dict (forward, downward, laptop, ...)
  Detector, Detection       detection ABC + plain dataclass result
  wait_vision_state_ready   the readiness gate a mission waits on (preflight)

The nodes (`camera_node`, `detector_node`, `detector_dual_node`,
`tracker_node`, `lock_node`, `flow/flow_node`) are entry points, not exports.
"""
import contextlib

# Refuses a release that is not the active one (B-153). Tools that run this
# package from source alone have no runtime beside it; the refusal is a
# SystemExit, which this does not catch.
with contextlib.suppress(ModuleNotFoundError):
    import mongla_runtime  # noqa: F401

from .cameras.camera   import Camera, FrameMeta
from .factory          import make_camera, make_camera_from_profile, BUILDERS
from .config           import CAMERA_PROFILES, get_profile
from .detection.detector import Detector, Detection
from .preflight        import (wait_vision_state_ready,
                               VisionNotReadyError, VisionStatus)

__all__ = [
    'Camera',
    'FrameMeta',
    'Detector',
    'Detection',
    'make_camera',
    'make_camera_from_profile',
    'BUILDERS',
    'CAMERA_PROFILES',
    'get_profile',
    'wait_vision_state_ready',
    'VisionNotReadyError',
    'VisionStatus',
]
