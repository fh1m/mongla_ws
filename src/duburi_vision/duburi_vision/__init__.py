"""duburi_vision -- perception for duburi.

Public exports:
  Camera                    abstract base every camera source implements
  make_camera               factory: source-name -> Camera
  make_camera_from_profile  factory: CAMERA_PROFILES dict -> Camera
  CAMERA_PROFILES           named profile dict (laptop, sim_front, ...)
  Detector, Detection       detection ABC + plain dataclass result
  YoloDetector              Ultralytics YOLO26 wrapper
  draw                      module of pure-function visual overlays

Tracking (ByteTrack + Kalman smoother) ships as tracker_node.py and the
tracking/ sub-package. Node entry points (`camera_node`, `detector_node`,
`tracker_node`, `vision_node`) are package-level scripts -- import the
modules to wire them in tests.
"""

from .cameras.camera   import Camera, FrameMeta
from .factory          import make_camera, make_camera_from_profile, BUILDERS
from .config           import CAMERA_PROFILES, get_profile
from .detection.detector import Detector, Detection
from .preflight        import (
    assert_vision_ready, wait_vision_state_ready,
    clear_cache as clear_preflight_cache,
    VisionNotReadyError, VisionStatus,
)

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
    'assert_vision_ready',
    'wait_vision_state_ready',
    'clear_preflight_cache',
    'VisionNotReadyError',
    'VisionStatus',
]
