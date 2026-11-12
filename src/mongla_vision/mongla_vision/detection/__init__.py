"""Detection: Detector ABC + YoloDetector (Ultralytics YOLO11, yolov11n default).

Conversion to vision_msgs lives in `messages.py` so detector code stays
pure-Python (testable without rclpy)."""

import os

from .hailo_device import SERVICE_ADDR

# HailoRT reads the service's address once, when the library first loads, and
# any module here may be the first to load it (hailo.py does, for a HEF).
os.environ.setdefault('HAILORT_SERVICE_ADDRESS', SERVICE_ADDR)

from .detector import Detector, Detection  # noqa: E402
from .gpu      import select_device  # noqa: E402

__all__ = ['Detector', 'Detection', 'select_device']
