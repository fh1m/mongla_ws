"""Detection: Detector ABC + YoloDetector (Ultralytics YOLO26).

Conversion to vision_msgs lives in `messages.py` so detector code stays
pure-Python (testable without rclpy)."""

from .detector import Detector, Detection
from .gpu      import select_device

__all__ = ['Detector', 'Detection', 'select_device']
