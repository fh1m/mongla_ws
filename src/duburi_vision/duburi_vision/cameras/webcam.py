"""WebcamCamera — local USB / built-in webcam via cv2.VideoCapture.

Robust to:
  * device index OR device path ("/dev/video0") — both work
  * driver returning False on the very first read (cold start)
  * resolution requests the driver silently downgrades — we read back
    what the driver actually gave us so CameraInfo never lies
  * `cap.read()` blocking briefly (no threading; the camera_node owns the
    timer and we honor the timer cadence by returning whatever's ready)
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import numpy as np

from .camera import Camera, FrameMeta


class WebcamCamera(Camera):
    source_kind = 'webcam'

    def __init__(self, device=0, width=640, height=480, fps=30,
                 frame_id='laptop_cam', name='laptop', logger=None):
        self.name      = str(name)
        self._device   = device
        self._frame_id = str(frame_id)
        self._req      = (int(width), int(height), int(fps))
        self._log      = logger

        self._cap         = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"webcam: cv2.VideoCapture({device!r}) failed to open. "
                f"Try a different index, check /dev/video* perms, or pass device='/dev/video0'.")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)        # avoid stale frames

        self._actual_w   = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)  or width)
        self._actual_h   = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or height)
        self._actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS)        or fps)

        if self._log:
            self._log.info(
                f'[CAM ] webcam {device!r} opened: requested {width}x{height}@{fps} '
                f'-> got {self._actual_w}x{self._actual_h}@{self._actual_fps:.1f}')

        self._idx          = 0
        self._last_ok      = 0.0
        self._consec_fail  = 0

    def read(self) -> Tuple[Optional[np.ndarray], FrameMeta]:
        ok, frame = self._cap.read()
        meta = FrameMeta(
            frame_index=self._idx,
            width=self._actual_w,
            height=self._actual_h,
        )
        if not ok or frame is None:
            self._consec_fail += 1
            meta.fresh = False
            return None, meta

        self._consec_fail = 0
        self._last_ok     = meta.stamp_monotonic
        self._idx        += 1
        meta.fresh        = True
        return frame, meta

    def is_healthy(self) -> bool:
        return (self._cap.isOpened()
                and self._consec_fail < 30
                and (time.monotonic() - self._last_ok) < 2.0)

    def info(self) -> dict:
        return {
            'name':        self.name,
            'source_kind': self.source_kind,
            'width':       self._actual_w,
            'height':      self._actual_h,
            'fps':         self._actual_fps,
            'frame_id':    self._frame_id,
            'device':      self._device,
        }

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
