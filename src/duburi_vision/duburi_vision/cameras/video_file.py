"""VideoFileCamera — run the vision pipeline on a pre-recorded video file.

Useful for offline tuning and pre-pool testing: point the pipeline at a
.mp4 / .avi / .mkv file and iterate on model weights, class lists, and
gain values without needing live hardware.

Behaviour
---------
- `loop:=true` (default) rewinds to frame 0 when the file ends so detection
  keeps running indefinitely.  Use `loop:=false` to stop at EOF.
- The frame rate defaults to the file's encoded FPS.  Override with `fps` if
  you want faster/slower playback (affects topic publish rate only — no
  interpolation is done).
- `read()` returns `(None, meta with fresh=False)` on decode failure; the
  camera_node handles this the same way it handles a dropped USB frame.
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import numpy as np

from .camera import Camera, FrameMeta


class VideoFileCamera(Camera):
    source_kind = 'video_file'

    def __init__(self, path: str, width=0, height=0, fps=0,
                 loop=True, frame_id='video_cam', name='video', logger=None):
        self.name      = str(name)
        self._path     = str(path)
        self._frame_id = str(frame_id)
        self._loop     = bool(loop)
        self._log      = logger

        cap = cv2.VideoCapture(self._path)
        if not cap.isOpened():
            raise RuntimeError(
                f"video_file: cv2.VideoCapture({path!r}) failed to open. "
                f"Check the file path and that OpenCV was built with ffmpeg support.")

        self._cap = cap
        file_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)  or 640)
        file_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)
        file_fps = float(cap.get(cv2.CAP_PROP_FPS)        or 30.0)

        self._actual_w   = int(width)  if width  else file_w
        self._actual_h   = int(height) if height else file_h
        self._actual_fps = float(fps)  if fps    else file_fps

        if self._log:
            self._log.info(
                f'[CAM  ] video_file {path!r} opened: '
                f'{self._actual_w}x{self._actual_h}@{self._actual_fps:.1f} '
                f'loop={self._loop}')

        self._idx         = 0
        self._last_ok     = time.monotonic()
        self._consec_fail = 0
        self._eof         = False

    def read(self) -> Tuple[Optional[np.ndarray], FrameMeta]:
        meta = FrameMeta(
            frame_index=self._idx,
            width=self._actual_w,
            height=self._actual_h,
        )

        if self._eof:
            meta.fresh = False
            return None, meta

        ok, frame = self._cap.read()

        if not ok or frame is None:
            if self._loop:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self._cap.read()
            if not ok or frame is None:
                self._eof = not self._loop
                self._consec_fail += 1
                meta.fresh = False
                return None, meta

        if (self._actual_w, self._actual_h) != (
                int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))):
            frame = cv2.resize(frame, (self._actual_w, self._actual_h))

        self._consec_fail = 0
        self._last_ok     = meta.stamp_monotonic
        self._idx        += 1
        meta.fresh        = True
        return frame, meta

    def is_healthy(self) -> bool:
        return (self._cap.isOpened()
                and not self._eof
                and self._consec_fail < 30)

    def info(self) -> dict:
        return {
            'name':        self.name,
            'source_kind': self.source_kind,
            'width':       self._actual_w,
            'height':      self._actual_h,
            'fps':         self._actual_fps,
            'frame_id':    self._frame_id,
            'path':        self._path,
            'loop':        self._loop,
        }

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
