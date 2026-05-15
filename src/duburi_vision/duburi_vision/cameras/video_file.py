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
- `read()` returns `(last_frame, meta with fresh=False)` when paused or on
  decode failure; the camera_node handles the fresh=False case the same way it
  handles a dropped USB frame.

Playback controls (thread-safe)
--------------------------------
  cam.pause()                 → freeze playback; read() replays last good frame
  cam.resume()                → resume playback
  cam.toggle_pause() -> bool  → flip state; returns new is_paused value
  cam.seek_rel(seconds)       → seek ±N seconds from current position
  cam.seek_frames(n)          → seek ±N frames; if paused, exposes one fresh frame
  cam.position                → (current_frame, total_frames)
  cam.is_paused               → bool
"""

from __future__ import annotations

import threading
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
        self._total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0

        if self._log:
            self._log.info(
                f'[CAM  ] video_file {path!r} opened: '
                f'{self._actual_w}x{self._actual_h}@{self._actual_fps:.1f} '
                f'frames={self._total_frames}  loop={self._loop}')

        self._idx         = 0
        self._last_ok     = time.monotonic()
        self._consec_fail = 0
        self._eof         = False

        # Playback control — protected by _lock so ROS service callbacks and
        # the capture thread never race on _cap operations.
        self._lock:          threading.Lock       = threading.Lock()
        self._paused:        bool                 = False
        self._step_pending:  bool                 = False  # advance one frame while paused
        self._last_frame:    Optional[np.ndarray] = None

        # Frame-rate throttle — enforced outside the lock in read().
        # Without this, cv2.VideoCapture on a cached file decodes at ~3000 fps
        # and burns through the video in seconds.
        self._speed:           float = 1.0
        self._frame_interval:  float = 1.0 / self._actual_fps
        self._next_frame_time: float = time.monotonic()

    # ------------------------------------------------------------------ #
    #  Playback control API                                                #
    # ------------------------------------------------------------------ #

    def pause(self) -> None:
        with self._lock:
            self._paused = True

    def resume(self) -> None:
        with self._lock:
            self._paused = False

    def toggle_pause(self) -> bool:
        with self._lock:
            self._paused = not self._paused
            return self._paused

    def seek_rel(self, seconds: float) -> None:
        """Seek by ±seconds from the current position. Thread-safe."""
        with self._lock:
            if not self._cap or not self._cap.isOpened():
                return
            delta  = int(seconds * self._actual_fps)
            cur    = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))
            hi     = max(0, self._total_frames - 1) if self._total_frames else cur + abs(delta)
            target = max(0, min(hi, cur + delta))
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, target)
            self._eof = False
            self._next_frame_time = time.monotonic()  # resume without stale sleep debt
            if self._paused:
                self._step_pending = True  # expose the seeked frame while paused

    def seek_frames(self, n: int) -> None:
        """Seek by exactly ±N frames. While paused, exposes one fresh frame at the new position."""
        with self._lock:
            if not self._cap or not self._cap.isOpened():
                return
            cur    = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))
            hi     = max(0, self._total_frames - 1) if self._total_frames else cur + abs(n)
            target = max(0, min(hi, cur + n))
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, target)
            self._eof = False
            self._next_frame_time = time.monotonic()
            if self._paused:
                self._step_pending = True

    # Speed step ladder — same values understood by display_node keybinds.
    _SPEED_STEPS: tuple = (0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 4.0)

    def set_speed(self, speed: float) -> float:
        """Set playback speed multiplier (0.1–4.0). Returns clamped value."""
        speed = max(0.05, float(speed))
        with self._lock:
            self._speed = speed
        return speed

    def speed_step_up(self) -> float:
        with self._lock:
            cur = self._speed
            for s in self._SPEED_STEPS:
                if s > cur + 0.01:
                    self._speed = s
                    return s
            return self._speed

    def speed_step_down(self) -> float:
        with self._lock:
            cur = self._speed
            for s in reversed(self._SPEED_STEPS):
                if s < cur - 0.01:
                    self._speed = s
                    return s
            return self._speed

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def position(self) -> tuple[int, int]:
        """(current_frame, total_frames) — best-effort; not lock-protected for speed."""
        if not self._cap:
            return 0, self._total_frames
        return int(self._cap.get(cv2.CAP_PROP_POS_FRAMES)), self._total_frames

    @property
    def is_paused(self) -> bool:
        return self._paused

    # ------------------------------------------------------------------ #
    #  Camera ABC                                                          #
    # ------------------------------------------------------------------ #

    def read(self) -> Tuple[Optional[np.ndarray], FrameMeta]:
        meta = FrameMeta(
            frame_index=self._idx,
            width=self._actual_w,
            height=self._actual_h,
        )

        with self._lock:
            cap = self._cap
            # When paused: replay last frame, unless a step/seek was requested.
            if self._paused and not self._step_pending or cap is None:
                meta.fresh = False
                return self._last_frame, meta
            self._step_pending = False  # consume the one-shot step

            if self._eof:
                meta.fresh = False
                return self._last_frame, meta

            ok, frame = cap.read()

            if not ok or frame is None:
                if self._loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = cap.read()
                if not ok or frame is None:
                    self._eof = not self._loop
                    self._consec_fail += 1
                    meta.fresh = False
                    return self._last_frame, meta

            file_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            file_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if (self._actual_w, self._actual_h) != (file_w, file_h):
            frame = cv2.resize(frame, (self._actual_w, self._actual_h))

        self._consec_fail = 0
        self._last_ok     = meta.stamp_monotonic
        self._last_frame  = frame
        self._idx        += 1
        meta.fresh        = True

        # Throttle to target fps * speed — sleep is OUTSIDE the lock so
        # pause/seek calls are not blocked during the wait.
        now = time.monotonic()
        wait = self._next_frame_time - now
        if wait > 0.001:
            time.sleep(wait)
        self._next_frame_time = time.monotonic() + self._frame_interval / max(0.05, self._speed)

        return frame, meta

    def is_healthy(self) -> bool:
        return (self._cap is not None
                and self._cap.isOpened()
                and not self._eof
                and self._consec_fail < 30)

    def info(self) -> dict:
        cur, total = self.position
        return {
            'name':         self.name,
            'source_kind':  self.source_kind,
            'width':        self._actual_w,
            'height':       self._actual_h,
            'fps':          self._actual_fps,
            'frame_id':     self._frame_id,
            'path':         self._path,
            'loop':         self._loop,
            'total_frames': total,
            'position':     cur,
            'paused':       self._paused,
        }

    def close(self) -> None:
        with self._lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None
