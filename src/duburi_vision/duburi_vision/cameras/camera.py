"""Abstract Camera — every image source implements this.

Contract
--------
read() -> (frame: np.ndarray BGR | None, meta: FrameMeta)
    `frame` is the latest available BGR frame as a HxWx3 uint8 ndarray, or
    None when no fresh sample is ready (camera not yet warmed up, ROS topic
    not published yet, dropped frame, etc). Callers MUST tolerate None on
    every call — never crash, never block waiting for a frame inside a ROS
    timer.

    `meta` is always returned (never None) and carries the wall-clock
    capture time and a frame counter. Even when the frame itself is None
    the meta is useful for "how long since I last saw a frame" diagnostics.

is_healthy() -> bool
    True when the source is currently producing usable frames. Used for
    startup banners and the diag log line, never for runtime fallback —
    one source per launch is the design (mirrors duburi_sensors).

info() -> dict
    Snapshot of the source's view of the world: width, height, fps,
    frame_id, source_kind. Used by camera_node to populate
    sensor_msgs/CameraInfo and by the debug overlay to print the source.

close()
    Release any underlying resources (cv2 capture, threads, subscriptions).
    Safe to call multiple times.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


@dataclass
class FrameMeta:
    """Per-read metadata. Filled in by the source on every read() call.

    `stamp_monotonic` is `time.monotonic()` at capture; used for FPS and
    staleness checks. `stamp_wall` is `time.time()` and is what we put
    on the ROS Image header.
    """
    frame_index:     int   = 0
    stamp_monotonic: float = field(default_factory=time.monotonic)
    stamp_wall:      float = field(default_factory=time.time)
    width:           int   = 0
    height:          int   = 0
    fresh:           bool  = False     # True iff a NEW frame was returned this call


class Camera:
    name:        str = 'base'
    source_kind: str = 'abstract'      # 'webcam' / 'ros_topic' / 'jetson' / ...

    def read(self) -> Tuple[Optional[np.ndarray], FrameMeta]:
        raise NotImplementedError

    def is_healthy(self) -> bool:
        return False

    def info(self) -> dict:
        return {
            'name':        self.name,
            'source_kind': self.source_kind,
            'width':       0,
            'height':      0,
            'fps':         0.0,
            'frame_id':    self.name,
        }

    def close(self) -> None:
        pass

    def __repr__(self) -> str:
        return f'<{self.__class__.__name__} name={self.name} kind={self.source_kind}>'
