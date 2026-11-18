"""The lock's field-of-view warning reports the bearing in the frame's own
pixels.

`LockNode._K` is scaled to the anchor backend's grid (320x240), while the box
and the frame are full camera pixels. Fed to `assess` unscaled, every bearing
in the warning was off by the ratio of the two grids -- a plausible angle,
wrong. Truth by construction: 300 px right of centre at a full-frame fx of
500 px is atan(300 / 500).
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('cv2')
pytest.importorskip('rclpy')

from mongla_vision.lock_publish import LockPublishMixin   # noqa: E402


class _Log:
    def warn(self, *_a):
        pass


class _Node(LockPublishMixin):
    def __init__(self):
        be_w, be_h, full_w, full_h, fx = 320, 240, 640, 480, 500.0
        s = be_w / full_w
        self._anchor = SimpleNamespace(_be=SimpleNamespace(w=be_w, h=be_h))
        self._K = np.array([[fx * s, 0, 320 * s], [0, fx * s, 240 * s], [0, 0, 1]])
        self._K_rect = None
        self._gray = np.zeros((full_h, full_w), np.uint8)

    def _gray_now(self):
        return self._gray

    def get_logger(self):
        return _Log()


def test_the_bearing_is_in_full_frame_pixels():
    n = _Node()
    cx, cy = 320 + 300, 240
    n._note_visibility(cx - 5, cy - 5, cx + 5, cy + 5)
    assert n._vis_state.bearing_deg == pytest.approx(
        math.degrees(math.atan(300 / 500.0)), abs=0.05)
