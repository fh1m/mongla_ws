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
    def __init__(self):
        self.lines = []

    def warn(self, msg, *_a):
        self.lines.append(msg)


def _bearing(lines):
    """The bearing the operator reads, off the warning itself."""
    import re
    return float(re.search(r'bearing=([+-][0-9.]+) deg', lines[-1]).group(1))


class _Node(LockPublishMixin):
    def __init__(self):
        be_w, be_h, full_w, full_h, fx = 320, 240, 640, 480, 500.0
        s = be_w / full_w
        self._anchor = SimpleNamespace(_be=SimpleNamespace(w=be_w, h=be_h))
        self._K_grid = (float(be_w), float(be_h))
        self._K = np.array([[fx * s, 0, 320 * s], [0, fx * s, 240 * s], [0, 0, 1]])
        self._K_rect = None
        self._gray = np.zeros((full_h, full_w), np.uint8)
        self.log = _Log()

    def _gray_now(self):
        return self._gray

    def get_logger(self):
        return self.log


def test_the_bearing_is_in_full_frame_pixels():
    n = _Node()
    cx, cy = 320 + 300, 240
    n._note_visibility(cx - 5, cy - 5, cx + 5, cy + 5)
    assert _bearing(n.log.lines) == pytest.approx(
        math.degrees(math.atan(300 / 500.0)), abs=0.05)


def test_the_warning_runs_without_an_anchor_bank():
    """K was only ever built on the anchor backend's grid, so a lock whose
    bank failed to load had no K and the frame-edge warning went silent.
    With no bank, K stays on the camera's own grid."""
    from types import SimpleNamespace as NS
    from mongla_vision.lock_inputs import LockInputsMixin

    class _N(LockInputsMixin, LockPublishMixin):
        _anchor = None
        _K = None
        _medium = 'air'
        _gray = np.zeros((480, 640), np.uint8)

        def _gray_now(self):
            return self._gray

        log = _Log()

        def get_logger(self):
            return NS(info=lambda *_: None, warn=self.log.warn)

    n = _N()
    n._on_info(NS(k=[500.0, 0, 320, 0, 500.0, 240, 0, 0, 1], d=[],
                  width=640, height=480))
    cx, cy = 320 + 300, 240
    n._note_visibility(cx - 5, cy - 5, cx + 5, cy + 5)
    assert _bearing(n.log.lines) == pytest.approx(
        math.degrees(math.atan(300 / 500.0)), abs=0.05)
