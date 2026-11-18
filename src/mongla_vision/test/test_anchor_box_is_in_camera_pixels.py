"""The anchor rung's box is in the camera's pixels, not the backend's.

The anchor resizes every frame to its backend grid (XFeat: 320x240) before it
finds features, and `enrol` scales the detector's box INTO that grid. The
ladder, the follower and /lock all work in the camera's own pixels (640x360 on
the vehicle, a different scale AND aspect). The box went out unscaled: a
target at x=600 was published at x=300, compared against the follower in the
wrong grid, and reseeded the follower on the wrong patch.

Truth by construction: enrol on a frame with the detector's box, locate on the
SAME frame, and the anchor's box must be that box. The backend is a stand-in
with XFeat's contract (resize, keypoints in backend pixels, unit descriptors)
so the test needs no model file.
"""
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')
pytest.importorskip('rclpy')

from mongla_vision.anchor.anchor import Anchor            # noqa: E402
from mongla_vision.anchor.xfeat_onnx import XFeatONNX     # noqa: E402
from mongla_vision.lock_anchor import AnchorMixin         # noqa: E402


class _Backend:
    w, h = 320, 240

    def detect(self, gray):
        g = cv2.resize(gray, (self.w, self.h))
        pts = cv2.goodFeaturesToTrack(g, 800, 0.01, 4)
        k = np.zeros((0, 2), np.float32) if pts is None else pts.reshape(-1, 2)
        k = k[(k[:, 0] >= 4) & (k[:, 0] < self.w - 4)
              & (k[:, 1] >= 4) & (k[:, 1] < self.h - 4)]
        d = np.stack([g[int(y) - 4:int(y) + 4, int(x) - 4:int(x) + 4]
                      .astype(np.float32).ravel() for x, y in k]) if len(k) \
            else np.zeros((0, 64), np.float32)
        d -= d.mean(axis=1, keepdims=True)
        d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-6)
        return k.astype(np.float32), d

    match = staticmethod(XFeatONNX.match)


class _Node(AnchorMixin):
    def __init__(self, anchor):
        self._anchor = SimpleNamespace(_be=anchor._be)
        self._follower = None
        self._anchor_header = None

    def _publish_decomposition(self, *_a):
        pass


def test_the_box_comes_back_in_camera_pixels():
    rng = np.random.default_rng(5)
    frame = (rng.random((360, 640)) * 255).astype(np.uint8)
    frame = cv2.GaussianBlur(frame, (3, 3), 0)
    box = (440.0, 120.0, 600.0, 280.0)            # right side of the frame
    a = Anchor(_Backend(), min_inliers=15)
    a.snap(frame, roi=box)
    pose = a.locate(frame)
    assert pose.ok, 'the stand-in backend did not match the same frame'
    n = _Node(a)
    n._anchor_pose = pose
    ab, _conf = n._anchor_box(frame, anchor_ran=True)
    assert ab == pytest.approx(box, abs=6.0), ab
