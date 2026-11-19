"""A box the model placed in the letterbox bars is not reported.

The 640x360 frame letterboxes into a square with 140-row grey bars above and
below. A box there clamps to the frame edge with y1 >= y2 -- negative area,
centre pinned to the edge -- and was published as a detection to steer on.
"""
import types

import numpy as np

from mongla_vision.detection.hailo import HailoDetector


def _det():
    me = types.SimpleNamespace(_size=640, _allow_ids=None, _names={0: 'gate'},
                               _assoc_conf=0.1, _max_det=100)
    return me


def test_a_box_wholly_in_the_bottom_bar_is_dropped():
    # 640x360 into 640: scale 1, pad_y 140. Bottom bar is rows 500..640.
    in_bar = np.array([[520 / 640, 100 / 640, 600 / 640, 200 / 640, 0.9]])
    in_frame = np.array([[200 / 640, 100 / 640, 300 / 640, 200 / 640, 0.9]])
    out = HailoDetector._boxes_to_detections(
        _det(), [np.vstack([in_bar, in_frame])], 640, 360, 1.0, 0, 140)
    assert len(out) == 1
    x1, y1, x2, y2 = out[0].xyxy
    assert (y1, y2) == (60.0, 160.0) and x2 > x1
