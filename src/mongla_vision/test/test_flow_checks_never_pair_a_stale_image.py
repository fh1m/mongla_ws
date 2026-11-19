"""A failed planar fit leaves no image yaw or scale rate behind.

`_planar_refit` falls back to the median when the rigid fit fails, and the
interval is still accepted. The image yaw and scale rate it would have set
were then the PREVIOUS interval's, and `_cross_check_yaw` paired that stale
image yaw with this interval's gyro -- feeding the camera-gyro time-offset
correlation a pair from two different instants -- while `_cross_check_height`
fed a stale divergence against a fresh vertical speed. Absent is NaN, and
both checks skip it.
"""
import math

import numpy as np

from mongla_vision.flow.flow_checks import FlowChecksMixin
from mongla_vision.flow.flow_math import HeightFromDivergence


class _Recorder:
    def __init__(self):
        self.calls = []

    def add_image_yaw(self, t, v):
        self.calls.append(('img', t, v))

    def add_gyro_yaw(self, t, v):
        self.calls.append(('gyro', t, v))


class _Stub(FlowChecksMixin):
    _use_planar = True
    _intr = None
    _refract = None
    _undistort = False
    _ransac_px = 2.0
    _yaw_sign = 1.0
    _yaw_tol = 0.3
    _td_estimate = True
    _td_last_fit = 0.0

    def __init__(self):
        self._n_median_fallback = 0
        self._last_yaw_img = 0.4          # what the previous interval measured
        self._last_scale_rate = 0.3
        self._yaw_rate_buf = [(0.0, 0.4), (10.0, 0.4)]
        self._td_est = _Recorder()
        self._h_optical = HeightFromDivergence()
        self._last_height = 1.0
        self._depth = (0.0, 1.0)

    def _vz_down(self):
        return 0.2


def _fail_the_fit(s):
    pts = np.array([[10.0, 10.0], [20.0, 10.0], [30.0, 40.0]], np.float32)
    s._anchor_pts = pts
    status = np.ones((3, 1), np.uint8)
    gray = np.zeros((360, 640), np.uint8)
    s._planar_refit(gray, pts + 1.0, status, 0.1, (1.0, 1.0), 3, 0.0)


def test_a_failed_fit_leaves_NaN_not_the_last_value():
    s = _Stub()
    _fail_the_fit(s)
    assert s._n_median_fallback == 1
    assert math.isnan(s._last_yaw_img) and math.isnan(s._last_scale_rate)


def test_the_yaw_check_does_not_feed_the_time_offset_a_stale_pair():
    s = _Stub()
    _fail_the_fit(s)
    s._cross_check_yaw(5.0, 0.1)
    assert s._td_est.calls == []


def test_the_height_check_does_not_add_a_stale_divergence():
    s = _Stub()
    _fail_the_fit(s)
    s._cross_check_height()
    assert s._h_optical.n_samples == 0


def test_the_node_starts_with_no_image_yaw():
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision' / 'flow'
           / 'flow_node.py').read_text()
    assert "self._last_yaw_img = float('nan')" in src
    assert "self._last_scale_rate = float('nan')" in src
