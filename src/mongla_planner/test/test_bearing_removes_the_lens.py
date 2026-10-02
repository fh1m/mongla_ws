"""`bearing_to` removes the forward lens before reading an angle (issue #24).

It read `(u, cy)` through the flat-port rectifier only -- no lens model, and
every box on the principal row. On the forward calibration (k1 = -0.363) a box
at u = 1150 of 1280 read 29.1 deg in air against a true 32.7 deg. Truth here is
OpenCV's forward projection of a ray at a known azimuth; the code under test
inverts it.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from mongla_planner.mongla_dsl import MonglaMission

cv2 = pytest.importorskip('cv2')

_CAL = (Path(__file__).resolve().parents[2] / 'mongla_vision' / 'config'
        / 'calibration' / 'pi_forward_1280x720.json')
_cal = json.loads(_CAL.read_text())
K = np.asarray(_cal['camera_matrix'], float)
D = [float(v) for v in _cal['distortion_coefficients']]
W, H = float(_cal['image_width']), float(_cal['image_height'])


def _pixel(azimuth_deg, elevation_deg):
    ray = np.array([[math.tan(math.radians(azimuth_deg)),
                     math.tan(math.radians(elevation_deg)), 1.0]])
    img, _ = cv2.projectPoints(ray, np.zeros(3), np.zeros(3), K, np.asarray(D))
    return img.reshape(2)


def _fake(u, v, with_d=True):
    m = MagicMock()
    m.medium = 'air'
    m._cam_k = {'forward': (K[0][0], K[1][1], K[0][2], K[1][2])}
    m._img_size = {'forward': (W, H)}
    m._cam_d = {'forward': tuple(D) if with_d else ()}
    m.absolute_heading.return_value = 0.0
    m.where_offset.side_effect = lambda cls, camera=None: (u - W / 2.0) / (W / 2.0)
    m._records.side_effect = lambda cam, stale: [('gate', u, v, 80.0, 80.0, 0.9)]
    m.bearing_to = MonglaMission.bearing_to.__get__(m)
    return m


@pytest.mark.parametrize('azimuth,elevation', [(32.7, 0.0), (20.0, 15.0),
                                               (-25.0, -12.0), (5.0, 0.0)])
def test_a_box_reads_its_true_azimuth(azimuth, elevation):
    """Including boxes OFF the principal row: lens and port are radial, so the
    row a box sits on moves where it lands horizontally."""
    u, v = _pixel(azimuth, elevation)
    got = _fake(u, v).bearing_to('gate', camera='forward')
    assert ((got + 180.0) % 360.0) - 180.0 == pytest.approx(azimuth, abs=0.3)


def test_the_issue_s_number_without_the_lens():
    """The defect, pinned so the test above is known to be able to fail."""
    u, v = _pixel(32.7, 0.0)
    got = _fake(u, v, with_d=False).bearing_to('gate', camera='forward')
    assert ((got + 180.0) % 360.0) - 180.0 == pytest.approx(29.1, abs=0.3)
