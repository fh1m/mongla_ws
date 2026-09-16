"""The heading anchor against GROUND TRUTH, through the real PnP solver.

`test_heading_anchor.py` checks `absolute_heading` against the formula it is
written from, which cannot catch a sign convention that disagrees with what
`solve_pnp` actually returns. Here a board is placed at a known compass bearing,
the hull at a known heading, the board's corners are projected through a pinhole
camera, the pose is SOLVED, and the anchored heading must equal the hull's.
"""
import math

import numpy as np
import pytest

pytest.importorskip('cv2')
from duburi_vision.anchor.pose import solve_pnp                     # noqa: E402
from duburi_localization.heading_anchor import absolute_heading     # noqa: E402


def _compass(deg):
    a = math.radians(deg)
    return np.array([math.sin(a), math.cos(a), 0.0])     # x east, y north, z up


def _anchored(hull_deg, board_deg, off_axis_m=0.3):
    fwd, right = _compass(hull_deg), _compass(hull_deg + 90.0)
    down = np.array([0.0, 0.0, -1.0])
    world_to_cam = np.vstack([right, down, fwd])          # x right, y down, z forward
    centre = 2.0 * fwd + off_axis_m * right
    normal = _compass(board_deg)
    xb = np.cross(np.array([0.0, 0.0, 1.0]), normal)
    xb /= np.linalg.norm(xb)
    if np.cross(xb, down) @ normal < 0:                   # board +z = its outward face
        xb = -xb
    obj = np.array([[x, y, 0.0] for x in (-0.3, 0.3) for y in (-0.3, 0.3)])
    K = np.array([[850.0, 0, 640.0], [0, 850.0, 360.0], [0, 0, 1.0]])
    img = []
    for p in obj:
        c = K @ (world_to_cam @ (centre + p[0] * xb + p[1] * down))
        img.append(c[:2] / c[2])
    pose = solve_pnp(obj, np.array(img), K)
    assert pose.ok, pose.reason
    return absolute_heading(pose.yaw_deg, board_deg)


@pytest.mark.parametrize('hull,board', [(180, 0), (190, 0), (170, 0),
                                        (45, 200), (300, 150), (95, 260)])
def test_the_anchor_recovers_the_true_hull_heading(hull, board):
    err = (_anchored(hull, board) - hull + 180.0) % 360.0 - 180.0
    assert abs(err) < 0.05
