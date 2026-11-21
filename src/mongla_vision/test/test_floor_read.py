"""The grout and lane drawn over the downward picture lie where the floor has them.

Truth by construction: a rendered floor with a known period, angle and lane
(`test_tile_grating.tiled_floor_with_lane`: grout 120, tile 200, lane 25 grey).
Each predicted grout line is sampled along its length; on the grout its median
reads near 120, half a period away near 200. A drawing off by half a tile
looks exactly as tidy as a right one, which is why it is sampled, not looked at.
"""
import math

import numpy as np
import pytest

from mongla_vision.flow.floor_read import grid_segments, grout_lines, lane_axis, lane_segment
from mongla_vision.flow.pool_lines import detect as detect_lane
from mongla_vision.flow.tile_grating import measure

from test_tile_grating import tiled_floor_with_lane


def _along(img, seg, n=400):
    (x0, y0), (x1, y1) = seg
    xs = np.clip(np.round(np.linspace(x0, x1, n)).astype(int), 0, img.shape[1] - 1)
    ys = np.clip(np.round(np.linspace(y0, y1, n)).astype(int), 0, img.shape[0] - 1)
    return img[ys, xs]


def _grout_medians(img, lines, period, shift=0.0):
    size = (img.shape[1], img.shape[0])
    moved = [(nd, off + shift) for nd, off in lines]
    segs = grid_segments(moved, period, size)
    # The lane is darker than grout; a line crossing it is judged on the rest.
    return [float(np.median(v[v > 60])) for v in (_along(img, s) for s in segs)
            if (v > 60).sum() > 50]


CASES = [(p, a) for p in (42.0, 65.0, 88.0) for a in (3.0, 17.0, 31.0, 62.0, 80.0)]


@pytest.mark.parametrize('period,angle', CASES)
@pytest.mark.parametrize('decimate', (3,))           # the flow node's; measure answers there
def test_the_drawn_grout_lies_on_the_grout(period, angle, decimate):
    img = tiled_floor_with_lane(period, angle, 0.0)
    g = measure(img, decimate=decimate)
    assert g is not None
    lines = grout_lines(img, g.period_px, g.angle_deg, decimate)
    on = _grout_medians(img, lines, g.period_px)
    off = _grout_medians(img, lines, g.period_px, shift=g.period_px / 2)
    assert len(on) >= 4
    assert np.median(on) < 160, (on, lines)
    assert np.median(off) > 180, off


def test_a_phase_read_backwards_is_caught():
    """The injected defect: the carrier's maximum taken for the grout."""
    img = tiled_floor_with_lane(65.0, 17.0, 0.0)
    g = measure(img, decimate=3)
    lines = grout_lines(img, g.period_px, g.angle_deg, 3)
    wrong = [(nd, off + g.period_px / 2) for nd, off in lines]
    assert np.median(_grout_medians(img, wrong, g.period_px)) > 160


@pytest.mark.parametrize('period,angle', [(65.0, 8.9), (42.0, 31.0), (88.0, 62.0)])
@pytest.mark.parametrize('decimate', (3,))           # the flow node's (flow_floor._read_the_lane)
def test_the_drawn_lane_lies_on_the_lane(period, angle, decimate):
    # pool_lines finds a band wider than a tile (a lane is 0.25 m, a tile 0.15 m).
    img = tiled_floor_with_lane(period, angle, period * 1.5)
    small = img if decimate == 1 else __import__('cv2').resize(
        img, None, fx=1 / decimate, fy=1 / decimate, interpolation=__import__('cv2').INTER_AREA)
    line = detect_lane(small)
    assert line is not None
    size = (img.shape[1], img.shape[0])
    direction, off = lane_axis(line.angle_deg, line.offset_px, decimate, size)
    # The renderer's lane centre is half a period from the frame centre,
    # along the lane's normal: the offset to a few pixels, not just inside a
    # band wide enough to hide a missing decimation factor.
    assert abs(off - period / 2) < 3.0, (off, period / 2)
    seg = lane_segment(direction, off, size)
    assert np.median(_along(img, seg)) < 60
    # Moved off by its own width, the line reads tile and grout.
    t = math.radians(direction)
    assert np.median(_along(img, lane_segment(direction, off + period * 1.5, size))) > 100, t
