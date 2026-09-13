"""A lane line is the most reliable feature in a swimming pool. Read it.

⛔ THE FIRST DESIGN READ THE TILES AND REPORTED 0.00 DEGREES CONFIDENTLY.
Hough-on-edges votes per edge pixel, and a tiled floor offers dozens of tile
joints against a lane line's two sides -- so the grid won every vote and the
answer was always 0 or 90. The control that proved it is
`test_a_tiled_floor_with_NO_line_is_refused`: a frame containing only tiles
was ACCEPTED as a line.

Measured after the rewrite onto second moments, on a TILED floor:

    heading      worst error 0.17 deg across 0-180
    cross-track  exact to the pixel
    blur         unchanged through 21 px of Gaussian

That is what makes it worth having: World Aquatics fixes lane centres at
exactly 2.5 m, the markings dark and 0.2-0.3 m wide running the pool's full
length. Those are published world constants, not our pool's -- so unlike the
tile grating this instrument needs no deck calibration.
"""
import math

import numpy as np
import pytest

from duburi_localization.pool_lines import (
    CROSS_LINE_FROM_WALL_M, LANE_SPACING_M, LaneLine, detect,
)

RNG = np.random.default_rng(0)


def pool(n=480, angle_deg=0.0, offset_px=0.0, width_px=40, tile=32,
         noise=0, blur=0):
    """Pale tiled floor with one dark lane line at a known angle and offset."""
    import cv2

    im = np.full((n, n), 190, np.float32)
    if tile:
        y, x = np.mgrid[0:n, 0:n]
        im += 12 * ((x // tile + y // tile) % 2)
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32)
    t = math.radians(angle_deg)
    d = -(xx - n / 2) * math.sin(t) + (yy - n / 2) * math.cos(t) - offset_px
    if width_px > 0:
        im[np.abs(d) <= width_px / 2] = 45
    if noise:
        im = im + RNG.normal(0, noise, im.shape)
    im = np.clip(im, 0, 255).astype(np.uint8)
    if blur:
        im = cv2.GaussianBlur(im, (blur, blur), 0)
    return im


# --------------------------------------------------------------------------- #
#  NEGATIVE CONTROLS -- the first one killed the first design
# --------------------------------------------------------------------------- #
def test_a_tiled_floor_with_NO_line_is_refused():
    """⛔ THE CONTROL THAT MATTERS. The Hough version ACCEPTED this and
    returned 0.00 degrees. A line detector that prefers the numerous background
    to the single foreground is not a line detector."""
    assert detect(pool(width_px=0)) is None


def test_a_bare_floor_is_refused():
    assert detect(np.full((480, 480), 190, np.uint8)) is None


def test_white_noise_is_refused():
    assert detect(RNG.integers(0, 255, (480, 480)).astype(np.uint8)) is None


def test_a_round_dark_blob_is_refused():
    """A prop, a drain or a patch of algae. It is dark and large and has no
    direction -- the elongation test is what says so."""
    yy, xx = np.mgrid[0:480, 0:480]
    blob = np.where((yy - 240) ** 2 + (xx - 240) ** 2 < 80 ** 2, 45, 190)
    assert detect(blob.astype(np.uint8)) is None


def test_a_flat_frame_and_a_tiny_frame_are_refused():
    assert detect(np.full((480, 480), 128, np.uint8)) is None
    assert detect(np.zeros((16, 16), np.uint8)) is None


def test_a_nan_frame_is_refused():
    assert detect(np.full((64, 64), np.nan, np.float32)) is None


# --------------------------------------------------------------------------- #
#  POSITIVE -- scored against known truth, on a TILED floor throughout
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('angle', [0.0, 12.0, 30.0, 45.0, 67.0, 90.0, 120.0, 155.0])
def test_the_heading_is_recovered_within_a_quarter_degree(angle):
    """On the same tiled floor that defeated the first design."""
    g = detect(pool(angle_deg=angle))
    assert g is not None, f'refused at {angle}'
    err = min(abs(g.angle_deg - angle % 180.0),
              180.0 - abs(g.angle_deg - angle % 180.0))
    assert err < 0.25, f'{angle} -> {g.angle_deg}'


@pytest.mark.parametrize('offset', [-80.0, -30.0, 0.0, 40.0, 90.0])
def test_the_cross_track_offset_is_exact(offset):
    g = detect(pool(angle_deg=0.0, offset_px=offset))
    assert g is not None
    assert abs(g.offset_px - offset) < 2.0


def test_it_survives_blur_that_destroys_every_corner():
    """The turbid-water case. A lane line is 0.2-0.3 m of high-contrast dark
    spanning the frame -- it outlives the blur that erases the features a
    tracker needs."""
    import cv2

    for k in (9, 21):
        g = detect(pool(angle_deg=25.0, blur=k))
        assert g is not None, f'lost at {k} px blur'
        assert abs(g.angle_deg - 25.0) < 0.5


def test_it_survives_sensor_noise():
    g = detect(pool(angle_deg=25.0, noise=15))
    assert g is not None and abs(g.angle_deg - 25.0) < 0.5


# --------------------------------------------------------------------------- #
#  what it means
# --------------------------------------------------------------------------- #
def test_the_heading_is_modulo_ONE_EIGHTY_not_ninety():
    """A line is 1-D, so it is only TWO-fold ambiguous -- strictly better than
    the tile grid's four-fold. The ambiguity is returned, never invented away."""
    ln = LaneLine(angle_deg=30.0, offset_px=0.0, support=100, spread_deg=0.1)
    assert ln.heading_deg() == pytest.approx(30.0)
    assert ln.heading_deg(mount_yaw_deg=200.0) == pytest.approx(10.0)


def test_cross_track_converts_to_metres_with_a_height():
    """Same pinhole scaling the flow path uses, so it inherits whatever height
    is available -- including the one the tile grating reads off the floor."""
    ln = LaneLine(angle_deg=0.0, offset_px=100.0, support=100, spread_deg=0.1)
    assert ln.cross_track_m(1000.0, 2.0) == pytest.approx(0.2)
    assert math.isnan(ln.cross_track_m(0.0, 2.0))


def test_the_rulebook_constants_are_the_published_ones():
    """World Aquatics pool certification. These are NOT our pool's numbers to
    be measured on deck -- they are a worldwide standard, which is exactly why
    this instrument needs no venue calibration."""
    assert LANE_SPACING_M == 2.5
    assert CROSS_LINE_FROM_WALL_M == 2.0


def test_a_SHORT_dark_streak_is_refused():
    """⛔ THE SPAN GATE, and it had no control until this test.

    A lane line runs the full length of the pool, so in any view containing one
    it reaches two edges of the frame. A short dark streak is a shadow, a
    cable, a crack or the edge of a prop -- it has a direction, and that
    direction means nothing about the pool.
    """
    im = np.full((480, 480), 190, np.uint8)
    im[230:250, 180:300] = 45          # elongated, dark, but only 120 px long
    assert detect(im) is None


def test_a_frame_with_NO_dark_minority_is_refused():
    """⛔ THE CONTRAST GATE, likewise previously untested.

    Thresholding on a percentile ALWAYS returns pixels -- the darkest 8 % of a
    uniform field is still 8 % of it. Without a gap between that minority and
    the floor, the "line" is just the low end of the noise, and its orientation
    is whatever the noise happened to do.
    """
    im = (190 + RNG.normal(0, 3, (480, 480))).clip(0, 255).astype(np.uint8)
    assert detect(im) is None


def test_a_gentle_gradient_is_not_a_lane_line():
    """The same illumination ramp that fooled three versions of the grating.
    Here the darkest 8 % is one edge of the frame -- elongated, spanning, and
    entirely an artefact of the lighting."""
    im = np.tile(np.linspace(120, 220, 480), (480, 1)).astype(np.uint8)
    assert detect(im) is None


def test_a_LARGE_round_dark_region_is_refused():
    """⛔ ISOLATES THE ELONGATION GATE. The small blob above is caught by the
    span test, so it proves nothing about elongation. This one SPANS the frame
    and has sharp edges -- it passes every other gate, and only "a lane line is
    long and narrow" rejects it. A big dark patch on the floor, a shadow under
    a prop, or the open water beyond the tiles."""
    yy, xx = np.mgrid[0:480, 0:480]
    blob = np.where((yy - 240) ** 2 + (xx - 240) ** 2 < 200 ** 2, 45, 190)
    assert detect(blob.astype(np.uint8)) is None
