"""The tile height must REACH the velocity, and must be OFF until measured.

Two failure modes, both of which this repo has shipped before:

  a knob wired to nothing   the parameter exists, the module exists, and no
                            value ever reaches the consumer
  a world constant assumed  `tile_m` is a VENUE property. Our pool is not the
                            competition's, and a wrong tile size does not fail
                            -- it scales every velocity by the ratio of the
                            two, with nothing logged

So these tests check reachability and the default, not the arithmetic (which
`test_tile_grating.py` owns against known truth).
"""
import pathlib

SRC = (pathlib.Path(__file__).resolve().parents[1]
       / 'duburi_vision' / 'flow' / 'flow_node.py').read_text()


def test_the_floor_reader_exists_and_is_CALLED():
    """A module nothing calls scores nothing -- the defect that shipped the
    invariant filter with zero callers."""
    assert 'def _read_the_floor' in SRC
    assert 'self._read_the_floor(gray, t)' in SRC


def test_it_is_called_BEFORE_the_height_is_consumed():
    """Read after use and the tile height is always one frame stale against
    the frame that scaled its velocity by it."""
    called = SRC.index('self._read_the_floor(gray, t)')
    used = SRC.index('if self._tile_height is not None:')
    assert called < used, 'the floor is read after its height is used'


def test_the_tile_height_is_PREFERRED_over_the_pool_depth_path():
    """`pool_depth_m` is, by this node's own comment, "the one input nobody
    measures carefully". The floor's answer contains no pool depth at all."""
    assert 'if self._tile_height is not None:\n            h = self._tile_height' in SRC


def test_tile_m_defaults_to_OFF():
    """⛔ A tile size is a venue constant. Defaulting it to any number would
    silently rescale every velocity in a pool that does not match."""
    assert "declare_parameter('tile_m', 0.0)" in SRC


def test_an_untiled_floor_CLEARS_the_height_rather_than_holding_it():
    """A stale height is worse than none: the caller cannot tell it is stale,
    and height multiplies every velocity."""
    block = SRC[SRC.index('def _read_the_floor'):SRC.index('def _cross_check_height')]
    assert 'self._tile_height = None' in block


def test_a_disagreement_is_REPORTED_not_silently_resolved():
    block = SRC[SRC.index('def _read_the_floor'):SRC.index('def _cross_check_height')]
    assert 'the FLOOR says' in block


def test_a_missing_localization_package_disables_it_instead_of_crashing():
    """The flow node must still run if the estimation package is absent."""
    block = SRC[SRC.index('def _read_the_floor'):SRC.index('def _cross_check_height')]
    assert 'self._tile_m = 0.0' in block


# --------------------------------------------------------------------------- #
#  EXECUTED -- the grep tests above pass whether or not the body raises
# --------------------------------------------------------------------------- #
import math                                                     # noqa: E402

import numpy as np                                              # noqa: E402
import pytest                                                   # noqa: E402

pytest.importorskip('rclpy')
pytest.importorskip('cv_bridge')
pytest.importorskip('duburi_localization.tile_grating')


@pytest.fixture(scope='module', autouse=True)
def _ros():
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(**params):
    from test_flow_node import _make                            # noqa: E402
    n = _make(**params)
    n.published = {'grid': [], 'lane': []}
    n._pub_grid.publish = lambda m: n.published['grid'].append(m.data)
    n._pub_lane.publish = lambda m: n.published['lane'].append(m.data)
    return n


def _tiles(n=720, period=18.0, angle_deg=0.0):
    y, x = np.mgrid[0:n, 0:n].astype(np.float32)
    t = math.radians(angle_deg)
    u = x * math.cos(t) + y * math.sin(t)
    v = -x * math.sin(t) + y * math.cos(t)
    im = 128 + 50 * (np.sign(np.sin(2 * np.pi * u / period))
                     + np.sign(np.sin(2 * np.pi * v / period)))
    return np.clip(im, 0, 255).astype(np.uint8)


def _lane(n=720, angle_deg=33.0, width_px=60):
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32)
    t = math.radians(angle_deg)
    d = -(xx - n / 2) * math.sin(t) + (yy - n / 2) * math.cos(t)
    im = np.full((n, n), 190, np.float32)
    im[np.abs(d) <= width_px / 2] = 45
    return im.astype(np.uint8)


def test_with_tile_m_set_the_floor_is_READ_and_the_grid_PUBLISHED():
    n = _node(tile_m=0.025)
    try:
        n._read_the_floor(_tiles(angle_deg=10.0), 1.0)
        assert n._tile_height is not None and n._tile_height > 0.0
        assert len(n.published['grid']) == 1
        assert abs(n.published['grid'][0] - 10.0) < 1.5, n.published['grid']
    finally:
        n.destroy_node()


def test_with_lane_lines_on_the_lane_heading_is_PUBLISHED():
    n = _node(lane_lines=True)
    try:
        n._read_the_floor(_lane(angle_deg=33.0), 1.0)
        assert len(n.published['lane']) == 1, n.published
        assert abs(n.published['lane'][0] - 33.0) < 1.0, n.published['lane']
        assert n.published['grid'] == []          # tile_m still off
    finally:
        n.destroy_node()


def test_both_OFF_by_default_publish_nothing():
    n = _node()
    try:
        n._read_the_floor(_lane(), 1.0)
        n._read_the_floor(_tiles(), 2.0)
        assert n.published == {'grid': [], 'lane': []}
    finally:
        n.destroy_node()
