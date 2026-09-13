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
