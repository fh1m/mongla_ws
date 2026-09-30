"""Flow scale uses the LENS height, not the barometer's (issue #27).

Scale is `h / (f * dt)`, so a vertical gap between the Bar30 port and the lens
is a fixed multiplicative error on every velocity and distance.
"""
import pytest

from mongla_vision.flow.flow_math import height_above_floor


def test_unmeasured_is_the_old_baro_height_exactly():
    assert height_above_floor(1.6, -0.9) == pytest.approx(0.7)
    assert height_above_floor(1.6, -0.9, None) == pytest.approx(0.7)


def test_a_lens_below_the_port_is_NEARER_the_floor():
    """FRD z is down: lens z - baro z = +0.10 means the lens is 10 cm lower."""
    assert height_above_floor(1.6, -0.9, +0.10) == pytest.approx(0.60)


def test_the_size_of_the_error_it_removes():
    """#27's case: port 10 cm above the lens, lens at 0.60 m. Uncorrected, the
    baro height is 0.70 -- every velocity 16.7 % long."""
    baro_h = height_above_floor(1.6, -0.9)
    lens_h = height_above_floor(1.6, -0.9, 0.10)
    assert baro_h / lens_h - 1.0 == pytest.approx(0.1667, abs=1e-3)


def test_an_offset_that_puts_the_lens_under_the_floor_is_refused():
    assert height_above_floor(1.0, -0.95, 0.10) is None


def test_both_flow_nodes_read_the_offset_from_the_one_frames_file():
    """No capability ships unreachable: both callers must pass it."""
    import pathlib
    root = pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision' / 'flow'
    for name in ('flow_node.py', 'distance_estimation_node.py'):
        src = (root / name).read_text(encoding='utf-8')
        assert 'baro_to_downward_cam_z()' in src, name
        assert 'self._lens_below_baro)' in src, name
