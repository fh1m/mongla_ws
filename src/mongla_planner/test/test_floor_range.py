"""MonglaMission.floor_range EXECUTED on a fake mission, against constructed truth."""
from unittest.mock import MagicMock

import pytest

from mongla_planner.mongla_dsl import MonglaMission

F = 500.0


def _m(boxes, size=(640.0, 480.0)):
    m = MagicMock()
    m.camera = 'forward'
    m._img_size = {'forward': size}
    m.focal_px.return_value = F
    m._records.return_value = boxes
    return m


def _drum_at(range_m, below_m, h_px=60.0):
    """A box whose FOOT sits where a level camera sees floor at range_m."""
    foot_v = 240.0 + F * below_m / range_m
    return ('drum', 320.0, foot_v - h_px / 2.0, 40.0, h_px, 0.8)


def test_the_foot_on_the_floor_gives_the_range_with_no_width():
    got = MonglaMission.floor_range(_m([_drum_at(3.0, 1.0)]), 'drum',
                                    plane_below_m=1.0, pitch_deg=0.0)
    assert got[0] == pytest.approx(3.0, rel=1e-9)
    assert got[1] > 0.0


def test_it_reads_the_FOOT_not_the_box_centre():
    # Same foot, twice the box height: the centre moves, the range must not.
    a = MonglaMission.floor_range(_m([_drum_at(3.0, 1.0, 40.0)]), 'drum',
                                  plane_below_m=1.0, pitch_deg=0.0)
    b = MonglaMission.floor_range(_m([_drum_at(3.0, 1.0, 120.0)]), 'drum',
                                  plane_below_m=1.0, pitch_deg=0.0)
    assert a[0] == pytest.approx(b[0], rel=1e-9)


def test_a_surface_hung_target_reads_its_TOP_edge():
    top_v = 240.0 - F * 0.3 / 2.5   # 6.8 deg; at 4 m it is 4.3 and REFUSED
    box = ('gate', 320.0, top_v + 50.0, 300.0, 100.0, 0.9)
    got = MonglaMission.floor_range(_m([box]), 'gate', plane_below_m=-0.3, pitch_deg=0.0)
    assert got[0] == pytest.approx(2.5, rel=1e-9)


def test_a_foot_cut_by_the_frame_bottom_is_refused():
    box = ('drum', 320.0, 440.0, 40.0, 80.0, 0.8)      # bottom edge at 480
    assert MonglaMission.floor_range(_m([box]), 'drum', plane_below_m=1.0, pitch_deg=0.0) is None


def test_nothing_visible_is_None():
    assert MonglaMission.floor_range(_m([]), 'drum', plane_below_m=1.0, pitch_deg=0.0) is None


def test_a_gate_bar_too_far_to_graze_is_refused_not_guessed():
    top_v = 240.0 - F * 0.3 / 4.0                      # 4.3 deg < MIN_GRAZING_DEG
    box = ('gate', 320.0, top_v + 50.0, 300.0, 100.0, 0.9)
    assert MonglaMission.floor_range(_m([box]), 'gate', plane_below_m=-0.3, pitch_deg=0.0) is None


def test_pitch_has_no_default_because_a_level_hull_is_an_assumption():
    with pytest.raises(TypeError):
        MonglaMission.floor_range(_m([_drum_at(3.0, 1.0)]), 'drum', plane_below_m=1.0)


def test_a_nose_up_hull_is_corrected_by_passing_negative_pitch():
    # Hull pitched 2 deg nose-up: the level-camera answer is wrong, the corrected one is right.
    import math
    p = math.radians(-2.0)
    a, b = 0.0, 1.0 / 3.0                      # truth: floor point 3 m ahead, 1 m below
    # pixel row for the hull-frame ray (1, 0, 1/3) seen by a camera pitched p below horizontal
    zc = math.cos(p) * 1.0 + math.sin(p) * b
    yc = -math.sin(p) * 1.0 + math.cos(p) * b
    box = ('drum', 320.0, 240.0 + F * yc / zc - 30.0, 40.0, 60.0, 0.8)
    right = MonglaMission.floor_range(_m([box]), 'drum', plane_below_m=1.0, pitch_deg=-2.0)
    level = MonglaMission.floor_range(_m([box]), 'drum', plane_below_m=1.0, pitch_deg=0.0)
    assert right[0] == pytest.approx(3.0, rel=1e-9)
    assert abs(level[0] - 3.0) > 0.2


def _through_port(X, Y, Z, fx, fy, cx, cy, n=1.333):
    """Pixel of a WATER point seen through a flat port (ray trace, not the code)."""
    import math
    rw = math.hypot(X / Z, Y / Z)
    if rw == 0.0:
        return cx, cy
    ra = math.tan(math.asin(n * math.sin(math.atan(rw))))
    return cx + fx * (X / Z) * ra / rw, cy + fy * (Y / Z) * ra / rw


FANTECH = (851.23, 858.15, 675.42, 290.28)     # pi_forward_1280x720.json, air


def test_floor_range_is_true_through_a_flat_port_off_centre():
    """A drum 3.0 m ahead and 1.0 m right, its foot 1.0 m below the camera, seen
    by the forward camera through the port. Ray-traced pixels; the answer must be
    the true horizontal range, not the range of a pinhole with one focal."""
    import math
    fx, fy, cx, cy = FANTECH
    fwd, right, below = 3.0, 1.0, 1.0
    u, v = _through_port(right, below, fwd, fx, fy, cx, cy)
    m = _m([('drum', u, v - 30.0, 40.0, 60.0, 0.8)], size=(1280.0, 720.0))
    m._cam_k = {'forward': FANTECH}
    got = MonglaMission.floor_range(m, 'drum', plane_below_m=below, pitch_deg=0.0)
    assert got[0] == pytest.approx(math.hypot(fwd, right), rel=0.005)


def test_range_to_is_true_through_a_flat_port_off_centre(monkeypatch):
    """A 1.5 m gate at 4.0 m, 1.2 m off-axis: edges ray-traced through the port."""
    import mongla_vision.target_geometry as tg
    monkeypatch.setattr(tg, 'width_for', lambda name: 1.5)
    fx, fy, cx, cy = FANTECH
    Z, X = 4.0, 1.2
    ul, v = _through_port(X - 0.75, 0.0, Z, fx, fy, cx, cy)
    ur, _ = _through_port(X + 0.75, 0.0, Z, fx, fy, cx, cy)
    m = _m([('gate', (ul + ur) / 2.0, v, ur - ul, 200.0, 0.9)], size=(1280.0, 720.0))
    m._cam_k = {'forward': FANTECH}
    got = MonglaMission.range_to(m, 'gate')
    assert got[0] == pytest.approx(Z, rel=0.01)


def test_air_uses_the_calibration_as_a_plain_pinhole():
    """On the bench (medium=air) nothing is refracted: a pinhole-projected foot
    must range true with the same K, or a bench check would read ~25 % short."""
    import math
    fx, fy, cx, cy = FANTECH
    fwd, right, below = 3.0, 1.0, 1.0
    u, v = cx + fx * right / fwd, cy + fy * below / fwd
    m = _m([('drum', u, v - 30.0, 40.0, 60.0, 0.8)], size=(1280.0, 720.0))
    m._cam_k = {'forward': FANTECH}
    m.medium = 'air'
    got = MonglaMission.floor_range(m, 'drum', plane_below_m=below, pitch_deg=0.0)
    assert got[0] == pytest.approx(math.hypot(fwd, right), rel=1e-6)
