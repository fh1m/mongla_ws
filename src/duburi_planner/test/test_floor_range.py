"""DuburiMission.floor_range EXECUTED on a fake mission, against constructed truth."""
from unittest.mock import MagicMock

import pytest

from duburi_planner.duburi_dsl import DuburiMission

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
    got = DuburiMission.floor_range(_m([_drum_at(3.0, 1.0)]), 'drum',
                                    plane_below_m=1.0, pitch_deg=0.0)
    assert got[0] == pytest.approx(3.0, rel=1e-9)
    assert got[1] > 0.0


def test_it_reads_the_FOOT_not_the_box_centre():
    # Same foot, twice the box height: the centre moves, the range must not.
    a = DuburiMission.floor_range(_m([_drum_at(3.0, 1.0, 40.0)]), 'drum',
                                  plane_below_m=1.0, pitch_deg=0.0)
    b = DuburiMission.floor_range(_m([_drum_at(3.0, 1.0, 120.0)]), 'drum',
                                  plane_below_m=1.0, pitch_deg=0.0)
    assert a[0] == pytest.approx(b[0], rel=1e-9)


def test_a_surface_hung_target_reads_its_TOP_edge():
    top_v = 240.0 - F * 0.3 / 2.5   # 6.8 deg; at 4 m it is 4.3 and REFUSED
    box = ('gate', 320.0, top_v + 50.0, 300.0, 100.0, 0.9)
    got = DuburiMission.floor_range(_m([box]), 'gate', plane_below_m=-0.3, pitch_deg=0.0)
    assert got[0] == pytest.approx(2.5, rel=1e-9)


def test_a_foot_cut_by_the_frame_bottom_is_refused():
    box = ('drum', 320.0, 440.0, 40.0, 80.0, 0.8)      # bottom edge at 480
    assert DuburiMission.floor_range(_m([box]), 'drum', plane_below_m=1.0, pitch_deg=0.0) is None


def test_nothing_visible_is_None():
    assert DuburiMission.floor_range(_m([]), 'drum', plane_below_m=1.0, pitch_deg=0.0) is None


def test_a_gate_bar_too_far_to_graze_is_refused_not_guessed():
    top_v = 240.0 - F * 0.3 / 4.0                      # 4.3 deg < MIN_GRAZING_DEG
    box = ('gate', 320.0, top_v + 50.0, 300.0, 100.0, 0.9)
    assert DuburiMission.floor_range(_m([box]), 'gate', plane_below_m=-0.3, pitch_deg=0.0) is None


def test_pitch_has_no_default_because_a_level_hull_is_an_assumption():
    with pytest.raises(TypeError):
        DuburiMission.floor_range(_m([_drum_at(3.0, 1.0)]), 'drum', plane_below_m=1.0)


def test_a_nose_up_hull_is_corrected_by_passing_negative_pitch():
    # Hull pitched 2 deg nose-up: the level-camera answer is wrong, the corrected one is right.
    import math
    p = math.radians(-2.0)
    a, b = 0.0, 1.0 / 3.0                      # truth: floor point 3 m ahead, 1 m below
    # pixel row for the hull-frame ray (1, 0, 1/3) seen by a camera pitched p below horizontal
    zc = math.cos(p) * 1.0 + math.sin(p) * b
    yc = -math.sin(p) * 1.0 + math.cos(p) * b
    box = ('drum', 320.0, 240.0 + F * yc / zc - 30.0, 40.0, 60.0, 0.8)
    right = DuburiMission.floor_range(_m([box]), 'drum', plane_below_m=1.0, pitch_deg=-2.0)
    level = DuburiMission.floor_range(_m([box]), 'drum', plane_below_m=1.0, pitch_deg=0.0)
    assert right[0] == pytest.approx(3.0, rel=1e-9)
    assert abs(level[0] - 3.0) > 0.2


def test_the_calibrated_principal_point_is_used_not_the_frame_centre():
    # The forward camera's cy sits 70 px above centre at 720 p. A foot placed
    # for THAT principal point must range true; the frame-centre reading would
    # be ~6 deg of pitch off.
    m = _m([], size=(1280.0, 720.0))
    m._cam_k = {'forward': (F, F, 675.4, 290.3)}
    foot_v = 290.3 + F * 1.0 / 3.0
    m._records.return_value = [('drum', 675.4, foot_v - 30.0, 40.0, 60.0, 0.8)]
    got = DuburiMission.floor_range(m, 'drum', plane_below_m=1.0, pitch_deg=0.0)
    assert got[0] == pytest.approx(3.0, rel=1e-6)
