"""Closing the dead zone: a computed standoff, then a search that can be preempted.

The failure being removed is a hull parked where the prop is present and
undetectable, with the mission unable to tell "not there" from "too far to
see". The two tests that matter are the one proving the standoff comes from the
prop rather than a habit, and the one proving a detection abandons a search leg
mid-flight instead of after it.
"""
import math

from unittest.mock import MagicMock

import pytest

from mongla_planner.mongla_dsl import MonglaMission
from mongla_localization.resection import Fix


def _fake(seen_after=None, fix_ok=True):
    """`seen_after` = number of detected() calls before the class appears."""
    m = MagicMock()
    m.HFOV_WATER_DEG_BY_CAMERA = MonglaMission.HFOV_WATER_DEG_BY_CAMERA
    m._course = None
    m.focal_px = MonglaMission.focal_px.__get__(m)
    m.standoff_for_prop = MonglaMission.standoff_for_prop.__get__(m)
    m._run_watching = MonglaMission._run_watching.__get__(m)
    m.fix_position.return_value = Fix(fix_ok, x_m=0.0, y_m=0.0,
                                      residual_m=0.2, separation_deg=60.0,
                                      reason='' if fix_ok else 'no fix')
    calls = {'n': 0}

    def detected(cls, camera=None, stale_after=1.0):
        calls['n'] += 1
        return seen_after is not None and calls['n'] > seen_after
    m.detected.side_effect = detected
    m.detected_calls = calls
    return m


# --- the focal, derived from one measurement --------------------------------


def test_the_focal_is_derived_from_the_measured_field_of_view():
    # Downward (global shutter): 640 px at 46.7 deg water gives 741 px, which
    # the flow node independently reports as its water focal.
    m = _fake()
    m.camera = 'downward'
    assert MonglaMission.focal_px(m, 640.0) == pytest.approx(741.0, abs=1.5)


def test_each_camera_has_its_own_focal():
    # The forward camera is the Fantech (73.9 deg air -> 53.6 water). One shared
    # 46.7 constant made every forward range 17 % long.
    m = _fake()
    assert MonglaMission.focal_px(m, 640.0, camera='forward') == pytest.approx(634.0, abs=2.0)
    assert MonglaMission.focal_px(m, 640.0, camera='downward') == pytest.approx(741.0, abs=1.5)


def test_a_live_calibration_beats_the_fallback():
    # CameraInfo for the forward camera at 1280x720, fx 851.23 (air). Through
    # the port that is 53.6 deg; a DIFFERENT K must move the focal with it.
    m = _fake()
    m._img_size = {'forward': (1280.0, 720.0)}
    m._cam_k = {'forward': (851.23, 858.15, 675.42, 290.28)}
    assert MonglaMission.hfov_water_deg(m, 'forward') == pytest.approx(53.6, abs=0.1)
    m._cam_k = {'forward': (1027.87, 1033.86, 617.3, 373.0)}
    assert MonglaMission.hfov_water_deg(m, 'forward') == pytest.approx(46.7, abs=0.1)


def test_the_fallback_table_matches_the_committed_calibrations():
    import json, math
    from pathlib import Path
    from mongla_vision.optics import fov_air_to_water
    cal = Path(__file__).resolve().parents[2] / 'mongla_vision' / 'config' / 'calibration'
    for cam in ('forward', 'downward'):
        d = json.loads((cal / f'pi_{cam}_1280x720.json').read_text())
        fx, w = d['camera_matrix'][0][0], d['image_width']
        water = fov_air_to_water(2.0 * math.degrees(math.atan(w / 2.0 / fx)))
        assert MonglaMission.HFOV_WATER_DEG_BY_CAMERA[cam] == pytest.approx(water, abs=0.1), cam


# --- the standoff comes from the prop ---------------------------------------


def test_a_slalom_pipe_is_pixel_limited_and_a_gate_is_water_limited():
    fake = _fake()
    pipe = fake.standoff_for_prop('red_pipe', visibility_m=6.0)
    gate = fake.standoff_for_prop('gate', visibility_m=6.0)
    assert pipe < 2.0, 'a 33 mm pipe is only detectable from very close'
    assert gate == pytest.approx(6.0), 'a gate runs out of water, not pixels'


def test_an_unknown_class_falls_back_to_visibility_not_to_zero():
    # A standoff of 0.0 would drive the hull into the prop.
    assert _fake().standoff_for_prop('not_a_prop', visibility_m=5.0) == pytest.approx(5.0)


# --- the approach ------------------------------------------------------------


def test_seeing_it_after_the_approach_skips_the_search():
    fake = _fake(seen_after=0)                       # visible immediately
    got = MonglaMission.acquire(fake, 'gate')
    assert got.ok and got.branch == 'approach'
    fake.yaw_right.assert_not_called()


# --- the search --------------------------------------------------------------


def test_not_seeing_it_starts_an_expanding_search():
    fake = _fake(seen_after=None)                    # never visible
    got = MonglaMission.acquire(fake, 'gate', max_legs=4)
    assert not got.ok
    assert 'not where the map says' in got.error
    assert fake.yaw_right.call_count == 4


def test_a_detection_abandons_the_leg_MID_FLIGHT():
    # THE POINT. A search that finishes its pattern before looking has already
    # swum past the answer. `detected` starts returning True during the first
    # leg, so the run must stop before that leg's full distance is covered.
    fake = _fake(seen_after=2)
    got = MonglaMission.acquire(fake, 'gate', max_legs=6)
    assert got.ok and got.branch.startswith('leg')
    total = sum(c.args[0] for c in fake.move_forward_dist.call_args_list)
    assert total < 3.0, f'kept swimming after the target appeared ({total} m)'


def test_the_legs_are_closed_with_the_distance_verb_never_timed():
    fake = _fake(seen_after=None)
    MonglaMission.acquire(fake, 'gate', max_legs=2)
    assert fake.move_forward_dist.called
    assert not fake.move_forward.called, 'a timed leg is a guessed leg'


def test_the_search_is_sized_from_the_fix_residual():
    # A worse fix must search a bigger box, without anyone choosing a number.
    tight = _fake(seen_after=None)
    tight.fix_position.return_value = Fix(True, residual_m=0.05,
                                          separation_deg=60.0)
    loose = _fake(seen_after=None)
    loose.fix_position.return_value = Fix(True, residual_m=2.0,
                                          separation_deg=60.0)
    MonglaMission.acquire(tight, 'gate', max_legs=6)
    MonglaMission.acquire(loose, 'gate', max_legs=6)
    t = sum(c.args[0] for c in tight.move_forward_dist.call_args_list)
    l = sum(c.args[0] for c in loose.move_forward_dist.call_args_list)
    assert l > t


def test_run_watching_stops_at_the_requested_distance_when_nothing_appears():
    fake = _fake(seen_after=None)
    assert MonglaMission._run_watching(fake, 2.0, 'gate', None, 1.0) is False
    total = sum(c.args[0] for c in fake.move_forward_dist.call_args_list)
    assert total == pytest.approx(2.0)
