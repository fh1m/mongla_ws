"""Every branch the package's suite never executed, each with the behaviour it owes.

Found by line coverage on 2026-11-20 (the audit's line-by-line pass). Three of
them were defects rather than missing tests: a zero quaternion fused as a
level hull, the ZUPT's accelerometer veto never exercised, and the large-angle
quaternion branches that publish /mongla/odom's orientation near 180 deg.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from mongla_localization import course_survey, floor_plane, frames, resection
from mongla_localization import localization_inputs as li
from mongla_localization import localization_publish as lp
from mongla_localization.inekf import RIEKF, State, _wrap180, so3_exp

from test_localization_node import _Imu, _node


# ── inputs ───────────────────────────────────────────────────────────────────
def test_a_zero_quaternion_is_no_attitude_not_a_level_hull():
    assert li._R_from_quat(0.0, 0.0, 0.0, 0.0) is None
    assert li._R_from_quat(float('nan'), 0.0, 0.0, 1.0) is None
    n = _node()
    n._on_imu(_Imu(100.0))      # the first sample only sets the time base
    yawed = _Imu(100.02)        # a real attitude, 40 deg off level-north
    yawed.orientation = SimpleNamespace(w=math.cos(0.35), x=0.0, y=0.0, z=math.sin(0.35))
    n._on_imu(yawed)
    held = n._board_R.copy()
    att = n._n['att']
    zero = _Imu(100.04)
    zero.orientation = SimpleNamespace(w=0.0, x=0.0, y=0.0, z=0.0)
    n._on_imu(zero)
    assert n._n['att'] == att, 'the zero quaternion was fused'
    assert np.allclose(n._board_R, held), 'the zero quaternion replaced the board attitude'


def test_a_hull_shaking_without_turning_is_not_still():
    """Gyro quiet, accelerometer swinging: a hull being pushed along a line.
    The spread veto is what keeps the ZUPT off it."""
    n = _node(disarmed=True)
    t = 100.0
    n._on_imu(_Imu(t))
    for i in range(150):
        t += 0.02
        bump = 0.4 if i % 2 else -0.4          # spread 0.8 > STILL_ACCEL_SPREAD
        n._on_imu(_Imu(t, accel=(bump, 0.0, -9.80665)))
    assert 0.8 > li.STILL_ACCEL_SPREAD
    assert n._n['zupt'] == 0


@pytest.mark.parametrize('roll, pitch, yaw', [
    (0, 0, 180), (0, 0, -179.9), (0, 0, 90), (179, 0, 0), (0, 89, 0),
    (170, 10, 175), (-120, 30, -160), (5, -3, 179.5)])
def test_the_published_quaternion_round_trips_at_every_heading(roll, pitch, yaw):
    """`_quat_from_R` has four branches, chosen by which diagonal is largest;
    the suite only ever ran the first. Near a 180 deg heading it takes the
    others, and a sign slip there turns /mongla/odom round."""
    r, p, y = (math.radians(a) for a in (roll, pitch, yaw))
    R = so3_exp(np.array([0, 0, y])) @ so3_exp(np.array([0, p, 0])) @ so3_exp(np.array([r, 0, 0]))
    q = lp._quat_from_R(R)
    assert abs(sum(c * c for c in q) - 1.0) < 1e-9
    assert np.allclose(li._R_from_quat(*q), R, atol=1e-9)


def test_a_non_finite_heading_is_ignored():
    n = _node()
    n._on_imu(_Imu(100.0))
    before = n._yaw_offset_deg
    n._on_heading(SimpleNamespace(data=float('nan')))
    assert n._yaw_offset_deg == before


def test_a_fix_without_its_own_sigma_uses_the_parameter():
    """z <= 0 means the producer had no sigma: the node's `fix_sigma` (0.5 m)
    stands in, never a zero, which would be infinite confidence."""
    hdr = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
    for z, taken in ((0.0, True), (0.01, False)):
        n = _node()
        n._fix_sigma = 0.5
        n._filter.P[6, 6] = n._filter.P[7, 7] = 0.05 ** 2
        n._on_fix(SimpleNamespace(header=hdr, point=SimpleNamespace(x=0.6, y=0.0, z=z)))
        assert (n._n['fix'] == 1) is taken, z


def test_an_inertial_gap_is_warned_once_and_coasted():
    n = _node()
    n._imu_gap_warned = False
    said = []
    n.get_logger = lambda: SimpleNamespace(warning=said.append, info=lambda *a: None)
    n._on_imu(_Imu(100.0))
    n._on_imu(_Imu(101.0))
    n._on_imu(_Imu(102.0))
    assert n._n['gap'] == 2
    assert len([m for m in said if 'inertial gap' in m]) == 1


# ── the filter ───────────────────────────────────────────────────────────────
def test_wrap180_on_both_sides():
    assert _wrap180(-190.0) == pytest.approx(170.0)
    assert _wrap180(190.0) == pytest.approx(-170.0)
    assert _wrap180(-180.0) == pytest.approx(180.0)


def test_a_yaw_update_on_a_vertical_hull_does_not_divide_by_zero():
    st = State()
    st.R = so3_exp(np.array([0.0, math.radians(90.0), 0.0]))
    f = RIEKF(state=st)
    f.P[3, 3] = f.P[4, 4] = 0.01            # velocity observed, so the gate is open
    ok = f.update_yaw(0.0, sigma_deg=2.0)
    assert isinstance(ok, bool)
    assert np.all(np.isfinite(f.P)) and np.all(np.isfinite(f.X.R))


# ── around the filter ────────────────────────────────────────────────────────
def test_resection_refuses_two_props_on_one_line():
    """Bearings 180 deg apart are one sight line: no crossing, so no position.
    The separation gate refuses it first (a line is not a ray, issue #37); with
    the gate lowered to zero, the singular-geometry guard is what remains."""
    sight, where = {'a': 90.0, 'b': 270.0}, {'a': (0.0, 5.0), 'b': (0.0, -5.0)}
    assert 'span only 0.0 deg' in resection.fix_from_bearings(sight, where).reason
    fx = resection.fix_from_bearings(sight, where, min_separation_deg=0.0)
    assert not fx.ok and 'singular' in fx.reason


def test_a_camera_with_no_focal_length_gives_no_range():
    assert floor_plane.intersect(320, 240, fx=0.0, fy=500.0, cx=320, cy=240,
                                 plane_below_m=1.0) is None


def test_the_baro_to_camera_offset_needs_both_and_is_their_difference():
    o = frames.Offsets(downward_cam=(0.1, 0.0, 0.08), forward_cam=None, baro=(0.0, 0.0, -0.02))
    assert o.baro_to_downward_cam_z() == pytest.approx(0.10)
    assert frames.Offsets(downward_cam=(0.1, 0, 0.08), forward_cam=None,
                          baro=None).baro_to_downward_cam_z() is None


def test_the_survey_cli_writes_the_deck_copy_and_refuses_a_positive_depth(
        tmp_path, monkeypatch, capsys):
    monkeypatch.setenv('MONGLA_COURSE_DIR', str(tmp_path))
    assert course_survey.main(['--course', 'robosub26', '--prop', 'gate',
                               '--depth', '1.0']) == 2
    assert course_survey.main(['--course', 'robosub26', '--prop', 'no_such_prop',
                               '--x', '1', '--y', '2']) == 2
    assert 'refused' in capsys.readouterr().out
    assert course_survey.main(['--course', 'robosub26', '--prop', 'gate',
                               '--x', '3.5', '--y', '0.0', '--bearing', '0']) == 0
    out = capsys.readouterr().out
    assert 'wrote' in out and 'MEASURED' in out
    assert (tmp_path / 'robosub26.yaml').is_file()
    with pytest.raises(FileNotFoundError):
        course_survey.survey('no_such_course', 'gate', {'x': 1.0, 'y': 1.0})


# ── the rest of the unrun lines ──────────────────────────────────────────────
def test_the_model_aid_off_does_nothing():
    n = _node()
    n._model_aid = False
    before = dict(n._n)
    n._maybe_model_aid()
    assert n._n == before


def test_a_heading_the_filter_refuses_leaves_the_offset_alone():
    """The offset and the filter's state move together or not at all; an
    offset moved alone would publish a heading the filter does not hold."""
    n = _node()
    n._on_imu(_Imu(100.0))
    n._on_imu(_Imu(100.02))
    assert n._board_R is not None
    before = n._yaw_offset_deg
    n._apply = lambda *a, **k: False
    n._on_heading(SimpleNamespace(data=90.0))
    assert n._yaw_offset_deg == before


def test_a_zero_sigma_on_a_zero_covariance_is_refused_not_a_crash():
    """S = 0 cannot be inverted. The node would have died on LinAlgError."""
    f = RIEKF()
    f.P[:] = 0.0
    assert f.update_depth(-1.0, sigma=0.0) is False


def test_a_course_whose_pool_is_not_a_mapping_is_refused(tmp_path, monkeypatch):
    from mongla_localization import course_map
    monkeypatch.setenv('MONGLA_COURSE_DIR', str(tmp_path))
    (tmp_path / 'bad.yaml').write_text('pool: 3\nprops: {}\n')
    with pytest.raises(ValueError, match='must be a mapping'):
        course_map.load_course('bad')


def test_a_lever_arm_in_millimetres_is_refused(tmp_path):
    p = tmp_path / 'frames.yaml'
    p.write_text('imu_to:\n  downward_cam: [0.0, 0.0, 120.0]\n')
    with pytest.raises(ValueError, match='millimetres'):
        frames.load(p)


def test_pose_cluster_helpers_say_nothing_rather_than_a_number():
    from mongla_localization import pose_cluster as pc
    assert math.isnan(pc._median([]))
    assert math.isnan(pc._circular_median([]))
    s, e = pc._fit([1.0, 2.0], [1.0, 2.0])
    assert math.isnan(s) and e == float('inf')
    s, e = pc._fit([1.0, 1.0, 1.0], [1.0, 2.0, 3.0])
    assert math.isnan(s) and e == float('inf')
    two = [pc.PoseSample(t=0.0, yaw_deg=0.0, range_m=3.0),
           pc.PoseSample(t=1.0, yaw_deg=0.0, range_m=5.0)]
    assert pc._range_at(two, 1.0) == pytest.approx(4.0)


def test_a_reset_cluster_starts_from_nothing():
    from mongla_localization import pose_cluster as pc
    c = pc.PoseCluster()
    c.add(pc.PoseSample(t=1.0, yaw_deg=10.0))
    c.reset()
    assert not c.fuse(now=1.0).decided


def test_a_taped_lever_arm_loads_as_metres(tmp_path):
    p = tmp_path / 'frames.yaml'
    p.write_text('imu_to:\n  downward_cam: [0.10, 0.0, 0.08]\n  baro: [0.0, 0.0, -0.02]\n')
    o = frames.load(p)
    assert o.downward_cam == (0.10, 0.0, 0.08)
    assert o.unmeasured() == ['forward_cam']


def _pair(t, bearing, true_world, swap=False, noise=0.0):
    """One frame of a planar target seen from heading 0: the true branch is
    the target's world yaw, the mirror its reflection about the viewing ray."""
    from mongla_localization import pose_cluster as pc
    a, b = true_world + noise, 2.0 * bearing - true_world - noise
    if swap:
        a, b = b, a
    return pc.PoseSample(t=t, yaw_deg=a, alt_yaw_deg=b, bearing_deg=bearing,
                         vehicle_yaw_deg=0.0, range_m=3.0)


def test_the_branches_are_tracked_through_a_label_swap():
    """The producer's first/second order is arbitrary per frame; tracking by
    continuity must put each branch back on its own track."""
    from mongla_localization import pose_cluster as pc
    frames_ = [_pair(i, 5.0 * i, 30.0, swap=i in (2, 4)) for i in range(5)]
    r = pc.resolve_mirror(frames_)
    assert r['resolved']
    flat = r['track_a'] if abs(r['slope_a']) < abs(r['slope_b']) else r['track_b']
    assert flat == pytest.approx([30.0] * 5)


def test_a_mirror_pair_too_noisy_to_resolve_is_not_an_answer():
    from mongla_localization import pose_cluster as pc
    c = pc.PoseCluster()
    for i in range(6):
        c.add(_pair(float(i), 4.0 * i, 30.0, noise=25.0 if i % 2 else -25.0))
    out = c.fuse(now=5.0)
    assert not out.decided
    assert 'unresolved' in out.reason
