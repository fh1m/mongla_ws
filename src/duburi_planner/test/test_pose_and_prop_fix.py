"""The mission-facing half of localization: EXECUTED, not grepped.

Every test here calls the real unbound method with a fake mission, the same
pattern `test_fix_position.py` uses. That matters more than usual for this
surface: `pose()`, `range_to()` and `fix_from_prop()` reached the vehicle
covered only by a reachability test that greps for their names, and a grep
passes whether or not the body raises.
"""
import math

from unittest.mock import MagicMock

import pytest

from duburi_localization.course_map import Course, Prop
from duburi_planner.duburi_dsl import DuburiMission


class _Odom:
    def __init__(self, frame='pool', x=1.5, y=-2.0, z=-1.2, yaw_deg=30.0):
        r = math.radians(yaw_deg) * 0.5
        self.header = type('H', (), {'frame_id': frame})()
        pos = type('P', (), {'x': x, 'y': y, 'z': z})()
        quat = type('Q', (), {'w': math.cos(r), 'x': 0.0,
                              'y': 0.0, 'z': math.sin(r)})()
        inner = type('I', (), {'position': pos, 'orientation': quat})()
        self.pose = type('PP', (), {'pose': inner})()


def _mission(odom=None, warned=None):
    m = MagicMock()
    m._odom = odom
    m._odom_sub = object()          # already subscribed: skip the ROS path
    m._pose_frame_warned = False
    m.log = MagicMock()
    return m


# --------------------------------------------------------------------------- #
#  pose(): absence is safe, a plausible wrong number is not
# --------------------------------------------------------------------------- #
def test_a_pool_frame_pose_is_returned_and_decoded():
    got = DuburiMission.pose(_mission(_Odom(yaw_deg=30.0)))
    assert got is not None
    x, y, z, yaw = got
    assert x == pytest.approx(1.5)
    assert y == pytest.approx(-2.0)
    assert z == pytest.approx(-1.2)
    assert yaw == pytest.approx(30.0, abs=1e-6)


def test_an_UNANCHORED_pose_is_withheld():
    """⛔ Before the anchor the estimate is in the board's boot frame, so
    comparing it against a course coordinate compares across a rotation. The
    numbers look exactly like a good reading."""
    m = _mission(_Odom(frame='odom'))
    assert DuburiMission.pose(m) is None
    assert m.log.warning.called


def test_the_withholding_warning_fires_once_not_every_call():
    m = _mission(_Odom(frame='odom'))
    for _ in range(5):
        DuburiMission.pose(m)
    assert m.log.warning.call_count == 1


def test_any_frame_opts_in_to_the_unanchored_reading():
    """A diagnostic may genuinely want to watch the filter converge. Opt-in,
    so it cannot be what a mission does by accident."""
    got = DuburiMission.pose(_mission(_Odom(frame='odom')), any_frame=True)
    assert got is not None and got[0] == pytest.approx(1.5)


def test_no_odom_at_all_is_None():
    """With nothing published the call must time out and answer None.

    `timeout=0` still takes the deadline path; a real `Node` is needed because
    the method pumps the executor rather than sleeping (the mission thread owns
    it, so a bare sleep would receive nothing).
    """
    import rclpy
    from rclpy.node import Node

    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = Node('pose_timeout_probe')
    try:
        m = _mission(None)
        m.client = type('C', (), {'node': node})()
        assert DuburiMission.pose(m, timeout=0.05) is None
    finally:
        node.destroy_node()
        if started and rclpy.ok():
            rclpy.shutdown()


def test_yaw_decodes_across_the_wrap():
    for deg in (0.0, 89.0, 179.0, -179.0, -90.0):
        got = DuburiMission.pose(_mission(_Odom(yaw_deg=deg)))
        assert got[3] == pytest.approx(deg, abs=1e-6)


# --------------------------------------------------------------------------- #
#  range_to(): the pinhole range and its RANGE-DEPENDENT sigma
# --------------------------------------------------------------------------- #
def _ranger(boxes, width_px=640.0, f_px=500.0):
    m = MagicMock()
    m.camera = 'forward'
    m._img_size = {'forward': (width_px, 480.0)}
    m.focal_px.return_value = f_px
    m._records.return_value = boxes
    return m


def test_range_is_the_pinhole_relation():
    # width_for('gate') is the real table; use a class that is in it.
    from duburi_vision.target_geometry import width_for
    w = width_for('gate')
    if w <= 0.0:
        pytest.skip('no gate width in the table')
    # A box of f*W/Z px must read back as Z.
    f_px, z_true = 500.0, 3.0
    px = f_px * w / z_true
    got = DuburiMission.range_to(_ranger([('gate', 0.0, 0.0, px, px, 0.9)],
                                         f_px=f_px), 'gate')
    assert got is not None
    z, sigma = got
    assert z == pytest.approx(z_true, rel=1e-6)


def test_sigma_grows_with_the_SQUARE_of_range():
    """`dZ/dw = -Z^2/(fW)`. A constant sigma would tell the filter a 5 m
    reading is as good as a 1 m one, which is how a landmark fix poisons a
    position estimate."""
    from duburi_vision.target_geometry import width_for
    w = width_for('gate')
    if w <= 0.0:
        pytest.skip('no gate width in the table')
    f_px = 500.0
    near = DuburiMission.range_to(
        _ranger([('gate', 0, 0, f_px * w / 1.0, 10, 0.9)], f_px=f_px), 'gate')
    far = DuburiMission.range_to(
        _ranger([('gate', 0, 0, f_px * w / 2.0, 10, 0.9)], f_px=f_px), 'gate')
    # Twice the range, four times the sigma.
    assert far[1] / near[1] == pytest.approx(4.0, rel=0.01)


def test_the_LARGEST_box_sets_the_range_not_the_most_confident():
    """Size is what range is read off. A small confident box is a worse range
    than a large doubtful one -- same rule as `identity.pick_structure`."""
    from duburi_vision.target_geometry import width_for
    if width_for('gate') <= 0.0:
        pytest.skip('no gate width in the table')
    boxes = [('gate', 0, 0, 20.0, 10, 0.99), ('gate', 0, 0, 80.0, 10, 0.30)]
    z, _ = DuburiMission.range_to(_ranger(boxes), 'gate')
    z_small, _ = DuburiMission.range_to(
        _ranger([('gate', 0, 0, 20.0, 10, 0.99)]), 'gate')
    assert z < z_small          # the bigger box is the nearer, chosen, range


def test_an_unknown_class_has_no_range():
    assert DuburiMission.range_to(_ranger([]), 'not_a_real_prop_xyz') is None


def test_a_class_with_no_box_has_no_range():
    from duburi_vision.target_geometry import width_for
    if width_for('gate') <= 0.0:
        pytest.skip('no gate width in the table')
    assert DuburiMission.range_to(_ranger([]), 'gate') is None


# --------------------------------------------------------------------------- #
#  fix_from_prop(): one prop pins the hull
# --------------------------------------------------------------------------- #
def _course(**placed):
    props = {}
    for name, xy in placed.items():
        props[name] = Prop(name, x_m=xy[0], y_m=xy[1], detect_class=name,
                           measured=True)
    return Course('t', props)


def _prop_fixer(course, *, offset=0.0, bearing=0.0, rng=(3.0, 0.09)):
    m = MagicMock()
    m._course = course
    m._heading_offset = offset
    m.camera = 'forward'
    m.log = MagicMock()
    m.bearing_to.return_value = bearing
    m.range_to.return_value = rng
    m._publish_fix = MagicMock()
    return m


def test_one_prop_at_a_known_bearing_and_range_places_the_hull():
    """⛔ THE DVL WE DO NOT HAVE. `fix_position()` needs TWO props 12 degrees
    apart; one prop of known width at a surveyed position pins the hull on its
    own, which is visible far more often."""
    m = _prop_fixer(_course(gate=(10.0, 0.0)), bearing=0.0, rng=(4.0, 0.16))
    got = DuburiMission.fix_from_prop(m, 'gate')
    assert got.ok
    # Due north of us at 4 m, so we are 4 m back along that bearing.
    assert got.x_m == pytest.approx(6.0, abs=1e-6)
    assert got.y_m == pytest.approx(0.0, abs=1e-6)


def test_the_bearing_geometry_is_right_at_ninety_degrees():
    m = _prop_fixer(_course(gate=(5.0, 5.0)), bearing=90.0, rng=(2.0, 0.04))
    got = DuburiMission.fix_from_prop(m, 'gate')
    assert got.ok
    assert got.x_m == pytest.approx(5.0, abs=1e-6)
    assert got.y_m == pytest.approx(3.0, abs=1e-6)


def test_the_range_sigma_is_carried_to_the_filter():
    m = _prop_fixer(_course(gate=(10.0, 0.0)), rng=(4.0, 0.16))
    DuburiMission.fix_from_prop(m, 'gate')
    _args, kwargs = m._publish_fix.call_args
    assert kwargs['sigma'] == pytest.approx(0.16)


def test_an_unanchored_heading_is_refused():
    m = _prop_fixer(_course(gate=(10.0, 0.0)))
    m._heading_offset = None
    got = DuburiMission.fix_from_prop(m, 'gate')
    assert not got.ok and 'anchor_on' in got.reason
    assert not m._publish_fix.called


def test_a_prop_with_no_surveyed_position_is_refused():
    course = Course('t', {'gate': Prop('gate', detect_class='gate')})
    got = DuburiMission.fix_from_prop(_prop_fixer(course), 'gate')
    assert not got.ok and 'position' in got.reason


def test_a_prop_that_is_not_visible_is_refused():
    m = _prop_fixer(_course(gate=(10.0, 0.0)))
    m.bearing_to.return_value = None
    got = DuburiMission.fix_from_prop(m, 'gate')
    assert not got.ok and 'visible' in got.reason
    assert not m._publish_fix.called


def test_a_prop_with_no_width_has_no_range_and_is_refused():
    m = _prop_fixer(_course(gate=(10.0, 0.0)))
    m.range_to.return_value = None
    got = DuburiMission.fix_from_prop(m, 'gate')
    assert not got.ok and 'range' in got.reason


def test_no_course_is_refused_by_name():
    got = DuburiMission.fix_from_prop(_prop_fixer(None), 'gate')
    assert not got.ok and 'use_course' in got.reason


def test_a_prop_not_in_the_course_is_refused():
    got = DuburiMission.fix_from_prop(_prop_fixer(_course(gate=(1.0, 1.0))),
                                      'slalom')
    assert not got.ok and 'slalom' in got.reason
