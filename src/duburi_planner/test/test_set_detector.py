"""`set_detector` reaches any declared detector parameter from a mission.

A capability that is not in the DSL cannot be used by a mission, and this
session added two the DSL could not touch: `masks` (mask decode on/off, the
dominant host cost -- 0.93 ms boxes-only against 2.98 ms with masks) and
`publish_contours`. One generic setter covers those and every parameter added
later, instead of a new DSL method per knob.
"""
from unittest.mock import MagicMock

import pytest

from duburi_planner.duburi_dsl import DuburiMission


def _fake():
    m = MagicMock()
    m._detector_node.return_value = '/duburi_detector_forward'
    return m


def test_one_parameter_is_written_to_the_named_node():
    fake = _fake()
    DuburiMission.set_detector(fake, masks=False)
    fake._set_detector_param.assert_called_once_with(
        '/duburi_detector_forward', 'masks', False)


def test_several_parameters_in_one_call():
    fake = _fake()
    DuburiMission.set_detector(fake, max_det=10, iou=0.5)
    written = {c.args[1]: c.args[2] for c in fake._set_detector_param.call_args_list}
    assert written == {'max_det': 10, 'iou': 0.5}


def test_an_empty_call_is_refused():
    # A no-arg call is a mission that thinks it changed something.
    with pytest.raises(ValueError):
        DuburiMission.set_detector(_fake())


def test_a_rejected_parameter_raises_rather_than_warning():
    # Silence here would be a mission believing a knob it depends on took.
    fake = _fake()
    fake._set_detector_param.side_effect = RuntimeError('rejected: not declared')
    with pytest.raises(RuntimeError):
        DuburiMission.set_detector(fake, nonsense=1)


def test_the_camera_selects_the_node():
    fake = _fake()
    DuburiMission.set_detector(fake, camera='downward', masks=True)
    assert fake._detector_node.call_args.args[0] == 'downward'


# --- set_node: every subsystem in the stack, not just the detector ----------
#
# A census of the tree found 164 declared parameters across 13 nodes and a DSL
# that could write two groups: the detector, and the manager's vision.*. The
# tracker's coast, the camera's exposure, the lock ladder's authority windows,
# the flow front end and PnP were launch-only -- frozen at whatever was typed
# before the vehicle went in the water.


def _node_fake(camera='forward'):
    m = MagicMock()
    m.camera = camera
    m._NODE_SUFFIX = DuburiMission._NODE_SUFFIX
    m._subsystem_node = lambda kind, cam=None: DuburiMission._subsystem_node(
        m, kind, cam)
    return m


@pytest.mark.parametrize('kind,expect', [
    ('detector', '/duburi_detector_forward'),
    ('camera',   '/duburi_camera_forward'),
    ('tracker',  '/duburi_tracker_forward'),
    ('lock',     '/duburi_lock_forward'),
    ('pnp',      '/duburi_pnp_forward'),
    ('flow',     '/duburi_flow_velocity'),
    ('manager',  '/duburi_manager'),
])
def test_each_subsystem_resolves_to_its_node(kind, expect):
    assert DuburiMission._subsystem_node(_node_fake(), kind) == expect


def test_the_camera_selects_the_per_camera_instance():
    fake = _node_fake()
    assert DuburiMission._subsystem_node(
        fake, 'tracker', 'downward') == '/duburi_tracker_downward'


def test_single_instance_nodes_ignore_the_camera():
    fake = _node_fake()
    assert DuburiMission._subsystem_node(
        fake, 'flow', 'downward') == '/duburi_flow_velocity'


def test_an_unknown_subsystem_is_refused_by_name():
    with pytest.raises(ValueError):
        DuburiMission._subsystem_node(_node_fake(), 'sonar')


def test_set_node_writes_to_the_resolved_node():
    fake = _node_fake()
    DuburiMission.set_node(fake, 'tracker', coast_s=1.2)
    fake._set_node_param.assert_called_once_with(
        '/duburi_tracker_forward', 'coast_s', 1.2)


def test_set_node_refuses_an_empty_call():
    with pytest.raises(ValueError):
        DuburiMission.set_node(_node_fake(), 'tracker')


# --- side_on: the gate's divider, not the frame centre ----------------------


def _rec(cls, cx, cy, w, h, score=0.9):
    return (cls, float(cx), float(cy), float(w), float(h), float(score))


def _side_fake(records, camera='forward'):
    m = MagicMock()
    m.camera = camera
    m._resolve_camera = lambda c: c or camera
    m._records.return_value = records
    return m


def test_side_on_measures_against_the_structure():
    # Gate spans 200..600 (midline 400); placard at 250 is on its LEFT half.
    recs = [_rec('gate', 400, 250, 400, 300), _rec('rescue', 250, 170, 40, 40)]
    assert DuburiMission.side_on(_side_fake(recs), 'rescue') == 'left'


def test_side_on_survives_an_off_axis_hull():
    # The gate sits left in the image (0..400, midline 200). The placard at 300
    # is on the gate's RIGHT -- but LEFT of a 640-wide frame's centre, which is
    # what where() would have said.
    recs = [_rec('gate', 200, 250, 400, 300), _rec('repair', 300, 170, 40, 40)]
    assert DuburiMission.side_on(_side_fake(recs), 'repair') == 'right'


def test_side_on_is_unknown_without_the_structure():
    recs = [_rec('rescue', 250, 170, 40, 40)]
    assert DuburiMission.side_on(_side_fake(recs), 'rescue') == 'unknown'


def test_side_on_is_unknown_when_the_symbol_is_not_on_the_structure():
    recs = [_rec('gate', 400, 250, 400, 300), _rec('rescue', 900, 170, 40, 40)]
    assert DuburiMission.side_on(_side_fake(recs), 'rescue') == 'unknown'


def test_side_on_with_no_frame_is_unknown():
    assert DuburiMission.side_on(_side_fake([]), 'rescue') == 'unknown'


# --- a duplicate node name makes a parameter write a coin flip --------------


def _graph(names):
    m = MagicMock()
    m.client.node.get_node_names.return_value = names
    return m


def test_a_duplicated_node_name_refuses_the_write():
    fake = _graph(['duburi_tracker_forward'] * 10 + ['duburi_manager'])
    with pytest.raises(RuntimeError) as exc:
        DuburiMission._refuse_duplicate_node(fake, '/duburi_tracker_forward')
    assert '10 nodes' in str(exc.value)


def test_a_single_node_is_fine():
    fake = _graph(['duburi_tracker_forward', 'duburi_manager'])
    DuburiMission._refuse_duplicate_node(fake, '/duburi_tracker_forward')


def test_an_unreadable_graph_does_not_block_a_legitimate_write():
    fake = MagicMock()
    fake.client.node.get_node_names.side_effect = RuntimeError('discovery hiccup')
    DuburiMission._refuse_duplicate_node(fake, '/duburi_tracker_forward')


def test_the_leading_slash_is_not_what_makes_them_different():
    fake = _graph(['/duburi_tracker_forward', 'duburi_tracker_forward'])
    with pytest.raises(RuntimeError):
        DuburiMission._refuse_duplicate_node(fake, 'duburi_tracker_forward')
