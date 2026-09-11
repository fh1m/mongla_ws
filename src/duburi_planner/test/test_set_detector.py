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
