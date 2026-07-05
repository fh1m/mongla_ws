"""duburi.set_vision_param -- per-mission control of a manager vision.* tunable.

Verifies the ergonomics (prefix handling + float coercion) without a live
manager: the real method runs against a MagicMock self, and we assert what it
forwards to _set_manager_param.
"""

from unittest.mock import MagicMock

from duburi_planner.duburi_dsl import DuburiMission


def test_adds_vision_prefix_when_omitted():
    fake = MagicMock()
    DuburiMission.set_vision_param(fake, 'max_depth_m', -1.6)
    fake._set_manager_param.assert_called_once_with('vision.max_depth_m', -1.6)


def test_keeps_existing_vision_prefix():
    fake = MagicMock()
    DuburiMission.set_vision_param(fake, 'vision.surge_sign', -1)
    fake._set_manager_param.assert_called_once_with('vision.surge_sign', -1.0)


def test_coerces_value_to_float():
    fake = MagicMock()
    DuburiMission.set_vision_param(fake, 'depth_ceiling', -0.4)
    (name, value), _ = fake._set_manager_param.call_args
    assert name == 'vision.depth_ceiling'
    assert isinstance(value, float) and value == -0.4
