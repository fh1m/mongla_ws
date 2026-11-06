"""mongla.set_vision_param -- per-mission control of a manager vision.* tunable.

Verifies the ergonomics (prefix handling + float coercion) without a live
manager: the real method runs against a MagicMock self, and we assert what it
forwards to _set_manager_param.
"""

from unittest.mock import MagicMock

from mongla_planner.mongla_dsl import MonglaMission


def test_adds_vision_prefix_when_omitted():
    fake = MagicMock()
    MonglaMission.set_vision_param(fake, 'lost_grace_s', 1.5)
    fake._set_manager_param.assert_called_once_with('vision.lost_grace_s', 1.5)


def test_keeps_existing_vision_prefix():
    fake = MagicMock()
    MonglaMission.set_vision_param(fake, 'vision.surge_sign', -1)
    fake._set_manager_param.assert_called_once_with('vision.surge_sign', -1.0)


def test_coerces_value_to_float():
    fake = MagicMock()
    MonglaMission.set_vision_param(fake, 'lost_grace_s', 2)
    (name, value), _ = fake._set_manager_param.call_args
    assert name == 'vision.lost_grace_s'
    assert isinstance(value, float) and value == 2.0
