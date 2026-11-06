"""Downward/depth vision tunables (moved off per-align kwargs).

surge_sign is a vision.* deck param that the manager substitutes into a
vision_align goal when the mission omits it. This pins the param default and
the field->param mapping; no ROS node needed.
"""

from mongla_manager.vision_tunables import (
    VISION_PARAM_DEFAULTS,
    runtime_defaults_for_command,
)


def test_new_params_have_expected_defaults():
    # surge_sign is PERMANENT -1 (downward mount).
    assert VISION_PARAM_DEFAULTS['vision.surge_sign'] == -1.0
    # The depth-setpoint bounds went with the depth-setpoint axis: srot refuses
    # it, so a bound would be a knob wired to nothing.
    assert 'vision.max_depth_m' not in VISION_PARAM_DEFAULTS
    assert 'vision.depth_ceiling' not in VISION_PARAM_DEFAULTS


def test_vision_align_runtime_defaults_map_surge_sign():
    # Snapshot as the manager reads it (param name -> value).
    snapshot = dict(VISION_PARAM_DEFAULTS)
    out = runtime_defaults_for_command('vision_align', snapshot)
    assert out['surge_sign'] == -1.0
    assert 'max_depth_m' not in out and 'depth_ceiling_m' not in out


def test_non_vision_command_gets_empty_defaults():
    assert runtime_defaults_for_command('move_forward', dict(VISION_PARAM_DEFAULTS)) == {}
