"""Downward/depth vision tunables (moved off per-align kwargs).

surge_sign / max_depth_m / depth_ceiling are now vision.* deck params that the
manager substitutes into a vision_align goal when the mission omits them. This
pins the param defaults + the field->param mapping; no ROS node needed.
"""

from duburi_manager.vision_tunables import (
    VISION_PARAM_DEFAULTS,
    runtime_defaults_for_command,
)


def test_new_params_have_expected_defaults():
    # surge_sign is PERMANENT -1 (downward mount); depth bounds default OFF (0.0)
    # so the FORWARD torpedo align is unchanged.
    assert VISION_PARAM_DEFAULTS['vision.surge_sign'] == -1.0
    assert VISION_PARAM_DEFAULTS['vision.max_depth_m'] == 0.0
    assert VISION_PARAM_DEFAULTS['vision.depth_ceiling'] == 0.0


def test_vision_align_runtime_defaults_map_the_three_fields():
    # Snapshot as the manager reads it (param name -> value).
    snapshot = dict(VISION_PARAM_DEFAULTS)
    out = runtime_defaults_for_command('vision_align', snapshot)
    assert out['surge_sign'] == -1.0
    assert out['max_depth_m'] == 0.0
    assert out['depth_ceiling_m'] == 0.0   # note: field is depth_ceiling_m, param vision.depth_ceiling


def test_non_vision_command_gets_empty_defaults():
    assert runtime_defaults_for_command('move_forward', dict(VISION_PARAM_DEFAULTS)) == {}
