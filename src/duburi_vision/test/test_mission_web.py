"""Pure-helper tests for the mission-web console (no ROS spin).

Covers the four load-bearing pure functions the node delegates to:
parameter-type mapping (the SetParameters-convergence surface), the
camera-exclusivity target list, the pixel-offset math the DSL author reads,
and the SSE snapshot assembly.
"""

import pytest

from duburi_vision.web.dashboard_state import (
    param_value_for, active_camera_targets, bbox_metrics, build_snapshot,
    build_camera_view,
)


# ---- param_value_for: the type surface must match the detector's params ----
def test_conf_is_double_even_from_int():
    # Browser JSON `conf: 1` arrives as int; detector `conf` is a DOUBLE param.
    assert param_value_for('conf', 1) == ('double', 1.0)
    assert param_value_for('conf', 0.55) == ('double', 0.55)


def test_max_det_is_integer():
    assert param_value_for('max_det', 30) == ('integer', 30)
    assert param_value_for('max_det', 30.0) == ('integer', 30)


def test_paused_accepts_bool_and_strings():
    assert param_value_for('paused', True) == ('bool', True)
    assert param_value_for('paused', 'false') == ('bool', False)
    assert param_value_for('paused', '1') == ('bool', True)


def test_string_params():
    assert param_value_for('classes', 'gate,rescue') == ('string', 'gate,rescue')
    assert param_value_for('active_model', 'torpedo_blood_hole') == ('string', 'torpedo_blood_hole')
    assert param_value_for('model_conf', 'gate=0.5') == ('string', 'gate=0.5')


def test_unknown_param_infers_type_bool_before_int():
    # bool is a subclass of int -> must be checked first.
    assert param_value_for('whatever', True) == ('bool', True)
    assert param_value_for('whatever', 7) == ('integer', 7)
    assert param_value_for('whatever', 'x') == ('string', 'x')


def test_bad_numeric_values_raise_valueerror_not_typeerror():
    # A control write must never let float(None)/int({}) bubble up and kill the
    # handler thread -- param_value_for raises a clean ValueError the node maps
    # to ok:false. Covers conf=null, conf={...}, conf=[...], max_det="abc".
    for bad in (None, {'x': 1}, [1, 2]):
        with pytest.raises(ValueError):
            param_value_for('conf', bad)
        with pytest.raises(ValueError):
            param_value_for('max_det', bad)
    with pytest.raises(ValueError):
        param_value_for('conf', 'not-a-number')
    with pytest.raises(ValueError):
        param_value_for('max_det', 'not-an-int')


def test_string_param_accepts_anything_coercible():
    # classes/active_model are strings -> even a number stringifies, never raises.
    assert param_value_for('classes', 5) == ('string', '5')
    assert param_value_for('active_model', 'gate') == ('string', 'gate')


# ---- active_camera_targets: exclusivity ------------------------------------
def test_exclusivity_pauses_all_but_target():
    assert active_camera_targets('forward', ('forward', 'downward')) == ['downward']
    assert active_camera_targets('downward', ('forward', 'downward')) == ['forward']


def test_exclusivity_unknown_target_pauses_all_known():
    assert active_camera_targets('sideways', ('forward', 'downward')) == ['forward', 'downward']


# ---- bbox_metrics: dx/dy from FRAME centre + fill --------------------------
def test_bbox_metrics_offsets_and_fill():
    # 640x480 frame, box centred at (400,300), 64x48 -> dx=+80, dy=+60, fill=1%.
    dx, dy, fill = bbox_metrics(400, 300, 64, 48, (640, 480))
    assert dx == 80.0 and dy == 60.0
    assert round(fill, 3) == 1.0


def test_bbox_metrics_centre_is_zero_offset():
    dx, dy, _ = bbox_metrics(320, 240, 10, 10, (640, 480))
    assert dx == 0.0 and dy == 0.0


def test_bbox_metrics_unknown_frame_is_none():
    assert bbox_metrics(1, 2, 3, 4, None) == (None, None, None)
    assert bbox_metrics(1, 2, 3, 4, (0, 0)) == (None, None, None)


# ---- build_camera_view / build_snapshot ------------------------------------
def _cam(**over):
    base = {'present': True, 'paused': False, 'active_model': 'gate_rescue_repair',
            'models': ['gate_rescue_repair', 'slalom_red_pipe'], 'conf': 0.35,
            'classes': ['gate', 'rescue'], 'fps': 12.3, 'frame': (640, 480),
            'dets': [{'cls': 'gate', 'conf': 0.9, 'cx': 400, 'cy': 300, 'w': 64, 'h': 48,
                      'vis': 0.5, 'id': '7'},
                     {'cls': 'gate', 'conf': 0.4, 'cx': 320, 'cy': 240, 'w': 10, 'h': 10,
                      'vis': None, 'id': ''}]}
    base.update(over)
    return base


def test_camera_view_sorts_by_conf_and_counts():
    v = build_camera_view(_cam())
    # strongest first
    assert [d['conf'] for d in v['dets']] == [0.9, 0.4]
    assert v['dets'][0]['dx'] == 80.0 and v['dets'][0]['dy'] == 60.0
    # per-class counts: 2 gates, best 0.9
    assert v['counts']['gate'] == {'n': 2, 'best': 0.9}


def test_snapshot_shape_and_active_flag():
    store = {'cameras': {'forward': _cam(), 'downward': _cam(present=False)},
             'active_camera': 'forward',
             'state': {'armed': True, 'mode': 'ALT_HOLD', 'yaw': 12.0,
                       'depth': -1.2, 'batt': 15.6},
             'ts': 111.0}
    snap = build_snapshot(store, video_port=8080)
    assert snap['video_port'] == 8080
    assert snap['active_camera'] == 'forward'
    assert snap['cameras']['downward']['present'] is False
    assert snap['cameras']['forward']['fps'] == 12.3
    assert snap['state']['mode'] == 'ALT_HOLD'


def test_stale_class_has_no_count_entry():
    # 'repair' is an expected class but never detected -> absent from counts, so
    # the UI renders it as "(not seen)".
    v = build_camera_view(_cam(classes=['gate', 'repair']))
    assert 'repair' not in v['counts']
    assert 'gate' in v['counts']
