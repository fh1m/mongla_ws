"""Orchestration tests for the two-verb vision DSL (align / move).

These pin the recover-don't-fail contract:
  * neither verb raises on a miss -- TIMEOUT / LOST return VisionResult(ok=False);
  * ALIGNED returns ok=True;
  * on LOST with a fallback, the mission-authored fallback runs once and the
    loop re-enters vision within the same duration budget;
  * a 2-arg fallback receives a `should_stop` predicate.

The server action is mocked via a fake `_send`, so no ROS / MAVLink is needed.
"""

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from duburi_planner.vision_dsl import _AnchorDSL, _VisionDSL, VisionResult
from duburi_planner.client import MoveFailed
from duburi_control.motion_vision import ALIGNED, LOST, TIMEOUT, NO_CAMERA


def _result(code, err=0.0, *, fill=0.0, x=math.nan, y=math.nan, elapsed=0.0):
    return SimpleNamespace(final_value=float(code), error_value=float(err),
                           fill_frac=float(fill), end_x_px=float(x),
                           end_y_px=float(y), elapsed_s=float(elapsed),
                           success=True, message=str(code))


def _mission(send):
    """Fake DuburiMission exposing only what _VisionDSL touches."""
    m = MagicMock()
    m.camera = 'forward'
    m.target = 'gate'
    m._send = send
    m.detected.return_value = False
    m.log = MagicMock()
    return m


def _dsl(send):
    return _VisionDSL(_mission(send))


# --------------------------------------------------------------------------- #
#  Loud-abort contract: a missing detector must RAISE out of the verb, NOT be  #
#  swallowed by the never-raise wrapper into a silent FAILED (the pool bug).   #
# --------------------------------------------------------------------------- #
def test_align_propagates_detector_preflight_abort():
    m = _mission(MagicMock(return_value=_result(ALIGNED)))
    m._ensure_detector.side_effect = RuntimeError('Detector node ... NOT FOUND')
    m._detector_node.return_value = '/duburi_detector_forward'
    with pytest.raises(RuntimeError, match='NOT FOUND'):
        _VisionDSL(m).align('gate', yaw=0, lat=0)


def test_move_propagates_detector_preflight_abort():
    m = _mission(MagicMock(return_value=_result(ALIGNED, 0.9)))
    m._ensure_detector.side_effect = RuntimeError('Detector node ... NOT FOUND')
    m._detector_node.return_value = '/duburi_detector_forward'
    with pytest.raises(RuntimeError, match='NOT FOUND'):
        _VisionDSL(m).move('gate', fwd=80)


# --------------------------------------------------------------------------- #
#  align                                                                       #
# --------------------------------------------------------------------------- #
def test_align_ok_on_aligned():
    dsl = _dsl(MagicMock(return_value=_result(ALIGNED, 8.0)))
    res = dsl.align('gate', yaw=0, lat=0)
    assert isinstance(res, VisionResult)
    assert res.ok is True
    assert res.reason == 'ALIGNED'


def test_align_never_raises_on_timeout():
    dsl = _dsl(MagicMock(return_value=_result(TIMEOUT, 120.0)))
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert res.ok is False
    assert res.reason == 'TIMEOUT'


def test_align_no_camera_returns_false():
    dsl = _dsl(MagicMock(return_value=_result(NO_CAMERA)))
    res = dsl.align('gate', yaw=0)
    assert res.ok is False
    assert res.reason == 'NO_CAMERA'


def test_align_requires_an_axis():
    dsl = _dsl(MagicMock(return_value=_result(ALIGNED)))
    with pytest.raises(ValueError):
        dsl.align('gate')          # no lat/yaw/depth -> programming error


def test_align_runs_fallback_once_then_reenters():
    # First server call reports LOST, fallback runs, second call ALIGNED.
    send = MagicMock(side_effect=[_result(LOST, 50.0), _result(ALIGNED, 5.0)])
    dsl = _dsl(send)
    calls = []

    def creep(_duburi):
        calls.append('creep')

    res = dsl.align('gate', yaw=0, lat=0, fallback=creep, duration=10)
    assert res.ok is True
    assert calls == ['creep']       # fallback ran exactly once
    assert send.call_count == 2     # re-entered the vision loop after fallback


def test_align_lost_without_fallback_returns_false():
    dsl = _dsl(MagicMock(return_value=_result(LOST, 50.0)))
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert res.ok is False
    assert res.reason == 'LOST'


def test_align_never_dies_on_server_failure():
    # A server-side failure (bad camera, ALT_HOLD rejected, disarmed, abort)
    # raises MoveFailed inside _send. The verb MUST catch it and return a
    # non-fatal VisionResult so the mission moves on -- never propagate.
    dsl = _dsl(MagicMock(side_effect=MoveFailed(
        'Goal "vision_align" FAILED: ALT_HOLD rejected')))
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert isinstance(res, VisionResult)
    assert res.ok is False
    assert res.reason == 'FAILED'


def test_align_never_dies_on_unexpected_exception():
    # Even a totally unexpected error must not escape a vision verb.
    dsl = _dsl(MagicMock(side_effect=RuntimeError('rclpy hiccup')))
    res = dsl.align('gate', yaw=0, duration=0.5)
    assert res.ok is False
    assert res.reason == 'FAILED'


def test_align_two_arg_fallback_gets_should_stop():
    send = MagicMock(side_effect=[_result(LOST), _result(ALIGNED)])
    dsl = _dsl(send)
    seen = {}

    def sweep(_duburi, should_stop):
        seen['callable'] = callable(should_stop)

    dsl.align('gate', yaw=0, fallback=sweep, duration=10)
    assert seen.get('callable') is True


def test_align_fallback_exception_does_not_propagate():
    send = MagicMock(side_effect=[_result(LOST), _result(ALIGNED)])
    dsl = _dsl(send)

    def bad(_duburi):
        raise RuntimeError('search blew up')

    res = dsl.align('gate', yaw=0, fallback=bad, duration=10)
    assert res.ok is True           # bad fallback swallowed, loop re-entered


# --------------------------------------------------------------------------- #
#  move                                                                        #
# --------------------------------------------------------------------------- #
def test_move_ok_on_reach():
    dsl = _dsl(MagicMock(return_value=_result(ALIGNED, fill=0.85)))
    res = dsl.move('gate', fwd=80)
    assert res.ok is True
    assert res.fill == pytest.approx(0.85)   # fill now from fill_frac, not error_value


def test_move_never_raises_on_timeout():
    dsl = _dsl(MagicMock(return_value=_result(TIMEOUT, 0.4)))
    res = dsl.move('gate', fwd=80, duration=0.5)
    assert res.ok is False


def test_move_never_dies_on_server_failure():
    dsl = _dsl(MagicMock(side_effect=MoveFailed('boom')))
    res = dsl.move('gate', fwd=80, duration=0.5)
    assert res.ok is False
    assert res.reason == 'FAILED'


# --------------------------------------------------------------------------- #
#  Rich end-state: where/how the verb finished (hybrid vision+control)         #
# --------------------------------------------------------------------------- #
def test_result_carries_end_position_on_timeout():
    # TIMEOUT but the target WAS seen ending 42px left, 8px below centre:
    # the mission can branch on x_px even though the verb failed.
    dsl = _dsl(MagicMock(return_value=_result(TIMEOUT, 42.0, x=-42.0, y=8.0)))
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert res.ok is False and res.status == 'TIMEOUT'
    assert res.x_px == pytest.approx(-42.0)
    assert res.y_px == pytest.approx(8.0)
    assert res.saw_target is True          # x_px present -> was seen


def test_result_saw_target_false_when_never_seen():
    # NaN end-position (never detected) -> saw_target False, so a mission tells
    # "ended off-centre" from "never saw it".
    dsl = _dsl(MagicMock(return_value=_result(LOST, 0.0)))   # x/y default NaN
    res = dsl.align('gate', yaw=0, lat=0, duration=0.5)
    assert res.ok is False
    assert res.saw_target is False
    assert math.isnan(res.x_px)


def test_result_aligned_exposes_end_position_too():
    # End-state is populated on SUCCESS as well (useful for the next task).
    dsl = _dsl(MagicMock(return_value=_result(ALIGNED, 5.0, x=2.0, y=-1.0,
                                              elapsed=3.2)))
    res = dsl.align('gate', yaw=0, lat=0)
    assert res.ok is True
    assert res.x_px == pytest.approx(2.0) and res.saw_target is True
    assert res.elapsed_s == pytest.approx(3.2)


def test_result_bool_still_works_for_back_compat():
    # The truthiness contract is unchanged: `if duburi.vision.align(...)`.
    assert bool(_dsl(MagicMock(return_value=_result(ALIGNED, x=0.0))).align(
        'gate', yaw=0)) is True
    assert bool(_dsl(MagicMock(return_value=_result(TIMEOUT))).align(
        'gate', yaw=0, duration=0.3)) is False


# --------------------------------------------------------------------------- #
#  detected() case-insensitive (control path lowercases both sides)           #
# --------------------------------------------------------------------------- #
def _rec(cls, cx=320.0, cy=240.0, w=40.0, h=40.0, conf=0.9):
    """Build a DetRecord tuple: (class_lower, cx, cy, w, h, conf)."""
    return (cls, cx, cy, w, h, conf)


def _fake_msg(*classes, cx=320.0, cy=240.0, w=40.0, h=40.0, conf=0.9):
    """Minimal Detection2DArray duck-type for driving _on_detections (no ROS).

    Humble-flat Pose2D (center has .x/.y, no .position) so _det_center takes the
    flat branch; one ObjectHypothesisWithPose per class with .hypothesis.class_id.
    """
    dets = []
    for c in classes:
        hyp    = SimpleNamespace(hypothesis=SimpleNamespace(class_id=c, score=conf))
        center = SimpleNamespace(x=cx, y=cy)
        bbox   = SimpleNamespace(center=center, size_x=w, size_y=h)
        dets.append(SimpleNamespace(results=[hyp], bbox=bbox))
    return SimpleNamespace(detections=dets)


def _ready_mission(monkeypatch, dd, camera='forward'):
    """A DuburiMission with the pump stubbed + subscription faked (no ROS)."""
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera=camera)
    monkeypatch.setattr(m, '_pump_detections', lambda *a, **k: None)
    m._det_subs[camera] = object()   # _subscribe_detections() early-returns
    return m


def test_detected_is_case_insensitive(monkeypatch):
    import duburi_planner.duburi_dsl as dd
    m = _ready_mission(monkeypatch, dd)
    # Drive the real callback so _det_seen is populated the way ROS would.
    m._on_detections('forward', _fake_msg('gate'))

    assert m.detected('gate') is True      # case-insensitive match
    assert m.detected('GATE') is True
    assert m.detected('flare') is False


def test_detected_survives_single_frame_flicker(monkeypatch):
    # THE core reacquire fix: a class that drops from one raw frame still counts
    # as present within the recency window, so `while not detected()` reacquires
    # the instant it reappears instead of chasing per-frame dropouts.
    import duburi_planner.duburi_dsl as dd
    clock = {'t': 100.0}
    monkeypatch.setattr(dd._time, 'monotonic', lambda: clock['t'])
    m = _ready_mission(monkeypatch, dd)

    m._on_detections('forward', _fake_msg('red_pipe'))   # seen at t=100.0
    assert m.detected('red_pipe') is True

    clock['t'] = 100.3                                    # 0.3 s later: flicker
    m._on_detections('forward', _fake_msg('flare'))      # red_pipe missing this frame
    # Latest frame lacks red_pipe, but it was seen 0.3 s ago < 1.0 s window:
    assert m.detected('red_pipe', stale_after=1.0) is True


def test_detected_false_after_window_expires(monkeypatch):
    import duburi_planner.duburi_dsl as dd
    clock = {'t': 50.0}
    monkeypatch.setattr(dd._time, 'monotonic', lambda: clock['t'])
    m = _ready_mission(monkeypatch, dd)

    m._on_detections('forward', _fake_msg('red_pipe'))   # seen at t=50.0
    clock['t'] = 51.5                                     # 1.5 s later, no new sighting
    assert m.detected('red_pipe', stale_after=1.0) is False


def test_where_reflects_latest_frame_not_seen_window(monkeypatch):
    # where()/where_offset() must read the LATEST frame, never the last-seen
    # memory -- bearing has to be the target's current position, not a stale one.
    import duburi_planner.duburi_dsl as dd
    clock = {'t': 0.0}
    monkeypatch.setattr(dd._time, 'monotonic', lambda: clock['t'])
    m = _ready_mission(monkeypatch, dd)
    m._img_size['forward'] = (640.0, 480.0)

    m._on_detections('forward', _fake_msg('red_pipe', cx=64.0))  # left, t=0
    assert m.where('red_pipe') == 'left'

    clock['t'] = 0.3
    m._on_detections('forward', _fake_msg('flare'))     # latest frame: no red_pipe
    assert m.detected('red_pipe') is True               # still within window
    assert m.where('red_pipe') == 'unknown'             # but bearing is from latest frame


# --------------------------------------------------------------------------- #
#  Pure cores: _eval_detected / _eval_where (no ROS)                          #
# --------------------------------------------------------------------------- #
def test_eval_detected_present_absent_caseinsensitive():
    import duburi_planner.duburi_dsl as dd
    recs = [_rec('gate'), _rec('flare')]
    assert dd._eval_detected(recs, 'gate') is True
    assert dd._eval_detected(recs, 'GATE') is True       # case-insensitive
    assert dd._eval_detected(recs, ' Flare ') is True    # trimmed
    assert dd._eval_detected(recs, 'hole') is False
    assert dd._eval_detected([], 'gate') is False


def test_eval_where_left_center_right():
    import duburi_planner.duburi_dsl as dd
    W = 640.0
    # cx=64 -> offset -0.8 -> left; cx=320 -> 0 -> center; cx=576 -> +0.8 -> right
    assert dd._eval_where([_rec('gate', cx=64.0)],  'gate', W, 0.15)[0] == 'left'
    assert dd._eval_where([_rec('gate', cx=320.0)], 'gate', W, 0.15)[0] == 'center'
    assert dd._eval_where([_rec('gate', cx=576.0)], 'gate', W, 0.15)[0] == 'right'


def test_eval_where_largest_area_wins():
    import duburi_planner.duburi_dsl as dd
    # Two gates: a tiny one on the left, a big one on the right -> right wins.
    recs = [_rec('gate', cx=50.0,  w=10.0, h=10.0),
            _rec('gate', cx=600.0, w=200.0, h=200.0)]
    label, offset = dd._eval_where(recs, 'gate', 640.0, 0.15)
    assert label == 'right'
    assert offset > 0.0


def test_eval_where_unknown_when_no_width_or_absent():
    import duburi_planner.duburi_dsl as dd
    assert dd._eval_where([_rec('gate')], 'gate', 0.0, 0.15) == ('unknown', None)
    assert dd._eval_where([], 'gate', 640.0, 0.15) == ('unknown', None)
    assert dd._eval_where([_rec('flare')], 'gate', 640.0, 0.15) == ('unknown', None)


def test_wait_for_returns_true_when_target_appears(monkeypatch):
    import duburi_planner.duburi_dsl as dd
    m = _ready_mission(monkeypatch, dd)
    # Target absent for the first 2 polls; on the 3rd the pump lands a frame.
    state = {'n': 0}
    def fake_pump(cam):
        state['n'] += 1
        if state['n'] >= 3:
            m._on_detections(cam, _fake_msg('gate'))
    monkeypatch.setattr(m, '_pump_detections', fake_pump)
    assert m.wait_for('gate', timeout=5.0) is True


def test_wait_for_returns_false_on_timeout(monkeypatch):
    import duburi_planner.duburi_dsl as dd
    m = _ready_mission(monkeypatch, dd)   # pump stubbed; no frame ever lands
    assert m.wait_for('gate', timeout=0.2) is False


def test_where_returns_label_from_cache(monkeypatch):
    import time as _t
    import duburi_planner.duburi_dsl as dd
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    monkeypatch.setattr(m, '_pump_detections', lambda *a, **k: None)
    m._img_size['forward'] = (640.0, 480.0)
    m._det_cache['forward'] = (_t.monotonic(), [_rec('gate', cx=64.0)])
    assert m.where('gate') == 'left'
    assert m.where_offset('gate') < 0.0
    assert m.where('hole') == 'unknown'


def test_detector_node_derivation():
    # Single naming rule: /duburi_detector_<camera>, camera defaults to the
    # mission's sticky camera; explicit node always wins.
    import duburi_planner.duburi_dsl as dd
    m = dd.DuburiMission(MagicMock(), MagicMock(), camera='forward')
    assert m._detector_node() == '/duburi_detector_forward'
    assert m._detector_node('downward') == '/duburi_detector_downward'
    assert m._detector_node(node='/duburi_detector_custom') == '/duburi_detector_custom'


def test_move_passes_fill_and_mode_to_server():
    send = MagicMock(return_value=_result(ALIGNED, 0.9))
    dsl = _dsl(send)
    dsl.move('red_pipe', fwd=60, mode='height')
    _, kwargs = send.call_args
    assert kwargs['fwd_fill'] == pytest.approx(60.0)
    assert kwargs['mode'] == 'height'


# --------------------------------------------------------------------------- #
#  align fire / fire_t -> goal fields (mid-hold payload fire)                  #
# --------------------------------------------------------------------------- #
def test_align_fire_list_sends_csv_channels_and_fire_t():
    send = MagicMock(return_value=_result(ALIGNED))
    _dsl(send).align('hole', yaw=0, lat=0, hold=4, fire=[1, 2], fire_t=1)
    _, kwargs = send.call_args
    assert kwargs['fire_channels'] == '1,2'
    assert kwargs['fire_t'] == pytest.approx(1.0)


def test_align_fire_scalar_sends_single_channel():
    send = MagicMock(return_value=_result(ALIGNED))
    _dsl(send).align('hole', yaw=0, hold=3, fire=1)
    _, kwargs = send.call_args
    assert kwargs['fire_channels'] == '1'


def test_align_no_fire_defaults_to_empty():
    send = MagicMock(return_value=_result(ALIGNED))
    _dsl(send).align('gate', yaw=0, lat=0)
    _, kwargs = send.call_args
    assert kwargs['fire_channels'] == ''
    assert kwargs['fire_t'] == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
#  align lock_on -> lock_target goal field (continuity lock)                   #
# --------------------------------------------------------------------------- #
def test_align_lock_on_sets_lock_target():
    send = MagicMock(return_value=_result(ALIGNED))
    _dsl(send).align('hole', lat=0, depth=0, lock_on=True)
    _, kwargs = send.call_args
    assert kwargs['lock_target'] is True


def test_align_lock_off_by_default():
    send = MagicMock(return_value=_result(ALIGNED))
    _dsl(send).align('gate', yaw=0, lat=0)
    _, kwargs = send.call_args
    assert kwargs['lock_target'] is False


# --------------------------------------------------------------------------- #
#  In-process detector param control + loud preflight                          #
# --------------------------------------------------------------------------- #
def test_param_value_types():
    # bool is checked before int (bool subclasses int) so a paused flag never
    # gets coerced to an integer parameter.
    import duburi_planner.duburi_dsl as dd
    assert dd._param_value(True).type == dd.ParameterType.PARAMETER_BOOL
    assert dd._param_value(True).bool_value is True
    assert dd._param_value(0.6).type == dd.ParameterType.PARAMETER_DOUBLE
    assert dd._param_value(0.6).double_value == pytest.approx(0.6)
    assert dd._param_value(3).type == dd.ParameterType.PARAMETER_INTEGER
    assert dd._param_value('gate').type == dd.ParameterType.PARAMETER_STRING
    assert dd._param_value('gate').string_value == 'gate'


def test_ensure_detector_aborts_loudly_when_node_absent():
    # The whole point of the fix: a missing detector node is a LOUD abort
    # (RuntimeError) with an actionable launch hint, not a swallowed warning.
    import duburi_planner.duburi_dsl as dd
    client = MagicMock()
    client.node.create_client.return_value.wait_for_service.return_value = False
    m = dd.DuburiMission(client, MagicMock(), camera='forward')
    with pytest.raises(RuntimeError, match='NOT FOUND'):
        m._ensure_detector('/duburi_detector_forward', timeout=0.01)


def test_set_detector_param_sends_typed_param(monkeypatch):
    import duburi_planner.duburi_dsl as dd
    client = MagicMock()
    m = dd.DuburiMission(client, MagicMock(), camera='forward')
    monkeypatch.setattr(m, '_ensure_detector', lambda *a, **k: None)
    cli = MagicMock()
    cli.wait_for_service.return_value = True
    cli.call_async.return_value.result.return_value = SimpleNamespace(
        results=[SimpleNamespace(successful=True, reason='')])
    client.node.create_client.return_value = cli
    monkeypatch.setattr(dd.rclpy, 'spin_until_future_complete', lambda *a, **k: None)

    m._set_detector_param('/duburi_detector_forward', 'conf', 0.6)

    req = cli.call_async.call_args[0][0]
    p = req.parameters[0]
    assert p.name == 'conf'
    assert p.value.type == dd.ParameterType.PARAMETER_DOUBLE
    assert p.value.double_value == pytest.approx(0.6)


def test_set_detector_param_raises_on_rejection(monkeypatch):
    import duburi_planner.duburi_dsl as dd
    client = MagicMock()
    m = dd.DuburiMission(client, MagicMock(), camera='forward')
    monkeypatch.setattr(m, '_ensure_detector', lambda *a, **k: None)
    cli = MagicMock()
    cli.wait_for_service.return_value = True
    cli.call_async.return_value.result.return_value = SimpleNamespace(
        results=[SimpleNamespace(successful=False, reason='not in registry')])
    client.node.create_client.return_value = cli
    monkeypatch.setattr(dd.rclpy, 'spin_until_future_complete', lambda *a, **k: None)

    with pytest.raises(RuntimeError, match='registry'):
        m.set_model('bogus_model')


def test_move_passthrough_sends_negative_sentinel():
    # move('gate') with no fwd = pass-through. The DSL must wire fwd_fill=-1
    # (a value that survives the manager's 0.0==unset rule) so the control
    # loop selects pass-through instead of the 95% spec default.
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.move('gate')
    _, kwargs = send.call_args
    assert kwargs['fwd_fill'] < 0.0


def test_move_explicit_fill_is_not_sentinel():
    send = MagicMock(return_value=_result(ALIGNED, 0.8))
    dsl = _dsl(send)
    dsl.move('gate', fwd=80)
    _, kwargs = send.call_args
    assert kwargs['fwd_fill'] == pytest.approx(80.0)


# --------------------------------------------------------------------------- #
#  align -- 0-offset centring (the pool-day "yaw=0, lat=0" case)              #
# --------------------------------------------------------------------------- #
def test_align_zero_offsets_build_axes_and_center():
    # align('gate', yaw=0, lat=0) must activate both axes (0 = centre) and
    # send offset_*=0.0 -- never silently drop a zero-valued axis.
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    res = dsl.align('gate', yaw=0, lat=0)
    assert res.ok is True
    _, kwargs = send.call_args
    assert set(kwargs['axes'].split(',')) == {'lat', 'yaw'}
    assert kwargs['offset_lat'] == pytest.approx(0.0)
    assert kwargs['offset_yaw'] == pytest.approx(0.0)


def test_align_single_zero_axis_only_that_axis():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('gate', yaw=0)
    _, kwargs = send.call_args
    assert kwargs['axes'] == 'yaw'
    assert kwargs['offset_yaw'] == pytest.approx(0.0)


def test_align_nonzero_offset_passed_through():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('gate', lat=64, yaw=0)
    _, kwargs = send.call_args
    assert kwargs['offset_lat'] == pytest.approx(64.0)
    assert kwargs['offset_yaw'] == pytest.approx(0.0)


def test_align_hold_sent_when_given():
    # align(hold=2.5) must wire hold_s=2.5 so the server station-keeps.
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('hole', yaw=0, lat=0, hold=2.5)
    _, kwargs = send.call_args
    assert kwargs['hold_s'] == pytest.approx(2.5)


def test_align_hold_defaults_to_zero():
    # No hold -> hold_s=0.0 (exit-on-stable, unchanged behaviour).
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('gate', yaw=0, lat=0)
    _, kwargs = send.call_args
    assert kwargs['hold_s'] == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
#  anchor -- XFeat superglue                                                   #
# --------------------------------------------------------------------------- #
def test_anchor_snap_dispatches_verb():
    send = MagicMock(return_value=_result(1))
    dsl = _dsl(send)
    assert dsl.anchor_snap() is True
    cmd, kwargs = send.call_args[0][0], send.call_args[1]
    assert cmd == 'vision_anchor_snap'
    assert kwargs['camera'] == 'forward'


def test_anchor_clear_dispatches_verb():
    send = MagicMock(return_value=_result(1))
    dsl = _dsl(send)
    assert dsl.anchor_clear() is True
    assert send.call_args[0][0] == 'vision_anchor_clear'


def test_anchor_align_sends_fire_channels_csv():
    send = MagicMock(return_value=_result(ALIGNED, 8.0))
    dsl = _dsl(send)
    res = dsl.anchor_align(err=15, theta=0.04, hold=2.0, fire=[1, 2], match=20)
    assert res.ok is True
    cmd, kwargs = send.call_args[0][0], send.call_args[1]
    assert cmd == 'vision_anchor_align'
    assert kwargs['fire_channels'] == '1,2'
    assert kwargs['hold_s'] == pytest.approx(2.0)
    assert kwargs['min_inliers'] == pytest.approx(20.0)
    assert kwargs['theta_thresh'] == pytest.approx(0.04)


def test_anchor_align_no_fire_is_empty():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.anchor_align()
    _, kwargs = send.call_args
    assert kwargs['fire_channels'] == ''
    assert kwargs['min_inliers'] == pytest.approx(0.0)


def test_anchor_align_single_fire_int():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.anchor_align(fire=1)
    _, kwargs = send.call_args
    assert kwargs['fire_channels'] == '1'


def test_anchor_snap_with_name_sends_ref_name():
    send = MagicMock(return_value=_result(1))
    dsl = _dsl(send)
    assert dsl.anchor_snap('hole') is True
    _, kwargs = send.call_args
    assert kwargs['ref_name'] == 'hole'


def test_anchor_snap_no_name_is_empty_ref():
    send = MagicMock(return_value=_result(1))
    dsl = _dsl(send)
    dsl.anchor_snap()
    _, kwargs = send.call_args
    assert kwargs['ref_name'] == ''
    assert kwargs['target_class'] == ''      # whole-frame snap


def test_anchor_snap_target_sends_detection_gate():
    send = MagicMock(return_value=_result(1))
    dsl = _dsl(send)
    dsl.anchor_snap(target='hole', conf=0.6, err=30)
    _, kwargs = send.call_args
    assert kwargs['target_class'] == 'hole'
    assert kwargs['conf'] == pytest.approx(0.6)
    assert kwargs['err_px'] == pytest.approx(30.0)


def test_anchor_align_positional_name_loads_disk_ref():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.anchor_align('hole', hold=2.0)
    _, kwargs = send.call_args
    assert kwargs['ref_name'] == 'hole'
    assert kwargs['hold_s'] == pytest.approx(2.0)


# --------------------------------------------------------------------------- #
#  duburi.anchor.* -- redesigned XFeat feature-lock namespace                  #
#  (delegates to the proven vision.anchor_* verbs; these pin the routing)      #
# --------------------------------------------------------------------------- #
def _anchor_dsl(send):
    """An _AnchorDSL whose delegate `.vision` is a real _VisionDSL over `send`."""
    m = _mission(send)
    m.vision = _VisionDSL(m)
    return _AnchorDSL(m)


def test_anchor_ns_snap_frame_is_whole_frame():
    send = MagicMock(return_value=_result(1))
    a = _anchor_dsl(send)
    assert a.snap(source='frame') is True
    cmd, kwargs = send.call_args[0][0], send.call_args[1]
    assert cmd == 'vision_anchor_snap'
    assert kwargs['target_class'] == ''      # whole-frame: no detection gate
    assert kwargs['ref_name'] == ''          # not persisted


def test_anchor_ns_snap_detection_sends_gate():
    send = MagicMock(return_value=_result(1))
    a = _anchor_dsl(send)
    a.snap(source='detection', target='hole', conf=0.6, err=30)
    _, kwargs = send.call_args
    assert kwargs['target_class'] == 'hole'
    assert kwargs['conf'] == pytest.approx(0.6)
    assert kwargs['err_px'] == pytest.approx(30.0)


def test_anchor_ns_snap_detection_defaults_to_sticky_target():
    send = MagicMock(return_value=_result(1))
    a = _anchor_dsl(send)                     # _mission sets target='gate'
    a.snap(source='detection')
    _, kwargs = send.call_args
    assert kwargs['target_class'] == 'gate'


def test_anchor_ns_save_persists_ref_name():
    send = MagicMock(return_value=_result(1))
    a = _anchor_dsl(send)
    assert a.save('gate', source='detection', target='gate') is True
    _, kwargs = send.call_args
    assert kwargs['ref_name'] == 'gate'       # persisted to references/gate.png
    assert kwargs['target_class'] == 'gate'


def test_anchor_ns_snap_save_kw_persists():
    send = MagicMock(return_value=_result(1))
    a = _anchor_dsl(send)
    a.snap(source='frame', save='board')
    _, kwargs = send.call_args
    assert kwargs['ref_name'] == 'board'


def test_anchor_ns_snap_bad_source_raises():
    a = _anchor_dsl(MagicMock(return_value=_result(1)))
    with pytest.raises(ValueError):
        a.snap(source='disk')                 # only detection|frame


def test_anchor_ns_align_ref_loads_and_locks():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    a = _anchor_dsl(send)
    res = a.align('hole', hold=2.0, fire=[1, 2], match=20)
    assert res.ok is True
    cmd, kwargs = send.call_args[0][0], send.call_args[1]
    assert cmd == 'vision_anchor_align'
    assert kwargs['ref_name'] == 'hole'
    assert kwargs['fire_channels'] == '1,2'
    assert kwargs['hold_s'] == pytest.approx(2.0)
    assert kwargs['min_inliers'] == pytest.approx(20.0)


def test_anchor_ns_align_no_ref_locks_last_snap():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    a = _anchor_dsl(send)
    a.align()
    _, kwargs = send.call_args
    assert kwargs['ref_name'] == ''


def test_anchor_ns_clear_dispatches():
    send = MagicMock(return_value=_result(1))
    a = _anchor_dsl(send)
    assert a.clear() is True
    assert send.call_args[0][0] == 'vision_anchor_clear'


# --------------------------------------------------------------------------- #
#  use_feature -- XFeat anchor detection-fallback fusion threads to the goal   #
# --------------------------------------------------------------------------- #
def test_align_use_feature_threads_to_goal():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('hole', lat=0, depth=0, use_feature=True)
    _, kwargs = send.call_args
    assert kwargs['use_feature'] is True


def test_align_use_feature_defaults_false():
    send = MagicMock(return_value=_result(ALIGNED, 0.0))
    dsl = _dsl(send)
    dsl.align('hole', lat=0)
    _, kwargs = send.call_args
    assert kwargs['use_feature'] is False
