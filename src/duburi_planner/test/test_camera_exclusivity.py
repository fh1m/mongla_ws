"""use_camera exclusivity: exactly one detector infers after a switch.

_activate_camera must pause EVERY other known detector (not just the previously
live one) so a stray `paused:=false` launch (both inferring from t=0) is
corrected on the first switch -- the concurrent-inference OOM fix. No ROS: the
real method runs against a fake self with mocked detector-param calls.
"""

from unittest.mock import MagicMock, patch

from duburi_planner.duburi_dsl import DuburiMission


def _fake(live=None, present=True):
    m = MagicMock()
    m._live_camera = live
    m._KNOWN_CAMERAS = DuburiMission._KNOWN_CAMERAS
    m._CAM_SWITCH_SETTLE_S = 0.0
    m.cam_switch_settle_s = 0.0
    # real _detector_present is a fast graph check; default present in tests.
    m._detector_present.return_value = present
    return m


def _activate(fake, name):
    with patch('duburi_planner.duburi_dsl._time.sleep'):
        DuburiMission._activate_camera(fake, name)


def test_first_switch_pauses_the_other_camera_even_with_no_prev():
    # live=None (fresh launch): switching to forward must STILL pause downward,
    # so a paused:=false launch can't leave both detectors inferring.
    fake = _fake(live=None)
    _activate(fake, 'forward')
    fake.pause_detector.assert_called_once_with('downward')
    fake.resume_detector.assert_called_once_with('forward')


def test_switch_never_pauses_the_target():
    fake = _fake(live='forward')
    _activate(fake, 'downward')
    paused = [c.args[0] for c in fake.pause_detector.call_args_list]
    assert 'downward' not in paused         # never pause the one we're resuming
    assert 'forward' in paused              # pause the other
    fake.resume_detector.assert_called_once_with('downward')


def test_noop_when_already_live():
    fake = _fake(live='forward')
    _activate(fake, 'forward')
    fake.pause_detector.assert_not_called()
    fake.resume_detector.assert_not_called()


def test_absent_detector_pause_is_swallowed():
    # single-camera run: pausing the missing counterpart must not raise.
    fake = _fake(live=None)
    fake.pause_detector.side_effect = RuntimeError('no downward node')
    _activate(fake, 'forward')              # must not raise
    fake.resume_detector.assert_called_once_with('forward')


def test_absent_counterpart_is_skipped_without_pause_call():
    # _detector_present False -> the other is never pause_detector'd, so a
    # single-camera run never enters _ensure_detector's 5 s wait_for_service.
    fake = _fake(live=None, present=False)
    _activate(fake, 'forward')
    fake.pause_detector.assert_not_called()
    fake.resume_detector.assert_called_once_with('forward')


# --- a QUERY is the first perception call, and it must resume something ------
#
# `detected()` / `wait_for()` / `where()` all funnel through _pump_detections.
# None of them reached _activate_camera, so against a launch that starts both
# detectors paused they polled a detector that would never infer: empty cache,
# forever, no error. That is why `vision_pi.launch.py` shipped `paused:=false`
# against its own description and paid ~60 Hz in chip contention instead.


def _pump_fake(live=None):
    m = MagicMock()
    m._live_camera = live
    m._det_warm = set()
    m._det_cache = {}
    m._PUMP_WARM_S = 0.0
    m._PUMP_COLD_S = 0.0          # deadline already passed -> no spin loop
    m._PUMP_SLICE_S = 0.0
    return m


def test_the_first_query_resumes_the_camera_it_reads():
    fake = _pump_fake(live=None)
    DuburiMission._pump_detections(fake, 'forward')
    fake._activate_camera.assert_called_once_with('forward')


def test_a_later_query_on_the_other_camera_does_not_switch():
    # A query is not a statement about which camera the mission steers on.
    # Switching here would cost the 1.5 s settle on EVERY iteration of a
    # mission that polls both cameras. `use_camera` is the way to switch.
    fake = _pump_fake(live='forward')
    DuburiMission._pump_detections(fake, 'downward')
    fake._activate_camera.assert_not_called()


def test_a_repeat_query_on_the_live_camera_does_not_re_activate():
    fake = _pump_fake(live='forward')
    DuburiMission._pump_detections(fake, 'forward')
    fake._activate_camera.assert_not_called()
