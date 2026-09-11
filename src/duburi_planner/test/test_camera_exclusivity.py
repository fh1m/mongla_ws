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
    m._resume_probed = set()
    m._det_cache = {}      # no frames yet -> the probe is the only evidence
    m._DISCOVERY_SETTLE_S = 0.0
    m._detector_present.return_value = True
    return m


def _resume(fake, camera):
    DuburiMission._resume_default_detector(fake, camera)


def test_the_first_query_resumes_the_camera_it_reads():
    fake = _pump_fake(live=None)
    _resume(fake, 'forward')
    fake._activate_camera.assert_called_once_with('forward')


def test_a_later_query_on_the_other_camera_does_not_switch():
    # A query is not a statement about which camera the mission steers on.
    # Switching here would cost the 1.5 s settle on EVERY iteration of a
    # mission that polls both cameras. `use_camera` is the way to switch.
    fake = _pump_fake(live='forward')
    _resume(fake, 'downward')
    fake._activate_camera.assert_not_called()


def test_a_repeat_query_on_the_live_camera_does_not_re_activate():
    fake = _pump_fake(live='forward')
    _resume(fake, 'forward')
    fake._activate_camera.assert_not_called()


def test_a_query_with_no_detector_node_never_activates():
    # A pure-control sim (or a test publishing its own /detections) has no
    # detector to resume, and _activate_camera would spend 5 s in
    # wait_for_service before the pump's freshness window even opens.
    fake = _pump_fake(live=None)
    fake._detector_present.return_value = False
    _resume(fake, 'forward')
    fake._activate_camera.assert_not_called()


# --- discovery settle: an instant graph read lies about a running node -------


def _present_fake(names_over_time):
    """`names_over_time` is popped one list per get_node_names() call."""
    m = MagicMock()
    m._detector_node.return_value = '/duburi_detector_forward'
    m.client.node.get_node_names.side_effect = list(names_over_time)
    return m


def test_presence_waits_for_discovery_when_asked():
    # First read is empty (discovery not settled), second finds the node.
    fake = _present_fake([[], ['duburi_detector_forward']])
    with patch('duburi_planner.duburi_dsl.rclpy.spin_once'):
        got = DuburiMission._detector_present(fake, 'forward', settle=1.0)
    assert got is True


def test_presence_without_settle_does_not_spin():
    # The exclusivity loop keeps the instant read: a false absent only skips a
    # pause it can redo, and the 5 s service wait is what it exists to avoid.
    fake = _present_fake([[]])
    with patch('duburi_planner.duburi_dsl.rclpy.spin_once') as spin:
        got = DuburiMission._detector_present(fake, 'forward')
    assert got is False
    spin.assert_not_called()


def test_presence_gives_up_at_the_budget():
    fake = _present_fake([[], [], [], [], [], [], [], [], [], [], [], []])
    with patch('duburi_planner.duburi_dsl.rclpy.spin_once'):
        got = DuburiMission._detector_present(fake, 'forward', settle=0.05)
    assert got is False


def test_the_probe_runs_once_per_camera():
    # The settle is not free; a repeat probe cannot learn anything new inside
    # one mission, and paying it per query would make a search loop crawl.
    fake = _pump_fake(live=None)
    fake._detector_present.return_value = False
    _resume(fake, 'forward')
    _resume(fake, 'forward')
    assert fake._detector_present.call_count == 1


def test_frames_in_hand_beat_the_probe():
    # A cache entry IS the detector, alive and publishing. Probing anyway would
    # spend the settle inside a query whose recency window is shorter than it.
    fake = _pump_fake(live=None)
    fake._det_cache = {'forward': (1.0, [])}
    _resume(fake, 'forward')
    fake._detector_present.assert_not_called()
    fake._activate_camera.assert_not_called()


# --- camera_available: the branch a mission needs before it commits ----------


def _avail_fake(present):
    m = MagicMock()
    m.camera = 'forward'
    m._camera_available = {}
    m._DISCOVERY_SETTLE_S = 0.0
    m._detector_present.return_value = present
    return m


def test_camera_available_reports_a_live_detector():
    fake = _avail_fake(True)
    assert DuburiMission.camera_available(fake, 'downward') is True


def test_camera_available_reports_an_absent_detector_and_warns():
    fake = _avail_fake(False)
    assert DuburiMission.camera_available(fake, 'downward') is False
    assert fake.log.warning.called, 'an absent camera must be said out loud'


def test_camera_available_is_cached():
    fake = _avail_fake(False)
    DuburiMission.camera_available(fake, 'downward')
    DuburiMission.camera_available(fake, 'downward')
    assert fake._detector_present.call_count == 1


def test_camera_available_defaults_to_the_sticky_camera():
    fake = _avail_fake(True)
    DuburiMission.camera_available(fake)
    fake._detector_present.assert_called_once()
    assert fake._detector_present.call_args.args[0] == 'forward'
