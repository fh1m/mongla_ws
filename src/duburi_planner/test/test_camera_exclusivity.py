"""use_camera exclusivity: exactly one detector infers after a switch.

_activate_camera must pause EVERY other known detector (not just the previously
live one) so a stray `paused:=false` launch (both inferring from t=0) is
corrected on the first switch -- the concurrent-inference OOM fix. No ROS: the
real method runs against a fake self with mocked detector-param calls.
"""

from unittest.mock import MagicMock, patch

from duburi_planner.duburi_dsl import DuburiMission


def _fake(live=None):
    m = MagicMock()
    m._live_camera = live
    m._KNOWN_CAMERAS = DuburiMission._KNOWN_CAMERAS
    m._CAM_SWITCH_SETTLE_S = 0.0
    m.cam_switch_settle_s = 0.0
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
