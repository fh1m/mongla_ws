"""vision_check grades the whole mission graph, and refuses the states that
look healthy and are not.

The grading is a pure function of what one window observed, so each failure
the operator must not miss is built here from the observation that produces
it: a dead detector that reads like a paused one, a detector under the
measured bar, a tracker that never started, flow or the localiser down, and a
board IMU at the wrong rate.
"""
from collections import Counter

from mongla_vision.utils.check_pipeline import (DET_MIN_HZ, FAIL, OK, WARN,
                                                Observed, grade_camera,
                                                grade_graph)


def _healthy() -> Observed:
    o = Observed()
    for cam in ('forward', 'downward'):
        o.frames_hz[cam] = 30.0 if cam == 'forward' else 204.0
        o.det_hz[cam] = 30.0
        o.tracks_hz[cam] = 30.0
        o.seeing[cam] = 'ok'
        o.paused[cam] = False
        o.lock_publishers[cam] = 1
        o.classes[cam] = Counter()
    o.flow_quality_n, o.flow_used_n = 6, 2
    o.flow_calibration = '{"focal": 741}'
    o.imu_hz, o.state_seen, o.odom_hz = 50.0, True, 50.0
    return o


def _status(rows, check):
    return {c: s for s, c, _v in rows}[check]


def _fails(rows):
    return [c for s, c, _v in rows if s == FAIL]


def test_a_healthy_graph_has_no_FAIL():
    assert _fails(grade_graph(_healthy())) == []


def test_a_paused_detector_is_a_warning_and_a_dead_one_a_FAIL():
    """Both publish nothing. Only the detector's own answer tells them apart."""
    o = _healthy()
    o.det_hz['downward'] = 0.0
    o.paused['downward'] = True
    assert _status(grade_graph(o), 'downward detections') == WARN
    o.paused['downward'] = False
    assert _status(grade_graph(o), 'downward detections') == FAIL
    o.paused['downward'] = None              # nothing answered at all
    assert _status(grade_graph(o), 'downward detections') == FAIL


def test_a_detector_under_the_measured_bar_fails():
    o = _healthy()
    o.det_hz['forward'] = DET_MIN_HZ - 0.1
    assert _status(grade_graph(o), 'forward detections') == FAIL


def test_detections_without_tracks_fail():
    o = _healthy()
    o.tracks_hz['forward'] = 0.0
    assert _status(grade_graph(o), 'forward tracks') == FAIL


def test_every_part_of_the_graph_can_fail_on_its_own():
    cases = {
        'forward frames': lambda o: o.frames_hz.__setitem__('forward', 0.0),
        'downward lock ladder': lambda o: o.lock_publishers.__setitem__('downward', 0),
        'flow': lambda o: setattr(o, 'flow_quality_n', 0),
        'board IMU': lambda o: setattr(o, 'imu_hz', 10.1),     # the board's own default
        'board state': lambda o: setattr(o, 'state_seen', False),
        'odometry': lambda o: setattr(o, 'odom_hz', 0.0),
    }
    for check, breaks in cases.items():
        o = _healthy()
        breaks(o)
        assert _fails(grade_graph(o)) == [check], check


def test_a_camera_that_cannot_see_warns_without_failing():
    o = _healthy()
    o.seeing['forward'] = 'covered'
    rows = grade_graph(o)
    assert _status(rows, 'forward sight') == WARN
    assert _fails(rows) == []


def test_one_camera_mode_grades_only_that_camera():
    o = _healthy()
    rows = grade_camera(o, 'forward')
    assert {c for _s, c, _v in rows} == {'forward frames', 'forward sight',
                                         'forward detections', 'forward tracks'}
    assert all(s == OK for s, _c, _v in rows)


def test_a_slow_camera_is_named_as_the_cause():
    """Measured on Kutum in dim light: the Fantech delivered 9.5 fps, and the
    detector 9.9 Hz. The FAIL must say the camera is why."""
    o = _healthy()
    o.frames_hz['forward'], o.det_hz['forward'] = 9.5, 9.9
    v = {c: v for _s, c, v in grade_graph(o)}['forward detections']
    assert 'camera delivers 9.5 fps' in v
