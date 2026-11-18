"""The coast ladder must stay ORDERED, in seconds, at the shipped defaults.

The rungs, innermost first:

    vision.coast_s  <  vision.lost_grace_s  <  tracker max_predict_s
                                            <= tracker track_buffer / 30

Every rung is seconds of capture time, so no detection rate can reorder them.
The two tracker rungs used to be frame counts sized from a hand-set
`frame_rate`, and each speed-up of perception walked the outer rung inside an
inner one: a target dropped mid-lock while the tracker still published and
looked healthy (BUGS.md B-114). The rate independence itself is proven by
`test_roboflow_tracker.py::test_the_coast_window_is_seconds_on_capture_time_at_any_rate`
and `test_the_kalman_expiry_is_seconds_at_any_rate` below.
"""
import pathlib
import re

_PKG = pathlib.Path(__file__).resolve().parents[1]
_TRACKER = _PKG / 'mongla_vision' / 'tracker_node.py'
_TUNABLES = (_PKG.parents[0] / 'mongla_manager' / 'mongla_manager'
             / 'vision_tunables.py')


def _node_default(name):
    m = re.search(rf"declare_parameter\('{name}',\s*([0-9.]+)\)",
                  _TRACKER.read_text())
    assert m, f'{name} is gone from tracker_node'
    return float(m.group(1))


def _tunable(name):
    m = re.search(rf"'vision\.{name}':\s*([0-9.]+)", _TUNABLES.read_text())
    assert m, f'vision.{name} is gone'
    return float(m.group(1))


def test_the_rungs_are_ordered_in_seconds():
    coast, grace = _tunable('coast_s'), _tunable('lost_grace_s')
    kal = _node_default('max_predict_s')
    buf = _node_default('track_buffer') / 30.0
    assert coast < grace < kal <= buf, (
        f'coast_s {coast} < lost_grace_s {grace} < max_predict_s {kal} '
        f'<= track_buffer/30 {buf:.2f} no longer holds: an outer rung ends '
        f'before an inner one and the coast truncates mid-lock.')


def test_the_node_has_no_frame_rate_to_get_wrong():
    """A `frame_rate` parameter is how both previous truncations happened."""
    src = _TRACKER.read_text()
    assert "declare_parameter('frame_rate'" not in src
    assert 'max_predict_frames' not in src


def _kalman_survival_s(rate_hz, max_predict_s=1.5):
    from mongla_vision.tracking.kalman import TrackKalmanSmoother
    kal = TrackKalmanSmoother(max_predict_s=max_predict_s)
    dt, t = 1.0 / rate_hz, 0.0
    kal.smooth(1, 100.0, 100.0, t, predicted=False)
    t += dt
    kal.smooth(1, 100.0, 100.0, t, predicted=False)
    gone = t
    while not kal.is_expired(1):
        t += dt
        kal.smooth(1, 100.0, 100.0, t, predicted=True)
    return t - gone


def test_the_kalman_expiry_is_seconds_at_any_rate():
    """1.5 s of coast at 20 Hz and at 98 Hz. In frames it was 0.31 s at 98."""
    for hz in (20.0, 32.0, 98.0):
        assert 1.5 <= _kalman_survival_s(hz) < 1.5 + 1.0 / hz + 1e-9, hz
