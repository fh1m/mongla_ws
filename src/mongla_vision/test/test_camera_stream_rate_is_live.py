"""The viewers' video rates change while the camera streams.

`stream_rate_hz` and `preview_rate_hz` feed only the browsers' JPEG topics
(detection reads image_raw), and Batighor's helm lowers `stream` when a
browser's link lags (mongla_batighor/link.py). Read once at start, the
governor could not act: the change was refused as "read once when the
camera opens". A rate is a period on the publisher, so it applies at once,
even on a source that cannot change camera controls (a recorded clip).
"""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision import camera_node as CN                    # noqa: E402


def _node():
    node = object.__new__(CN.CameraNode)
    node._cam = SimpleNamespace(source_kind='video_file')     # no set_controls
    node._jpeg_out = [[None, 1 / 30.0, 0.0], [None, 1 / 5.0, 0.0]]
    node.has_parameter = lambda name: True                    # declared, so a change
    return node


def _p(name, value):
    return SimpleNamespace(name=name, value=value)


def test_a_new_stream_rate_sets_the_period_on_any_source():
    node = _node()
    r = node._on_set([_p('stream_rate_hz', 7.5)])
    assert r.successful, r.reason
    assert abs(node._jpeg_out[0][1] - 1 / 7.5) < 1e-12
    r = node._on_set([_p('preview_rate_hz', 0.0)])
    assert r.successful and node._jpeg_out[1][1] is None        # 0 turns it off


def test_a_rate_that_is_not_a_rate_is_refused_and_changes_nothing():
    node = _node()
    for bad in (-1.0, float('nan'), float('inf'), 'fast'):
        r = node._on_set([_p('stream_rate_hz', bad)])
        assert not r.successful
        assert abs(node._jpeg_out[0][1] - 1 / 30.0) < 1e-12


def test_camera_controls_on_a_clip_are_still_refused():
    r = _node()._on_set([_p('exposure_us', 500)])
    assert not r.successful and 'V4L2' in r.reason
