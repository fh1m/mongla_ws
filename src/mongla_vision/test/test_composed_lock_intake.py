"""The composed lock ladder takes frames by reference and dies cleanly.

Measured on the vehicle 2026-10-03 (measured-bars 28.6): composing the ladder
into the detector process with topic intake took both detectors from 71 to
21.6 Hz combined, the cost being rclpy intake under the shared GIL; frames by
reference (`direct_feed`), grey on demand and HailoRT's scheduler brought it
to ~64 Hz with XFeat running on the chip.
"""
import contextlib
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter as P            # noqa: E402
from std_msgs.msg import Header                       # noqa: E402


@contextlib.contextmanager
def _node(**params):
    from mongla_vision.lock_node import LockNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = LockNode(parameter_overrides=[P(k, value=v)
                                         for k, v in params.items()])
    try:
        yield node
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


def _topics(node):
    return {s.topic_name for s in node.subscriptions}


def test_direct_feed_subscribes_to_no_image_or_camera_info():
    with _node(direct_feed=True, anchor=False) as n:
        topics = _topics(n)
        assert not any(t.endswith('image_raw') for t in topics), topics
        assert not any(t.endswith('camera_info') for t in topics), topics


def test_the_topic_path_still_subscribes_without_it():
    """Replay and the standalone node take frames from a bag or a topic."""
    with _node(anchor=False) as n:
        topics = _topics(n)
        assert any(t.endswith('image_raw') for t in topics)
        assert any(t.endswith('camera_info') for t in topics)


def test_submit_frame_stores_a_reference_and_greys_on_demand():
    with _node(direct_feed=True, anchor=False) as n:
        frame = np.zeros((48, 64, 3), np.uint8)
        frame[..., 2] = 200
        h = Header()
        h.stamp.sec, h.stamp.nanosec = 5, 7
        n.submit_frame(frame, h)
        assert n._recent[-1][1] is frame, 'the frame was copied'
        g = n._gray_now()
        assert g.ndim == 2 and g.shape == (48, 64) and int(g[0, 0]) > 0
        assert n._frame_for(h).ndim == 2, 'a snap must get a grey frame'


def test_destroy_stops_the_ladder_thread_first():
    """Destroyed mid-tick, the ladder published on a dead handle and logged a
    fault on every Ctrl-C (found by tools/composed_lock_harness.py). Checked
    BEFORE rclpy shuts down: shutdown alone also ends the loop, which would
    hide a destroy that does not."""
    from mongla_vision.lock_node import LockNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    try:
        n = LockNode(parameter_overrides=[P('direct_feed', value=True),
                                          P('anchor', value=False)])
        t = n._thread
        assert t.is_alive()
        n.destroy_node()
        assert not t.is_alive(), 'the ladder thread outlived destroy_node'
    finally:
        if started:
            rclpy.shutdown()
