"""The reference patch must come from the frame its box was computed on.

`snap` defines the anchor's reference patch, and that patch IS the object
model: its width is taken to be the target's true width, so every later range
is scaled by it. It was snapped from the CURRENT frame using the LAST
detection's box, with nothing comparing the two stamps -- and this repo's own
measurement puts the detector 32 ms median / 48 p95 behind the frame in hand,
which at these rates is one to two frames. On a moving hull that box no longer
bounds the target in the frame it is applied to.

Steering rungs cannot wait, which is why they use freshness decay. Snapping
happens ONCE, so waiting for the right frame costs a detection and removes the
error instead of bounding it.
"""
import contextlib
import types

import numpy as np
import pytest

from mongla_vision.lock_node import _stamp_key


def _hdr(sec, nanosec=0):
    from std_msgs.msg import Header
    h = Header()
    h.stamp.sec, h.stamp.nanosec = sec, nanosec
    return h


@contextlib.contextmanager
def _node():
    import rclpy
    from mongla_vision.lock_node import LockNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = LockNode()
    try:
        yield node
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


def test_a_stamp_key_does_not_lose_nanoseconds():
    """Compared as a float, two different frames can read as the same one.

    ROS epoch seconds are ~1.8e9, where a float64 resolves about 240 ns -- so
    frames a few hundred nanoseconds apart would compare EQUAL and the wrong
    one would be paired, silently, which is the failure this exists to stop.
    """
    a, b = _stamp_key(_hdr(1789068980, 1)), _stamp_key(_hdr(1789068980, 2))
    assert a != b
    assert float(f'{1789068980}.{1:09d}') == float(f'{1789068980}.{2:09d}'), (
        'the float collision this guard is about no longer reproduces; the '
        'integer comparison is still correct, but re-derive the reason')


def test_an_unstamped_header_is_not_a_match():
    """A zero stamp must not pair with another zero stamp -- that is the
    absence of a stamp, not agreement about one."""
    assert _stamp_key(_hdr(0, 0)) is None
    assert _stamp_key(None) is None


def test_the_frame_matching_the_detection_is_returned():
    pytest.importorskip('rclpy')
    with _node() as node:
        frames = {}
        for i in range(4):
            g = np.full((8, 8), i, np.uint8)
            frames[i] = g
            node._recent.append((_stamp_key(_hdr(100 + i)), g))
        got = node._frame_for(_hdr(101))
        assert got is frames[1], (
            'the wrong frame was paired with the detection -- the reference '
            'patch would be snapped from an image the box does not describe')


def test_a_frame_that_aged_out_defers_the_snap_rather_than_faking_it():
    """Returning the newest frame instead would bake the skew into the object
    model with nothing logged. None means 'wait for the next detection'."""
    pytest.importorskip('rclpy')
    with _node() as node:
        for i in range(4):
            node._recent.append((_stamp_key(_hdr(100 + i)),
                                 np.zeros((8, 8), np.uint8)))
        assert node._frame_for(_hdr(50)) is None
        assert node._frame_for(_hdr(0, 0)) is None


def test_the_ring_is_bounded():
    """An unbounded history is a leak on a node that runs a whole mission."""
    pytest.importorskip('rclpy')
    with _node() as node:
        for i in range(500):
            node._recent.append((_stamp_key(_hdr(i + 1)),
                                 np.zeros((8, 8), np.uint8)))
        assert len(node._recent) <= 8, len(node._recent)


def test_the_snap_site_uses_the_paired_frame():
    """Wiring: the helper exists and the snap actually goes through it."""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py').read_text()
    i = src.index('def _loop')
    body = src[i:]
    assert 'self._anchor.snap(snap_gray' in body, (
        'snap no longer uses the stamp-paired frame')
    assert 'self._anchor.snap(gray' not in body, (
        'snap fell back to the newest frame')
