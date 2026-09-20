"""The publish rate gate must come BEFORE the decode, not after.

`read()` is where the JPEG is decoded. The gate used to sit inside
`_publish()`, i.e. after it -- so with capture at 210 fps and a 40 Hz publish
this node decoded 210 frames a second and discarded 170 of them. Measured A/B
on the Pi, same instrument, downward paused:

    forward camera CPU    63.1 %  ->  17.6 %
    whole vision stack   128.5 %  ->  71.1 %
    detections age med    25.74   ->  26.35 ms   (unchanged, within noise)
    detections interval max 86.03 ->  59.42 ms   (better)

The round-31 note claiming "the mailbox decouples capture from DECODE" was
true of the SOURCE and false of this loop, which was the only reader and read
at capture rate. Skipping the read is safe and is the entire point of a
mailbox: an unread frame is replaced, so waiting for the slot and then reading
yields the NEWEST frame rather than whichever one finished decoding.

These tests RUN the loop and count decodes. A source grep would have passed
through the defect -- `self._min_period` appeared in both the broken and the
fixed arrangement, and only its POSITION differed.
"""
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision import camera_node as CN                    # noqa: E402


class _Meta:
    def __init__(self):
        self.fresh = True
        self.stamp_wall = time.time()
        self.stamp_monotonic = time.monotonic()
        self.width = 64
        self.height = 48
        self.frame_index = 0


class _CountingCam:
    """Always has a fresh frame. `read()` stands in for the decode, because in
    the real source that is exactly where `cv2.imdecode` runs."""

    def __init__(self):
        self.reads = 0

    def read(self):
        self.reads += 1
        return object(), _Meta()


def _loop_for(seconds, *, publish_hz):
    """Run the SHIPPING `_capture_loop` against a fake camera for a while."""
    node = object.__new__(CN.CameraNode)
    node._cam = _CountingCam()
    node._sink = None          # un-composed path
    node._min_period = (1.0 / publish_hz) if publish_hz > 0 else 0.0
    node._last_pub = 0.0
    node._IDLE_WAIT_S = CN.CameraNode._IDLE_WAIT_S
    published = []

    def _fake_publish(_frame, _meta):
        if node._min_period > 0.0:
            node._last_pub = time.monotonic()
        published.append(time.monotonic())
    node._publish = _fake_publish

    stop = threading.Event()

    class _Rclpy:
        @staticmethod
        def ok():
            return not stop.is_set()
    real, CN.rclpy = CN.rclpy, _Rclpy()
    try:
        t = threading.Thread(target=node._capture_loop, daemon=True)
        t.start()
        time.sleep(seconds)
        stop.set()
        t.join(timeout=2.0)
    finally:
        CN.rclpy = real
    return node._cam.reads, len(published)


def test_a_throttled_node_decodes_ONLY_what_it_publishes():
    """THE FIX. The camera can hand over a frame every 4.8 ms; at 40 Hz we
    want ~40 decodes a second, not ~210."""
    reads, published = _loop_for(1.0, publish_hz=40)
    assert published >= 25, f'only {published} publishes -- the gate is stuck'
    # One read per publish, plus at most a couple of slack iterations.
    assert reads <= published + 3, (
        f'{reads} decodes for {published} publishes -- the rate gate is '
        f'behind the decode again')


def test_an_unthrottled_node_still_reads_every_frame():
    """`publish_rate_hz<=0` means "follow the camera" and must be untouched --
    it is the downward camera's configuration."""
    reads, published = _loop_for(0.3, publish_hz=0)
    assert reads == published, (reads, published)
    assert reads > 50, f'only {reads} reads -- the uncapped path was throttled'


def test_the_slot_helper_is_zero_when_no_rate_is_pinned():
    node = object.__new__(CN.CameraNode)
    node._min_period = 0.0
    node._last_pub = 0.0
    assert node._time_to_next_slot() == 0.0


def test_the_slot_helper_counts_down_and_goes_non_positive():
    node = object.__new__(CN.CameraNode)
    node._min_period = 0.050
    node._last_pub = time.monotonic()
    first = node._time_to_next_slot()
    assert 0.0 < first <= 0.050
    node._last_pub = time.monotonic() - 0.060
    assert node._time_to_next_slot() <= 0.0


def test_the_gate_never_sleeps_longer_than_a_shutdown_can_wait():
    """A 0.5 Hz publish must not make Ctrl-C wait two seconds."""
    node = object.__new__(CN.CameraNode)
    node._cam = _CountingCam()
    node._sink = None          # un-composed path
    node._min_period = 2.0
    node._last_pub = time.monotonic()
    node._IDLE_WAIT_S = CN.CameraNode._IDLE_WAIT_S
    node._publish = lambda *_a: None
    stop = threading.Event()

    class _Rclpy:
        @staticmethod
        def ok():
            return not stop.is_set()
    real, CN.rclpy = CN.rclpy, _Rclpy()
    try:
        t = threading.Thread(target=node._capture_loop, daemon=True)
        t.start()
        time.sleep(0.05)
        stop.set()
        t.join(timeout=0.5)
        assert not t.is_alive(), 'loop slept through shutdown'
    finally:
        CN.rclpy = real
