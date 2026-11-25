"""A replayed frame is stamped when it is handed over, not before the wait.

`VideoFileCamera.read` paces playback by sleeping out the frame interval.
The stamp was taken BEFORE that sleep, so at 0.1x a 30 fps clip each frame
was handed over a third of a second older than its stamp said it was, and
every freshness gate downstream measured the replay's pacing as latency.
"""
import time

import cv2
import numpy as np

from mongla_vision.cameras.video_file import VideoFileCamera


def test_the_stamp_is_taken_after_the_pacing_wait(tmp_path):
    path = str(tmp_path / 'clip.avi')
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), 30.0, (64, 48))
    for i in range(5):
        out.write(np.full((48, 64, 3), i * 40, np.uint8))
    out.release()
    cam = VideoFileCamera(path, loop=False)
    cam.set_speed(0.1)                      # 333 ms between frames
    cam.read()
    _, meta = cam.read()
    handed = time.monotonic()
    cam.close()
    assert meta.fresh
    assert handed - meta.stamp_monotonic < 0.05


def test_a_ros_topic_frame_carries_one_instant_on_both_clocks():
    """RosTopicCamera kept the arrival on the monotonic clock but stamped
    the wall clock at read: one frame, two instants, and the header (wall)
    aged by however long the frame sat before the camera loop read it."""
    import types
    from sensor_msgs.msg import Image
    from mongla_vision.cameras.ros_topic import RosTopicCamera
    node = types.SimpleNamespace(create_subscription=lambda *a, **k: object())
    cam = RosTopicCamera(node=node, topic='/x')
    msg = Image(height=2, width=2, encoding='bgr8', step=6, data=bytes(12))
    cam._on_image(msg)
    time.sleep(0.2)
    _, meta = cam.read()
    offset = time.time() - time.monotonic()
    assert abs((meta.stamp_wall - meta.stamp_monotonic) - offset) < 0.02


def test_a_ros_topic_frame_is_as_old_as_its_header_says():
    """The source stamped arrival and never read the header, so a frame
    delayed in transport looked live (audit 2026-11-24). A header from
    another clock still falls back to arrival."""
    import types
    from sensor_msgs.msg import Image
    from mongla_vision.cameras.ros_topic import RosTopicCamera
    node = types.SimpleNamespace(create_subscription=lambda *a, **k: object())

    def frame_with_header_age(age):
        cam = RosTopicCamera(node=node, topic='/x')
        msg = Image(height=2, width=2, encoding='bgr8', step=6, data=bytes(12))
        t = time.time() - age
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        cam._on_image(msg)
        _, meta = cam.read()
        return time.monotonic() - meta.stamp_monotonic
    assert 0.28 < frame_with_header_age(0.3) < 0.4
    assert frame_with_header_age(3600.0) < 0.05       # another clock: arrival
