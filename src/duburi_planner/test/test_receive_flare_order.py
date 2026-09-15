"""`duburi.receive_flare_order()` decodes real Image messages end to end."""
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')
import rclpy                                          # noqa: E402
from sensor_msgs.msg import Image                     # noqa: E402
from rclpy.node import Node                           # noqa: E402

from duburi_planner.duburi_dsl import DuburiMission   # noqa: E402

BGR = {'R': (40, 40, 255), 'B': (255, 60, 30), 'Y': (30, 230, 255)}


def _frames(order, fps=15.0):
    plan = [('.', 1.5)]
    for _ in range(2):
        for c in order:
            plan += [(c, 0.6), ('.', 0.4)]
        plan += [('.', 1.6)]
    out, t = [], 1000.0
    for sym, dur in plan:
        for _ in range(int(round(dur * fps))):
            img = np.full((240, 320, 3), (70, 90, 60), np.uint8)
            if sym != '.':
                cv2.circle(img, (160, 120), 12, BGR[sym], -1)
            out.append((t, img)); t += 1 / fps
    return out


def test_an_order_flashed_at_the_camera_is_returned():
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = Node('flare_order_probe')
    pub = node.create_publisher(Image, '/duburi/vision/forward/image_raw', 50)
    frames = _frames(('B', 'R', 'Y'))
    m = MagicMock()
    m.camera = 'forward'
    m.client = type('C', (), {'node': node})()
    state = {'i': 0}

    real_spin = rclpy.spin_once

    def spin_and_publish(n, timeout_sec=0.0):
        if state['i'] < len(frames):
            t, img = frames[state['i']]
            msg = Image()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = 'bgr8'
            msg.step = img.shape[1] * 3
            msg.data = img.tobytes()
            msg.header.stamp.sec = int(t)
            msg.header.stamp.nanosec = int((t - int(t)) * 1e9)
            pub.publish(msg)
            state['i'] += 1
        real_spin(n, timeout_sec=0.005)

    try:
        import duburi_planner.duburi_dsl as dsl
        orig = dsl.rclpy.spin_once
        dsl.rclpy.spin_once = spin_and_publish
        try:
            got = DuburiMission.receive_flare_order(m, timeout=20.0)
        finally:
            dsl.rclpy.spin_once = orig
    finally:
        node.destroy_node()
        if started and rclpy.ok():
            rclpy.shutdown()
    assert got == ('B', 'R', 'Y')
