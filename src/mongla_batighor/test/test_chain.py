"""The chain view reads back a synthetic chain whose delays were chosen.

Written before `chain.py` (the scorer first). Truth by construction: each
stage's messages are stamped a chosen time before they are seen, at a chosen
rate, so a meter that misreads a clock, a window or a unit fails here and not
on a pool day. Bars, set before the code: ages within 0.5 ms of truth and
rates within 2 % on the pure meter; within 5 ms through ROS. A stage that
sent nothing reads absent (None), never 0.
"""
import json
import time

import pytest

from mongla_batighor import chain


def _feed(meter, key, hz, age_s, t0, t1, stamped=True):
    t = t0
    while t < t1:
        meter.note(key, (t - age_s) if stamped else None, t)
        t += 1.0 / hz


def test_the_meter_reads_back_chosen_delays_and_rates():
    m = chain.ChainMeter(window_s=5.0)
    _feed(m, 'frame:downward', 60.0, 0.012, 100.0, 105.0)
    _feed(m, 'detect:downward', 60.0, 0.031, 100.0, 105.0)
    _feed(m, 'imu', 50.0, 0.004, 100.0, 105.0)
    _feed(m, 'rpm', 10.0, 0.0, 100.0, 105.0, stamped=False)
    s = m.summary(105.0)
    for key, hz, age in (('frame:downward', 60, 12.0), ('detect:downward', 60, 31.0),
                         ('imu', 50, 4.0)):
        assert s[key]['hz'] == pytest.approx(hz, rel=0.02), key
        assert s[key]['age_ms_p50'] == pytest.approx(age, abs=0.5), key
        assert s[key]['age_ms_p95'] == pytest.approx(age, abs=0.5), key
    assert s['rpm']['hz'] == pytest.approx(10, rel=0.02)
    assert s['rpm']['age_ms_p50'] is None          # no stamp: no age, not 0


def test_a_stage_that_sent_nothing_is_absent_and_an_old_one_goes_quiet():
    m = chain.ChainMeter(window_s=5.0)
    _feed(m, 'lock:forward', 30.0, 0.02, 100.0, 101.0)
    s = m.summary(110.0)                        # nothing for 9 s
    assert s['lock:forward']['hz'] == 0.0
    assert s['lock:forward']['age_ms_p50'] is None
    assert s['lock:forward']['last_s'] == pytest.approx(9.0, abs=0.05)
    assert 'demand' not in s or s['demand']['hz'] == 0.0


def test_every_registry_row_says_where_its_time_comes_from():
    for st in chain.stages():
        assert st.topic.startswith('/mongla/'), st
        assert st.clock, f'{st.key}: a stage must name its clock or say it has none'
    keys = [s.key for s in chain.stages()]
    assert len(keys) == len(set(keys))
    absent = [s for s in chain.stages() if not s.stamped]
    assert all(s.ask for s in absent), 'a stage with no stamp names the ask for one'


def test_the_node_reads_a_published_chain_within_the_bar():
    rclpy = pytest.importorskip('rclpy')
    from sensor_msgs.msg import CameraInfo
    from geometry_msgs.msg import Vector3Stamped
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data  # noqa: F401
    rclpy.init()
    try:
        node = chain.ChainNode(cameras=('downward',), period_s=0.2)
        src = rclpy.create_node('chain_truth')
        from mongla_vision import qos
        pub_cam = src.create_publisher(CameraInfo, '/mongla/vision/downward/camera_info',
                                       qos.CAMERA_INFO)
        pub_dem = src.create_publisher(Vector3Stamped, '/mongla/demand', 10)
        out = []
        src.create_subscription(String, chain.TOPIC, lambda m: out.append(m.data), 10)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(src)

        def stamp(msg, age):
            t = time.time() - age
            msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
            return msg
        end = time.monotonic() + 3.0
        nxt = time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= nxt:
                pub_cam.publish(stamp(CameraInfo(), 0.040))
                pub_dem.publish(stamp(Vector3Stamped(), 0.010))
                nxt += 1 / 30.0
            ex.spin_once(timeout_sec=0.002)
        s = json.loads(out[-1])['stages']
        assert s['frame:downward']['age_ms_p50'] == pytest.approx(40.0, abs=5.0)
        assert s['demand']['age_ms_p50'] == pytest.approx(10.0, abs=5.0)
        assert s['frame:downward']['hz'] == pytest.approx(30.0, rel=0.15)
        node.destroy_node()
        src.destroy_node()
    finally:
        rclpy.shutdown()


def test_nobody_watching_costs_no_subscription():
    """Subscribed for good it cost 17 % of a Pi core with no viewer. With no
    subscriber to the chain it holds none of its stages' subscriptions; a
    viewer brings them, and they go when it does."""
    rclpy = pytest.importorskip('rclpy')
    from std_msgs.msg import String
    rclpy.init()
    try:
        node = chain.ChainNode(cameras=('downward',), period_s=0.05)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)

        def spin(s):
            end = time.monotonic() + s
            while time.monotonic() < end:
                ex.spin_once(timeout_sec=0.01)
        spin(0.5)
        assert list(node.subscriptions) == []
        viewer = rclpy.create_node('chain_viewer')
        sub = viewer.create_subscription(String, chain.TOPIC, lambda m: None, 10)
        ex.add_node(viewer)
        spin(1.0)
        assert len(list(node.subscriptions)) == len(chain.stages(('downward',)))
        viewer.destroy_subscription(sub)
        spin(1.0)
        assert list(node.subscriptions) == []
        node.destroy_node()
        viewer.destroy_node()
    finally:
        rclpy.shutdown()
