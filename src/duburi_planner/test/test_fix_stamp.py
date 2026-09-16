"""A pool fix is stamped with the capture time of the frame it came from."""
from types import SimpleNamespace
from unittest.mock import MagicMock

from duburi_planner.duburi_dsl import DuburiMission


def _dsl():
    m = MagicMock()
    m.__dict__['_det_capture'] = {'forward': (1234, 500)}
    pub = MagicMock()
    m.__dict__['_fix_pub'] = pub
    from builtin_interfaces.msg import Time
    stamp = Time(sec=9999, nanosec=1)
    m.client.node.get_clock.return_value.now.return_value.to_msg.return_value = stamp
    return m, pub


def test_a_fix_from_a_camera_carries_that_frames_capture_stamp():
    m, pub = _dsl()
    DuburiMission._publish_fix(m, 1.0, 2.0, sigma=0.3, camera='forward')
    msg = pub.publish.call_args[0][0]
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1234, 500)


def test_without_a_camera_frame_it_falls_back_to_now():
    m, pub = _dsl()
    DuburiMission._publish_fix(m, 1.0, 2.0, camera='downward')
    msg = pub.publish.call_args[0][0]
    assert msg.header.stamp.sec == 9999
