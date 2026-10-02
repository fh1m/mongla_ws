"""Ctrl-C's disarm confirmation is not killed by the launch (issue #21 item 7).

`ros2 launch` sends SIGTERM `sigterm_timeout` (5 s by default) after SIGINT.
The emergency disarm polled up to 15 s, so a slow confirmation was killed
mid-poll and the operator never saw whether it took.
"""
import importlib.util
import pathlib
import types

import pytest

pytest.importorskip('rclpy')

_LAUNCH = (pathlib.Path(__file__).resolve().parents[1] / 'launch'
           / 'bringup.launch.py')


def test_the_emergency_disarm_is_bounded_and_asks_for_that_bound():
    import mongla_manager.auv_manager_node as amn
    asked = []

    class _Fc:
        def disarm(self, timeout=15.0):
            asked.append(timeout)
            return True, 'DISARMED'

        def stop_motion(self):
            pass

        def send_neutral(self):
            pass

    fc = _Fc()
    node = types.SimpleNamespace(
        fc=fc, pixhawk=fc,
        mongla=types.SimpleNamespace(request_abort=lambda: None,
                                     _heading_lock=None),
        heartbeat=types.SimpleNamespace(stop=lambda: None),
        yaw_source=types.SimpleNamespace(close=lambda: None),
        _vision_states={})
    try:
        amn._emergency_stop(node)
    except Exception:          # later teardown steps need a real node
        pass
    assert asked == [amn.EMERGENCY_DISARM_S]


def test_the_launch_grace_outlives_the_disarm():
    import mongla_manager.auv_manager_node as amn
    spec = importlib.util.spec_from_file_location('_bringup', _LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.MANAGER_SIGTERM_TIMEOUT_S >= amn.EMERGENCY_DISARM_S + 2.0
    # ...and the BUILT launch action carries it -- read off the action, not
    # the file's text, so a commented-out argument cannot pass.
    from launch_ros.actions import Node
    mgr = [e for e in mod.generate_launch_description().entities
           if isinstance(e, Node) and e.node_executable == 'start']
    assert len(mgr) == 1
    subs = mgr[0]._ExecuteLocal__sigterm_timeout
    assert ''.join(s.text for s in subs) == str(mod.MANAGER_SIGTERM_TIMEOUT_S)
