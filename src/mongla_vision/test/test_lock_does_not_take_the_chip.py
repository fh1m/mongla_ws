"""A standalone lock node must not open the Hailo (found on the vehicle).

The chip allows one VDevice per process. lock_node runs in its own, so with
XFeat on the HEF it raced the detector process for the chip; when it won,
both cameras' detectors failed with HAILO_OUT_OF_PHYSICAL_DEVICES.
"""
import importlib.util
import pathlib

import pytest

pytest.importorskip('launch_ros')

_LAUNCH = (pathlib.Path(__file__).resolve().parents[1] / 'launch'
           / 'vision_pi.launch.py')


def test_the_vehicle_launch_starts_no_separate_lock_process():
    """XFeat is on the chip on the vehicle -- which is safe ONLY because the
    ladder is composed into the detector process. A separate lock_node in the
    Pi launch would race the detector for the VDevice again."""
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location('_vpi', _LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    exes = [e.node_executable for e in mod.generate_launch_description().entities
            if isinstance(e, Node)]
    assert 'lock_node' not in exes, exes
    assert 'detector_dual_node' in exes


def test_the_node_default_is_the_cpu_too():
    import rclpy
    from mongla_vision.lock_node import LockNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    n = LockNode()
    try:
        assert n.get_parameter('anchor_xfeat_hef').value is False
    finally:
        n.destroy_node()
        if started:
            rclpy.shutdown()
