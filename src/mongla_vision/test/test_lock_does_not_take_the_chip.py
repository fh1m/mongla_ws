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


def test_the_vehicle_launch_runs_the_anchor_on_the_cpu():
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location('_vpi', _LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    args = {e.name: e for e in mod.generate_launch_description().entities
            if isinstance(e, DeclareLaunchArgument)}
    assert args['anchor_xfeat_hef'].default_value[0].text == 'false'


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
