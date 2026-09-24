"""`device_path:=` must actually change the device the camera opens.

⛔ THE DEFECT. A profile carries its own `device_path` key (pi_forward is
`/dev/mongla_cam_forward`). `camera_node` overrode `profile['device']` only, so
the v4l2 mailbox -- which reads `device_path` -- kept opening the profile's udev
symlink, on a host where that symlink does not exist. Measured on the vehicle:

    [CAM ] device_path (port-stable) -> /dev/video0
    [CAM ] v4l2 mailbox failed on '/dev/mongla_cam_forward'

The override was logged as applied on the line above the one that did not apply
it, so every log said the parameter worked. The whole graph then produced no
images, no detections and no lock, and every rung reported 0 %.

Reads the source rather than importing it: `camera_node` needs rclpy and a ROS
graph, which is why nothing caught this before hardware.
"""
import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / 'mongla_vision' / 'camera_node.py'


def test_the_override_writes_the_key_the_profile_uses():
    src = SRC.read_text()
    i = src.index("if device_path:")
    window = src[i:i + 700]
    assert "profile['device_path'] = device_path" in window, (
        "camera_node overrides profile['device'] but not profile['device_path'], "
        "so a profile's own udev symlink survives and the v4l2 mailbox opens it "
        "instead of the operator's device.")
    assert "profile['device'] = device_path" in window, (
        'the int-device path still needs the override too')


def test_the_profile_really_does_carry_that_key():
    """If profiles stop carrying `device_path`, the test above is guarding
    nothing and should be revisited rather than silently passing."""
    cfg = (SRC.parent / 'config.py').read_text()
    assert "'device_path'" in cfg, (
        'no profile carries device_path any more -- re-derive this guard')
