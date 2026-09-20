"""A profile for the WRONG HOST must say so, not fail inside V4L2.

`pi_forward` names /dev/mongla_cam_forward, a udev symlink that exists on the
Pi box; `forward` names the Jetson's Blue Robotics unit. bringup defaults
`camera_profile:=pi_forward` to match its own `flight_controller:=srot`, so a
Jetson operator passing `vision:=true` without `camera_profile:=forward` gets
a device that is not there.

Without this the failure surfaces from inside the camera open, naming neither
the profile nor the host -- on a pool deck that is a lost session for a launch
argument. The vision include is gated on `vision:=true` (default false), so
this is the only path that reaches it.

Reads the source: the check must be provable with no camera and no ROS.
"""
import pathlib

_PKG = pathlib.Path(__file__).resolve().parents[1]
_NODE = _PKG / 'mongla_vision' / 'camera_node.py'


def test_camera_node_checks_that_a_profile_device_EXISTS():
    src = _NODE.read_text()
    assert 'os.path.exists(probe)' in src, (
        'camera_node never checks that a profile device exists, so the '
        'wrong-host profile fails inside the V4L2 open with no mention of '
        'the profile or the host.')


def test_the_message_NAMES_the_fix_not_just_the_problem():
    """An error that does not say what to type is a puzzle, not a message.

    Matched on short contiguous fragments: the message is built from adjacent
    f-strings, so any phrase spanning the join does not exist in the source.
    A first draft of this test searched for one such phrase and failed
    against correct code.
    """
    src = _NODE.read_text()
    i = src.find('not exist on this host')
    assert i > 0, 'the wrong-host error message is gone'
    window = src[i:i + 800]
    assert 'camera_profile' in window, (
        'the wrong-host error does not name `camera_profile`, the argument '
        'the operator actually has to change.')
    assert 'pi_' in window, (
        'the message does not distinguish the Pi profiles from the Jetson '
        'ones, which is the discrimination the operator must make.')
