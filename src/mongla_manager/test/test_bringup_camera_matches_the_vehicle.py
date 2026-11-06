"""B47 -- bringup's camera hardware must be the vehicle's camera.

⛔ WHY THIS FILE EXISTS. `bringup.launch.py` starts the Pi + SROT vehicle, but
`camera` used to default to `forward` -- a generic USB profile with no fourcc
and no calibration. Nothing tied the hardware profile to the vehicle, and the
mismatch is silent.

Measured on the Pi, running the documented bringup line:

    average rate: 5.003          (the Fantech does 15.00 with fourcc: MJPG;
                                  the `forward` profile declares no fourcc,
                                  so it negotiates YUYV and loses 3x)
    k: [0.0, ...]                (no calibration declares `forward`, correctly
                                  -- `forward` and `pi_forward` are DIFFERENT
                                  PHYSICAL CAMERAS)

A frame rate reads as "vision is laggy" and a zero K only bites when the
srot vision uplink is switched on, so neither surfaces as a fault.

THE ROLE IS NOT THE HARDWARE, AND THIS IS THE WHOLE POINT. `camera` names the
topics (/mongla/vision/<camera>/...) and the nodes (mongla_detector_<camera>)
that ~96 call sites and the mission DSL's own `camera='forward'` default bind
to. Renaming it to `pi_forward` to fix the hardware would have moved every
topic and broken every mission. `camera_profile` moves the hardware and
leaves the role alone -- the same split `vision_pi.launch.py` already makes
between `fwd_profile` and the fixed `mongla_camera_forward` node name.

Text-level on purpose, like test_launch_and_node_defaults_agree.py: no ROS
runtime, and it reads the file the operator reads.
"""
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.abspath(os.path.join(_HERE, '..'))
_BRINGUP = os.path.join(_PKG, 'launch', 'bringup.launch.py')
_VISION_PKG = os.path.abspath(
    os.path.join(_PKG, '..', 'mongla_vision'))
_VISION_LAUNCH = os.path.join(_VISION_PKG, 'launch', 'vision.launch.py')
_CAMERAS_YAML = os.path.join(_VISION_PKG, 'config', 'cameras.yaml')

def _defaults(path):
    src = open(path).read()
    return dict(re.findall(
        r"DeclareLaunchArgument\(\s*'([A-Za-z0-9_]+)'\s*,\s*"
        r"default_value=\s*'([^']*)'", src))


def test_the_camera_profile_is_the_pi_camera():
    profile = _defaults(_BRINGUP).get('camera_profile')
    assert profile is not None, (
        'bringup declares no camera_profile. Without it the camera ROLE and '
        'the camera HARDWARE are the same string, so pointing a role at '
        'different hardware renames every topic.')
    assert profile.startswith('pi_'), (
        f'camera_profile defaults to {profile!r}, which is not the vehicle '
        f'camera. Measured cost of exactly this mismatch: 5.00 Hz instead of '
        f'15.00, and CameraInfo.k all zero.')


def test_the_camera_ROLE_default_has_not_been_renamed():
    """The role is load-bearing: ~96 sites and the DSL default bind to it.

    If this ever fails because someone "fixed" the hardware by renaming the
    role, the fix is camera_profile, not this assertion.
    """
    d = _defaults(_BRINGUP)
    assert d.get('camera') == 'forward', (
        f"bringup's camera role default is {d.get('camera')!r}, not "
        f"'forward'. That renames /mongla/vision/forward/* and "
        f"mongla_detector_forward, which the mission DSL's own default "
        f"camera='forward' still expects. Change camera_profile instead.")


def test_bringup_actually_PASSES_the_profile_through():
    """A declared-but-unpassed argument is inert and reads as correct -- the
    same class as the four configs in mongla_vision measured to reach
    nothing."""
    src = open(_BRINGUP).read()
    assert re.search(r"'profile'\s*:\s*LaunchConfiguration\('camera_profile'\)",
                     src), (
        'bringup declares camera_profile but never passes it to the vision '
        'launch, so it changes nothing and the default stays whatever '
        'vision.launch.py picks.')


def test_vision_launch_ACCEPTS_a_profile_distinct_from_the_role():
    src = open(_VISION_LAUNCH).read()
    assert re.search(r"DeclareLaunchArgument\(\s*'profile'", src), (
        'vision.launch.py has no `profile` argument, so bringup passing one '
        'is silently dropped -- IncludeLaunchDescription raises only for '
        'MISSING REQUIRED args, never for extra ones.')
    # and it must actually reach camera_node's profile parameter
    assert "LaunchConfiguration('profile')" in src, (
        '`profile` is declared but never read.')


def test_every_profile_named_by_a_default_EXISTS_in_cameras_yaml():
    if not os.path.isfile(_CAMERAS_YAML):
        pytest.skip('cameras.yaml not present')
    text = open(_CAMERAS_YAML).read()
    known = set(re.findall(r'^\s{4}([a-z_][a-z0-9_]*):\s*$', text, re.M))
    known |= set(re.findall(r'^\s{4}([a-z_][a-z0-9_]*):\s*\{', text, re.M))
    assert known, 'parsed no profiles -- has cameras.yaml changed shape?'
    d = _defaults(_BRINGUP)
    for key in ('camera', 'camera_profile'):
        val = d.get(key)
        if val:
            assert val in known, (
                f'bringup {key} defaults to {val!r}, which is not a profile '
                f'in cameras.yaml. camera_node would fall through to its '
                f'non-profile branch. Known: {sorted(known)}')
