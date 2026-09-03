"""`fps`: 0 means "the profile decides", a positive value overrides it.

This is the round-26 defect and its second half. There, a LAUNCH default of
30.0 silently beat every camera profile -- a 70 Hz camera ran at 30 and nothing
anywhere looked wrong. The fix removed the launch default and left the node's
own `declare_parameter('fps', 30)` able to do exactly the same thing, while the
comment two hundred lines below already claimed the default was 0.

The override earns its place separately: the forward camera delivers 68 Hz raw
against a detector that consumes ~50, and on this Pi the surplus starves the
OTHER camera through USB/CPU contention. Capping it needed an edit to a
checked-in config until now.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

SRC = (Path(__file__).resolve().parents[1] / 'duburi_vision'
       / 'camera_node.py').read_text()


def test_the_node_default_is_the_sentinel_not_a_rate():
    """A concrete default here re-creates the round-26 bug from inside the
    node: every profile silently capped, no warning anywhere."""
    assert "declare_parameter('fps',             0)" in SRC


def test_a_positive_value_overrides_the_profile():
    assert "if fps_override > 0:" in SRC
    assert "profile['fps'] = fps_override" in SRC


def test_the_override_says_what_it_replaced():
    """A cap that does not name the profile value it beat is how round 26
    stayed invisible for a whole round."""
    i = SRC.index('fps_override > 0')
    assert 'fps override' in SRC[i:i + 500]
    assert "get_profile(profile_name).get('fps')" in SRC[i:i + 500]


def test_the_non_profile_branch_still_never_asks_a_driver_for_zero():
    """The sentinel must not leak into a device call."""
    assert 'fps_explicit = fps_param if fps_param > 0 else 30' in SRC


def test_the_launch_passes_it_through_per_camera():
    """One shared value cannot serve two cameras: measured 68 Hz forward and
    15 Hz downward on the same vehicle."""
    launch = (Path(__file__).resolve().parents[1] / 'launch'
              / 'vision_pi.launch.py').read_text()
    assert "DeclareLaunchArgument('fwd_fps', default_value='60')" in launch
    assert "DeclareLaunchArgument('dwn_fps', default_value='0')" in launch
    assert "'fps':         LaunchConfiguration(fps_arg)" in launch


def test_the_forward_cap_is_a_measured_value_not_a_round_number():
    """It is capped BELOW what the camera can deliver, on purpose, and the
    measurement is in the file. A cap that looks like a downgrade needs its
    evidence beside it or the next person raises it back."""
    launch = (Path(__file__).resolve().parents[1] / 'launch'
              / 'vision_pi.launch.py').read_text()
    i = launch.index("'fwd_fps'")
    ctx = launch[max(0, i - 1400):i]
    assert 'det/s' in ctx and 'uncapped' in ctx
