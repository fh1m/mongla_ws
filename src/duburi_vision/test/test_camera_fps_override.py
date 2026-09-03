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
    ctx = SRC[i:i + 900]
    assert 'fps override' in ctx
    assert "get_profile(profile_name).get('fps')" in ctx
    # ...at WARN. The launch files pin the process default to `warn`, so an
    # info line is invisible -- which is how round 26's silent cap survived a
    # whole round.
    assert 'get_logger().warning(' in ctx


def test_the_non_profile_branch_still_never_asks_a_driver_for_zero():
    """The sentinel must not leak into a device call."""
    assert 'fps_explicit = fps_param if fps_param > 0 else 30' in SRC


def test_the_launch_passes_it_through_per_camera():
    """One shared value cannot serve two cameras: measured 68 Hz forward and
    15 Hz downward on the same vehicle."""
    launch = (Path(__file__).resolve().parents[1] / 'launch'
              / 'vision_pi.launch.py').read_text()
    assert "DeclareLaunchArgument('fwd_fps', default_value='0')" in launch
    assert "DeclareLaunchArgument('dwn_fps', default_value='0')" in launch
    # The cameras moved INTO the detector process (round 32), so the value now
    # reaches them as a `<cam>_cam_` prefixed parameter on the composed
    # launcher rather than through a per-process `camera()` factory. The
    # requirement is unchanged and is what this asserts: each camera gets its
    # OWN value, all the way down.
    assert "'fwd_cam_fps':             LaunchConfiguration('fwd_fps')" in launch
    assert "'dwn_cam_fps':             LaunchConfiguration('dwn_fps')" in launch


def test_the_reversal_of_the_cap_carries_BOTH_measurements():
    """The cap was measured correctly and is now wrong, so the file has to hold
    the old evidence AND the new -- otherwise the next person reads a bare 0,
    finds the +53 % det/s result in git history, and caps it again.

    This test was inverted rather than deleted: it used to assert the cap's
    evidence was present, and it is still the right assertion, now over both
    sides of the reversal."""
    launch = (Path(__file__).resolve().parents[1] / 'launch'
              / 'vision_pi.launch.py').read_text()
    i = launch.index("'fwd_fps'")
    ctx = launch[max(0, i - 2600):i]
    assert 'det/s' in ctx, 'the cap\'s own throughput evidence is gone'
    assert 'age' in ctx and 'CPU' in ctx, 'the latency evidence that reversed it is gone'
    assert 'YUYV' in ctx, 'why MJPEG survived the review is not stated'
