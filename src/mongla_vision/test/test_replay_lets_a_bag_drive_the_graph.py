"""Replay mode must invert BOTH halves of the camera coupling.

Recorded sessions are the only way to regression-test the vision stack without
a live camera and a person standing in front of it. That only works if a bag
can drive the real graph, and two things in `detector_dual_node` prevent it by
design:

  1. `direct_feed=True` makes the detector REFUSE its image_raw subscription,
     because normally the camera hands it frames in-process and subscribing
     too would infer every picture twice.
  2. a camera that fails to open takes its detector down with it, so a
     camera-less process has nothing left subscribed for the bag to play into.

Both are correct for a vehicle and fatal for a replay, so both are inverted
under `replay`. If either regresses, `ros2 bag play` feeds a graph that is
listening to nothing -- which looks exactly like a recording with no
detections in it, and would be read as a detector bug.
"""
import os
import re

import pytest

SRC = os.path.join(os.path.dirname(__file__), '..', 'mongla_vision',
                   'detector_dual_node.py')
LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch',
                      'vision_pi.launch.py')


@pytest.fixture(scope='module')
def dual_src():
    with open(SRC) as fh:
        return fh.read()


def test_replay_parameter_is_declared(dual_src):
    assert "declare_parameter('replay'" in dual_src, (
        'the replay switch is gone -- a bag can no longer drive the graph')


def test_direct_feed_is_off_under_replay(dual_src):
    assert "direct = not bool(self.get_parameter('replay').value)" in dual_src
    assert "Parameter('direct_feed', value=direct)" in dual_src, (
        'direct_feed is hardcoded again; under replay the detector will not '
        'subscribe and every recorded frame is dropped in silence')


def test_replay_keeps_the_detector_and_builds_no_camera(dual_src):
    # The replay branch must come BEFORE the CameraNode try/except, or the
    # camera is constructed anyway and raises on a device that is not there.
    replay_at = dual_src.index("launcher.get_parameter('replay')")
    camera_at = dual_src.index('cam_node = CameraNode(')
    assert replay_at < camera_at, (
        'the replay branch moved after CameraNode -- replay will now try to '
        'open a camera that a recorded session does not need')
    branch = dual_src[replay_at:camera_at]
    assert 'nodes.append(det)' in branch, (
        'replay drops the detector; nothing will consume the played frames')
    assert 'live.append(' in branch, (
        'replay does not register the camera as live, so the launcher will '
        'exit(1) believing no camera came up')


def test_launch_exposes_replay_and_passes_it_down():
    with open(LAUNCH) as fh:
        src = fh.read()
    assert re.search(r"DeclareLaunchArgument\(\s*'replay'", src), (
        'replay is no longer a launch argument')
    assert "'replay':         LaunchConfiguration('replay')" in src, (
        'replay is declared but never reaches the node -- the argument would '
        'be accepted and silently ignored, the worst of both')
