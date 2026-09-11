"""One absent camera must not take the working one down.

`CameraNode` raises after five open retries, and that raise used to walk
straight out of the build loop: an unplugged downward USB killed the FORWARD
detector too and the process exited 1. Measured on the vehicle after a reboot
dropped `/dev/duburi_cam_downward` -- the whole vision stack refused to start
on a hull whose forward camera was fine.

No ROS: the build loop is driven with fakes, so the test says what the loop
does rather than what a camera does.
"""
from unittest.mock import MagicMock, patch

import pytest

from duburi_vision import detector_dual_node as dn


def _run(camera_factory):
    """Drive main() with fake nodes; return (launcher, built camera names)."""
    built: list[str] = []

    def _camera(name, **kw):
        role = name.rsplit('_', 1)[-1]
        node = camera_factory(role)          # raises for an absent camera
        built.append(role)
        return node

    launcher = MagicMock()
    launcher.get_logger.return_value = MagicMock()
    with patch.object(dn, 'rclpy'), \
         patch.object(dn, '_Launcher', return_value=launcher), \
         patch.object(dn, 'DetectorNode', MagicMock()), \
         patch.object(dn, 'CameraNode', side_effect=_camera), \
         patch.object(dn, 'MultiThreadedExecutor') as ex:
        try:
            dn.main()
        except SystemExit as exc:
            return launcher, built, exc.code
    return launcher, built, None


def test_a_missing_downward_camera_leaves_forward_running():
    def factory(role):
        if role == 'downward':
            raise RuntimeError("webcam: cv2.VideoCapture('/dev/duburi_cam_downward') failed")
        return MagicMock()

    launcher, built, code = _run(factory)
    assert built == ['forward'], 'forward must still come up'
    assert code is None, 'a degraded stack must not exit'


def test_a_missing_forward_camera_leaves_downward_running():
    # Order matters: forward is built FIRST, so its failure must not abort
    # the loop before downward is even attempted.
    def factory(role):
        if role == 'forward':
            raise RuntimeError('no forward device')
        return MagicMock()

    launcher, built, code = _run(factory)
    assert built == ['downward']
    assert code is None


def test_no_camera_at_all_exits_rather_than_looking_healthy():
    # Degrading to zero eyes silently is how a dead stack passes a node check.
    def factory(role):
        raise RuntimeError('no devices')

    launcher, built, code = _run(factory)
    assert built == []
    assert code == 1


def test_both_cameras_present_builds_both():
    launcher, built, code = _run(lambda role: MagicMock())
    assert built == ['forward', 'downward']
    assert code is None
