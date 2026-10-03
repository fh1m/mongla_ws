"""One absent camera must not take the working one down.

`CameraNode` raises after five open retries, and that raise used to walk
straight out of the build loop: an unplugged downward USB killed the FORWARD
detector too and the process exited 1. Measured on the vehicle after a reboot
dropped `/dev/mongla_cam_downward` -- the whole vision stack refused to start
on a hull whose forward camera was fine.

No ROS: the build loop is driven with fakes, so the test says what the loop
does rather than what a camera does.
"""
from unittest.mock import MagicMock, patch

import pytest

from mongla_vision import detector_dual_node as dn


def _run(camera_factory, replay=False, lock=False):
    """Drive main() with fake nodes; return (launcher, built camera names).

    ⛔ `replay` must be set explicitly. A bare MagicMock returns a truthy
    object for EVERY parameter, so `get_parameter('replay').value` reads as
    True and the loop builds no cameras at all -- the fake silently answers a
    question it was never told about, and every assertion here then measures
    replay mode instead of camera degradation.
    """
    built: list[str] = []

    def _camera(name, **kw):
        role = name.rsplit('_', 1)[-1]
        node = camera_factory(role)          # raises for an absent camera
        built.append(role)
        return node

    launcher = MagicMock()
    launcher.get_logger.return_value = MagicMock()
    launcher.get_parameter.side_effect = lambda name: MagicMock(
        value={'replay': replay, 'lock': lock}.get(name, MagicMock()))
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
            raise RuntimeError("webcam: cv2.VideoCapture('/dev/mongla_cam_downward') failed")
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


def test_replay_builds_no_cameras_and_keeps_every_detector():
    """Replay drives the graph from a recorded bag, so there is no camera to
    build -- but the detectors must survive, or the played frames arrive at a
    process with nothing subscribed and the bag looks empty."""
    def factory(role):
        raise AssertionError('replay must not construct a camera')

    launcher, built, code = _run(factory, replay=True)
    assert built == [], 'replay built a camera it has no use for'
    assert code is None, 'replay exited as though no camera came up'


def test_the_lock_ladder_is_composed_beside_each_live_camera():
    """One process, one Hailo VDevice: the ladder is built here, not as a
    separate lock_node that races the detector for the chip (2026-10-03)."""
    made = {}

    def _lock(name, **kw):
        node = MagicMock()
        made[name] = (node, kw)
        return node

    def factory(role):
        if role == 'forward':
            raise RuntimeError('no forward device')
        return MagicMock()

    with patch('mongla_vision.lock_node.LockNode', side_effect=_lock):
        launcher, built, code = _run(factory, lock=True)
    assert code is None
    # Built BEFORE its camera (so the first frame has both consumers), and
    # torn down again when that camera does not come up: one ladder per LIVE
    # camera, and none left behind for the dead one.
    made['mongla_lock_forward'][0].destroy_node.assert_called_once()
    # Live: frames by reference, so the ladder subscribes to no image topic.
    kw = made['mongla_lock_downward'][1]
    direct = {p.name: p.value for p in kw['parameter_overrides']
              if p.name == 'direct_feed'}
    assert direct == {'direct_feed': True}


def test_only_the_lock_values_the_launch_sets_are_forwarded():
    """Per-camera `fwd_lock_*` beats nothing, shared `lock_*` reaches both,
    and nothing the launch did not set is invented here."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    started = not rclpy.ok()
    if started:
        rclpy.init()
    try:
        orig = dn.Node.__init__

        def patched(self, name, **kw):
            kw['parameter_overrides'] = [
                Parameter('lock', value=True),
                Parameter('lock_act_conf', value=0.6),
                Parameter('fwd_lock_target_class', value='gate'),
                Parameter('dwn_lock_target_class', value='bin')]
            orig(self, name, **kw)
        with patch.object(dn.Node, '__init__', patched):
            launcher = dn._Launcher()
        fwd = {p.name: p.value for p in launcher.lock_overrides('fwd', 'forward')}
        dwn = {p.name: p.value for p in launcher.lock_overrides('dwn', 'downward')}
        launcher.destroy_node()
    finally:
        if started:
            rclpy.shutdown()
    assert fwd == {'camera': 'forward', 'act_conf': 0.6, 'target_class': 'gate'}
    assert dwn == {'camera': 'downward', 'act_conf': 0.6, 'target_class': 'bin'}
