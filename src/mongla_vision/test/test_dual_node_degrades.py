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


def _run(camera_factory, replay=False, lock=False, cameras='forward,downward',
         flow=False, flow_cls=None):
    """Drive main() with fake nodes; return (launcher, built camera names).

    `replay` must be set explicitly. A bare MagicMock returns a truthy
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
        value={'replay': replay, 'lock': lock, 'flow': flow,
               'cameras': cameras}.get(name, MagicMock()))
    flow_cls = flow_cls or MagicMock()
    with patch('mongla_vision.flow.flow_node.FlowVelocityNode', flow_cls), \
         patch.object(dn, 'rclpy'), \
         patch.object(dn, '_Launcher', return_value=launcher), \
         patch.object(dn, 'DetectorNode', MagicMock()), \
         patch.object(dn, 'CameraNode', side_effect=_camera), \
         patch('mongla_runtime.spin.make_executor',
               return_value=(MagicMock(), 'fake')):
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


def test_flow_values_reach_the_composed_flow_node_with_the_prefix_stripped():
    """`flow_*` values reach flow_node under its own names, on the downward
    camera, fed by reference unless replaying; nothing else is forwarded."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    started = not rclpy.ok()
    if started:
        rclpy.init()
    try:
        orig = dn.Node.__init__

        def patched(self, name, **kw):
            kw['parameter_overrides'] = [
                Parameter('flow', value=True),
                Parameter('flow_pool_depth_m', value=1.6),
                Parameter('flow_medium', value='water'),
                Parameter('lock_act_conf', value=0.6)]
            orig(self, name, **kw)
        with patch.object(dn.Node, '__init__', patched):
            launcher = dn._Launcher()
        live = {p.name: p.value for p in launcher.flow_overrides(True)}
        replay = {p.name: p.value for p in launcher.flow_overrides(False)}
        launcher.destroy_node()
    finally:
        if started:
            rclpy.shutdown()
    assert live == {'camera': 'downward', 'direct_feed': True,
                    'pool_depth_m': 1.6, 'medium': 'water'}
    assert replay['direct_feed'] is False


def test_flow_rides_the_downward_camera_and_dies_with_it():
    """Flow is built beside the downward camera and fed through its tee; a
    downward camera that does not open takes flow down with it rather than
    leaving a node waiting for frames that will never come."""
    flow_cls = MagicMock()
    _run(lambda role: MagicMock(), flow=True, flow_cls=flow_cls)
    assert flow_cls.call_count == 1, 'flow is built once, for downward only'

    flow_cls = MagicMock()

    def no_downward(role):
        if role == 'downward':
            raise RuntimeError('no downward device')
        return MagicMock()
    _run(no_downward, flow=True, flow_cls=flow_cls)
    flow_cls.return_value.destroy_node.assert_called_once()


def test_the_tee_hands_every_frame_to_every_consumer():
    """The detector is offered each frame as a spare; the lock and flow take
    every one, and intrinsics go to whoever reads them."""
    det, lock, flow = MagicMock(), MagicMock(), MagicMock(spec=['submit_frame'])
    tee = dn._Tee(det, lock, flow)
    assert tee.wants_frame()
    tee.submit_frame('f', 'h')
    tee.submit_info('i')
    det.submit_frame.assert_called_once_with('f', 'h', spare=True)
    lock.submit_frame.assert_called_once_with('f', 'h')
    flow.submit_frame.assert_called_once_with('f', 'h')
    lock.submit_info.assert_called_once_with('i')


def test_a_process_per_camera_builds_only_its_own_camera():
    """`cameras` selects what this process holds, so the launch can run one
    process per camera through the HailoRT service."""
    launcher, built, code = _run(lambda role: MagicMock(), cameras='downward')
    assert built == ['downward'], built
    assert code is None


def test_a_per_camera_lock_override_beats_the_generic_one():
    """`fwd_lock_X` sorts before `lock_X`, and rclpy keeps the last override
    of a name: built in sorted order, the generic value won (second-pass
    audit F13). A real node reads what the composed lock would."""
    import rclpy
    from types import SimpleNamespace
    from rclpy.node import Node
    from mongla_vision.detector_dual_node import _Launcher
    vals = {'lock_act_conf': 0.30, 'fwd_lock_act_conf': 0.60, 'dwn_lock_act_conf': 0.90}
    fake = SimpleNamespace(_parameters=vals,
                           get_parameter=lambda n: SimpleNamespace(value=vals[n]))
    started = not rclpy.ok()
    if started:
        rclpy.init()
    try:
        n = Node('t_lock_overrides', parameter_overrides=_Launcher.lock_overrides(fake, 'fwd', 'forward'))
        assert n.declare_parameter('act_conf', 0.45).value == 0.60
        n.destroy_node()
    finally:
        if started:
            rclpy.shutdown()
