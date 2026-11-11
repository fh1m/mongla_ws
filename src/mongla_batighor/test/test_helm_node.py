"""The helm node, end to end: what an operator's request does at /mongla/move.

A fake manager records every goal that reaches it. Requests go through the
real service, the way foxglove_bridge relays a browser's call, so each test
reads the one thing that matters: which goals the vehicle was sent.
"""
import json
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

import mongla_batighor.seat as seat_rules
from mongla_batighor.helm import LATCHED, HelmNode
from mongla_interfaces.action import Move
from mongla_interfaces.srv import Helm


class FakeManager(Node):
    """Holds a move goal for HOLD_S, as a running verb would; ends the rest at once."""

    HOLD_S = 3.0

    def __init__(self):
        super().__init__('fake_manager')
        self.goals = []
        ActionServer(self, Move, '/mongla/move', self._run)

    def _run(self, handle):
        self.goals.append((handle.request.cmd, list(handle.request.set_fields)))
        if handle.request.cmd.startswith('move_'):
            t0 = time.monotonic()
            while time.monotonic() - t0 < self.HOLD_S:
                time.sleep(0.02)
        handle.succeed()
        return Move.Result(success=True, message='ok')


class Rig(tuple):
    """(call, sent, manager, states, log), with the fake detectors as `.dets`."""

    def __new__(cls, items, dets):
        r = super().__new__(cls, items)
        r.dets = dets
        return r


class FakeDetector(Node):
    """A detector's settings and nothing else, under the real node's name."""

    def __init__(self, camera):
        super().__init__(f'mongla_detector_{camera}')
        for name, value in (('conf', 0.35), ('nms_floor', 0.05), ('nms_iou', 0.0),
                            ('active_model', 'gate_sharks'), ('classes', ''),
                            ('paused', camera != 'forward'),
                            ('models', 'gate=gate_sharks,bins_v2')):
            self.declare_parameter(name, value)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setattr(seat_rules, 'LOST_S', 0.6)
    rclpy.init()
    helm, mgr, me = HelmNode(log_path=tmp_path / 'helm.log'), FakeManager(), Node('browser')
    dets = {c: FakeDetector(c) for c in ('forward', 'downward')}
    ex = MultiThreadedExecutor(num_threads=4)
    for n in (helm, mgr, me, *dets.values()):
        ex.add_node(n)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    cli = me.create_client(Helm, '/mongla/batighor/helm')
    assert cli.wait_for_service(timeout_sec=5.0)
    states = []
    me.create_subscription(String, '/mongla/batighor/helm_state',
                           lambda m: states.append(json.loads(m.data)), LATCHED)
    assert helm.move.wait_for_server(timeout_sec=5.0)
    for c in dets:
        assert helm._set[c].wait_for_service(timeout_sec=5.0)

    def call(who, op, **kw):
        req = Helm.Request(who=who, op=op, **kw)
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.01)
        return fut.result()

    def sent(n=1, timeout=3.0):
        t0 = time.monotonic()
        while len(mgr.goals) < n and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        return [g[0] for g in mgr.goals]

    yield Rig((call, sent, mgr, states, tmp_path / 'helm.log'), dets)
    ex.shutdown()
    for n in (helm, mgr, me, *dets.values()):
        n.destroy_node()
    rclpy.shutdown()


def test_a_watcher_cannot_drive_and_nothing_reaches_the_vehicle(rig):
    call, sent, mgr, _, _ = rig
    r = call('raf', 'command', cmd='move_forward', fields_json='{"duration": 1}')
    assert not r.ok and 'pilot' in r.message
    assert sent(1, timeout=0.5) == []


def test_the_pilots_verb_arrives_with_the_fields_they_typed(rig):
    call, sent, mgr, _, _ = rig
    assert call('ana', 'request').ok
    r = call('ana', 'command', cmd='move_forward', fields_json='{"duration": 1, "gain": 0}')
    assert r.ok, r.message
    assert sent() == ['move_forward']
    assert sorted(mgr.goals[0][1]) == ['duration', 'gain']


def test_stop_reaches_the_vehicle_from_anyone(rig):
    call, sent, _, _, _ = rig
    call('ana', 'request')
    assert call('raf', 'command', cmd='stop').ok
    assert sent() == ['stop']


def test_arm_without_both_ticks_is_refused_before_the_vehicle(rig):
    call, sent, _, _, _ = rig
    call('ana', 'request')
    r = call('ana', 'command', cmd='arm', confirms=['propellers_clear'])
    assert not r.ok and 'kill_switch_manned' in r.message
    assert sent(1, timeout=0.5) == []


def test_a_silent_pilots_running_verb_is_followed_by_stop(rig):
    call, sent, _, states, _ = rig
    call('ana', 'request')
    call('ana', 'command', cmd='move_forward', fields_json='{"duration": 3}')
    # No heartbeat: the helm is lost after LOST_S, while the move still runs.
    assert sent(2, timeout=3.0) == ['move_forward', 'stop']
    # The stop is sent first and the freed helm published after it, so the
    # state may arrive a moment later (seen on jazzy CI).
    assert _until(lambda: states and states[-1]['pilot'] == '')


def test_a_beating_pilot_keeps_the_helm_and_nothing_is_stopped(rig):
    call, sent, _, states, _ = rig
    call('ana', 'request')
    call('ana', 'command', cmd='move_forward', fields_json='{"duration": 1}')
    for _ in range(8):
        call('ana', 'heartbeat')
        time.sleep(0.2)
    assert sent(2, timeout=0.2) == ['move_forward']
    assert states[-1]['pilot'] == 'ana'


def test_every_request_is_in_the_log_with_its_operator(rig):
    call, _, _, _, log = rig
    call('ana', 'request')
    call('raf', 'command', cmd='move_fwd')
    lines = [json.loads(l) for l in log.read_text().splitlines()]
    refused = [l for l in lines if l.get('cmd') == 'move_fwd']
    assert refused and refused[0]['operator'] == 'raf' and not refused[0]['ok']
    assert any('ana is the pilot' in l.get('event', '') for l in lines)


def test_a_running_mission_is_not_stopped_when_the_pilot_goes_quiet(rig):
    """Owner, 2026-11-12: a dropped laptop must not end a run the vehicle is
    flying by itself. The mission's own keep-alive is what says one runs."""
    call, sent, mgr, _, _ = rig
    beat = mgr.create_publisher(String, '/mongla/mission_alive', 10)
    stop_beating = threading.Event()

    def mission():
        while not stop_beating.wait(0.2):
            beat.publish(String(data='qualify'))
    threading.Thread(target=mission, daemon=True).start()
    try:
        call('ana', 'request')
        call('ana', 'command', cmd='move_forward', fields_json='{"duration": 3}')
        assert sent(2, timeout=2.0) == ['move_forward']
    finally:
        stop_beating.set()


def _until(check, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if check():
            return True
        time.sleep(0.05)
    return False


def _param(node, name):
    return node.get_parameter(name).value


def test_any_named_operator_tunes_a_detector_when_no_mission_runs(rig):
    call, _, _, states, _ = rig
    det = rig.dets['forward']
    r = call('raf', 'tune', cmd='forward.conf', fields_json='0.6')
    assert r.ok, r.message
    assert _until(lambda: _param(det, 'conf') == 0.6)
    # and the state every browser reads says so, within the 1 s poll
    assert _until(lambda: states and states[-1].get('tuning', {})
                  .get('forward', {}).get('conf') == 0.6)


def test_the_state_carries_the_model_registry_the_panel_offers(rig):
    """active_model picks from `models`; the panel lists its keys, so the
    helm must pass the registry on, read only."""
    _, _, _, states, _ = rig
    assert _until(lambda: states and states[-1].get('tuning', {})
                  .get('downward', {}).get('models') == 'gate=gate_sharks,bins_v2')


def test_tuning_is_closed_while_a_mission_runs(rig):
    call, _, mgr, _, _ = rig
    beat = mgr.create_publisher(String, '/mongla/mission_alive', 10)
    for _ in range(5):
        beat.publish(String(data='qualify'))
        time.sleep(0.05)
    r = call('raf', 'tune', cmd='forward.conf', fields_json='0.6')
    assert not r.ok and 'qualify' in r.message
    time.sleep(0.3)
    assert _param(rig.dets['forward'], 'conf') == 0.35


def test_a_setting_outside_the_list_or_of_the_wrong_type_is_refused(rig):
    call, _, _, _, _ = rig
    for target, raw in (('forward.paused', 'true'), ('forward.conf', '"high"'),
                        ('side.conf', '0.5'), ('forward.classes', '3')):
        assert not call('raf', 'tune', cmd=target, fields_json=raw).ok, target
    assert _param(rig.dets['forward'], 'paused') is False


def test_the_pilot_switches_the_camera_as_use_camera_does(rig):
    call, _, _, states, _ = rig
    fwd, dwn = rig.dets['forward'], rig.dets['downward']
    assert not call('raf', 'camera', cmd='downward').ok
    call('ana', 'request')
    r = call('ana', 'camera', cmd='downward')
    assert r.ok, r.message
    assert _until(lambda: _param(fwd, 'paused') is True and _param(dwn, 'paused') is False)
    assert _until(lambda: states and states[-1].get('camera') == 'downward')
