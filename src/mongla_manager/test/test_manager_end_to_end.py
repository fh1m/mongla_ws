"""The manager node, whole, under a MultiThreadedExecutor (issue #22).

Every other manager test calls a method on a stub. Nothing exercised the
part a real run depends on: the action server, the busy gate, cancel, the
abort flag reaching a running move, and the ACK relay from the board -- the
goal / cancel / execute interplay across threads. Here the real node talks
real MAVLink over loopback to `fake_srot_board`, and a real action client
drives it.
"""
from __future__ import annotations

import os
import socket
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fake_srot_board  # noqa: E402,F401  (MAVLink 2 before pymavlink loads)

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mongla_interfaces.action')

from action_msgs.msg import GoalStatus                    # noqa: E402
from mongla_interfaces.action import Move                 # noqa: E402

CMD_SROT_MOVE, MOVE_STOP = 31000, 6


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(scope='module')
def rig():
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    import mongla_manager.auv_manager_node as amn

    port = _free_udp_port()
    board = fake_srot_board.FakeSrotBoard(port).start()
    started = not rclpy.ok()
    if started:
        rclpy.init()
    orig = Node.__init__

    def patched(self, name, **kw):
        kw['parameter_overrides'] = list(kw.get('parameter_overrides', [])) + [
            Parameter('flight_controller', value='srot'),
            Parameter('mav_device', value=f'udpin:127.0.0.1:{port}')]
        orig(self, name, **kw)

    Node.__init__ = patched
    try:
        node = amn.AUVManagerNode()
    finally:
        Node.__init__ = orig
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    client_node = rclpy.create_node('e2e_client')
    ex.add_node(client_node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    ac = ActionClient(client_node, Move, '/mongla/move')
    assert ac.wait_for_server(timeout_sec=10.0), 'action server never came up'
    yield ac, board, node
    ex.shutdown()
    node.destroy_node()
    client_node.destroy_node()
    board.close()
    if started:
        rclpy.shutdown()


def _send(ac, cmd, **fields):
    g = Move.Goal()
    g.cmd = cmd
    for k, v in fields.items():
        setattr(g, k, v)
    fut = ac.send_goal_async(g)
    t = time.monotonic()
    while not fut.done() and time.monotonic() - t < 5.0:
        time.sleep(0.01)
    assert fut.done(), f'{cmd}: no goal response'
    return fut.result()


def _wait(gh, limit=15.0):
    fut = gh.get_result_async()
    t = time.monotonic()
    while not fut.done() and time.monotonic() - t < limit:
        time.sleep(0.01)
    assert fut.done(), 'no result'
    return fut.result()


def _stops(board):
    return sum(1 for m in board.commands(CMD_SROT_MOVE) if int(m.param1) == MOVE_STOP)


@pytest.fixture()
def armed(rig):
    ac, board, _ = rig
    r = _wait(_send(ac, 'arm'))
    assert r.result.success and board.armed
    yield ac, board
    _wait(_send(ac, 'disarm'))


def test_a_move_runs_on_the_board_and_reports_its_ack(armed):
    ac, board = armed
    before = len(board.commands(CMD_SROT_MOVE))
    r = _wait(_send(ac, 'move_forward', duration=1.0, gain=30.0))
    assert r.status == GoalStatus.STATUS_SUCCEEDED and r.result.success
    assert len(board.commands(CMD_SROT_MOVE)) == before + 1


def test_the_busy_gate_refuses_a_second_motion_goal(armed):
    ac, board = armed
    first = _send(ac, 'move_forward', duration=3.0, gain=30.0)
    time.sleep(0.4)
    second = _send(ac, 'move_back', duration=1.0, gain=30.0)
    assert not second.accepted, 'two motion verbs ran at once'
    _wait(_send(ac, 'stop'))
    _wait(first)


def test_stop_bypasses_the_gate_and_brakes_the_running_move(armed):
    """Safety rule 2: `stop` always executes, and the move it interrupts ends
    as not-succeeded with MOVE_STOP on the wire."""
    ac, board = armed
    first = _send(ac, 'move_forward', duration=3.0, gain=30.0)
    time.sleep(0.4)
    stops = _stops(board)
    stop = _send(ac, 'stop')
    assert stop.accepted, 'stop was refused while a move ran'
    assert _wait(stop).result.success
    r = _wait(first)
    assert not r.result.success
    assert _stops(board) > stops


def test_a_cancel_brakes_the_board_and_ends_CANCELED(armed):
    ac, board = armed
    gh = _send(ac, 'move_forward', duration=3.0, gain=30.0)
    time.sleep(0.4)
    stops = _stops(board)
    t = time.monotonic()
    cf = gh.cancel_goal_async()
    while not cf.done():
        time.sleep(0.01)
    r = _wait(gh)
    assert r.status == GoalStatus.STATUS_CANCELED, (
        f'status {r.status}: a cancel must not read as a failed verb')
    assert not r.result.success
    assert _stops(board) > stops, 'the cancel never braked the board'
    assert time.monotonic() - t < 2.5


def test_a_refused_verb_puts_nothing_on_the_wire(armed):
    ac, board = armed
    n = len(board.received)
    r = _wait(_send(ac, 'lock_heading', target=10.0))
    assert not r.result.success and 'not supported' in r.result.message
    cmds = [m for m in board.received[n:] if m.get_type() == 'COMMAND_LONG'
            and m.command == CMD_SROT_MOVE]
    assert cmds == []


def test_a_move_while_disarmed_is_refused_by_the_host(rig):
    ac, board, _ = rig
    assert not board.armed
    r = _wait(_send(ac, 'move_forward', duration=1.0, gain=30.0))
    assert not r.result.success and 'disarmed' in r.result.message.lower()
