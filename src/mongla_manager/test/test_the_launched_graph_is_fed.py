"""The vehicle's launch file, launched: every subscription is fed, or says who feeds it.

Most topic names are built at run time (`f'{ns}/velocity'`), so no static scan
can tell a subscription with a publisher from one listening to nothing. This
launches the real `bringup.launch.py` against `fake_srot_board` and reads the
live graph. A subscription with no publisher must be in `FED_ELSEWHERE`, which
names the source file that publishes it and the switch or mission that starts
it; the test checks that file really does, so the table cannot go stale.

Vision is off here: CI has no cameras and no Hailo. The rows below are
exactly the edges that vision, flow and a running mission add.
"""
from __future__ import annotations

import json
import os
import random
import signal
import socket
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
import fake_srot_board  # noqa: E402

pytest.importorskip('rclpy')

# topic -> (source that publishes it, what starts that publisher)
FED_ELSEWHERE = {
    '/mongla/localization/fix': (
        'src/mongla_planner/mongla_planner/dsl_position.py',
        "a mission's fix_from_prop or place, and the lock when vision is up"),
    '/mongla/localization/heading': (
        'src/mongla_planner/mongla_planner/dsl_heading.py',
        "a mission's anchor_on"),
    '/mongla/mission_alive': (
        'src/mongla_planner/mongla_planner/mission.py',
        'a running mission; its silence is how the manager knows one died'),
    '/mongla/vision/downward/velocity': (
        'src/mongla_vision/mongla_vision/flow/flow_node.py', 'flow:=true'),
    '/mongla/vision/downward/floor_grid_deg': (
        'src/mongla_vision/mongla_vision/flow/flow_node.py',
        'flow:=true with tile_m set'),
    '/mongla/vision/downward/lane_heading_deg': (
        'src/mongla_vision/mongla_vision/flow/flow_node.py',
        'flow:=true with lane_lines:=true'),
}

NODES = {'/mongla_manager', '/mongla_localization'}
_INTERNAL = ('/parameter_events', '/rosout')

_PROBE = textwrap.dedent('''
    import json, sys, time, rclpy
    rclpy.init()
    n = rclpy.create_node('graph_probe')
    want = set(json.loads(sys.argv[1]))

    def snapshot():
        out = {}
        for name, ns in n.get_node_names_and_namespaces():
            full = ns.rstrip('/') + '/' + name
            if full in want:
                subs = n.get_subscriber_names_and_types_by_node(name, ns)
                out[full] = {t: len(n.get_publishers_info_by_topic(t)) for t, _ in subs}
        return out

    # A node is listed before it has made its endpoints, and the manager makes
    # most of its own only after its constructor has read the board (payload
    # roles, one parameter at a time). So: wait until it publishes its state,
    # then read the graph once it has held still for 3 s.
    def up():
        return n.count_publishers('/mongla/state') > 0

    deadline, last, still = time.monotonic() + 120.0, None, 0
    while time.monotonic() < deadline:
        now = snapshot()
        still = still + 1 if (now == last and want <= set(now) and up()) else 0
        last = now
        if still >= 6:
            break
        time.sleep(0.5)
    print(json.dumps(last))
''')


@pytest.fixture(scope='module')
def graph(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('graph')
    deck = tmp / 'bringup.yaml'
    deck.write_text('{}\n')
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    board = fake_srot_board.FakeSrotBoard(port).start()
    # Own domain, loopback discovery: works in containers without multicast,
    # and keeps this graph apart from anything else on the machine.
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(100, 200)),
               ROS_LOCALHOST_ONLY='1', MONGLA_BRINGUP=str(deck),
               HOME=str(tmp))
    log = open(tmp / 'launch.log', 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'mongla_manager', 'bringup.launch.py',
         f'mav_device:=udpin:127.0.0.1:{port}', 'vision:=false'],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        r = subprocess.run([sys.executable, '-c', _PROBE, json.dumps(sorted(NODES))],
                           env=env, capture_output=True, text=True, timeout=150)
        assert r.returncode == 0, r.stderr[-2000:]
        yield json.loads(r.stdout.strip().splitlines()[-1]), tmp / 'launch.log'
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
        log.close()
        board.close()


def _subs(graph):
    return {t: n for node in graph[0].values() for t, n in node.items()
            if not t.startswith(_INTERNAL) and '/_action/' not in t
            and not t.endswith(('/describe_parameters', '/get_parameters',
                                '/get_parameter_types', '/list_parameters',
                                '/set_parameters', '/set_parameters_atomically'))}


def test_both_vehicle_nodes_came_up(graph):
    assert NODES <= set(graph[0]), (
        f'launched {sorted(graph[0])}; log: {graph[1].read_text()[-2000:]}')


def test_every_subscription_is_fed_or_says_who_feeds_it(graph):
    unfed = sorted(t for t, n in _subs(graph).items()
                   if n == 0 and t not in FED_ELSEWHERE)
    assert not unfed, (
        f'subscribed with no publisher, and no row saying who publishes it: {unfed}')


def test_every_row_is_still_a_real_subscription(graph):
    stale = sorted(set(FED_ELSEWHERE) - set(_subs(graph)))
    assert not stale, f'rows for topics nothing here subscribes to any more: {stale}'


@pytest.mark.parametrize('topic', sorted(FED_ELSEWHERE))
def test_the_named_source_really_publishes_it(topic):
    path = FED_ELSEWHERE[topic][0]
    text = (ROOT / path).read_text(encoding='utf-8')
    leaf = topic.rsplit('/', 1)[1]
    assert 'create_publisher' in text and (
        f"/{leaf}'" in text or f'/{leaf}"' in text), (
        f'{path} does not publish anything ending in /{leaf}')
