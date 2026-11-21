"""The mission's text and the line that sent each verb reach every operator.

Batighor shows the plan beside the map with the running line marked
(mission.MissionText). Through a real node and the real client's `send`: a
hook that is never called, a line from the wrong file, or a topic that
drifted fails here.
"""
import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from mongla_planner.client import MonglaClient          # noqa: E402
from mongla_planner.mission import MissionText          # noqa: E402


def _helper(client):
    client.send('stop')                                  # sent from another function


def _mission(client):
    client.send('stop')                                  # LINE_A
    _helper(client)


def _line_of(tag):
    with open(__file__) as f:
        return next(i for i, l in enumerate(f, 1) if l.rstrip().endswith(tag))


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node('mission_text_test')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _listen(node, topic):
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    got = []
    node.create_subscription(String, topic, lambda m: got.append(json.loads(m.data)),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    return got


def _spin(node):
    t = time.monotonic()
    while time.monotonic() - t < 0.5:
        rclpy.spin_once(node, timeout_sec=0.05)


def test_each_verb_says_which_line_of_the_mission_sent_it(node, monkeypatch):
    lines = _listen(node, '/mongla/mission/line')
    client = MonglaClient(node)
    # No action server here: stop at the goal, after the hook has run.
    monkeypatch.setattr(client._client, 'send_goal_async',
                        lambda *a, **k: (_ for _ in ()).throw(RuntimeError('no server')))
    client.on_send = MissionText(node, 'probe', _mission).on_send
    with pytest.raises(RuntimeError):
        _mission(client)                                 # stops at its first verb
    with pytest.raises(RuntimeError):
        _helper(client)
    _spin(node)
    # Each verb reads as the innermost line of this file that sent it.
    assert [g['line'] for g in lines] == [_line_of('# LINE_A'), _line_of('# sent from another function')]
    assert all(g['cmd'] == 'stop' for g in lines)


def test_the_whole_file_is_published_once_latched(node):
    MissionText(node, 'probe', _mission)
    got = _listen(node, '/mongla/mission/source')       # subscribed AFTER: latched
    _spin(node)
    assert len(got) == 1
    src = got[0]
    assert src['name'] == 'probe' and src['file'] == __file__
    assert 'def _helper' in src['text']                  # helpers in the file too
    assert src['first_line'] == _line_of('def _mission(client):')


def test_a_frame_outside_the_mission_file_reports_no_line(node):
    t = MissionText(node, 'probe', _mission)
    t.file = '/nowhere.py'
    assert t.where() == 0


def test_the_runner_wires_the_hook_for_every_mission(monkeypatch):
    """`main()` itself, not a test, connects the client to MissionText: a
    runner that stopped calling it would publish no line and pass the rest."""
    from mongla_planner import mission as runner
    seen = {}

    class _Client:
        def __init__(self, node):
            self.node, self.on_send = node, None

        def wait_for_connection(self, timeout=15.0):
            pass

    class _Mongla:
        def __init__(self, client, log):
            self.client = client

        def log_scoreboard(self, **k):
            pass

    def probe(mongla, log):
        seen['hook'] = mongla.client.on_send
    monkeypatch.setattr(runner, 'discover', lambda: {'probe': probe})
    monkeypatch.setattr(runner, 'MonglaClient', _Client)
    monkeypatch.setattr(runner, 'MonglaMission', _Mongla)
    monkeypatch.setattr(runner, '_safe_shutdown', lambda m: None)
    from mongla_planner import preflight
    monkeypatch.setattr(preflight, 'run', lambda *a, **k: None)
    with pytest.raises(SystemExit) as e:
        runner.main(['probe'])
    assert e.value.code == 0
    hook = seen['hook']
    assert getattr(hook, '__self__', None).__class__ is MissionText
    assert hook.__self__.file == __file__
