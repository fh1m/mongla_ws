"""Lichtblick's Teleop pad drives through the helm, one short step per press."""
import json

from mongla_batighor.helm import TELEOP_TOPIC, teleop_step
from mongla_batighor import views


def test_each_key_is_one_short_step():
    assert teleop_step(1.0, 0.0)[0] == 'move_forward'
    assert teleop_step(-1.0, 0.0)[0] == 'move_back'
    assert teleop_step(0.0, 1.0) == ('yaw_left', json.dumps({'target': 15.0}))
    assert teleop_step(0.0, -1.0)[0] == 'yaw_right'
    assert teleop_step(0.0, 0.0) is None
    assert json.loads(teleop_step(1.0, 0.0)[1])['duration'] <= 1.0


def test_the_pad_and_the_helm_name_one_topic():
    assert views.TELEOP_TOPIC == TELEOP_TOPIC
    kind, cfg = views.teleop()
    assert kind == 'Teleop' and cfg['topic'] == TELEOP_TOPIC


def test_the_buttons_only_send_verbs_open_to_everyone():
    from mongla_batighor.seat import ANYONE
    for cmd in ('stop', 'surface', 'disarm'):
        kind, cfg = views.call(cmd, '#000000', '')
        assert kind == 'CallService' and json.loads(cfg['requestPayload'])['cmd'] in ANYONE
