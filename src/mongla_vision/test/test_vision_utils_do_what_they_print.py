"""Bench tools run what they print, end, and say when they fail (audit vl_16 02, 03, 07)."""
from types import SimpleNamespace

from mongla_control.commands import fields_for
from mongla_interfaces.action import Move
from mongla_vision.utils import check_thrust, display_keys


def test_a_zero_gain_typed_for_the_thrust_check_runs_as_zero():
    src = open(check_thrust.__file__).read()
    assert 'goal.set_fields' in src
    goal = Move.Goal(cmd='vision_align', gain=0.0, duration=5.0)
    goal.set_fields = ['gain', 'duration']
    assert fields_for('vision_align', goal)['gain'] == 0.0


def test_the_thrust_check_never_waits_without_a_deadline():
    src = open(check_thrust.__file__).read()
    assert 'spin_until_future_complete(node, result_future)\n' not in src
    assert 'spin_until_future_complete(node, send_future)\n' not in src


def test_an_unsent_pause_leaves_the_state_alone():
    said = []
    calls = []
    client = SimpleNamespace(service_is_ready=lambda: False,
                             call_async=lambda r: calls.append(r))
    node = SimpleNamespace(_is_paused=False, _pause_client=client,
                           get_logger=lambda: SimpleNamespace(warn=said.append))
    display_keys._send_pause(node, True)
    assert node._is_paused is False and not calls and said
    client.service_is_ready = lambda: True
    display_keys._send_pause(node, True)
    assert node._is_paused is True and len(calls) == 1
