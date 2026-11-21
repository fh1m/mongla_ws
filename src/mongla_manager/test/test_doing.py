"""What the vehicle is doing (doing.py): the record Batighor reads on /mongla/doing."""
import json
import math
import types

from mongla_manager.doing import CANCELED, DONE, FAILED, KEEP, REFUSED, RUNNING, Doing


def _req(**f):
    return types.SimpleNamespace(set_fields=list(f), **f)


def _clock():
    t = [100.0]

    def now():
        t[0] += 1.0
        return t[0]
    return now


def test_a_verb_runs_then_ends_with_its_fields_and_result():
    d = Doing(_clock())
    doc = json.loads(d.start('go_to', _req(x_m=3.0, y_m=-1.5), 'g1'))
    assert doc['now']['verb'] == 'go_to' and doc['now']['state'] == RUNNING
    assert doc['now']['fields'] == {'x_m': 3.0, 'y_m': -1.5}
    doc = json.loads(d.end('g1', DONE, 'arrived', 0.2))
    assert doc['now'] is None and doc['running'] == 0
    last = doc['recent'][0]
    assert (last['verb'], last['state'], last['final_value']) == ('go_to', DONE, 0.2)
    assert last['ended'] > last['since']


def test_a_safety_verb_preempting_a_move_is_not_closed_by_the_move_ending():
    d = Doing(_clock())
    d.start('move_forward', _req(duration=3.0), 'move')
    d.start('stop', _req(), 'stop')
    doc = json.loads(d.end('move', FAILED, 'aborted'))
    assert doc['now']['verb'] == 'stop', 'the move ending closed the stop'
    doc = json.loads(d.end('stop', DONE, 'stopped'))
    assert [r['verb'] for r in doc['recent']] == ['stop', 'move_forward']


def test_a_refusal_is_recorded_and_changes_nothing_running():
    d = Doing(_clock())
    d.start('vision_align', _req(camera='downward'), 'a')
    doc = json.loads(d.refused('move_back', _req(duration=1.0), 'another verb is running'))
    assert doc['now']['verb'] == 'vision_align'
    assert doc['recent'][0]['state'] == REFUSED
    assert 'another verb' in doc['recent'][0]['message']


def test_absent_values_are_null_never_a_number():
    d = Doing(_clock())
    doc = json.loads(d.start('set_depth', _req(target=math.nan), 'g'))
    assert doc['now']['fields'] == {'target': None}
    doc = json.loads(d.end('g', CANCELED, '', math.nan))
    assert 'final_value' not in doc['recent'][0]


def test_only_the_last_few_are_kept():
    d = Doing(_clock())
    for i in range(KEEP + 5):
        d.start('stop', _req(), str(i))
        d.end(str(i), DONE)
    assert len(json.loads(d.text())['recent']) == KEEP


def test_a_goal_sent_without_set_fields_shows_what_differs_from_a_fresh_goal():
    import pytest
    Move = pytest.importorskip('mongla_interfaces.action').Move
    g = Move.Goal()
    g.cmd, g.duration, g.camera = 'vision_move', 4.0, 'forward'
    d = Doing(_clock())
    assert json.loads(d.start('vision_move', g, 'g'))['now']['fields'] == {
        'duration': 4.0, 'camera': 'forward'}
