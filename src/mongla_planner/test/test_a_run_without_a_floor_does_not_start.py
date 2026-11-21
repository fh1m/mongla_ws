"""A mission does not start while flow has no floor; a missing depth floor is said every time.

`preflight.height_decision` is what the runner asks before a mission moves.
Without a floor, flow refuses every interval and the run navigates on the IMU
alone while looking healthy, so the runner refuses unless the operator says
`--allow-no-height` (a bench run). The depth floor stays a warning, printed at
every start until it is set.
"""
import types

from mongla_planner import preflight


def test_no_floor_refuses_the_run():
    ok, msg = preflight.height_decision('none', allow_no_height=False)
    assert not ok and msg.startswith('ERROR') and 'course' in msg


def test_the_operator_can_say_it_is_a_bench_run():
    ok, msg = preflight.height_decision('none', allow_no_height=True)
    assert ok and msg.startswith('WARNING')


def test_every_real_source_starts():
    for src in ('tiles', 'survey', 'survey, no position', 'pool_depth'):
        ok, msg = preflight.height_decision(src, allow_no_height=False)
        assert ok and msg.startswith('ok'), src


def test_no_flow_node_warns_but_starts():
    ok, msg = preflight.height_decision(None, allow_no_height=False)
    assert ok and 'no velocity' in msg


def test_an_unset_depth_floor_is_said_and_a_set_one_is_not():
    assert preflight.shield_warning(0.0).startswith('WARNING')
    assert preflight.shield_warning(None).startswith('WARNING')
    assert preflight.shield_warning('x').startswith('WARNING')
    assert preflight.shield_warning(2.5) == ''


def test_the_mission_course_beats_the_deck(monkeypatch, tmp_path):
    deck = tmp_path / 'bringup.yaml'
    deck.write_text('course: deck_pool\n')
    monkeypatch.setenv('MONGLA_BRINGUP', str(deck))
    mod = types.ModuleType('fake_mission')
    mod.COURSE = 'mission_pool'
    import sys
    sys.modules['fake_mission'] = mod

    def run(m, log):
        pass
    run.__module__ = 'fake_mission'
    assert preflight.course_for(run) == 'mission_pool'
    mod.COURSE = None
    assert preflight.course_for(run) == 'deck_pool'
