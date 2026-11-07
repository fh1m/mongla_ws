"""survey_course: places each prop it can see, never translates, ends surfaced."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from mongla_localization.course_map import Course, Prop
from mongla_planner.missions import survey_course as sc


def _run(monkeypatch, seen=True):
    monkeypatch.setattr(sc, 'COURSE', 't')
    monkeypatch.setattr(sc, 'ANCHOR_PROP', 'gate')
    monkeypatch.setattr(sc, 'MODEL', 'any_model')
    monkeypatch.setattr(sc, 'SURVEY_PROPS', ('bucket', 'flare'))
    m = MagicMock()
    m.use_course.return_value = Course('t', {
        'gate': Prop('gate', bearing_deg=0.0, detect_class='gate', measured=True),
        'bucket': Prop('bucket', detect_class='bucket'),
        'flare': Prop('flare', detect_class='flare')})
    m.vision.align.return_value = SimpleNamespace(saw_target=seen)
    m.place_prop.return_value = SimpleNamespace(ok=True, x_m=1.0, y_m=2.0,
                                                sigma_m=0.3, reason='')
    lines = []
    sc.run(m, lines.append)
    return m, lines


def test_every_visible_prop_is_placed_and_written(monkeypatch):
    m, lines = _run(monkeypatch)
    assert [c.args[0] for c in m.place_prop.call_args_list] == ['bucket', 'flare']
    assert all(c.kwargs['commit'] for c in m.place_prop.call_args_list)
    assert 'placed 2 of 2' in lines[-1]


def test_it_never_translates_and_surface_is_the_last_motion(monkeypatch):
    m, _ = _run(monkeypatch)
    names = [c[0] for c in m.method_calls]
    for verb in ('move_forward', 'move_back', 'move_left', 'move_right', 'vision.move'):
        assert verb not in names
    motion = [n for n in names if n in ('surface', 'stop', 'set_depth', 'turn')]
    assert motion[-1] == 'surface' and names[-1] == 'disarm'


def test_an_unseen_prop_is_skipped_not_placed(monkeypatch):
    m, lines = _run(monkeypatch, seen=False)
    m.place_prop.assert_not_called()
    assert 'placed 0 of 2' in lines[-1]


def test_unfilled_tunables_refuse_before_arming():
    m = MagicMock()
    lines = []
    sc.run(m, lines.append)
    m.arm.assert_not_called()
    assert 'fill' in lines[0]
