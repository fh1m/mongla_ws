"""The layouts are what views.py generates: one table, so a new sensor or camera
is one row there, and no hand-made layout drifts from it."""
import json

from mongla_batighor import views
from mongla_batighor.seat import CAMERAS


def test_every_committed_layout_is_what_the_table_generates():
    for name in views.VIEWS:
        path = views.LAYOUTS / f'{views.FILE.get(name, name)}.json'
        assert path.read_text() == views.render(name), \
            f'{path.name} is stale: ros2 run mongla_batighor views --write'


def test_no_layout_is_made_by_hand():
    made = {views.FILE.get(n, n) for n in views.VIEWS}
    assert {p.stem for p in views.LAYOUTS.glob('*.json')} == made


def test_a_split_is_weighted_as_written():
    tree = views.row((1, 'a'), (1, 'b'), (2, 'c'))
    assert tree['splitPercentage'] == 25.0
    assert tree['second']['splitPercentage'] == 33.3
    assert views.col('only') == 'only'


def test_a_new_camera_reaches_every_per_camera_view(monkeypatch):
    monkeypatch.setattr(views, 'CAMERAS', (*CAMERAS, 'aft'))
    for name in ('cockpit', 'vision', 'weak_link'):
        text = json.dumps(views.build(name))
        assert '/mongla/vision/aft/' in text, name


def test_every_panel_in_a_view_is_placed_once():
    for name in views.VIEWS:
        built = views.build(name)
        tree = json.dumps(built['layout'])
        for pid in built['configById']:
            assert tree.count(f'"{pid}"') == 1, (name, pid)


def test_the_tool_checks_and_writes_the_tree_it_is_pointed_at(tmp_path, capsys):
    assert views.main(['--dir', str(tmp_path / 'nowhere')]) == 2
    assert views.main(['--dir', str(tmp_path)]) == 1          # nothing there yet: all stale
    assert views.main(['--dir', str(tmp_path), '--write']) == 0
    assert views.main(['--dir', str(tmp_path)]) == 0
    assert (tmp_path / 'mongla_layout.json').read_text() == views.render('cockpit')
