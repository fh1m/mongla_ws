"""course_survey writes MEASURED positions to the deck copy, and the loader uses it."""
import pytest

from mongla_localization import course_map as cm
from mongla_localization.course_survey import survey, main


def test_the_packaged_sauvc_course_is_a_template_with_nothing_invented():
    c = cm.load_course('sauvc26')
    assert 'final_gate' in c.props and 'orange_flare' in c.props
    assert all(not p.has_position and not p.measured for p in c.props.values())
    assert c.props['flare_red'].detect_class == 'flare_red'


def test_a_survey_writes_the_deck_copy_which_then_wins(tmp_path, monkeypatch):
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    out = survey('sauvc26', 'final_gate', {'x': 16.0, 'y': -0.4, 'bearing': 0.0})
    assert out.parent == tmp_path
    c = cm.load_course('sauvc26')
    assert c.source == str(out)
    assert c.position_of('final_gate') == (16.0, -0.4)
    assert c.bearing_of('final_gate') == 0.0 and c.props['final_gate'].measured
    assert not c.props['orange_flare'].measured          # untouched props stay nominal
    survey('sauvc26', 'orange_flare', {'x': 6.0, 'y': 1.0})
    c = cm.load_course('sauvc26')
    assert c.position_of('final_gate') == (16.0, -0.4)   # earlier survey kept
    assert c.position_of('orange_flare') == (6.0, 1.0)


def test_the_package_copy_is_never_written(tmp_path, monkeypatch):
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    before = (cm._PKG_COURSES / 'sauvc26.yaml').read_text()
    survey('sauvc26', 'final_gate', {'x': 1.0, 'y': 2.0})
    assert (cm._PKG_COURSES / 'sauvc26.yaml').read_text() == before


def test_an_unknown_prop_is_refused_without_a_class(tmp_path, monkeypatch):
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    with pytest.raises(KeyError):
        survey('sauvc26', 'mystery', {'x': 1.0, 'y': 1.0})
    survey('sauvc26', 'mystery', {'x': 1.0, 'y': 1.0}, detect_class='mystery')
    assert cm.load_course('sauvc26').position_of('mystery') == (1.0, 1.0)


def test_a_positive_depth_is_refused_by_the_cli(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    assert main(['--course', 'sauvc26', '--prop', 'flare_red', '--x', '9', '--y', '2',
                 '--depth', '0.8']) == 2
    assert not (tmp_path / 'sauvc26.yaml').exists()


def test_the_survey_writes_the_venue_numbers(tmp_path, monkeypatch):
    """Floor soundings, tile, depth floor and a prop width: all in the one course file."""
    from mongla_localization import course_survey as cs
    from mongla_localization.course_map import load_course
    monkeypatch.setenv('MONGLA_COURSE_DIR', str(tmp_path))
    assert cs.main(['--course', 'sauvc26', '--floor', '0', '0', '1.28',
                    '--floor', '17.5', '0', '2.0', '--shield', '1.5', '--tile', '0.25']) == 0
    assert cs.main(['--course', 'sauvc26', '--prop', 'final_gate', '--width', '1.5']) == 0
    c = load_course('sauvc26')
    assert c.pool_floor() == [(0.0, 0.0, 1.28), (17.5, 0.0, 2.0)]
    assert c.shield_max_depth_m() == 1.5 and c.pool['tile_m'] == 0.25
    assert c.widths_by_class()['final_gate'] == 1.5
    assert cs.main(['--course', 'sauvc26', '--floor', '1', '1', '-1.2']) == 2


def test_the_survey_writes_the_pool_outline_and_refuses_a_line(tmp_path, monkeypatch):
    """`--corner` (Batighor's map): three corners or more, written in order, read back."""
    from mongla_localization import course_survey as cs
    from mongla_localization.course_map import load_course
    monkeypatch.setenv('MONGLA_COURSE_DIR', str(tmp_path))
    assert load_course('sauvc26').pool_outline() == []
    corners = ['--corner', '0', '-8', '--corner', '0', '8', '--corner', '25', '8',
               '--corner', '25', '-8']
    assert cs.main(['--course', 'sauvc26', *corners]) == 0
    assert load_course('sauvc26').pool_outline() == [(0, -8), (0, 8), (25, 8), (25, -8)]
    assert cs.main(['--course', 'sauvc26', '--corner', '0', '0', '--corner', '1', '1']) == 2
    bad = tmp_path / 'sauvc26.yaml'
    bad.write_text(bad.read_text().replace('- - 25.0\n    - 8.0\n', '- - 25.0\n'))
    with pytest.raises(ValueError, match='outline'):
        load_course('sauvc26')
