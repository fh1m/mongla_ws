"""course_survey writes MEASURED positions to the deck copy, and the loader uses it."""
import pytest
import yaml

from duburi_localization import course_map as cm
from duburi_localization.course_survey import survey, main


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
