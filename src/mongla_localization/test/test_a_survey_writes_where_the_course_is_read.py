"""`course_survey` writes the copy the loader reads first (audit vl_1 #2)."""
import pathlib

from mongla_localization import course_map as cm
from mongla_localization import course_survey as cs


def test_with_no_env_the_survey_writes_the_deck(monkeypatch, tmp_path):
    monkeypatch.delenv(cm._ENV, raising=False)
    monkeypatch.setattr(pathlib.Path, 'home', lambda: tmp_path)
    assert cs.deck_dir() == tmp_path / 'mongla' / 'deck' / 'courses'
    assert cs.deck_dir() == cm._search_dirs()[0]
