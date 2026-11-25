"""A fit made on the vehicle goes to the deck, is listed with the shipped ones,
and is what the camera binds first (bead edm)."""
import json

from mongla_vision.calibration import binding, guide_files


def _cal(path, applies_to, fx=500.0):
    path.write_text(json.dumps({
        'applies_to': [applies_to], 'medium': 'air',
        'image_width': 1280, 'image_height': 720,
        'camera_matrix': [[fx, 0, 640], [0, fx, 360], [0, 0, 1]]}))


def test_the_deck_is_the_default_store(tmp_path, monkeypatch):
    monkeypatch.setattr(guide_files, 'DECK_CALIBRATION', str(tmp_path / 'cal'))
    assert guide_files.deck_calibration_dir() == str(tmp_path / 'cal')


def test_the_deck_wins_over_a_shipped_file_of_the_same_name(tmp_path, monkeypatch):
    deck, shipped = tmp_path / 'deck', tmp_path / 'shipped'
    deck.mkdir(); shipped.mkdir()
    _cal(deck / 'pi_x_1280x720.json', 'pi_x', fx=600.0)
    _cal(shipped / 'pi_x_1280x720.json', 'pi_x', fx=500.0)
    _cal(shipped / 'pi_y_1280x720.json', 'pi_y')
    monkeypatch.setattr(guide_files, 'default_calibration_dir', lambda: str(shipped))
    lib = {c['file']: c for c in guide_files.library(str(deck))}
    assert lib['pi_x_1280x720.json']['fx'] == 600.0
    assert lib['pi_x_1280x720.json']['where'] == 'deck'
    assert lib['pi_y_1280x720.json']['where'] == 'shipped'


def test_the_camera_binds_the_deck_first(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    deck = tmp_path / 'mongla' / 'deck' / 'calibration'
    deck.mkdir(parents=True)
    _cal(deck / 'pi_zz_1280x720.json', 'pi_zz')
    assert binding.calibration_for_profile('pi_zz') == str(deck / 'pi_zz_1280x720.json')
