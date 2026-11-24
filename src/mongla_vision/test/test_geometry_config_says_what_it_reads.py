"""Tool and target geometry: absent is absent, a frame is checked, the deck is
read (audit vl_3 F1, F5, F7, F8, F9)."""
import math
import pathlib

from mongla_vision import target_geometry as tg
from mongla_vision import tool_geometry as tool

LAUNCH = pathlib.Path(__file__).resolve().parents[1] / 'launch'


def test_a_tool_with_no_numbers_is_not_measured(monkeypatch):
    monkeypatch.setattr(tool, '_load', lambda: {'grabber2': {'camera': 'forward'}})
    assert tool.is_measured('grabber2') is False
    assert tool.offset_m('grabber2') is None
    assert tool.pixel_offset('grabber2', fx=500, fy=500, range_m=2.0) is None


def test_an_offset_is_refused_in_the_other_cameras_image(monkeypatch):
    monkeypatch.setattr(tool, '_load', lambda: {'dropper': {
        'camera': 'downward', 'x': 0.05, 'y': 0.0, 'z': 0.0}})
    assert tool.pixel_offset('dropper', fx=500, fy=500, range_m=1.0,
                             camera='downward') == (25.0, 0.0)
    assert tool.pixel_offset('dropper', fx=500, fy=500, range_m=1.0,
                             camera='forward') is None


def test_an_unknown_width_is_nan_not_zero():
    assert math.isnan(tg.width_for('no_such_prop_anywhere'))
    assert math.isnan(tg.width_for(''))


def test_the_deck_override_is_read(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MONGLA_TARGET_GEOMETRY', raising=False)
    assert str(tmp_path / 'mongla' / 'deck' / 'target_geometry.yaml') in tg._override_paths()


def test_vision_dual_can_be_given_a_calibration():
    src = (LAUNCH / 'vision_dual.launch.py').read_text()
    assert "'calibration': LaunchConfiguration(cal_arg)" in src
