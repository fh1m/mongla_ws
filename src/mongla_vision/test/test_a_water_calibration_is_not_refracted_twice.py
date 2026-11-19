"""A calibration fitted with the board in WATER is not published as the lens.

The camera publishes K on CameraInfo, and every consumer (flow, the lock,
pnp_node, the DSL's bearings) treats it as the in-AIR lens and applies the
flat-port refraction itself (`optics.rectifier_for`). `solver.py
--medium water` writes the in-water K, which already contains the port, so
publishing it would refract twice: every range off by a factor of n. Both the
profile binding and an explicit `calibration:=` path refuse such a file.
"""
import json
import types

from mongla_vision.calibration import binding


def _write(d, name, medium):
    p = d / name
    p.write_text(json.dumps({
        'applies_to': ['pi_downward'], 'medium': medium,
        'image_width': 1280, 'image_height': 720,
        'camera_matrix': [[700.0, 0, 640.0], [0, 700.0, 360.0], [0, 0, 1]],
        'distortion_coefficients': [0, 0, 0, 0, 0]}))
    return p


def test_the_binding_never_resolves_to_a_water_calibration(tmp_path, monkeypatch):
    monkeypatch.setattr(binding, '_candidate_dirs', lambda: [str(tmp_path)])
    _write(tmp_path, 'a_water.json', 'water')
    assert binding.calibration_for_profile('pi_downward') == ''
    air = _write(tmp_path, 'b.json', 'air')
    assert binding.calibration_for_profile('pi_downward') == str(air)


def test_an_explicit_water_calibration_is_refused(tmp_path):
    from mongla_vision.camera_node import CameraNode
    p = _write(tmp_path, 'w.json', 'water')
    log = []
    params = {'calibration': str(p), 'profile': 'pi_downward'}
    node = types.SimpleNamespace(
        get_parameter=lambda n: types.SimpleNamespace(value=params[n]),
        get_logger=lambda: types.SimpleNamespace(
            info=log.append, warn=log.append, warning=log.append))
    assert CameraNode._load_calibration(node) is None
    assert any('water' in m for m in log)
