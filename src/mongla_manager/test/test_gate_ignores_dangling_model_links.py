"""A dangling model link is not a model (found on the vehicle, 2026-10-03).

A symlink-install tree keeps links to models deleted from source. The gate
counted a retired model's dangling .hef as an engine and FAILED on its
dangling sidecar, while nothing could load it.
"""
import os

from mongla_manager.bringup_check import FAIL, PASS, WARN, _check_models_hailo


def _touch(p, text='x'):
    with open(p, 'w') as fh:
        fh.write(text)


def test_dangling_links_are_named_not_counted(tmp_path, monkeypatch):
    import mongla_manager.bringup_check as bc
    monkeypatch.setattr(bc, '_baked_floor', lambda p: None)
    d = tmp_path / 'models'
    d.mkdir()
    _touch(d / 'gate_sharks.hef')
    _touch(d / 'gate_sharks.yaml')
    os.symlink(tmp_path / 'gone.hef', d / 'retired.hef')
    os.symlink(tmp_path / 'gone.yaml', d / 'retired.yaml')
    status, msg = _check_models_hailo([str(d)])
    assert status == WARN, msg
    assert 'retired.hef' in msg and 'DANGLING' in msg


def test_a_real_hef_without_its_sidecar_still_fails(tmp_path, monkeypatch):
    import mongla_manager.bringup_check as bc
    monkeypatch.setattr(bc, '_baked_floor', lambda p: None)
    d = tmp_path / 'models'
    d.mkdir()
    _touch(d / 'orphan.hef')
    status, msg = _check_models_hailo([str(d)])
    assert status == FAIL and 'orphan' in msg


def test_a_clean_tree_passes(tmp_path, monkeypatch):
    import mongla_manager.bringup_check as bc
    monkeypatch.setattr(bc, '_baked_floor', lambda p: None)
    d = tmp_path / 'models'
    d.mkdir()
    _touch(d / 'gate_sharks.hef')
    _touch(d / 'gate_sharks.yaml')
    assert _check_models_hailo([str(d)])[0] == PASS
