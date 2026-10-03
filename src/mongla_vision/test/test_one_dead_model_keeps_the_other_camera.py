"""A model that will not load costs ITS camera, not both (2026-10-03).

On the vehicle the forward detector was live on gate_sharks.hef; the downward
default had no .hef, fell back to the .pt, failed on "No module named
'torch'", and its os._exit took the forward detector down too.
"""
import types

import pytest


def test_a_missing_engine_is_named_not_reported_as_missing_torch(monkeypatch, tmp_path):
    import importlib.util as iu
    from mongla_vision.detection import factory
    real = iu.find_spec
    monkeypatch.setattr(iu, 'find_spec',
                        lambda name, *a, **k: None if name == 'torch' else real(name, *a, **k))
    pt = tmp_path / 'bin_fire_blood.pt'
    pt.write_bytes(b'x')
    with pytest.raises(FileNotFoundError) as e:
        factory.make_detector(model_path=str(pt))
    assert 'no compiled .hef' in str(e.value)
    assert 'hailo_compile.sh bin_fire_blood' in str(e.value)


def _launcher():
    seen = []
    log = types.SimpleNamespace(error=seen.append, fatal=seen.append,
                                warning=seen.append, info=seen.append)
    return types.SimpleNamespace(get_logger=lambda: log), seen


def test_one_failed_detector_keeps_the_process_and_the_other_camera(monkeypatch):
    import mongla_vision.detector_dual_node as dd
    exits = []
    monkeypatch.setattr(dd.os, '_exit', exits.append)
    launcher, seen = _launcher()
    live, failed, state = ['forward', 'downward'], [], {'built': True}
    hook = dd._make_failure_hook(launcher, live, failed, state)
    hook(types.SimpleNamespace(_cam_name='downward', get_name=lambda: 'x'),
         RuntimeError('no .hef'))
    assert exits == [] and failed == ['downward']
    assert any('downward DETECTOR did NOT come up' in s for s in seen)
    hook(types.SimpleNamespace(_cam_name='forward', get_name=lambda: 'x'),
         RuntimeError('device busy'))
    assert exits == [2], 'with no detector left it must not idle'


def test_nothing_is_decided_before_composition_finishes(monkeypatch):
    import mongla_vision.detector_dual_node as dd
    exits = []
    monkeypatch.setattr(dd.os, '_exit', exits.append)
    launcher, _ = _launcher()
    live, failed, state = ['forward'], [], {'built': False}
    hook = dd._make_failure_hook(launcher, live, failed, state)
    hook(types.SimpleNamespace(_cam_name='forward', get_name=lambda: 'x'),
         RuntimeError('x'))
    assert exits == [], 'downward is not built yet -- too early to give up'


def test_the_detector_calls_its_owner_instead_of_exiting(monkeypatch):
    import mongla_vision.detector_node as dn
    exits, called = [], []
    monkeypatch.setattr(dn.os, '_exit', exits.append)

    def boom(**k):
        raise FileNotFoundError('no .hef')
    monkeypatch.setattr(dn, 'make_detector', boom, raising=False)
    log = types.SimpleNamespace(fatal=lambda *a: None, info=lambda *a: None)
    stub = types.SimpleNamespace(get_logger=lambda: log,
                                 _init_failure_fn=lambda d, e: called.append(e))
    dn.DetectorNode._load_single_model_async(
        stub, model_path='m', device='', conf=0.5, iou=0.5, imgsz=640,
        half=False, max_det=10, allowlist=None)
    assert exits == [] and len(called) == 1
