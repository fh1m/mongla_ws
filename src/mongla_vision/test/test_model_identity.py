"""Model identity: stem normalization + key/stem resolution in the detector.

Pins the single-cam fix -- set_model() must accept the model STEM (what
missions/ClassRef send) as well as the registry KEY, so a plain single-model
launch and an alias-keyed registry both switch by the same name a mission uses.
"""

from mongla_vision.detector_node import _model_stem, DetectorNode


def test_model_stem_bare_and_path_and_ext():
    assert _model_stem('gate_sharks') == 'gate_sharks'
    assert _model_stem('/models/gate_sharks.pt') == 'gate_sharks'
    assert _model_stem('  gate_sharks.engine ') == 'gate_sharks'
    assert _model_stem('yolov11n') == 'yolov11n'


def _node_with_registry(registry_keys, stem_to_key):
    """A bare DetectorNode carrying just the two dicts _resolve_model_key reads."""
    n = object.__new__(DetectorNode)
    n._registry = {k: object() for k in registry_keys}
    n._stem_to_key = dict(stem_to_key)
    return n


def test_resolve_by_key():
    # alias-keyed registry: models:=gate=gate_sharks  -> use('gate') works
    n = _node_with_registry(['gate'], {'gate_sharks': 'gate'})
    assert n._resolve_model_key('gate') == 'gate'


def test_resolve_by_stem():
    # ...and set_model('gate_sharks') (the STEM / ClassRef) resolves too
    n = _node_with_registry(['gate'], {'gate_sharks': 'gate'})
    assert n._resolve_model_key('gate_sharks') == 'gate'


def test_resolve_stem_with_path_or_ext():
    n = _node_with_registry(['gate'], {'gate_sharks': 'gate'})
    assert n._resolve_model_key('/x/gate_sharks.pt') == 'gate'


def test_resolve_unknown_is_none():
    n = _node_with_registry(['gate'], {'gate_sharks': 'gate'})
    assert n._resolve_model_key('slalom_red_pipe') is None


def test_bare_stem_registry_key_equals_stem():
    # bare launch models:=gate_sharks -> key == stem; both resolve
    n = _node_with_registry(['gate_sharks'], {'gate_sharks': 'gate_sharks'})
    assert n._resolve_model_key('gate_sharks') == 'gate_sharks'
