"""Model identity: stem normalization + key/stem resolution in the detector.

Pins the single-cam fix -- set_model() must accept the model STEM (what
missions/ClassRef send) as well as the registry KEY, so a plain single-model
launch and an alias-keyed registry both switch by the same name a mission uses.
"""

from mongla_vision.detector_node import _model_stem, DetectorNode


def test_model_stem_bare_and_path_and_ext():
    assert _model_stem('gate_rescue_repair') == 'gate_rescue_repair'
    assert _model_stem('/models/gate_rescue_repair.pt') == 'gate_rescue_repair'
    assert _model_stem('  gate_rescue_repair.engine ') == 'gate_rescue_repair'
    assert _model_stem('yolov11n') == 'yolov11n'


def _node_with_registry(registry_keys, stem_to_key):
    """A bare DetectorNode carrying just the two dicts _resolve_model_key reads."""
    n = object.__new__(DetectorNode)
    n._registry = {k: object() for k in registry_keys}
    n._stem_to_key = dict(stem_to_key)
    return n


def test_resolve_by_key():
    # alias-keyed registry: models:=gate=gate_rescue_repair  -> use('gate') works
    n = _node_with_registry(['gate'], {'gate_rescue_repair': 'gate'})
    assert n._resolve_model_key('gate') == 'gate'


def test_resolve_by_stem():
    # ...and set_model('gate_rescue_repair') (the STEM / ClassRef) resolves too
    n = _node_with_registry(['gate'], {'gate_rescue_repair': 'gate'})
    assert n._resolve_model_key('gate_rescue_repair') == 'gate'


def test_resolve_stem_with_path_or_ext():
    n = _node_with_registry(['gate'], {'gate_rescue_repair': 'gate'})
    assert n._resolve_model_key('/x/gate_rescue_repair.pt') == 'gate'


def test_resolve_unknown_is_none():
    n = _node_with_registry(['gate'], {'gate_rescue_repair': 'gate'})
    assert n._resolve_model_key('slalom_red_pipe') is None


def test_bare_stem_registry_key_equals_stem():
    # bare launch models:=gate_rescue_repair -> key == stem; both resolve
    n = _node_with_registry(['gate_rescue_repair'], {'gate_rescue_repair': 'gate_rescue_repair'})
    assert n._resolve_model_key('gate_rescue_repair') == 'gate_rescue_repair'
