"""Per-model conf parsing (detector_node._parse_model_conf).

The parser feeds the live 'model_conf' param: a CSV of name=conf pairs that
override individual registry models. It must be crash-proof on junk (a
live-tuned param must never take down the node) and numeric-strict.
"""

from duburi_vision.detector_node import _parse_model_conf, DetectorNode


def test_single_pair():
    assert _parse_model_conf('torpedo_blood_hole=0.55') == {'torpedo_blood_hole': 0.55}


def test_multiple_pairs():
    got = _parse_model_conf('gate=0.35, torpedo_blood_hole=0.55 ,slalom=0.4')
    assert got == {'gate': 0.35, 'torpedo_blood_hole': 0.55, 'slalom': 0.4}


def test_empty_is_empty_dict():
    assert _parse_model_conf('') == {}
    assert _parse_model_conf('   ') == {}


def test_skips_bare_and_malformed():
    # bare entry (no '='), no name, and non-numeric conf are all skipped, not fatal
    assert _parse_model_conf('gate,=0.5,torpedo=abc,slalom=0.4') == {'slalom': 0.4}


def test_whitespace_tolerant():
    assert _parse_model_conf('  gate = 0.6 ') == {'gate': 0.6}


# --------------------------------------------------------------------------- #
#  _apply_model_conf must resolve by STEM too (the b155736 identity-bug class) #
# --------------------------------------------------------------------------- #
# Regression guard: set_conf(model='<stem>') was silently no-op'ing on a
# single-model launch (_active_name stays None) and an aliased registry
# (models:=gate=stem, keyed by alias) because the per-model-conf path did an
# exact-key lookup. It now mirrors the active_model handler's key-OR-stem resolve.

class _FakeDet:
    def __init__(self):
        self.conf = None
    def update_conf(self, c):
        self.conf = float(c)


class _Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass


def _bare_node():
    n = object.__new__(DetectorNode)
    n._registry = {}
    n._stem_to_key = {}
    n._single_model_name = None
    n._det = None
    n.get_logger = lambda: _Log()
    return n


def test_single_model_conf_by_stem_applies():
    n = _bare_node()
    n._det = _FakeDet()
    n._single_model_name = 'gate_rescue_repair'   # single-model launch: _active_name is None
    n._apply_model_conf('gate_rescue_repair=0.55')
    assert n._det.conf == 0.55                     # was silently dropped before the fix


def test_single_model_conf_wrong_stem_is_noop_not_crash():
    n = _bare_node()
    n._det = _FakeDet()
    n._single_model_name = 'gate_rescue_repair'
    n._apply_model_conf('some_other_model=0.55')   # names a model not loaded
    assert n._det.conf is None                      # warned, not applied, no crash


def test_aliased_registry_conf_by_stem_applies():
    n = _bare_node()
    det = _FakeDet()
    n._registry = {'gate': det}                     # models:=gate=gate_nano_100ep
    n._stem_to_key = {'gate_nano_100ep': 'gate'}
    n._apply_model_conf('gate_nano_100ep=0.62')     # tighten by STEM
    assert det.conf == 0.62


def test_registry_conf_by_key_still_works():
    n = _bare_node()
    det = _FakeDet()
    n._registry = {'gate': det}
    n._stem_to_key = {'gate_nano_100ep': 'gate'}
    n._apply_model_conf('gate=0.40')                # the documented key form still applies
    assert det.conf == 0.40
