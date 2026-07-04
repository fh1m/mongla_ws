"""Per-model conf parsing (detector_node._parse_model_conf).

The parser feeds the live 'model_conf' param: a CSV of name=conf pairs that
override individual registry models. It must be crash-proof on junk (a
live-tuned param must never take down the node) and numeric-strict.
"""

from duburi_vision.detector_node import _parse_model_conf


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
