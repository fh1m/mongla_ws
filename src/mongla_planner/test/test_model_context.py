"""ClassRef identity: a ClassRef must carry the model STEM, not the DSL alias.

The detector matches set_model() against the stem (single-model launch or a
registry key/stem). Missions register `mongla.models(alias='stem')` where the
alias almost always differs from the stem, so a ClassRef that carried the alias
would send set_model('<alias>') -- which no detector knows. These pin the fix.
"""

from mongla_planner.model_context import ClassRef, ModelRegistry


def test_classref_carries_stem_not_alias():
    m = ModelRegistry()
    # alias 'robosub' != stem 'gate_rescue_repair' -- the real mission pattern.
    m(robosub=('gate_rescue_repair', ['gate', 'rescue', 'repair']))
    ref = m.robosub.gate
    assert isinstance(ref, ClassRef)
    assert ref.model_name == 'gate_rescue_repair'   # STEM, the detector identity
    assert ref.class_name == 'gate'


def test_classref_stem_for_bare_string_registration():
    m = ModelRegistry()
    m(gate='gate_flare_medium_100ep')     # bare stem, no class list
    ref = m.gate.gate
    assert ref.model_name == 'gate_flare_medium_100ep'
    assert ref.class_name == 'gate'


def test_classref_index_access_also_uses_stem():
    m = ModelRegistry()
    m(bins=('bin_fire_blood', ['fire', 'blood']))
    ref = m.bins[0]
    assert ref.model_name == 'bin_fire_blood'
    assert ref.class_name == 'fire'


def test_unknown_class_still_raises_with_alias_in_message():
    m = ModelRegistry()
    m(gate=('gate_rescue_repair', ['gate', 'rescue']))
    try:
        _ = m.gate.typo
        assert False, "expected AttributeError"
    except AttributeError as exc:
        assert 'gate' in str(exc)   # the handle alias names the model in the error
