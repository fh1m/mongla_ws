"""The board params that silently disable safety behaviour.

Twenty-one board parameters default to zero and several of them gate a SAFETY
behaviour rather than a convenience. A disabled gate is invisible: nothing
errors, nothing logs, the feature simply never happens -- the defect class that
has already cost this project a round three times (vision verbs in SURFACE, the
disarmed SROT_MOVE, MTUNE_EN).

`safety_gate_findings` is pure, so every branch is verified here with no board.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.fc import srot_protocol as sp     # noqa: E402


def _all_good():
    """A board with every gate live."""
    return {n: (1.0 if exp is None else exp)
            for n, exp, _s, _g in sp.SAFETY_GATES}


def test_a_fully_configured_board_reports_NOTHING():
    assert sp.safety_gate_findings(_all_good()) == []


def test_LEAK_EN_off_is_CRIT_and_names_BOTH_things_it_gates():
    """Measured 0 on this vehicle's board. It gates the failsafe AND the
    pre-arm refusal, so 'leak detected' never blocks arming either."""
    v = _all_good(); v['LEAK_EN'] = 0.0
    f = sp.safety_gate_findings(v)
    assert len(f) == 1
    sev, name, text = f[0]
    assert sev == 'CRIT' and name == 'LEAK_EN'
    assert 'failsafe' in text and 'pre-arm' in text


def test_ARMING_CHECK_off_is_CRIT_because_it_disables_EVERY_other_check():
    """fw arming.cpp:13 -- `if (arming_check < 0.5f) return true;` short-circuits
    the whole function, so the leak and battery refusals above go with it."""
    v = _all_good(); v['ARMING_CHECK'] = 0.0
    f = [x for x in sp.safety_gate_findings(v) if x[1] == 'ARMING_CHECK']
    assert f and f[0][0] == 'CRIT'
    assert 'pre-arm' in f[0][2]


def test_ESPNOW_EN_off_names_the_FOUR_features_that_go_dark_together():
    v = _all_good(); v['ESPNOW_EN'] = 0.0
    txt = [x[2] for x in sp.safety_gate_findings(v) if x[1] == 'ESPNOW_EN'][0]
    for expected in ('thruster volts', 'kill state', 'mixer', 'failsafe'):
        assert expected in txt, expected


def test_an_UNREADABLE_param_is_a_FINDING_not_a_pass():
    """⛔ Absence is not zero and it is not OK either. A parameter that never
    answered is the case where we know least, so it must not read as healthy."""
    v = _all_good(); v['LEAK_EN'] = None
    f = [x for x in sp.safety_gate_findings(v) if x[1] == 'LEAK_EN']
    assert f and 'UNREADABLE' in f[0][2]
    assert f[0][0] == 'CRIT'


def test_a_MISSING_key_is_treated_as_unreadable():
    """A dict that never got the key at all is the same state as no answer."""
    assert len(sp.safety_gate_findings({})) == len(sp.SAFETY_GATES)


def test_MOT_BAT_V_MAX_wants_ANY_positive_value_not_a_specific_one():
    """It is a pack voltage, so there is no single right number -- only 'set'."""
    v = _all_good(); v['MOT_BAT_V_MAX'] = 16.8
    assert [x for x in sp.safety_gate_findings(v) if x[1] == 'MOT_BAT_V_MAX'] == []
    v['MOT_BAT_V_MAX'] = 0.0
    f = [x for x in sp.safety_gate_findings(v) if x[1] == 'MOT_BAT_V_MAX']
    assert f and f[0][0] == 'WARN', 'a timed leg being pack-dependent is not CRIT'


def test_severities_separate_SAFETY_from_BEHAVIOUR():
    """CRIT must mean 'a safety behaviour is off'. If everything were CRIT the
    section would be noise and an operator would learn to skip it."""
    crit = {n for n, _e, s, _g in sp.SAFETY_GATES if s == 'CRIT'}
    warn = {n for n, _e, s, _g in sp.SAFETY_GATES if s == 'WARN'}
    assert crit == {'LEAK_EN', 'ESPNOW_EN', 'ARMING_CHECK',
                    'FS_BAT_ENABLE', 'FS_GCS_ENABLE'}
    assert warn == {'MOT_BAT_V_MAX', 'THR_TRIM_EN'}


def test_the_table_NEVER_becomes_a_set_of_values_to_WRITE():
    """Enabling a failsafe threshold or a motor-spinning mode is an operator
    decision on a specific hull. A host that quietly set these would be worse
    than one that never looked -- so nothing in the reader may PARAM_SET."""
    import inspect
    from mongla_manager import srot_connect
    src = inspect.getsource(srot_connect.read_safety_gates)
    assert 'param_set' not in src.lower()
