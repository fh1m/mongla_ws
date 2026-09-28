"""A model that LOADS is not a model that WORKS.

⛔ THE DEFECT. Every model check in `bringup_check` asked whether weights are
present and whether an engine exists. None asked whether the graph can tell the
prop from the water -- and the vehicle's default forward graph,
`gate_rescue_repair`, claims a gate on 90.8 % of gate-free frames, with **+0.0
points** of separation between gate-present and gate-absent footage at every
bar measured (B-59). It loaded perfectly, ran at full rate, and passed every
check we had. `gate_sharks`, sitting in the same directory with a compiled
`.hef`, measures **+84.2 points at 0.30** and **+96.8 at 0.45** on the same
clips with the same tool.

⭐ THE MEASUREMENT LIVES IN THE SIDECAR, beside the weights it describes, so a
copy or a rename cannot separate them. `recommended_conf: null` means MEASURED
AND FOUND UNUSABLE -- which is a different state from absent, and the check
must not collapse them.
"""
from __future__ import annotations

import pathlib

import pytest
import yaml

_MODELS = (pathlib.Path(__file__).resolve().parents[3]
           / 'src' / 'mongla_vision' / 'models')


def _sidecar(stem):
    return yaml.safe_load((_MODELS / f'{stem}.yaml').read_text())


def test_the_measurement_ships_beside_the_weights():
    """A number in a doc drifts from the file it describes; a number in the
    sidecar travels with it."""
    for stem in ('gate_sharks', 'gate_rescue_repair'):
        d = _sidecar(stem)
        assert 'separation' in d, f'{stem} lost its separation block'
        assert 'gate' in d['separation']


def test_the_bad_graph_is_marked_unusable_for_gate_not_merely_unmeasured():
    """⛔ THE DISTINCTION THAT MATTERS. Absent means nobody has run the tool.
    `null` means somebody ran it and the answer was no."""
    g = _sidecar('gate_rescue_repair')['separation']['gate']
    assert g['measured'] is not None, 'this WAS measured -- do not mark it unknown'
    assert g['recommended_conf'] is None
    assert g['at_0_30'][2] == 0.0, 'the +0.0 separation is the whole finding'


def test_the_good_graph_carries_the_bar_it_earned():
    g = _sidecar('gate_sharks')['separation']['gate']
    assert g['recommended_conf'] == pytest.approx(0.45)
    assert g['at_0_45'][2] > 90.0
    assert g['at_0_30'][2] > 80.0


def test_the_sidecar_records_how_and_where_it_was_measured():
    """A bar with no method and no venue cannot be re-derived or challenged."""
    for stem in ('gate_sharks', 'gate_rescue_repair'):
        g = _sidecar(stem)['separation']['gate']
        assert 'negative_clip_check' in g['tool']
        assert g['venue']


def test_the_other_classes_are_not_swept_up_by_the_gate_verdict():
    """⚠ The measurement is about `gate`. `rescue` and `repair` were never
    tested, and marking them bad on the gate's evidence would be the same
    unmeasured confidence this check exists to stop -- in the other
    direction."""
    sep = _sidecar('gate_rescue_repair')['separation']
    for cls in ('rescue', 'repair'):
        assert sep[cls]['measured'] is None


def test_bringup_FAILS_on_a_graph_measured_unusable():
    """⛔ IT MUST FAIL, NOT WARN. A hallucinating detector is worse than no
    detector, because the mission acts on it."""
    from mongla_manager.bringup_check import FAIL, _check_model_separation

    st, det = _check_model_separation()
    assert st == FAIL
    assert 'gate_rescue_repair:gate' in det


def test_the_failure_names_the_usable_graph_and_the_way_out():
    """A refusal that does not say what to do instead gets overridden."""
    from mongla_manager.bringup_check import _check_model_separation

    _st, det = _check_model_separation()
    assert 'gate_sharks' in det
    assert 'fwd_models' in det, 'the run-both form is the resolution'


def test_an_unmeasured_graph_is_unknown_rather_than_bad():
    """⚠ Most graphs have not been through the tool. Reporting them as
    failures would train the operator to ignore this line."""
    import pathlib as _p

    stems = {p.stem for p in _MODELS.glob('*.yaml')}
    unmeasured = [s for s in stems
                  if 'separation' not in (_sidecar(s) or {})]
    assert unmeasured, 'expected some graphs to be unmeasured'
    from mongla_manager.bringup_check import _check_model_separation
    _st, det = _check_model_separation()
    for s in unmeasured:
        assert f'{s}:' not in det, f'{s} is unmeasured, not condemned'
