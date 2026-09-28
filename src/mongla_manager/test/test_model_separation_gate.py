"""A model that LOADS is not a model that WORKS.

⛔ THE HOLE THIS CLOSED. Every model check in `bringup_check` asked whether
weights are present and whether an engine exists. None asked whether the graph
can tell the prop from the water -- and the vehicle's default forward graph,
`gate_rescue_repair`, claimed a gate on 90.8 % of gate-free frames, with **+0.0
points** of separation between gate-present and gate-absent footage at every
bar measured (B-59). It loaded perfectly, ran at full rate, and passed every
check we had. `gate_sharks`, in the same directory, measures **+84.2 points at
0.30** and **+96.8 at 0.45** on the same clips with the same tool.

That graph was retired on 2026-09-28 and its weights moved to
`~/models/retired/`. **So this file must not test against it** -- the check has
to keep working when no bad model is installed, which is the normal state and
the one a regression would hide in.

⚠ THE VERDICT LIVES IN THE SIDECAR, beside the weights it describes, so a copy
or a rename cannot separate them. `recommended_conf: null` means MEASURED AND
FOUND UNUSABLE, which is a different state from absent, and the check must not
collapse them.
"""
from __future__ import annotations

import pathlib

import pytest
import yaml

from mongla_manager.bringup_check import (FAIL, PASS, WARN,
                                          _check_model_separation)

_MODELS = (pathlib.Path(__file__).resolve().parents[3]
           / 'src' / 'mongla_vision' / 'models')


def _write(d, stem, sep):
    doc = {'names': {0: 'gate'}, 'path': '.'}
    if sep is not None:
        doc['separation'] = sep
    (d / f'{stem}.yaml').write_text(yaml.safe_dump(doc))


@pytest.fixture
def fake_models(tmp_path, monkeypatch):
    """A models dir we control, so the check is tested rather than the
    contents of the real one -- which changes when a model is retired."""
    import mongla_manager.bringup_check as bc
    monkeypatch.setattr(bc, '_models_dirs', lambda: [str(tmp_path)])
    return tmp_path


# ── the shipped state ───────────────────────────────────────────────────────


def test_the_shipped_gate_model_carries_the_bar_it_earned():
    """The real sidecar, because this number is what the vehicle flies on."""
    g = yaml.safe_load((_MODELS / 'gate_sharks.yaml').read_text())
    assert g['separation']['gate']['recommended_conf'] == pytest.approx(0.45)
    assert g['separation']['gate']['at_0_45'][2] > 90.0
    assert g['separation']['gate']['at_0_30'][2] > 80.0


def test_the_bar_records_how_and_where_it_was_measured():
    """A bar with no method and no venue cannot be re-derived or challenged."""
    g = yaml.safe_load((_MODELS / 'gate_sharks.yaml').read_text())['separation']
    assert 'negative_clip_check' in g['gate']['tool']
    assert g['gate']['venue']
    assert g['gate']['measured']


def test_the_retired_graph_is_really_gone():
    """⛔ It measured +0.0 points and it does not fly. Its WEIGHTS are kept at
    ~/models/retired/ so the measurement stays reproducible, but nothing in the
    shipped model set may resolve to it."""
    # ⛔ ANY EXTENSION, not just .yaml and .pt. The first version checked
    # those two and would have stayed green on a stale `.hef`: `build_mongla.
    # sh`'s sync is ADDITIVE, so weights left in the Pi's ~/models copy back
    # into the tree on the next vehicle-side build. Caught by a fresh-context
    # review, not by this test.
    stragglers = sorted(p.name for p in _MODELS.glob('gate_rescue_repair.*'))
    assert not stragglers, (
        f'the retired graph is back in the shipped model set: {stragglers}. '
        f'build_mongla.sh syncs additively -- check the vehicle\'s ~/models.')


def test_the_real_model_set_passes_the_check():
    """The normal state: no graph is marked unusable, so bringup goes green on
    this line."""
    st, det = _check_model_separation()
    assert st == PASS, det
    assert 'gate_sharks:gate@0.45' in det


# ── the check itself, against sidecars we control ───────────────────────────


def test_a_graph_measured_unusable_FAILS_the_preflight(fake_models):
    """⛔ IT MUST FAIL, NOT WARN. A hallucinating detector is worse than no
    detector, because the mission acts on it."""
    _write(fake_models, 'good', {'gate': {'measured': '2026-01-01',
                                          'recommended_conf': 0.45}})
    _write(fake_models, 'bad', {'gate': {'measured': '2026-01-01',
                                         'recommended_conf': None}})
    st, det = _check_model_separation()
    assert st == FAIL
    assert 'bad:gate' in det


def test_the_failure_names_the_usable_graph_and_the_way_out(fake_models):
    """A refusal that does not say what to do instead gets overridden."""
    _write(fake_models, 'good', {'gate': {'measured': '2026-01-01',
                                          'recommended_conf': 0.45}})
    _write(fake_models, 'bad', {'gate': {'measured': '2026-01-01',
                                         'recommended_conf': None}})
    _st, det = _check_model_separation()
    assert 'good:gate@0.45' in det
    assert 'fwd_models' in det, 'the run-both form is the resolution'


def test_an_unmeasured_graph_is_UNKNOWN_rather_than_bad(fake_models):
    """⚠ Most graphs have not been through the tool. Reporting them as
    failures would train the operator to ignore this line."""
    _write(fake_models, 'never_tested', None)
    st, det = _check_model_separation()
    assert st == WARN
    assert 'never_tested:' not in det
    assert 'negative_clip_check' in det, 'say how to fix it'


def test_a_measured_class_does_not_condemn_its_siblings(fake_models):
    """⚠ The verdict is PER CLASS. `rescue` and `repair` were never tested on
    the retired graph, and condemning them on the gate's evidence would be the
    same unmeasured confidence this check exists to stop, pointed the other
    way."""
    _write(fake_models, 'mixed', {'gate': {'measured': '2026-01-01',
                                           'recommended_conf': 0.45},
                                  'other': {'measured': None}})
    st, det = _check_model_separation()
    assert st == PASS
    assert 'mixed:gate@0.45' in det
    assert 'mixed:other' not in det


def test_a_malformed_sidecar_does_not_take_the_preflight_down(fake_models):
    """A typo in a YAML file must not stop an operator flying."""
    (fake_models / 'broken.yaml').write_text('separation: [this is not a map\n')
    _write(fake_models, 'good', {'gate': {'measured': '2026-01-01',
                                          'recommended_conf': 0.45}})
    st, _det = _check_model_separation()
    assert st in (PASS, WARN)
