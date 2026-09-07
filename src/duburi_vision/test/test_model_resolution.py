"""Which artifact a bare model STEM resolves to, per machine.

Stem identity is what keeps missions, `ClassRef` and `duburi.use('<stem>')`
backend-agnostic: the same `gate_rescue_repair` is a `.pt` on a dev box, an
`.engine` on the Jetson and a `.hef` on the Pi. The resolution order is the
only thing that makes that true, and getting it wrong is silent -- a stale
`.pt` of the same stem runs at 3 Hz and looks like a slow chip.
"""
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.detection.yolo import _resolve_model_path   # noqa: E402


@pytest.fixture
def tree(tmp_path, monkeypatch):
    """A fake source models/ dir, so these do not depend on what is checked in."""
    models = tmp_path / 'models'
    models.mkdir()
    monkeypatch.setattr('duburi_vision.detection.yolo._find_src_models_dir',
                        lambda: models)
    monkeypatch.delenv('DUBURI_HEF_DIR', raising=False)
    return models


def _touch(d, name):
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_bytes(b'\0')
    return str(d / name)


def test_an_explicit_path_is_passed_through(tree):
    assert _resolve_model_path('/abs/x.hef') == '/abs/x.hef'
    assert _resolve_model_path('/abs/x.pt') == '/abs/x.pt'


def test_the_compiled_artifact_wins_over_the_source_weights(tree):
    """The .pt is kept beside the .hef (it is what the .hef was built from).
    Resolving to it would run PyTorch on a Pi -- 3 Hz, and nothing says why."""
    _touch(tree, 'gate.pt')
    _touch(tree, 'gate.hef')
    assert _resolve_model_path('gate').endswith('gate.hef')


def test_the_pt_is_still_the_fallback_on_a_box_with_no_accelerator(tree):
    _touch(tree, 'gate.pt')
    assert _resolve_model_path('gate').endswith('gate.pt')


def test_DUBURI_HEF_DIR_is_read(tree, tmp_path, monkeypatch):
    """`tools/pi_env.sh` has exported this since the Pi was built and NOTHING
    read it, so the export was a lie."""
    hefs = tmp_path / 'hailo_models'
    _touch(hefs, 'gate.hef')
    monkeypatch.setenv('DUBURI_HEF_DIR', str(hefs))
    assert _resolve_model_path('gate') == str(hefs / 'gate.hef')


def test_DUBURI_HEF_DIR_beats_a_stale_stem_in_the_tree(tree, tmp_path, monkeypatch):
    """The failure this exists to stop: the env names the artifact compiled for
    THIS machine, and a same-stem .pt in the source tree silently wins if the
    tree is searched first."""
    _touch(tree, 'gate.pt')
    hefs = tmp_path / 'hailo_models'
    _touch(hefs, 'gate.hef')
    monkeypatch.setenv('DUBURI_HEF_DIR', str(hefs))
    assert _resolve_model_path('gate').endswith('hailo_models/gate.hef')


def test_an_unset_or_empty_env_does_not_change_a_dev_box(tree, monkeypatch):
    _touch(tree, 'gate.pt')
    for val in ('', '   '):
        monkeypatch.setenv('DUBURI_HEF_DIR', val)
        assert _resolve_model_path('gate').endswith('gate.pt')


def test_a_stem_missing_from_the_env_dir_falls_through_rather_than_failing(
        tree, tmp_path, monkeypatch):
    """A Pi carries some stems as HEFs and others not. Pointing at the dir must
    not make every other stem unresolvable."""
    _touch(tree, 'other.pt')
    hefs = tmp_path / 'hailo_models'
    _touch(hefs, 'gate.hef')
    monkeypatch.setenv('DUBURI_HEF_DIR', str(hefs))
    assert _resolve_model_path('other').endswith('other.pt')


def test_a_tilde_in_the_env_is_expanded(tree, tmp_path, monkeypatch):
    """`pi_env.sh` writes `~/hailo_models`, and an unexpanded tilde is a
    directory that does not exist -- which falls through silently."""
    home = tmp_path / 'home'
    _touch(home / 'hailo_models', 'gate.hef')
    monkeypatch.setenv('HOME', str(home))
    monkeypatch.setenv('DUBURI_HEF_DIR', '~/hailo_models')
    assert _resolve_model_path('gate').endswith('hailo_models/gate.hef')
