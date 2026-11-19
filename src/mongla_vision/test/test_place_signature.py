"""The bank's place signature is VLAD over the shipped vocabulary.

The evidence that VLAD ranks the right reference better than the 64-D mean is
`tools/studies/bank_shortlist_accuracy.py` on real clips (measured-bars section
20.2). These tests hold the wiring: the vocabulary ships and loads, the
signature has the shape and norm the bank's dot product assumes, a scene is
nearer to itself than to another, the bank asks this module, and a bank saved
under the old signature is refused rather than ranked against the new one.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mongla_vision.anchor import place_signature as ps          # noqa: E402

PKG = Path(__file__).resolve().parents[1]


def test_the_vocabulary_ships_and_loads():
    w = ps.load_vocabulary()
    assert w.shape == (ps.K_WORDS, ps.DIM) and w.dtype == np.float32
    assert np.isfinite(w).all()
    assert 'config/*.npz' in (PKG / 'setup.py').read_text(), (
        'the vocabulary is not installed: the vehicle would raise at the first enrolment')


def test_a_signature_is_one_unit_vector_and_nothing_is_zeros():
    rng = np.random.default_rng(0)
    d = rng.normal(size=(1024, 64)).astype(np.float32)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    s = ps.signature(d)
    assert s.shape == (ps.SIG_DIM,)
    assert np.linalg.norm(s) == pytest.approx(1.0, abs=1e-5)
    for empty in (np.zeros((0, 64), np.float32), None):
        z = ps.signature(empty)
        assert z.shape == (ps.SIG_DIM,) and not z.any(), 'absence must not look like a place'


def _scene(words, which, n, rng, noise=0.05):
    """Descriptors that sit near the given words: a stand-in for one place."""
    pick = rng.choice(which, n)
    d = words[pick] + noise * rng.normal(size=(n, 64)).astype(np.float32)
    return (d / np.linalg.norm(d, axis=1, keepdims=True)).astype(np.float32)


def test_a_scene_is_nearer_to_a_resample_of_itself_than_to_another_scene():
    w = ps.load_vocabulary()
    rng = np.random.default_rng(1)
    a1, a2 = (_scene(w, list(range(0, 8)), 1024, rng) for _ in range(2))
    b = _scene(w, list(range(8, 16)), 1024, rng)
    assert ps.signature(a1) @ ps.signature(a2) > ps.signature(a1) @ ps.signature(b) + 0.2


def test_the_bank_signature_is_this_modules_and_a_bank_ranks_by_it():
    from mongla_vision.anchor.bank import CheckpointBank
    rng = np.random.default_rng(2)
    d = _scene(ps.load_vocabulary(), list(range(16)), 512, rng)
    assert np.array_equal(CheckpointBank._signature(d), ps.signature(d))


def test_a_bank_saved_with_the_mean_signature_is_refused(tmp_path):
    sys.path.insert(0, str(PKG / 'test'))
    from test_anchor_bank import bank, view
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9, label='gate', force=True)
    p = tmp_path / 'old.npz'
    b.save(str(p))
    z = dict(np.load(str(p)))
    z['version'] = np.array([3])
    np.savez(str(p), **z)
    fresh = bank()
    with pytest.raises(ValueError, match='mean signatures'):
        fresh.load(str(p))
    assert fresh.size == 0
