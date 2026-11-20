"""The held-out-pool split keeps a held-out frame out of training.

tools/studies/pool_holdout.py builds the leave-one-pool-out dataset. Teams fork
each other's datasets, so a held-out frame can sit in a training source under
another name; if it leaks, the held-out score reads high. Its self-test plants
a re-encoded copy and must drop it; with the dedupe off it must fail.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools' / 'studies'))
import pool_holdout  # noqa: E402


def test_the_self_test_passes():
    assert pool_holdout._self_test() == 0


def test_it_fails_with_the_dedupe_off(monkeypatch):
    monkeypatch.setattr(pool_holdout, 'near', lambda h, bank, bits: False)
    assert pool_holdout._self_test() == 1


def test_it_fails_when_a_gate_pole_counts_as_a_gate(monkeypatch):
    monkeypatch.setattr(pool_holdout, 'COARSE',
                        tuple(x for x in pool_holdout.COARSE if x[0] != 'pole'))
    assert pool_holdout._self_test() == 1
