"""Our XFeat picks and describes keypoints exactly as upstream's own code does.

`data/xfeat_upstream_reference.npz` is upstream's `detectAndCompute`
(verlab/accelerated_features) run on seeded synthetic heads, written by
`tools/studies/xfeat_reliability.py --write-fixture`; the heads are
regenerated here from the seed. Three behaviours ride on it, each worth
correct matches on the archive truth harness (measured-bars section 92):
the top-k is ranked by keypoint probability times reliability, descriptors
are sampled bicubic from the per-cell normalised map, and the last row and
column are dropped as upstream's sampling drops them.
"""
import importlib.util
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))
from mongla_vision.anchor import xfeat_onnx as X            # noqa: E402

_spec = importlib.util.spec_from_file_location(
    'xfeat_reliability', HERE.parents[3] / 'tools' / 'studies' / 'xfeat_reliability.py')
S = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(S)


def _ours():
    feats, kl, rel = S.fixture_heads()
    return X.keypoints_and_descriptors(feats, kl, 0.05, S.FIX_TOPK, rel=rel)


def test_the_same_keypoints_and_descriptors_as_upstream():
    ref = np.load(Path(S.FIXTURE))
    k, d = _ours()
    feats, kl, _ = S.fixture_heads()
    assert len(X._nms(X._pixel_shuffle_scores(kl[0]), 0.05)) > 2 * S.FIX_TOPK, \
        'the fixture must make the top-k choose, or ranking is untested'
    assert [tuple(p) for p in k.astype(int)] == [tuple(p) for p in ref['keypoints']]
    assert np.abs(d - ref['descriptors']).max() < 1e-5


def test_dropping_the_reliability_ranking_is_caught():
    ref = np.load(Path(S.FIXTURE))
    feats, kl, _ = S.fixture_heads()
    k, _ = X.keypoints_and_descriptors(feats, kl, 0.05, S.FIX_TOPK)
    assert {tuple(p) for p in k.astype(int)} != {tuple(p) for p in ref['keypoints']}
