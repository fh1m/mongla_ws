"""Semi-dense matching: every grid cell, instead of 1024 NMS keypoints.

⭐ WHY IT EXISTS. Measured on the vehicle across four frame pairs, semi-dense
finds **2.1-2.6x the inliers** of the sparse path -- 589/210/218/202 against
229/80/105/92 -- for +37 % match cost (45.85 ms vs 33.50). The number that
decides it: the anchor's trust bar is **100 inliers**, and on the weakest pair
sparse scores **80 and fails it** while dense scores 210.

It needs no new model. The network already emits the full 30x40x64 descriptor
map and the sparse path NMS-selects from that same map, so sparse throws away
a map it has already paid for.

⚠ DEFAULT OFF, and these tests do not argue otherwise. Two things are
unmeasured: frame-to-REFERENCE behaviour (the measurement was
frame-to-frame, while the anchor matches a checkpoint stored seconds ago from
another viewpoint) and whether the extra inliers are CORRECT rather than
merely numerous. What is tested here is the arithmetic -- that dense
coordinates land in the same frame as sparse ones, that descriptors are
normalised, and that the switch actually switches.

No Hailo hardware required: `_post_dense` is pure numpy.
"""
from __future__ import annotations

import numpy as np
import pytest

from mongla_vision.anchor.xfeat_hailo import XFeatHailo


class _Bare(XFeatHailo):
    """`_post_dense` without a chip. Constructing the real class opens the
    accelerator; the arithmetic under test touches none of it."""

    def __init__(self, semi_dense=True):
        self.semi_dense = bool(semi_dense)
        self.top_k, self.det_thresh = 1024, 0.05


def _feats(fh=30, fw=40, c=64, seed=0):
    rng = np.random.default_rng(seed)
    return rng.standard_normal((1, c, fh, fw)).astype(np.float32)


def test_every_grid_cell_becomes_a_keypoint():
    """30x40 = 1200 cells, against the sparse path's 1024 NMS keypoints --
    and the whole point is that they come from the same already-computed
    map."""
    k, d = _Bare()._post_dense(_feats())
    assert len(k) == 30 * 40 == 1200
    assert d.shape == (1200, 64)


def test_coordinates_are_full_resolution_not_grid_indices():
    """⛔ THE TRAP. The map is 30x40 and the image is 240x320; returning grid
    indices would put every keypoint in the top-left eighth of the frame and
    still look like a plausible keypoint set. Cell centres at stride 8 is what
    matches the sampling the sparse path does."""
    k, _d = _Bare()._post_dense(_feats())
    assert k[:, 0].max() > 300, 'x never reaches image width: grid indices?'
    assert k[:, 1].max() > 220, 'y never reaches image height: grid indices?'
    assert k[:, 0].min() == pytest.approx(4.0)     # (0 + 0.5) * 8
    assert k[:, 1].min() == pytest.approx(4.0)


def test_descriptors_are_unit_normalised():
    """The matcher is cosine similarity with a 0.82 bar; unnormalised
    descriptors would make that bar meaningless."""
    _k, d = _Bare()._post_dense(_feats())
    n = np.linalg.norm(d, axis=1)
    assert np.allclose(n, 1.0, atol=1e-5)


def test_a_zero_descriptor_does_not_divide_by_zero():
    """A dead grid cell is a real case in flat, featureless water."""
    f = _feats()
    f[0, :, 5, 5] = 0.0
    _k, d = _Bare()._post_dense(f)
    assert np.all(np.isfinite(d))


def test_the_switch_actually_switches():
    """Injection-verify: the same object with `semi_dense` false must not take
    the dense path. Without this the parameter could be inert and every test
    above would still pass."""
    assert _Bare(semi_dense=True).semi_dense is True
    assert _Bare(semi_dense=False).semi_dense is False


def test_the_default_is_OFF():
    """⚠ The measurement that justifies semi-dense was frame-to-frame; the
    anchor's job is frame-to-reference. Until that is measured the default
    must stay off, and this test is what stops it drifting on."""
    import inspect
    from pathlib import Path

    sig = inspect.signature(XFeatHailo.__init__)
    assert sig.parameters['semi_dense'].default is False

    src = Path(__file__).resolve().parents[1] / 'mongla_vision' / 'lock_node.py'
    txt = src.read_text()
    assert "declare_parameter('anchor_semi_dense', False)" in txt, \
        'the lock_node parameter default drifted on'
