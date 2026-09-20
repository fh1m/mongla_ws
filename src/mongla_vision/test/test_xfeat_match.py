"""The matcher's optimisations must return THE SAME MATCHES.

That is the bar, and it is not pedantry: the anchor's entire value is that its
failures are decorrelated from the detector's. A matcher that is faster and
returns *different* correspondences is a different matcher, and every number
measured about the anchor -- inliers, hold length, the 175-frame rescue --
would no longer describe it.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.anchor import xfeat_onnx as X            # noqa: E402


def _desc(n, seed):
    d = np.random.default_rng(seed).standard_normal((n, 64)).astype(np.float32)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


@pytest.mark.parametrize('n0,n1', [(1024, 1024), (700, 300), (256, 1024), (9, 11)])
def test_the_threaded_matmul_does_not_change_which_points_MATCH(n0, n1):
    """NOT bit-identical, and asserting that would be an overclaim that passes
    on one machine and fails on the next: splitting the rows changes BLAS
    blocking and so summation order, worth up to ~2 float32 ulp. What must hold
    is that no PAIR changes, which is what the argmaxes decide."""
    d0, d1 = _desc(n0, 0), _desc(n1, 1)
    ref = d0 @ d1.T
    for th in (1, 2, 3, 4):
        got = X._simmat(d0, d1, th)
        assert np.allclose(got, ref, atol=1e-6, rtol=0), \
            f'{th} threads moved the similarity further than rounding'
        assert np.array_equal(got.argmax(axis=1), ref.argmax(axis=1))
        assert np.array_equal(got.argmax(axis=0), ref.argmax(axis=0))


def test_the_fast_column_argmax_is_BIT_identical():
    """`argmax(axis=0)` was 67x slower than axis=1 for the same work -- a column
    reduction over a row-major array. cv2 walks it in its own kernel."""
    for seed in range(4):
        sim = np.random.default_rng(seed).standard_normal((512, 400)).astype(np.float32)
        assert np.array_equal(X._argmax0(sim), sim.argmax(axis=0))


def test_the_column_argmax_falls_back_rather_than_failing():
    """Older OpenCV has no `reduceArgMax`. Correct and slow beats fast and
    absent -- the anchor must still run."""
    import builtins
    real = builtins.__import__

    def no_cv2(name, *a, **k):
        if name == 'cv2':
            raise ImportError('no cv2')
        return real(name, *a, **k)

    sim = np.random.default_rng(0).standard_normal((64, 48)).astype(np.float32)
    builtins.__import__ = no_cv2
    try:
        assert np.array_equal(X._argmax0(sim), sim.argmax(axis=0))
    finally:
        builtins.__import__ = real


def test_match_returns_the_same_pairs_at_every_thread_count():
    """The end-to-end property. Thread count is a performance knob; if it can
    change which keypoints pair up it is a correctness knob and must not ship."""
    d0, d1 = _desc(1024, 2), _desc(1024, 3)
    base = X.XFeatONNX.match(d0, d1, 0.0, threads=1)
    for th in (2, 3, 4):
        got = X.XFeatONNX.match(d0, d1, 0.0, threads=th)
        assert np.array_equal(got[0], base[0]) and np.array_equal(got[1], base[1])


def test_mutual_matching_survives_the_optimisation():
    """A one-way match maps every keypoint of a blank frame onto whichever
    reference point is least dissimilar -- a full match list and a confident,
    meaningless homography. The fast path must not quietly become one-way."""
    d0 = _desc(200, 4)
    blank = np.repeat(d0[:1], 200, axis=0)      # 200 copies of ONE descriptor
    i0, i1 = X.XFeatONNX.match(d0, blank, 0.82)
    assert len(i0) <= 1, f'{len(i0)} mutual matches against a degenerate set'


def test_the_default_thread_count_leaves_cores_for_the_control_loop():
    """4 threads is 3.9x and takes the whole box. This Pi also runs two
    detectors and the 50 Hz loop; an anchor that starves them to run 3 Hz
    faster is a net loss."""
    assert X.MATCH_THREADS == 2


def test_the_thread_setting_is_read_at_CALL_time_not_import_time():
    """`threads=MATCH_THREADS` as a default argument binds at DEFINITION time,
    so changing the module constant afterwards does nothing. It invalidated
    this module's own first A/B -- all four thread counts secretly ran at 2,
    and the numbers looked plausible because they were real, just all the same
    configuration."""
    seen = []
    real = X._simmat
    X._simmat = lambda a, b, th: seen.append(th) or real(a, b, th)
    try:
        d0, d1 = _desc(300, 5), _desc(300, 6)
        X.MATCH_THREADS = 3
        X.XFeatONNX.match(d0, d1, 0.82)
        X.MATCH_THREADS = 1
        X.XFeatONNX.match(d0, d1, 0.82)
        X.XFeatONNX.match(d0, d1, 0.82, threads=4)   # explicit still wins
    finally:
        X._simmat = real
        X.MATCH_THREADS = 2
    assert seen == [3, 1, 4], f'thread setting not honoured at call time: {seen}'
