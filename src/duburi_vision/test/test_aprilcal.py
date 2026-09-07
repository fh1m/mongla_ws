"""Max ERE and next-best-pose: the AprilCal machinery, pinned.

⛔ WHY THIS EXISTS. Reprojection RMS is quiet exactly where you have no data,
so it cannot tell a well-constrained calibration from an under-determined
one. This project has three instances: fx of 835.7, 969.9 and 1011.2, each
with a comfortable residual, and calib.io says it in one line -- "low
reprojection error does not equal a good camera calibration".

Max ERE (Richardson et al., AprilCal, IROS 2013) asks the other question:
how far apart would two equally plausible calibrations put the same point?
Measured on the operator's own 19 frames it read 37.18 px while every other
metric reported "HIGH" or "OK" -- it is the number that screams.

Their measured payoff, and the reason the suggester is worth its runtime:
novices using guided suggestion vs novices using plain OpenCV reached mean
reprojection error 0.229 vs 0.728 px and WORST-case 1.651 vs 38.646 px, a
23x difference on the number that breaks a bearing.

Loads the tool BY PATH, never by package import -- these live in tools/ and
importing this package from a worktree resolves against the main workspace's
stale install tree.
"""
import importlib.util
import pathlib

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

_TOOL = (pathlib.Path(__file__).resolve().parents[3] / 'tools' / 'fov_solve.py')


@pytest.fixture(scope='module')
def fs():
    if not _TOOL.is_file():
        pytest.skip('tools/fov_solve.py not present')
    spec = importlib.util.spec_from_file_location('fov_solve_under_test',
                                                  str(_TOOL))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _K(fx, fy=None, cx=640.0, cy=360.0):
    fy = fy or fx
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], float)


_D = np.zeros((1, 5))
_W, _H = 1280, 720


class TestMaxERE:
    def test_identical_calibrations_disagree_by_ZERO(self, fs):
        """The floor. If this is not 0 the metric has an offset and every
        threshold built on it is wrong."""
        e, _ = fs.max_ere([(_K(900), _D)] * 5, _W, _H)
        assert e == pytest.approx(0.0, abs=1e-9)

    def test_it_grows_MONOTONICALLY_with_disagreement(self, fs):
        """A quality metric that is not monotonic in the thing it measures
        cannot be used as a stopping criterion."""
        got = []
        for sd in (1, 5, 20, 50):
            s = [(_K(900 + d), _D) for d in (-sd, -sd / 2, 0, sd / 2, sd)]
            got.append(fs.max_ere(s, _W, _H)[0])
        assert got == sorted(got), f'not monotonic: {got}'
        # and roughly linear in the spread, which is what makes "px" a
        # meaningful unit here rather than an index
        assert got[3] / got[0] == pytest.approx(50.0, rel=0.25)

    def test_principal_point_uncertainty_is_SEEN(self, fs):
        """cx uncertainty shifts every projected point, so it must register.
        A metric that only watched focal length would miss the 26 px
        principal-point offset that makes `bearing.py` necessary."""
        e, _ = fs.max_ere([(_K(900, cx=640 + d), _D)
                           for d in (-5, -2.5, 0, 2.5, 5)], _W, _H)
        assert e > 1.0

    def test_a_single_draw_is_NaN_not_a_number(self, fs):
        """One sample has no spread. Returning 0.0 would read as 'perfect'
        -- the absence-is-not-zero trap, in a quality metric."""
        e, _ = fs.max_ere([(_K(900), _D)], _W, _H)
        assert np.isnan(e)

    def test_it_names_WHERE_the_worst_point_is(self, fs):
        """The location is what makes it actionable: it says which part of
        the frame the data has not constrained."""
        _, worst = fs.max_ere([(_K(900 + d), _D) for d in (-20, 0, 20)],
                              _W, _H)
        assert worst is not None and len(worst) == 2


class TestNextBestPose:
    """The suggester, on synthetic data with a known answer."""

    @staticmethod
    def _views(fs, K, D, poses, objp, rng):
        out = []
        for rvec, tvec in poses:
            v = fs.synth_view(objp, K, D, np.array(rvec, float),
                              np.array(tvec, float), _W, _H, rng=rng)
            if v is not None:
                out.append(v)
        return out

    def test_it_refuses_before_it_can_fit(self, fs):
        """Under six views there is nothing to base a suggestion on, and
        inventing one would send the operator somewhere arbitrary while
        looking authoritative."""
        objp = fs.board_points(8, 6, 0.025)
        cand, ere, table = fs.suggest_next_pose(objp, [], (_W, _H),
                                                [((1, 1), 20)])
        assert cand is None and np.isnan(ere) and table == []

    def test_a_suggestion_LOWERS_predicted_max_ere(self, fs):
        """The whole claim in one assertion: the pose it picks must be
        predicted better than the pose it rejects. Without this the
        suggester is an expensive random number generator."""
        rng = np.random.default_rng(1)
        objp = fs.board_points(8, 6, 0.025)
        K, D = _K(900), _D
        # A deliberately poor, clustered starting set -- all central, all
        # near square-on, which is what an unguided operator produces.
        poses = [((0.05 * i, 0.03 * i, 0.0), (0.0, 0.0, 0.85 + 0.02 * i))
                 for i in range(8)]
        ips = self._views(fs, K, D, poses, objp, rng)
        if len(ips) < 6:
            pytest.skip('synthetic views did not land in frame')
        cands = [((y, x), t) for y in range(3) for x in range(3)
                 for t in (10, 25, 40)]
        best, ere, table = fs.suggest_next_pose(objp, ips, (_W, _H), cands)
        if best is None or len(table) < 3:
            pytest.skip('no candidate landed in frame for this geometry')
        eres = [e for e, _ in table]
        assert ere == min(eres), 'it did not return the best candidate'
        assert ere < np.mean(eres), (
            f'the suggestion ({ere:.3f}) is no better than an average '
            f'candidate ({np.mean(eres):.3f}) -- then it is not suggesting')
