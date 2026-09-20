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

The solver now lives IN the package (`mongla_vision.calibration.solver`)
rather than in `tools/`, because recalibrating a camera on competition ground
is a mission capability -- so it installs, and it is covered here.
"""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')


@pytest.fixture(scope='module')
def fs():
    from mongla_vision.calibration import solver
    return solver


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


class TestItShipsWithThePackage:
    """Calibration must be REACHABLE on the vehicle, not just present.

    ⛔ A capability that exists only as a file in `tools/` is not a
    capability at a competition: `tools/` is not installed, so on a fresh
    deploy the command simply is not there. This package has four recorded
    instances of a config or a helper that reached nothing; an entry point
    that quietly disappears would be a fifth, and it would be discovered on
    the day a lens gets knocked.
    """

    @staticmethod
    def _setup_py():
        import pathlib
        return (pathlib.Path(__file__).resolve().parents[1]
                / 'setup.py').read_text()

    def test_calibrate_is_a_console_script(self):
        s = self._setup_py()
        assert 'mongla_vision.calibration.guide:main' in s, (
            'the guided capture is no longer an entry point -- '
            '`ros2 run mongla_vision calibrate` would not exist on a fresh '
            'install, and nobody finds that out until they need it')
        assert 'mongla_vision.calibration.solver:main' in s

    def test_the_calibration_package_is_INSTALLED(self):
        """`find_packages` must pick the subpackage up. A module that is
        imported in tests but not installed passes here and fails on the
        vehicle -- which is the exact shape of the worktree/stale-install
        trap this project has already been bitten by."""
        s = self._setup_py()
        assert 'find_packages' in s or 'mongla_vision.calibration' in s, (
            'setup.py neither uses find_packages nor names the calibration '
            'subpackage, so it will not be installed')


class TestItRunsWithoutROS:
    """The calibration maths must not need rclpy. It needs cv2 and numpy.

    ⛔ THIS BROKE ON THE OPERATOR, mid-session, on the Solve button, with a
    complete 24-frame capture already on disk. The `tools/` shim did
    `from mongla_vision.calibration.solver import main` after putting the
    source tree on sys.path, which drags in `mongla_vision/__init__.py` ->
    `preflight` -> `rclpy`. Without a sourced ROS that is a
    ModuleNotFoundError.

    Needing ROS was an accident of the import path, never a requirement --
    and calibration is exactly the job you may have to do on a laptop, in a
    tent, from a folder of images, with no workspace sourced. The guard is
    static so it holds even where rclpy happens to be importable.
    """

    @staticmethod
    def _src(name):
        import pathlib
        return (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
                / 'calibration' / name).read_text()

    @pytest.mark.parametrize('name', ('solver.py', 'guide.py'))
    def test_no_ros_imports_in_the_calibration_modules(self, name):
        src = self._src(name)
        for bad in ('import rclpy', 'from rclpy', 'rclpy.init',
                    'from rcl_interfaces', 'import sensor_msgs'):
            assert bad not in src, (
                f'{name} imports ROS ({bad!r}). Calibration must run from a '
                f'folder of images on any machine -- and this exact coupling '
                f'already failed on the Solve button with a full capture set '
                f'on disk.')

    def test_the_shim_does_not_import_the_PACKAGE(self):
        """The package `__init__` is what pulls ROS in, so the shim has to
        load the module by path. A plain package import here reads as
        correct and fails only where there is no ROS -- which is the one
        place the shim exists to serve."""
        import pathlib
        for t in ('fov_solve.py', 'fov_calibrate_web.py'):
            p = pathlib.Path(__file__).resolve().parents[3] / 'tools' / t
            if not p.is_file():
                continue
            # Parse, do not grep: the docstring DESCRIBES the old import,
            # and a substring check flagged the explanation as the defect.
            import ast as _ast
            tree = _ast.parse(p.read_text())
            for node in _ast.walk(tree):
                mod = None
                if isinstance(node, _ast.ImportFrom):
                    mod = node.module or ''
                elif isinstance(node, _ast.Import):
                    mod = ','.join(a.name for a in node.names)
                if mod and mod.split('.')[0] == 'mongla_vision':
                    pytest.fail(
                        f'tools/{t} imports the package again ({mod}) -- that '
                        f're-runs mongla_vision/__init__ and needs rclpy')
            s = p.read_text()
            assert 'spec_from_file_location' in s, (
                f'tools/{t} no longer loads by path')


class TestItTakesOverCleanly:
    """A restart must just work, and must not shoot itself.

    ⛔ ON COMPETITION GROUND YOU DO NOT GO PID HUNTING. A second launch died
    on `OSError: [Errno 98] Address already in use`, and the remedy -- find
    the process, notice there are TWO of them because `ros2 run` execs the
    node as a child, kill both -- is the wrong job beside a pool with a run
    slot ticking.

    The dangerous half is self-preservation: the reaper matches on "looks
    like this tool", and the process running it looks exactly like this tool.
    So does the `ros2 run` wrapper that launched it, which shares its process
    group and whose death takes the node with it.
    """

    @staticmethod
    def _guide():
        import importlib.util
        import pathlib
        p = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
             / 'calibration' / 'guide.py')
        spec = importlib.util.spec_from_file_location('guide_under_test', p)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m

    def test_it_never_reaps_ITSELF(self):
        """The whole safety property. If this regresses the tool kills itself
        on startup and the failure looks like 'it just exits'."""
        import os
        g = self._guide()
        # A free port: nothing holds it, so the only candidates are the
        # name matches -- which include this very process.
        killed = g.reap_previous(59_431, wait_s=0.5)
        assert os.getpid() not in killed, 'the reaper targeted its own pid'

    def test_it_spares_its_own_process_GROUP(self):
        """`ros2 run` is the parent and shares the group; killing it kills
        the node. The pid check alone does not cover that."""
        import os
        g = self._guide()
        killed = g.reap_previous(59_432, wait_s=0.5)
        my_pg = os.getpgid(0)
        for k in killed:
            try:
                assert os.getpgid(k) != my_pg, (
                    f'pid {k} shares our process group and was targeted')
            except OSError:
                pass

    def test_the_opt_out_exists(self):
        """Taking over is the DEFAULT, but running two deliberately has to
        stay possible -- the A/B rig does exactly that with other nodes."""
        import pathlib
        s = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
             / 'calibration' / 'guide.py').read_text()
        assert "'--no-reap'" in s
        assert 'if not a.no_reap:' in s


class TestTheSolveButtonPointsAtSomethingReal:
    """The solve subprocess path must exist.

    ⛔ IT DID NOT. When calibration moved into the package, `fov_solve.py`
    became `solver.py` and the argv that launches it kept the old name. A
    stale string surviving a rename is invisible until someone presses the
    button -- and the button is pressed at the END of a capture session, so
    the cost is a whole calibration's worth of the operator's time. That is
    the same defect class as the calibration file named for a camera it was
    not taken with.
    """

    def test_the_solver_script_exists_where_guide_looks(self):
        import pathlib
        import re
        cal = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
               / 'calibration')
        src = (cal / 'guide.py').read_text()
        m = re.search(r"os\.path\.join\(here,\s*'([^']+\.py)'\)", src)
        assert m, 'guide.py no longer builds a solver path this test can read'
        assert (cal / m.group(1)).is_file(), (
            f'guide.py launches {m.group(1)!r}, which does not exist beside '
            f'it -- the Solve button would fail after a full capture session')
