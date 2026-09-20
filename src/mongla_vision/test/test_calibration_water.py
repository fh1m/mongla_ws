"""Calibrating IN WATER: a separate answer, kept separate.

⛔ WHY THIS MODE EXISTS AT ALL. The normal path is the Pinax one -- calibrate
once in AIR and correct the flat port's refraction analytically, which is
what `RefractiveRectifier` already does. So an in-water calibration is not
the primary method; it is the VALIDATION of that correction, and
`BUGS.md` §11 item 2 asks for exactly it ("confirm the measured f_water = 741
against a tape in water").

That makes the danger obvious: a validation artefact mistaken for the
primary calibration puts a ~1.44x scale error on every range and velocity
the vehicle computes -- silent, and exactly the size that still looks
plausible. So the medium is recorded in the file, stated on the page, and
carried in the FILENAME.

Structure, deliberately: the medium logic is a pure function and is tested
cheaply; ONE end-to-end solve on rendered chessboards then proves that
function is actually WIRED, that the filename carries the medium, and that
the fit recovers the camera it was rendered with. The previous guard in this
area was a grep, and it stayed green through the very rename it existed to
catch -- so the expensive test is the one that earns its 45 s.
"""
import json
import math
import os

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')


@pytest.fixture(scope='module')
def fs():
    from mongla_vision.calibration import solver
    return solver


# ---------------------------------------------------------------- cheap
def test_air_leaves_the_fit_alone(fs):
    v = {'hfov_air': 70.0, 'vfov_air': 43.0,
         'hfov_water': 51.0, 'vfov_water': 32.0}
    out, note = fs.fov_for_medium(v, 'air')
    assert out == v and note is None


def test_water_makes_the_FITTED_figure_the_WATER_one(fs):
    v = {'hfov_air': 70.0, 'vfov_air': 43.0,
         'hfov_water': 51.0, 'vfov_water': 32.0}
    out, note = fs.fov_for_medium(v, 'water')
    # The fit saw refracted rays, so what `fov_from_K` labelled "air" IS the
    # water figure.
    assert out['hfov_water'] == pytest.approx(70.0)
    assert out['vfov_water'] == pytest.approx(43.0)
    # Air is now wider, by inverse Snell -- a flat port narrows the view.
    assert out['hfov_air'] == pytest.approx(
        2 * math.degrees(math.asin(1.333 * math.sin(math.radians(35.0)))))
    assert out['hfov_air'] > out['hfov_water']
    assert 'DERIVED' in note
    # It does not mutate the caller's dict; the ANSWER print and the file
    # must see one consistent object.
    assert v['hfov_air'] == 70.0


def test_the_conversion_is_never_applied_twice(fs):
    """The failure this exists to stop, stated as arithmetic.

    If water mode had instead run the air->water conversion on an already-
    in-water fit, the reported water FOV would be ~1.33x narrower than the
    truth. Assert we get the fit back, not the double conversion.
    """
    v = {'hfov_air': 70.0, 'vfov_air': 43.0,
         'hfov_water': 51.0, 'vfov_water': 32.0}
    out, _ = fs.fov_for_medium(v, 'water')
    assert out['hfov_water'] != pytest.approx(51.0, abs=1.0)


def test_the_filename_rule_has_ONE_definition(fs):
    assert fs.calibration_filename('pi_forward', 1280, 720) == \
        'pi_forward_1280x720.json'
    assert fs.calibration_filename('pi_forward', 1280, 720, 'water') == \
        'pi_forward_1280x720_water.json'
    # The guide serves what the solver installed; it must not restate the
    # rule. A second copy of a naming rule is how a calibration once
    # outlived the camera it was named for.
    from mongla_vision.calibration import guide
    assert 'calibration_filename' in open(guide.__file__).read()


# ------------------------------------------------------------ end to end
def _board(cols, rows, px=80, margin=1):
    nx, ny = cols + 1 + 2 * margin, rows + 1 + 2 * margin
    img = np.full((ny * px, nx * px), 255, np.uint8)
    for j in range(rows + 1):
        for i in range(cols + 1):
            if (i + j) % 2 == 0:
                img[(j + margin) * px:(j + margin + 1) * px,
                    (i + margin) * px:(i + margin + 1) * px] = 0
    return img, nx, ny, px, margin


def _render(K, rvec, tvec, cols, rows, square, W, H):
    img, nx, ny, px, margin = _board(cols, rows)
    ox = oy = -margin * square
    src = np.float32([[0, 0], [nx * px, 0], [nx * px, ny * px], [0, ny * px]])
    obj = np.float32([[ox, oy, 0], [ox + nx * square, oy, 0],
                      [ox + nx * square, oy + ny * square, 0],
                      [ox, oy + ny * square, 0]])
    dst, _ = cv2.projectPoints(obj, rvec, tvec, K, np.zeros(5))
    M = cv2.getPerspectiveTransform(src, dst.reshape(-1, 2).astype(np.float32))
    # INTER_AREA: without it the downscaled squares alias and
    # findChessboardCorners rejects most of the set (4/27 with the default).
    return cv2.warpPerspective(img, M, (W, H), borderValue=128,
                               flags=cv2.INTER_AREA)


_FX = 900.0
_W, _H = 1280, 720


@pytest.fixture(scope='module')
def water_solve(fs, tmp_path_factory):
    """One in-water solve, rendered and run once for the whole module."""
    d = str(tmp_path_factory.mktemp('views'))
    # DELIBERATELY does not exist: `--install` into a missing directory used
    # to raise FileNotFoundError *after* the whole 45 s solve, losing it. So
    # the end-to-end run is also the guard for that.
    inst = os.path.join(str(tmp_path_factory.mktemp('root')), 'not_yet')
    assert not os.path.exists(inst)
    K = np.array([[_FX, 0, _W / 2], [0, _FX, _H / 2], [0, 0, 1]])
    rng = np.random.default_rng(3)
    n = 0
    for tx in (-0.10, 0.0, 0.10):
        for ty in (-0.07, 0.07):
            for tilt in (0.0, 0.35):
                rv = np.array([tilt + rng.normal(0, .03),
                               tilt * 0.6 + rng.normal(0, .03),
                               rng.normal(0, .05)])
                tv = np.array([tx, ty, 0.60 + rng.normal(0, .01)])
                cv2.imwrite(os.path.join(d, f'v{n:03d}.png'),
                            _render(K, rv, tv, 8, 6, 0.025, _W, _H))
                n += 1
    import sys
    old = sys.argv
    sys.argv = ['solver', d, '--grid', '8x6', '--square', '0.025',
                '--applies-to', 'synth_cam', '--install', inst,
                '--medium', 'water']
    try:
        rc = fs.main()
    finally:
        sys.argv = old
    assert rc == 0
    files = os.listdir(inst)
    return files, json.load(open(os.path.join(inst, files[0])))


def test_the_installed_water_file_is_named_for_its_medium(water_solve):
    files, _ = water_solve
    assert files == ['synth_cam_1280x720_water.json']


def test_the_installed_water_file_records_what_it_is(water_solve):
    _, out = water_solve
    assert out['medium'] == 'water'
    assert 'DERIVED' in out['air_fov_note']


def test_the_water_FOV_is_the_one_the_fit_actually_produced(water_solve):
    """The wiring check: `fov_for_medium` reached the written file.

    2*atan(w / 2fx) from the installed K is the field of view the optimiser
    saw. In water mode that must be the WATER entry, not the air one.
    """
    _, out = water_solve
    fx = out['camera_matrix'][0][0]
    fitted = 2 * math.degrees(math.atan(out['image_width'] / (2 * fx)))
    assert out['hfov_deg_water'] == pytest.approx(fitted, abs=0.01)
    assert out['hfov_deg_air'] > out['hfov_deg_water']


def test_the_intrinsics_recover_the_camera_they_were_rendered_with(
        water_solve):
    """A negative control for the whole rig.

    Without this the tests above compare two wrong answers to each other and
    still pass.
    """
    _, out = water_solve
    K = out['camera_matrix']
    assert K[0][0] == pytest.approx(_FX, rel=0.02)
    assert K[1][1] == pytest.approx(_FX, rel=0.02)
    assert K[0][2] == pytest.approx(_W / 2, abs=15)
    assert K[1][2] == pytest.approx(_H / 2, abs=15)


# --------------------------------------------- the frames, not the filename
def test_each_medium_gets_its_own_capture_FOLDER(fs):
    from mongla_vision.calibration import guide
    assert guide.capture_dir('pi_forward', 'air') != \
        guide.capture_dir('pi_forward', 'water')
    assert guide.capture_dir('pi_forward', 'water').endswith('_water')


def test_switching_to_water_does_not_resume_the_AIR_capture():
    """⛔ The defect this exists for produces a CORRECT-LOOKING file.

    Solve installs `..._water.json`; the frames inside it were taken in air.
    Every naming and file-content test above passes, because the artifact is
    right and the INPUT is wrong -- the `_srot_drive` shape again. So the
    guard has to be on the argv the re-exec runs with.
    """
    from mongla_vision.calibration import guide
    argv = ['calibrate', '--device', '3', '--applies-to', 'pi_forward',
            '--out', guide.capture_dir('pi_forward', 'air')]
    new = guide.restart_argv(argv, 'pi_forward', 'air', medium='water')
    out = new[new.index('--out') + 1]
    assert out == guide.capture_dir('pi_forward', 'water')
    assert out != guide.capture_dir('pi_forward', 'air')
    assert new[new.index('--medium') + 1] == 'water'


def test_a_camera_switch_still_keeps_the_current_medium():
    from mongla_vision.calibration import guide
    new = guide.restart_argv(
        ['calibrate', '--medium', 'water'], 'pi_forward', 'water',
        device='5', applies_to='pi_downward')
    assert new[new.index('--out') + 1] == \
        guide.capture_dir('pi_downward', 'water')


def test_the_refraction_matches_OUR_OWN_measured_pair(fs):
    """Tie the conversion to a measurement, not to itself.

    Round 25 measured this camera at 63.8 deg air / 46.7 deg water +/-0.7,
    held-out validated. Feeding the water figure in must give the air one
    back. A sign flip or a wrong refractive index fails here and nowhere
    else -- the synthetic 70/43 pair above only checks the code against
    itself.
    """
    out, _ = fs.fov_for_medium(
        {'hfov_air': 46.72, 'vfov_air': 30.0,
         'hfov_water': 0.0, 'vfov_water': 0.0}, 'water')
    assert out['hfov_air'] == pytest.approx(63.8, abs=0.7)
    assert out['hfov_water'] == pytest.approx(46.72)
