"""The calibration LIBRARY: keep every calibration, and re-apply one.

⛔ WHY THIS EXISTS. A calibration costs an operator ten minutes, a printed
board and a lit room. On competition ground it costs those at the worst
possible moment. So every calibration this vehicle has ever produced is kept
in `mongla_vision/config/calibration/`, listed by the tool, and re-appliable
to a connected camera in one click.

Two invariants are the whole point of the tests below, and both are about
NOT LYING IN A FILE:

1. **Applying is an assertion, not a measurement.** The operator is saying
   "this is the same physical unit". The file must record that as
   `identity_source: operator asserted at apply time`, never as though the
   calibration had been captured on this camera. The day somebody asks why a
   calibration claims a camera it never saw, that field is the answer.

2. **Air and water never mix.** They differ by ~1.44x on this hull. A water
   calibration standing in for an air one is a 44 % scale error on every
   range and velocity the vehicle computes -- silent, and exactly the size
   that still looks plausible. The medium is in the file AND in the
   filename, so the two cannot overwrite each other.
"""
import json
import os

import pytest

pytest.importorskip('cv2')


@pytest.fixture(scope='module')
def g():
    from mongla_vision.calibration import guide
    return guide


def _write(d, name, **over):
    doc = {'image_width': 1280, 'image_height': 720,
           'camera_matrix': [[851.2, 0, 675.4], [0, 858.1, 290.3], [0, 0, 1]],
           'distortion_coefficients': [[0.0] * 5],
           'applies_to': ['pi_forward'], 'medium': 'air',
           'captured': '2026-09-07 12:00', 'camera': 'Fantech (usb 1bcf:2c99)',
           'views_used': 23, 'max_ere_px': 14.47, 'holdout_rms_px': 0.42,
           'hfov_deg_air': 73.9, 'hfov_deg_water': 53.6}
    doc.update(over)
    p = os.path.join(d, name)
    json.dump(doc, open(p, 'w'))
    return p


def test_library_lists_every_saved_calibration(g, tmp_path):
    d = str(tmp_path)
    _write(d, 'pi_forward_1280x720.json')
    _write(d, 'pi_downward_1280x720_water.json',
           applies_to=['pi_downward'], medium='water')
    lib = g.library(d)
    assert len(lib) == 2
    by = {e['file']: e for e in lib}
    assert by['pi_forward_1280x720.json']['medium'] == 'air'
    assert by['pi_downward_1280x720_water.json']['medium'] == 'water'
    # The fields an operator picks a calibration BY, not just its name.
    e = by['pi_forward_1280x720.json']
    assert e['fx'] == pytest.approx(851.2)
    assert e['views'] == 23 and e['max_ere_px'] == pytest.approx(14.47)


def test_library_skips_a_corrupt_file_rather_than_dying(g, tmp_path):
    d = str(tmp_path)
    _write(d, 'good_1280x720.json')
    open(os.path.join(d, 'broken.json'), 'w').write('{not json')
    # One bad file must not take the whole panel down -- the panel is how
    # the operator finds the GOOD ones.
    assert [e['file'] for e in g.library(d)] == ['good_1280x720.json']


def test_apply_stamps_the_identity_as_OPERATOR_ASSERTED(g, tmp_path,
                                                        monkeypatch):
    d = str(tmp_path)
    _write(d, 'pi_forward_1280x720.json')
    monkeypatch.setattr(g, 'camera_identity', lambda dev: {
        'usb_vid': '1bcf', 'usb_pid': '2c99', 'usb_serial': 'X1',
        'card': 'Other Cam', 'bus': 'usb-xhci-1'})
    name, err = g.apply_calibration(d, 'pi_forward_1280x720.json',
                                    '/dev/video7', 'pi_downward')
    assert err is None
    doc = json.load(open(os.path.join(d, name)))
    assert doc['identity_source'] == 'operator asserted at apply time'
    assert doc['applied_from'] == 'pi_forward_1280x720.json'
    assert doc['usb_pid'] == '2c99' and doc['card'] == 'Other Cam'
    assert doc['applies_to'] == ['pi_downward']
    # The intrinsics are the point -- they must survive unchanged.
    assert doc['camera_matrix'][0][0] == pytest.approx(851.2)


def test_apply_refuses_a_camera_with_no_usb_identity(g, tmp_path,
                                                     monkeypatch):
    d = str(tmp_path)
    _write(d, 'pi_forward_1280x720.json')
    monkeypatch.setattr(g, 'camera_identity', lambda dev: {})
    name, err = g.apply_calibration(d, 'pi_forward_1280x720.json',
                                    '/dev/video7', 'pi_downward')
    assert name is None and 'identity' in err
    # And it wrote nothing: a refusal that leaves a file behind is not a
    # refusal.
    assert os.listdir(d) == ['pi_forward_1280x720.json']


def test_apply_keeps_the_medium_in_the_destination_name(g, tmp_path,
                                                        monkeypatch):
    d = str(tmp_path)
    _write(d, 'pi_forward_1280x720_water.json', medium='water')
    monkeypatch.setattr(g, 'camera_identity', lambda dev: {'usb_vid': 'a'})
    name, _ = g.apply_calibration(d, 'pi_forward_1280x720_water.json',
                                  '/dev/video7', 'pi_downward')
    assert name == 'pi_downward_1280x720_water.json'


def test_apply_backs_up_rather_than_destroying_an_existing_calibration(
        g, tmp_path, monkeypatch):
    d = str(tmp_path)
    _write(d, 'src_1280x720.json')
    _write(d, 'pi_downward_1280x720.json',
           applies_to=['pi_downward'],
           camera_matrix=[[111.0, 0, 1], [0, 111.0, 1], [0, 0, 1]])
    monkeypatch.setattr(g, 'camera_identity', lambda dev: {'usb_vid': 'a'})
    g.apply_calibration(d, 'src_1280x720.json', '/dev/video7', 'pi_downward')
    baks = [f for f in os.listdir(d) if '.bak-' in f]
    assert len(baks) == 1
    old = json.load(open(os.path.join(d, baks[0])))
    assert old['camera_matrix'][0][0] == pytest.approx(111.0)


def test_the_shipped_calibrations_are_all_loadable_and_named_by_medium(g):
    """The library on disk is the deliverable -- guard it, not a fixture."""
    # Resolved by the SHIPPED resolver, so this also guards the thing that
    # made `/calibration.json` 404 once: the directory being right in a
    # source tree and wrong when installed.
    lib = g.library(g.default_calibration_dir())
    assert lib, 'mongla_vision ships no calibrations'
    for e in lib:
        assert e['fx'] and e['fx'] > 0, e['file']
        assert e['applies_to'], e['file']
        if e['medium'] != 'air':
            assert e['medium'] in e['file'], (
                f"{e['file']} is a {e['medium']} calibration whose name does "
                'not say so -- it can overwrite the air one')


def test_the_guide_never_imports_its_own_PACKAGE(g):
    """⛔ The calibration tool must run without a ROS environment.

    `tools/fov_calibrate_web.py` loads this module BY PATH for exactly that
    reason. A `from mongla_vision...` anywhere in it -- including inside a
    function, which is where it slipped in -- re-enters
    `mongla_vision/__init__.py`, which imports `preflight`, which imports
    `rclpy`, and the tool dies on any box without ROS sourced. That is not
    hypothetical: it happened on the first live run of the library panel,
    and the traceback named `ModuleNotFoundError: rclpy`.

    Parsed, not grepped -- a substring search matches this docstring.
    """
    import ast
    tree = ast.parse(open(g.__file__).read())
    bad = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            bad += [n.name for n in node.names
                    if n.name.split('.')[0] == 'mongla_vision']
        elif isinstance(node, ast.ImportFrom):
            if (node.module or '').split('.')[0] == 'mongla_vision':
                bad.append(node.module)
    assert not bad, (
        f'guide.py imports the package ({bad}) -- that drags in rclpy and '
        'breaks the ROS-free tools/ entry point. Load by path instead.')


def _fake_ws(root):
    """A colcon workspace shaped like the vehicle's, decoy and all."""
    real = os.path.join(root, 'src', 'mongla_vision', 'config', 'calibration')
    os.makedirs(real)
    # The decoy the tool created for itself on the Pi: an EMPTY
    # src/mongla_vision/config/calibration INSIDE the build tree, sitting
    # nearer to __file__ than the real one.
    pkg = os.path.join(root, 'build', 'mongla_vision', 'mongla_vision')
    os.makedirs(os.path.join(pkg, 'src', 'mongla_vision', 'config',
                             'calibration'))
    os.makedirs(os.path.join(pkg, 'calibration'))
    return real, os.path.join(pkg, 'calibration')


def test_the_calibration_dir_is_never_inside_a_GENERATED_tree(
        g, tmp_path, monkeypatch):
    """⛔ Reproduces the failure measured on the vehicle, 2026-09-07.

    Under `ros2 run` the resolver returned
    `<ws>/build/mongla_vision/mongla_vision/src/mongla_vision/config/
    calibration` -- an empty directory the tool had created ITSELF on an
    earlier run, because `main()` does `os.makedirs` on whatever this
    returns. Every symptom (library empty, /calibration.json 404, cameras
    reading NO CALIBRATION) pointed at missing calibrations rather than a
    wrong path.

    `build/` and `install/` are wiped by the next `colcon build`, so a
    calibration written there is lost and can never be committed.
    """
    root = str(tmp_path / 'mongla_ws')
    real, guide_home = _fake_ws(root)
    monkeypatch.setattr(g, '__file__', os.path.join(guide_home, 'guide.py'))
    monkeypatch.chdir(tmp_path)
    got = g.default_calibration_dir()
    assert os.path.realpath(got) == os.path.realpath(real), got
    parts = os.path.abspath(got).split(os.sep)
    assert 'build' not in parts and 'install' not in parts


def test_the_installed_layout_finds_the_source_tree(g, tmp_path, monkeypatch):
    """The `install/.../site-packages` layout, which is what `ros2 run` uses."""
    root = str(tmp_path / 'mongla_ws')
    real, _ = _fake_ws(root)
    home = os.path.join(root, 'install', 'mongla_vision', 'lib',
                        'python3.12', 'site-packages', 'mongla_vision',
                        'calibration')
    os.makedirs(home)
    monkeypatch.setattr(g, '__file__', os.path.join(home, 'guide.py'))
    monkeypatch.chdir(tmp_path)
    assert os.path.realpath(g.default_calibration_dir()) == \
        os.path.realpath(real)
