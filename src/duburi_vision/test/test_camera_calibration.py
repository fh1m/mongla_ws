"""camera_node's CameraInfo calibration path.

Guards three things, one of which fails SILENTLY if it regresses:

  * with no calibration, K stays zero -- the historical behaviour must be
    byte-identical, because every camera without a calibration file still has
    to stream;
  * intrinsics are SCALED to the streaming resolution. A calibration taken at
    1280x720 and published unscaled at 640x360 puts the principal point outside
    the image and doubles every angle derived from it, with no error anywhere.
    That is the one worth a test;
  * an unreadable calibration warns and degrades, it does not raise.

Runs without ROS or a camera: the two methods are extracted and bound to a
stub, so the test stays fast and has no rclpy dependency.
"""
import importlib.util
import json
import os
import tempfile
import textwrap
import types

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE = os.path.join(_HERE, '..', 'duburi_vision', 'camera_node.py')

CAL = {
    "image_width": 1280, "image_height": 720,
    "camera_matrix": [[600.0, 0.0, 640.0], [0.0, 600.0, 360.0], [0.0, 0.0, 1.0]],
    "distortion_coefficients": [0.1, -0.2, 0.0, 0.0, 0.05],
    "hfov_deg_air": 93.7,
}


def _real_resolver():
    """binding.calibration_for_profile, loaded BY PATH (never imported)."""
    src = os.path.join(_HERE, '..', 'duburi_vision', 'calibration', 'binding.py')
    spec = importlib.util.spec_from_file_location('_binding_for_camtest', src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.calibration_for_profile


def _methods():
    src = open(_NODE).read()
    body = textwrap.dedent(
        src[src.index("    def _load_calibration"):src.index("    def _handle_video_pause")])
    # `_load_calibration` now resolves from the profile when `calibration`
    # is unset, so the extracted body needs that name. Bound to the REAL
    # resolver, not a stub: a fake here would let the two drift and this
    # file's whole point is that the extraction stays faithful.
    ns = {'os': os, 'CameraInfo': object,
          'calibration_for_profile': _real_resolver()}
    exec(compile(body, 'camera_node', 'exec'), ns)
    return ns['_load_calibration'], ns['_fill_calibration']


class _Info:
    def __init__(self, w, h):
        self.width, self.height = w, h
        self.k = [0.0] * 9
        self.d = []
        self.p = [0.0] * 12
        self.r = []
        self.distortion_model = ''


class _Stub:
    """`get_parameter` used to ignore the name and answer every query with
    the calibration path. Harmless while one parameter was read; now
    `_load_calibration` reads `profile` too, and a stub that cannot tell
    them apart would answer the profile query with a file path and test a
    code path that cannot occur."""

    def __init__(self, path, profile=''):
        self._p = path
        self._profile = profile
        self._calib = None
        self.warned = []

    def get_parameter(self, name):
        return types.SimpleNamespace(
            value=self._profile if name == 'profile' else self._p)

    def get_logger(self):
        return types.SimpleNamespace(info=lambda m: None,
                                     warn=self.warned.append)


@pytest.fixture
def cal_file():
    f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
    json.dump(CAL, f)
    f.close()
    yield f.name
    os.unlink(f.name)


def test_no_calibration_AND_no_profile_leaves_camera_info_untouched():
    """The historical guarantee, now stated precisely.

    It used to read "no calibration parameter -> K stays zero". That is no
    longer the whole truth: an unset `calibration` is resolved from the
    PROFILE (see calibration/binding.py), because passing it explicitly in
    every launch file is what left `vision.launch.py` publishing k=0 on the
    mission path. The guarantee that survives -- and the one that matters --
    is that a camera NOBODY has calibrated still streams, with K left zero
    rather than borrowed from another lens."""
    load, fill = _methods()
    s = _Stub('', profile='')
    s._calib = load(s)
    info = _Info(640, 360)
    fill(s, info)
    assert s._calib is None
    assert info.k == [0.0] * 9, 'a camera with no calibration must be unchanged'


def test_an_uncalibrated_PROFILE_still_leaves_camera_info_untouched():
    """`forward` is a real profile with no calibration (it is the Jetson's
    Blue Robotics unit, not the Pi's Fantech). It must resolve to nothing
    rather than to the similarly-named `pi_forward` file."""
    load, fill = _methods()
    s = _Stub('', profile='forward')
    s._calib = load(s)
    info = _Info(640, 360)
    fill(s, info)
    assert s._calib is None, (
        "profile 'forward' picked up a calibration; no file declares it, so "
        "this can only be a name-similarity guess")
    assert info.k == [0.0] * 9


def test_an_unset_calibration_is_RESOLVED_from_a_known_profile():
    """The fix, pinned. This is what makes bringup publish a real K."""
    load, fill = _methods()
    s = _Stub('', profile='pi_forward')
    s._calib = load(s)
    assert s._calib is not None, (
        "profile 'pi_forward' did not resolve to its calibration -- the "
        "bringup path is back to publishing k=0 and nothing will say so")
    info = _Info(640, 360)
    fill(s, info)
    assert info.k[0] > 0.0, 'fx must be non-zero once a profile resolves'
    assert info.k[2] != 320.0, (
        'cx landed exactly on the frame centre, which is the value the '
        'FOV fallback invents -- the measured off-axis principal point was '
        'not applied')


def test_same_resolution_passes_intrinsics_through(cal_file):
    load, fill = _methods()
    s = _Stub(cal_file)
    s._calib = load(s)
    info = _Info(1280, 720)
    fill(s, info)
    assert info.k[0] == pytest.approx(600.0)
    assert info.k[2] == pytest.approx(640.0)
    assert info.distortion_model == 'plumb_bob'


def test_intrinsics_are_scaled_to_the_streaming_resolution(cal_file):
    """The silent one: unscaled K puts cx outside a 640-wide image."""
    load, fill = _methods()
    s = _Stub(cal_file)
    s._calib = load(s)
    info = _Info(640, 360)
    fill(s, info)
    assert info.k[0] == pytest.approx(300.0), 'fx not scaled'
    assert info.k[4] == pytest.approx(300.0), 'fy not scaled'
    assert info.k[2] == pytest.approx(320.0), 'cx not scaled'
    assert info.k[5] == pytest.approx(180.0), 'cy not scaled'
    assert info.k[2] < info.width, 'principal point fell outside the image'


def test_field_of_view_survives_the_rescale(cal_file):
    """Scaling must preserve the ANGLE -- that is the point of scaling at all."""
    import math
    load, fill = _methods()
    s = _Stub(cal_file)
    s._calib = load(s)
    fovs = []
    for w, h in ((1280, 720), (640, 360), (320, 180)):
        info = _Info(w, h)
        fill(s, info)
        fovs.append(2 * math.degrees(math.atan(w / (2 * info.k[0]))))
    assert max(fovs) - min(fovs) < 1e-9, f'FOV changed with resolution: {fovs}'


def test_unreadable_calibration_warns_and_degrades():
    """A vehicle that will not stream video is worse than one without K."""
    load, _ = _methods()
    s = _Stub('/nonexistent/definitely-not-here.json')
    s._calib = load(s)
    assert s._calib is None
    assert s.warned, 'a missing calibration must be reported, not swallowed'
