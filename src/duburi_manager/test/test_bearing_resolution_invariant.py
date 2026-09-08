"""B16 -- where the calibration-resolution defence actually lives, pinned.

`bearing.py` claimed IT scaled K for a resolution mismatch. It never did, and it
takes no calibration resolution to scale against. The register recorded that as a
missing defence; reading the callsites shows the defence EXISTS, one layer up,
and the defect was the comment pointing at the wrong file.

Two halves make a resolution mismatch unreachable, and both are asserted here so
that a future refactor cannot quietly remove one:

  1. `camera_node._fill_calibration` rescales fx/cx/fy/cy by the streamed/
     calibrated ratio before publishing CameraInfo.
  2. `VisionState._on_info` reads `k` and `width`/`height` from the SAME message,
     so the K a bearing is computed from always matches the size it is given.

If either stops being true, bearings are wrong by exactly the resolution ratio --
"every derived angle wrong by 2x, silently", in camera_node's own words.
"""

import ast
import inspect
import pathlib

import pytest


def _src(mod_relpath, func_name):
    """Read a function's source WITHOUT importing (camera_node needs cv2/ROS)."""
    root = pathlib.Path(__file__).resolve().parents[3]
    tree = ast.parse((root / mod_relpath).read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == func_name:
            return ast.get_source_segment((root / mod_relpath).read_text(encoding='utf-8'), node)
    raise AssertionError(f'{func_name} not found in {mod_relpath}')


def test_camera_node_still_rescales_K_to_the_streamed_resolution():
    body = _src('src/duburi_vision/duburi_vision/camera_node.py', '_fill_calibration')
    assert "sx = info.width / cal['w']" in body, 'the x resolution ratio is gone'
    assert "sy = info.height / cal['h']" in body, 'the y resolution ratio is gone'
    # fx, cx scale with x; fy, cy with y. All four, or the principal point walks.
    for idx, factor in ((0, 'sx'), (2, 'sx'), (4, 'sy'), (5, 'sy')):
        assert f'K[{idx}] *= {factor}' in body, f'K[{idx}] no longer scaled by {factor}'


def test_vision_state_reads_K_and_size_from_one_message():
    """They must be assigned in the same callback, or they can drift apart."""
    body = _src('src/duburi_manager/duburi_manager/vision_state.py', '_on_info')
    assert '_image_size' in body and '_K' in body, \
        'K and image_size must be latched together from one CameraInfo'


def test_bearing_does_not_silently_rescale():
    """The scaling belongs upstream; a second copy is a second way to be wrong."""
    from duburi_control import bearing
    body = inspect.getsource(bearing.bearing_from_pixels)
    code = '\n'.join(l.split('#', 1)[0] for l in body.splitlines())
    assert 'calib_w' not in code and 'calib_size' not in code, \
        'bearing.py grew a calibration-resolution argument; if that is deliberate, ' \
        'camera_node must stop scaling or the ratio is applied twice'


def test_bearing_math_is_unchanged_by_a_matched_rescale():
    """The real invariant, end to end: scaling K with the frame is a no-op on angle.

    A 1280x720 calibration streamed at 640x360 must give the SAME bearing, which
    is exactly what makes camera_node's rescale correct rather than merely present.
    """
    from duburi_control.bearing import bearing_from_pixels

    K_full = [1027.87, 0.0, 617.32, 0.0, 1027.87, 360.0, 0.0, 0.0, 1.0]
    full = bearing_from_pixels(800.0, 500.0, 100.0, 80.0,
                               width=1280, height=720, K=K_full)
    K_half = list(K_full)
    for i in (0, 2, 4, 5):
        K_half[i] /= 2.0
    half = bearing_from_pixels(400.0, 250.0, 50.0, 40.0,
                               width=640, height=360, K=K_half)

    assert full is not None and half is not None
    assert half.angle_x == pytest.approx(full.angle_x, abs=1e-12)
    assert half.angle_y == pytest.approx(full.angle_y, abs=1e-12)
    assert half.size_x == pytest.approx(full.size_x, abs=1e-12)
