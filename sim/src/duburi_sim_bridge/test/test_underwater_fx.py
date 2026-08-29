"""underwater_fx: the optimised path must still be the documented filter.

apply_underwater_fx was rewritten for speed (~35 ms -> ~4 ms per 640x480
frame) because it was starving Gazebo's render loop. Speed work is exactly the
kind that silently changes output, so the first test pins the maths against a
literal transcription of the original formula rather than against a golden
image -- a golden image would drift with any deliberate retune, while the
equivalence below only breaks if the refactor changed the filter.
"""

import numpy as np
import pytest

from duburi_sim_bridge.underwater_fx import (
    ATTEN_RGB,
    HAZE_RGB,
    apply_underwater_fx,
)


def _reference(bgr, depth_m, turbidity, backscatter, atten_scale):
    """The pre-optimisation formula, transcribed verbatim."""
    depth = max(0.05, abs(float(depth_m)))
    t = max(0.0, float(turbidity))
    img = bgr.astype(np.float32) / 255.0
    path = depth * (0.35 + 0.65 * t)
    atten = np.exp(-ATTEN_RGB * path * atten_scale).reshape(1, 1, 3)
    haze = HAZE_RGB.reshape(1, 1, 3)
    scatter = np.clip(backscatter * t * (1.0 - np.exp(-0.4 * path)), 0.0, 1.0)
    out = img * atten * (1.0 - scatter) + haze * scatter
    mean = out.mean(axis=(0, 1), keepdims=True)
    out = mean + (out - mean) * (1.0 - 0.35 * t)
    return np.clip(out * 255.0, 0, 255).astype(np.uint8)


@pytest.fixture
def frame():
    return np.random.default_rng(7).integers(0, 256, (96, 128, 3), dtype=np.uint8)


@pytest.mark.parametrize('depth,turb,back,scale', [
    (-1.0, 0.80, 0.75, 1.8),   # murky preset
    (-1.0, 0.45, 0.55, 1.0),   # competition preset
    (-0.4, 0.15, 0.30, 0.6),   # clear preset
    (-1.6, 0.00, 0.00, 1.0),   # degenerate: no turbidity at all
])
def test_lut_path_matches_the_original_formula(frame, depth, turb, back, scale):
    """Blur/noise/vignette off, so the comparison is exact up to rounding.

    The attenuate/haze/contrast chain is per-channel affine in the uint8 input,
    so collapsing it into a 256-entry LUT is algebra, not approximation. One
    LSB is the float->uint8 rounding boundary and nothing more.
    """
    got = apply_underwater_fx(frame, depth, turb, back, 0.0, 0.0, 0.0, scale)
    want = _reference(frame, depth, turb, back, scale)
    assert np.abs(got.astype(int) - want.astype(int)).max() <= 1


def test_effects_stay_in_range_and_shape(frame):
    out = apply_underwater_fx(frame, -1.0, 0.8, 0.75, 1.4, 0.02, 0.35, 1.8)
    assert out.shape == frame.shape and out.dtype == np.uint8


def test_successive_frames_do_not_share_a_noise_pattern(frame):
    """The noise bank picks a tile and rolls it; a fixed pattern would bake a
    static artefact into every training image drawn from this sim."""
    a = apply_underwater_fx(frame, -1.0, 0.5, 0.0, 0.0, 0.05, 0.0, 1.0)
    b = apply_underwater_fx(frame, -1.0, 0.5, 0.0, 0.0, 0.05, 0.0, 1.0)
    assert not np.array_equal(a, b)


def test_turbidity_actually_reduces_contrast(frame):
    """Ordering, not absolute numbers -- the presets are pool-tunable."""
    clear = apply_underwater_fx(frame, -1.0, 0.15, 0.30, 0.0, 0.0, 0.0, 0.6)
    murky = apply_underwater_fx(frame, -1.0, 0.80, 0.75, 0.0, 0.0, 0.0, 1.8)
    assert murky.std() < clear.std()


def test_every_registered_prop_has_a_detection_label():
    """A prop without a label is INVISIBLE to the bounding-box camera.

    Gazebo emits a box only for an entity carrying a SemanticLabel, and it says
    nothing about one that has none -- so an unlabelled prop is simply absent
    from every frame's annotations, with no warning anywhere. That is the same
    silent-empty-labels failure that once put target_mat in one table and not
    the other; this is its replacement guard.
    """
    pl = _prop_library()
    unlabelled = [n for n in pl.PROPS if pl.detection_label(n) == 0]
    assert not unlabelled, (
        f'props with no detection label, invisible to the box camera: '
        f'{unlabelled}')


def test_class_zero_stays_reserved():
    """gz-sim treats an unlabelled entity as label 0.

    If a real class ever took slot 0 it would be indistinguishable from
    background, and every unlabelled thing in the scene would annotate as it.
    """
    pl = _prop_library()
    assert pl.DETECTION_CLASSES[0] == '_background'
    assert 0 not in [pl.detection_label(n) for n in pl.PROPS]


def test_detection_classes_are_unique_and_cover_both_competitions():
    pl = _prop_library()
    assert len(set(pl.DETECTION_CLASSES)) == len(pl.DETECTION_CLASSES)
    assert any(c.startswith('sauvc_') for c in pl.DETECTION_CLASSES)
    assert any(c.startswith('robosub_') for c in pl.DETECTION_CLASSES)


def test_boxes_to_yolo_normalises_and_drops_background():
    """The class id arrives as a STRING and 0 means background."""
    from duburi_sim_bridge import box_labels

    class _H:
        def __init__(self, c): self.class_id = c
    class _R:
        def __init__(self, c): self.hypothesis = _H(c)
    class _P:
        def __init__(self, x, y): self.position = type('p', (), {'x': x, 'y': y})()
    class _B:
        def __init__(self, x, y, w, h):
            self.center, self.size_x, self.size_y = _P(x, y), w, h
    class _D:
        def __init__(self, c, x, y, w, h):
            self.results, self.bbox = [_R(c)], _B(x, y, w, h)
    class _M:
        def __init__(self, dets): self.detections = dets

    rows = box_labels.boxes_to_yolo(
        _M([_D('3', 320, 240, 64, 48),      # good
            _D('0', 100, 100, 64, 48),      # background -> dropped
            _D('3', 10, 10, 1, 1)]),        # sub-pixel -> dropped
        640, 480)
    assert len(rows) == 1
    cid, xc, yc, w, h = rows[0]
    assert cid == 3
    assert (xc, yc) == pytest.approx((0.5, 0.5))
    assert (w, h) == pytest.approx((0.1, 0.1))


def _prop_library():
    import importlib.util, os, sys
    from ament_index_python.packages import get_package_share_directory
    scripts = os.path.join(
        get_package_share_directory('duburi_sim_worlds'), 'scripts')
    sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location(
        'prop_library', os.path.join(scripts, 'prop_library.py'))
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault('prop_library', mod)
    spec.loader.exec_module(mod)
    return mod
