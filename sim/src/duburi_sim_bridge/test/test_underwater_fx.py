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


def test_gt_label_tables_cover_the_same_props():
    """A prop in one table but not the other is silently unlabelled.

    sauvc_target_mat was added to MODEL_TO_CLASS without half-extents; the
    projector skipped it at a `half is None` guard, classes.txt still listed
    target_mat, and 1469 recorded frames carried zero mat instances. The dataset
    looked complete and the class was never learnable. gt_labels now raises at
    import; this test states the invariant so it is not "fixed" back out.
    """
    from duburi_sim_bridge import gt_labels

    assert set(gt_labels.MODEL_TO_CLASS) == set(gt_labels.PROP_HALF_EXTENTS)
    for name in gt_labels.MODEL_TO_CLASS.values():
        assert name in gt_labels.CLASSES
