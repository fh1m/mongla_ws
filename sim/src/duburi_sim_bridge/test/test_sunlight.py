"""Moving caustics and surface glare.

These are the THIRD effect in `underwater_fx` that exists because the renderer
would not do it -- after `<scene><fog>` and the particle emitter, and now
`LensFlare`, which attaches to the camera (gz prints "Lens flare attached to
camera named" at -v 4) and produces no measurable output in this scene.
"""

import numpy as np
import pytest

from duburi_sim_bridge.underwater_fx import (CAMERA_HFOV, SunlightField,
                                             SurfaceGlare)


def _flat(v=0.5, h=120, w=160):
    return np.full((h, w, 3), v, dtype=np.float32)


def _rng(r=4.0, h=120, w=160):
    return np.full((h, w), r, dtype=np.float32)


def test_caustics_never_produce_negative_light():
    """There is no such thing as negative light, and the first version made it.

    The raw wave sum runs symmetrically about zero; used directly it drove the
    frame gain to -0.082, i.e. an inverted image. Light FOCUSES where the
    surface is concave and merely thins where it is convex.
    """
    f = SunlightField()
    for t in (0.0, 0.7, 3.3, 11.0):
        out = f.apply(_flat(), _rng(), (0.0, 0.0, -1.0, 0.0), t, 1.5, CAMERA_HFOV)
        assert out.min() > 0.0, f'negative light at t={t}'


def test_caustics_redistribute_light_rather_than_adding_it():
    """Turning caustics on must not silently expose the whole frame up."""
    f = SunlightField()
    img = _flat()
    out = f.apply(img.copy(), _rng(), (0.0, 0.0, -1.0, 0.0), 1.0, 0.6, CAMERA_HFOV)
    # TIGHT on purpose. At rel=0.06 this passed with the mean-centring removed
    # entirely -- the very defect it exists to catch. Caustics REDISTRIBUTE
    # light; a DC shift means turning them on silently exposes the frame.
    assert out.mean() == pytest.approx(img.mean(), rel=0.01)


def test_caustics_animate():
    """A baked pattern gives zero here. That difference IS the feature."""
    f = SunlightField()
    a = f.apply(_flat(), _rng(), (0.0, 0.0, -1.0, 0.0), 0.0, 0.6, CAMERA_HFOV)
    b = f.apply(_flat(), _rng(), (0.0, 0.0, -1.0, 0.0), 1.7, 0.6, CAMERA_HFOV)
    assert np.abs(a - b).mean() > 0.01


def test_caustics_are_anchored_to_the_WORLD_not_the_camera():
    """The whole difficulty. A pattern painted in image space swims with the
    camera and reads as a dirty lens; real caustics stay put and the vehicle
    moves through them."""
    f = SunlightField()
    here = f.apply(_flat(), _rng(), (0.0, 0.0, -1.0, 0.0), 0.0, 0.6, CAMERA_HFOV)
    there = f.apply(_flat(), _rng(), (5.0, 0.0, -1.0, 0.0), 0.0, 0.6, CAMERA_HFOV)
    assert np.abs(here - there).mean() > 0.01, 'pattern followed the camera'


def test_caustics_fade_with_distance():
    """Scattering blurs the net out long before the geometry disappears."""
    f = SunlightField()
    near = f.apply(_flat(), _rng(1.5), (0.0, 0.0, -1.0, 0.0), 0.0, 0.6, CAMERA_HFOV)
    far = f.apply(_flat(), _rng(18.0), (0.0, 0.0, -1.0, 0.0), 0.0, 0.6, CAMERA_HFOV)
    # MODULATION DEPTH, not std: the two frames sample the pattern at different
    # world scales, so their spatial statistics differ for reasons that have
    # nothing to do with the fade. Peak-to-trough is the thing the fade acts on,
    # and comparing std alone passed with the fade removed entirely.
    depth_near = float(near.max() - near.min())
    depth_far = float(far.max() - far.min())
    assert depth_near > depth_far * 1.5, (
        f'fade not applied: near {depth_near:.3f} vs far {depth_far:.3f}')


def test_everything_is_a_no_op_at_zero_strength():
    """Off must mean off -- these ship enabled, so the off path is the one a
    session uses to get a clean reference frame."""
    f = SunlightField()
    img = _flat()
    assert np.array_equal(f.apply(img.copy(), _rng(), (0, 0, -1, 0), 1.0, 0.0,
                                  CAMERA_HFOV), img)
    assert np.array_equal(SurfaceGlare().apply(img.copy(), _rng(), 0.0), img)


def test_no_range_image_means_no_caustics_rather_than_a_crash():
    """The depth cameras can be off; degrade, do not fail."""
    f = SunlightField()
    img = _flat()
    assert np.array_equal(f.apply(img.copy(), None, (0, 0, -1, 0), 1.0, 0.6,
                                  CAMERA_HFOV), img)


def test_glare_only_blooms_far_bright_things():
    """A prop 40 cm from the lens is bright and must NOT bloom; the surface at
    range is what does."""
    g = SurfaceGlare()
    img = _flat(0.3)
    img[10:30, 10:30] = 0.99                      # a bright patch
    near = g.apply(img.copy(), _rng(0.8), 0.6)
    far = g.apply(img.copy(), _rng(12.0), 0.6)
    assert far.mean() > near.mean()
