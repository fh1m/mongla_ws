"""Suspended particulate in `underwater_fx`.

This layer exists only because Gazebo's own particle emitter is INVISIBLE to
camera sensors -- measured with 0.4 m particles at 4000/s against a confirmed
live emitter, and the sensor frame was unchanged. So these assertions are the
only thing standing between "the vision pipeline sees particulate" and a
silently empty feature.
"""

import numpy as np

from duburi_sim_bridge.underwater_fx import ParticleField, apply_underwater_fx

FX = dict(depth_m=-1.0, turbidity=0.45, backscatter=0.55, blur_sigma=0.8,
          noise=0.0, vignette=0.25, atten_scale=1.0)


def _flat(value: int = 90) -> np.ndarray:
    return np.full((240, 320, 3), value, np.uint8)


def test_particulate_actually_changes_the_frame():
    base = _flat()
    off = apply_underwater_fx(base, **FX)
    on = apply_underwater_fx(base, **FX,
                             particles=ParticleField().render(base.shape, 0.35))
    changed = int((np.abs(on.astype(int) - off.astype(int)).max(axis=2) > 4).sum())
    assert changed > 200, f'particulate is invisible ({changed} px changed)'


def test_zero_strength_is_a_true_no_op():
    base = _flat()
    off = apply_underwater_fx(base, **FX)
    zero = apply_underwater_fx(base, **FX,
                               particles=ParticleField().render(base.shape, 0.0))
    assert np.array_equal(zero, off)


def test_field_drifts_but_stays_coherent():
    """The point of the field is temporal coherence.

    A speckle redrawn every frame is just `noise`, which the filter already
    has and which a detector ignores. Particles must persist and move a
    little -- both halves matter, so both are asserted.
    """
    shape = (240, 320, 3)
    f = ParticleField()
    before = f.render(shape, 0.35)
    f.step(0.1)
    after = f.render(shape, 0.35)

    moved = float(np.abs(after - before).sum())
    unrelated = float(np.abs(ParticleField(seed=99).render(shape, 0.35)
                             - before).sum())

    assert moved > 1.0, 'field is frozen -- a static decal, not drifting matter'
    assert moved < unrelated * 0.35, (
        'field resampled rather than drifted: a particle moved further than '
        'its own radius in 100 ms, so consecutive frames share no particles')


def test_step_is_bounded_against_a_stalled_frame():
    """A long gap between frames must not teleport the whole field."""
    shape = (240, 320, 3)
    f = ParticleField()
    before = f.render(shape, 0.35)
    f.step(45.0)
    assert np.isfinite(f.xy).all()
    assert (f.xy >= -0.06).all() and (f.xy <= 1.06).all()
    assert float(np.abs(f.render(shape, 0.35) - before).sum()) > 0.0
