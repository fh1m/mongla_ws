"""The floor as an instrument -- and the negative controls that shaped it.

⛔ READ THE NEGATIVES FIRST. Three discriminants were tried against a smooth
illumination gradient, which is what an underwater scene with light falloff
actually looks like. Two of them ACCEPTED it:

    share of spectral energy   35 % for the ramp vs 5 % for a real grid
    angular prominence         377 for the ramp vs 194 for a rotated grid
    harmonic at 2f             2164 for the ramp vs 6 for a real grid

Every one of those numbers looks like a finding. The test that works is where
the peak sits in the RADIAL spectrum: a ramp and a vignette peak at radius 2,
the lowest bin, because they ARE the lowest frequency; a grid peaks at radius 8
with essentially nothing inside it; white noise is flat at ratio 1.02.

So these tests are not decoration. They are the reason the module has one gate
instead of three wrong ones.
"""
import math

import numpy as np
import pytest

from duburi_localization.tile_grating import (
    MIN_PEAK_FRACTION, Grating, detect, shift_from_phase,
)

RNG = np.random.default_rng(0)


def grid(n=256, period=32.0, angle_deg=0.0, phase=0.0, noise=0.0, sine=False):
    """A square pool-tile floor, rotated and noised to order."""
    y, x = np.mgrid[0:n, 0:n].astype(np.float32)
    t = math.radians(angle_deg)
    u = x * math.cos(t) + y * math.sin(t)
    v = -x * math.sin(t) + y * math.cos(t)
    f = (lambda z: np.sin(z)) if sine else (lambda z: np.sign(np.sin(z)))
    im = 128 + 50 * (f(2 * np.pi * u / period + phase)
                     + f(2 * np.pi * v / period + phase))
    if noise:
        im = im + RNG.normal(0, noise, im.shape)
    return np.clip(im, 0, 255).astype(np.uint8)


# --------------------------------------------------------------------------- #
#  NEGATIVE CONTROLS -- each one refused a different wrong design
# --------------------------------------------------------------------------- #
def test_a_smooth_gradient_is_NOT_a_grating():
    """The control that killed two designs. An illumination ramp is a single
    half-cycle; it never repeats, so it is not periodic however much energy it
    concentrates."""
    assert detect(np.tile(np.linspace(0, 255, 256, dtype=np.uint8), (256, 1))) is None


def test_a_diagonal_gradient_is_NOT_a_grating():
    yy, xx = np.mgrid[0:256, 0:256]
    assert detect(((xx + yy) / 2).astype(np.uint8)) is None


def test_a_vignette_is_NOT_a_grating():
    """A dive light's falloff. Radially symmetric, peaks at DC."""
    yy, xx = np.mgrid[0:256, 0:256]
    vig = (200 * np.exp(-((yy - 128) ** 2 + (xx - 128) ** 2) / 2 / 80 ** 2))
    assert detect(vig.astype(np.uint8)) is None


def test_a_gradient_WITH_noise_is_still_refused():
    im = np.tile(np.linspace(0, 255, 256), (256, 1)) + RNG.normal(0, 15, (256, 256))
    assert detect(np.clip(im, 0, 255).astype(np.uint8)) is None


def test_white_noise_is_NOT_a_grating():
    assert detect(RNG.integers(0, 255, (256, 256)).astype(np.uint8)) is None


def test_a_flat_field_is_NOT_a_grating():
    assert detect(np.full((256, 256), 128, np.uint8)) is None


def test_too_small_an_image_is_refused():
    assert detect(np.zeros((16, 16), np.uint8)) is None


def test_a_nan_frame_is_refused_not_propagated():
    im = np.full((64, 64), np.nan, np.float32)
    assert detect(im) is None


# --------------------------------------------------------------------------- #
#  POSITIVE -- scored against KNOWN truth, never against another estimator
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('period', [16.0, 21.0, 24.0, 32.0, 40.0, 48.0])
def test_the_period_is_recovered_within_one_percent(period):
    """Sub-bin interpolation earns its place here: without it a 48 px grid
    reads back as 51.2 px, a 6.67 % error that lands on every height and so on
    every velocity derived from one."""
    g = detect(grid(period=period))
    assert g is not None
    assert abs(g.period_px - period) / period < 0.01


@pytest.mark.parametrize('angle', [0.0, 7.0, 15.0, 30.0, 44.0, 60.0, 75.0])
def test_the_grid_angle_is_recovered_within_one_degree(angle):
    """⛔ THE HEADLINE. This is an ABSOLUTE heading in the pool frame -- tiles
    are laid square to the walls -- obtained with no prop, no detection and no
    landmark in view."""
    g = detect(grid(period=32.0, angle_deg=angle))
    assert g is not None
    err = min(abs(g.angle_deg - angle % 90.0),
              90.0 - abs(g.angle_deg - angle % 90.0))
    assert err < 1.05, f'{angle} -> {g.angle_deg}'


def test_it_survives_heavy_blur_which_is_the_whole_point():
    """⛔ TURBID WATER IS THE CASE THIS EXISTS FOR. Blur destroys the corners a
    feature tracker needs and leaves the carrier untouched -- a low-pass filter
    cannot move a spectral peak. Measured: 21 px of Gaussian blur changes the
    recovered period by nothing at all."""
    import cv2
    clean = detect(grid(period=32.0, angle_deg=20.0))
    for k in (5, 11, 21):
        blurred = detect(cv2.GaussianBlur(grid(period=32.0, angle_deg=20.0),
                                          (k, k), 0))
        assert blurred is not None, f'lost at {k} px blur'
        assert abs(blurred.period_px - clean.period_px) < 0.5
        assert abs(blurred.angle_deg - clean.angle_deg) < 0.5


def test_it_survives_a_dim_low_contrast_scene():
    """Depth and turbidity both cut contrast. The peak's POSITION is what
    carries the information, and scaling every pixel cannot move it."""
    dim = (grid(period=32.0) * 0.3 + 90).astype(np.uint8)
    g = detect(dim)
    assert g is not None and abs(g.period_px - 32.0) < 0.5


def test_a_sine_grid_works_as_well_as_a_square_one():
    g = detect(grid(period=32.0, sine=True))
    assert g is not None and abs(g.period_px - 32.0) < 0.5


def test_noise_degrades_it_gracefully_and_then_it_REFUSES():
    """It must not fail loudly-wrong. Past its SNR it returns None, which is
    where LK flow takes over -- the two methods are complementary, and the
    literature says so (ADOPT beats DIC above 10 dB, DIC wins below)."""
    assert detect(grid(noise=10)) is not None
    assert detect(grid(noise=30)) is not None
    assert detect(grid(noise=90)) is None


# --------------------------------------------------------------------------- #
#  what the grating is FOR
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('h_true', [0.6, 1.0, 1.5, 2.0])
def test_height_comes_off_the_floors_own_tile_size(h_true):
    """`h = f * tile_m / pitch_px`. Height is the term that multiplies every
    optical-flow velocity, and today it comes from a bbox or a guess.

    On a 25 mm mosaic tile -- the common competition-pool surface -- the whole
    working depth range is inside the envelope.
    """
    f_px, tile_m = 1027.9, 0.025
    period = f_px * tile_m / h_true          # what that height would look like
    g = detect(grid(n=384, period=period))
    assert g is not None, f'refused at h={h_true} (period {period:.1f} px)'
    assert abs(g.height_m(f_px, tile_m) - h_true) < 0.05 * h_true


def test_the_operating_envelope_is_stated_not_assumed():
    """⛔ A BIG-TILE POOL AT LOW ALTITUDE IS OUTSIDE THIS INSTRUMENT. Five
    cycles must fit in frame, so the usable height depends on the venue's tile
    size -- which is a world constant that must be measured on deck, never
    inherited from the last pool."""
    from duburi_localization.tile_grating import usable_height_m

    lo_big, _ = usable_height_m(1027.9, 0.15, 480)
    lo_mosaic, hi_mosaic = usable_height_m(1027.9, 0.025, 480)
    assert lo_big > 1.5, 'a 150 mm tile needs real altitude'
    assert lo_mosaic < 0.5 < hi_mosaic, 'a 25 mm tile covers the working range'


def test_the_envelope_refuses_nonsense_inputs():
    from duburi_localization.tile_grating import usable_height_m

    lo, hi = usable_height_m(0.0, 0.15, 480)
    assert math.isnan(lo) and math.isnan(hi)


def test_height_is_NaN_rather_than_infinite_on_a_degenerate_period():
    g = Grating(period_px=0.0, angle_deg=0.0, phase=(0.0, 0.0), strength=1.0)
    assert math.isnan(g.height_m(1000.0, 0.15))


def test_the_heading_is_reported_modulo_ninety_and_says_so():
    """A square grid cannot tell one wall from another. The four-fold
    ambiguity is REAL and is returned rather than resolved by invention."""
    g = Grating(period_px=32.0, angle_deg=10.0, phase=(0.0, 0.0), strength=1.0)
    assert g.heading_deg() == pytest.approx(10.0)
    assert g.heading_deg(mount_yaw_deg=100.0) == pytest.approx(0.0)


def test_phase_shift_is_wrapped_to_half_a_period():
    a = detect(grid(period=32.0, phase=0.0))
    b = detect(grid(period=32.0, phase=0.5))
    assert a is not None and b is not None
    d = shift_from_phase(a, b)
    assert d is not None and abs(d) <= 32.0 / 2.0 + 1e-6


def test_a_changed_height_refuses_the_phase_comparison():
    """Different carrier, so the phase difference is meaningless."""
    a = detect(grid(period=32.0))
    b = detect(grid(period=16.0))
    assert shift_from_phase(a, b) is None


def test_shift_from_phase_handles_missing_inputs():
    assert shift_from_phase(None, None) is None


# --------------------------------------------------------------------------- #
#  our own simulator, which does NOT have a tiled floor
# --------------------------------------------------------------------------- #
def test_our_sim_floor_textures_are_correctly_refused():
    """⛔ A SIM-FIDELITY FINDING AS MUCH AS AN ALGORITHM ONE. Measured: our
    SAUVC and RoboSub floor textures carry 0.65 % and 0.67 % of their spectral
    energy in the dominant peak -- they are noise-like, not tiled. A real
    competition pool is tiled and our simulated one is not, so this capability
    cannot be rehearsed in sim until the texture is replaced.
    """
    import pathlib

    import cv2

    root = pathlib.Path(__file__).resolve().parents[3]
    base = root / 'sim/src/duburi_sim_worlds/models'
    for rel in ('sauvc_textures/pool_floor.png', 'robosub_textures/pool_floor.png'):
        path = base / rel
        if not path.exists():
            pytest.skip(f'{rel} not present')
        im = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        assert detect(im) is None, f'{rel} should not read as a grating'


# --------------------------------------------------------------------------- #
#  measure(): decimation, which is what makes this affordable at all
# --------------------------------------------------------------------------- #
def test_decimation_preserves_the_measurement():
    """⛔ THE COST ARGUMENT. A full 640x480 pass is 28.90 ms -- an entire frame
    budget at 30 Hz. At /3 it is ~1.5 ms. Decimation is free here because the
    quantity read is a COARSE frequency and an area-average is a low-pass
    filter, which cannot move a peak far below its cutoff."""
    from duburi_localization.tile_grating import measure

    im = grid(n=480, period=26.0, angle_deg=17.0)
    full = detect(im)
    dec = measure(im, decimate=3)
    assert full is not None and dec is not None
    assert abs(dec.period_px - full.period_px) / full.period_px < 0.02
    assert abs(dec.angle_deg - full.angle_deg) < 0.5


def test_the_period_is_returned_in_FULL_frame_pixels():
    """Getting this wrong divides every height by the decimation factor,
    silently. It is done once here rather than at each call site."""
    from duburi_localization.tile_grating import measure

    im = grid(n=480, period=30.0)
    dec = measure(im, decimate=3)
    assert dec is not None
    assert abs(dec.period_px - 30.0) < 1.0


def test_measure_still_refuses_the_negative_controls():
    """Decimation must not turn a gradient into a grating."""
    from duburi_localization.tile_grating import measure

    assert measure(np.tile(np.linspace(0, 255, 480, dtype=np.uint8), (480, 1))) is None
    assert measure(RNG.integers(0, 255, (480, 480)).astype(np.uint8)) is None


def test_measure_handles_no_frame():
    from duburi_localization.tile_grating import measure

    assert measure(None) is None


# --------------------------------------------------------------------------- #
#  snap_to_grid: a DRIFT BOUND, not a heading source
# --------------------------------------------------------------------------- #
def test_a_small_drift_is_pulled_back_onto_the_grid():
    """A free-running BNO drifts without bound. The grid's residual should be
    constant, so a moving residual IS the drift and can be removed."""
    from duburi_localization.tile_grating import snap_to_grid

    # Hull truly on the grid at 30 deg; gyro has drifted 3 deg.
    assert snap_to_grid(33.0, 30.0) == pytest.approx(30.0)
    assert snap_to_grid(27.0, 30.0) == pytest.approx(30.0)


def test_it_does_not_move_a_heading_already_on_the_grid():
    from duburi_localization.tile_grating import snap_to_grid

    assert snap_to_grid(30.0, 30.0) == pytest.approx(30.0)


def test_the_four_fold_ambiguity_is_respected_not_resolved():
    """A square grid looks identical from four directions, so the correction
    is to the NEAREST grid line -- never to 'the' grid line."""
    from duburi_localization.tile_grating import snap_to_grid

    # Facing ~120 deg with the grid reading 30: 120 is already on a grid line
    # (30 + 90), so nothing should move.
    assert snap_to_grid(120.0, 30.0) == pytest.approx(120.0)
    assert snap_to_grid(122.0, 30.0) == pytest.approx(120.0)


def test_a_large_disagreement_is_REFUSED_not_applied():
    """⛔ THE SAFETY. A correction beyond the bound means the estimate and the
    floor disagree about WHICH grid line is which -- a four-fold aliasing
    error. Applying it snaps the hull 90 degrees onto the wrong branch. A
    drifting heading is still roughly right; a confidently wrong one is not."""
    from duburi_localization.tile_grating import snap_to_grid

    assert snap_to_grid(30.0 + 40.0, 30.0, max_correction_deg=20.0) is None


def test_the_correction_never_exceeds_half_a_cell():
    """Whatever the inputs, the pull is bounded by 45 deg -- so a drifting yaw
    can never be moved further than half a grid cell."""
    from duburi_localization.tile_grating import snap_to_grid

    for yaw in range(0, 360, 7):
        for grid_a in (0.0, 13.0, 44.0, 89.0):
            got = snap_to_grid(float(yaw), grid_a, max_correction_deg=90.0)
            assert got is not None
            assert abs(got - yaw) <= 45.0 + 1e-9


def test_nan_and_none_are_refused():
    from duburi_localization.tile_grating import snap_to_grid

    assert snap_to_grid(float('nan'), 30.0) is None
    assert snap_to_grid(30.0, float('nan')) is None
    assert snap_to_grid(None, 30.0) is None
