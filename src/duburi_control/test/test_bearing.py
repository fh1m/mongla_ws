"""Pixels -> bearings, checked against the real measured calibration.

The failure mode this guards is not a crash: a wrong bearing is a plausible
number that aims the vehicle slightly off, forever.
"""
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.bearing import (          # noqa: E402
    Bearing, bearing_from_normalised, bearing_from_pixels,
)

# Our measured camera (srot 8049de9): 25 views, calibrateCameraRO, held-out
# validated. Inlined rather than read from the file so this test still runs on
# a machine without the vision package installed.
FX, FY, CX, CY = 1027.87, 1033.86, 617.32, 373.02
W, H = 1280, 720
K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
D = [-0.0909, 0.1492, -0.0063, -0.0048, -0.2775]
HFOV = math.radians(63.82)
VFOV = math.radians(38.40)


# --------------------------------------------------------------------------- #
#  The pinhole path
# --------------------------------------------------------------------------- #
def test_the_optical_axis_is_zero_bearing_not_the_frame_centre():
    """The load-bearing difference from the spec's linear form.

    A target at the PRINCIPAL POINT is dead ahead by definition. A target at
    the frame centre is not, unless the two coincide -- and on this camera they
    are 26 px apart, which is 1.26 deg. Driving `ex -> 0` therefore parks the
    vehicle 1.26 deg off the thing it is aiming at, and no gain fixes it
    because it is the wrong definition of zero, not a tracking error.
    """
    on_axis = bearing_from_pixels(CX, CY, 10, 10, width=W, height=H, K=K)
    assert on_axis.angle_x == pytest.approx(0.0, abs=1e-9)
    assert on_axis.angle_y == pytest.approx(0.0, abs=1e-9)

    frame_centre = bearing_from_pixels(W / 2, H / 2, 10, 10, width=W, height=H, K=K)
    assert math.degrees(frame_centre.angle_x) == pytest.approx(1.264, abs=0.01)
    assert abs(math.degrees(frame_centre.angle_x)) > 1.0     # not a rounding wobble


def test_signs_follow_the_spec():
    """VISION_API.md §1: +angle_x = target RIGHT, +angle_y = target BELOW
    (image Y grows down). A sign error here drives the hull away from the
    target at exactly the rate it should drive toward it."""
    right = bearing_from_pixels(CX + 200, CY, 10, 10, width=W, height=H, K=K)
    left = bearing_from_pixels(CX - 200, CY, 10, 10, width=W, height=H, K=K)
    below = bearing_from_pixels(CX, CY + 150, 10, 10, width=W, height=H, K=K)
    above = bearing_from_pixels(CX, CY - 150, 10, 10, width=W, height=H, K=K)
    assert right.angle_x > 0 and left.angle_x < 0
    assert below.angle_y > 0 and above.angle_y < 0


def test_the_edge_bearing_matches_the_measured_hfov():
    """Half the frame width must come out as half the calibrated HFOV --
    the two numbers come from the same fx, so disagreement means the
    conversion, not the calibration."""
    edge = bearing_from_pixels(W, CY, 10, 10, width=W, height=H, K=K)
    left_edge = bearing_from_pixels(0, CY, 10, 10, width=W, height=H, K=K)
    full = edge.angle_x - left_edge.angle_x
    assert math.degrees(full) == pytest.approx(63.82, abs=0.05)


def test_angular_size_does_not_shrink_when_the_target_moves_sideways():
    """size_x is the STANDOFF measure, so an apparent shrink reads as 'further
    away' and drives the hull forward. `atan(w/fx)` has exactly that bug: it
    implicitly assumes the box straddles the optical axis. Measuring from the
    box EDGES does not.

    (A real off-axis object does subtend slightly differently -- this asserts
    the effect is small and NOT the large systematic error the naive form has.)
    """
    box = 120.0
    centred = bearing_from_pixels(CX, CY, box, box, width=W, height=H, K=K)
    off = bearing_from_pixels(CX + 400, CY, box, box, width=W, height=H, K=K)
    ratio = off.size_x / centred.size_x
    assert 0.85 < ratio < 1.15, f'size changed {100*(1-ratio):.0f} % across the frame'

    naive_centred = 2 * math.atan(box / 2 / FX)
    assert centred.size_x == pytest.approx(naive_centred, rel=0.05)


# --------------------------------------------------------------------------- #
#  Fallback, and refusing to invent one
# --------------------------------------------------------------------------- #
def test_without_a_calibration_it_says_so():
    b = bearing_from_pixels(W / 2 + 320, H / 2, 50, 50, width=W, height=H,
                            hfov_rad=HFOV, vfov_rad=VFOV)
    assert b is not None and b.calibrated is False
    assert math.degrees(b.angle_x) == pytest.approx(63.82 / 4, abs=0.01)


def test_a_zeroed_K_is_not_a_calibration():
    """CameraInfo carries an all-zero k until a calibration file is loaded, so
    'K is present' and 'K is usable' are different questions. Answering the
    first one produces a division by zero or, worse, a bearing of zero."""
    zero_k = [0.0] * 9
    assert bearing_from_pixels(100, 100, 10, 10, width=W, height=H,
                               K=zero_k) is None
    fallback = bearing_from_pixels(100, 100, 10, 10, width=W, height=H,
                                   K=zero_k, hfov_rad=HFOV, vfov_rad=VFOV)
    assert fallback is not None and fallback.calibrated is False


def test_nothing_at_all_returns_none_rather_than_guessing():
    assert bearing_from_pixels(100, 100, 10, 10, width=W, height=H) is None
    assert bearing_from_pixels(100, 100, 10, 10, width=0, height=0, K=K) is None


# --------------------------------------------------------------------------- #
#  Distortion
# --------------------------------------------------------------------------- #
def test_undistortion_moves_the_edges_and_leaves_the_centre_alone():
    """k1=-0.09 is real barrel distortion. It must do nothing on the optical
    axis (r=0) and something measurable at the edge -- a no-op everywhere means
    the inverse iteration silently did not run."""
    axis_d = bearing_from_pixels(CX, CY, 10, 10, width=W, height=H, K=K, D=D)
    axis_n = bearing_from_pixels(CX, CY, 10, 10, width=W, height=H, K=K)
    assert axis_d.angle_x == pytest.approx(axis_n.angle_x, abs=1e-9)

    edge_d = bearing_from_pixels(50, CY, 10, 10, width=W, height=H, K=K, D=D)
    edge_n = bearing_from_pixels(50, CY, 10, 10, width=W, height=H, K=K)
    delta = abs(math.degrees(edge_d.angle_x - edge_n.angle_x))
    assert delta > 0.1, 'distortion had no effect at the frame edge'
    assert delta < 10.0, 'implausible correction -- check the coefficient order'


def test_undistort_round_trips_against_the_forward_model():
    """Apply Brown-Conrady forward, invert it, land back where you started.
    Without this the iteration could converge to something smooth and wrong."""
    from duburi_control.bearing import _undistort
    k1, k2, p1, p2, k3 = D
    for xn, yn in ((0.0, 0.0), (0.1, -0.05), (0.30, 0.17), (-0.28, 0.12)):
        r2 = xn * xn + yn * yn
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
        yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
        xr, yr = _undistort(xd, yd, D)
        assert xr == pytest.approx(xn, abs=1e-6)
        assert yr == pytest.approx(yn, abs=1e-6)


# --------------------------------------------------------------------------- #
#  The normalised entry point the control loop already has
# --------------------------------------------------------------------------- #
def test_normalised_and_pixel_forms_agree():
    a = bearing_from_normalised(0.5, -0.25, 0.2, 0.3, width=W, height=H, K=K, D=D)
    b = bearing_from_pixels(0.75 * W, 0.375 * H, 0.2 * W, 0.3 * H,
                            width=W, height=H, K=K, D=D)
    assert a.angle_x == pytest.approx(b.angle_x, abs=1e-9)
    assert a.size_y == pytest.approx(b.size_y, abs=1e-9)


def test_bearing_is_resolution_independent():
    """The entire point: the SAME physical direction must give the same
    bearing at any resolution, given a correctly scaled K. If this fails, the
    gains are still secretly a function of the camera and nothing was gained.
    """
    half = [v / 2 for v in K[:6]] + [0.0, 0.0, 1.0]
    half[8] = 1.0
    full = bearing_from_normalised(0.4, 0.2, 0.1, 0.1, width=W, height=H, K=K)
    small = bearing_from_normalised(0.4, 0.2, 0.1, 0.1, width=W // 2, height=H // 2,
                                    K=half)
    assert full.angle_x == pytest.approx(small.angle_x, abs=1e-9)
    assert full.angle_y == pytest.approx(small.angle_y, abs=1e-9)
