"""Plane orientation from the homography, against synthetic ground truth.

Synthetic rather than recorded on purpose: this is geometry, so the truth is
CONSTRUCTIBLE -- build H from a known R, t, plane normal and distance, then
check what comes back. A recorded clip can only show that a number is stable,
never that it is right, and "stable and wrong" is the failure mode here.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.anchor.geometry import plane_geometry     # noqa: E402

# The measured forward camera (`pi_downward_1280x720.json`).
K = np.array([[1027.873, 0.0, 617.323],
              [0.0, 1033.857, 373.022],
              [0.0, 0.0, 1.0]])
_KI = np.linalg.inv(K)
_RNG = np.random.default_rng(0)
# A board-sized patch, roughly where a torpedo board sits when centred.
_REF = np.stack([_RNG.uniform(500, 740, 80),
                 _RNG.uniform(280, 470, 80)], 1).astype(np.float32)


def _scene(tilt_deg=0.0, axis='x', approach_m=1.0, d=2.0):
    """H for a plane tilted `tilt_deg`, camera closing `approach_m`.

    `X_live = R X_ref + t`, so a camera moving TOWARD the plane gives a
    NEGATIVE t_z -- getting that sign backwards silently inverts the apparent
    scale, and it cost a round to notice.

    The default approach is 1.0 m rather than 0.5 because a steeply tilted
    plane moves LESS in the image for the same approach -- at 65 deg, 0.5 m
    yields only 10.2 px and falls under the baseline bar. The scene has to
    clear the bar for a test about tilt recovery to be about tilt recovery.
    """
    import cv2
    th = math.radians(tilt_deg)
    n = (np.array([math.sin(th), 0.0, -math.cos(th)]) if axis == 'x'
         else np.array([0.0, math.sin(th), -math.cos(th)]))
    t = np.array([0.0, 0.0, -approach_m])
    H = K @ (np.eye(3) - np.outer(t, n) / d) @ _KI
    H = H / H[2, 2]                       # as findHomography returns it
    live = cv2.perspectiveTransform(_REF.reshape(-1, 1, 2), H)
    return H, _REF, live.reshape(-1, 2)


@pytest.mark.parametrize('deg', [float(d) for d in range(0, 66, 5)])
def test_tilt_is_recovered_from_a_known_plane(deg):
    """Swept every 5 deg, not sampled at round numbers.

    The first version of this test used 0/10/30/45/60 and passed -- while
    `decomposeHomographyMat` returned ALL-NaN normals at 5, 15 and 20 deg. The
    failure is scattered rather than a clean degeneracy, so a sparse sample
    misses it by luck, and the sample I happened to pick missed all three. The
    module now pre-normalises H by the Euclidean middle singular value, which
    fixes every one; this sweep is what keeps that honest.
    """
    H, ref, live = _scene(tilt_deg=deg)
    g = plane_geometry(H, K, ref, live)
    assert g.ok
    assert g.tilt_deg == pytest.approx(deg, abs=0.5)


def test_the_SMALL_tilt_case_is_the_one_that_matters():
    """The point filter is not a refinement -- without it the answer is wrong
    exactly in the terminal-alignment regime. Measured against the obvious
    "first normal with n_z < 0" heuristic: 10 deg reads 1.42, 30 deg reads
    4.01. Both say "nearly square" for a board that is not."""
    for deg in (10.0, 30.0):
        H, ref, live = _scene(tilt_deg=deg)
        assert plane_geometry(H, K, ref, live).tilt_deg == pytest.approx(deg, abs=0.5)


def test_a_square_on_board_reads_zero_despite_the_sign_ambiguity():
    """At zero tilt the filter legitimately returns BOTH facing solutions and
    the normal can come back sign-flipped. Taking the tilt from the signed dot
    product would report 180 deg for a perfectly square board."""
    H, ref, live = _scene(tilt_deg=0.0)
    g = plane_geometry(H, K, ref, live)
    assert g.ok and g.tilt_deg == pytest.approx(0.0, abs=0.5)
    assert g.ambiguous, 'zero tilt is genuinely ambiguous and should say so'


def test_yaw_and_pitch_separate_the_two_axes_a_hull_can_ACT_on():
    """One combined angle says something is wrong; it does not say strafe or
    change depth."""
    hx = plane_geometry(*_pack(_scene(tilt_deg=25.0, axis='x')))
    hy = plane_geometry(*_pack(_scene(tilt_deg=25.0, axis='y')))
    assert abs(hx.yaw_deg) > 20.0 and abs(hx.pitch_deg) < 5.0
    assert abs(hy.pitch_deg) > 20.0 and abs(hy.yaw_deg) < 5.0


def _pack(scene):
    H, ref, live = scene
    return H, K, ref, live


def test_it_REFUSES_rather_than_guessing():
    """A wrong tilt is worse than no tilt: the caller acts on it. Every input
    that cannot support an answer must return ok=False, not a plausible one."""
    H, ref, live = _scene(tilt_deg=20.0)
    assert not plane_geometry(None, K, ref, live).ok
    assert not plane_geometry(H, None, ref, live).ok
    assert not plane_geometry(H, K, None, live).ok
    assert not plane_geometry(H, K, ref[:3], live[:3]).ok      # under-determined
    assert not plane_geometry(H, K, ref, live[:10]).ok         # mismatched


def test_a_FAILED_decomposition_refuses_instead_of_reporting_SQUARE():
    """The dangerous failure, and it is silent.

    A NaN normal passes `norm <= 0.0` (False for NaN), `n / NaN` is NaN, and
    `max(-1, min(1, NaN))` evaluates to 1.0 in CPython -- so `acos` returns 0.0
    and a decomposition that produced nothing is reported as a PERFECTLY SQUARE
    board. That is the one wrong answer a firing gate must never be handed, and
    it is what the code did before the finite check.
    """
    import mongla_vision.anchor.geometry as G
    H, ref, live = _scene(tilt_deg=30.0)
    real = G.cv2 if hasattr(G, 'cv2') else None      # module imports cv2 locally
    import cv2

    class _NaNCv2:
        error = cv2.error

        @staticmethod
        def decomposeHomographyMat(H, K):
            nan = np.full((3, 1), np.nan)
            return 4, [np.eye(3)] * 4, [nan] * 4, [nan] * 4

        @staticmethod
        def filterHomographyDecompByVisibleRefpoints(*_a, **_k):
            return np.array([[0]])

    import sys as _sys
    saved = _sys.modules['cv2']
    _sys.modules['cv2'] = _NaNCv2
    try:
        g = plane_geometry(H, K, ref, live)
    finally:
        _sys.modules['cv2'] = saved
    assert g.ok is False, 'a NaN decomposition was reported as a valid pose'
    assert not (g.tilt_deg == 0.0), 'and it was reported as SQUARE ON'


# --------------------------------------------------------------------------- #
#  Reaching the anchor
# --------------------------------------------------------------------------- #
def test_the_anchor_carries_the_plane_only_when_it_is_CALIBRATED():
    """No K means no answer, not a guessed one -- and it must stay None rather
    than an `ok=False` that reads like a failed decomposition."""
    from mongla_vision.anchor.anchor import Anchor
    from test_anchor import _Backend                     # noqa: F401

    a = Anchor(_Backend(n=200))
    a.snap(np.zeros((240, 320), np.uint8))
    assert a.locate(np.zeros((240, 320), np.uint8)).plane is None

    b = Anchor(_Backend(n=200), k=K)
    b.snap(np.zeros((240, 320), np.uint8))
    p = b.locate(np.zeros((240, 320), np.uint8))
    assert p.plane is not None and hasattr(p.plane, 'tilt_deg')



# --------------------------------------------------------------------------- #
#  The baseline bar
# --------------------------------------------------------------------------- #
def test_a_tilt_with_no_BASELINE_is_refused_not_guessed():
    """The finding from real footage. The plane term only exists when the
    camera TRANSLATES; with none, H is K R K^-1 and carries no plane at all.
    Answering anyway produced 47 deg of frame-to-frame swing on an archive clip
    -- a hull that was not moving that way."""
    H, ref, live = _scene(tilt_deg=25.0, approach_m=0.01)
    g = plane_geometry(H, K, ref, live)
    assert g.ok is False
    assert g.baseline_px > 0.0, 'the measured baseline must be reported anyway'


def test_the_bar_is_where_the_measurement_put_it():
    """12 px is not a guess: at 11.75 px the p90 tilt error first falls under
    10 deg (24.1/42.1 at 0.84 px, 1.8/5.6 at 11.75). A caller wanting a firing
    gate reads `baseline_px` and demands more."""
    from mongla_vision.anchor.geometry import MIN_BASELINE_PX
    assert MIN_BASELINE_PX == 12.0
    H, ref, live = _scene(tilt_deg=25.0)
    loose = plane_geometry(H, K, ref, live, min_baseline_px=0.0)
    assert loose.ok and loose.baseline_px > 12.0
    assert not plane_geometry(H, K, ref, live,
                              min_baseline_px=loose.baseline_px + 1.0).ok


def test_PURE_ROTATION_is_refused_by_the_decomposition_itself():
    """A camera that only rotates produces LARGE displacement and zero plane
    information -- so a baseline gate alone would pass it. It must be refused
    for the right reason, which is that the decomposition degenerates."""
    import math as _m
    a = _m.radians(3.0)
    Ry = np.array([[_m.cos(a), 0, _m.sin(a)], [0, 1, 0], [-_m.sin(a), 0, _m.cos(a)]])
    H = K @ Ry @ _KI
    H = H / H[2, 2]
    import cv2
    live = cv2.perspectiveTransform(_REF.reshape(-1, 1, 2), H).reshape(-1, 2)
    disp = float(np.median(np.linalg.norm(live - _REF, axis=1)))
    assert disp > 40.0, 'the point of this test is a LARGE displacement'
    assert not plane_geometry(H, K, _REF, live).ok
