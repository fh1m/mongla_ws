"""The five optical statistics, and the rule built on them.

These characterise WATER, not a task. Two clips of the same competition
behaved oppositely under the same preprocessing because their optics
differed, so "which prop is this" is the wrong axis and "what is this water
doing to the light" is the right one.

Pinned as PROPERTIES rather than values: the numbers move with the venue, and
that is the point. What must not move is that each statistic responds to the
degradation it is meant to detect, and that the recommendation refuses to
answer outside the range it has evidence for.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

cv2 = pytest.importorskip('cv2')

from mongla_vision.underwater import (        # noqa: E402
    WaterStats, analyse_frames, frame_stats,
)


def _scene(blur=0, sat=120, veil=0, seed=3):
    """A synthetic frame with controllable degradation."""
    # STRUCTURE, not pixel noise. Blurring white noise genuinely lowers its
    # stddev, so a noise-only scene would make the independence test assert
    # something false about the code. Real frames are large regions with
    # edges, and that is what has to be modelled -- my first version of this
    # fixture failed the test for exactly that reason, and the fixture was
    # what was wrong.
    hsv = np.zeros((240, 320, 3), np.uint8)
    hsv[:, :, 0] = 90                                  # green-ish hue
    hsv[:, :, 1] = sat
    v = np.full((240, 320), 70, np.uint8)
    for i in range(0, 320, 40):                        # bright vertical bars
        v[:, i:i + 20] = 190
    v[80:160, 100:220] = 130                           # a mid-tone block
    hsv[:, :, 2] = v
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    if blur:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    if veil:                                           # backscatter: add light
        img = cv2.addWeighted(img, 1.0 - veil, np.full_like(img, 200), veil, 0)
    return img


def test_sharpness_falls_with_blur():
    """Motion blur and scattering both flatten edges, and this is the
    quantity that sees them."""
    sharp = analyse_frames([_scene(blur=0)]).sharpness
    blurry = analyse_frames([_scene(blur=4)]).sharpness
    assert blurry < sharp * 0.2, (sharp, blurry)


def test_contrast_falls_with_backscatter():
    """Veiling light adds a constant to every pixel and crushes the dynamic
    range a detector needs -- distinct from blur, and not fixed by the same
    thing."""
    clear = analyse_frames([_scene(veil=0.0)]).contrast
    veiled = analyse_frames([_scene(veil=0.7)]).contrast
    assert veiled < clear * 0.5, (clear, veiled)


def test_saturation_tracks_absorption():
    """Red dies first, then green. A desaturated frame is a DEEP or
    long-range one -- and this separated the two clips where CLAHE helped and
    hurt more sharply than blur did."""
    rich = analyse_frames([_scene(sat=200)]).saturation
    washed = analyse_frames([_scene(sat=20)]).saturation
    assert rich > washed * 3, (rich, washed)


def test_the_statistics_are_INDEPENDENT():
    """Blur must not masquerade as low contrast, or the rule cannot tell a
    moving vehicle from murky water -- two problems with different fixes."""
    base = analyse_frames([_scene(blur=0, veil=0)])
    blurred = analyse_frames([_scene(blur=4, veil=0)])
    veiled = analyse_frames([_scene(blur=0, veil=0.7)])
    # blur moves sharpness hard and contrast comparatively little
    assert blurred.sharpness / base.sharpness < 0.2
    assert blurred.contrast / base.contrast > 0.5
    # veiling moves contrast hard
    assert veiled.contrast / base.contrast < 0.5


def test_medians_not_means_so_one_glint_cannot_move_it():
    """A surface flash or a passing wall is one frame. Characterising the
    water means the typical frame, not the loudest one -- the same reasoning
    that took the freshness thresholds off a maximum."""
    normal = [_scene(blur=2) for _ in range(20)]
    glint = [np.full((240, 320, 3), 255, np.uint8)]
    a = analyse_frames(normal)
    b = analyse_frames(normal + glint)
    assert abs(a.brightness - b.brightness) < 5.0


# --------------------------------------------------------------------------- #
#  The recommender is GONE, and stays gone
# --------------------------------------------------------------------------- #
def test_there_is_no_clahe_recommender():
    """It mapped these stats onto a CLAHE verdict using thresholds fitted to
    two clips, and a third venue proved the mapping wrong in the direction
    that matters: it said ON for the murkiest water in the archive, where
    CLAHE measurably does not help. Re-measured across 17 configurations it
    was never positive, and on the gate it cost 29 points.

    Asserted rather than merely deleted, because the tempting fix is to
    re-fit the thresholds on a third clip and ship it again."""
    import mongla_vision.underwater as u
    for gone in ('recommend', 'SHARP_MURKY', 'SAT_MURKY', 'SHARP_CLEAR',
                 'SAT_CLEAR'):
        assert not hasattr(u, gone), f'{gone} came back'
