"""The fast rung: carry a box across the frames the detector missed.

Measured on real competition footage, 700 consecutive frames per clip:

    clip           gaps  covered  refused  drift med  worst
    bin_front_3      49      211        0      9.1 px  38.8
    bin_front_1       5       36        4     32.4 px  52.1
    gate_back         -      101        0          -      -

9.1 px of median drift on a 640-wide frame is ~1.4 %, and the
forward-backward gate DOES refuse -- four times on the hardest clip. Both
halves matter: a follower that never refuses is a confident lie, and one that
always refuses is not a rung.

Cost: `goodFeaturesToTrack` + `calcOpticalFlowPyrLK` on a 320x240 ROI measured
**8.0 ms on ONE Pi core** with the full vision stack running -- 125 Hz for
something the loop needs at 50.

These tests are about the REFUSALS and the RESET, because those are what keep a
frame-to-frame tracker from becoming an unbounded drift generator.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.tracking.follower import (         # noqa: E402
    Follower, FollowResult, MIN_POINTS)


def _scene(w=320, h=240, seed=0):
    """A textured frame. Pure noise would be untrackable by construction and a
    flat frame would have nothing to track -- neither tests the logic."""
    rng = np.random.default_rng(seed)
    img = np.full((h, w), 110, np.uint8)
    for _ in range(70):
        x, y = rng.integers(10, w - 30), rng.integers(10, h - 30)
        cv2.rectangle(img, (x, y), (x + int(rng.integers(6, 22)),
                                    y + int(rng.integers(6, 22))),
                      int(rng.integers(0, 255)), -1)
    return cv2.GaussianBlur(img, (3, 3), 0)


def _shift(img, dx, dy):
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]),
                          borderMode=cv2.BORDER_REFLECT)


# --------------------------------------------------------------------------- #
#  It follows
# --------------------------------------------------------------------------- #
def test_a_translated_scene_moves_the_box_by_the_same_amount():
    """THE core behaviour: the box goes where the content went."""
    a = _scene()
    f = Follower()
    assert f.reset(a, (80, 60, 200, 170)) >= MIN_POINTS
    r = f.step(_shift(a, 7, -4))
    assert r.ok
    assert r.xyxy[0] == pytest.approx(87, abs=2.0)
    assert r.xyxy[1] == pytest.approx(56, abs=2.0)


def test_the_box_KEEPS_its_size():
    """A follower estimates translation, not scale. Letting the box breathe
    would feed a fake range change to anything reading bbox fill."""
    a = _scene()
    f = Follower()
    f.reset(a, (80, 60, 200, 170))
    r = f.step(_shift(a, 5, 5))
    assert r.ok
    assert (r.xyxy[2] - r.xyxy[0]) == pytest.approx(120, abs=1e-3)
    assert (r.xyxy[3] - r.xyxy[1]) == pytest.approx(110, abs=1e-3)


# --------------------------------------------------------------------------- #
#  It refuses -- the half that makes it safe
# --------------------------------------------------------------------------- #
def test_it_refuses_before_it_has_been_seeded():
    assert Follower().step(_scene()).ok is False


def test_a_featureless_box_seeds_NOTHING_rather_than_pretending():
    """A flat region has nothing to track. Seeding anyway would produce a box
    that translates by whatever noise the flow returns."""
    flat = np.full((240, 320), 128, np.uint8)
    f = Follower()
    assert f.reset(flat, (100, 80, 200, 180)) == 0
    assert not f.active
    assert f.step(flat).ok is False


def test_an_unrelated_frame_is_REFUSED_not_followed():
    """The forward-backward gate. Given a completely different scene, LK still
    returns displacements -- confidently, for every point. Without the round
    trip the follower would publish a box built entirely from those."""
    f = Follower()
    f.reset(_scene(seed=1), (80, 60, 200, 170))
    r = f.step(_scene(seed=99))
    assert r.ok is False
    assert not f.active          # and it drops itself rather than limping on


def test_confidence_is_zero_on_a_refusal():
    assert FollowResult(ok=False, survival=0.9).confidence == 0.0
    assert FollowResult(ok=True, survival=0.5).confidence == pytest.approx(0.5)


# --------------------------------------------------------------------------- #
#  It is bounded -- reset is what makes frame-to-frame safe
# --------------------------------------------------------------------------- #
def test_reset_reseeds_from_the_detection_and_clears_old_drift():
    """Every accepted detection reseeds, which is what stops the accumulated
    error growing without bound. A follower that could not be reset would be a
    drift generator with a confidence field."""
    a = _scene()
    f = Follower()
    f.reset(a, (80, 60, 200, 170))
    f.step(_shift(a, 6, 0))
    n = f.reset(a, (10, 10, 90, 90))
    assert n >= MIN_POINTS
    r = f.step(_shift(a, 3, 0))
    assert r.ok
    assert r.xyxy[0] == pytest.approx(13, abs=2.0)   # the NEW box, not the old


def test_drop_deactivates_it():
    a = _scene()
    f = Follower()
    f.reset(a, (80, 60, 200, 170))
    assert f.active
    f.drop()
    assert not f.active
    assert f.step(a).ok is False


def test_a_box_outside_the_frame_is_clamped_not_crashed():
    """Boxes arrive from a detector on a different resolution and from a
    follower that has drifted; neither is guaranteed to be inside."""
    a = _scene()
    f = Follower()
    f.reset(a, (-50, -50, 5000, 5000))     # must not raise
    f.reset(a, (319, 239, 321, 241))       # degenerate, must not raise
