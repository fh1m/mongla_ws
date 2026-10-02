"""98 corners from a patch with nothing in it, and 88 % of them die.

⛔ THE TRAP, MEASURED. `goodFeaturesToTrack` returns MORE corners from a flat,
noisy patch than from a textured one -- sensor noise in a flat region produces
plenty of local maxima, and the detector has no way to know they are noise. So
`Follower.reset()` returned a healthy point count for a box that could not be
carried at all, and the ladder discovered that thirty frames later while
holding a box it believed in.

Over 155 boxes on four archive clips, binned by patch grey-level std:

    patch std     boxes   corners found (p50)   LK survival (p50)
      0 - 2          30            98                 0.12
      2 - 5          65            23                 0.36
      5 - 10         21            22                 0.91
     20+             19            21                 0.95

⭐ THE FIX IS A REFUSAL, NOT A RESCUE. Two model-free rungs were built and
measured for exactly these frames and BOTH failed: a colour back-projection
loses to a decoy box on 69 % of gate.mkv frames, and NCC template matching
returns a confident 0.891 peak whose margin over its own second-best is +0.005.
The failure is not in the algorithm -- the patch carries 0.92 bits of entropy
where a trackable one carries 1.58. There is nothing there to track, so the
honest response is to say so at seed time.
"""
from __future__ import annotations

import numpy as np
import pytest

from mongla_vision.tracking.follower import MIN_PATCH_STD, Follower
import sys as _sys_ct
import pathlib as _pl_ct
_sys_ct.path.insert(0, str(_pl_ct.Path(__file__).resolve().parents[3] / 'tools'))
from code_text import code_of, code_of_file  # noqa: E402  (issue #22)


def _flat_noise(h=120, w=160, std=1.0, seed=0):
    """A patch with the statistics of water: no structure, a little noise.
    This is the input that produced 98 corners and 0.12 survival."""
    rng = np.random.default_rng(seed)
    return np.clip(128 + rng.normal(0, std, (h, w)), 0, 255).astype(np.uint8)


def _textured(h=120, w=160, seed=1):
    rng = np.random.default_rng(seed)
    img = (rng.random((h, w)) * 255).astype(np.uint8)
    for y in range(0, h, 16):
        img[y:y + 3, :] = 255
    return img


def test_the_noise_patch_really_does_yield_plenty_of_corners():
    """⛔ THE PREMISE, CHECKED FIRST. If `goodFeaturesToTrack` refused this
    patch by itself the gate would be pointless -- the whole finding is that it
    does NOT refuse, and returns a reassuring number."""
    import cv2

    roi = _flat_noise(std=1.0)
    p = cv2.goodFeaturesToTrack(roi, maxCorners=120, qualityLevel=0.01,
                                minDistance=7, blockSize=7)
    assert p is not None and len(p) >= 20, \
        'the premise fails: this patch yields no corners, so no gate is needed'


def test_a_patch_with_no_information_is_refused_at_seed_time():
    f = Follower()
    assert f.reset(_flat_noise(std=1.0), (10.0, 10.0, 150.0, 110.0)) == 0
    assert not f.active
    assert f.last_patch_std < MIN_PATCH_STD


def test_a_textured_patch_is_still_accepted():
    """⚠ THE OTHER HALF. A gate that refuses everything is not a gate."""
    f = Follower()
    assert f.reset(_textured(), (10.0, 10.0, 150.0, 110.0)) > 0
    assert f.active
    assert f.last_patch_std >= MIN_PATCH_STD


def test_the_refusal_says_why():
    """A refusal indistinguishable from "no corners found" sends the next
    reader to the wrong place. The std is kept so the reason is readable."""
    f = Follower()
    f.reset(_flat_noise(std=0.5), (10.0, 10.0, 150.0, 110.0))
    assert np.isfinite(f.last_patch_std)
    assert f.last_patch_std < 2.0


def test_the_bar_is_the_measured_one_not_a_round_number():
    """⛔ THE BAR IS AN OPERATING POINT, chosen by the cost of each mistake:
    at 2.0 a refusal is right 86.7 % of the time for a 19.4 % refusal rate,
    where 4.0 buys a lower miss rate at 49.7 % refused. Raising it is a
    decision about coverage, not a tidy-up."""
    assert MIN_PATCH_STD == pytest.approx(2.0)


def test_the_gate_runs_before_the_corner_detector():
    """Cheap first. `roi.std()` is one pass; `goodFeaturesToTrack` is an
    eigenvalue per pixel. Order matters on a 30 Hz budget."""
    import ast
    from pathlib import Path

    src = code_of_file(Path(__file__).resolve().parents[1] / 'mongla_vision' / 'tracking'
           / 'follower.py')
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == 'reset')
    body = ast.dump(fn)
    assert 'MIN_PATCH_STD' in body, 'the patch gate is not in reset()'
    assert body.index('MIN_PATCH_STD') < body.index('goodFeaturesToTrack'), \
        'the cheap refusal must come before the expensive corner detector'
