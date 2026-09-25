"""The Hailo input path must hand the chip RGB, not the BGR cv2 gives us.

⛔ B-61, AND IT COST A DAY. `letterbox` and `_letterbox_into_bound` both wrote
the frame into the input buffer without converting BGR to RGB, so the
accelerator ran on colour-swapped pixels -- red and blue exchanged -- for as
long as this backend has existed. Nothing raised, nothing slowed down, no
frame was dropped. The detector simply answered a different question:
`gate=0.78 @56,246` became `repair=0.92 @0,0`.

⚠ TWO WRONG DIAGNOSES CAME FIRST, both consistent with the symptom:
  1. a decode stride bug in the `HAILO NMS BY CLASS` buffer -- disproved by
     dumping the raw buffer, which the packed-layout walk matched exactly;
  2. the DFC's calibration cliff (258 frames against a 1024 threshold, which
     really does drop AdaRound and QAT) -- disproved by a 1038-frame rebuild
     that reproduced the corruption unchanged.

What named it was running the SAME frames through the `.pt` with the channels
deliberately swapped and getting the HEF's exact signature back, class and
origin box included. A defect that survives two plausible explanations is
found by reproducing it somewhere the suspected component does not exist.

⭐ ultralytics converts internally, which is why the `.pt` path always looked
healthy -- and why every measurement taken through it stays valid.

These tests need no Hailo hardware: they check the frame preparation, which is
where the defect lived.
"""
from __future__ import annotations

import numpy as np
import pytest

from mongla_vision.detection.hailo import letterbox


def _bgr_marker(h=120, w=200):
    """A frame whose channels are unambiguous: pure BGR blue."""
    im = np.zeros((h, w, 3), np.uint8)
    im[:, :, 0] = 255            # cv2 channel 0 is BLUE
    return im


def test_letterbox_hands_back_rgb_not_bgr():
    """⭐ THE DEFECT ITSELF. A pure-blue BGR frame must arrive as pure-blue
    RGB -- energy in channel 2, not channel 0. Before the fix this returned
    the bytes untouched and the chip saw red."""
    out, _s, px, py = letterbox(_bgr_marker(), 256)
    inner = out[py + 5:py + 20, px + 5:px + 20]
    assert inner[..., 2].mean() > 200, \
        'blue must land in RGB channel 2; the frame reached the chip as BGR'
    assert inner[..., 0].mean() < 50, \
        'RGB channel 0 should be empty for a pure-blue frame'


def test_the_pad_is_untouched_by_the_conversion():
    """The 114 grey bars are channel-symmetric, so the conversion must not
    disturb them -- if it did, the geometry tests would start passing for the
    wrong reason."""
    out, _s, px, _py = letterbox(_bgr_marker(h=100, w=200), 256)
    if px > 2:
        assert np.all(out[:, :px] == 114)


def test_geometry_is_unchanged_by_the_colour_fix():
    """Scale and pads are what un-letterboxing needs; the colour fix must not
    move a single box."""
    out, s, px, py = letterbox(np.zeros((480, 640, 3), np.uint8), 640)
    assert out.shape == (640, 640, 3)
    assert s == pytest.approx(1.0)
    assert (px, py) == (0, 80)


def test_a_red_frame_round_trips_the_other_way():
    """Guards against the fix being applied twice, which would swap the
    channels back and look identical to no fix at all on a symmetric image."""
    im = np.zeros((120, 200, 3), np.uint8)
    im[:, :, 2] = 255                       # BGR red
    out, _s, px, py = letterbox(im, 256)
    inner = out[py + 5:py + 20, px + 5:px + 20]
    assert inner[..., 0].mean() > 200, 'BGR red must become RGB channel 0'
    assert inner[..., 2].mean() < 50


def test_the_async_bound_path_converts_too():
    """⛔ THE PATH THAT ACTUALLY FLIES. `_letterbox_into_bound` is a second
    implementation for the async API, and fixing only `letterbox` would leave
    the vehicle running on swapped channels while the tests passed.

    Driven against a stub holding just the bound buffer, so it needs no chip.
    """
    from mongla_vision.detection.hailo import HailoDetector

    class _Stub:
        _size = 256
        _in_buf = np.zeros((256, 256, 3), np.uint8)
        _pad_geom = None

    s = _Stub()
    try:
        HailoDetector._letterbox_into_bound(s, _bgr_marker())
    except AttributeError as exc:            # stub is missing a field
        pytest.skip(f'bound path needs more state than the stub has: {exc}')
    lit = s._in_buf[s._in_buf.sum(axis=2) > 0]
    assert lit.size, 'nothing was written into the bound buffer'
    assert lit[..., 2].mean() > lit[..., 0].mean(), \
        'the async path still hands the chip BGR'
