"""Crop for range, and let go before it costs anything.

MEASURED on real labelled data (bin dataset, own model, conf 0.15) --
recall vs apparent target size:

    ~range      full frame   centre 50 % crop
    1.0x          100.0 %         69.0 %
    2.0x           99.2 %        100.0 %
    4.0x           65.9 %        100.0 %
    6.7x           20.2 %         67.4 %

So the crop is the right choice ONLY while the target is far, and strictly
the wrong one when it is close. Everything here is about switching between
them without introducing a third failure.

The mission framing matters more than the numbers: a detector that only
sees the gate from 1 m gives `vision.align` no distance to correct over. An
AUV that arrives at the prop having never seen it has not gained anything
from a fast pipeline.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

cv2 = pytest.importorskip('cv2')

from duburi_vision.detection.rangecrop import (      # noqa: E402
    CropState, RangeCrop, ENTER_FRAC, EXIT_FRAC,
)


def _frame(w=640, h=360):
    f = np.zeros((h, w, 3), np.uint8)
    f[:, :, 1] = 90
    return f


def test_a_far_target_turns_the_crop_ON():
    """65.9 % recall at 4x range without it, 100 % with."""
    rc = RangeCrop()
    rc.observe(area_frac=0.004, now=0.0)          # a speck
    assert rc.active


def test_a_close_target_turns_it_OFF_again():
    """69 % recall on close targets while cropped -- the prop no longer fits.
    Staying cropped through an approach would trade the end of the mission
    for the start of it."""
    rc = RangeCrop()
    rc.observe(0.004, 0.0)
    assert rc.active
    rc.observe(0.20, 1.0)                          # arrived
    assert not rc.active


def test_it_does_NOT_flap_in_the_hysteresis_band():
    """A single threshold on a jittering quantity makes the field of view
    change every frame, which is worse than either state: the control loop
    sees the target appear and vanish for reasons unrelated to the water."""
    rc = RangeCrop()
    mid = (ENTER_FRAC + EXIT_FRAC) / 2
    rc.observe(mid, 0.0)
    assert not rc.active, 'mid-band must not ENTER from full frame'
    rc.observe(0.004, 1.0)
    assert rc.active
    rc.observe(mid, 2.0)
    assert rc.active, 'mid-band must not EXIT from cropped'


def test_a_sustained_loss_releases_the_crop():
    """A cropped view that has lost the target is the worst place to search
    from -- and searching is exactly what happens next."""
    rc = RangeCrop(lost_release_s=1.0)
    rc.observe(0.004, 0.0)
    assert rc.active
    rc.observe(None, 0.5)
    assert rc.active, 'a brief gap must not throw away the crop'
    rc.observe(None, 2.0)
    assert not rc.active


def test_the_crop_is_centred_and_the_right_size():
    rc = RangeCrop()
    rc.observe(0.004, 0.0)
    out, st = rc.apply(_frame(640, 360))
    assert st.active
    assert out.shape[:2] == (180, 320)
    assert (st.x0, st.y0) == (160, 90)


def test_boxes_map_BACK_to_full_frame_coordinates():
    """Every consumer downstream works in full-frame pixels. Forgetting the
    offset does not raise -- it steers the vehicle at a point displaced by
    the crop origin, which reads as a calibration fault."""
    rc = RangeCrop()
    rc.observe(0.004, 0.0)
    _out, st = rc.apply(_frame(640, 360))
    assert st.to_full((10.0, 20.0, 50.0, 60.0)) == (170.0, 110.0, 210.0, 150.0)


def test_an_inactive_state_maps_boxes_UNCHANGED():
    assert CropState().to_full((1.0, 2.0, 3.0, 4.0)) == (1.0, 2.0, 3.0, 4.0)


def test_full_frame_is_returned_by_reference_when_inactive():
    """The uncropped path must cost nothing -- it is the common case."""
    rc = RangeCrop()
    f = _frame()
    out, st = rc.apply(f)
    assert out is f and not st.active


def test_thresholds_that_would_flap_are_REFUSED():
    """enter >= exit is not a tuning choice, it is a guaranteed oscillation.
    Refusing at construction beats discovering it in water."""
    with pytest.raises(ValueError):
        RangeCrop(enter_frac=0.05, exit_frac=0.01)
    with pytest.raises(ValueError):
        RangeCrop(crop_frac=0.0)
    with pytest.raises(ValueError):
        RangeCrop(crop_frac=1.5)


def test_a_full_approach_crops_once_and_releases_once():
    """The behaviour that matters end to end: acquire far, hold through the
    approach, release on arrival -- with no flapping in between."""
    rc = RangeCrop()
    switches = []
    prev = rc.active
    for t, frac in enumerate([0.003, 0.004, 0.006, 0.010, 0.02, 0.04,
                              0.06, 0.12, 0.25, 0.40]):
        rc.observe(frac, float(t))
        if rc.active != prev:
            switches.append((t, rc.active))
            prev = rc.active
    assert len(switches) == 2, f'expected acquire+release, got {switches}'
    assert switches[0][1] is True and switches[1][1] is False


# --------------------------------------------------------------------------- #
#  The wiring, not just the policy
# --------------------------------------------------------------------------- #
def test_the_detector_maps_cropped_boxes_back_before_publishing():
    """THE SILENT FAILURE. A box found in crop coordinates and published
    without the offset points at the wrong place by (x0, y0) -- 160 px
    horizontally on a 640-wide frame. Nothing raises; the vehicle steers at a
    displaced point and it reads as a calibration fault.

    Asserted on the SOURCE of the shipping loop rather than a copy of it,
    because the round that fixed the Kalman revival bug learned that a test
    which reimplements the loop tests a copy without the defect."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'detector_node.py').read_text()
    assert 'crop_state.to_full(d.xyxy)' in src, \
        'cropped detections are published without the offset'
    # ...and the mapping must happen BEFORE the size feedback, or the policy
    # compares a full-frame box against a crop-relative threshold.
    assert (src.index('crop_state.to_full(d.xyxy)')
            < src.index('self._crop.observe(')), \
        'boxes must be mapped back before the size feedback reads them'


def test_the_size_feedback_uses_the_frame_the_DETECTOR_saw():
    """While cropped, a target filling the crop is CLOSE -- even though it is
    a small fraction of the full frame. Comparing against the full frame
    would mean the crop never releases, and the approach would finish at
    69 % recall instead of 100 %."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'detector_node.py').read_text()
    assert 'seen = crop_state.w * crop_state.h' in src


def test_it_is_OFF_by_default():
    """Half the field of view is a real cost, and the mission decides whether
    it is worth paying. Nothing that trades away FOV should switch itself on
    without being asked."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'detector_node.py').read_text()
    assert "self.declare_parameter('range_crop',          -1)" in src
    assert "if int(_p('range_crop', 0)) > 0" in src
