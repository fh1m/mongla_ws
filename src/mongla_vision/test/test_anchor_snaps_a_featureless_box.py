"""A box on a featureless frame snaps nothing, and says so.

`Anchor.snap` scaled the ROI into backend pixels only when the backend found
keypoints, then stored the scaled box regardless: with a box and no keypoints
it raised NameError instead of returning 0, and the bank's `'empty'` refusal
-- written for exactly a near-empty reference -- was never reached.
"""
import numpy as np
import pytest

from mongla_vision.anchor.anchor import Anchor


class _Blank:
    w, h = 320, 240

    def detect(self, gray):
        return np.zeros((0, 2), np.float32), np.zeros((0, 64), np.float32)


def test_a_box_with_no_keypoints_returns_zero():
    a = Anchor(_Blank())
    n = a.snap(np.zeros((360, 640), np.uint8), roi=(100, 50, 300, 250))
    assert n == 0 and not a.has_reference


def test_the_box_is_still_stored_in_backend_pixels():
    a = Anchor(_Blank())
    a.snap(np.zeros((360, 640), np.uint8), roi=(100, 50, 300, 250))
    assert a.reference_roi == pytest.approx((50.0, 100.0 / 3.0, 150.0, 500.0 / 3.0))


def test_the_bank_refuses_it_as_empty():
    from mongla_vision.anchor.bank import CheckpointBank
    b = CheckpointBank(_Blank())
    r = b.enrol(np.zeros((360, 640), np.uint8), roi=(100, 50, 300, 250),
                det_conf=0.9, label='gate', force=True)
    assert not r.accepted and r.reason == "empty"
