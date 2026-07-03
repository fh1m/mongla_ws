"""Unit tests for FeatureFusedVisionState -- the use_feature detection fallback.

Pin the v1 fusion contract (no ROS spun -- fakes for VisionState + AnchorState):
  * detection PRIMARY: a real bbox_error Sample is returned unchanged;
  * substitute ONLY when the detector returns None AND the anchor is LOCKED;
  * substitute ex/ey come from homography tx/ty normalized by the REAL image
    size (not a hardcoded 320/240 half-frame);
  * theta is ignored; the substitute is coasted=False (so it keeps authority);
  * no lock / no anchor -> degrade to detection-only (None).
"""

import pytest

from duburi_manager.vision_state import FeatureFusedVisionState, Sample
from duburi_manager.anchor_state import AnchorSample, STATE_LOCKED, STATE_LOST


class _FakeVision:
    def __init__(self, sample, size=(640, 480)):
        self._sample = sample
        self._size = size
        self.calls = []

    def image_size(self):
        return self._size

    def bbox_error(self, class_name='', **kw):
        self.calls.append((class_name, kw))
        return self._sample

    def is_fresh(self, *_a, **_k):     # a method the adapter must delegate
        return 'delegated'


class _FakeAnchor:
    def __init__(self, pose):
        self._pose = pose

    def pose(self):
        return self._pose


def _det(ex=0.2, ey=-0.1):
    return Sample(ex=ex, ey=ey, h_frac=0.3, w_frac=0.2, age_s=0.05,
                  class_id='hole', score=0.9)


def _pose(tx, ty, state=STATE_LOCKED, age=0.04):
    return AnchorSample(tx_px=tx, ty_px=ty, theta_rad=0.5,   # theta must be IGNORED
                        state=state, conf=25.0, age_s=age)


# --------------------------------------------------------------------------- #
def test_detection_present_is_returned_unchanged():
    det = _det()
    fused = FeatureFusedVisionState(_FakeVision(det), _FakeAnchor(_pose(320, 240)))
    out = fused.bbox_error('hole')
    assert out is det                       # detection PRIMARY, no substitution


def test_substitutes_anchor_when_detection_absent_and_locked():
    # image 640x480 -> half = (320, 240); tx=320 -> ex=+1.0, ty=-120 -> ey=-0.5
    fused = FeatureFusedVisionState(_FakeVision(None, size=(640, 480)),
                                    _FakeAnchor(_pose(320, -120)))
    out = fused.bbox_error('hole')
    assert out is not None
    assert out.ex == pytest.approx(1.0)
    assert out.ey == pytest.approx(-0.5)
    assert out.coasted is False             # must keep freshness authority
    assert out.age_s == pytest.approx(0.04) # the anchor's real age
    assert out.class_id == 'hole'


def test_normalizes_by_real_image_size_not_320_240():
    # a 1280x720 frame: tx=320 -> ex=320/640=0.5 (NOT 320/320=1.0 of a fixed half)
    fused = FeatureFusedVisionState(_FakeVision(None, size=(1280, 720)),
                                    _FakeAnchor(_pose(320, 180)))
    out = fused.bbox_error('hole')
    assert out.ex == pytest.approx(0.5)
    assert out.ey == pytest.approx(0.5)     # 180 / 360


def test_no_substitute_when_anchor_not_locked():
    fused = FeatureFusedVisionState(_FakeVision(None),
                                    _FakeAnchor(_pose(320, 240, state=STATE_LOST)))
    assert fused.bbox_error('hole') is None  # only substitute on a real LOCK


def test_no_substitute_when_no_anchor_pose():
    fused = FeatureFusedVisionState(_FakeVision(None), _FakeAnchor(None))
    assert fused.bbox_error('hole') is None


def test_no_anchor_object_degrades_to_detection_only():
    fused = FeatureFusedVisionState(_FakeVision(None), None)
    assert fused.bbox_error('hole') is None


def test_image_size_and_unknown_methods_delegate():
    fv = _FakeVision(None, size=(800, 600))
    fused = FeatureFusedVisionState(fv, _FakeAnchor(None))
    assert fused.image_size() == (800, 600)
    assert fused.is_fresh(1.0) == 'delegated'   # __getattr__ passthrough


def test_substitute_clamps_to_band():
    # tx far beyond the frame -> ex clamped to +1.5 (align_loop's band)
    fused = FeatureFusedVisionState(_FakeVision(None, size=(640, 480)),
                                    _FakeAnchor(_pose(9999, 9999)))
    out = fused.bbox_error('hole')
    assert out.ex == pytest.approx(1.5)
    assert out.ey == pytest.approx(1.5)
