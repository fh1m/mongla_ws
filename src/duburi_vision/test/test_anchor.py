"""Anchor matcher + homography tests.

homography.extract_error is pure (hand-built matrices). XFeatMatcher is tested
against a MONKEYPATCHED fake hub model -- never loads real torch.hub -- mirroring
the _FakeYOLO approach in test_detector.py.
"""

import sys
import types
import numpy as np
import pytest

from duburi_vision.anchor.homography import extract_error
from duburi_vision.anchor.anchor import AnchorError


# --------------------------------------------------------------------------- #
#  homography.extract_error -- pure                                            #
# --------------------------------------------------------------------------- #
def test_identity_homography_is_zero_error():
    H = np.eye(3)
    e = extract_error(H, (480, 640), n_inliers=50, confidence=0.9)
    assert abs(e.tx_px) < 1e-6
    assert abs(e.ty_px) < 1e-6
    assert abs(e.theta_rad) < 1e-6
    assert e.n_inliers == 50


def test_translation_homography_gives_positive_tx():
    # H maps live->reference; a +40px x-shift means the reference centre lands
    # 40px to the right -> +tx.
    H = np.array([[1.0, 0.0, 40.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]])
    e = extract_error(H, (480, 640))
    assert e.tx_px == pytest.approx(40.0, abs=1e-6)
    assert e.ty_px == pytest.approx(0.0, abs=1e-6)


def test_rotation_homography_gives_theta():
    import math
    a = math.radians(10.0)
    R = np.array([[math.cos(a), -math.sin(a), 0.0],
                  [math.sin(a),  math.cos(a), 0.0],
                  [0.0,          0.0,         1.0]])
    e = extract_error(R, (480, 640))
    assert e.theta_rad == pytest.approx(a, abs=1e-4)


def test_degenerate_homography_is_safe():
    e = extract_error(None, (480, 640))
    assert isinstance(e, AnchorError)
    assert e.tx_px == 0.0 and e.ty_px == 0.0


# --------------------------------------------------------------------------- #
#  XFeatMatcher -- fake hub model                                             #
# --------------------------------------------------------------------------- #
class _FakeXFeatModel:
    """Returns a fixed keypoint grid; match shifts current pts by +dx in x.

    set DX to control the horizontal shift between current and reference
    correspondences so findHomography recovers a known tx.
    """
    DX = 0.0

    def to(self, _device):
        return self

    def detectAndCompute(self, frame, top_k=None):
        # A deterministic 4x4 grid of keypoints spread across the frame.
        h, w = frame.shape[:2]
        xs = np.linspace(0.2 * w, 0.8 * w, 4)
        ys = np.linspace(0.2 * h, 0.8 * h, 4)
        kpts = np.array([[x, y] for y in ys for x in xs], dtype=np.float32)
        return [{
            'keypoints':   kpts,
            'scores':      np.ones(len(kpts), dtype=np.float32),
            'descriptors': np.zeros((len(kpts), 64), dtype=np.float32),
        }]

    def match_lighterglue(self, d0, d1, min_conf=0.1):
        # d0 = current, d1 = reference. Current pts are the reference pts shifted
        # by +DX in x, so H(current->reference) recovers -DX... we want the live
        # centre to map +DX when the reference sits to the right: build current
        # = reference - DX so applying H to live moves it +DX. Keep it simple:
        # current keypoints are reference shifted by -DX.
        ref = np.asarray(d1['keypoints'], dtype=np.float32)
        cur = ref.copy()
        cur[:, 0] -= self.DX
        idx = np.stack([np.arange(len(ref)), np.arange(len(ref))], axis=1)
        return cur, ref, idx


def _fake_torch(monkeypatch):
    fake_torch = types.ModuleType('torch')

    class _Cuda:
        @staticmethod
        def is_available():
            return False
    fake_torch.cuda = _Cuda()

    def _hub_load(repo, name, **kw):
        return _FakeXFeatModel()
    fake_torch.hub = types.SimpleNamespace(load=_hub_load)
    monkeypatch.setitem(sys.modules, 'torch', fake_torch)
    return fake_torch


def _matcher(monkeypatch, dx=0.0):
    _fake_torch(monkeypatch)
    _FakeXFeatModel.DX = dx
    from duburi_vision.anchor.xfeat import XFeatMatcher
    return XFeatMatcher(top_k=64, device='cpu')


def test_match_none_before_reference(monkeypatch):
    m = _matcher(monkeypatch)
    assert m.is_loaded() is True
    assert m.has_reference() is False
    assert m.match(np.zeros((480, 640, 3), dtype=np.uint8)) is None


def test_same_frame_near_zero_error(monkeypatch):
    m = _matcher(monkeypatch, dx=0.0)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame) is True
    e = m.match(frame)
    assert e is not None
    assert abs(e.tx_px) < 1e-3
    assert abs(e.ty_px) < 1e-3


def test_shifted_frame_positive_tx(monkeypatch):
    # Reference is 30px to the right of the current view -> +tx.
    m = _matcher(monkeypatch, dx=30.0)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame) is True
    e = m.match(frame)
    assert e is not None
    assert e.tx_px == pytest.approx(30.0, abs=2.0)
    assert e.n_inliers >= 8
