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
    """Returns a fixed keypoint grid; match models a physical AUV drift.

    DRIFT_RIGHT_PX = how far the AUV has strafed RIGHT of the reference pose. A
    forward camera then sees the scene shifted LEFT, so the live (current)
    features sit DRIFT_RIGHT_PX to the left of the reference features. The
    matcher's ref->live homography must report tx < 0 (reference appears left ->
    strafe LEFT to close the loop) -- this is what makes the control negative
    feedback rather than runaway.
    """
    DRIFT_RIGHT_PX = 0.0

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
        # d0 = current (live), d1 = reference. AUV drifted right -> live features
        # shifted left -> cur_x = ref_x - DRIFT_RIGHT_PX. xfeat.match then computes
        # findHomography(ref, cur) (ref->live), so tx = -DRIFT_RIGHT_PX.
        ref = np.asarray(d1['keypoints'], dtype=np.float32)
        cur = ref.copy()
        cur[:, 0] -= self.DRIFT_RIGHT_PX
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


def _matcher(monkeypatch, drift_right=0.0):
    _fake_torch(monkeypatch)
    _FakeXFeatModel.DRIFT_RIGHT_PX = drift_right
    from duburi_vision.anchor.xfeat import XFeatMatcher
    return XFeatMatcher(top_k=64, device='cpu')


def test_match_none_before_reference(monkeypatch):
    m = _matcher(monkeypatch)
    assert m.is_loaded() is True
    assert m.has_reference() is False
    assert m.match(np.zeros((480, 640, 3), dtype=np.uint8)) is None


def test_same_frame_near_zero_error(monkeypatch):
    m = _matcher(monkeypatch, drift_right=0.0)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame) is True
    e = m.match(frame)
    assert e is not None
    assert abs(e.tx_px) < 1e-3
    assert abs(e.ty_px) < 1e-3


def test_rightward_drift_gives_negative_tx(monkeypatch):
    # PHYSICS, not convention: AUV strafed 30px RIGHT of the reference -> the
    # reference content now appears LEFT in the live frame -> tx < 0 so the
    # control law (strafe toward +tx) drives LEFT, opposing the drift. A
    # positive tx here would mean the loop runs AWAY from the lock (the bug).
    m = _matcher(monkeypatch, drift_right=30.0)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame) is True
    e = m.match(frame)
    assert e is not None
    assert e.tx_px == pytest.approx(-30.0, abs=2.0)
    assert e.n_inliers >= 8


# --------------------------------------------------------------------------- #
#  references -- disk save/load round-trip                                    #
# --------------------------------------------------------------------------- #
def test_reference_save_load_roundtrip(monkeypatch, tmp_path):
    from duburi_vision.anchor import references
    monkeypatch.setattr(references, 'references_dir', lambda: tmp_path)
    frame = np.full((48, 64, 3), 127, dtype=np.uint8)
    frame[10:20, 30:40] = 255
    assert references.load_reference('hole') == (None, None)   # absent
    path = references.save_reference('hole', frame)
    assert path is not None and path.exists()
    back, bbox = references.load_reference('hole')
    assert back is not None and back.shape == frame.shape and bbox is None


def test_reference_name_is_sanitised(monkeypatch, tmp_path):
    from duburi_vision.anchor import references
    monkeypatch.setattr(references, 'references_dir', lambda: tmp_path)
    # Path-traversal / spaces must not escape the references dir.
    p = references.reference_path('../../etc/pwn name')
    assert p.parent == tmp_path
    assert '/' not in p.name[:-4]            # only the .png slash-free stem


def test_reference_bbox_sidecar_roundtrip(monkeypatch, tmp_path):
    from duburi_vision.anchor import references
    monkeypatch.setattr(references, 'references_dir', lambda: tmp_path)
    frame = np.full((48, 64, 3), 100, dtype=np.uint8)
    # Whole-frame: no sidecar, bbox None on load.
    references.save_reference('whole', frame)
    img, bbox = references.load_reference('whole')
    assert img is not None and bbox is None
    # Crop: sidecar written, bbox restored on load.
    references.save_reference('crop', frame, bbox=(10, 12, 40, 44))
    img, bbox = references.load_reference('crop')
    assert img is not None and bbox == (10, 12, 40, 44)


# --------------------------------------------------------------------------- #
#  crop reference -- keypoints offset into full-frame coords                  #
# --------------------------------------------------------------------------- #
def test_crop_reference_offsets_keypoints_to_full_frame(monkeypatch):
    # A bbox crop must store keypoints in FULL-frame coords (offset by x1,y1)
    # and full-frame image_size, so extract_error -- and its sign -- is unchanged.
    m = _matcher(monkeypatch, drift_right=0.0)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    bbox = (200, 150, 360, 330)   # off-centre-ish crop
    assert m.set_reference(frame, bbox=bbox) is True
    ref = m._ref
    assert ref['image_size'] == (640, 480)          # FULL frame, not crop
    kp = np.asarray(ref['keypoints'])
    # The fake grid spans 20-80% of the CROP; offset must push them past x1=200.
    assert kp[:, 0].min() >= 200 - 1
    assert kp[:, 0].max() <= 360 + 1


def test_crop_reference_preserves_sign(monkeypatch):
    # Rightward drift with a crop reference must STILL give tx<0 (the sign fix
    # is inherited because keypoints live in full-frame coords). Note: this
    # exercises the OFFSET WIRING + sign math; the fake match ignores live
    # descriptors, so real-model crop-match quality is pool-gated.
    m = _matcher(monkeypatch, drift_right=25.0)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame, bbox=(240, 180, 400, 300)) is True
    e = m.match(frame)
    assert e is not None
    assert e.tx_px < 0


# --------------------------------------------------------------------------- #
#  crop_gate -- detection-gated snap selection (pure)                         #
# --------------------------------------------------------------------------- #
def _det(cls, score, cx, cy, sx, sy):
    """Hand-built Detection2D duck-type for the gate."""
    from types import SimpleNamespace as NS
    return NS(
        bbox=NS(center=NS(position=NS(x=cx, y=cy)), size_x=sx, size_y=sy),
        results=[NS(hypothesis=NS(class_id=cls, score=score))])


def test_qualifying_bbox_picks_centred_target():
    from duburi_vision.anchor.crop_gate import qualifying_bbox
    dets = [_det('hole', 0.8, 320, 240, 80, 80),     # centred, qualifies
            _det('gate', 0.9, 320, 240, 200, 200)]   # wrong class
    box = qualifying_bbox(dets, 640, 480, 'hole', conf=0.5, err_px=40, pad=1.0)
    assert box == (280, 200, 360, 280)               # 80x80 around (320,240)


def test_qualifying_bbox_case_insensitive_and_conf_gate():
    from duburi_vision.anchor.crop_gate import qualifying_bbox
    dets = [_det('HOLE', 0.4, 320, 240, 80, 80)]     # right class, too low conf
    assert qualifying_bbox(dets, 640, 480, 'hole', conf=0.5, err_px=0) is None
    dets = [_det('HOLE', 0.7, 320, 240, 80, 80)]     # case-insensitive match
    assert qualifying_bbox(dets, 640, 480, 'hole', conf=0.5, err_px=0) is not None


def test_qualifying_bbox_centre_gate_rejects_off_centre():
    from duburi_vision.anchor.crop_gate import qualifying_bbox
    # cx=500 is 180px right of centre (320) -> outside err=40 -> rejected.
    dets = [_det('hole', 0.9, 500, 240, 80, 80)]
    assert qualifying_bbox(dets, 640, 480, 'hole', conf=0.5, err_px=40) is None
    # err<=0 disables the gate -> the same off-centre detection qualifies.
    assert qualifying_bbox(dets, 640, 480, 'hole', conf=0.5, err_px=0) is not None


def test_qualifying_bbox_pads_and_clamps_to_frame():
    from duburi_vision.anchor.crop_gate import qualifying_bbox
    # Near the left edge: pad would go negative -> clamp x1 to 0.
    dets = [_det('hole', 0.9, 30, 240, 80, 80)]
    box = qualifying_bbox(dets, 640, 480, 'hole', conf=0.5, err_px=0, pad=1.3)
    assert box is not None and box[0] == 0


def test_qualifying_bbox_none_when_absent():
    from duburi_vision.anchor.crop_gate import qualifying_bbox
    dets = [_det('gate', 0.9, 320, 240, 80, 80)]
    assert qualifying_bbox(dets, 640, 480, 'hole', conf=0.5, err_px=40) is None
    assert qualifying_bbox([], 640, 480, 'hole', conf=0.5, err_px=40) is None


# --------------------------------------------------------------------------- #
#  empty / low-texture keypoint guard (the IndexError crash root cause)        #
# --------------------------------------------------------------------------- #
def _matcher_model(monkeypatch, model):
    """Build an XFeatMatcher backed by an arbitrary fake hub model."""
    fake_torch = types.ModuleType('torch')

    class _Cuda:
        @staticmethod
        def is_available():
            return False
    fake_torch.cuda = _Cuda()
    fake_torch.hub = types.SimpleNamespace(load=lambda *a, **k: model)
    monkeypatch.setitem(sys.modules, 'torch', fake_torch)
    from duburi_vision.anchor.xfeat import XFeatMatcher
    return XFeatMatcher(top_k=64, device='cpu')


def _kp_dict(n):
    kpts = np.array([[10.0 + i, 10.0 + i] for i in range(n)], dtype=np.float32)
    return [{
        'keypoints':   kpts,
        'scores':      np.ones(len(kpts), dtype=np.float32),
        'descriptors': np.zeros((len(kpts), 64), dtype=np.float32),
    }]


class _FewKPModel(_FakeXFeatModel):
    """detectAndCompute returns too few keypoints (low-texture frame)."""
    N = 2

    def detectAndCompute(self, frame, top_k=None):
        return _kp_dict(self.N)


class _DwindlingModel(_FakeXFeatModel):
    """16 keypoints for the reference snap, then 2 -- and match must NOT be
    called on the too-few frame (that is the IndexError path)."""
    def __init__(self):
        self._calls = 0

    def detectAndCompute(self, frame, top_k=None):
        self._calls += 1
        return _kp_dict(16 if self._calls == 1 else 2)

    def match_lighterglue(self, *a, **k):
        raise AssertionError('match_lighterglue must not run on <4 keypoints')


def test_set_reference_rejects_low_texture(monkeypatch):
    m = _matcher_model(monkeypatch, _FewKPModel())
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame) is False     # 2 kp < _MIN_KP_REF -> rejected
    assert m.has_reference() is False


def test_match_short_circuits_on_few_keypoints(monkeypatch):
    # Reference snaps fine (16 kp); the live frame then yields 2 kp. match()
    # must short-circuit to a clean no-lock result WITHOUT entering
    # match_lighterglue (which raises IndexError on an empty reduction).
    m = _matcher_model(monkeypatch, _DwindlingModel())
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame) is True
    e = m.match(frame)                          # would AssertionError if it ran
    assert e is not None and e.n_inliers == 0
    assert m.last_match() is None


def test_last_match_populated_after_good_match(monkeypatch):
    m = _matcher(monkeypatch, drift_right=10.0)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert m.set_reference(frame) is True
    assert m.match(frame) is not None
    lm = m.last_match()
    assert lm is not None and len(lm) == 3      # (ref, cur, mask)


# --------------------------------------------------------------------------- #
#  match overlay -- pure draw (no ROS)                                         #
# --------------------------------------------------------------------------- #
def test_draw_match_overlay_runs_and_is_none_safe():
    from duburi_vision.anchor.overlay import draw_match_overlay
    img = np.zeros((100, 120, 3), dtype=np.uint8)
    ref  = np.array([[10, 10], [50, 50], [80, 40]], dtype=np.float64)
    cur  = np.array([[12, 11], [48, 52], [82, 39]], dtype=np.float64)
    mask = np.array([[1], [1], [0]], dtype=np.uint8)
    out = draw_match_overlay(img, (ref, cur, mask), (5, 5, 90, 90), min_inliers=2)
    assert out.shape == img.shape
    assert (out[:, :, 1] > 0).any()             # something green was drawn
    # None match (no lock) must not raise and still annotates.
    assert draw_match_overlay(img, None, None, min_inliers=2).shape == img.shape
