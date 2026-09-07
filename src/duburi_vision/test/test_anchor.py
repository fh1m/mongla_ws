"""The anchor: hold a target with NO detection.

Measured end to end on real competition footage -- detector and anchor over the
SAME 600 consecutive frames, reference snapped on the first frame the detector
was confident:

    clip              det seen   blind   anchor held   rescued
    torp_up_1          425/600     175           175      100 %
    bin_front_1         12/600      39            39      100 %
    gate_back            2/600       1             1      100 %
    octagon_Bottom     600/600       0             -        -

On `torp_up_1` the detector lost the target for 175 consecutive frames and the
anchor produced a trusted pose on every one.

WHAT THE ANCHOR ACTUALLY HOLDS, stated so it is not overclaimed: the snapped
VIEW, not the prop. For an approach or a station-keep those are the same thing.
For a target that moves independently of the scene they are not, and the anchor
would hold the background -- which is why it is a rung beneath the detector and
never a replacement for it.

The tests below are about the SIGN and the REFUSALS, because those are what make
it safe to act on. A test that only checked "a pose comes out" would pass for a
homography fitted to noise.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.anchor.anchor import (            # noqa: E402
    Anchor, AnchorPose, MIN_INLIERS, pose_from_homography)


# --------------------------------------------------------------------------- #
#  The sign convention -- reversing it is positive feedback
# --------------------------------------------------------------------------- #
def test_a_reference_shifted_RIGHT_reports_positive_tx():
    """THE load-bearing sign. `H` maps REFERENCE -> LIVE, so the reference
    centre pushed through it is "where the reference appears now", which must
    carry the SAME sign as a detection's offset-from-centre (+x = right).

    Swapping the argument order to findHomography inverts every axis into
    positive feedback and the hull drives AWAY from the lock. That was a real
    bug in an earlier attempt at this, and it does not announce itself -- the
    pose still looks like a pose."""
    H = np.array([[1, 0, 40.0], [0, 1, 0], [0, 0, 1]])     # live is ref + 40 px
    tx, ty, theta, scale = pose_from_homography(H, 320, 240)
    assert tx == pytest.approx(40.0)
    assert ty == pytest.approx(0.0)


def test_a_reference_shifted_DOWN_reports_positive_ty():
    H = np.array([[1, 0, 0], [0, 1, 25.0], [0, 0, 1]])
    tx, ty, _t, _s = pose_from_homography(H, 320, 240)
    assert ty == pytest.approx(25.0)


def test_scale_comes_from_the_determinant_not_from_H00():
    """A pure ROTATION must not read as a scale change -- they drive different
    axes. |H[0,0]| would report cos(theta) and shrink the apparent range every
    time the hull rolls."""
    import math
    a = math.radians(30)
    R = np.array([[math.cos(a), -math.sin(a), 0],
                  [math.sin(a),  math.cos(a), 0], [0, 0, 1]])
    _tx, _ty, theta, scale = pose_from_homography(R, 320, 240)
    assert theta == pytest.approx(a, abs=1e-6)
    assert scale == pytest.approx(1.0, abs=1e-6)      # NOT cos(30) = 0.866


def test_scale_above_one_means_the_reference_got_BIGGER():
    H = np.diag([2.0, 2.0, 1.0])
    _tx, _ty, _th, scale = pose_from_homography(H, 320, 240)
    assert scale == pytest.approx(2.0)


# --------------------------------------------------------------------------- #
#  The refusals -- what makes a pose safe to act on
# --------------------------------------------------------------------------- #
class _Backend:
    """A stand-in with the real interface. No ONNX: what is under test here is
    the REFUSAL logic, which is exercisable without a network, and the network
    is measured against torch XFeat separately on real frames."""
    h, w = 240, 320

    def __init__(self, n=200, seed=0):
        rng = np.random.default_rng(seed)
        self.k = rng.random((n, 2)).astype(np.float32) * [self.w, self.h]
        d = rng.standard_normal((n, 64)).astype(np.float32)
        self.d = d / np.linalg.norm(d, axis=1, keepdims=True)

    def detect(self, gray):
        return self.k, self.d

    @staticmethod
    def match(d0, d1, min_cossim=0.82):
        n = min(len(d0), len(d1))
        return np.arange(n), np.arange(n)


class _MatchingBackend(_Backend):
    """`_Backend` matches by INDEX, which is only correct when the reference is
    the whole frame -- an ROI reference is a subset, so index i on each side is
    a different point and the homography is fitted to nonsense. That is a
    property of the fake, not of the anchor, and it is exactly the kind of
    harness error that reads as a code defect. This one matches on the
    descriptors, the way the real backend does."""

    def match(self, d0, d1, min_cossim=0.82):
        sim = d0 @ d1.T
        j = sim.argmax(axis=1)
        i = np.arange(len(d0))
        keep = sim[i, j] >= min_cossim
        return i[keep], j[keep]


def test_no_reference_means_no_pose():
    a = Anchor(_Backend())
    assert not a.has_reference
    assert a.locate(np.zeros((240, 320), np.uint8)).ok is False


def test_snap_reports_the_keypoint_count_as_a_health_signal():
    """Returned rather than a bool because it separates a workable reference
    (~1000 keypoints on our footage) from a hopeless one -- ORB found SEVEN in
    an entire Mirpur frame. A caller that snaps an empty reference must learn
    it now, not when the lock silently never engages."""
    a = Anchor(_Backend(n=137))
    assert a.snap(np.zeros((240, 320), np.uint8)) == 137
    assert a.reference_keypoints == 137


def test_an_identical_view_locates_with_many_inliers():
    a = Anchor(_Backend(n=200))
    a.snap(np.zeros((240, 320), np.uint8))
    p = a.locate(np.zeros((240, 320), np.uint8))
    assert p.ok and p.inliers >= MIN_INLIERS
    assert abs(p.tx) < 1e-3 and abs(p.ty) < 1e-3      # same view -> no offset


def test_too_few_inliers_REFUSES_rather_than_returning_a_weak_pose():
    """Below the bar a homography is not less accurate, it is arbitrary.
    Returning it with a low confidence would invite a caller to steer on it."""
    a = Anchor(_Backend(n=200), min_inliers=100000)
    a.snap(np.zeros((240, 320), np.uint8))
    p = a.locate(np.zeros((240, 320), np.uint8))
    assert p.ok is False
    assert p.inliers > 0            # it DID match -- the refusal is the gate


def test_clear_drops_the_reference():
    a = Anchor(_Backend())
    a.snap(np.zeros((240, 320), np.uint8))
    a.clear()
    assert not a.has_reference


def test_confidence_is_zero_when_not_ok():
    """The control path treats this like detection confidence, so a refused
    pose must not carry usable confidence."""
    assert AnchorPose(ok=False, inliers=900).confidence == 0.0
    assert AnchorPose(ok=True, inliers=100).confidence == pytest.approx(1.0)
    assert 0.0 < AnchorPose(ok=True, inliers=20).confidence < 1.0


# --------------------------------------------------------------------------- #
#  Target vs scene -- the distinction that surprises everyone
# --------------------------------------------------------------------------- #
def test_an_ROI_snap_keeps_ONLY_the_keypoints_inside_the_box():
    """Locking the whole frame means hundreds of background keypoints outvote
    the subject, so the homography reports what the ROOM is doing -- on a
    static camera the lock correctly sits still while someone walks through it.
    That is right for station-keeping and useless for following a prop.

    An ROI snap keys the reference on the target's box instead. This is what a
    torpedo run needs: lock the BOARD, not the pool wall behind it."""
    be = _Backend(n=400)
    a = Anchor(be)
    whole = a.snap(np.zeros((240, 320), np.uint8))
    part = a.snap(np.zeros((240, 320), np.uint8), roi=(0, 0, 160, 120))
    assert whole == 400
    assert 0 < part < whole                    # a real subset, not all or none


def test_ROI_keypoints_keep_FULL_FRAME_coordinates():
    """Cropping the IMAGE would shift the origin and silently move every pose
    the anchor reports. Filtering keypoints leaves every sign and scale
    downstream identical, so `pose_from_homography` never needs to know which
    mode was used."""
    be = _Backend(n=400)
    a = Anchor(be)
    a.snap(np.zeros((240, 320), np.uint8), roi=(100, 80, 200, 160))
    k = a._ref_kpts
    assert len(k) > 0
    # inside the ROI in FULL-frame terms -- not re-based to the crop
    assert k[:, 0].min() >= 100 - 1e-6 and k[:, 0].max() <= 200 + 1e-6
    assert k[:, 1].min() >= 80 - 1e-6 and k[:, 1].max() <= 160 + 1e-6


def test_the_ROI_is_scaled_from_the_CALLERS_resolution():
    """The backend works at its own size; the box arrives in the caller's. Not
    scaling it would silently select the wrong region -- and a wrong region
    still produces keypoints, a homography and a confident pose."""
    be = _Backend(n=400)
    a = Anchor(be)
    # a 640x480 caller asking for its own left half must select the backend's
    # left half, not a 320-wide slab of a 320-wide image (i.e. everything)
    n_half = a.snap(np.zeros((480, 640), np.uint8), roi=(0, 0, 320, 480))
    n_all = a.snap(np.zeros((480, 640), np.uint8), roi=(0, 0, 640, 480))
    assert n_half < n_all


def test_a_locked_pose_carries_the_reference_FOOTPRINT():
    """A point says the lock moved; the quad says what it is locked onto -- and
    a footprint leaving the frame is the earliest warning that the reference is
    about to become unusable, which a centre offset cannot express."""
    a = Anchor(_Backend(n=200))
    a.snap(np.zeros((240, 320), np.uint8))
    p = a.locate(np.zeros((240, 320), np.uint8))
    assert p.ok and p.corners is not None
    assert np.asarray(p.corners).shape == (4, 2)


# --------------------------------------------------------------------------- #
#  The ROI has to reach the FOOTPRINT and the POSE, not just the keypoints
# --------------------------------------------------------------------------- #
def test_an_ROI_anchor_reports_the_ROIs_footprint_not_the_whole_frame():
    """The bug this guards: `reference_corners` was fitted with a whole-frame
    quad regardless of the ROI, so an ROI-snapped anchor reported a footprint
    covering the ENTIRE image -- and `lock_node` derives its published target
    box from that quad. The result is a confident, useless answer that looks
    completely normal until you draw it."""
    a = Anchor(_MatchingBackend(n=2000))
    a.snap(np.zeros((240, 320), np.uint8), roi=(100, 80, 200, 160))
    p = a.locate(np.zeros((240, 320), np.uint8))
    assert p.ok
    q = np.asarray(p.corners)
    assert q[:, 0].min() == pytest.approx(100, abs=2)
    assert q[:, 0].max() == pytest.approx(200, abs=2)
    assert q[:, 1].min() == pytest.approx(80, abs=2)
    assert q[:, 1].max() == pytest.approx(160, abs=2)


def test_an_ROI_pose_measures_the_TARGETS_offset_not_the_views():
    """Same fix, the half that steers. The point pushed through H is the
    REGION's centre; the offset is still measured from the FRAME centre. An
    off-centre ROI on an unmoved scene therefore reports a real offset -- which
    is what a lock on a prop sitting to one side must say. Whole-frame maths
    reports 0.0 and the hull holds a target it is not pointed at."""
    a = Anchor(_MatchingBackend(n=2000))
    a.snap(np.zeros((240, 320), np.uint8), roi=(100, 80, 200, 160))
    p = a.locate(np.zeros((240, 320), np.uint8))
    assert p.ok
    # ROI centre 150,120 against frame centre 160,120
    assert p.tx == pytest.approx(-10.0, abs=2.0)
    assert p.ty == pytest.approx(0.0, abs=2.0)


def test_a_locked_pose_carries_the_surviving_CORRESPONDENCES():
    """An inlier count says the match is good; it cannot say good *at what*.
    Two hundred inliers on the pool wall and two hundred on the torpedo board
    are the same number and completely different situations."""
    a = Anchor(_Backend(n=200))
    a.snap(np.zeros((240, 320), np.uint8))
    p = a.locate(np.zeros((240, 320), np.uint8))
    assert p.ok
    assert np.asarray(p.ref_pts).shape == np.asarray(p.live_pts).shape
    assert len(p.ref_pts) == p.inliers, 'inliers only, not every putative match'
