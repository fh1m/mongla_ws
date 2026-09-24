"""The checkpoint bank: many references, and a policy for when to add one.

WHY THIS EXISTS, WITH THE MEASUREMENT. `lock_node.py:633` snaps the anchor
reference ONCE -- the guard is literally `not self._anchor.has_reference` -- and
never refreshes it. Measured 2026-09-24 on the archive clips, through the
shipped backend at 320x240, inliers against that one frozen reference:

    clip                +1 s   +3 s   +5 s   +8 s
    mirpur_torpedo       189     65    134     24
    mirpur_torpedo_1      69     43     51     12   <- below the 15 bar
    mirpur_gate          195     36    237     31
    octagon              182    115     73     18
    torpedo (clear)      355    199    121     88

Every clip's minimum is its +8 s column. A reference decays; by 8 s it is close
to worthless, and on the murkiest clip it drops under the trust bar entirely.

The fake backend here is deterministic and carries no ONNX: each "scene" is a
set of identified points, a descriptor is a fixed function of a point's
identity, and a view is a subset of them. So a match between two views of the
same scene is exact, a match against a DIFFERENT scene is empty, and the decay
above is reproduced by dropping identities -- which is what lets the bank's
policy be tested rather than demonstrated.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.anchor.bank import CheckpointBank            # noqa: E402
from mongla_vision.anchor.anchor import MIN_INLIERS             # noqa: E402


# --------------------------------------------------------------------------- #
# A backend that is exactly as clever as the test needs and no cleverer.
# --------------------------------------------------------------------------- #
# One row per possible identity, and BIGGER than the largest identity any test
# builds (scene_id * 1000 + n_points). ⛔ At 8192 rows scene 9 aliased onto
# scene 1 -- 9000 % 8192 = 808 -- so "a different scene" quietly became the same
# scene and the cross-scene refusal test passed nothing. Identity collisions are
# exactly the failure this table exists to make impossible.
_PROJ = np.random.default_rng(0).standard_normal((65536, 64)).astype(np.float32)
_PROJ /= np.linalg.norm(_PROJ, axis=1, keepdims=True)


class FakeBackend:
    """detect() reads a scene out of the image; descriptors follow identity.

    The image is a carrier, not a picture: row 0 holds the point count across
    two cells (it exceeds 255), and each subsequent row holds one point as
    (ident_hi, ident_lo, x, y). Real pixels would make this a test of OpenCV's
    corner detector instead of a test of the bank.
    """
    w, h = 320, 240

    def detect(self, gray):
        n = int(gray[0, 0]) * 256 + int(gray[0, 1])
        if n == 0:
            return np.zeros((0, 2), np.float32), np.zeros((0, 64), np.float32)
        rows = gray[1:1 + n].astype(np.int32)
        ident = rows[:, 0] * 256 + rows[:, 1]
        kp = rows[:, 2:4].astype(np.float32)
        return kp, _PROJ[ident % len(_PROJ)]

    @staticmethod
    def match(d0, d1, min_cossim=0.82, threads=0):
        if len(d0) == 0 or len(d1) == 0:
            return np.zeros(0, np.int64), np.zeros(0, np.int64)
        sim = d0 @ d1.T
        i12 = sim.argmax(axis=1)
        i21 = sim.argmax(axis=0)
        idx0 = np.arange(len(i12))
        mutual = i21[i12] == idx0
        idx0 = idx0[mutual]
        idx1 = i12[idx0]
        keep = sim[idx0, idx1] >= min_cossim
        return idx0[keep], idx1[keep]


def view(scene_id, n_points, *, drop=0):
    """Render one view: identities `drop`..`n_points` of `scene_id`.

    `drop` removes identities from the FRONT, which is how viewpoint change is
    simulated -- points that left frame are gone, and the ones that remain are
    unmoved. That is the shape of the measured decay.

    ⛔ A POINT'S POSITION IS A FUNCTION OF ITS IDENTITY, never of its row. The
    first version of this helper drew positions from a sequential RNG, so
    dropping five identities shifted every remaining point and the homography
    between two views of one scene stopped being the identity -- the test then
    measured the harness rather than the bank. Two coprime multipliers keep the
    points spread without correlating x with y.
    """
    img = np.zeros((512, 4), np.uint8)
    idents = np.arange(drop, n_points) + scene_id * 1000
    img[0, 0] = len(idents) // 256
    img[0, 1] = len(idents) % 256
    for row, ident in enumerate(idents, start=1):
        img[row, 0] = (ident // 256) % 256
        img[row, 1] = ident % 256
        img[row, 2] = 5 + (int(ident) * 37) % 245
        img[row, 3] = 5 + (int(ident) * 101) % 245
    return img


def bank(**kw):
    return CheckpointBank(FakeBackend(), **kw)


# --------------------------------------------------------------------------- #
# 1. The bootstrap. This is the defect that made the ported policy dead code.
# --------------------------------------------------------------------------- #
def test_first_reference_is_enrolled_unconditionally():
    """`dino_idea_v12.py:463` gates enrolment on `template_score > 0.7`, but
    its own `match_template` returns 0.0 for an empty bank -- so the bank can
    never fill, and the author's warning at :995 ("No templates learned yet")
    is that deadlock firing. The first reference cannot be gated on agreeing
    with references that do not exist yet."""
    b = bank()
    assert b.size == 0
    assert b.enrol(view(1, 60), roi=None, det_conf=0.0).accepted
    assert b.size == 1


def test_the_bootstrap_deadlock_is_injection_verified():
    """Break it deliberately: require agreement even when empty, and watch the
    bank stay empty forever. A guard that has never failed is not a guard."""
    b = bank()
    b._require_agreement_always = True           # the v12 rule, restored
    for _ in range(20):
        b.enrol(view(1, 60), roi=None, det_conf=1.0)
    assert b.size == 0, 'the deadlock did not reproduce; the test is wrong'
    b._require_agreement_always = False
    assert b.enrol(view(1, 60), roi=None, det_conf=1.0).accepted


# --------------------------------------------------------------------------- #
# 2. Enrolment policy
# --------------------------------------------------------------------------- #
def test_a_low_confidence_detection_is_refused():
    b = bank(conf_floor=0.5)
    b.enrol(view(1, 60), roi=None, det_conf=0.9)
    r = b.enrol(view(1, 60, drop=5), roi=None, det_conf=0.1)
    assert not r.accepted and r.reason == 'confidence'
    assert b.size == 1


def test_a_view_that_does_not_agree_with_the_bank_is_refused():
    """Scene 2 shares no identities with scene 1, so it matches nothing. This
    is the gate that stops the bank learning a distractor."""
    b = bank()
    b.enrol(view(1, 60), roi=None, det_conf=0.9)
    r = b.enrol(view(2, 60), roi=None, det_conf=0.9)
    assert not r.accepted and r.reason == 'disagrees'
    assert b.size == 1


def test_capacity_is_capped():
    b = bank(capacity=5)
    b.enrol(view(1, 400), roi=None, det_conf=0.9)
    for i in range(1, 12):
        b.enrol(view(1, 400, drop=i * 5), roi=None, det_conf=0.9)
    assert b.size == 5


# --------------------------------------------------------------------------- #
# 3. locate(): best reference wins, and it says WHICH
# --------------------------------------------------------------------------- #
def test_locate_reports_the_best_reference_and_names_it():
    b = bank()
    b.enrol(view(2, 400, drop=380), roi=None, det_conf=0.9)       # poor, few pts
    b.enrol(view(1, 400), roi=None, det_conf=0.9, force=True)     # rich
    p = b.locate(view(1, 400))
    assert p.ok
    assert p.index == 1, 'the reference that actually matches should win'
    assert p.inliers >= MIN_INLIERS


def test_locate_on_a_different_scene_refuses():
    """The cross-clip negative, in miniature. A bank that answers here would
    hand the vehicle a lock on the wrong target, which is worse than LOST."""
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9)
    p = b.locate(view(2, 400))
    assert not p.ok
    assert p.inliers < MIN_INLIERS


def test_an_empty_bank_refuses_rather_than_raising():
    p = bank().locate(view(1, 60))
    assert not p.ok and p.index is None


# --------------------------------------------------------------------------- #
# 4. The meta-updater: WHEN to add a reference
# --------------------------------------------------------------------------- #
def test_quality_drop_asks_for_a_refresh():
    """The +8 s column in this module's docstring, in one assertion: as overlap
    with the stored reference decays, the bank must say so BEFORE the match
    falls under the trust bar."""
    b = bank(refresh_inliers=40)
    b.enrol(view(1, 400), roi=None, det_conf=0.9)
    assert not b.wants_refresh(view(1, 400, drop=10))
    assert b.wants_refresh(view(1, 400, drop=370))


def test_refresh_is_not_asked_for_while_the_match_is_strong():
    b = bank(refresh_inliers=40)
    b.enrol(view(1, 400), roi=None, det_conf=0.9)
    for drop in (0, 5, 20):
        assert not b.wants_refresh(view(1, 400, drop=drop))


# --------------------------------------------------------------------------- #
# 5. Eviction is by USEFULNESS, not by age
# --------------------------------------------------------------------------- #
def test_eviction_drops_the_least_useful_reference_not_the_oldest():
    """The +8 s table says usefulness decays, and age is only a proxy for it.
    The oldest reference is kept when it is still the one that matches."""
    b = bank(capacity=2)
    b.enrol(view(1, 400), roi=None, det_conf=0.9)                        # rich
    b.enrol(view(1, 400, drop=390), roi=None, det_conf=0.9, force=True)  # poor
    for _ in range(3):
        b.locate(view(1, 400))                       # exercises the yields
    b.enrol(view(1, 400, drop=200), roi=None, det_conf=0.9, force=True)
    assert b.size == 2
    assert b.yields[0] == max(b.yields), 'the useful reference was evicted'


# --------------------------------------------------------------------------- #
# 6. verify(): the LTMU verifier the ladder does not have
# --------------------------------------------------------------------------- #
def test_verify_scores_the_same_scene_and_rejects_another():
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9)
    assert b.verify(view(1, 400)) >= MIN_INLIERS
    assert b.verify(view(2, 400)) == 0


@pytest.mark.parametrize('n', [0, 1, 3])
def test_verify_on_a_thin_or_empty_bank_returns_zero_not_a_guess(n):
    b = bank()
    for i in range(n):
        b.enrol(view(1, 400, drop=i * 5), roi=None, det_conf=0.9, force=True)
    assert b.verify(view(9, 400)) == 0


# --------------------------------------------------------------------------- #
# 7. Scale: the shortlist, and a width that is measured rather than configured
# --------------------------------------------------------------------------- #
def test_a_large_bank_only_matches_the_shortlist():
    """The reason a bank may hold a hundred references. Matching is linear --
    measured 31.4 ms per reference on the Pi, so 100 would be 3.1 s against a
    333 ms budget -- and the shortlist is what breaks that linearity."""
    b = bank(capacity=40)
    for i in range(30):
        b.enrol(view(1, 400, drop=i * 5), roi=None, det_conf=0.9, force=True)
    assert b.size == 30
    p = b.locate(view(1, 400), shortlist=4)
    assert p.searched == 4, 'the whole bank was matched, not the shortlist'
    assert p.ok


def test_the_shortlist_never_searches_more_than_the_bank_holds():
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9)
    assert b.locate(view(1, 400), shortlist=9).searched == 1


def test_shortlist_width_falls_when_a_match_is_measured_to_be_slow():
    """⭐ The width is DERIVED from the cost measured on the machine it runs
    on. A constant would be wrong on one of dev box and Pi by 2.2x."""
    b = bank(period_s=0.333, max_shortlist=8, budget_fraction=0.45)
    assert b.shortlist_k() == 8, 'untimed, it must start at the ceiling'
    b._match_s = 0.0314                       # the Pi's measured 31.4 ms
    assert b.shortlist_k() == 4               # 0.333*0.45/0.0314
    b._match_s = 0.0144                       # the dev box's 14.4 ms
    assert b.shortlist_k() == 8               # affords the ceiling
    b._match_s = 0.5                          # absurdly slow
    assert b.shortlist_k() == 1, 'it must still search ONE, never zero'


def test_without_a_period_the_width_is_the_ceiling():
    """No budget declared means the caller has not asked for rationing, and
    silently rationing anyway would be a performance cliff nobody configured."""
    b = bank(max_shortlist=6)
    b._match_s = 1.0
    assert b.shortlist_k() == 6


def test_the_measured_match_cost_is_recorded():
    b = bank()
    assert b.match_ms == 0.0
    b.enrol(view(1, 400), roi=None, det_conf=0.9)
    b.locate(view(1, 400))
    assert b.match_ms > 0.0


# --------------------------------------------------------------------------- #
# 8. One bank, many things: props, checkpoints, places
# --------------------------------------------------------------------------- #
def test_a_label_restricts_the_search_to_one_kind_of_thing():
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9, label='gate')
    b.enrol(view(2, 400), roi=None, det_conf=0.9, label='torpedo', force=True)
    assert b.locate(view(2, 400), label='torpedo').ok
    assert not b.locate(view(2, 400), label='gate').ok


def test_locate_reports_WHAT_it_recognised_not_only_where():
    """Re-identification with the same machinery as tracking: searching every
    label answers "what is this", not merely "is this the gate"."""
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9, label='gate')
    b.enrol(view(2, 400), roi=None, det_conf=0.9, label='torpedo', force=True)
    b.enrol(view(3, 400), roi=None, det_conf=0.9, label='place:octagon',
            force=True)
    assert b.locate(view(3, 400)).label == 'place:octagon'
    assert b.locate(view(1, 400)).label == 'gate'


def test_an_unknown_label_refuses_rather_than_falling_back_to_everything():
    """Falling back to the whole bank would answer a question nobody asked,
    with a different target, at full confidence."""
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9, label='gate')
    p = b.locate(view(1, 400), label='bin')
    assert not p.ok and p.index is None


def test_verify_can_be_asked_about_one_label():
    b = bank()
    b.enrol(view(1, 400), roi=None, det_conf=0.9, label='gate')
    assert b.verify(view(1, 400), label='gate') >= MIN_INLIERS
    assert b.verify(view(1, 400), label='torpedo') == 0
