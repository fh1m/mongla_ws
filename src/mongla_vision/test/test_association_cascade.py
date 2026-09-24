"""The cascade must explain what it can cheaply, and refuse to invent.

Three claims under test:
  1. each stage sees only what the previous could not match -- that ordering
     is what keeps a 15-25 ms appearance stage off the hot path;
  2. ego-motion is MEASURED, so an unknown shift SKIPS the stage rather than
     asserting the vehicle did not move;
  3. a detection no stage explains becomes a new track, honestly.
"""
import math

import pytest

from mongla_vision.tracking.cascade import (APPEARANCE, EGO, MOTION, NEW,
                                            AssociationCascade,
                                            ego_pixel_shift, iou, shift_box)


def box(x, y, w=40, h=40):
    return (x, y, x + w, y + h)


# --------------------------------------------------------------------------- #
#  Stage 1 -- motion alone
# --------------------------------------------------------------------------- #
def test_an_overlapping_detection_is_matched_by_motion():
    c = AssociationCascade()
    out = c.associate([box(100, 100)], {7: box(105, 103)})
    assert out[0].stage == MOTION and out[0].track_id == 7


def test_one_track_cannot_take_two_detections():
    """Otherwise a single prediction absorbs a whole cluster and the second
    object silently inherits the first one's identity."""
    c = AssociationCascade()
    out = c.associate([box(100, 100), box(104, 104)], {7: box(102, 102)})
    assert sum(1 for a in out if a.track_id == 7) == 1


def test_a_distant_detection_is_not_matched_by_motion():
    c = AssociationCascade()
    out = c.associate([box(600, 400)], {7: box(100, 100)})
    assert out[0].stage != MOTION


# --------------------------------------------------------------------------- #
#  Stage 2 -- our own motion, measured
# --------------------------------------------------------------------------- #
def test_ego_motion_explains_a_box_that_motion_alone_could_not():
    """⭐ The vehicle moved, so the scene moved. BoT-SORT has to fit this from
    image content; we read it from a verified velocity sensor."""
    c = AssociationCascade()
    out = c.associate([box(200, 100)], {7: box(100, 100)}, ego_shift=(100, 0))
    assert out[0].stage == EGO and out[0].track_id == 7


def test_an_unknown_ego_shift_SKIPS_the_stage(  ):
    """⛔ A vehicle that cannot measure its own motion must not claim it did
    not move. NaN means unknown, and unknown is not zero."""
    c = AssociationCascade()
    out = c.associate([box(200, 100)], {7: box(100, 100)},
                      ego_shift=(float('nan'), float('nan')))
    assert out[0].stage == NEW


def test_a_zero_ego_shift_does_not_run_the_stage():
    """Nothing to compensate; running it would only be a second identical
    IoU test at a looser gate."""
    c = AssociationCascade()
    c.associate([box(600, 400)], {7: box(100, 100)}, ego_shift=(0.0, 0.0))
    assert c.stage_counts.get(EGO, 0) == 0


def test_ego_shift_is_nan_without_a_scale():
    """Pixels become metres only with an altitude. A constant here would put a
    plausible number where a measurement is missing."""
    dx, dy = ego_pixel_shift(0.5, 0.0, 0.0, 0.1, m_per_px=0.0, fx_px=500)
    assert math.isnan(dx) and math.isnan(dy)


def test_ego_shift_opposes_the_vehicle_motion():
    """The vehicle goes forward, so the scene slides backward."""
    dx, _ = ego_pixel_shift(1.0, 0.0, 0.0, 0.1, m_per_px=0.002, fx_px=500)
    assert dx < 0


def test_yaw_rate_contributes_to_the_shift():
    a, _ = ego_pixel_shift(0.0, 0.0, 0.0, 0.1, m_per_px=0.002, fx_px=500)
    b, _ = ego_pixel_shift(0.0, 0.0, 30.0, 0.1, m_per_px=0.002, fx_px=500)
    assert b != a


# --------------------------------------------------------------------------- #
#  Stage 3 -- appearance, and its cost bound
# --------------------------------------------------------------------------- #
def test_appearance_runs_only_on_what_the_cheap_stages_missed():
    """⭐ THE COST ARGUMENT. A Re-ID network at 15-25 ms/frame is unaffordable
    on every track and nearly free on the few motion could not explain."""
    calls = []

    def app(di, ctx):
        calls.append(di)
        return 42

    c = AssociationCascade(appearance=app)
    out = c.associate([box(100, 100), box(900, 700)], {7: box(102, 102)})
    assert calls == [1], f'appearance was asked about {calls}'
    assert out[0].stage == MOTION and out[1].stage == APPEARANCE


def test_the_appearance_budget_is_capped():
    """Even a detector firing on a cloud of silt must not uncap the cost."""
    calls = []

    def app(di, ctx):
        calls.append(di)
        return None

    c = AssociationCascade(appearance=app, max_appearance_calls=2)
    c.associate([box(i * 200, 0) for i in range(6)], {})
    assert len(calls) == 2


def test_appearance_refusing_yields_a_new_track_not_a_guess():
    c = AssociationCascade(appearance=lambda di, ctx: None)
    out = c.associate([box(900, 700)], {7: box(100, 100)})
    assert out[0].stage == NEW and out[0].track_id is None


def test_without_an_appearance_stage_the_cascade_still_works():
    c = AssociationCascade()
    out = c.associate([box(900, 700)], {7: box(100, 100)})
    assert out[0].stage == NEW


# --------------------------------------------------------------------------- #
#  The invariant that keeps this honest
# --------------------------------------------------------------------------- #
def test_nothing_is_fabricated():
    """A detection no stage explains becomes a NEW track. The ladder refuses
    to invent a position and this does not change that."""
    c = AssociationCascade()
    out = c.associate([box(900, 700)], {})
    assert out[0].track_id is None and out[0].stage == NEW


def test_every_detection_gets_exactly_one_outcome():
    c = AssociationCascade(appearance=lambda di, ctx: None)
    dets = [box(100, 100), box(300, 300), box(900, 700)]
    out = c.associate(dets, {7: box(102, 102), 8: box(305, 305)})
    assert len(out) == len(dets)
    assert [a.det_index for a in out] == [0, 1, 2]


def test_every_outcome_names_the_cue_that_decided_it():
    c = AssociationCascade()
    out = c.associate([box(100, 100)], {7: box(103, 103)})
    assert out[0].reason.strip() and out[0].stage in (MOTION, EGO, APPEARANCE)


def test_iou_and_shift_are_sane():
    assert iou(box(0, 0), box(0, 0)) == pytest.approx(1.0)
    assert iou(box(0, 0), box(500, 500)) == 0.0
    assert shift_box(box(0, 0), 10, 5) == (10, 5, 50, 45)
