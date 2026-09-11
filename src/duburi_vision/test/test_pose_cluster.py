"""The fuser must refuse the plausible wrong answer, not just produce a number.

The decisive case is the FLIP: a planar target's two PnP branches sit mirrored
about the normal, so a stream of per-frame answers is bimodal at +yaw and -yaw
and its MEAN is near zero -- a confident, stable, face-on reading of a board
the vehicle is badly off. Every test here is built from a constructed
distribution, so the decision rule is tested rather than whatever a pool
happened to show.
"""
import math

import pytest

from duburi_vision.pose_cluster import (
    Fused, PoseCluster, PoseSample, _circular_median, _wrap180,
)


def _feed(c, yaws, *, t0=0.0, dt=0.1, **kw):
    for i, y in enumerate(yaws):
        c.add(PoseSample(t=t0 + i * dt, yaw_deg=y, **kw))
    return c


# --- the flip, which is the whole point -------------------------------------


def test_a_clean_majority_wins_and_the_mean_would_have_been_wrong():
    # 7 frames at +28, 3 at the mirrored -28. The mean is +11.2 -- a number
    # that describes neither branch and points the hull between them.
    yaws = [28.0, 27.5, -28.0, 28.4, 27.9, -27.6, 28.1, -28.3, 28.2, 27.7]
    out = _feed(PoseCluster(), yaws).fuse()
    assert out.decided
    assert out.support == 7 and out.rival == 3
    assert abs(out.yaw_deg - 28.0) < 1.0
    assert abs(sum(yaws) / len(yaws)) < 12.0      # the mean IS near zero-ish
    assert abs(out.yaw_deg - sum(yaws) / len(yaws)) > 15.0


def test_an_even_fork_is_refused_rather_than_split():
    # 5 and 5. There is no answer here, and inventing one is the failure mode.
    c = _feed(PoseCluster(min_poses=6), [30.0] * 5 + [-30.0] * 5)
    out = c.fuse()
    assert not out.decided
    assert out.support == 5 and out.rival == 5
    assert 'needs 6' in out.reason


def test_support_below_the_floor_is_not_a_low_confidence_answer():
    out = _feed(PoseCluster(min_poses=4), [12.0, 12.2, 11.8]).fuse()
    assert not out.decided
    assert math.isnan(out.yaw_deg), 'a refused fuse must not hand back a number'


# --- gates ------------------------------------------------------------------


def test_a_coin_flip_frame_cannot_vote():
    # ambiguity -> 1 means the solver could not tell the branches apart. Such a
    # frame would otherwise manufacture a majority for whichever it happened to
    # report.
    c = PoseCluster(min_poses=3, max_ambiguity=0.9)
    _feed(c, [20.0, 20.1, 20.2])
    for y in (-20.0, -20.1, -20.3, -20.2):
        c.add(PoseSample(t=1.0, yaw_deg=y, ambiguity=0.99))
    out = c.fuse()
    assert out.decided and out.support == 3
    assert abs(out.yaw_deg - 20.1) < 0.5


def test_a_badly_reprojecting_frame_is_dropped():
    c = PoseCluster(min_poses=3, max_reproj_px=10.0)
    _feed(c, [15.0, 15.1, 15.2], reproj_px=3.0)
    for y in (-15.0, -15.2, -15.1, -15.3):
        c.add(PoseSample(t=1.0, yaw_deg=y, reproj_px=40.0))
    assert c.fuse().support == 3


def test_everything_gated_out_says_so():
    c = PoseCluster()
    for y in (10.0, 11.0, 12.0, 13.0):
        c.add(PoseSample(t=0.0, yaw_deg=y, ambiguity=0.99))
    out = c.fuse()
    assert not out.decided and 'gates' in out.reason


# --- the window -------------------------------------------------------------


def test_stale_poses_leave_the_window():
    c = PoseCluster(window_s=1.0, min_poses=3)
    _feed(c, [40.0] * 5, t0=0.0, dt=0.1)     # old, and a clear majority
    _feed(c, [-40.0] * 3, t0=10.0, dt=0.1)   # recent
    out = c.fuse(now=10.3)
    assert out.decided and out.support == 3
    assert out.yaw_deg < 0, 'the fuse must describe where the vehicle is NOW'


def test_an_empty_window_is_refused():
    out = PoseCluster().fuse()
    assert not out.decided and 'no poses' in out.reason


# --- the wrap, which splits a tight group into two halves 358 deg apart ------


def test_a_group_straddling_180_stays_one_group():
    out = _feed(PoseCluster(min_poses=4),
                [179.0, -179.5, 178.6, -179.9, 179.4]).fuse()
    assert out.decided and out.support == 5
    assert abs(_wrap180(out.yaw_deg - 179.0)) < 2.0


def test_circular_median_does_not_land_opposite_its_inputs():
    m = _circular_median([179.0, -179.0])
    assert abs(_wrap180(m - 180.0)) < 1.0, f'landed at {m}, should be near 180'


# --- range ------------------------------------------------------------------


def test_range_comes_from_the_winning_cluster_only():
    c = PoseCluster(min_poses=3)
    for y in (25.0, 25.1, 25.2):
        c.add(PoseSample(t=0.0, yaw_deg=y, range_m=2.0))
    for y in (-25.0, -25.1):
        c.add(PoseSample(t=0.0, yaw_deg=y, range_m=9.0))   # the losing branch
    out = c.fuse()
    assert out.decided and abs(out.range_m - 2.0) < 1e-6


def test_spread_reports_the_width_of_the_winning_cluster():
    out = _feed(PoseCluster(min_poses=4), [10.0, 12.0, 8.0, 10.0, 11.0]).fuse()
    assert out.decided
    assert 0.0 <= out.spread_deg <= 3.0
