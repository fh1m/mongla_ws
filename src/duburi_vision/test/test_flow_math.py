"""Pure-math tests for the downward optical-flow distance core (no ROS/cv2)."""

import math

import numpy as np
import pytest

from duburi_vision.distance.flow_math import (
    height_above_floor, rotation_flow_px, axis_unit, project, robust_flow,
    interp_rate, DistanceAccumulator,
)


# ── height sign (the formula-bug guard) ──────────────────────────────────────
def test_height_uses_pool_plus_negative_depth():
    # 0.5 m deep (depth_m=-0.5) in a 4 m pool -> 3.5 m above the floor.
    assert height_above_floor(4.0, -0.5) == pytest.approx(3.5)


def test_height_none_when_nonphysical():
    # depth below the floor / bad pool_depth -> None (caller holds last).
    assert height_above_floor(1.0, -2.0) is None
    assert height_above_floor(0.0, 0.0) is None


# ── rotation compensation ────────────────────────────────────────────────────
def test_rotation_flow_scales_with_f_and_rate_and_dt():
    dx, dy = rotation_flow_px(500.0, pitch_rate=0.2, roll_rate=0.1, dt=0.02)
    assert dy == pytest.approx(500.0 * 0.2 * 0.02)   # image-y <- pitch
    assert dx == pytest.approx(500.0 * 0.1 * 0.02)   # image-x <- roll


# ── axis projection (one axis only) ──────────────────────────────────────────
def test_axis_axial_is_forward_image_y():
    ax = axis_unit(0.0, lateral=False)         # yaw 0 -> +y image
    assert ax[0] == pytest.approx(0.0, abs=1e-9)
    assert ax[1] == pytest.approx(1.0)


def test_axis_lateral_is_perpendicular():
    ax = axis_unit(0.0, lateral=True)          # yaw 0 +90 -> +x image
    assert ax[0] == pytest.approx(1.0)
    assert ax[1] == pytest.approx(0.0, abs=1e-9)


def test_project_discards_cross_axis_component():
    # a flow with both components, projected onto +y, keeps only the y part.
    assert project((3.0, 4.0), axis_unit(0.0, lateral=False)) == pytest.approx(4.0)


# ── robust flow (outlier rejection) ──────────────────────────────────────────
def test_robust_flow_rejects_outlier_tracks():
    # 8 consistent tracks moving (+2,+1); 2 glare outliers moving wildly.
    p0 = np.array([[i, i] for i in range(10)], dtype=np.float64)
    disp = np.array([[2.0, 1.0]] * 8 + [[40.0, -30.0], [-25.0, 33.0]])
    p1 = p0 + disp
    status = np.ones(10)
    dx, dy = robust_flow(p0, p1, status, min_tracks=6)
    assert dx == pytest.approx(2.0, abs=0.3)
    assert dy == pytest.approx(1.0, abs=0.3)


def test_robust_flow_none_when_too_few_tracks():
    p0 = np.array([[0, 0], [1, 1]], dtype=np.float64)
    p1 = p0 + 1.0
    assert robust_flow(p0, p1, np.ones(2), min_tracks=6) is None


# ── rate interpolation (timestamp align) ─────────────────────────────────────
def test_interp_rate_linear_midpoint():
    buf = [(0.0, 0.0, 0.0), (1.0, 1.0, 2.0)]
    p, r = interp_rate(buf, 0.5)
    assert p == pytest.approx(0.5)
    assert r == pytest.approx(1.0)


def test_interp_rate_clamps_and_handles_empty():
    buf = [(1.0, 5.0, 6.0)]
    assert interp_rate(buf, 0.0) == (5.0, 6.0)   # before start -> first
    assert interp_rate(buf, 9.0) == (5.0, 6.0)   # after end -> last
    assert interp_rate([], 0.0) is None


# ── accumulator (start/reset/stop + metric fold) ─────────────────────────────
def test_accumulator_integrates_axial_metres():
    acc = DistanceAccumulator()
    acc.start(axis_yaw_rad=0.0, lateral=False)   # project onto +y image
    # 10 frames, each 5 px of forward (y) flow, height 2 m, f_px 500.
    # per frame: 5 * 2 / 500 = 0.02 m -> total 0.2 m.
    for _ in range(10):
        acc.add((0.0, 5.0), height_m=2.0, f_px=500.0)
    assert acc.stop() == pytest.approx(0.2)


def test_accumulator_ignores_cross_axis_flow():
    acc = DistanceAccumulator()
    acc.start(axis_yaw_rad=0.0, lateral=False)   # axial = +y
    for _ in range(10):
        acc.add((5.0, 0.0), height_m=2.0, f_px=500.0)   # pure x (cross-axis)
    assert acc.stop() == pytest.approx(0.0)


def test_accumulator_inactive_is_noop_and_reset():
    acc = DistanceAccumulator()
    acc.add((0.0, 5.0), 2.0, 500.0)              # not started -> no-op
    assert acc.distance_m == 0.0
    acc.start(0.0, False)
    acc.add((0.0, 5.0), 2.0, 500.0)
    assert acc.distance_m > 0.0
    acc.start(0.0, False)                        # re-start zeroes it
    assert acc.distance_m == 0.0
