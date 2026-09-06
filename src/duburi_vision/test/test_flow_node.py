"""flow_node -- the decisions, driven through the real node.

Not the maths (test_flow_math / test_flow_velocity cover that) and not the ROS
plumbing. These assert the things that make the difference between a velocity
sensor and a confident liar: that it REFUSES when a scale term is unknown, that
a refusal publishes quality 0 rather than 0 m/s, and that the medium selector
picks the focal length it says it does.

Driven through a constructed node with parameter overrides rather than a
reimplementation of its logic -- round 33's Kalman revival bug survived ten
green tests because the one test that could have caught it reimplemented the
node's loop and tested a copy without the bug.
"""
import math

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv2')
pytest.importorskip('cv_bridge')

from rclpy.parameter import Parameter                      # noqa: E402

from duburi_vision.distance.flow_node import (             # noqa: E402
    _F_AIR_PX, _F_WATER_PX, _GYRO_GAIN_DEFAULT, FlowVelocityNode,
)


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(**params):
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return FlowVelocityNode.__new__(FlowVelocityNode), overrides


def _make(**params):
    """A real node, with only the parameters under test overridden."""
    import rclpy.node

    overrides = [Parameter(k, value=v) for k, v in params.items()]
    # Build through the class so __init__ runs exactly as it does on the
    # vehicle; the node name is unique per test to avoid a name clash.
    orig_init = rclpy.node.Node.__init__

    def patched(self, name, **kw):
        kw.setdefault('parameter_overrides', [])
        kw['parameter_overrides'] = list(kw['parameter_overrides']) + overrides
        kw.setdefault('allow_undeclared_parameters', False)
        orig_init(self, name, **kw)

    rclpy.node.Node.__init__ = patched
    try:
        return FlowVelocityNode()
    finally:
        rclpy.node.Node.__init__ = orig_init


class TestScaleRefusal:
    """Both scale terms are CLEAN MULTIPLIERS on every number the node emits.
    A wrong one is invisible in every plot and integrated by everything
    downstream, so absence has to refuse rather than default."""

    def test_it_refuses_when_pool_depth_was_never_set(self):
        n = _make()
        try:
            ok, why = n._scale_ready()
            assert not ok
            assert 'pool_depth' in why
        finally:
            n.destroy_node()

    def test_a_pool_depth_of_zero_is_not_a_pool_depth(self):
        n = _make(pool_depth_m=0.0)
        try:
            assert not n._scale_ready()[0]
        finally:
            n.destroy_node()

    def test_it_accepts_an_explicit_pool_depth(self):
        n = _make(pool_depth_m=4.0)
        try:
            ok, why = n._scale_ready()
            assert ok, why
        finally:
            n.destroy_node()

    def test_an_unknown_medium_refuses(self):
        n = _make(pool_depth_m=4.0, medium='brine')
        try:
            ok, why = n._scale_ready()
            assert not ok and 'medium' in why
        finally:
            n.destroy_node()


class TestMediumSelectsFocalLength:
    """Round 25 measured BOTH: 63.8 deg air / 46.7 deg water, +-0.7, held-out
    validated against an external tape. At 640 px that is 514 and 741 -- a
    ratio of 1.44. Using the air number underwater is a 44 % multiplier on
    every velocity, which is the single most dangerous mistake available in
    this node."""

    def test_water_uses_the_water_focal_length(self):
        n = _make(pool_depth_m=4.0, medium='water')
        try:
            assert n._f_px == pytest.approx(_F_WATER_PX)
        finally:
            n.destroy_node()

    def test_air_uses_the_air_focal_length(self):
        n = _make(pool_depth_m=4.0, medium='air')
        try:
            assert n._f_px == pytest.approx(_F_AIR_PX)
        finally:
            n.destroy_node()

    def test_the_two_differ_by_the_measured_ratio_not_the_literature(self):
        """1.44 is ours. The flat-port literature says 25-33 %, and shipping
        1.33 would be 8 % wrong against our own held-out calibration."""
        assert _F_WATER_PX / _F_AIR_PX == pytest.approx(1.44, abs=0.01)


class TestGyroGainDefault:
    def test_the_default_gain_is_the_CANONICAL_one(self):
        """The bench rig measured -1.150/-1.095 and cut false velocity under
        pure rotation by 90 %. But rotation about the lens gives exactly
        f*omega*dt, so the 12 % excess needs an owner and has none -- a lever
        arm came out with the WRONG SIGN, an f of 577 contradicts a stronger
        calibration, and a dt bias is indistinguishable. measured-bars section
        12: "the axis mapping and the method carry over; THIS GAIN DOES NOT."

        A bad correction is worse than none: an unvalidated mapping already
        destroyed 35.7 cm of real travel this session."""
        assert _GYRO_GAIN_DEFAULT == -1.0

    def test_the_bench_gain_is_still_settable(self):
        n = _make(pool_depth_m=4.0, gyro_gain_x=-1.150, gyro_gain_y=-1.095)
        try:
            assert n._gx == pytest.approx(-1.150)
            assert n._gy == pytest.approx(-1.095)
        finally:
            n.destroy_node()


class TestQualityConvention:
    """MAVLink OPTICAL_FLOW_RAD: quality 0 means NO VALID FLOW. A refused
    interval must publish 0 and no velocity -- never 0 m/s, which an estimator
    cannot tell from a real standstill."""

    def test_a_refusal_sets_quality_zero(self):
        n = _make(pool_depth_m=4.0)
        try:
            n._refuse('because')
            assert n._last_quality == 0
            assert n._last_reason == 'because'
        finally:
            n.destroy_node()

    def test_a_real_but_poor_fix_never_reports_zero(self):
        """0 is reserved. A weak fix has to be distinguishable from no fix, or
        a consumer gating on `quality > 0` silently drops good-enough data."""
        from duburi_vision.distance.flow_velocity import FlowVelocity
        n = _make(pool_depth_m=4.0)
        try:
            worst = FlowVelocity(ok=True, vx=0.0, vy=0.0, rot_fraction=0.79,
                                 net_flow_px=0.6, height_m=1.0)
            q = n._quality(worst, n_used=6, disp=2.9)
            assert q >= 1
            assert q <= 255
        finally:
            n.destroy_node()

    def test_a_clean_fix_scores_higher_than_a_marginal_one(self):
        from duburi_vision.distance.flow_velocity import FlowVelocity
        n = _make(pool_depth_m=4.0)
        try:
            good = FlowVelocity(ok=True, vx=0.2, vy=0.0, rot_fraction=0.02,
                                net_flow_px=8.0, height_m=1.0)
            poor = FlowVelocity(ok=True, vx=0.2, vy=0.0, rot_fraction=0.75,
                                net_flow_px=0.6, height_m=1.0)
            assert (n._quality(good, n_used=60, disp=0.2)
                    > n._quality(poor, n_used=7, disp=2.5))
        finally:
            n.destroy_node()


class TestBucketedCorners:
    def test_corners_are_spread_across_the_frame(self):
        """A pool floor is a repetitive lattice, and corners clustered on one
        patch make a rotation and a translation look alike -- the two are
        separated by how the flow field VARIES across the frame."""
        import numpy as np

        n = _make(pool_depth_m=4.0, grid_buckets=4)
        try:
            rng = np.random.default_rng(0)
            gray = rng.integers(0, 255, (360, 640), dtype=np.uint8)
            pts = n._bucketed_corners(gray)
            assert pts is not None and len(pts) >= 16
            xy = pts.reshape(-1, 2)
            # Corners in at least half the quadrants, in both axes.
            assert (xy[:, 0] < 320).any() and (xy[:, 0] >= 320).any()
            assert (xy[:, 1] < 180).any() and (xy[:, 1] >= 180).any()
        finally:
            n.destroy_node()


class TestTimingCorrections:
    """The three deterministic terms, in the node. Each is LARGER than the
    0.218 ms residual the estimator removes, which is why they are corrected
    rather than absorbed."""

    def test_the_stamp_lands_at_the_interval_MIDPOINT(self):
        """Flow gives displacement over an interval, so the velocity is the
        AVERAGE across it. PX4 defines its flow delay 'to the middle of the
        optical flow integration interval'. Our adaptive baseline reaches
        0.75 s, so stamping at the end is wrong by up to 375 ms."""
        n = _make(pool_depth_m=4.0, exposure_us=0.0, estimate_time_offset=False)
        try:
            assert n._stamp_for(100.0, 0.5) == pytest.approx(99.75)
            assert n._stamp_for(100.0, 0.02) == pytest.approx(99.99)
        finally:
            n.destroy_node()

    def test_half_the_exposure_is_subtracted(self):
        """Mid-exposure is the convention. Ours is 15.7 ms on AUTO -- 7.85 ms,
        bigger on its own than Qin & Shen's 6 ms tolerance."""
        n = _make(pool_depth_m=4.0, exposure_us=157.0,
                  stamp_at_midpoint=False, estimate_time_offset=False)
        try:
            assert n._stamp_for(100.0, 0.02) == pytest.approx(100.0 - 0.00785)
        finally:
            n.destroy_node()

    def test_the_time_offset_shifts_the_stamp_the_RIGHT_WAY(self):
        """td > 0 means the image is LATE, so the corrected stamp is EARLIER.
        The opposite sign steers de-rotation backwards and doubles the
        residual while looking entirely plausible."""
        n = _make(pool_depth_m=4.0, exposure_us=0.0, stamp_at_midpoint=False,
                  estimate_time_offset=False, time_offset_s=0.030)
        try:
            assert n._stamp_for(100.0, 0.02) == pytest.approx(99.970)
        finally:
            n.destroy_node()

    def test_all_three_compose(self):
        n = _make(pool_depth_m=4.0, exposure_us=157.0, stamp_at_midpoint=True,
                  estimate_time_offset=False, time_offset_s=0.010)
        try:
            # midpoint 99.75, minus 7.85 ms exposure, minus 10 ms offset
            assert n._stamp_for(100.0, 0.5) == pytest.approx(
                99.75 - 0.00785 - 0.010)
        finally:
            n.destroy_node()

    def test_the_corrections_can_be_switched_OFF_for_an_A_B(self):
        """A timing change must be measurable against the old behaviour, or
        'it got better' is an assertion rather than a result."""
        n = _make(pool_depth_m=4.0, exposure_us=0.0, stamp_at_midpoint=False,
                  estimate_time_offset=False, time_offset_s=0.0)
        try:
            assert n._stamp_for(100.0, 0.5) == pytest.approx(100.0)
        finally:
            n.destroy_node()
