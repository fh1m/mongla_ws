"""Pure-math tests for the downward optical-flow distance core (no ROS/cv2)."""

import math
import pathlib

import numpy as np
import pytest

from mongla_vision.flow.flow_math import (
    height_above_floor, rotation_flow_px, axis_unit, project, robust_flow,
    interp_rate, integrate_rate, solve_planar_motion, DistanceAccumulator,
    HeightFromDivergence, Intrinsics,
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


# --------------------------------------------------------------------------- #
#  add_body_velocity -- metric velocity onto the latched axis
# --------------------------------------------------------------------------- #
class TestAddBodyVelocity:
    """The pixel round-trip this replaces got the axes BACKWARDS on the first
    attempt, and produced a plausible number rather than an error.
    `flow_velocity` returns `vx` from the image's *y* component, so the obvious
    tuple is reversed -- the same axis-swap class that cost four bench runs."""

    def test_on_heading_the_axial_axis_is_exactly_forward_speed(self):
        acc = DistanceAccumulator()
        acc.start(axis_yaw_rad=0.0, lateral=False)
        acc.add_body_velocity(vx=0.5, vy=0.0, yaw_rad=0.0, dt=2.0)
        assert acc.distance_m == pytest.approx(1.0)

    def test_the_latch_is_a_HEADING_not_a_frame(self):
        """Latched at 90 deg and driving forward at 90 deg is still pure axial
        travel -- only the error SINCE the latch matters."""
        acc = DistanceAccumulator()
        acc.start(axis_yaw_rad=math.radians(90.0), lateral=False)
        acc.add_body_velocity(vx=0.5, vy=0.0, yaw_rad=math.radians(90.0), dt=2.0)
        assert acc.distance_m == pytest.approx(1.0)

    def test_a_90_degree_heading_error_contributes_NOTHING_axially(self):
        """Forward motion perpendicular to the latched axis is not progress
        along it. If this ever reads 1.0, the projection has been dropped and
        the accumulator is measuring path length, not displacement."""
        acc = DistanceAccumulator()
        acc.start(axis_yaw_rad=0.0, lateral=False)
        acc.add_body_velocity(vx=0.5, vy=0.0, yaw_rad=math.radians(90.0), dt=2.0)
        assert acc.distance_m == pytest.approx(0.0, abs=1e-9)

    def test_lateral_latch_reads_body_right(self):
        acc = DistanceAccumulator()
        acc.start(axis_yaw_rad=0.0, lateral=True)
        acc.add_body_velocity(vx=0.0, vy=0.5, yaw_rad=0.0, dt=2.0)
        assert acc.distance_m == pytest.approx(1.0)

    def test_reversing_reduces_the_total(self):
        """Displacement, not distance travelled: out and back is zero."""
        acc = DistanceAccumulator()
        acc.start(axis_yaw_rad=0.0, lateral=False)
        acc.add_body_velocity(0.5, 0.0, 0.0, 2.0)
        acc.add_body_velocity(-0.5, 0.0, 0.0, 2.0)
        assert acc.distance_m == pytest.approx(0.0, abs=1e-9)

    def test_it_is_inert_until_started(self):
        """An accumulator that folds before start() silently books travel from
        whenever the node happened to boot."""
        acc = DistanceAccumulator()
        acc.add_body_velocity(10.0, 0.0, 0.0, 1.0)
        assert acc.distance_m == 0.0

    def test_a_nonpositive_dt_folds_nothing(self):
        acc = DistanceAccumulator()
        acc.start(0.0, False)
        acc.add_body_velocity(1.0, 0.0, 0.0, 0.0)
        acc.add_body_velocity(1.0, 0.0, 0.0, -1.0)
        assert acc.distance_m == 0.0


# --------------------------------------------------------------------------- #
#  solve_planar_motion -- one rigid motion, not N independent displacements
# --------------------------------------------------------------------------- #
class TestPlanarMotion:
    """MEASURED against a median on real texture, truth exact: with 1.5 deg of
    rotation in one interval the median reports 4.79 px of translation that did
    not happen, the rigid fit 0.055 px. At h=0.72 m that phantom is 6.7 mm of
    fictional travel PER INTERVAL."""

    W, H = 640, 360
    CX, CY = 320.0, 180.0

    def _pair(self, n=120, tx=0.0, ty=0.0, rot_deg=0.0, seed=0):
        import cv2
        rng = np.random.default_rng(seed)
        p0 = rng.uniform(40, min(self.W, self.H) - 40, (n, 2)).astype(np.float32)
        M = cv2.getRotationMatrix2D((self.CX, self.CY), rot_deg, 1.0)
        M[0, 2] += tx
        M[1, 2] += ty
        p1 = ((M[:, :2] @ p0.T).T + M[:, 2]).astype(np.float32)
        # Truth = where the OPTICAL AXIS went, which is what we want back.
        c = np.array([self.CX, self.CY, 1.0])
        moved = M @ c
        return p0, p1, float(moved[0] - self.CX), float(moved[1] - self.CY)

    def test_pure_translation_is_recovered(self):
        p0, p1, tdx, tdy = self._pair(tx=3.0, ty=7.0)
        pm = solve_planar_motion(p0, p1, None, 0.05, cx=self.CX, cy=self.CY)
        assert pm.ok, pm.reason
        assert pm.dx_px == pytest.approx(tdx, abs=0.05)
        assert pm.dy_px == pytest.approx(tdy, abs=0.05)

    def test_pure_rotation_yields_NO_translation(self):
        """The headline. A median turns rotation into phantom travel; a rigid
        fit reports the rotation as rotation and the translation as zero."""
        p0, p1, tdx, tdy = self._pair(rot_deg=1.5)
        pm = solve_planar_motion(p0, p1, None, 0.05, cx=self.CX, cy=self.CY)
        assert pm.ok, pm.reason
        assert math.hypot(pm.dx_px - tdx, pm.dy_px - tdy) < 0.1
        assert abs(pm.yaw_rate) > 0.1, 'rotation was not detected at all'

    def test_the_median_FAILS_the_same_case(self):
        """The control. Without it, the test above proves only that the fit
        works -- not that it was needed."""
        p0, p1, tdx, tdy = self._pair(rot_deg=1.5)
        st = np.ones(len(p0), dtype=np.uint8)
        med = robust_flow(p0.reshape(-1, 1, 2), p1.reshape(-1, 1, 2), st)
        assert med is not None
        assert math.hypot(med[0] - tdx, med[1] - tdy) > 1.0, (
            'the median coped, so this comparison proves nothing -- check the '
            'point distribution is not accidentally symmetric')

    def test_translation_survives_simultaneous_rotation(self):
        p0, p1, tdx, tdy = self._pair(tx=0.0, ty=6.0, rot_deg=3.0)
        pm = solve_planar_motion(p0, p1, None, 0.05, cx=self.CX, cy=self.CY)
        assert pm.ok, pm.reason
        assert math.hypot(pm.dx_px - tdx, pm.dy_px - tdy) < 0.2

    def test_the_translation_is_read_at_the_PRINCIPAL_POINT(self):
        """`estimateAffinePartial2D` returns tx/ty about the ORIGIN, so under
        rotation those carry a lever term of theta*|c| -- at 640x360 and 1 deg
        that is 3.3 px of fiction. Only the motion of the optical axis is the
        translation. This fails if anyone 'simplifies' it to M[:, 2]."""
        import cv2
        p0, p1, tdx, tdy = self._pair(rot_deg=2.0)
        pm = solve_planar_motion(p0, p1, None, 0.05, cx=self.CX, cy=self.CY)
        M, _ = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC)
        naive = math.hypot(M[0, 2], M[1, 2])
        assert naive > 3.0, 'the naive readout should be badly wrong here'
        assert math.hypot(pm.dx_px, pm.dy_px) < 0.2

    def test_it_rejects_a_moving_object_rather_than_averaging_it_in(self):
        """A third of the points belong to something else moving. A median
        with a MAD gate can be dragged; a global model cannot be satisfied by
        two motions at once, so RANSAC calls them outliers."""
        p0, p1, tdx, tdy = self._pair(ty=6.0, n=150, seed=3)
        p1 = p1.copy()
        p1[:50, 0] += 25.0          # a rogue cluster moving sideways
        pm = solve_planar_motion(p0, p1, None, 0.05, cx=self.CX, cy=self.CY)
        assert pm.ok, pm.reason
        assert pm.n_inliers <= 110, pm.n_inliers
        assert math.hypot(pm.dx_px - tdx, pm.dy_px - tdy) < 0.5

    def test_too_few_points_REFUSES_rather_than_fitting_noise(self):
        p0, p1, _, _ = self._pair(n=4, ty=5.0)
        pm = solve_planar_motion(p0, p1, None, 0.05, cx=self.CX, cy=self.CY)
        assert not pm.ok and 'need' in pm.reason

    def test_scale_change_is_reported(self):
        """Divergence of the field is range rate -- a second opinion on
        altitude, from a sensor a thruster wake cannot disturb."""
        import cv2
        rng = np.random.default_rng(1)
        p0 = rng.uniform(40, 320, (120, 2)).astype(np.float32)
        M = cv2.getRotationMatrix2D((self.CX, self.CY), 0.0, 1.02)
        p1 = ((M[:, :2] @ p0.T).T + M[:, 2]).astype(np.float32)
        pm = solve_planar_motion(p0, p1, None, 0.1, cx=self.CX, cy=self.CY)
        assert pm.ok
        assert pm.scale_rate == pytest.approx(0.02 / 0.1, rel=0.05)


class TestIntegrateRate:
    """De-rotation subtracts f*omega*dt, so over an adaptive baseline the MEAN
    rate is the quantity needed -- a midpoint sample assumes linearity that a
    half-swing does not have."""

    def test_a_constant_rate_is_its_own_mean(self):
        buf = [(t * 0.02, 0.4, -0.2) for t in range(50)]
        p, r = integrate_rate(buf, 0.1, 0.5)
        assert p == pytest.approx(0.4, abs=1e-6)
        assert r == pytest.approx(-0.2, abs=1e-6)

    def test_a_full_swing_has_a_mean_near_zero_where_the_midpoint_peaks(self):
        """The case that motivates it: a rate that swings +A then -A has a
        mean of ~0 and a midpoint sample at the PEAK. Over a 0.5 s baseline
        the midpoint would subtract a rotation that never net happened."""
        buf = [(t * 0.01, math.sin(2 * math.pi * t / 100.0), 0.0)
               for t in range(101)]
        mean_p, _ = integrate_rate(buf, 0.0, 1.0)
        mid_p, _ = interp_rate(buf, 0.5)
        assert abs(mean_p) < 0.05, mean_p
        assert abs(mid_p) < 0.05 or abs(mid_p) > 0.5

    def test_empty_buffer_is_None_not_zero(self):
        assert integrate_rate([], 0.0, 1.0) is None


# --------------------------------------------------------------------------- #
#  HeightFromDivergence -- height from the image, checked by the barometer
# --------------------------------------------------------------------------- #
class TestHeightFromDivergence:
    """Height is a CLEAN MULTIPLIER on every velocity this sensor produces,
    and today it comes from a pool depth someone typed in. The flow field's
    divergence plus the barometer's vertical speed measure it directly."""

    def test_it_recovers_a_known_height(self):
        h = HeightFromDivergence()
        true_h, vz = 2.5, 0.20
        for _ in range(10):
            h.add(scale_rate=vz / true_h, vz_ms=vz)
        assert h.height_m == pytest.approx(true_h, rel=1e-6)

    def test_it_says_NOTHING_in_level_flight(self):
        """The critical gate. With no depth change the divergence is zero and
        `vz / scale_rate` is 0/0 -- noise wearing a number. A height invented
        here would silently rescale every distance in the mission."""
        h = HeightFromDivergence()
        for _ in range(50):
            assert h.add(scale_rate=0.0005, vz_ms=0.0008) is None
        assert h.height_m is None

    def test_it_withholds_an_answer_until_it_has_evidence(self):
        h = HeightFromDivergence()
        for _ in range(4):
            h.add(scale_rate=0.08, vz_ms=0.2)
        assert h.height_m is None, 'answered on four samples'
        h.add(scale_rate=0.08, vz_ms=0.2)
        assert h.height_m is not None

    def test_a_single_wild_sample_does_not_move_the_answer(self):
        """Median, not mean: one bad interval on a passing fish or a glint
        should not rescale the mission."""
        h = HeightFromDivergence()
        for _ in range(20):
            h.add(scale_rate=0.1, vz_ms=0.25)      # 2.5 m
        h.add(scale_rate=0.01, vz_ms=0.25)          # 25 m, absurd
        assert h.height_m == pytest.approx(2.5, rel=0.05)

    def test_absurd_heights_are_refused_not_stored(self):
        h = HeightFromDivergence(max_h=12.0)
        assert h.add(scale_rate=0.001, vz_ms=0.5) is None   # 500 m
        assert h.n_samples == 0

    def test_it_flags_a_wrong_pool_depth(self):
        """The point of the whole thing: a typed pool depth that is 30 % out
        should be caught before it becomes a scale error in the mission."""
        h = HeightFromDivergence()
        for _ in range(10):
            h.add(scale_rate=0.2 / 2.0, vz_ms=0.2)          # truly 2.0 m
        assert h.disagreement(2.0) == pytest.approx(0.0, abs=1e-6)
        assert h.disagreement(2.6) == pytest.approx(0.3, abs=0.01)

    def test_disagreement_is_None_when_it_has_no_opinion(self):
        """Absence is not agreement."""
        h = HeightFromDivergence()
        assert h.disagreement(2.0) is None

    def test_the_sign_convention_holds_descending(self):
        """vz positive DOWNWARD, and descending grows the image. If either
        sign flips, the height comes out negative and is refused -- which is
        the safe direction, but the test pins the intent."""
        h = HeightFromDivergence()
        assert h.add(scale_rate=0.10, vz_ms=0.25) == pytest.approx(2.5)
        assert h.add(scale_rate=-0.10, vz_ms=0.25) is None


# --------------------------------------------------------------------------- #
#  Intrinsics -- fx != fy, the principal point is not the centre, lenses distort
# --------------------------------------------------------------------------- #
class TestIntrinsics:
    """Three assumptions the flow path made, all wrong, all axis-dependent --
    which is why they surfaced as a 3.08 % asymmetry between forward/back
    (ratio 1.0354) and lateral (1.0044) on real 30 cm slides. A height error
    cannot do that; height scales both axes identically."""

    CAL = (pathlib.Path(__file__).resolve().parents[1] / 'config' /
           'calibration' / 'pi_downward_1280x720.json')

    def _i(self, w=640, h=360):
        return Intrinsics.from_json(str(self.CAL), w, h)

    def test_the_calibration_scales_to_the_working_resolution(self):
        """Calibrated at 1280x720, run at 640x360. Applying it unscaled is a
        clean factor-of-two error that nothing would report."""
        full = self._i(1280, 720)
        half = self._i(640, 360)
        assert half.fx == pytest.approx(full.fx / 2, rel=1e-9)
        assert half.cy == pytest.approx(full.cy / 2, rel=1e-9)

    def test_the_pixels_are_not_square(self):
        """fy/fx = 1.0058, so the axis riding image-y reads 0.58 % high when
        one focal length is used for both."""
        i = self._i()
        assert i.fy != i.fx
        assert i.fy / i.fx == pytest.approx(1.0058, abs=0.001)

    def test_the_principal_point_is_NOT_the_frame_centre(self):
        """It is 11.4 px out in x. The rigid fit evaluates its translation AT
        this point, so under rotation the error is a lever arm about the wrong
        pivot."""
        i = self._i()
        assert abs(i.cx - 320.0) > 5.0, i.cx
        assert i.cx == pytest.approx(308.7, abs=1.0)

    def test_undistortion_leaves_the_centre_alone(self):
        """The sanity check that the model is being applied the right way
        round: distortion is zero at the principal point by construction."""
        i = self._i()
        p = np.array([[i.cx, i.cy]], dtype=np.float32)
        out = i.undistort_points(p)
        assert np.hypot(out[0][0] - i.cx, out[0][1] - i.cy) < 0.05

    def test_distortion_is_ASYMMETRIC_between_the_axes(self):
        """The mechanism behind the measured asymmetry. A 640x360 frame is
        1.8x wider than tall, so the horizontal axis samples a radial range
        the vertical one never reaches -- measured 3.37 px of correction at
        the x edge against 1.62 px at the y edge."""
        i = self._i()
        xe = np.array([[630.0, i.cy]], dtype=np.float32)
        ye = np.array([[i.cx, 355.0]], dtype=np.float32)
        ux, uy = i.undistort_points(xe), i.undistort_points(ye)
        shift_x = abs(ux[0][0] - 630.0)
        shift_y = abs(uy[0][1] - 355.0)
        assert shift_x > 1.5 * shift_y, (shift_x, shift_y)

    def test_a_missing_distortion_model_is_a_no_op_not_a_crash(self):
        i = Intrinsics(500.0, 500.0, 320.0, 180.0, dist=None)
        p = np.array([[100.0, 100.0]], dtype=np.float32)
        assert np.allclose(i.undistort_points(p), p)


class TestRefractiveRectifier:
    """A flat port is not a pinhole, and one focal length cannot hide it."""

    FX, FY, CX, CY, N = 513.94, 516.93, 308.7, 186.5, 1.333

    def _rec(self):
        from mongla_vision.flow.flow_math import RefractiveRectifier
        return RefractiveRectifier(self.FX, self.FY, self.CX, self.CY, n=self.N)

    def test_the_principal_point_is_a_FIXED_POINT(self):
        """Rectification must not move the centre. If it did, every downstream
        consumer of cx/cy -- the planar fit reads its translation AT the
        principal point -- would silently be reading it somewhere else."""
        r = self._rec()
        out = r.rectify(np.array([[self.CX, self.CY]], dtype=np.float64))
        assert out[0][0] == pytest.approx(self.CX, abs=1e-9)
        assert out[0][1] == pytest.approx(self.CY, abs=1e-9)

    def test_the_local_focal_length_GROWS_with_field_angle(self):
        """The defect itself. If this is flat, there is nothing to correct and
        the whole class is dead weight."""
        r = self._rec()
        f0, f200, f320 = (r.local_focal_px(x) for x in (0.0, 200.0, 320.0))
        assert f0 < f200 < f320                     # monotone
        assert f0 == pytest.approx(self.FX * self.N, rel=1e-6)   # paraxial
        # The measured spread that justifies shipping this: >10 % corner-centre
        f_corner = r.local_focal_px(math.hypot(320.0, 180.0))
        assert (f_corner / f0 - 1.0) > 0.10

    def test_it_INVERTS_the_port_exactly(self):
        """Forward-simulate the physics, rectify, and require a TRUE pinhole:
        a point on a plane at height h must land at f_ref * (R/h). This is the
        property the velocity conversion depends on, so assert THAT, not an
        intermediate angle."""
        r = self._rec()
        h = 0.72
        # Stay inside the frame. A water ray past the critical angle
        # asin(1/n) = 48.6 deg cannot refract into air at all -- that is
        # physics, not a bug, and this camera's half-FOV in water is 23.4 deg,
        # so the frame never reaches it. R/h = tan(23.4 deg) = 0.433 at the
        # edge; 0.9/0.72 = 1.25 was off the sensor entirely.
        for R in (0.05, 0.1, 0.2, 0.30):
            tw = math.atan2(R, h)
            ta = math.asin(math.sin(tw) * self.N)     # water -> air
            u = self.CX + self.FX * math.tan(ta)      # lands on the sensor
            got = r.rectify(np.array([[u, self.CY]], dtype=np.float64))[0][0]
            want = self.CX + r.f_ref * (R / h)   # a TRUE pinhole at f_ref
            assert got == pytest.approx(want, rel=1e-9)

    def test_a_wrong_index_shows_up_as_SCALE(self):
        """n is a physical constant, not a tuning knob -- but if someone sets
        it wrong the failure must be a clean scale error, not a shape change,
        because a scale error is what the pool tape can catch."""
        from mongla_vision.flow.flow_math import RefractiveRectifier
        a = self._rec()
        b = RefractiveRectifier(self.FX, self.FY, self.CX, self.CY, n=1.0)
        # n = 1 is "no water": the rectifier must reduce to the identity.
        pts = np.array([[100.0, 90.0], [500.0, 300.0]])
        assert np.allclose(b.rectify(pts), pts, atol=1e-9)
        assert not np.allclose(a.rectify(pts), pts, atol=1e-3)

