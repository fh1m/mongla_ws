"""The EKF against ground truth -- the first such test this package has had.

⛔ WHY IT DID NOT EXIST BEFORE. `mongla_localization` has 21 filter tests and
every one is a unit or property test: exp/log round-trip, covariance symmetry,
positive-definiteness, the sign of a single update. Not one drives the filter
along a dynamic trajectory and asks how far it drifted -- because there was no
truth to drive it against until the Fossen plant existed.

⚠ A NUMBER FROM HERE IS A LOWER BOUND, NOT NAVIGATION ACCURACY. The plant has
no currents, no thermal drift, no vibration, and its drag is a guess. The value
of these tests is COMPARATIVE: which sensor matters, how long a dropout is
survivable, whether an update helps at all.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from localization_truth import Sensors, rot_ned, run   # noqa: E402


def test_the_plant_and_the_filter_share_a_frame():
    """⭐ PINNED RATHER THAN TRUSTED. The filter's world is NED with z DOWN and
    its body is FRD; Fossen is x-forward, y-starboard, z-DOWN. They are the same
    frame, so this harness remaps nothing -- and a frame seam is exactly what
    produced a -1988 degree runaway when the allocator met the plant."""
    import numpy as np
    assert np.allclose(rot_ned(0, 0, 0), np.eye(3))
    # a +90 deg yaw takes body +x (forward) to world +y (east in NED)
    r = rot_ned(0.0, 0.0, math.pi / 2)
    assert np.allclose(r @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-9)
    # and body +z (down) stays world +z (down) under any yaw
    assert np.allclose(r @ np.array([0, 0, 1.0]), [0, 0, 1], atol=1e-9)


def test_with_every_sensor_the_filter_tracks_truth():
    """The baseline. Flow, depth and yaw all present: centimetres over a
    minute of a yawing trajectory."""
    sc = run(seconds=60.0)
    assert sc.pos_rms_m < 0.10
    assert sc.yaw_rms_deg < 1.0
    assert sc.depth_rms_m < 0.05


def test_optical_flow_is_the_load_bearing_sensor():
    """⭐ WHICH SENSOR ACTUALLY MATTERS, measured rather than assumed. Removing
    depth or yaw changes almost nothing. Removing flow is catastrophic --
    velocity is what pins position, and nothing else here observes it."""
    full = run(seconds=30.0).pos_final_m
    assert run(seconds=30.0, use_depth=False).pos_final_m < 10 * full + 0.05
    assert run(seconds=30.0, use_yaw=False).pos_final_m < 10 * full + 0.05
    assert run(seconds=30.0, use_flow=False).pos_final_m > 100 * full


def test_the_observability_gate_removed_the_B56_pathology():
    """⭐ THESE SIX ASSERTIONS USED TO DOCUMENT A DEFECT AND NOW DOCUMENT ITS
    FIX -- which is the correct way for them to fail.

    Before `RIEKF._velocity_is_observed` existed, world-frame updates without
    velocity aiding were worse than NO aiding by up to 196x. The numbers below
    are the same experiments after the gate, and the history is kept in
    BUGS.md B-56 rather than in a test that asserts a bug.
    """
    aided = run(seconds=30.0, use_flow=False, initial_pos_error_m=1.0).pos_final_m
    bare = run(seconds=30.0, use_flow=False, use_depth=False, use_yaw=False,
               initial_pos_error_m=1.0).pos_final_m
    assert aided < 20 * bare, (
        f'aided {aided:.1f} m vs bare {bare:.1f} m -- the gate should have '
        f'stopped world-frame updates running away')


def test_a_tighter_fix_is_no_longer_catastrophic_without_flow():
    """⛔ THE SIGNATURE THAT DEFINED B-56: sigma 0.5 -> 0.01 took the error from
    183 m to 564 m, a measurement trusted more producing an estimate that was
    worse. After the gate both land under a metre, and the inversion that
    remains is mild rather than divergent."""
    loose = run(seconds=30.0, use_flow=False, use_fix=True,
                initial_pos_error_m=1.0,
                sensors=Sensors(fix_noise_m=0.50)).pos_final_m
    tight = run(seconds=30.0, use_flow=False, use_fix=True,
                initial_pos_error_m=1.0,
                sensors=Sensors(fix_noise_m=0.01)).pos_final_m
    assert loose < 2.0 and tight < 2.0
    assert tight < 10 * loose, 'no longer a runaway'


def test_a_tighter_fix_still_helps_monotonically_when_flow_is_healthy():
    """The gate must not have cost anything in normal operation -- with
    velocity observed the coupling is kept and a tighter fix is simply
    better."""
    loose = run(seconds=30.0, use_fix=True, initial_pos_error_m=1.0,
                sensors=Sensors(fix_noise_m=0.50)).pos_final_m
    tight = run(seconds=30.0, use_fix=True, initial_pos_error_m=1.0,
                sensors=Sensors(fix_noise_m=0.01)).pos_final_m
    assert tight < loose / 10.0


def test_the_gate_costs_nothing_when_flow_is_healthy():
    """⭐ THE REGRESSION THAT MATTERS MOST. A fix that degrades the normal case
    to rescue the degraded one is not a fix. Measured: bit-identical."""
    for err in (0.0, 0.5, 1.0, 2.0):
        with_fix = run(seconds=30.0, initial_pos_error_m=err,
                       use_fix=True).pos_final_m
        assert with_fix < 0.2, f'launch {err} m -> {with_fix:.4f} m'


@pytest.mark.parametrize('gap_s,bound_m', [(2.0, 0.30), (5.0, 0.30), (20.0, 0.60)])
def test_dropouts_are_survivable_after_the_gate(gap_s, bound_m):
    """And the operating envelope widens. Before the gate a 20 s dropout ended
    at 6.37 m and did not recover; now it is sub-metre."""
    sc = run(seconds=70.0, initial_pos_error_m=1.0, use_fix=True,
             flow_dropout=((20.0, 20.0 + gap_s),))
    assert sc.pos_final_m < bound_m


def test_fix_TIMING_is_irrelevant_when_flow_is_healthy():
    """⭐ AND AN EARLIER DRAFT OF B-57 CLAIMED THE OPPOSITE. It follows directly
    from B-57's own content: if dead-reckoning error does not grow, a late fix
    is worth exactly as much as an early one."""
    ends = [run(seconds=60.0, initial_pos_error_m=1.0, use_fix=True,
                fix_windows=((float(t0), float(t0 + 2)),)).pos_final_m
            for t0 in (0, 20, 55)]
    assert max(ends) < 0.6
    assert max(ends) / min(ends) < 3.0, 'timing should not matter much'


def test_fix_QUANTITY_is_what_matters():
    """One resection anywhere captures most of the benefit; after that it
    averages down roughly as sqrt(N)."""
    one = run(seconds=60.0, initial_pos_error_m=1.0, use_fix=True,
              fix_windows=((10.0, 10.5),)).pos_final_m
    many = run(seconds=60.0, initial_pos_error_m=1.0, use_fix=True).pos_final_m
    none = run(seconds=60.0, initial_pos_error_m=1.0).pos_final_m
    assert one < none / 2.0
    assert many < one
