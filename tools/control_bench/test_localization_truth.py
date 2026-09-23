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


def test_world_frame_updates_without_velocity_aiding_are_WORSE_THAN_NOTHING():
    """⛔⛔ THE DEFECT THIS HARNESS FOUND, AND IT IS COUNTER-INTUITIVE.

    With no flow, adding depth and yaw makes horizontal position about 27x
    WORSE than running on the IMU alone. A measurement cannot legitimately harm
    an orthogonal state by that much.

    The mechanism is in the filter's own source and its own docstring already
    warns of it: `update_depth` carries `H[0,0:3] = -skew(X.p)[2]`, an attitude
    coupling that grows LINEARLY with distance from the origin, and the method
    is documented as "an IMPERFECT measurement for a right-invariant filter".

    While flow pins velocity that coupling is harmless. Once it does not, a
    depth innovation injects attitude error; in a RIGHT-invariant filter an
    attitude correction rotates the whole believed trajectory; that moves
    position further from the origin; which makes the coupling larger still.

    ⚠ This is NOT a claim that the filter is broken in normal operation -- with
    flow present it tracks to centimetres. It is a claim about its ENVELOPE."""
    aided = run(seconds=30.0, use_flow=False).pos_final_m
    bare = run(seconds=30.0, use_flow=False, use_depth=False,
               use_yaw=False).pos_final_m
    assert aided > 10 * bare, f'aided {aided:.1f} m vs bare {bare:.1f} m'
    # and it corrupts ATTITUDE too, which is the tell that it is coupling
    assert (run(seconds=30.0, use_flow=False, use_yaw=False).yaw_rms_deg
            > 3 * run(seconds=30.0, use_flow=False, use_depth=False,
                      use_yaw=False).yaw_rms_deg)


@pytest.mark.parametrize('gap_s,bound_m', [(2.0, 0.10), (5.0, 0.20), (10.0, 1.5)])
def test_the_filter_recovers_from_a_realistic_flow_dropout(gap_s, bound_m):
    """⭐ THE OPERATING LIMIT, and the number nobody had. The bottom camera
    loses the floor in turbid or deep water; how long can it be gone?

        2 s   0.03 m final   recovers
        5 s   0.05 m         recovers
       10 s   0.47 m         recovers, degraded
       20 s   6.37 m         DOES NOT recover
    """
    sc = run(seconds=70.0, flow_dropout=((20.0, 20.0 + gap_s),))
    assert sc.pos_final_m < bound_m


def test_a_twenty_second_dropout_does_not_recover():
    """The other side of the limit, stated so the envelope has both edges."""
    assert run(seconds=70.0, flow_dropout=((20.0, 40.0),)).pos_final_m > 2.0


def test_gating_world_updates_helps_LONG_dropouts_and_costs_short_ones():
    """⚠ THE MITIGATION IS A TRADE, NOT A FIX, and it is recorded as one.

    Withholding depth and yaw while flow is missing breaks the amplification
    loop above -- 1.3x to 1.6x better on dropouts of 10 s and longer. But on a
    5 s gap it is about 2x WORSE, because over that span the world-frame
    updates are still net useful.

    So "always gate" is wrong and "never gate" is wrong. Anyone implementing
    this needs the dropout duration, which means it belongs with whatever
    already tracks flow health, not inside the filter."""
    long_as_is = run(seconds=70.0, flow_dropout=((20.0, 40.0),)).pos_final_m
    long_gated = run(seconds=70.0, flow_dropout=((20.0, 40.0),),
                     gate_world_updates=True).pos_final_m
    assert long_gated < long_as_is

    short_as_is = run(seconds=70.0, flow_dropout=((20.0, 25.0),)).pos_final_m
    short_gated = run(seconds=70.0, flow_dropout=((20.0, 25.0),),
                      gate_world_updates=True).pos_final_m
    assert short_gated > short_as_is, 'the trade must be visible, not assumed'


def test_a_gyro_bias_is_absorbed_rather_than_integrated():
    """The filter estimates gyro bias; a constant offset should be learned and
    removed, not integrated into a growing heading error."""
    clean = run(seconds=60.0)
    biased = run(seconds=60.0,
                 sensors=Sensors(gyro_bias=(0.0, 0.0, math.radians(1.0))))
    assert biased.yaw_rms_deg < clean.yaw_rms_deg + 2.0
