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


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ The landmark position fix -- is it a cure, or a second B-56?
# ═══════════════════════════════════════════════════════════════════════════ #

def test_a_harness_that_starts_the_filter_at_truth_is_cheating():
    """⛔ THE MISTAKE THIS FILE MADE FIRST, kept as a test so it is not made
    again. The filter is built believing +-1 m of position uncertainty
    (`P0_position`). Starting it at the true origin hands it an accuracy it has
    no way to know it has -- and then ANY absolute-position measurement looks
    harmful, because it is being scored against an error that was free.

    The reading that produced was "a landmark fix makes things 12x worse",
    which was an artefact. With the error the filter's own covariance claims,
    the fix helps by 5-20x."""
    cheat = run(seconds=30.0, initial_pos_error_m=0.0, use_fix=True).pos_final_m
    honest = run(seconds=30.0, initial_pos_error_m=1.0, use_fix=True).pos_final_m
    no_fix = run(seconds=30.0, initial_pos_error_m=1.0).pos_final_m
    assert cheat > run(seconds=30.0, initial_pos_error_m=0.0).pos_final_m
    assert honest < no_fix / 5.0, 'honestly initialised, the fix must help'


@pytest.mark.parametrize('start_err', [0.5, 1.0, 2.0])
def test_flow_aided_dead_reckoning_HOLDS_error_rather_than_reducing_it(start_err):
    """⭐ THE STRUCTURAL FACT, and it is more useful than a defect would have
    been. Nothing in the flow/depth/yaw set observes ABSOLUTE horizontal
    position, so the filter carries whatever error it launched with, almost
    exactly, for the whole run:

        start 0.5 m -> end 0.4949 m
        start 1.0 m -> end 0.9929 m
        start 2.0 m -> end 1.9907 m

    Dead reckoning that is excellent still cannot tell you where you are."""
    end = run(seconds=30.0, initial_pos_error_m=start_err).pos_final_m
    assert end == pytest.approx(start_err, rel=0.05)


def test_the_landmark_fix_is_the_only_thing_that_removes_launch_error():
    """⭐ AND IT CONVERGES ANY START TO THE SAME PLACE -- about 0.10 m with a
    0.5 m fix at 2 Hz -- regardless of how wrong the launch position was. So a
    fix EARLY is worth far more than a fix often: until the first one arrives,
    the whole run is offset."""
    ends = [run(seconds=30.0, initial_pos_error_m=e, use_fix=True).pos_final_m
            for e in (0.5, 1.0, 2.0)]
    assert all(e < 0.2 for e in ends)
    assert max(ends) / min(ends) < 1.5, 'it should converge to the same place'


def test_the_fix_rescues_the_B56_regime():
    """The landmark fix is the CURE for B-56, not another instance of it. With
    no flow, world-frame aiding runs away -- and a position fix pins `p`, which
    is precisely what the `-skew(p)` coupling needed."""
    broken = run(seconds=30.0, use_flow=False).pos_final_m
    fixed = run(seconds=30.0, use_flow=False, use_fix=True).pos_final_m
    assert fixed < broken / 100.0


def test_B56_survives_honest_initialisation_and_is_worse():
    """⛔ THE DEFECT IS NOT AN ARTEFACT OF THE CHEATING START. Re-measured with
    the error the filter's own covariance claims, world-frame aiding without
    velocity aiding is worse than NO aiding by up to 196x -- against the 27x
    first recorded. B-56's severity is revised upward, not withdrawn."""
    for err in (0.5, 1.0):
        aided = run(seconds=30.0, use_flow=False,
                    initial_pos_error_m=err).pos_final_m
        bare = run(seconds=30.0, use_flow=False, use_depth=False,
                   use_yaw=False, initial_pos_error_m=err).pos_final_m
        assert aided > 50 * bare, f'{err} m: aided {aided:.1f} bare {bare:.1f}'


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ Fix timing and quantity -- the mission-design numbers
# ═══════════════════════════════════════════════════════════════════════════ #

def test_fix_TIMING_is_irrelevant_when_flow_is_healthy():
    """⭐ AND AN EARLIER DRAFT OF B-57 CLAIMED THE OPPOSITE. It follows directly
    from B-57's own content: if dead-reckoning error does not grow, a late fix
    is worth exactly as much as an early one. Measured across a 60 s run, a 2 s
    resection window anywhere lands in the same place."""
    ends = [run(seconds=60.0, initial_pos_error_m=1.0, use_fix=True,
                fix_windows=((float(t0), float(t0 + 2)),)).pos_final_m
            for t0 in (0, 20, 55)]
    assert max(ends) < 0.5
    assert max(ends) / min(ends) < 2.0, 'timing should not matter'


def test_fix_QUANTITY_is_what_matters():
    """One resection anywhere captures most of the benefit (1.0 m -> 0.38 m);
    after that it averages down roughly as sqrt(N)."""
    one = run(seconds=60.0, initial_pos_error_m=1.0, use_fix=True,
              fix_windows=((10.0, 10.5),)).pos_final_m
    many = run(seconds=60.0, initial_pos_error_m=1.0, use_fix=True).pos_final_m
    none = run(seconds=60.0, initial_pos_error_m=1.0).pos_final_m
    assert one < none / 2.0, 'a single fix must remove most of the launch error'
    assert many < one / 4.0, 'and more fixes must keep helping'


def test_a_TIGHTER_fix_helps_with_flow_and_HURTS_without_it():
    """⛔⛔ THE DEFINITIVE B-56 SIGNATURE, and the control that proves it is the
    filter rather than this harness -- the two columns differ only in whether
    the velocity update is applied.

        sigma 0.50 -> 0.01     with flow:  0.1030 -> 0.0038   (27x BETTER)
                               no flow:     183.3 -> 564.5    (3x WORSE)

    A measurement you trust MORE producing an estimate that is WORSE is not a
    modelling subtlety. `H[:,0:3] = -skew(X.p)` injects an attitude correction
    scaled by the Kalman gain, so high gain is the accelerant."""
    tight_ok = run(seconds=30.0, use_fix=True, initial_pos_error_m=1.0,
                   sensors=Sensors(fix_noise_m=0.01)).pos_final_m
    loose_ok = run(seconds=30.0, use_fix=True, initial_pos_error_m=1.0,
                   sensors=Sensors(fix_noise_m=0.50)).pos_final_m
    assert tight_ok < loose_ok / 10.0, 'with flow, tighter must be better'

    tight_bad = run(seconds=30.0, use_flow=False, use_fix=True,
                    initial_pos_error_m=1.0,
                    sensors=Sensors(fix_noise_m=0.01)).pos_final_m
    loose_bad = run(seconds=30.0, use_flow=False, use_fix=True,
                    initial_pos_error_m=1.0,
                    sensors=Sensors(fix_noise_m=0.50)).pos_final_m
    assert tight_bad > 2.0 * loose_bad, 'without flow, tighter must be WORSE'


def test_the_508x_rescue_was_an_artefact_of_a_perfect_start():
    """⚠ RETRACTED CLAIM, kept as a test. B-56 was briefly recorded as cured by
    landmark fixes (99.3 m -> 0.195 m). That held only at EXACTLY zero launch
    error. Give the filter the error its own covariance claims and the rescue
    collapses."""
    perfect = (run(seconds=30.0, use_flow=False).pos_final_m
               / run(seconds=30.0, use_flow=False, use_fix=True).pos_final_m)
    realistic = (run(seconds=30.0, use_flow=False, initial_pos_error_m=0.25).pos_final_m
                 / run(seconds=30.0, use_flow=False, use_fix=True,
                       initial_pos_error_m=0.25).pos_final_m)
    assert perfect > 100.0
    assert realistic < 3.0, 'the rescue must not survive an honest start'
