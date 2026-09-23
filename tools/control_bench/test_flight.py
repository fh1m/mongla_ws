"""The closed loop: firmware cascade + our allocator + the Fossen plant.

⛔ THE TEST THIS FILE EXISTS FOR IS THE FRAME SEAM. The allocator reads
`hull_geometry.yaml` and therefore works in the CAD frame; the plant is Fossen.
The first version of `flight.py` wired them together without converting, the
yaw loop had POSITIVE feedback, and the hull ran to -1988 degrees in six
seconds.

⚠ And a runaway was the LUCKY outcome. Had the loop gain been lower the same
sign error would have produced slow convergence that looked like a tuning
problem -- and someone would have spent a pool day retuning a sign.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from flight import Vehicle, body_to_cad, cad_to_body      # noqa: E402
from plant import Damping, Inertia                        # noqa: E402

BAND = Damping.from_cd(cd_axial=0.25, cd_transverse=1.00,
                       provenance='band midpoint, UNMEASURED -- tests only')


# ═══════════════════════════════════════════════════════════════════════════ #
#  The frame seam
# ═══════════════════════════════════════════════════════════════════════════ #

def test_only_heave_and_yaw_flip_between_the_frames():
    """body = (cad.y, cad.x, -cad.z). Swapping two axes and negating a third
    is a PROPER rotation, so moments follow forces -- and exactly the two
    z-referenced axes come back negated."""
    got = body_to_cad(surge=1.0, sway=1.0, heave=1.0,
                      roll=1.0, pitch=1.0, yaw=1.0)
    assert got['surge'] == 1.0 and got['sway'] == 1.0
    assert got['roll'] == 1.0 and got['pitch'] == 1.0
    assert got['heave'] == -1.0, 'body z is DOWN, cad z is up'
    assert got['yaw'] == -1.0, 'the sign that caused the -1988 degree runaway'


def test_the_conversion_is_its_own_inverse():
    """An involution, so there is no 'which way round' to get wrong."""
    orig = dict(surge=0.3, sway=-0.2, heave=0.7, roll=-0.1, pitch=0.4, yaw=-0.6)
    assert cad_to_body(**body_to_cad(**orig)) == pytest.approx(orig)


def test_yaw_feedback_is_NEGATIVE():
    """⭐ THE REGRESSION TEST. Drop the frame conversion and this is the one
    that screams: the hull must approach the commanded heading, not flee it."""
    v = Vehicle(damping=BAND)
    tr = v.fly(seconds=4.0, hold_yaw=math.radians(20.0))
    assert math.degrees(tr.yaw[-1]) == pytest.approx(20.0, abs=1.0)
    assert abs(math.degrees(tr.yaw[-1])) < 180.0, 'a runaway, not a controller'


# ═══════════════════════════════════════════════════════════════════════════ #
#  The loop behaves like a controller
# ═══════════════════════════════════════════════════════════════════════════ #

@pytest.mark.parametrize('step_deg', [5.0, 20.0, 45.0])
def test_a_yaw_step_converges(step_deg):
    v = Vehicle(damping=BAND)
    tr = v.fly(seconds=8.0, hold_yaw=math.radians(step_deg))
    assert math.degrees(tr.yaw[-1]) == pytest.approx(step_deg, abs=0.5)


def test_roll_is_never_commanded_because_it_cannot_be():
    """The allocator refuses roll rather than zeroing it, and the closed loop
    must carry that refusal rather than quietly dropping it."""
    v = Vehicle(damping=BAND)
    tr = v.fly(seconds=2.0, hold_yaw=0.0,
               stick=lambda t: (0.6, 0.0, 0.0))      # hard roll stick
    assert any('roll' in r for r in tr.refused), (
        'a roll demand on a hull with no roll authority must be REFUSED')


def test_the_hull_rights_itself_in_roll_without_a_thruster():
    """⭐ THE QUESTION THAT DECIDES WHETHER ROLL CAN STAY UNACTUATED. Knocked
    over and left alone, does BG bring it back? Nothing actuates roll, so the
    answer is entirely the restoring moment's."""
    v = Vehicle(damping=BAND)
    v.plant.eta[3] = math.radians(20.0)
    tr = v.fly(seconds=12.0, hold_yaw=0.0)
    assert abs(math.degrees(tr.roll[-1])) < 20.0, 'roll must decay, not grow'


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ What MOT_SPIN_MIN costs, and what it does NOT cost
# ═══════════════════════════════════════════════════════════════════════════ #

def _limit_cycle_pp_deg(quantise: bool, **kw) -> float:
    v = Vehicle(damping=BAND, **kw)
    tr = v.fly(seconds=10.0, hold_yaw=math.radians(10.0), quantise=quantise)
    tail = [math.degrees(y) for y, t in zip(tr.yaw, tr.t) if t > 8.0]
    return max(tail) - min(tail)


def test_the_spin_min_floor_causes_a_yaw_limit_cycle():
    """The actuator floor cannot be held at zero, so the hold hunts. Removing
    the quantiser removes the hunt, which is what identifies the cause."""
    assert _limit_cycle_pp_deg(True) > 20 * _limit_cycle_pp_deg(False)


def test_the_limit_cycle_is_a_TENTH_OF_A_DEGREE_and_that_matters():
    """⛔ A CORRECTION TO OUR OWN UPSTREAM ARGUMENT, and it belongs in a test so
    it is not quietly forgotten. Asks I and M lean on MOT_SPIN_MIN hurting
    precision alignment. Measured in closed loop, the steady-state cost is of
    order a TENTH OF A DEGREE peak-to-peak -- about 3.5 mm at 2 m range, which
    is far below the pixel tolerances the vision servo works to.

    So the floor's real cost is NOT steady-state pointing jitter. It is the
    16.22 % LURCH when a small correction finally crosses the gate. The asks
    should argue the lurch, and this test is the reason why."""
    pp = _limit_cycle_pp_deg(True)
    assert 0.001 < pp < 0.5, f'{pp} deg -- outside the order of magnitude seen'


def test_drag_does_not_change_the_limit_cycle():
    """⭐ THE UNCERTAINTY THAT DOES NOT MATTER HERE. Drag is the plant's
    largest unknown, and the limit cycle is a small oscillation about zero
    velocity where quadratic damping vanishes -- so this conclusion survives
    the whole plausible drag band unchanged. Worth pinning: it is the reason
    the result is quotable at all."""
    out = []
    for cd_a, cd_t in ((0.15, 0.80), (0.30, 1.20)):
        d = Damping.from_cd(cd_axial=cd_a, cd_transverse=cd_t,
                            provenance='sweep endpoint')
        v = Vehicle(damping=d)
        tr = v.fly(seconds=10.0, hold_yaw=math.radians(10.0))
        tail = [math.degrees(y) for y, t in zip(tr.yaw, tr.t) if t > 8.0]
        out.append(max(tail) - min(tail))
    assert out[0] == pytest.approx(out[1], rel=1e-6)


def test_the_limit_cycle_DOES_depend_on_unmeasured_thrust_and_mass():
    """⚠ THE UNCERTAINTY THAT DOES. Across the plausible thrust and mass band
    the amplitude moves by more than an order of magnitude, so the number is an
    ORDER OF MAGNITUDE and not a measurement. Recorded so nobody quotes three
    significant figures off this bench."""
    lo = _limit_cycle_pp_deg(True, max_thrust_n=40.0,
                             inertia=Inertia(mass_kg=8.0))
    hi = _limit_cycle_pp_deg(True, max_thrust_n=8.0,
                             inertia=Inertia(mass_kg=13.6))
    assert hi / max(lo, 1e-9) > 5.0


def test_each_run_starts_from_rest():
    """⛔ A BENCH THAT CARRIES STATE BETWEEN RUNS IS WORSE THAN NO BENCH,
    because its output is plausible. `fly` once reset the controller and not the
    plant, so a reused Vehicle began each run from the previous run's attitude
    -- and the artefact read as a feedforward effect."""
    v = Vehicle(damping=BAND)
    v.fly(seconds=3.0, hold_yaw=math.radians(90.0))
    assert abs(math.degrees(v.plant.attitude[2])) > 45.0, 'setup: it did slew'
    tr = v.fly(seconds=3.0, hold_yaw=math.radians(10.0))
    assert abs(math.degrees(tr.yaw[0])) < 1e-6, 'run two began mid-air'
    assert math.degrees(tr.yaw[-1]) == pytest.approx(10.0, abs=0.5)


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ Round 3a -- the feedforward gains, and the row the bench KILLED
# ═══════════════════════════════════════════════════════════════════════════ #

def test_the_feedforward_layer_is_actually_reached():
    """⛔ THE BUG THAT MADE THE FIRST 3a RUN MEANINGLESS. `feedforward::apply`
    returns the demands untouched unless `mode` is one of its `stabilized` set,
    and FlightMode::STABILIZE is 0 -- an earlier run passed 3, which is not a
    FlightMode at all. Every gain then measured identically, which reads as
    "the layer does nothing" rather than "it was never called"."""
    from control_bench import Board
    from flight import MODE_STABILIZE
    b = Board().from_board()
    b.set_feedforward(drag_yaw=0.01, drag_rll=0.0, drag_pit=0.0)
    _, _, yaw, _ = b.feedforward(yaw=0.0, gz=2.0, mode=MODE_STABILIZE)
    assert yaw == pytest.approx(0.01 * 2.0 * 2.0, rel=1e-4)
    _, _, off, _ = b.feedforward(yaw=0.0, gz=2.0, mode=19)      # MANUAL
    assert off == 0.0, 'MANUAL must be raw passthrough'


def test_a_drag_gain_far_above_ideal_is_destabilising():
    """The safety margin, measured. Around the ideal value the response
    improves; at 4x the loop over-cancels its own drag, and the result is
    WORSE than leaving the gain at zero."""
    v = Vehicle(damping=BAND)
    tgt = math.radians(90.0)
    good = v.fly(seconds=12.0, hold_yaw=tgt, drag_gain_scale=1.0)
    bad = v.fly(seconds=12.0, hold_yaw=tgt, drag_gain_scale=4.0)
    assert bad.overshoot(tgt) > 10 * good.overshoot(tgt)
    off = v.fly(seconds=12.0, hold_yaw=tgt, drag_gain_scale=None)
    assert bad.settle_time(tgt, math.radians(2)) > off.settle_time(tgt, math.radians(2))


def test_the_yaw_axis_is_RATE_limited_not_drag_limited():
    """⭐ THE RESULT THAT KILLS ROUND 3a's YAW ROW.

    A 90 deg slew peaks at 98.7 % of `PILOT_YAW_RATE`, so the axis spends the
    manoeuvre against its own rate CAP -- a parameter -- rather than against
    drag. Feedforward cancels drag, and there is no drag to cancel while the
    rate command is saturated."""
    v = Vehicle(damping=BAND)
    tr = v.fly(seconds=12.0, hold_yaw=math.radians(90.0))
    cap = math.radians(v.board.get_param('PILOT_YAW_RATE'))
    assert max(abs(g) for g in tr.gz) / cap > 0.95


def test_there_is_no_regime_where_drag_feedforward_matters():
    """⭐ AND THE REASON IT IS A CLEAN KILL RATHER THAN A SMALL EFFECT.

        max_yaw_rate / ATC_ANG_YAW_P = 2.793 / 6.0 = 0.4655 rad = 26.7 deg

    Above that heading error the cascade commands the rate cap and stays there,
    so drag feedforward has no room. Below it the rate is small, and drag is
    quadratic in rate, so there is nothing to cancel. The two regimes meet with
    no gap between them."""
    v = Vehicle(damping=BAND)
    cap = math.radians(v.board.get_param('PILOT_YAW_RATE'))
    crossover = cap / v.board.get_param('ATC_ANG_YAW_P')
    assert math.degrees(crossover) == pytest.approx(26.7, abs=0.5)

    small = v.fly(seconds=8.0, hold_yaw=crossover * 0.5)
    assert max(abs(g) for g in small.gz) < cap, 'below crossover: not capped'
    # and at that rate the drag torque is a rounding error next to the peak
    q = v.plant.damping.quad[5]
    w = max(abs(g) for g in small.gz)
    assert q * w * w < 0.05 * v.max_thrust_n


def test_the_hull_can_sustain_more_yaw_rate_than_the_cap_allows():
    """⭐ ROBUST ACROSS EVERY UNMEASURED TERM, which is what makes it sayable.
    Steady state is allocator-max torque against quadratic drag. At every
    corner of the plausible drag and thrust band the hull sustains MORE than
    PILOT_YAW_RATE = 160 deg/s -- worst case 1.10x, best 3.69x. The cap is a
    choice, not a limit."""
    worst = None
    for cd_rot in (0.4, 0.9):
        for thrust in (8.0, 40.0):
            d = Damping.from_cd(cd_axial=0.25, cd_transverse=1.0,
                                cd_rot=cd_rot, provenance='sweep corner')
            v = Vehicle(damping=d, max_thrust_n=thrust)
            a = v.alloc.allocate(yaw=10.0)          # ask past saturation
            mz = abs(sum(v.B[5][j] * a.thrusts[j] for j in range(5))) * thrust
            w = math.sqrt(mz / d.quad[5])
            worst = w if worst is None else min(worst, w)
    assert math.degrees(worst) > 160.0, (
        f'worst-corner sustainable yaw {math.degrees(worst):.0f} deg/s')


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ Round 6 / B-13 -- is 500 Hz justified?
# ═══════════════════════════════════════════════════════════════════════════ #

def _burst(t):
    """A 1.5 N.m yaw kick, 0.2 s wide -- a wake, or a wall push."""
    return (0, 0, 0, 0, 0, 1.5 if 2.0 <= t < 2.2 else 0.0)


def _peak_dev_deg(hz, tau=0.0):
    v = Vehicle(damping=BAND, thruster_tau_s=tau)
    tr = v.fly(seconds=8.0, hold_yaw=0.0, control_hz=hz, disturbance=_burst)
    return max(abs(math.degrees(y)) for y in tr.yaw)


def test_decimation_holds_the_controller_output_rather_than_stretching_dt():
    """⛔ THE TWO ARE DIFFERENT EXPERIMENTS. The plant always integrates at the
    full rate; only the controller runs less often, and its output is HELD
    between updates. Passing a bigger dt to the plant would measure the
    integrator, and would flatter the slow rates by deleting the zero-order
    hold whose lag is the entire effect under test."""
    v = Vehicle(damping=BAND)
    slow = v.fly(seconds=1.0, dt=0.002, control_hz=50.0, hold_yaw=0.0,
                 stick=lambda t: (0.0, 0.0, 0.3))
    assert len(slow.t) == 500, 'the plant must still tick at 2 ms'
    # the held command shows up as runs of identical torque
    runs = sum(1 for a, b in zip(slow.torque_yaw, slow.torque_yaw[1:]) if a == b)
    assert runs > 400, 'a 50 Hz controller on a 500 Hz plant must hold output'


def test_a_quiet_slew_does_not_care_about_loop_rate_at_all():
    """⭐ PART ONE OF THE B-13 ANSWER. Settle time on a 90 deg slew is flat
    from 500 Hz to 10 Hz, because the axis is rate-limited: it spends the
    manoeuvre against PILOT_YAW_RATE, and a cap does not care how often you
    recompute it."""
    v = Vehicle(damping=BAND)
    tgt = math.radians(90.0)
    times = [v.fly(seconds=14.0, hold_yaw=tgt, control_hz=hz)
             .settle_time(tgt, math.radians(2.0)) for hz in (500, 100, 50)]
    assert max(times) / min(times) < 1.10


def test_disturbance_rejection_is_where_loop_rate_shows_up():
    """⭐ PART TWO, and the number the field does not have. Measured against a
    torque burst, with no actuator lag:

        500 -> 250 Hz   +2 %
        500 -> 100 Hz   +8 %
        500 ->  50 Hz  +21 %
        500 ->  25 Hz  +66 %

    So the knee is between 100 and 50 Hz, and 500 Hz buys about 8 % over
    100 Hz -- on THIS model, which has no sensor noise and no unmodelled
    dynamics, both of which punish slow loops harder."""
    base = _peak_dev_deg(500)
    assert _peak_dev_deg(250) / base < 1.05
    assert _peak_dev_deg(100) / base < 1.15
    assert _peak_dev_deg(50) / base < 1.35
    assert _peak_dev_deg(25) / base > 1.4, 'it must degrade eventually'


def test_actuator_lag_dominates_loop_rate_by_two_orders_of_magnitude():
    """⭐⭐ THE RESULT THAT REFRAMES THE WHOLE QUESTION, and it refutes our own
    upstream ask M section 3.

    Against the same burst, at 500 Hz throughout, thruster first-order lag:

        tau 0.00 s    0.394 deg
        tau 0.10 s    2.030 deg      5x worse
        tau 0.59 s   51.487 deg    131x worse

    while dropping the loop from 500 Hz to 50 Hz costs 21 % at worst. Actuator
    response time is worth one to two ORDERS OF MAGNITUDE more than control
    rate, and ask M section 3 explicitly said a faster thruster was not where
    the win is. It is."""
    no_lag = _peak_dev_deg(500, tau=0.0)
    lag = _peak_dev_deg(500, tau=0.59)
    rate_cost = _peak_dev_deg(50, tau=0.0) / no_lag
    lag_cost = lag / no_lag
    assert lag_cost > 50.0, f'lag cost only {lag_cost:.1f}x'
    assert rate_cost < 1.35
    assert lag_cost / rate_cost > 40.0


def test_halving_thruster_lag_beats_a_tenfold_loop_rate_increase():
    """The comparison stated as a decision rule, at a realistic operating
    point: from tau = 0.10 s, halving the lag is worth more than going from
    50 Hz to 500 Hz -- by about two to one."""
    from_rate = _peak_dev_deg(50, tau=0.10) - _peak_dev_deg(500, tau=0.10)
    from_lag = _peak_dev_deg(500, tau=0.10) - _peak_dev_deg(500, tau=0.05)
    assert from_lag > 1.5 * from_rate
