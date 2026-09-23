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
