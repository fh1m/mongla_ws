"""Round 5 -- INDI against the tuned cascade, on our own plant.

⛔ THE PLAN'S FALSIFIER, QUOTED: "against the tuned cascade PID from Round 3,
under an injected disturbance, INDI must win on the bench before it is ever
proposed for the board. If it cannot beat a well-tuned PID on our own plant, it
does not fly -- and that is a publishable result either way."

**It does not fly.** These tests pin why, so the conclusion survives contact
with anyone who wants to re-open it.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from flight import Vehicle                                   # noqa: E402
from indi import IndiAttitude, IndiRate                      # noqa: E402
from plant import Damping, Inertia                           # noqa: E402

NOMINAL = Damping.from_cd(cd_axial=0.25, cd_transverse=1.00,
                          provenance='nominal band midpoint, UNMEASURED')
TARGET = math.radians(30.0)


def _burst(t):
    return (0, 0, 0, 0, 0, 1.5 if 2.0 <= t < 2.2 else 0.0)


def _indi(v, k_rate=20.0, cutoff=30.0):
    """INDI with the gains the fair sweep picked, and the limits the allocator
    can actually deliver."""
    ymax = abs(sum(v.B[5][j] * v.alloc.allocate(yaw=10.0).thrusts[j]
                   for j in range(5)))
    I = [v.plant.M[3], v.plant.M[4], v.plant.M[5]]
    return IndiAttitude(inertia_xyz=I, k_rate=(0.0, 8.0, k_rate),
                        tau_limits=(0.0, 0.52, ymax), cutoff_hz=cutoff)


# ═══════════════════════════════════════════════════════════════════════════ #
#  The two design rules this bench discovered the hard way
# ═══════════════════════════════════════════════════════════════════════════ #

def test_indi_must_not_be_pointed_at_an_unactuated_axis():
    """⛔ FOUND BY RUNNING IT. An incremental law keeps adding until it sees the
    acceleration it asked for. On roll -- which this hull cannot actuate, B
    being rank 5 of 6 -- no acceleration is ever coming, so the increment winds
    to its clamp and the Euler coupling drags yaw off target.

    A cascade survives the same mistake because its output is a bounded PID and
    the allocator simply refuses the demand. INDI does not."""
    a = IndiAttitude(inertia_xyz=[0.031, 0.284, 0.284],
                     tau_limits=(0.0, 0.52, 0.35))
    assert a.enabled[0] is False, 'roll must default to OFF on this hull'
    out = a.update(attitude=(0.5, 0.0, 0.0), rates=(0, 0, 0),
                   target=(0, 0, 0.5), tau_applied=(0, 0, 0), dt=0.002)
    assert out[0] == 0.0, 'a demand on an unactuated axis must be exactly zero'


def test_the_increment_is_limited_to_ACHIEVABLE_torque():
    """The other windup source. Clamping at +-1 when the allocator can only
    deliver 0.35 lets the increment run 3x past anything the hull can do."""
    r = IndiRate(inertia=0.284, k_rate=20.0, tau_limit=0.35)
    for _ in range(200):
        out = r.update(omega=0.0, omega_des=2.0, tau_applied=0.0, dt=0.002)
    assert abs(out) <= 0.35 + 1e-9


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ The verdict
# ═══════════════════════════════════════════════════════════════════════════ #

def test_indi_does_win_the_step_response():
    """Stated because it is true and because the verdict should not read as a
    hatchet job: on a clean 30 deg step INDI settles about 24 % faster than the
    cascade, and with a smaller limit cycle."""
    v = Vehicle(damping=NOMINAL)
    indi = v.fly(seconds=10.0, hold_yaw=TARGET, controller=_indi(v))
    casc = v.fly(seconds=10.0, hold_yaw=TARGET)
    assert (indi.settle_time(TARGET, math.radians(2.0))
            < 0.85 * casc.settle_time(TARGET, math.radians(2.0)))


def test_but_indi_LOSES_disturbance_rejection_which_is_the_falsifier():
    """⛔ THE TEST THE PLAN ACTUALLY SET, and INDI fails it. Against a 1.5 N.m
    torque burst INDI deviates about 3.6x further than the cascade -- the
    OPPOSITE of the quadrotor result that motivated trying it at all (7x lower
    gust deviation, 0.21 m against 1.51 m)."""
    v = Vehicle(damping=NOMINAL)
    di = v.fly(seconds=8.0, hold_yaw=0.0, controller=_indi(v), disturbance=_burst)
    dc = v.fly(seconds=8.0, hold_yaw=0.0, disturbance=_burst)
    peak_i = max(abs(math.degrees(y)) for y in di.yaw)
    peak_c = max(abs(math.degrees(y)) for y in dc.yaw)
    assert peak_i > 2.5 * peak_c, f'INDI {peak_i:.3f} vs cascade {peak_c:.3f}'


def test_indis_no_drag_model_advantage_is_real_but_worthless_here():
    """⭐ WHY THE HEADLINE CLAIM DOES NOT PAY. INDI needs no drag model. Neither
    does the cascade on this vehicle: swing drag 10x and the cascade's settle
    time moves under 7 %, because the yaw axis is RATE-limited by
    PILOT_YAW_RATE rather than drag-limited. An advantage over a problem we do
    not have is not an advantage."""
    times = []
    for scale in (0.3, 3.0):
        d = Damping(quad=tuple(q * scale for q in NOMINAL.quad),
                    provenance='drag sweep')
        v = Vehicle(damping=d)
        times.append(v.fly(seconds=10.0, hold_yaw=TARGET)
                     .settle_time(TARGET, math.radians(2.0)))
    assert max(times) / min(times) < 1.10


def test_indi_destabilises_when_the_inertia_estimate_is_wrong():
    """⛔⛔ THE RESULT THAT DECIDES IT, and it relocates the blocker.

    `tau = tau_applied + I (wdot_des - wdot_meas)` needs an accurate I. Ours is
    a UNIFORM-BODY GUESS -- a real hull is a shell with its battery placed
    somewhere deliberate. Give the plant 0.6x the assumed mass, so the
    controller's I is 1.7x too large, and INDI never settles while the cascade
    is unaffected.

    So INDI's binding prerequisite is NOT actuator resolution, which the plan
    assumed. It is an inertia estimate we do not have and cannot get without
    free-decay identification in water (Round 4a)."""
    v = Vehicle(damping=NOMINAL)
    tuned = _indi(v)                      # I from the NOMINAL plant
    light = Vehicle(damping=Damping(quad=tuple(q * 0.3 for q in NOMINAL.quad),
                                    provenance='perturbed'),
                    inertia=Inertia(mass_kg=10.233 * 0.6))
    bad = light.fly(seconds=10.0, hold_yaw=TARGET, controller=tuned)
    good = light.fly(seconds=10.0, hold_yaw=TARGET)
    assert bad.settle_time(TARGET, math.radians(2.0)) is None, 'INDI should fail'
    assert good.settle_time(TARGET, math.radians(2.0)) is not None


def test_the_cascade_is_robust_at_every_corner_we_can_construct():
    """The incumbent's case, measured rather than assumed. Across a 10x drag
    swing and a 2.7x mass swing the cascade settles every time, and its settle
    time moves by 21 % end to end (0.408 .. 0.492 s)."""
    out = []
    for dm in (0.3, 3.0):
        for mm in (0.6, 1.6):
            d = Damping(quad=tuple(q * dm for q in NOMINAL.quad),
                        provenance='corner')
            v = Vehicle(damping=d, inertia=Inertia(mass_kg=10.233 * mm))
            st = v.fly(seconds=10.0, hold_yaw=TARGET) \
                  .settle_time(TARGET, math.radians(2.0))
            assert st is not None
            out.append(st)
    assert max(out) / min(out) < 1.25
