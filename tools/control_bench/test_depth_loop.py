"""⭐ The depth loop, closed for the first time.

⛔ CLAUDE.md's largest standing unknown: *"The depth loop has never run closed.
`SROT_MOVE` enters the board's automatic mode and EVERY primitive there closes
the depth loop -- including a plain `move_forward`. An in-air move is not
partial validation (at ~0 m, target and measurement agree)."*

This runs the board's own `depth_control.cpp` against the Fossen plant. Not
water -- but the first time the code has run with feedback at all.

⚠ THE SIGN CONVENTION FLIPS AT THE MAVLINK BOUNDARY, and getting it wrong is
how the first three attempts here flew the hull to +20 m:

    inside the firmware   depth is POSITIVE metres DOWN
                          (`s_target` is clamped >= 0, and SURFACE sets it to 0)
    over `VFR_HUD`        depth is reported NEGATIVE below the surface
    Fossen / the plant    z is positive DOWN, and `Plant.depth_m` flips it to
                          the outward convention

So a caller of `depth::update` must pass POSITIVE-down, while everything it
reads off the wire is negative-down.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from flight import Vehicle, body_to_cad, dshot_to_fraction   # noqa: E402
from plant import Damping, Inertia                           # noqa: E402

BAND = Damping.from_cd(cd_axial=0.25, cd_transverse=1.00,
                       provenance='band midpoint, UNMEASURED -- tests only')


def dive(target_down_m, *, damping=BAND, mass=10.233, buoyancy_n=0.0,
         seconds=40.0, dt=0.002):
    """Hold `target_down_m` (POSITIVE metres down). Returns (final, overshoot,
    peak_rate)."""
    v = Vehicle(damping=damping, inertia=Inertia(mass_kg=mass))
    v.plant.nu = [0.0] * 6
    v.plant.eta = [0.0] * 6
    v.board.reset()
    v.board.depth_reset(target_down_m)
    peak = rate = 0.0
    for _ in range(int(seconds / dt)):
        down = -v.plant.depth_m
        out, _ = v.board.depth_update(stick_throttle=0.0, depth_m=down, dt=dt)
        # ⭐ The firmware is emphatic and corroborates it three ways: POSITIVE
        # throttle ASCENDS. Measured on this allocator, a NEGATIVE heave
        # ascends. So the demand is negated on the way in.
        a = v.alloc.allocate(**body_to_cad(heave=-out))
        u = [dshot_to_fraction(v.board, x) for x in a.thrusts]
        tau = v._wrench(u)
        tau[2] += buoyancy_n                  # +z is DOWN, so +N sinks
        v.plant.step(tau, dt)
        peak = max(peak, -v.plant.depth_m)
        rate = max(rate, abs(v.plant.nu[2]))
    return -v.plant.depth_m, peak - target_down_m, rate


@pytest.mark.parametrize('target', [0.5, 1.5, 3.0])
def test_the_depth_loop_reaches_and_holds_its_target(target):
    """It closes. ~17 mm of steady-state error on every target tried."""
    final, _, _ = dive(target)
    assert final == pytest.approx(target, abs=0.05)


def test_the_integrator_absorbs_net_buoyancy():
    """⭐ WHAT DEPTH_I IS FOR, and the one thing a real hull guarantees: it is
    never exactly neutral. Plus or minus 2 N of standing force is rejected
    completely."""
    for buoy in (+2.0, -2.0):
        final, _, _ = dive(1.5, buoyancy_n=buoy)
        assert final == pytest.approx(1.5, abs=0.02), f'{buoy} N left an offset'


def test_overshoot_saturates_at_a_fixed_DISTANCE():
    """⭐ THE MISSION-DESIGN NUMBER, and it is a distance rather than a
    percentage. The hull accelerates to a drag-limited descent (~0.68 m/s) and
    then needs a fixed distance to stop, so the overshoot is the same whether
    the step is 1 m or 5 m:

        target 0.5 m -> 0.202 m over      target 2.0 m -> 0.307 m
        target 1.0 m -> 0.307 m           target 5.0 m -> 0.307 m
    """
    overs = {t: dive(t)[1] for t in (1.0, 2.0, 5.0)}
    assert max(overs.values()) / min(overs.values()) < 1.10, overs
    assert 0.25 < overs[2.0] < 0.40


def test_the_overshoot_survives_the_whole_uncertainty_band():
    """0.263 .. 0.391 m across drag, thrust and mass -- a 1.5x spread, which is
    tight enough to design against. Quote 0.4 m of clearance, not 0.3."""
    out = []
    for cd in (0.80, 1.20):
        for m in (8.0, 13.6):
            d = Damping.from_cd(cd_axial=0.25, cd_transverse=cd,
                                provenance='sweep corner')
            out.append(dive(2.0, damping=d, mass=m)[1])
    assert 0.20 < min(out) and max(out) < 0.50


def test_a_shallow_target_overshoots_LESS_because_it_never_gets_up_to_speed():
    """The 0.5 m case is the exception that proves the mechanism: the hull does
    not reach terminal descent, so it needs less distance to stop."""
    assert dive(0.5)[1] < dive(2.0)[1]


def test_the_sign_convention_is_positive_down_inside_the_firmware():
    """⛔ THE TRAP THAT FLEW THE HULL TO +20 m THREE TIMES. `s_target` is
    clamped to >= 0 and SURFACE sets it to 0, so the firmware's depth is
    POSITIVE metres down -- while `VFR_HUD` reports negative below the surface.
    Passing the outward convention inward makes every target unreachable and
    the loop drives the hull straight up."""
    v = Vehicle(damping=BAND)
    v.board.depth_reset(2.0)
    out_deep, tgt = v.board.depth_update(stick_throttle=0.0, depth_m=3.0, dt=0.002)
    out_shallow, _ = v.board.depth_update(stick_throttle=0.0, depth_m=1.0, dt=0.002)
    assert tgt == pytest.approx(2.0)
    # deeper than target -> POSITIVE -> ascend; shallower -> NEGATIVE -> descend
    assert out_deep > 0.0, 'measured deeper than target must command ascend'
    assert out_shallow < 0.0, 'measured shallower must command descend'
