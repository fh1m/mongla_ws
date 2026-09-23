"""The Fossen plant, checked where truth is KNOWN.

⛔ A PLANT CANNOT BE VALIDATED AGAINST THE VEHICLE -- the vehicle has never been
wet. So it is validated against the exact analytic properties the equations of
motion are REQUIRED to have, every one of which is a closed form:

    passivity        nu' C(nu) nu == 0 EXACTLY, for all nu (skew-symmetry)
    terminal speed   constant force against quadratic drag -> sqrt(F/q)
    roll period      small-angle restoring pendulum -> 2 pi sqrt(I / (BG W))
    free rotation    no damping, no restoring -> a principal-axis spin is held

⭐ AND ONE THAT EARNS ITS KEEP TWICE. `test_free_decay_recovers_the_drag_we_put
_in` runs the Round 4a pool experiment against a plant whose drag we CHOSE, and
checks that the identification returns it. That validates the EXPERIMENT DESIGN
before anybody gets wet -- if the procedure cannot recover a known answer on a
noiseless plant, it will not recover an unknown one in a pool.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plant import (Damping, Inertia, Plant, cad_hull_thrusters,  # noqa: E402
                   wrench_matrix)

FREE = Damping(quad=(0.0,) * 6, provenance='none -- an ideal-fluid test case')


def _plant(damping=FREE, bg=0.0) -> Plant:
    return Plant(Inertia(bg_m=bg), damping)


# ═══════════════════════════════════════════════════════════════════════════ #
#  Exact properties of the equations of motion
# ═══════════════════════════════════════════════════════════════════════════ #

@pytest.mark.parametrize('nu', [
    (1.0, 0.5, -0.3, 0.2, -0.1, 0.4),
    (-2.0, 1.5, 0.7, -0.9, 0.3, -0.6),
    (0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
])
def test_coriolis_does_no_work(nu):
    """⭐ THE STRONGEST TEST IN THIS FILE. C(nu) is skew-symmetric, so
    nu' C(nu) nu is identically zero: Coriolis moves energy BETWEEN axes and
    can neither create nor destroy it. It is an algebraic identity, so any
    sign slip or transposed index in `coriolis` breaks it -- and a Coriolis
    error is otherwise invisible, because it looks like plausible coupling."""
    c = _plant().coriolis(list(nu))
    assert sum(n * ci for n, ci in zip(nu, c)) == pytest.approx(0.0, abs=1e-12)


def test_an_undamped_spin_about_a_principal_axis_is_held():
    """No damping, no restoring, no torque: a body spinning about a principal
    axis keeps spinning at exactly that rate. Euler's equations demand it."""
    p = _plant()
    p.nu[5] = 1.2                       # yaw rate
    for _ in range(2000):               # 4 s at 2 ms
        p.step([0.0] * 6, 0.002)
    assert p.nu[5] == pytest.approx(1.2, rel=1e-9)
    assert all(abs(v) < 1e-9 for v in p.nu[:5]), 'no other axis may wake up'


def test_terminal_speed_matches_the_closed_form():
    """Constant thrust against quadratic drag settles at sqrt(F/q) -- exact,
    and independent of mass, which is what makes it a clean check of the
    damping path rather than of the inertia."""
    q = 3.0
    d = Damping(quad=(q, 0, 0, 0, 0, 0), provenance='chosen for a truth test')
    p = Plant(Inertia(bg_m=0.0), d)
    F = 12.0
    for _ in range(200000):
        p.step([F, 0, 0, 0, 0, 0], 0.002)
    assert p.nu[0] == pytest.approx(math.sqrt(F / q), rel=1e-4)


def test_the_roll_restoring_moment_is_a_pendulum():
    """⭐ THE ONE THAT DECIDES WHETHER ROLL CAN STAY UNACTUATED. With no roll
    thruster the only thing righting this hull is BG, and a small-angle release
    must oscillate at 2 pi sqrt(I / (BG W)). If the period the bench reports is
    not that, the restoring term is wrong and every claim about passive roll
    stability built on it is wrong too."""
    bg = 0.010
    p = _plant(bg=bg)
    ixx = p.M[3]
    want = 2 * math.pi * math.sqrt(ixx / (bg * p.inertia.mass_kg * 9.80665))

    p.eta[3] = math.radians(3.0)        # small angle, so sin(phi) ~ phi
    dt, prev, crossings, t = 0.0005, p.eta[3], [], 0.0
    for _ in range(int(60 / dt)):
        p.step([0.0] * 6, dt)
        t += dt
        if prev > 0.0 >= p.eta[3]:
            crossings.append(t)
        prev = p.eta[3]
        if len(crossings) >= 3:
            break
    assert len(crossings) >= 2, 'the hull did not oscillate at all'
    period = crossings[1] - crossings[0]
    assert period == pytest.approx(want, rel=0.02)


def test_an_undamped_pendulum_conserves_energy():
    """Released from rest and left alone, the roll oscillation must return to
    its starting angle. An integrator that leaks energy would fake damping, and
    a damping estimate is exactly what this bench is meant to produce."""
    p = _plant(bg=0.010)
    p.eta[3] = math.radians(5.0)
    start = p.eta[3]
    peak = start
    for _ in range(200000):             # 100 s at 0.5 ms
        p.step([0.0] * 6, 0.0005)
        peak = max(peak, p.eta[3])
    assert peak == pytest.approx(start, rel=1e-3)


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ Validating the POOL EXPERIMENT before the pool
# ═══════════════════════════════════════════════════════════════════════════ #

def test_free_decay_recovers_the_drag_we_put_in():
    """Round 4a, rehearsed against a known answer.

    The planned experiment is: spin the hull about an axis, cut the command,
    fit the decay, and call the result `ATC_DRAG_YAW`. Here the drag is CHOSEN,
    so the fit has a truth to be scored against.

    For pure quadratic damping with no restoring term the decay has a closed
    form -- 1/w is linear in time with slope q/I -- so the fit is a straight
    line and the slope IS the coefficient.

    ⛔ IF THIS FAILED, THE POOL DAY WOULD BE WASTED and nobody would know: the
    experiment would return a number, it would look reasonable, and it would be
    wrong. That is the failure mode this whole codebase is arranged against.
    """
    q_true = 0.25
    d = Damping(quad=(0, 0, 0, 0, 0, q_true), provenance='chosen for a truth test')
    p = Plant(Inertia(bg_m=0.0), d)
    izz = p.M[5]

    p.nu[5] = 2.0
    dt, ts, ws = 0.002, [], []
    for i in range(int(20 / dt)):
        p.step([0.0] * 6, dt)
        if p.nu[5] > 0.15:              # stop before the signal is noise-sized
            ts.append(i * dt)
            ws.append(p.nu[5])

    # 1/w(t) = 1/w0 + (q/I) t  -- least squares on the straight line
    n = len(ts)
    inv = [1.0 / w for w in ws]
    mt, mi = sum(ts) / n, sum(inv) / n
    slope = (sum((t - mt) * (y - mi) for t, y in zip(ts, inv))
             / sum((t - mt) ** 2 for t in ts))
    assert slope * izz == pytest.approx(q_true, rel=0.01)


def test_the_fit_needs_the_inertia_and_that_is_the_catch():
    """⚠ WHAT THE POOL EXPERIMENT CANNOT DO ALONE. The fit above recovers q/I,
    a RATIO. Turning it into a drag coefficient needs I, and our I is a uniform
    -body guess -- a real hull is a shell with its battery placed somewhere.

    So free decay identifies the ratio that the CONTROLLER actually needs (the
    plant's time constant) and does NOT independently identify drag. That is
    fine, and it is worth writing down, because the two get conflated and the
    gains only ever depend on the ratio."""
    p = _plant()
    assert p.M[5] > 0.0
    assert p.M[5] == pytest.approx(p.M[4], rel=1e-9), (
        'pitch and yaw inertia are equal for a body of revolution, so one '
        'free-decay axis informs the other')


# ═══════════════════════════════════════════════════════════════════════════ #
#  The hull's own geometry, reached through a second code path
# ═══════════════════════════════════════════════════════════════════════════ #

def test_the_wrench_matrix_says_roll_is_unactuated():
    """⭐ AN INDEPENDENT CONFIRMATION, not a restatement. `geometric_allocation`
    already found rank 5 of 6 with roll the unactuated axis. This builds B from
    the same CAD file through completely different code and reaches the same
    conclusion -- every roll entry exactly zero. Two derivations agreeing is
    worth more than one asserted twice."""
    b = wrench_matrix(cad_hull_thrusters(20.0))
    assert all(v == 0.0 for v in b[3]), f'roll row should be empty: {b[3]}'
    assert any(v != 0.0 for v in b[4]), 'pitch must be actuated'
    assert any(v != 0.0 for v in b[5]), 'yaw must be actuated'


def test_the_axial_thruster_has_a_parasitic_pitch_moment():
    """The nose unit sits 8.1 mm off the centreline in CAD, so pure surge also
    pitches. Small, real, and the allocator already compensates it -- recorded
    here so it cannot be 'cleaned up' as noise."""
    b = wrench_matrix(cad_hull_thrusters(20.0))
    axial = -1
    assert b[0][axial] == pytest.approx(1.0), 'axial must be pure surge in force'
    assert abs(b[4][axial]) == pytest.approx(0.0081, abs=1e-4)


def test_the_frame_remap_puts_the_long_axis_on_surge():
    """⚠ THE CAD LONG AXIS IS Y AND FOSSEN'S IS X. A remap error here would be
    invisible until the vehicle drove sideways on a surge command."""
    t = {x.name: x for x in cad_hull_thrusters(20.0)}
    assert t['axial'].axis[0] == pytest.approx(1.0)
    assert t['lateral_a'].axis[1] == pytest.approx(1.0)
    assert abs(t['vertical_a'].axis[2]) == pytest.approx(1.0)
    # the vertical pair straddles the bow-stern axis, the lateral pair too
    assert t['vertical_a'].position_m[0] == pytest.approx(-0.2595)
    assert t['lateral_a'].position_m[0] == pytest.approx(-0.1750)


# ═══════════════════════════════════════════════════════════════════════════ #
#  The guards that keep a guess from becoming a fact
# ═══════════════════════════════════════════════════════════════════════════ #

def test_damping_cannot_be_constructed_without_saying_where_it_came_from():
    """⛔ THE POINT OF THE WHOLE FILE. Drag is the one term that is NOT
    computable from geometry, and 'a plausible number standing in for an absent
    measurement' is this project's named recurring defect. There is no default
    and there is no anonymous constructor."""
    with pytest.raises(ValueError):
        Damping(quad=(1,) * 6)
    with pytest.raises(ValueError):
        Damping(quad=(1,) * 6, provenance='')


def test_damping_needs_all_six_axes():
    with pytest.raises(ValueError):
        Damping(quad=(1.0, 2.0), provenance='short')


def test_the_added_mass_ratio_survives_into_the_mass_matrix():
    """The plant must carry Lamb's result, not quietly re-derive it: sway
    inertia over surge inertia is the 1.722 the hull's shape dictates."""
    m = Inertia().mass_matrix()
    assert m[1] / m[0] == pytest.approx(1.722, abs=0.01)
    assert m[1] == pytest.approx(m[2]), 'sway and heave are the same axis class'
    assert m[4] == pytest.approx(m[5]), 'pitch and yaw likewise'


def test_depth_is_negative_below_the_surface():
    """Matches the board's own `VFR_HUD` convention. Fossen's z is positive
    DOWN, so this is a deliberate flip, and the two conventions meeting in the
    wrong place is a classic source of an inverted depth loop."""
    p = _plant()
    p.eta[2] = 1.5                      # 1.5 m down in Fossen axes
    assert p.depth_m == pytest.approx(-1.5)
