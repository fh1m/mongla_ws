"""The geometric allocator, against the hull it claims to describe.

These are TRUTH tests. The matrix is not compared against another matrix we
wrote; each case asserts something physical about a vehicle whose thruster
positions were read off the CAD on 2026-09-23 -- that pushing two tunnels the
same way makes no moment, that pushing them opposite ways makes yaw with the
measured lever arm, and that no combination of any of them makes roll.

⚠ Units are signed fractions of one thruster's output, never newtons. There is
no `k_n_per_rpm2` in this tree and the CAD has no material, so nothing here
converts to force or predicts motion.
"""
import math
from pathlib import Path

import pytest

from mongla_control.geometric_allocation import (
    AXES, B, THRUSTERS, UNACTUATED, GeometricAllocator, build_b)

IDX = {name: i for i, name in enumerate(AXES)}


@pytest.fixture()
def alloc():
    return GeometricAllocator()


# ── the constant must not drift from the CAD it mirrors ─────────────────────

def test_the_mirrored_geometry_matches_the_yaml_it_came_from():
    """`mongla_control` depends on pyserial and nothing else, so the yaml is
    not parsed at runtime and the geometry lives as a module constant. That is
    one truth in two copies -- the same risk `MIXER` carries against
    `mixer.cpp`, handled the same way: by comparing them rather than trusting."""
    yaml = pytest.importorskip('yaml')
    path = Path(__file__).resolve().parents[1] / 'config' / 'hull_geometry.yaml'
    if not path.exists():
        pytest.skip(f'{path} not present')

    spec = yaml.safe_load(path.read_text())['thrusters']
    assert len(spec) == len(THRUSTERS), 'thruster COUNT drifted from the CAD'
    for entry, (name, pos, axis) in zip(spec, THRUSTERS):
        assert entry['name'] == name
        assert entry['position_m'] == pytest.approx(list(pos), abs=1e-9)
        assert [float(v) for v in entry['axis']] == pytest.approx(list(axis))


# ── what the geometry says about the vehicle ────────────────────────────────

def test_roll_is_unactuated_and_that_is_derived_not_declared():
    """Every thruster's line of action passes through the roll axis. This is a
    property of the positions, so it is COMPUTED from them -- if a thruster ever
    moves off-centre, the list follows it instead of lying."""
    assert UNACTUATED == ('roll',)


def test_the_five_other_axes_are_actuated():
    for axis in ('sway', 'surge', 'heave', 'pitch', 'yaw'):
        assert axis not in UNACTUATED


def test_the_yaw_lever_arm_is_the_measured_one():
    """0.1750 m, read off the CAD -- not a sign. This is the number a +-1 mixer
    throws away."""
    assert B[IDX['yaw']][0] == pytest.approx(0.1750, abs=1e-6)
    assert B[IDX['yaw']][1] == pytest.approx(-0.1750, abs=1e-6)


def test_the_pitch_lever_arm_is_the_measured_one():
    assert B[IDX['pitch']][2] == pytest.approx(-0.2595, abs=1e-6)
    assert B[IDX['pitch']][3] == pytest.approx(0.2595, abs=1e-6)


def test_the_axial_offset_produces_a_real_parasitic_pitch():
    """The axial unit sits 8.1 mm off the centreline in Z, so surge makes a
    small pitch. It is tiny, it is real, and a +-1 matrix cannot express it at
    all -- which is the kind of coupling that shows up as a trim nobody can
    explain."""
    assert B[IDX['pitch']][4] == pytest.approx(-0.0081, abs=1e-6)
    assert abs(B[IDX['pitch']][4]) < 0.01


# ── the physics, stated as behaviour ────────────────────────────────────────

def test_two_lateral_tunnels_together_make_sway_and_no_yaw(alloc):
    """They sit at +-0.1750 m, so driving them the same way cancels the moment
    exactly. If this fails, the two positions are not symmetric."""
    got = alloc.allocate(sway=0.5)

    assert got.achieved[IDX['sway']] == pytest.approx(0.5, abs=1e-9)
    assert got.achieved[IDX['yaw']] == pytest.approx(0.0, abs=1e-9)
    assert got.thrusts[0] == pytest.approx(got.thrusts[1], abs=1e-9)


def test_two_lateral_tunnels_opposed_make_yaw_and_no_sway(alloc):
    got = alloc.allocate(yaw=0.05)

    assert got.achieved[IDX['yaw']] == pytest.approx(0.05, abs=1e-9)
    assert got.achieved[IDX['sway']] == pytest.approx(0.0, abs=1e-9)
    assert got.thrusts[0] == pytest.approx(-got.thrusts[1], abs=1e-9)


def test_heave_uses_only_the_vertical_pair(alloc):
    got = alloc.allocate(heave=0.4)

    assert got.thrusts[0] == pytest.approx(0.0, abs=1e-9)
    assert got.thrusts[1] == pytest.approx(0.0, abs=1e-9)
    assert got.achieved[IDX['heave']] == pytest.approx(0.4, abs=1e-9)


def test_surge_comes_from_the_axial_unit(alloc):
    """It is the ONLY source of forward thrust on this hull."""
    got = alloc.allocate(surge=0.6)

    assert got.thrusts[4] == pytest.approx(0.6, abs=1e-9)
    assert got.achieved[IDX['surge']] == pytest.approx(0.6, abs=1e-9)
    # The lateral pair contributes nothing: their axis has no surge component.
    assert got.thrusts[0] == pytest.approx(0.0, abs=1e-9)
    assert got.thrusts[1] == pytest.approx(0.0, abs=1e-9)


def test_the_vertical_pair_TRIMS_OUT_the_axial_parasitic_pitch(alloc):
    """⭐ THE CAPABILITY A +-1 MIXER CANNOT HAVE, and it was not anticipated when
    this file was first written -- the test asserted the verticals stayed at
    zero, and the allocator was right where the test was wrong.

    The axial unit sits 8.1 mm off centre, so 0.6 of surge alone would make a
    pitch of -0.00486. Asked for surge with pitch = 0, the solver fires the
    vertical pair at -+0.009364 and cancels it EXACTLY.

    A +-1 mixer carries no moment arms, so it cannot know the axial pitches the
    hull at all: the vehicle would simply nose up or down whenever it
    accelerated, and it would read as a trim nobody could explain."""
    got = alloc.allocate(surge=0.6)

    uncorrected = B[IDX['pitch']][4] * 0.6
    assert uncorrected == pytest.approx(-0.00486, abs=1e-5)

    assert got.achieved[IDX['pitch']] == pytest.approx(0.0, abs=1e-12)
    assert got.thrusts[2] == pytest.approx(-got.thrusts[3], abs=1e-12)
    assert abs(got.thrusts[2]) > 1e-4, 'the verticals must actually work here'


def test_letting_pitch_float_is_not_the_same_as_demanding_zero(alloc):
    """The trim above is not a side effect -- it happens because pitch=0 was
    REQUESTED. This pins that the solver is answering the whole wrench, not
    patching one axis."""
    demanded = alloc.allocate(surge=0.6, pitch=0.0)
    tilted = alloc.allocate(surge=0.6, pitch=0.05)

    assert demanded.achieved[IDX['pitch']] == pytest.approx(0.0, abs=1e-12)
    assert tilted.achieved[IDX['pitch']] == pytest.approx(0.05, abs=1e-9)
    assert tilted.thrusts[2] != pytest.approx(demanded.thrusts[2], abs=1e-6)


# ── refusal, which is the point ─────────────────────────────────────────────

def test_a_roll_demand_is_REFUSED_not_quietly_dropped(alloc):
    """⛔ The failure this guards. Rounding an impossible request to zero lets a
    controller integrate against an axis with no actuator behind it, forever."""
    got = alloc.allocate(roll=0.5)

    assert 'roll' in got.refused
    assert got.residual[IDX['roll']] == pytest.approx(0.5, abs=1e-9)
    assert all(t == pytest.approx(0.0, abs=1e-9) for t in got.thrusts)


def test_a_roll_demand_does_not_corrupt_the_achievable_axes(alloc):
    """Asking for something impossible must not degrade what IS possible."""
    clean = alloc.allocate(surge=0.3, yaw=0.04)
    dirty = alloc.allocate(surge=0.3, yaw=0.04, roll=0.9)

    assert dirty.thrusts == pytest.approx(clean.thrusts, abs=1e-9)
    assert 'roll' in dirty.refused


def test_no_roll_asked_means_nothing_refused(alloc):
    assert alloc.allocate(surge=0.2, heave=0.2).refused == ()


# ── a dead thruster is a limit, not a rewrite ───────────────────────────────

def test_killing_a_lateral_tunnel_costs_yaw_but_keeps_the_vehicle_flying(alloc):
    """⭐ The single strongest argument for allocating over geometry: a +-1
    mixer cannot express a dead thruster AT ALL. Here it is one call, and the
    remaining four re-solve."""
    alloc.disable('lateral_a')

    got = alloc.allocate(surge=0.5, heave=0.3)

    assert got.achieved[IDX['surge']] == pytest.approx(0.5, abs=1e-9)
    assert got.achieved[IDX['heave']] == pytest.approx(0.3, abs=1e-9)
    assert got.thrusts[0] == 0.0
    assert got.disabled == ('lateral_a',)


def test_killing_a_lateral_tunnel_makes_sway_and_yaw_inseparable(alloc):
    """With one lateral tunnel left, sway and yaw share a single actuator: you
    cannot have one without the other. The allocator must SHOW that rather than
    pretending it delivered both."""
    alloc.disable('lateral_a')

    got = alloc.allocate(sway=0.4, yaw=0.0)

    assert abs(got.achieved[IDX['yaw']]) > 1e-3, (
        'a single lateral tunnel must induce yaw')


def test_killing_both_lateral_tunnels_makes_yaw_unactuated_too(alloc):
    """`unactuated` is recomputed from the LIVE geometry, so a fault grows the
    list. That is what makes it useful as a health signal."""
    alloc.disable('lateral_a', 'lateral_b')

    assert set(alloc.unactuated) >= {'roll', 'yaw', 'sway'}
    assert 'yaw' in alloc.allocate(yaw=0.1).refused


def test_disabling_an_unknown_thruster_raises(alloc):
    with pytest.raises(KeyError):
        alloc.disable('nonexistent')


def test_every_thruster_dead_produces_nothing_and_says_so(alloc):
    alloc.disable(*[t[0] for t in THRUSTERS])

    got = alloc.allocate(surge=1.0)

    assert all(t == 0.0 for t in got.thrusts)
    assert got.residual[IDX['surge']] == pytest.approx(1.0)


# ── saturation preserves direction ──────────────────────────────────────────

def test_an_over_large_demand_is_scaled_not_clipped(alloc):
    """A clipped mix points somewhere nobody asked for. Uniform scale-down
    keeps the wrench's direction and loses only magnitude -- the same choice
    the firmware makes per group."""
    got = alloc.allocate(heave=4.0)

    assert got.saturated
    assert max(abs(t) for t in got.thrusts) == pytest.approx(1.0, abs=1e-9)
    assert got.achieved[IDX['heave']] > 0


def test_uniform_scaling_keeps_the_ratio_between_axes(alloc):
    """⚠ THIS IS A PROPERTY OF THE SCALING FALLBACK, NOT OF THE ALLOCATOR.

    It held for every saturating demand until saturation handling was added,
    and the test was written as if it were universal. It is not: the priority
    fit deliberately gives up direction-preservation to rank the axes, and this
    test failed the moment the default changed -- correctly. The contract is
    per-mode, so the test is now per-mode."""
    small = alloc.allocate(surge=0.2, heave=0.1, prioritise=False)
    huge = alloc.allocate(surge=4.0, heave=2.0, prioritise=False)

    r_small = small.achieved[IDX['surge']] / small.achieved[IDX['heave']]
    r_huge = huge.achieved[IDX['surge']] / huge.achieved[IDX['heave']]
    assert r_huge == pytest.approx(r_small, rel=1e-6)


def test_priority_does_NOT_preserve_direction_and_that_is_the_point(alloc):
    """The trade it makes, stated as a test so it cannot be mistaken for a bug.
    The priority fit bends the wrench's direction so the higher-ranked axis is
    delivered whole; uniform scaling keeps the direction and shorts both."""
    want = {'surge': 4.0, 'heave': 2.0}
    scaled = alloc.allocate(**want, prioritise=False)
    ranked = alloc.allocate(**want)

    r_scaled = scaled.achieved[IDX['surge']] / scaled.achieved[IDX['heave']]
    r_ranked = ranked.achieved[IDX['surge']] / ranked.achieved[IDX['heave']]
    assert r_ranked != pytest.approx(r_scaled, rel=1e-3)
    assert ranked.achieved[IDX['heave']] > scaled.achieved[IDX['heave']]


def test_a_feasible_demand_is_not_reported_saturated(alloc):
    assert not alloc.allocate(surge=0.3).saturated


# ── the quantitative claim about the +-1 mixer ──────────────────────────────

def test_the_geometric_columns_are_unit_vectors_not_signs():
    """The whole argument for this module. A +-1 entry used as a force sum
    overstates a 45-degree contribution by 1/cos(45) = 1.414; every force column
    here has unit norm by construction, so there is nothing to overstate."""
    for j in range(len(THRUSTERS)):
        norm = math.sqrt(sum(B[i][j] ** 2 for i in range(3)))
        assert norm == pytest.approx(1.0, abs=1e-9)

    assert 1.0 / math.cos(math.radians(45)) == pytest.approx(1.4142, abs=1e-4)


def test_build_b_is_a_pure_function_of_the_geometry():
    """Position and axis in, matrix out -- so a CAD change is a data change."""
    moved = (('only', (0.0, 0.5, 0.0), (1.0, 0.0, 0.0)),)

    b = build_b(moved)

    assert b[IDX['sway']][0] == pytest.approx(1.0)
    assert b[IDX['yaw']][0] == pytest.approx(-0.5)   # r x axis, arm 0.5 m


# ── priority: who comes up short when the thrusters run out (#44) ─────────

def test_the_ladder_is_the_boards_ladder_renamed():
    """One law, two vocabularies. The order is `allocation.py`'s; this module
    restates it only because the bench loads it without the package. If either
    table moves alone, this fails."""
    from mongla_control import allocation
    from mongla_control.geometric_allocation import (
        BOARD_AXIS, HORIZONTAL_PRIORITY, VERTICAL_PRIORITY)
    assert tuple(BOARD_AXIS[a] for a in HORIZONTAL_PRIORITY) == allocation.HORIZONTAL_PRIORITY
    assert tuple(BOARD_AXIS[a] for a in VERTICAL_PRIORITY) == allocation.VERTICAL_PRIORITY


def test_an_unactuated_axis_holds_no_rank():
    """Issue #9: roll heads the vertical ladder and this hull cannot produce
    roll. It is dropped before ranking, so pitch is first, not second."""
    from mongla_control.geometric_allocation import priority_order
    assert priority_order() == ('yaw', 'pitch', 'sway', 'heave', 'surge')


L_YAW = 0.1750


@pytest.mark.parametrize('sway,yaw', [(1.5, 0.10), (1.5, 0.15), (1.9, 0.20)])
def test_yaw_is_delivered_whole_and_sway_gets_what_is_left(alloc, sway, yaw):
    """⭐ THE FALSIFIER FROM #44, and the closed form it must equal.

    The lateral pair is a 2x2 block: yaw needs a differential of yaw/L, and
    sway gets 2*limit minus that. Total-error least squares delivered 88 %,
    60 % and 11 % of these yaws -- the last worse than the uniform scale it was
    written to beat -- because a unit of yaw error weighs 1/33 of a unit of
    sway error in mixed units."""
    got = alloc.allocate(sway=sway, yaw=yaw)

    assert got.achieved[IDX['yaw']] == pytest.approx(yaw, abs=1e-9)
    assert got.achieved[IDX['sway']] == pytest.approx(2.0 - yaw / L_YAW, abs=1e-9)


def test_pitch_is_delivered_whole_before_heave(alloc):
    """The vertical half: attitude before throttle. Least squares gave up 29 %
    of this pitch to keep heave."""
    got = alloc.allocate(heave=1.2, pitch=0.30)

    assert got.achieved[IDX['pitch']] == pytest.approx(0.30, abs=1e-9)
    assert got.achieved[IDX['heave']] < 1.2


def test_a_saturating_surge_no_longer_steals_heave_authority(alloc):
    """⛔ THE FAILURE THE FIRMWARE'S OWN COMMENT DESCRIBES, one layer up. Its
    mixer note records that a global scale meant "a hard forward burst silently
    cost a third of the vehicle's roll/pitch authority, in the manoeuvre where
    you want it most". Heave outranks surge, so it stays whole."""
    want = {'surge': 1.4, 'heave': 0.9}
    scaled = alloc.allocate(**want, prioritise=False)
    ranked = alloc.allocate(**want)

    assert scaled.achieved[IDX['heave']] < 0.7        # authority thrown away
    assert ranked.achieved[IDX['heave']] == pytest.approx(0.9, abs=1e-6)
    assert 'axial' in ranked.clamped


HULLS = [(), ('lateral_b',), ('vertical_a',), ('lateral_b', 'vertical_a', 'vertical_b')]


@pytest.mark.parametrize('disabled', HULLS)
def test_no_axis_is_shorted_while_its_thrusters_have_room(disabled):
    """The property that makes it a fit and not a cut: an axis comes up short
    only because some thruster it needs is AT its stop. Also the #60 guard --
    a thruster pinned and then scaled off its stop is exactly room left on the
    table. Seeded, 2000 demands per hull, live and dead thrusters alike."""
    import random
    from mongla_control.geometric_allocation import priority_order
    rng = random.Random(44)
    a = GeometricAllocator(disabled=disabled)
    order = priority_order(a.unactuated)

    for _ in range(2000):
        req = {k: rng.uniform(-3.0, 3.0) for k in order}
        got = a.allocate(**req)
        assert max(abs(t) for t in got.thrusts) <= 1.0 + 1e-9, req
        if any(abs(got.residual[IDX[k]]) > 1e-6 for k in order):
            if a.allocate(**req, prioritise=False).scale < 1.0:
                assert got.clamped, req


@pytest.mark.parametrize('disabled', [(), ('vertical_a',)])
def test_the_top_axis_never_does_worse_than_uniform_scaling(disabled):
    """Lexicographic, not total: the FIRST-ranked axis gets at least what the
    uniform scale would have given it. Total error in mixed units is not a
    metric this allocator answers to (#44).

    ⚠ Only on hulls where yaw has a column of its own. With a lateral tunnel
    dead, yaw and sway come from ONE thruster and are a single direction
    (`test_killing_a_lateral_tunnel_makes_sway_and_yaw_inseparable`); ranking
    two halves of one column has no meaning, and the bound does not hold."""
    import random
    from mongla_control.geometric_allocation import priority_order
    rng = random.Random(4444)
    a = GeometricAllocator(disabled=disabled)
    order = priority_order(a.unactuated)

    for _ in range(2000):
        req = {k: rng.uniform(-3.0, 3.0) for k in order}
        top = order[0]
        ranked = a.allocate(**req)
        scaled = a.allocate(**req, prioritise=False)
        assert (abs(ranked.residual[IDX[top]])
                <= abs(scaled.residual[IDX[top]]) + 1e-9), req


def test_a_clamped_thruster_sits_exactly_at_its_limit(alloc):
    got = alloc.allocate(heave=1.9, pitch=0.45)

    assert got.clamped
    for name in got.clamped:
        assert abs(got.thrusts[got.names.index(name)]) == pytest.approx(1.0, abs=1e-9)


def test_nothing_is_clamped_when_the_demand_fits(alloc):
    got = alloc.allocate(heave=0.3, pitch=0.05)

    assert got.clamped == ()
    assert not got.saturated


def test_no_thruster_ever_exceeds_its_limit(alloc):
    """The invariant the whole routine exists to keep. Redistribution must never
    hand back a command the actuator cannot execute."""
    for req in ({'heave': 5.0}, {'sway': 3.0, 'yaw': 0.4},
                {'surge': 9.0, 'heave': 9.0, 'pitch': 1.0}):
        got = alloc.allocate(**req)
        assert max(abs(t) for t in got.thrusts) <= 1.0 + 1e-9, req


def test_priority_still_refuses_an_unactuated_axis(alloc):
    """Saturation handling must not quietly re-admit an impossible request."""
    got = alloc.allocate(heave=1.9, roll=0.8)

    assert 'roll' in got.refused
    assert got.residual[IDX['roll']] == pytest.approx(0.8, abs=1e-9)


def test_a_single_thruster_left_is_still_bounded(alloc):
    """With a single thruster left the fit has one axis to grant, and the
    output must still sit at the limit, not past it."""
    alloc.disable('lateral_a', 'lateral_b', 'vertical_a', 'vertical_b')

    got = alloc.allocate(surge=5.0)

    assert abs(got.thrusts[4]) == pytest.approx(1.0, abs=1e-9)
    assert got.saturated
