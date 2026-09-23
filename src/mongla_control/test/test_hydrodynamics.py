"""Lamb's added-mass coefficients, checked against cases where truth is KNOWN.

⛔ WHY THESE ARE TRUTH TESTS AND NOT AGREEMENT TESTS. Comparing this
implementation against another implementation, or against the sim's BlueROV2
numbers, would measure agreement and could not rank them. Potential-flow added
mass has EXACT closed-form answers in two limits, and both are reachable from
the same function:

    a sphere (fineness -> 1)   k1 = k2 = 1/2 exactly, k' = 0 exactly
    a slender rod (f -> inf)   k1 -> 0, k2 -> 1 exactly

The sphere result -- that a sphere drags half its own displaced mass with it --
is one of the oldest exact results in hydrodynamics. The slender limit is the
2D circular cylinder, whose added mass equals its displaced mass exactly. An
implementation that hits both limits and is monotone between them is right for
the reason the limits are right, not because it agrees with something.
"""
import math

import pytest

from mongla_control import hydrodynamics as hy


# --------------------------------------------------------------------------- #
#  The exact limits
# --------------------------------------------------------------------------- #

def test_the_sphere_limit_is_one_half():
    """⭐ A sphere carries exactly half its displaced mass as added mass, on
    both axes, and has ZERO added inertia in rotation. Approached from the
    prolate side, because the formulas are singular at e = 0."""
    k = hy.lamb_coefficients(1.0, 0.99999)
    assert k.k1 == pytest.approx(0.5, abs=1e-3)   # tight: this one is EXACT
    assert k.k2 == pytest.approx(0.5, abs=1e-3)
    assert k.kp == pytest.approx(0.0, abs=1e-3)


def test_the_slender_limit_is_zero_and_one():
    """A very long thin body: nothing to push axially, and transversely it is a
    2D cylinder, whose added mass equals its displaced mass exactly."""
    k = hy.lamb_coefficients(1000.0, 1.0)
    assert k.k1 == pytest.approx(0.0, abs=1e-3)
    assert k.k2 == pytest.approx(1.0, abs=1e-3)


def test_the_slender_limit_of_the_ROTATIONAL_coefficient_is_one():
    """⛔ ADDED BECAUSE INJECTION-VERIFICATION CAUGHT ITS ABSENCE. Multiplying
    k' by 0.93 passed all sixteen earlier tests: the sphere limit checks
    k' == 0, and 0.93 * 0 is still 0. The coefficient the PITCH AND YAW loops
    depend on was effectively untested.

    The second anchor, and it is exact. A slender body rotating about a
    transverse axis is a stack of 2D cylinder sections by strip theory, each
    carrying added mass equal to its own displaced mass, so its added moment of
    inertia tends to the displaced fluid's own moment about that axis -- a ratio
    of exactly 1. With k'(sphere) = 0 and k'(slender) = 1 the curve is pinned at
    both ends, and a multiplicative error can no longer hide.
    """
    assert hy.lamb_coefficients(1000.0, 1.0).kp == pytest.approx(1.0, abs=1e-3)


def test_the_rotational_coefficient_is_bounded_and_monotone():
    """Between the anchors it must rise, and it may never leave [0, 1]: a body
    cannot carry a negative added inertia, nor more than the fluid it displaces
    has about the same axis."""
    kps = [hy.lamb_coefficients(f, 1.0).kp for f in (1.5, 2, 3, 4, 6, 10, 50)]
    assert all(0.0 <= v <= 1.0 for v in kps)
    assert all(a < b for a, b in zip(kps, kps[1:]))


def test_this_hull_pays_a_real_rotational_penalty():
    """⚠ THE NUMBER THAT SIZES THE ATTITUDE LOOPS. At fineness 4 the hull
    carries 61 % of the displaced fluid's transverse moment as added inertia,
    so pitch and yaw are ~1.6x more sluggish than the dry vehicle suggests.
    Anyone tuning ATC_RAT_PIT_* or ATC_RAT_YAW_* against a dry-spun hull is
    tuning the wrong plant."""
    kp = hy.hull_added_mass().kp
    assert kp == pytest.approx(0.611, abs=0.01)
    assert 0.5 < kp < 0.7


@pytest.mark.parametrize('fineness,k1,k2', [
    # Lamb art. 114 as tabulated in the standard references, for the fineness
    # ratios an AUV actually has.
    (2.0,  0.209, 0.702),
    (4.0,  0.082, 0.860),
    (5.0,  0.059, 0.895),
    (10.0, 0.021, 0.960),
])
def test_against_the_published_table(fineness, k1, k2):
    """A CROSS-CHECK, at the tolerance the evidence deserves -- not the proof.

    ⚠ THE TOLERANCE HERE IS DELIBERATELY LOOSER THAN THE LIMIT TESTS ABOVE, and
    the reason is about evidence rather than about physics. These four rows are
    three-decimal table values; the limits are exact closed forms. When the
    f = 2 row disagreed by 0.0022 the tempting move was to adjust the
    implementation until it matched -- which would have been fitting the code to
    the weaker source. The implementation hits BOTH exact limits to 1e-3, so the
    table row is the number in doubt, not the formula.

    Kept at 5e-3 because catching a gross error is worth having and a 0.3 %
    argument with a transcribed table is not.
    """
    k = hy.lamb_coefficients(fineness, 1.0)
    assert k.k1 == pytest.approx(k1, abs=5e-3)
    assert k.k2 == pytest.approx(k2, abs=5e-3)


def test_the_coefficients_are_monotone_in_fineness():
    """Slenderer means cheaper axially and dearer transversely, always."""
    ks = [hy.lamb_coefficients(f, 1.0) for f in (1.5, 2, 3, 4, 6, 10)]
    assert all(a.k1 > b.k1 for a, b in zip(ks, ks[1:]))
    assert all(a.k2 < b.k2 for a, b in zip(ks, ks[1:]))


def test_an_oblate_body_is_refused():
    """These coefficients describe a PROLATE spheroid. Handing them a body
    wider than it is long returns a number that means nothing, so it raises."""
    with pytest.raises(ValueError):
        hy.lamb_coefficients(0.2, 0.5)


# --------------------------------------------------------------------------- #
#  ⭐ The mass-free result
# --------------------------------------------------------------------------- #

def test_the_effective_mass_ratio_needs_no_mass():
    """The number that survives nearly every unknown: no material, no scale, no
    volume, no density. 1.72x more INERTIA laterally than axially, from the
    fineness ratio alone.

    ⛔ INERTIA ONLY. It does not follow that strafing is worse than turning and
    driving -- this hull has TWO lateral thrusters against ONE axial unit, so
    the heavier axis may also be the better-actuated one. That comparison needs
    the allocator and a drag number."""
    assert hy.effective_mass_ratio() == pytest.approx(1.722, abs=0.01)


def test_this_hull_is_slender_enough_to_care():
    k = hy.hull_added_mass()
    assert k.fineness == pytest.approx(4.03, abs=0.01)
    assert k.k1 < 0.1, 'surge should be nearly free of added mass'
    assert k.k2 > 0.85, 'sway and heave should nearly double the inertia'


def test_no_roll_coefficient_is_offered():
    """⚠ RETRACTED CLAIM, kept as a test so it is not re-argued. This file
    briefly said roll added inertia is "EXACTLY zero, and physical". That is
    true of a SMOOTH body of revolution; ours has four transverse bores, which
    is exactly something for the fluid to grip, so the real value is small but
    non-zero. Offering no coefficient is the honest position -- and it matters
    more here than elsewhere, because roll is the axis this hull cannot
    actuate, so a wrong zero would be a false claim about the least
    controllable degree of freedom."""
    assert not hasattr(hy.hull_added_mass(), 'k_roll')


# --------------------------------------------------------------------------- #
#  The mass claim that `hull_geometry.yaml` said was impossible
# --------------------------------------------------------------------------- #

def test_mass_is_computable_and_comes_out_as_a_band():
    lo, best, hi = hy.mass_band_kg()
    assert 7.0 < lo < best < hi < 15.0
    assert best == pytest.approx(10.2, abs=0.2)


def test_the_free_flooding_tunnels_are_subtracted():
    """⛔ THE ERROR THIS TEST EXISTS FOR, and it shipped for one commit. Four
    84 mm bores pass clean through the hull: the water in them moves WITH the
    vehicle, so it is inside the envelope and outside the displaced volume.
    Omitting them overstated the mass by 23 % -- enough that the published
    LOWER bound (11.1 kg) sat above the corrected best estimate (10.2 kg)."""
    assert hy.tunnel_void_m3() * 1000 == pytest.approx(3.106, abs=0.01)
    gross = hy.enclosing_cylinder_volume_m3() * hy.CP_TORPEDO
    assert hy.displaced_volume_m3() == pytest.approx(gross - hy.tunnel_void_m3())
    assert hy.tunnel_void_m3() / gross > 0.20, 'this is not a rounding error'


def test_the_tunnel_void_matches_the_cad_geometry():
    """Not a hardcoded number: recomputed from the bores in hull_geometry.yaml,
    so a CAD change that moves a tunnel fails here instead of drifting."""
    import math
    from pathlib import Path

    import yaml
    cfg = (Path(__file__).resolve().parents[1] / 'config' / 'hull_geometry.yaml')
    d = yaml.safe_load(cfg.read_text())
    total = 0.0
    for t in d['thrusters']:
        if t.get('bore_mm'):
            r = t['bore_mm'] / 2000.0
            total += math.pi * r * r * (max(t['extent_mm']) / 1000.0)
    assert hy.tunnel_void_m3() == pytest.approx(total, rel=1e-6)


def test_the_band_is_ordered_by_prismatic_coefficient():
    """The spheroid is the lower bound and the bare cylinder the upper; a real
    faired hull is between them and nearer the cylinder."""
    assert hy.CP_SPHEROID < hy.CP_TORPEDO < hy.CP_CYLINDER
    # ⚠ The upper bound is the bare cylinder LESS the tunnels -- displaced
    # volume is never the raw envelope on a hull with through-bores.
    assert (hy.displaced_volume_m3(hy.CP_CYLINDER)
            == pytest.approx(hy.enclosing_cylinder_volume_m3()
                             - hy.tunnel_void_m3()))


def test_sea_water_is_not_fresh_water():
    """2.8 %, which lands on every force claim. A number that does not say
    which water it assumed is incomplete."""
    fresh = hy.neutral_mass_kg(rho=hy.RHO_FRESH)
    sea = hy.neutral_mass_kg(rho=hy.RHO_SEA)
    assert sea > fresh
    assert sea / fresh == pytest.approx(hy.RHO_SEA / hy.RHO_FRESH)


# --------------------------------------------------------------------------- #
#  ⭐ The sim seed is the wrong SHAPE CLASS
# --------------------------------------------------------------------------- #

def test_the_bluerov2_seed_is_refuted_for_this_hull():
    """`sim/.../mongla_heavy/model.sdf` carries BlueROV2 Heavy's added mass,
    and the plan named it a seed "for a bench, never a shipped value". It is
    worse than that: a boxy ROV and a fineness-4 body of revolution are
    different shape classes, and the axial term is off by nearly 6x. Computing
    from our own geometry beats seeding from theirs."""
    k = hy.hull_added_mass()
    bluerov_k1 = 6.356674 / 13.5      # xx / mass
    bluerov_k2 = 18.686327 / 13.5     # zz / mass
    assert bluerov_k1 / k.k1 > 5.0, 'the axial disagreement is the headline'
    assert bluerov_k2 > 1.0, 'their heave added mass EXCEEDS their rigid mass'
    assert k.k2 < 1.0, 'potential flow forbids that for a body of revolution'


def test_uniform_inertia_is_flagged_as_the_weak_one():
    """Sanity only. A real hull is a shell with placed masses, so the uniform
    figure sizes a gain and never claims a moment -- free-decay does that."""
    ixx, iyy = hy.uniform_inertia_per_kg()
    assert ixx < iyy, 'a slender body is far easier to roll than to pitch'
    assert iyy / ixx == pytest.approx(8.6, abs=0.3)


# --------------------------------------------------------------------------- #
#  ⭐ Thruster response -- the term the closed-loop bench found dominates
# --------------------------------------------------------------------------- #

def test_the_duct_term_beats_the_rotor_term():
    """Which term to design against. A thruster makes thrust by throwing water,
    and the water column takes longer to accelerate than the rotor does to spin
    up -- so duct geometry matters more than propeller inertia."""
    rotor = hy.rotor_spinup_s(inertia_kg_m2=5.0e-6, torque_nm=0.104, rpm=3000)
    duct = hy.duct_response_s(bore_mm=84, duct_length_mm=150, thrust_n=20.0)
    assert rotor < 0.030
    assert duct > 3 * rotor


def test_our_tunnels_are_already_far_quicker_than_a_T100():
    """~68-79 ms against the ~0.59 s reported for a T100-class unit. Short
    tunnel thrusters are inherently fast, and we start ahead."""
    r = hy.hull_thruster_response_s(20.0)
    assert 0.06 < r['vertical'] < r['lateral'] < 0.09
    assert r['lateral'] < 0.59 / 5


def test_duct_length_is_the_biggest_lever():
    """tau ~ L exactly. Halving the duct halves the lag -- and it is a CAD
    change, not a motor or ESC change."""
    long_ = hy.duct_response_s(bore_mm=84, duct_length_mm=150, thrust_n=20.0)
    half = hy.duct_response_s(bore_mm=84, duct_length_mm=75, thrust_n=20.0)
    assert half == pytest.approx(long_ / 2.0, rel=1e-9)


def test_thrust_helps_only_as_a_square_root():
    """tau ~ 1/sqrt(T): four times the thrust buys half the lag. Worth knowing
    before anyone proposes a bigger motor to fix response time."""
    a = hy.duct_response_s(bore_mm=84, duct_length_mm=150, thrust_n=10.0)
    b = hy.duct_response_s(bore_mm=84, duct_length_mm=150, thrust_n=40.0)
    assert b == pytest.approx(a / 2.0, rel=1e-9)


def test_a_wider_bore_is_slower():
    """⚠ THE TRADE THAT IS EASY TO GET BACKWARDS. A bigger bore makes more
    thrust for the same disc loading AND responds more slowly, because there is
    more water to move. tau ~ bore."""
    narrow = hy.duct_response_s(bore_mm=60, duct_length_mm=150, thrust_n=20.0)
    wide = hy.duct_response_s(bore_mm=100, duct_length_mm=150, thrust_n=20.0)
    assert wide / narrow == pytest.approx(100.0 / 60.0, rel=1e-9)


def test_zero_thrust_has_no_response_time_and_is_refused():
    with pytest.raises(ValueError):
        hy.duct_response_s(bore_mm=84, duct_length_mm=150, thrust_n=0.0)
