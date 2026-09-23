"""What the hull's SHAPE implies, before anybody weighs anything.

⛔ THE CLAIM THIS FILE EXISTS TO RETRACT. `hull_geometry.yaml` said:

    mass_kg: null   # Onshape reports NO MATERIAL on any part, so mass cannot
                    # be computed. A scale closes it. Every force claim waits here.

That is false, and it blocked the entire force domain for no reason. Two facts:

  1. **Volume is pure geometry.** A material assignment converts volume to mass;
     it is not needed to have a volume. The CAD envelope alone bounds it.
  2. **A neutrally buoyant vehicle's mass EQUALS its displaced water mass.**
     m = rho * V. That is not a measurement waiting on a scale -- it is a design
     constraint. An AUV that is not near-neutral spends vertical thrust holding
     depth, and ours has two vertical thrusters, not a ballast system.

So mass is computable today, to a band, and the band is narrow enough to design
against. A scale still collapses it, and it should still be run.

⭐ AND THE BEST RESULT HERE NEEDS NO VOLUME AT ALL. Lamb's added-mass
coefficients are functions of the FINENESS RATIO alone. They are dimensionless,
mass-free, volume-free, and exact for a body of revolution -- which our hull is,
to within the 4 mm difference between its 176.1 mm and 172.1 mm cross-section.
Everything in `LAMB` below is knowable with a tape measure.

WHAT THIS IS NOT. It is not a substitute for Round 4's in-water identification.
Added mass is potential flow -- it is the honest part. DRAG is viscous and no
closed form predicts it to better than tens of percent; `coast_down` is what
measures that. Nothing here produces a drag coefficient.

Sources: H. Lamb, *Hydrodynamics*, 6th ed. 1932, art. 114 (the ellipsoid
coefficients); T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion
Control*, 2011, sec. 6.3 (their use as M_A).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# ── the hull, as measured off the CAD ────────────────────────────────────────
# 2026-09-20, validated against Onshape to 0.08 %. Millimetres.
HULL_LENGTH_MM = 702.0
HULL_BEAM_MM = 176.1          # the X cross-section
HULL_HEIGHT_MM = 172.1        # the Z cross-section

# ⚠ WATER IS NOT ONE DENSITY, and the difference is not negligible: 2.8 %
# between a fresh pool and the sea, which lands directly on every force claim.
# The competition pool decides which one applies, and a claim that does not say
# which it used is incomplete.
RHO_FRESH = 997.0             # kg/m^3, 25 C
RHO_SEA = 1025.0              # kg/m^3, nominal

# Prismatic coefficient: displaced volume as a fraction of the enclosing
# cylinder. It is the ONE shape number the envelope cannot give us.
#   2/3   a pure prolate spheroid -- the LOWER bound for a faired body
#   0.80  a torpedo with a parallel mid-body, which is what the CAD looks like
#   1.00  a bare cylinder -- the upper bound, and not a real hull
CP_SPHEROID = 2.0 / 3.0
CP_TORPEDO = 0.80
CP_CYLINDER = 1.0

# ⛔ THE TUNNELS DO NOT DISPLACE, and forgetting that is a 23 % error.
# Four through-bores pass clean through the hull: water sits in them and moves
# with the vehicle, so they are inside the envelope and OUTSIDE the displaced
# volume. The first version of this module omitted them and published a mass
# band whose LOWER bound was above the corrected best estimate.
#
# Read from `config/hull_geometry.yaml`, which is the CAD: two lateral bores
# 150 mm long and two vertical bores 130.2 mm long, all 84 mm.
TUNNEL_BORE_MM = 84.0
TUNNEL_LENGTHS_MM = (150.0, 150.0, 130.2, 130.2)


@dataclass(frozen=True)
class AddedMass:
    """Lamb's inertia coefficients. Dimensionless; multiply by displaced mass.

    `k1` acts along the long axis, `k2` on both transverse axes, and `kp`
    (Lamb's k') on rotation about a transverse axis -- pitch and yaw.

    ⚠ THESE DESCRIBE A SMOOTH SEALED SPHEROID, WHICH OUR HULL IS NOT. Four
    84 mm bores pass through it. They break the axisymmetry the formulas assume
    and they let fluid through the body, so on the real hull k2 and k' are
    over-estimates of the potential-flow part and the tunnels add an entrained
    term the formulas know nothing about. Treat every coefficient here as the
    SHAPE CONTRIBUTION of the faired envelope, good to maybe 10-20 % on the
    transverse axes, and let Round 4's free-decay measure the truth.

    ⛔ THERE IS NO ROLL TERM, and that is a LIMIT OF THE MODEL, not a claim
    about the vehicle. For a smooth body of revolution spun about its own axis
    the potential-flow added inertia is exactly zero -- there is nothing for the
    fluid to grip. Our hull has four transverse bores, which is precisely
    something to grip, so its real roll added inertia is small but NOT zero.
    Since roll is also the axis this hull cannot actuate, a wrong "exactly
    zero" here would be a claim about the least controllable axis, and it is
    withdrawn rather than repeated.
    """
    k1: float
    k2: float
    kp: float
    fineness: float
    eccentricity: float


def lamb_coefficients(length_m: float, diameter_m: float) -> AddedMass:
    """Added-mass coefficients of a prolate spheroid, from Lamb art. 114.

    Pure geometry: no mass, no density, no material. `diameter_m` is the
    cross-section of the equivalent body of revolution.

    The integrals Lamb calls alpha_0 and beta_0 are

        alpha_0 = (2(1-e^2)/e^3) (1/2 ln((1+e)/(1-e)) - e)
        beta_0  = 1/e^2 - ((1-e^2)/(2 e^3)) ln((1+e)/(1-e))

    and the coefficients follow as k = X/(2-X).
    """
    a = length_m / 2.0
    b = diameter_m / 2.0
    if not (a > b > 0.0):
        raise ValueError(
            f'need a prolate body: half-length {a} must exceed half-diameter '
            f'{b}. A hull that is wider than it is long is not what these '
            f'coefficients describe.')
    e = math.sqrt(1.0 - (b / a) ** 2)
    ln = math.log((1.0 + e) / (1.0 - e))
    alpha0 = (2.0 * (1.0 - e ** 2) / e ** 3) * (0.5 * ln - e)
    beta0 = 1.0 / e ** 2 - ((1.0 - e ** 2) / (2.0 * e ** 3)) * ln
    kp = ((e ** 4 * (beta0 - alpha0))
          / ((2.0 - e ** 2) * (2.0 * e ** 2 - (2.0 - e ** 2) * (beta0 - alpha0))))
    return AddedMass(k1=alpha0 / (2.0 - alpha0),
                     k2=beta0 / (2.0 - beta0),
                     kp=kp, fineness=a / b, eccentricity=e)


def hull_added_mass() -> AddedMass:
    """This vehicle's coefficients, from the CAD envelope.

    ⚠ The cross-section is 176.1 x 172.1 mm -- not quite round. The mean is
    used, and the 4 mm of asymmetry is below the precision of treating a faired
    hull as a spheroid at all.
    """
    return lamb_coefficients(HULL_LENGTH_MM / 1000.0,
                             (HULL_BEAM_MM + HULL_HEIGHT_MM) / 2000.0)


def enclosing_cylinder_volume_m3() -> float:
    """The envelope's circumscribing cylinder -- the volume upper bound."""
    r = (HULL_BEAM_MM + HULL_HEIGHT_MM) / 4000.0
    return math.pi * r * r * (HULL_LENGTH_MM / 1000.0)


def tunnel_void_m3() -> float:
    """Volume of the four free-flooding thruster tunnels. 3.106 L.

    ⚠ This is water that travels WITH the vehicle. It is not displaced, so it
    subtracts from buoyancy -- and it is entrained, so it adds to the mass that
    has to be accelerated. Those are two separate effects of the same 3.1 L and
    only the first one is modelled here.
    """
    r = TUNNEL_BORE_MM / 2000.0
    return sum(math.pi * r * r * (L / 1000.0) for L in TUNNEL_LENGTHS_MM)


def displaced_volume_m3(cp: float = CP_TORPEDO) -> float:
    """Displaced volume: the faired envelope MINUS the free-flooding tunnels."""
    return enclosing_cylinder_volume_m3() * cp - tunnel_void_m3()


def neutral_mass_kg(cp: float = CP_TORPEDO, rho: float = RHO_FRESH) -> float:
    """⭐ The vehicle's DRY mass, ASSUMING it is ballasted neutral.

    ⛔ THE ASSUMPTION IS THE WHOLE CONTENT, so state it wherever this is
    quoted. If the hull floats or sinks, this is the displaced mass and not the
    vehicle's, and the two differ by exactly the net buoyancy -- which is the
    quantity a pool test measures directly and a scale does not.
    """
    return displaced_volume_m3(cp) * rho


def mass_band_kg(rho: float = RHO_FRESH) -> tuple[float, float, float]:
    """(lower, best, upper) mass in kg, across the prismatic-coefficient band.

    Returned as a BAND rather than a number because the prismatic coefficient
    is the one input the envelope cannot supply. Quote the band, not the middle.
    """
    return (neutral_mass_kg(CP_SPHEROID, rho),
            neutral_mass_kg(CP_TORPEDO, rho),
            neutral_mass_kg(CP_CYLINDER, rho))


def effective_mass_ratio() -> float:
    """⭐ How much more INERTIA this hull has laterally than axially. Mass-free.

    Survives nearly every unknown -- no mass, no volume, no material, no
    density -- because it is (1+k2)/(1+k1) and falls straight out of the
    fineness ratio. Subject to the tunnel caveat in `AddedMass`.

    ⛔ IT IS A STATEMENT ABOUT INERTIA AND NOTHING ELSE. It does NOT follow
    that strafing is worse than turning and driving, and an earlier version of
    this file said exactly that. Which manoeuvre wins depends on the THRUST
    available per axis and on drag, and this hull has TWO lateral thrusters
    against ONE axial unit -- so the axis with more inertia may also be the axis
    with more force. Settle that with the geometric allocator and a drag
    measurement, not with this number.
    """
    k = hull_added_mass()
    return (1.0 + k.k2) / (1.0 + k.k1)


def uniform_inertia_per_kg() -> tuple[float, float]:
    """(Ixx, Iyy=Izz) per kg for a UNIFORM spheroid of this shape, m^2.

    ⚠ UNIFORM IS WRONG AND THIS IS THE WEAKEST FUNCTION HERE. A real hull is a
    shell with its heavy parts (battery, pressure housing) placed deliberately,
    so the true inertia is not the uniform figure and can differ substantially.
    Use this to size a gain, never to claim a moment. The honest instrument is
    free-decay rotation in water -- Round 4a.
    """
    a = HULL_LENGTH_MM / 2000.0
    b = (HULL_BEAM_MM + HULL_HEIGHT_MM) / 4000.0
    return (0.4 * b * b, 0.2 * (a * a + b * b))


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⭐ Thruster response time -- the term that dominates everything else
# ═══════════════════════════════════════════════════════════════════════════ #
#
# Measured on the closed-loop bench 2026-09-23: against a yaw torque burst,
# first-order thruster lag costs ONE TO TWO ORDERS OF MAGNITUDE more than
# control loop rate. tau 0 -> 0.59 s made the peak deviation 131x worse, while
# dropping the loop from 500 Hz to 50 Hz cost 21 %.
#
# So for a CUSTOM thruster, response time is the property to design for, and
# these are the two terms that set it.


def rotor_spinup_s(*, inertia_kg_m2: float, torque_nm: float,
                   rpm: float) -> float:
    """Time for the rotor and propeller to reach `rpm`. tau = J w / Q.

    Small: of order 15 ms for a 2212-class bell with a light resin propeller.
    It is NOT the dominant term -- see `duct_response_s` -- but it is the one a
    heavy propeller makes worse, and it scales linearly with inertia.
    """
    if torque_nm <= 0.0:
        raise ValueError('a rotor with no torque never spins up')
    return inertia_kg_m2 * (rpm * 2.0 * math.pi / 60.0) / torque_nm


def duct_response_s(*, bore_mm: float, duct_length_mm: float,
                    thrust_n: float, rho: float = RHO_FRESH) -> float:
    """⭐ THE DOMINANT TERM, and it is pure geometry.

        tau = L * sqrt(rho * A / T)

    A thruster does not make thrust by spinning -- it makes thrust by throwing
    water backwards, and the water in the duct has to be accelerated first.
    From momentum theory the slipstream reaches v = sqrt(T / (rho A)), and the
    entrained column rho*A*L must be brought to it, which gives the expression
    above.

    ⭐ THE THREE DESIGN LEVERS FALL STRAIGHT OUT OF IT:

        tau ~ L           halve the duct, halve the lag. THE BIGGEST ONE.
        tau ~ 1/sqrt(T)   four times the thrust, half the lag
        tau ~ bore        a narrower bore is faster, but makes less thrust

    For this hull's CAD tunnels at 20 N: 79 ms on the 150 mm lateral bores and
    68 ms on the 130 mm vertical ones. That is already SEVEN TIMES faster than
    the ~0.59 s reported for a T100-class unit, before anybody designs
    anything -- short tunnel thrusters are inherently quick.

    ⚠ FIRST-ORDER AND IDEAL. Momentum theory ignores duct wall friction, the
    inlet contraction, and the fact that a stopped propeller has to break away
    before any of this starts. Treat it as the term that RANKS design choices,
    not as a number to quote. A step-response test on the real unit settles it.
    """
    area = math.pi * (bore_mm / 2000.0) ** 2
    if thrust_n <= 0.0:
        raise ValueError('thrust must be positive to define a response time')
    return (duct_length_mm / 1000.0) * math.sqrt(rho * area / thrust_n)


def hull_thruster_response_s(thrust_n: float = 20.0) -> dict:
    """This hull's four tunnels, from `hull_geometry.yaml`'s own bores."""
    return {
        'lateral': duct_response_s(bore_mm=TUNNEL_BORE_MM,
                                   duct_length_mm=TUNNEL_LENGTHS_MM[0],
                                   thrust_n=thrust_n),
        'vertical': duct_response_s(bore_mm=TUNNEL_BORE_MM,
                                    duct_length_mm=TUNNEL_LENGTHS_MM[2],
                                    thrust_n=thrust_n),
    }
