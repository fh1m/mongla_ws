"""A Fossen 6-DOF plant, so the board's control code can be flown off-vehicle.

⛔ WHAT THIS IS FOR. Until now the bench was OPEN LOOP: stick in, DShot out,
nothing fed back. It could say what the board commands and nothing about what
the vehicle then does, so every closed-loop question -- does the feedforward
layer help, does INDI beat the cascade, is 500 Hz justified -- still cost a pool
session. This closes the loop.

    M nu_dot + C(nu) nu + D(nu) nu + g(eta) = tau            Fossen 2011, 7.3

⚠ AND WHAT IT IS NOT. It is not the water. The bench's OTHER half runs the
board's real compiled control code and is validated against live captures to
under one DShot count; NOTHING in this file is validated against anything,
because the vehicle has never been wet. Added mass here is computed from hull
geometry and is the honest part. DRAG IS NOT MEASURED AND HAS NO DEFAULT -- see
`Damping`. A plant result is a statement about a MODEL, and every number this
file produces carries that.

⭐ THE DISCIPLINE THAT MAKES IT USEFUL ANYWAY. An unidentified plant still
answers COMPARATIVE questions, provided the answer survives the uncertainty.
`sweep_drag` exists for exactly that: run the comparison across the whole
plausible drag band and report the conclusion only if it holds throughout. A
conclusion that flips inside the band is not a finding, it is a coin toss with
extra steps.

FRAME. Fossen body-fixed: x forward, y starboard, z DOWN, so depth is +z and
heave is positive downward.
    nu  = [u v w p q r]   body-frame linear and angular velocity
    eta = [x y z phi theta psi]
⚠ The CAD frame is not this frame: there the long axis is Y. The mapping is
CAD +Y -> body +x, and `hull_geometry.yaml` says so at the top. A sign error
here is the kind that looks like a tuning problem for a whole pool day.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

_CONTROL = Path(__file__).resolve().parents[2] / 'src' / 'mongla_control'


def _load_pure(name: str):
    """Import one module out of `mongla_control` WITHOUT importing the package.

    ⚠ A PLAIN IMPORT DOES NOT WORK AND THE REASON IS WORTH RECORDING.
    `mongla_control/__init__.py` imports the whole stack down to
    `mongla_interfaces.action.Move`, so `from mongla_control import
    hydrodynamics` pulls in ROS -- and the bench exists precisely so that
    control questions need no ROS, no vehicle and no built workspace.

    ⭐ Same shape as the finding we sent the firmware team about
    `state_types.h` dragging in FreeRTOS: a pure module made unusable by what
    its neighbours import. Ours is the cheaper fix -- a lazy `__getattr__` in
    `__init__` -- and it should be made.
    """
    import importlib.util
    path = _CONTROL / 'mongla_control' / f'{name}.py'
    spec = importlib.util.spec_from_file_location(f'_bench_{name}', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


hy = _load_pure('hydrodynamics')

G = 9.80665


# ═══════════════════════════════════════════════════════════════════════════ #
#  Configuration -- everything unmeasured is REQUIRED, with no default
# ═══════════════════════════════════════════════════════════════════════════ #

@dataclass(frozen=True)
class Damping:
    """Hydrodynamic damping. ⛔ THERE IS NO DEFAULT AND THERE MUST NOT BE ONE.

    Added mass is potential flow and computable from the hull's shape; drag is
    viscous and no closed form gets it to better than tens of percent. The
    project's recurring defect is "a plausible number standing in for an absent
    measurement", and a drag default would be exactly that -- invisible, in the
    denominator of every settling time the bench reports.

    So you must pass one, and `from_cd` makes the assumption legible by asking
    for a drag COEFFICIENT, which is a thing you can argue about, rather than a
    force, which is a thing you cannot.

    Linear terms are small for a vehicle at these Reynolds numbers and default
    to zero; the quadratic terms carry the physics.
    """
    quad: tuple[float, ...]            # N/(m/s)^2 and N.m/(rad/s)^2, per axis
    lin: tuple[float, ...] = (0.0,) * 6
    provenance: str = ''               # where these came from. Say it.

    def __post_init__(self):
        if len(self.quad) != 6 or len(self.lin) != 6:
            raise ValueError('damping needs all six axes')
        if not self.provenance:
            raise ValueError(
                'Damping.provenance is required: say where these came from '
                '("Cd=0.25 guess", "coast-down 2026-10-xx", "BlueROV2 seed"). '
                'A drag number with no story is the defect this bench exists '
                'to prevent.')

    @staticmethod
    def from_cd(*, cd_axial: float, cd_transverse: float,
                cd_rot: float = 0.6, rho: float = hy.RHO_FRESH,
                provenance: str) -> 'Damping':
        """Quadratic damping from drag coefficients and the hull's own areas.

        ⚠ THE COEFFICIENTS ARE THE GUESS. For a faired body of revolution at
        fineness 4 the published range is roughly:
            axial       0.15 .. 0.30 on frontal area   (streamlined)
            transverse  0.80 .. 1.20 on planform area  (a bluff cylinder)
        Our hull also has four open tunnels, which no smooth-body figure
        covers and which can only raise axial drag. Sweep the band; do not
        trust a point.
        """
        d = (hy.HULL_BEAM_MM + hy.HULL_HEIGHT_MM) / 2000.0
        L = hy.HULL_LENGTH_MM / 1000.0
        a_front = math.pi * (d / 2.0) ** 2
        a_plan = L * d
        ax = 0.5 * rho * cd_axial * a_front
        tr = 0.5 * rho * cd_transverse * a_plan
        # Rotational: integrate the strip drag r^2 |r| over the half-length.
        rot = 0.5 * rho * cd_rot * d * (L / 2.0) ** 4 / 4.0
        return Damping(quad=(ax, tr, tr, rot * 0.1, rot, rot),
                       provenance=provenance)


@dataclass(frozen=True)
class Inertia:
    """Mass and inertia, rigid plus added.

    Defaults come from `hydrodynamics.py`: the mass from the neutral-buoyancy
    argument and the added-mass coefficients from Lamb. ⚠ The rigid INERTIA is
    the weak one -- it assumes a uniform body, and a real hull is a shell with
    its battery placed somewhere deliberate. Free-decay measures the truth.
    """
    mass_kg: float = field(default_factory=lambda: hy.mass_band_kg()[1])
    rho: float = hy.RHO_FRESH
    bg_m: float = 0.010          # CG below CB, metres. The righting moment.

    def displaced_mass_kg(self) -> float:
        return hy.displaced_volume_m3() * self.rho

    def mass_matrix(self) -> tuple[float, ...]:
        """The diagonal of M = M_RB + M_A, in Fossen body axes.

        ⛔ ROLL ADDED INERTIA IS TAKEN AS ZERO HERE AND THAT IS OPTIMISTIC.
        Potential flow gives exactly zero for a smooth body of revolution, but
        this hull has four transverse bores for the fluid to grip, so the true
        value is small and positive. Zero makes roll look MORE agile than it
        is -- and roll is the axis this vehicle cannot actuate, so the error
        runs in the unsafe direction. Stated rather than silently carried.
        """
        k = hy.hull_added_mass()
        m, md = self.mass_kg, self.displaced_mass_kg()
        ixx, iyy = hy.uniform_inertia_per_kg()
        # Added inertia references the DISPLACED fluid's moment, not the body's.
        iyy_d = md * iyy / m if m else 0.0
        return (m * (1 + k.k1),                 # surge
                m * (1 + k.k2),                 # sway
                m * (1 + k.k2),                 # heave
                m * ixx,                        # roll  -- no added term
                m * iyy + k.kp * iyy_d,         # pitch
                m * iyy + k.kp * iyy_d)         # yaw


@dataclass(frozen=True)
class Thruster:
    """One thruster: where it is, which way it pushes, how hard.

    ⛔ `max_thrust_n` IS UNMEASURED ON THIS VEHICLE. `k_n_per_rpm2` is null and
    `MOT_THST_EXPO = 0.65` is fitted to a T200 we are not flying. Every force
    the plant produces scales linearly with it, so an absolute result (a
    settling time in seconds, a distance in metres) is only as good as this
    number -- while a COMPARISON between two controllers on the same plant is
    nearly independent of it. Prefer comparisons.
    """
    name: str
    position_m: tuple[float, float, float]
    axis: tuple[float, float, float]
    max_thrust_n: float


def cad_hull_thrusters(max_thrust_n: float) -> tuple[Thruster, ...]:
    """The five-thruster CAD hull, in FOSSEN body axes.

    ⚠ THE AXIS REMAP IS THE WHOLE POINT OF THIS FUNCTION. `hull_geometry.yaml`
    is in the CAD frame, where the long axis is Y and Z is up-ish. Fossen is
    x-forward, y-starboard, z-DOWN. The mapping applied here is

        CAD +Y -> body +x      (long axis, bow positive)
        CAD +X -> body +y      (lateral bore -> starboard)
        CAD -Z -> body +z      (vertical bore -> DOWN)

    ⛔ THE BOW DIRECTION IS STILL A CONVENTION, NOT A MEASUREMENT.
    `hull_geometry.yaml` says so explicitly: the axial unit sits at CAD
    y = -0.3306 with a duct ring just beyond it at y = -351, which suggests
    that end is the stern. Taken here as bow = CAD +Y. If that is backwards,
    surge reverses -- verify against the vehicle before trusting any sign.
    """
    import yaml
    cfg = _CONTROL / 'config' / 'hull_geometry.yaml'
    spec = yaml.safe_load(cfg.read_text())
    out = []
    for t in spec['thrusters']:
        cx, cy, cz = t['position_m']
        ax, ay, az = t['axis']
        out.append(Thruster(
            name=t['name'],
            position_m=(cy, cx, -cz),
            axis=(ay, ax, -az),
            max_thrust_n=max_thrust_n))
    return tuple(out)


def wrench_matrix(thrusters) -> tuple[tuple[float, ...], ...]:
    """B, 6xN: per-thruster unit force -> body wrench. Rows Fx Fy Fz Mx My Mz."""
    rows = [[0.0] * len(thrusters) for _ in range(6)]
    for j, t in enumerate(thrusters):
        n = math.sqrt(sum(v * v for v in t.axis)) or 1.0
        u = [v / n for v in t.axis]
        p = t.position_m
        m = (p[1] * u[2] - p[2] * u[1],
             p[2] * u[0] - p[0] * u[2],
             p[0] * u[1] - p[1] * u[0])
        for i in range(3):
            rows[i][j] = u[i]
            rows[i + 3][j] = m[i]
    return tuple(tuple(r) for r in rows)


# ═══════════════════════════════════════════════════════════════════════════ #
#  The plant
# ═══════════════════════════════════════════════════════════════════════════ #

class Plant:
    """Fossen 6-DOF rigid body + added mass + damping + restoring."""

    def __init__(self, inertia: Inertia, damping: Damping):
        self.inertia = inertia
        self.damping = damping
        self.M = inertia.mass_matrix()
        self.nu = [0.0] * 6          # u v w p q r
        self.eta = [0.0] * 6         # x y z phi theta psi

    # -- the terms ---------------------------------------------------------- #

    def coriolis(self, nu) -> list:
        """C(nu) nu for a DIAGONAL M.

        The skew-symmetric parameterisation (Fossen 6.43) is LINEAR in the mass
        entries, so C(M_RB + M_A) = C(M_RB) + C(M_A) and one pass over the
        combined diagonal is exact -- not an approximation, given both are
        diagonal.
        """
        u, v, w, p, q, r = nu
        m1, m2, m3, m4, m5, m6 = self.M
        return [m3 * w * q - m2 * v * r,
                m1 * u * r - m3 * w * p,
                m2 * v * p - m1 * u * q,
                (m3 - m2) * v * w + (m6 - m5) * q * r,
                (m1 - m3) * u * w + (m4 - m6) * p * r,
                (m2 - m1) * u * v + (m5 - m4) * p * q]

    def damping_force(self, nu) -> list:
        d, q = self.damping.lin, self.damping.quad
        return [d[i] * nu[i] + q[i] * nu[i] * abs(nu[i]) for i in range(6)]

    def restoring(self, eta) -> list:
        """g(eta) -- the righting moment from CG sitting below CB.

        ⭐ THIS IS WHY ROLL CAN BE LEFT UNACTUATED. With no roll thruster the
        only thing returning the hull to upright is BG, and the bench can now
        say whether it is enough. Assumes neutral buoyancy, so the
        force rows are zero and only the moments survive.
        """
        phi, theta = eta[3], eta[4]
        w = self.inertia.mass_kg * G
        bg = self.inertia.bg_m
        return [0.0, 0.0, 0.0,
                bg * w * math.cos(theta) * math.sin(phi),
                bg * w * math.sin(theta),
                0.0]

    def derivative(self, nu, eta, tau) -> tuple[list, list]:
        c = self.coriolis(nu)
        d = self.damping_force(nu)
        g = self.restoring(eta)
        nu_dot = [(tau[i] - c[i] - d[i] - g[i]) / self.M[i] for i in range(6)]
        return nu_dot, self._eta_dot(nu, eta)

    @staticmethod
    def _eta_dot(nu, eta) -> list:
        """Body rates -> world rates. Rotation then the Euler-angle Jacobian."""
        u, v, w, p, q, r = nu
        phi, th, psi = eta[3], eta[4], eta[5]
        cf, sf, ct, st, cp, sp = (math.cos(phi), math.sin(phi), math.cos(th),
                                  math.sin(th), math.cos(psi), math.sin(psi))
        # ⚠ A pitch of exactly +-90 deg makes the Euler Jacobian singular. This
        # hull cannot pitch there under its own restoring moment, and guarding
        # would hide a plant that had gone somewhere it should not -- so it is
        # left to blow up loudly instead of being silently clamped.
        tt = math.tan(th)
        return [cp * ct * u + (cp * st * sf - sp * cf) * v + (cp * st * cf + sp * sf) * w,
                sp * ct * u + (sp * st * sf + cp * cf) * v + (sp * st * cf - cp * sf) * w,
                -st * u + ct * sf * v + ct * cf * w,
                p + sf * tt * q + cf * tt * r,
                cf * q - sf * r,
                (sf / ct) * q + (cf / ct) * r]

    # -- integration -------------------------------------------------------- #

    def step(self, tau, dt: float) -> None:
        """One RK4 step. `tau` is the body wrench [Fx Fy Fz Mx My Mz].

        RK4 rather than Euler because the bench's whole value is that a bench
        result is a statement about the CONTROL CODE. Euler at 2 ms adds its own
        phase lag, and a controller comparison would then partly measure the
        integrator instead of the controllers.
        """
        n0, e0 = list(self.nu), list(self.eta)

        def f(n, e):
            return self.derivative(n, e, tau)

        k1n, k1e = f(n0, e0)
        k2n, k2e = f([n0[i] + 0.5 * dt * k1n[i] for i in range(6)],
                     [e0[i] + 0.5 * dt * k1e[i] for i in range(6)])
        k3n, k3e = f([n0[i] + 0.5 * dt * k2n[i] for i in range(6)],
                     [e0[i] + 0.5 * dt * k2e[i] for i in range(6)])
        k4n, k4e = f([n0[i] + dt * k3n[i] for i in range(6)],
                     [e0[i] + dt * k3e[i] for i in range(6)])
        for i in range(6):
            self.nu[i] = n0[i] + dt / 6.0 * (k1n[i] + 2 * k2n[i] + 2 * k3n[i] + k4n[i])
            self.eta[i] = e0[i] + dt / 6.0 * (k1e[i] + 2 * k2e[i] + 2 * k3e[i] + k4e[i])

    # -- readouts the control code wants ------------------------------------ #

    @property
    def body_rates(self) -> tuple[float, float, float]:
        """(gx, gy, gz) rad/s -- what the board's gyro would report."""
        return (self.nu[3], self.nu[4], self.nu[5])

    @property
    def attitude(self) -> tuple[float, float, float]:
        """(roll, pitch, yaw) rad -- what the board's AHRS would report."""
        return (self.eta[3], self.eta[4], self.eta[5])

    @property
    def depth_m(self) -> float:
        """⚠ NEGATIVE BELOW THE SURFACE, matching what the board reports over
        `VFR_HUD`. Fossen's z is positive DOWN, so this is a deliberate sign
        flip and not a bug -- see CLAUDE.md section 2."""
        return -self.eta[2]
