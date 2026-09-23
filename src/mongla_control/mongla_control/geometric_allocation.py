"""Allocate a wrench to thrusters using the hull's real geometry.

⛔ WHY THIS EXISTS, AND WHY IT IS NOT `allocation.py`. That module mirrors the
BOARD's mixer -- an 8x6 matrix of +-1 for a vectored-6DOF frame -- to predict
what the firmware will do to a demand we already composed. This module does the
opposite job for a different vehicle: given a wrench we WANT, it solves for the
thruster commands that produce it, on the five-thruster hull now in fabrication.

The two describe different vehicles and both are correct about theirs. Keeping
them apart is deliberate; merging them would produce one matrix that is wrong
about both.

WHAT A +-1 MIXER COSTS. Used as a force sum, +-1 entries overstate surge and
sway by 1/cos(45 deg) = 41 %, they carry no moment arms at all, and they cannot
express a dead thruster. Here the moment arms are measured: 0.1750 m for yaw,
0.2595 m for pitch, read off the CAD.

⚠ THE UNITS ARE NOT NEWTONS, AND THIS MODULE WILL NOT PRETEND OTHERWISE.
`thrust` is a signed fraction of one thruster's full output, -1..1. Converting
that to force needs `k_n_per_rpm2`, which has no value anywhere in this tree on
purpose, and predicting motion needs a mass the CAD cannot give (no material is
assigned to any part). So the wrench rows are "force-like" and "moment-like" in
consistent units, exact in DIRECTION and RATIO, and silent about magnitude.

⚠ THE FRAME IS THE CAD'S, AND ITS SIGNS ARE NOT SETTLED. Y is the long axis, X
the lateral tunnels' bore, Z the vertical tunnels'. Which end of Y is the bow,
and whether the axial unit is a tractor or a pusher, are open questions
(`hull_geometry.yaml`). Magnitudes and moment arms below are measured; a sign
convention is a decision, and this module does not invent one.

⛔ ROLL IS UNACTUATED AND A ROLL DEMAND IS REFUSED, NOT ROUNDED TO ZERO. Every
thruster's line of action passes through the roll axis, so no combination of
them produces roll -- it is a property of the geometry, not a tuning limit.
Silently dropping a roll request would let a caller believe the vehicle is
holding an attitude it cannot hold.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

# ⚠ ONE TRUTH, TWO COPIES -- handled the way `MIXER` handles it. These mirror
# `config/hull_geometry.yaml`, which was read off the Onshape model on
# 2026-09-23. `mongla_control` deliberately depends on nothing but pyserial, so
# the file is not parsed at runtime; `test_geometric_allocation.py` asserts the
# two agree instead of trusting them to.
#
# ⚠ AND THE CAD IS A MOVING DESIGN: custom thrusters are being designed and not
# all are fitted. Re-run `tools/pull_hull_geometry.js` when it changes.
#
#   name          position (m, CAD frame)        axis
THRUSTERS: Tuple[Tuple[str, Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ('lateral_a',  (0.0, -0.1750, 0.0),    (1.0, 0.0, 0.0)),
    ('lateral_b',  (0.0,  0.1750, 0.0),    (1.0, 0.0, 0.0)),
    ('vertical_a', (0.0, -0.2595, 0.0),    (0.0, 0.0, 1.0)),
    ('vertical_b', (0.0,  0.2595, 0.0),    (0.0, 0.0, 1.0)),
    ('axial',      (0.0, -0.3306, 0.0081), (0.0, 1.0, 0.0)),
)

# Wrench rows, in the CAD frame. Named for what they physically are so a
# diagnostic can say WHICH authority ran out rather than printing an index.
AXES: Tuple[str, ...] = ('sway', 'surge', 'heave', 'pitch', 'roll', 'yaw')
#                          Fx      Fy       Fz      Mx      My     Mz

# An axis is unactuated when no thruster contributes to it at all. Computed,
# never declared: if a thruster moves, this follows it.
_UNACTUATED_TOL = 1e-6


def _cross(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def build_b(thrusters=THRUSTERS) -> Tuple[Tuple[float, ...], ...]:
    """The 6xN allocation matrix: force rows then moment rows.

    Column j is what thruster j produces at unit output -- its axis as a force,
    and `r x axis` as a moment. That is the whole model, and it is why the
    moment arms are real distances instead of signs.
    """
    rows = [[0.0] * len(thrusters) for _ in range(6)]
    for j, (_name, pos, axis) in enumerate(thrusters):
        n = (axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) ** 0.5
        unit = tuple(c / n for c in axis)
        moment = _cross(pos, unit)
        for i in range(3):
            rows[i][j] = unit[i]
            rows[i + 3][j] = moment[i]
    return tuple(tuple(r) for r in rows)


B = build_b()


def unactuated_axes(b=B) -> Tuple[str, ...]:
    """Which wrench axes this geometry cannot produce, at all."""
    return tuple(AXES[i] for i in range(6)
                 if max((abs(v) for v in b[i]), default=0.0) <= _UNACTUATED_TOL)


UNACTUATED = unactuated_axes()


# ── the solve ────────────────────────────────────────────────────────────────

def _solve(a: list, rhs: list) -> list:
    """Gaussian elimination with partial pivoting. Raises on a singular system.

    Small and explicit on purpose: `mongla_control` depends on pyserial and
    nothing else, and a 5x5 solve does not justify pulling numpy into a package
    the flight path imports.
    """
    n = len(a)
    m = [list(row) + [rhs[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError('singular normal matrix: two thrusters are '
                             'collinear, or a column is all zero')
        m[col], m[piv] = m[piv], m[col]
        for r in range(n):
            if r == col:
                continue
            f = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= f * m[col][c]
    return [m[i][n] / m[i][i] for i in range(n)]


@dataclass(frozen=True)
class Allocation:
    """What the allocator decided, and what it could not do.

    `residual` is the part of the request the geometry cannot produce. It is
    reported rather than hidden because an unachievable component is exactly
    what a controller must know: integrating against it is windup with no
    actuator behind it.
    """
    thrusts: Tuple[float, ...]
    achieved: Tuple[float, ...]
    residual: Tuple[float, ...]
    scale: float                       # 1.0 = nothing was scaled back
    refused: Tuple[str, ...]           # axes asked for that cannot be produced
    disabled: Tuple[str, ...]

    @property
    def saturated(self) -> bool:
        return self.scale < 1.0

    @property
    def names(self) -> Tuple[str, ...]:
        return tuple(t[0] for t in THRUSTERS)


class GeometricAllocator:
    """Least-squares allocation over the measured geometry.

    A dead thruster is a LIMIT, not a rewrite: `disable('lateral_a')` removes
    its column and the remaining four re-solve. A +-1 mixer cannot express that
    at all, which is the single strongest argument in the literature for
    allocating over geometry.
    """

    def __init__(self, thrusters=THRUSTERS, disabled: Sequence[str] = ()):
        self._thrusters = tuple(thrusters)
        self._disabled = tuple(disabled)
        self._rebuild()

    # ---- configuration --------------------------------------------------- #

    def _rebuild(self) -> None:
        names = [t[0] for t in self._thrusters]
        for d in self._disabled:
            if d not in names:
                raise KeyError(f'no thruster named {d!r}; have {names}')
        self._live = tuple(i for i, t in enumerate(self._thrusters)
                           if t[0] not in self._disabled)
        live = [self._thrusters[i] for i in self._live]
        self._b = build_b(live) if live else tuple(() for _ in range(6))
        self._unactuated = unactuated_axes(self._b)

    def disable(self, *names: str) -> 'GeometricAllocator':
        """Mark thrusters dead. Returns self so it reads as one statement."""
        self._disabled = tuple(dict.fromkeys(self._disabled + names))
        self._rebuild()
        return self

    def enable_all(self) -> 'GeometricAllocator':
        self._disabled = ()
        self._rebuild()
        return self

    @property
    def b(self):
        return self._b

    @property
    def unactuated(self) -> Tuple[str, ...]:
        """Axes this geometry cannot produce IN ITS CURRENT STATE -- so killing
        a thruster can add to this list, which is the point of reporting it."""
        return self._unactuated

    # ---- the allocation -------------------------------------------------- #

    def allocate(self, *, sway=0.0, surge=0.0, heave=0.0,
                 pitch=0.0, roll=0.0, yaw=0.0,
                 limit: float = 1.0) -> Allocation:
        """Solve for thruster outputs producing the requested wrench.

        ⛔ AN UNACTUATED AXIS IS REFUSED, NOT SILENTLY DROPPED. Asking for roll
        on this hull is not a small request -- it is an impossible one, and a
        caller that cannot tell the difference will integrate against it
        forever.

        On saturation the whole solution is scaled back uniformly, preserving
        the wrench's DIRECTION and losing only its magnitude. That is the same
        choice the firmware makes per group, and for the same reason: a clipped
        mix points somewhere nobody asked for.
        """
        want = [sway, surge, heave, pitch, roll, yaw]

        refused = tuple(AXES[i] for i in range(6)
                        if AXES[i] in self._unactuated and abs(want[i]) > 1e-9)

        n = len(self._live)
        if n == 0:
            zeros = (0.0,) * 6
            return Allocation((0.0,) * len(self._thrusters), zeros, tuple(want),
                              1.0, refused, self._disabled)

        # Least squares over the ACHIEVABLE subspace. B is 6xN with full column
        # rank for this hull, so (B^T B) is NxN and invertible, and the normal
        # equations give the minimum-error solution directly. The unachievable
        # part of the request does not enter the solve -- it comes back as the
        # residual, which is exactly where a controller should see it.
        bt_b = [[sum(self._b[k][i] * self._b[k][j] for k in range(6))
                 for j in range(n)] for i in range(n)]
        bt_w = [sum(self._b[k][i] * want[k] for k in range(6)) for i in range(n)]
        try:
            u = _solve(bt_b, bt_w)
        except ValueError:
            # A degenerate geometry is a refusal, not a guess.
            zeros = (0.0,) * 6
            return Allocation((0.0,) * len(self._thrusters), zeros, tuple(want),
                              1.0, refused + ('degenerate',), self._disabled)

        peak = max((abs(v) for v in u), default=0.0)
        scale = min(1.0, limit / peak) if peak > limit else 1.0
        u = [v * scale for v in u]

        full = [0.0] * len(self._thrusters)
        for slot, idx in enumerate(self._live):
            full[idx] = u[slot]

        achieved = tuple(sum(self._b[i][j] * u[j] for j in range(n))
                         for i in range(6))
        residual = tuple(want[i] - achieved[i] for i in range(6))
        return Allocation(tuple(full), achieved, residual, scale, refused,
                          self._disabled)
