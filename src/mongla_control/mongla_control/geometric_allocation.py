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
    # ⛔ z IS THE PROP HUB, NOT THE MOTOR (issue #9). The A2212's bounding box
    # centres 8.06 mm off the axis because its mount bracket pulls the box
    # sideways; the prop hub in the same export sits at +0.12 mm and the duct
    # ring at 0.00. A thruster's position is its THRUST LINE. The 8.06 mm
    # invented a surge->pitch moment the allocator then spent the vertical pair
    # cancelling.
    ('axial',      (0.0, -0.3306, 0.00012), (0.0, 1.0, 0.0)),
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


# Moments are taken about this point, in the CAD frame.
#
# ⛔ IT IS THE BOUNDING-BOX CENTRE BECAUSE THE CENTRE OF MASS IS UNKNOWN, NOT
# BECAUSE THE TWO ARE THE SAME (issue #9). Onshape has no material on any part,
# so `hull_geometry.yaml` holds `unknown.centre_of_mass_m: null`, and a test
# keeps this constant at the origin until that entry gets a value -- then
# requires the two to agree. What it moves when set: a CoM `h` below the
# thrust plane makes the roll row `h` times the sway row, so pure sway rolls
# the hull and roll stops being cleanly unactuated; and every arm shifts by
# the offset. Measure it (a float test and a tilt test), do not estimate it.
REFERENCE_M: Tuple[float, float, float] = (0.0, 0.0, 0.0)


def build_b(thrusters=THRUSTERS, reference=REFERENCE_M) -> Tuple[Tuple[float, ...], ...]:
    """The 6xN allocation matrix: force rows then moment rows.

    Column j is what thruster j produces at unit output -- its axis as a force,
    and `(r - reference) x axis` as a moment. That is the whole model, and it
    is why the moment arms are real distances instead of signs.
    """
    rows = [[0.0] * len(thrusters) for _ in range(6)]
    for j, (_name, pos, axis) in enumerate(thrusters):
        n = (axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) ** 0.5
        unit = tuple(c / n for c in axis)
        moment = _cross(tuple(p - c for p, c in zip(pos, reference)), unit)
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


# ── who wins when the thrusters run out ─────────────────────────────────────
#
# ⛔ ONE LAW, TWO VOCABULARIES. The ORDER is `allocation.HORIZONTAL_PRIORITY`
# and `allocation.VERTICAL_PRIORITY` -- yaw before lateral before forward, and
# attitude before throttle -- renamed into this module's axes. It is restated
# rather than imported because `tools/control_bench` loads this file without
# the package (importing `mongla_control` pulls in ROS), and
# `test_geometric_allocation.py` fails the moment the two disagree.
#
# Why yaw first is written beside the board's table: heading error points the
# tool at the wrong place; a slightly slow approach does not.
BOARD_AXIS = {'yaw': 'yaw', 'sway': 'lateral', 'surge': 'forward',
              'roll': 'roll', 'pitch': 'pitch', 'heave': 'throttle'}
HORIZONTAL_PRIORITY: Tuple[str, ...] = ('yaw', 'sway', 'surge')
VERTICAL_PRIORITY: Tuple[str, ...] = ('roll', 'pitch', 'heave')


def priority_order(unactuated: Sequence[str] = UNACTUATED) -> Tuple[str, ...]:
    """The order axes are granted authority in, for one geometry.

    An unactuated axis is dropped BEFORE ranking (issue #9): on the tunnel hull
    roll heads the vertical ladder, and leaving it there would make it a rank
    nothing can fill. The two ladders are then interleaved by rank -- yaw and
    pitch, then sway and heave, then surge -- because they share thrusters
    wherever the geometry couples them (the axial unit's offset puts surge into
    pitch), and running one ladder to completion first would let ITS last axis
    outrank the other's first.
    """
    ladders = [[a for a in ladder if a not in unactuated]
               for ladder in (HORIZONTAL_PRIORITY, VERTICAL_PRIORITY)]
    order = []
    for rank in range(max(len(l) for l in ladders)):
        order += [l[rank] for l in ladders if rank < len(l)]
    return tuple(order)


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
    clamped: Tuple[str, ...] = ()      # thrusters pinned at a limit

    @property
    def saturated(self) -> bool:
        return self.scale < 1.0 or bool(self.clamped)

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

    def _least_squares(self, want, cols):
        """Minimum-error thruster outputs over the columns in `cols`."""
        n = len(cols)
        bt_b = [[sum(self._b[k][cols[i]] * self._b[k][cols[j]] for k in range(6))
                 for j in range(n)] for i in range(n)]
        bt_w = [sum(self._b[k][cols[i]] * want[k] for k in range(6))
                for i in range(n)]
        return _solve(bt_b, bt_w)

    def allocate(self, *, sway=0.0, surge=0.0, heave=0.0,
                 pitch=0.0, roll=0.0, yaw=0.0,
                 limit: float = 1.0, prioritise: bool = True) -> Allocation:
        """Solve for thruster outputs producing the requested wrench.

        ⛔ AN UNACTUATED AXIS IS REFUSED, NOT SILENTLY DROPPED. Asking for roll
        on this hull is not a small request -- it is an impossible one, and a
        caller that cannot tell the difference will integrate against it
        forever.

        On saturation the axes are granted authority in `priority_order` --
        yaw first -- so the axis that comes up short is the one the stack
        ranked last, and `residual` says by how much. `prioritise=False` scales
        the whole solution uniformly instead, keeping the wrench's direction
        and losing magnitude on every axis alike; it is kept as the baseline
        the priority fit is measured against.
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
        try:
            u = self._least_squares(want, list(range(n)))
        except ValueError:
            # A degenerate geometry is a refusal, not a guess.
            zeros = (0.0,) * 6
            return Allocation((0.0,) * len(self._thrusters), zeros, tuple(want),
                              1.0, refused + ('degenerate',), self._disabled)

        scale = 1.0

        if max((abs(v) for v in u), default=0.0) > limit:
            if prioritise:
                u = self._prioritise(want, limit)
            else:
                # Uniform scale: keeps the wrench's DIRECTION and loses its
                # magnitude on every axis alike.
                scale = limit / max(abs(v) for v in u)
                u = [v * scale for v in u]

        full = [0.0] * len(self._thrusters)
        for slot, idx in enumerate(self._live):
            full[idx] = u[slot]

        achieved = tuple(sum(self._b[i][j] * u[j] for j in range(n))
                         for i in range(6))
        residual = tuple(want[i] - achieved[i] for i in range(6))
        clamped = tuple(self._thrusters[self._live[s]][0] for s in range(n)
                        if abs(u[s]) >= limit - 1e-9)
        return Allocation(tuple(full), achieved, residual, scale, refused,
                          self._disabled, clamped)

    # ---- priority ------------------------------------------------------ #

    def _prioritise(self, want, limit):
        """Grant each axis the most it can have, in `priority_order`.

        ⛔ WHY NOT LEAST SQUARES OVER THE SATURATED SET (issue #44). The first
        version pinned the thrusters that ran out and re-solved the rest for
        minimum TOTAL error. Total error over these rows adds a force-like
        residual (fraction of one thruster) to a moment-like one (fraction x
        metres), so with a 0.175 m yaw arm a unit of yaw error cost 1/33 of a
        unit of sway error -- and the solver kept sway and dropped heading:
        sway 1.9 + yaw 0.2 delivered 11 % of the yaw, worse than the uniform
        scale it was written to beat. There is no weighting that is "right" in
        mixed units; there is only an ORDER, and the order is a control
        decision already made in `allocation.py`.

        ⭐ AND ON THIS HULL NOTHING IS LOST BY IT. With all five live, B over the
        five actuated axes is square and invertible: there is no null space, so
        a saturating demand can only choose WHICH axis comes up short. This
        makes that choice explicitly.

        The solve is linear, so each axis's own thruster vector is fixed and
        granting it a fraction `lam` of its request is a 1-D bound on every
        thruster: the largest `lam` in [0, 1] keeping all of them inside the
        limit, with what higher axes were granted already in place. That is the
        same sequential fit `allocation._fit_group` does on the board's mixer.
        """
        n = len(self._live)
        u = [0.0] * n
        order = [AXES.index(a) for a in priority_order(self._unactuated)
                 if abs(want[AXES.index(a)]) > 1e-12]
        dirs = {}
        for k in order:
            single = [0.0] * 6
            single[k] = want[k]
            dirs[k] = self._least_squares(single, list(range(n)))
        granted = {k: 0.0 for k in order}

        # ⚠ MORE THAN ONE PASS, AND WHY. Where the geometry couples axes, a
        # LOWER axis can move a thruster AWAY from its stop: any off-axis
        # thruster's moment compensation, or the cross-terms a dead tunnel
        # leaves behind, can pull a pinned pair inward after a higher axis
        # pinned it. One pass would leave that room unspent and short the
        # higher axis while its thrusters had it.
        # Each pass only ADDS to a grant, highest rank first, so a later pass
        # can never take from an axis what an earlier one gave.
        for _ in range(len(order) + 1):
            moved = False
            for k in order:
                du = dirs[k]
                lam = 1.0 - granted[k]
                for i in range(n):
                    if abs(du[i]) <= 1e-12:
                        continue
                    bound = limit if du[i] > 0 else -limit
                    lam = min(lam, (bound - u[i]) / du[i])
                if lam > 1e-12:
                    granted[k] += lam
                    u = [u[i] + lam * du[i] for i in range(n)]
                    moved = True
            if not moved:
                break
        return u
