"""Close the loop: the board's own control code flying the Fossen plant.

    stabilize / acro   FIRMWARE, compiled from the checkout   attitude -> torque
      -> allocator     OURS (geometric_allocation)            torque  -> 5 demands
      -> oneToDshot    FIRMWARE, per thruster                 demand  -> DShot
      -> plant                                                force   -> motion
      -> gyro + AHRS                                          motion  -> back in

⛔ WHY THE FIRMWARE MIXER IS NOT IN THAT CHAIN, and it is not an oversight.
`mixer.cpp` allocates for `vectored_6dof` -- EIGHT T200s at 45 degrees. The hull
in CAD has FIVE orthogonal thrusters with roll unactuated. Pushing one through
the other would simulate a vehicle that does not exist, so the loop uses our
geometric allocator exactly where the firmware's matrix is wrong for this hull,
and real firmware everywhere the firmware is hull-INDEPENDENT:

    stabilize / acro   attitude error -> torque. No geometry in it. REAL.
    oneToDshot         one demand -> one DShot value, incl. MOT_SPIN_MIN. REAL.
    mixer.cpp          the one hull-specific piece, and the one we replace.

⭐ THAT SUBSTITUTION IS ALSO THE MEASUREMENT. `compare_allocators` runs the
identical manoeuvre both ways on the same plant, which is what turns upstream
ask K from an argument into a number.

⚠ MOT_SPIN_MIN IS DELIBERATELY INSIDE THE LOOP. The DShot value is converted
back to a thrust fraction rather than the pre-quantisation demand being used, so
the 16.22 % smallest-non-zero-output floor acts on every tick -- which is the
single effect most likely to decide whether precision alignment works.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from control_bench import Board                                   # noqa: E402
from plant import (Damping, Inertia, Plant, _load_pure,           # noqa: E402
                   cad_hull_thrusters, wrench_matrix)

ga = _load_pure('geometric_allocation')

DSHOT_SPAN = 999.0


# ═══════════════════════════════════════════════════════════════════════════ #
#  ⛔ THE FRAME SEAM, and it produced a runaway the first time it was missed
# ═══════════════════════════════════════════════════════════════════════════ #
#
# `geometric_allocation` reads `hull_geometry.yaml` straight through, so it
# works in the CAD FRAME: X lateral, Y the long axis, Z the vertical bore. The
# plant is Fossen: x forward, y starboard, z DOWN. The map between them,
#
#     body = (cad.y, cad.x, -cad.z)
#
# is a PROPER rotation (determinant +1), so moments transform exactly like
# forces -- and two of the six axes come back negated:
#
#     surge  sway  roll  pitch    same sign
#     heave  yaw                  INVERTED, because body z = -cad z
#
# ⚠ THE FIRST VERSION OF THIS FILE MISSED IT and the yaw loop ran away to
# -1988 degrees in six seconds -- positive feedback, because the allocator
# produced the opposite moment to the one the cascade asked for. It is worth
# noticing that a runaway is the LUCKY outcome: had the gain been lower it
# would merely have converged slowly and looked like a tuning problem.
#
# So the conversion is a named function with a test, not an inline minus sign.

_FLIP = ('heave', 'yaw')


def body_to_cad(**axes) -> dict:
    """Body-frame axis demands -> the CAD-frame names the allocator uses."""
    return {k: (-v if k in _FLIP else v) for k, v in axes.items()}


def cad_to_body(**axes) -> dict:
    """The inverse. It IS the same map -- the transform is an involution."""
    return body_to_cad(**axes)


@dataclass
class Trace:
    """What a run produced. Lists, one entry per tick."""
    t: list = field(default_factory=list)
    roll: list = field(default_factory=list)
    pitch: list = field(default_factory=list)
    yaw: list = field(default_factory=list)
    gz: list = field(default_factory=list)
    torque_yaw: list = field(default_factory=list)
    thrusts: list = field(default_factory=list)
    refused: list = field(default_factory=list)

    def settle_time(self, target: float, band: float, signal='yaw') -> float | None:
        """First time `signal` enters +-band of target AND never leaves.

        Returns None rather than a number when it never settles, because a
        verb that reports success while the vehicle is still moving is the
        failure this codebase exists to refuse.
        """
        s = getattr(self, signal)
        ok = None
        for i in range(len(s) - 1, -1, -1):
            if abs(s[i] - target) > band:
                break
            ok = self.t[i]
        return ok

    def overshoot(self, target: float, signal='yaw') -> float:
        s = getattr(self, signal)
        if not s or target == 0.0:
            return max((abs(v) for v in s), default=0.0)
        return max(0.0, (max(s) - target) / abs(target)) if target > 0 else \
               max(0.0, (target - min(s)) / abs(target))


def dshot_to_fraction(board: Board, demand: float) -> float:
    """One thruster demand -> the fraction the board ACTUALLY produces.

    Round-trips through the firmware's `oneToDshot`, so `MOT_SPIN_MIN`, the
    thrust-curve expo and the 0.005 centre gap are all applied by their real
    implementations rather than re-described here.
    """
    v = board.one_to_dshot(demand, 1)
    if v >= 1049:
        return (v - 1048) / DSHOT_SPAN
    if v <= 1047:
        return -(v - 47) / DSHOT_SPAN
    return 0.0


class Vehicle:
    """The board, the allocator, the thrusters and the water, wired together."""

    def __init__(self, *, damping: Damping, max_thrust_n: float = 20.0,
                 inertia: Inertia | None = None, from_board: bool = True):
        self.board = Board()
        # ⛔ The cascade must run on the VEHICLE's gains, not config.h's. The
        # defaults differ by 3.556x on PILOT_YAW_RATE alone.
        self.board.from_board() if from_board else self.board.defaults()
        self.plant = Plant(inertia or Inertia(), damping)
        self.thrusters = cad_hull_thrusters(max_thrust_n)
        self.B = wrench_matrix(self.thrusters)
        self.max_thrust_n = max_thrust_n
        self.alloc = ga.GeometricAllocator()

    def _wrench(self, thrusts) -> list:
        return [sum(self.B[i][j] * thrusts[j] for j in range(len(thrusts)))
                * self.max_thrust_n for i in range(6)]

    def fly(self, *, seconds: float, dt: float = 0.002,
            stick=lambda t: (0.0, 0.0, 0.0),
            hold_yaw: float | None = 0.0,
            disturbance=lambda t: (0.0,) * 6,
            quantise: bool = True) -> Trace:
        """Run the closed loop. `stick` returns (roll, pitch, yaw) each tick.

        `quantise=False` bypasses `oneToDshot` and drives the plant with the raw
        allocator demand -- which is not the vehicle, and exists only to MEASURE
        what MOT_SPIN_MIN costs by difference.
        """
        self.board.reset()
        if hold_yaw is not None:
            self.board.hold_yaw(hold_yaw)
        tr = Trace()
        t = 0.0
        for _ in range(int(seconds / dt)):
            roll, pitch, yaw = self.plant.attitude
            gx, gy, gz = self.plant.body_rates
            sr, sp, sy = stick(t)
            tq_r, tq_p, tq_y = self.board.stabilize(
                stick_roll=sr, stick_pitch=sp, stick_yaw=sy,
                roll=roll, pitch=pitch, yaw=yaw, gx=gx, gy=gy, gz=gz, dt=dt)

            a = self.alloc.allocate(
                **body_to_cad(roll=tq_r, pitch=tq_p, yaw=tq_y))
            u = [dshot_to_fraction(self.board, d) if quantise else d
                 for d in a.thrusts]
            tau = self._wrench(u)
            dist = disturbance(t)
            self.plant.step([tau[i] + dist[i] for i in range(6)], dt)

            tr.t.append(t); tr.roll.append(roll); tr.pitch.append(pitch)
            tr.yaw.append(yaw); tr.gz.append(gz); tr.torque_yaw.append(tq_y)
            tr.thrusts.append(tuple(u)); tr.refused.append(a.refused)
            t += dt
        return tr


def sweep_drag(build, run, *, cds=((0.15, 0.80), (0.25, 1.00), (0.30, 1.20))):
    """⭐ RUN A COMPARISON ACROSS THE WHOLE DRAG BAND, not at one guess.

    Drag is the plant's unmeasured term. A conclusion that holds at every
    corner of the plausible band is a finding; one that flips inside it is a
    coin toss, and reporting it as a finding is how an unidentified plant
    produces confident nonsense.

    `build(damping)` returns whatever the caller needs; `run(thing)` returns a
    comparable number. Yields (cd_axial, cd_transverse, result).
    """
    for cd_a, cd_t in cds:
        d = Damping.from_cd(cd_axial=cd_a, cd_transverse=cd_t,
                            provenance=f'SWEPT band point Cd={cd_a}/{cd_t}, UNMEASURED')
        yield cd_a, cd_t, run(build(d))
