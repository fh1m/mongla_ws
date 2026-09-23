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

# ⛔ FlightMode, from `include/state_types.h`. NOT a magic number -- the first
# version of this file passed `mode=3`, which is not a FlightMode at all, so
# `feedforward::apply` took its `stabilized` branch as FALSE and returned the
# demands untouched. Every feedforward gain then measured identically, which
# reads as "the layer does nothing" rather than as "it was never called".
MODE_STABILIZE = 0
MODE_ACRO = 1
MODE_DEPTH_HOLD = 2
MODE_AUTO = 23


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
                 inertia: Inertia | None = None, from_board: bool = True,
                 thruster_tau_s: float = 0.0):
        """`thruster_tau_s` -- first-order lag from commanded to delivered
        thrust. ⛔ IT IS NOT ZERO ON A REAL THRUSTER, and the default is zero
        only because ours is unmeasured. A T100-class unit's thrust dead time is
        reported around 0.59 s; a small custom unit will be faster, but it will
        not be instant. It is the term most likely to dominate every argument
        about loop rate, so it is modelled explicitly rather than assumed away.
        """
        self.board = Board()
        # ⛔ The cascade must run on the VEHICLE's gains, not config.h's. The
        # defaults differ by 3.556x on PILOT_YAW_RATE alone.
        self.board.from_board() if from_board else self.board.defaults()
        self.plant = Plant(inertia or Inertia(), damping)
        self.thrusters = cad_hull_thrusters(max_thrust_n)
        self.B = wrench_matrix(self.thrusters)
        self.max_thrust_n = max_thrust_n
        self.thruster_tau_s = thruster_tau_s
        self.alloc = ga.GeometricAllocator()

    def _wrench(self, thrusts) -> list:
        return [sum(self.B[i][j] * thrusts[j] for j in range(len(thrusts)))
                * self.max_thrust_n for i in range(6)]

    def torque_per_unit_demand(self) -> float:
        """N.m produced by one unit of NORMALISED axis demand.

        The allocator works in units of (moment arm x thrust fraction), so a
        demand the allocator can meet exactly becomes `demand * max_thrust_n`
        newton-metres. This is the conversion the firmware's feedforward gains
        are implicitly denominated in, and it is why a gain that is right on
        one vehicle is meaningless on another.
        """
        return self.max_thrust_n

    def ideal_drag_gain(self, axis: int = 5) -> float:
        """⭐ The `ATC_DRAG_*` value that would cancel THIS PLANT's drag exactly.

        The firmware adds `atc_drag_yaw * gz * |gz|` to the normalised demand;
        the plant subtracts `q * r * |r|` newton-metres. They cancel when
        `atc_drag * torque_per_unit = q`.

        ⚠ CIRCULAR IF USED ALONE, and that is the point of `fly`'s
        `drag_gain_scale`. We chose the plant's drag, so of course a gain
        derived from it cancels. The question worth asking is not "does a
        perfect gain help" but "how wrong may it be before it hurts", because
        on the real vehicle it will never be perfect.
        """
        return self.plant.damping.quad[axis] / self.torque_per_unit_demand()

    def fly(self, *, seconds: float, dt: float = 0.002,
            stick=lambda t: (0.0, 0.0, 0.0),
            hold_yaw: float | None = 0.0,
            disturbance=lambda t: (0.0,) * 6,
            drag_gain_scale: float | None = None,
            control_hz: float | None = None,
            controller=None,
            quantise: bool = True) -> Trace:
        """Run the closed loop. `stick` returns (roll, pitch, yaw) each tick.

        `quantise=False` bypasses `oneToDshot` and drives the plant with the raw
        allocator demand -- which is not the vehicle, and exists only to MEASURE
        what MOT_SPIN_MIN costs by difference.
        """
        # ⛔ RESET THE PLANT, NOT JUST THE BOARD. An earlier version reset only
        # the controller, so a reused Vehicle started each run from the previous
        # run's attitude and rates -- and the results looked like a feedforward
        # effect rather than like leftover state. A bench that carries state
        # between runs is worse than no bench, because its output is plausible.
        self.plant.nu = [0.0] * 6
        self.plant.eta = [0.0] * 6
        delivered = [0.0] * 5
        self._applied_norm = (0.0, 0.0, 0.0)
        if controller is not None:
            controller.reset()
        self.board.reset()
        # ⚠ The five feedforward gains ship at 0.0 on the live board. Anything
        # non-zero here is an EXPLORATION of what they would do, never a
        # description of the vehicle.
        ff = (0.0 if drag_gain_scale is None
              else drag_gain_scale * self.ideal_drag_gain())
        self.board.set_feedforward(drag_yaw=ff, drag_rll=0.0, drag_pit=0.0)
        if hold_yaw is not None:
            self.board.hold_yaw(hold_yaw)
        tr = Trace()
        t = 0.0
        # ⛔ DECIMATION IS NOT THE SAME AS A BIGGER `dt`, and conflating them
        # would answer a different question. The PLANT always integrates at
        # `dt`; only the CONTROLLER is run less often, and its output is HELD
        # between updates -- which is what a slower control task actually does.
        # Passing a larger dt to the plant would instead measure the
        # integrator, and would flatter the slow rates by removing the
        # zero-order hold whose lag is the entire effect under test.
        every = 1 if control_hz is None else max(1, round(1.0 / (control_hz * dt)))
        held = None
        for k in range(int(seconds / dt)):
            roll, pitch, yaw = self.plant.attitude
            gx, gy, gz = self.plant.body_rates
            sr, sp, sy = stick(t)
            if controller is not None:
                if held is None or k % every == 0:
                    # ⭐ INDI needs the torque ACTUALLY delivered, which after
                    # the allocator, the DShot quantiser and the thruster lag is
                    # not the torque that was asked for. Feeding back the demand
                    # instead is the `use_achieved=False` case, and it is what
                    # upstream ask B exists to make possible on the real wire.
                    held = controller.update(
                        attitude=(roll, pitch, yaw), rates=(gx, gy, gz),
                        target=(0.0, 0.0, hold_yaw or 0.0),
                        tau_applied=self._applied_norm, dt=dt * every)
                tq_r, tq_p, tq_y = held
            elif held is None or k % every == 0:
                # ⚠ The controller's own dt must be its ACTUAL period, or its
                # integral and derivative terms are scaled for a rate it is not
                # running at -- which would measure a detuned controller rather
                # than a slower one.
                tq_r, tq_p, tq_y = self.board.stabilize(
                    stick_roll=sr, stick_pitch=sp, stick_yaw=sy,
                    roll=roll, pitch=pitch, yaw=yaw,
                    gx=gx, gy=gy, gz=gz, dt=dt * every)
                held = (tq_r, tq_p, tq_y)
                tq_r, tq_p, tq_y = held
            else:
                tq_r, tq_p, tq_y = held

            if ff:
                # `feedforward::apply` -- the firmware's own drag / cross-
                # coupling layer, between the cascade and the mixer. Mode 3 is
                # `learn=False` keeps the CoB auto-trim out of it,
                # because a trim that learns during a scripted test is a
                # confound, not a feature.
                tq_r, tq_p, tq_y, _ = self.board.feedforward(
                    roll=tq_r, pitch=tq_p, yaw=tq_y, throttle=0.0,
                    gx=gx, gy=gy, gz=gz, mode=MODE_STABILIZE, learn=False)
            a = self.alloc.allocate(
                **body_to_cad(roll=tq_r, pitch=tq_p, yaw=tq_y))
            cmd = [dshot_to_fraction(self.board, d) if quantise else d
                   for d in a.thrusts]
            if self.thruster_tau_s > 0.0:
                # First-order lag, per thruster. The ESC and the rotor cannot
                # change the water's momentum instantly.
                alpha = dt / (dt + self.thruster_tau_s)
                delivered = [delivered[i] + alpha * (cmd[i] - delivered[i])
                             for i in range(len(cmd))]
                u = list(delivered)
            else:
                u = cmd
            tau = self._wrench(u)
            # What the actuators REALLY produced, back in the board's
            # normalised torque units -- the signal ask B would put on the wire.
            self._applied_norm = tuple(
                tau[i] / self.max_thrust_n for i in (3, 4, 5))
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
