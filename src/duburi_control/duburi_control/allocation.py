"""What the mixer will actually deliver, computed before we ask for it.

⛔ THE VEHICLE SILENTLY LOSES AUTHORITY AND NOTHING ON THIS SIDE KNOWS. The
board's mixer computes a per-group scale-down when a demand drives any thruster
past full (`mixer.cpp`: `if (maxabs[g] > 1.0f) out[m] /= maxabs[g]`) and then
throws that number away -- it is never reported on any message. So when a
manoeuvre saturates, the host keeps commanding a demand the hull is not
receiving, sees no response, and pushes harder.

That is the classic integrator wind-up against an actuator limit, and it is
worst exactly where it hurts: a close-in vision alignment, where `ki_lat` is
integrating a lateral error while the horizontal group is already clipped. The
loop cannot tell "the target is not moving because I am too weak" from "the
target is not moving because I am saturated".

⛔ AND NO FIRMWARE CHANGE IS NEEDED TO FIX IT, which is the point of this file.
The demand is OURS -- we composed it -- and the mixer matrix is a published
constant of the frame. So the saturation is a pure function of numbers we
already hold, and can be computed BEFORE the frame is sent rather than
inferred afterwards from a vehicle that will not tell us.

The firmware half is still worth asking for (reporting `maxabs` costs two
lines and would confirm this against the board's own arithmetic), but it is a
cross-check, not a dependency.

THE BLOCK-DIAGONAL STRUCTURE IS LOAD-BEARING and was a real firmware bug once.
Motors 1-4 are non-zero only in yaw/forward/lateral; motors 5-8 only in
roll/pitch/throttle. The two groups share no axis and no thruster, so they
saturate INDEPENDENTLY. Computing one global scale coupled them: a saturating
forward command scaled down roll and pitch even though the verticals were
nowhere near their limits -- a hard forward burst silently cost a third of the
vehicle's attitude authority, in the manoeuvre where you want it most.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

# Columns: roll, pitch, yaw, throttle, forward, lateral. Rows: motors 1..8.
# SUB_FRAME_VECTORED_6DOF, mirrored from the firmware's `mixer.cpp` M[][].
# ⚠ TWO COPIES OF ONE TRUTH. `test_srot_protocol_drift.py` reads the firmware
# headers directly for the wire constants; this matrix is the same class of
# shared fact and carries the same risk, so it is asserted against the
# firmware source by `test_allocation.py` rather than trusted.
MIXER = (
    (0.0,  0.0,  1.0,  0.0, -1.0,  1.0),   # 1 FR horiz
    (0.0,  0.0, -1.0,  0.0, -1.0, -1.0),   # 2 FL horiz
    (0.0,  0.0, -1.0,  0.0,  1.0,  1.0),   # 3 RR horiz
    (0.0,  0.0,  1.0,  0.0,  1.0, -1.0),   # 4 RL horiz
    (1.0, -1.0,  0.0, -1.0,  0.0,  0.0),   # 5 FR vert
    (-1.0, -1.0, 0.0, -1.0,  0.0,  0.0),   # 6 FL vert
    (1.0,  1.0,  0.0, -1.0,  0.0,  0.0),   # 7 RR vert
    (-1.0,  1.0, 0.0, -1.0,  0.0,  0.0),   # 8 RL vert
)
N_HORIZ = 4

# Groups, by the axes they own. Naming them lets a diagnostic say WHICH
# authority ran out instead of only that something did.
HORIZONTAL_AXES = ('yaw', 'forward', 'lateral')
VERTICAL_AXES = ('roll', 'pitch', 'throttle')


@dataclass(frozen=True)
class Allocation:
    """What the mixer does to a demand."""
    horizontal_scale: float      # 1.0 = delivered in full, 0.5 = half
    vertical_scale: float
    delivered: Tuple[float, ...]   # the six axes as the hull will feel them

    @property
    def saturated(self) -> bool:
        return self.horizontal_scale < 1.0 or self.vertical_scale < 1.0

    @property
    def worst_scale(self) -> float:
        return min(self.horizontal_scale, self.vertical_scale)

    def limiting_group(self) -> str:
        """Which authority ran out: 'horizontal', 'vertical', 'both', ''."""
        h = self.horizontal_scale < 1.0
        v = self.vertical_scale < 1.0
        if h and v:
            return 'both'
        if h:
            return 'horizontal'
        return 'vertical' if v else ''


def allocate(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
             throttle: float = 0.0, forward: float = 0.0,
             lateral: float = 0.0) -> Allocation:
    """Run the board's own mixer arithmetic, ahead of the board.

    Every argument is a normalised axis demand in [-1, 1], the same units the
    MANUAL_CONTROL path uses. Returns the per-group scale the mixer will apply
    and the demand as actually delivered.
    """
    demand = (float(roll), float(pitch), float(yaw),
              float(throttle), float(forward), float(lateral))
    max_h = 0.0
    max_v = 0.0
    for m, row in enumerate(MIXER):
        v = sum(row[c] * demand[c] for c in range(6))
        a = abs(v)
        if m < N_HORIZ:
            max_h = max(max_h, a)
        else:
            max_v = max(max_v, a)
    # Scale only when past full. Below it the mixer does nothing, and
    # reporting a scale < 1.0 there would invent a limit that is not there.
    sh = 1.0 / max_h if max_h > 1.0 else 1.0
    sv = 1.0 / max_v if max_v > 1.0 else 1.0
    delivered = (demand[0] * sv, demand[1] * sv, demand[2] * sh,
                 demand[3] * sv, demand[4] * sh, demand[5] * sh)
    return Allocation(horizontal_scale=sh, vertical_scale=sv,
                      delivered=delivered)


def headroom(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
             throttle: float = 0.0, forward: float = 0.0,
             lateral: float = 0.0) -> Tuple[float, float]:
    """(horizontal, vertical) demand multiplier still available before clipping.

    A value of 1.0 means the demand is exactly at the limit; 2.0 means it
    could be doubled. Below 1.0 it is already clipped.

    This is what a controller should consult before integrating: asking for
    more when headroom is 1.0 buys nothing and winds the integrator up against
    a wall it cannot see.
    """
    a = allocate(roll, pitch, yaw, throttle, forward, lateral)
    demand = (roll, pitch, yaw, throttle, forward, lateral)
    max_h = max((abs(sum(MIXER[m][c] * demand[c] for c in range(6)))
                 for m in range(N_HORIZ)), default=0.0)
    max_v = max((abs(sum(MIXER[m][c] * demand[c] for c in range(6)))
                 for m in range(N_HORIZ, len(MIXER))), default=0.0)
    inf = float('inf')
    return (inf if max_h <= 0.0 else 1.0 / max_h,
            inf if max_v <= 0.0 else 1.0 / max_v)


def largest_axis_within_budget(axis: str, *, others: dict | None = None
                               ) -> float:
    """The biggest demand on `axis` that still fits, given the others.

    ⛔ THE USE CASE THAT MOTIVATES IT. A vision loop wants the largest lateral
    correction the hull can actually deliver while a heading lock is already
    spending yaw authority. Asking for more than this is not merely wasted --
    the mixer scales the WHOLE GROUP uniformly, so an over-large lateral
    demand also shrinks the yaw the lock is relying on. Saturation does not
    just clip the axis that caused it; it steals from its neighbours.
    """
    base = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'throttle': 0.0, 'forward': 0.0, 'lateral': 0.0}
    base.update(others or {})
    if axis not in base:
        raise KeyError(f'{axis!r} is not a mixer axis: {sorted(base)}')
    names = ('roll', 'pitch', 'yaw', 'throttle', 'forward', 'lateral')
    idx = names.index(axis)
    rows = range(N_HORIZ) if axis in HORIZONTAL_AXES else range(N_HORIZ, len(MIXER))
    best = 1.0
    for m in rows:
        coeff = MIXER[m][idx]
        if abs(coeff) < 1e-12:
            continue
        rest = sum(MIXER[m][c] * base[names[c]] for c in range(6) if c != idx)
        # |rest + coeff * x| <= 1 for both signs of the bound.
        for bound in (1.0, -1.0):
            x = (bound - rest) / coeff
            if x >= 0.0:
                best = min(best, x)
    return max(0.0, min(1.0, best))


# Priority when the horizontal group runs out. The order is a CONTROL decision
# and is stated here rather than buried in a caller.
#
# ⛔ YAW FIRST, AND THAT IS THE OPPOSITE OF WHAT UNIFORM SCALING DOES. The
# board scales the whole group together, so a large forward demand steals yaw
# in the same proportion -- and yaw is the axis a launcher, a dropper and a
# gate transit all depend on. Heading error points the tool at the wrong place;
# a slightly slow approach does not. The quadrotor literature reaches the same
# conclusion for the same reason (attitude before position before yaw, because
# attitude error is unrecoverable and a position error is merely late); ours
# ranks differently only because our "attitude" is held by a separate group.
HORIZONTAL_PRIORITY = ('yaw', 'lateral', 'forward')
VERTICAL_PRIORITY = ('roll', 'pitch', 'throttle')

_AXIS_NAMES = ('roll', 'pitch', 'yaw', 'throttle', 'forward', 'lateral')


def prioritise(demand: dict, *, horizontal_priority: Sequence[str] = HORIZONTAL_PRIORITY,
               vertical_priority: Sequence[str] = VERTICAL_PRIORITY) -> dict:
    """Fit a demand inside the mixer by sacrificing the LEAST important axis.

    Uniform scaling is the wrong response to saturation and the reason is
    concrete: it degrades every axis equally, including the one the manoeuvre
    exists to get right. Asking for 100 % forward while holding a heading does
    not just arrive slowly -- it arrives slowly AND off-heading, because the
    board scaled the yaw down by the same factor.

    This gives the high-priority axes what they ask for and spends whatever is
    left on the rest, in order. Returns a NEW demand dict; the input is not
    modified, so a caller can log both and see what it gave up.

    Purely host-side: it changes the numbers we send, not how the board mixes
    them, so it needs no firmware and cannot desynchronise from one.
    """
    out = {name: float(demand.get(name, 0.0)) for name in _AXIS_NAMES}
    for group in (horizontal_priority, vertical_priority):
        _fit_group(out, group)
    return out


def _fit_group(out: dict, order: Sequence[str]) -> None:
    """Give each axis in `order` the most it can have, high priority first."""
    granted = {name: 0.0 for name in order}
    for name in order:
        want = out[name]
        if want == 0.0:
            continue
        # The most this axis may have, with everything already granted in
        # place and everything not yet considered set to zero.
        room = largest_axis_within_budget(name, others=dict(granted))
        granted[name] = math.copysign(min(abs(want), room), want)
    out.update(granted)
