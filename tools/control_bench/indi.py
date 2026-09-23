"""INDI attitude control, for the bench only.

⛔ THIS IS NOT FIRMWARE AND MUST NOT BE MISTAKEN FOR IT. Everything else the
bench runs is the board's own compiled code. This is a HOST-SIDE
reimplementation of a controller the board does not have, written to answer one
question before we ever propose it: **does INDI beat the tuned cascade on our
own plant?** If it cannot win here it does not fly, and that is a publishable
result either way -- the sweep found zero underwater INDI applications until
Cuttlefish (DFKI, IROS 2024).

THE LAW. Incremental Nonlinear Dynamic Inversion takes the plant's own measured
acceleration as the model, so it needs no drag model at all:

    omega_dot_des  = K (omega_des - omega)              the rate loop
    tau            = tau_applied + I (omega_dot_des - omega_dot_meas)

Everything the vehicle does not know about itself -- drag, added mass, Coriolis,
a fouled propeller, a current -- shows up in `omega_dot_meas` and is cancelled
without ever being modelled. That is the whole claim, and it is why the DFKI
work states INDI "requires only a 6x6 mass-inertia matrix and an actuation
model".

⚠ TWO THINGS MAKE OR BREAK IT, and both are modelled here rather than assumed:

  1. `tau_applied` must be the torque the actuators ACTUALLY produced, not the
     torque that was asked for. With `MOT_SPIN_MIN` quantising every output,
     those differ by up to 16 % of full scale. This is exactly what upstream
     ask B (the achieved wrench on the wire) exists to provide, and INDI is
     blocked on it. `use_achieved=False` measures what happens without it.

  2. THE INCREMENT MUST BE LIMITED TO ACHIEVABLE TORQUE. An incremental law
     keeps adding until it sees the acceleration it asked for. Point it at a
     torque the allocator cannot deliver and it winds up without bound -- there
     is no acceleration coming, so the increment never stops. A PID gets away
     with a loose output clamp; INDI does not.

  3. IT MUST NOT BE POINTED AT AN UNACTUATED AXIS AT ALL. Discovered on this
     bench: running INDI on roll -- which this hull cannot actuate, the
     allocator's B having rank 5 of 6 -- wound the roll increment to its clamp,
     and the Euler coupling then dragged yaw off target while the yaw axis
     itself sat saturated. The cascade survives the same mistake because its
     output is a bounded PID and the allocator simply refuses the demand.
     `axes=` makes the actuated set explicit rather than assumed.

  4. FILTER SYNCHRONISATION. `omega_dot_meas` has to be filtered or the
     differentiated gyro is noise. But then `tau_applied` must be delayed by
     the SAME filter, or the increment compares a filtered present against an
     unfiltered past and the loop fights its own lag. Getting this wrong is the
     classic INDI failure and it looks like a tuning problem.
"""
from __future__ import annotations

import math


class IndiRate:
    """One axis of incremental rate control."""

    def __init__(self, *, inertia: float, k_rate: float, cutoff_hz: float = 20.0,
                 tau_limit: float = 1.0, use_achieved: bool = True):
        self.I = inertia
        self.k = k_rate
        self.fc = cutoff_hz
        self.limit = tau_limit
        self.use_achieved = use_achieved
        self.reset()

    def reset(self) -> None:
        self._w_prev = None
        self._wdot_f = 0.0        # filtered angular acceleration
        self._tau_f = 0.0         # tau_applied through the SAME filter
        self._tau_cmd = 0.0

    def update(self, *, omega: float, omega_des: float,
               tau_applied: float, dt: float) -> float:
        """One tick. Returns the torque demand, normalised like the board's."""
        if dt <= 0.0:
            return self._tau_cmd

        # Raw angular acceleration by backward difference. Noisy by nature --
        # that is what the filter is for, and what the sync then has to undo.
        if self._w_prev is None:
            wdot_raw = 0.0
        else:
            wdot_raw = (omega - self._w_prev) / dt
        self._w_prev = omega

        alpha = dt / (dt + 1.0 / (2.0 * math.pi * self.fc))
        self._wdot_f += alpha * (wdot_raw - self._wdot_f)

        # ⭐ THE SYNCHRONISATION. `tau_applied` goes through an IDENTICAL filter
        # so the increment is taken between two quantities with the same lag.
        # Comparing a filtered acceleration against an unfiltered torque is the
        # standard way to make INDI oscillate while looking like bad gains.
        src = tau_applied if self.use_achieved else self._tau_cmd
        self._tau_f += alpha * (src - self._tau_f)

        wdot_des = self.k * (omega_des - omega)
        tau = self._tau_f + self.I * (wdot_des - self._wdot_f)
        self._tau_cmd = max(-self.limit, min(self.limit, tau))
        return self._tau_cmd


class IndiAttitude:
    """Outer angle P into three INDI rate loops.

    The OUTER loop is deliberately identical in form to the board's
    (`ang_yaw_p` etc.), so a comparison against the cascade isolates the INNER
    loop -- which is the only thing INDI changes. Beating a cascade by also
    changing the outer loop would measure nothing.
    """

    def __init__(self, *, inertia_xyz, k_angle=(4.0, 4.0, 6.0),
                 k_rate=(8.0, 8.0, 8.0), rate_limit=2.793,
                 cutoff_hz: float = 20.0, use_achieved: bool = True,
                 tau_limits=(1.0, 1.0, 1.0), axes=(False, True, True)):
        """`axes` -- which of (roll, pitch, yaw) this controller may drive.

        ⛔ ROLL DEFAULTS TO OFF because this hull cannot actuate it. That is not
        a tuning choice, it is the geometry: `geometric_allocation` finds B to
        be rank 5 of 6 with roll the null axis, and the plant's own wrench
        matrix reaches the same conclusion independently. Roll is held by the
        restoring moment instead.

        `tau_limits` should be what the ALLOCATOR can actually achieve on each
        axis, not +-1. See the windup note in the module docstring.
        """
        self.k_angle = k_angle
        self.rate_limit = rate_limit
        self.enabled = tuple(axes)
        self.axes = [IndiRate(inertia=inertia_xyz[i], k_rate=k_rate[i],
                              cutoff_hz=cutoff_hz, use_achieved=use_achieved,
                              tau_limit=tau_limits[i])
                     for i in range(3)]

    def reset(self) -> None:
        for a in self.axes:
            a.reset()

    def update(self, *, attitude, rates, target, tau_applied, dt):
        out = []
        for i in range(3):
            if not self.enabled[i]:
                out.append(0.0)      # an axis we cannot drive gets a true zero
                continue
            err = target[i] - attitude[i]
            if i == 2:                       # yaw wraps
                err = (err + math.pi) % (2 * math.pi) - math.pi
            w_des = max(-self.rate_limit,
                        min(self.rate_limit, self.k_angle[i] * err))
            out.append(self.axes[i].update(omega=rates[i], omega_des=w_des,
                                           tau_applied=tau_applied[i], dt=dt))
        return tuple(out)
