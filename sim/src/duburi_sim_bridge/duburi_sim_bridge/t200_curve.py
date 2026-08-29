#!/usr/bin/env python3

"""Blue Robotics T200: the real thrust curve, in place of a straight line.

WHY THIS EXISTS. The chain from ArduSub to water is

    ArduSub --PWM--> ArduPilotPlugin --cmd_thrust--> Thruster plugin --> joint

and ArduPilotPlugin can only apply `(normalised_pwm + offset) * multiplier` --
an affine map. So the simulated thruster was perfectly linear in PWM, symmetric
forward and reverse, with no deadband and no spin-up. A real T200 is none of
those things, and the gap is not small: against Blue Robotics' published
performance data at 16 V, a straight line is wrong by up to **12.24 N, 24 % of
full thrust**.

That error is not noise, it is structural. It is largest at low command --
exactly where station-keeping, alignment and every fine correction live. A
controller tuned in a sim where 5 % stick gives 5 % thrust learns gains that do
not survive contact with a thruster that gives nothing until 7 % and then jumps.

This node sits in the middle of that chain: it takes the linear thrust ArduSub's
plugin computed, recovers the PWM that produced it, and republishes the thrust
the datasheet says a T200 actually makes at that PWM.

DATA. `T200-Public-Performance-Data-10-20V-September-2019.xlsx`, Blue Robotics'
own bollard-pull measurements. Per voltage:

    volts   deadband (us)   max fwd   max rev   rev/fwd
    12      1464-1536       36.4 N    -28.6 N    78.5 %
    14      1468-1532       44.4 N    -34.5 N    77.7 %
    16      1472-1528       51.4 N    -39.9 N    77.6 %
    18      1472-1528       59.0 N    -45.0 N    76.3 %
    20      1476-1528       65.9 N    -49.4 N    74.9 %

Three facts a linear model misses, all of them things an operator feels:

  DEADBAND ~+/-28 us. Nothing happens near neutral. This is why a real vehicle
  will not hold a millimetre-precise station: small corrections do nothing at
  all until they suddenly do something.

  REVERSE IS ~78 % OF FORWARD. The prop is not symmetric. A vehicle that
  accelerates one way and brakes the other does not behave the same in both,
  and any manoeuvre tuned only in forward is over-tuned in reverse.

  THE CURVE IS QUADRATIC, not linear. Fitted per side, a quadratic tracks the
  data to within 1.8 N where a straight line is out by 12.2 N.

WHAT THIS IS NOT. Bollard-pull data: thrust with the vehicle held still. It does
not capture thrust falling as inflow rises (the Thruster plugin's own
`wake_fraction`/`alpha_1`/`alpha_2` cover that), nor motor spin-up, which is
applied here as a first-order lag rather than measured.
"""

from __future__ import annotations

import math
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

# Fitted from the published data, per side, on u = (pwm - deadband_edge) /
# (endstop - deadband_edge) in [0, 1]. thrust = c0*u^2 + c1*u + c2.
#
# Quadratic on purpose: it tracks the measurements to 1.8 N against a straight
# line's 12.2 N, and a cubic only reaches 1.3 N -- not worth the extra term for
# a curve whose own run-to-run spread is larger than that.
T200 = {
    12.0: {'db': (1464.0, 1536.0), 'fwd': (23.13, 17.35, -0.30), 'rev': (-15.34, -13.99, 0.24)},
    14.0: {'db': (1468.0, 1532.0), 'fwd': (24.55, 22.71, -0.37), 'rev': (-17.31, -18.29, 0.30)},
    16.0: {'db': (1472.0, 1528.0), 'fwd': (25.69, 27.95, -0.43), 'rev': (-19.09, -22.71, 0.35)},
    18.0: {'db': (1472.0, 1528.0), 'fwd': (27.66, 32.62, -0.49), 'rev': (-20.94, -25.30, 0.39)},
    20.0: {'db': (1476.0, 1528.0), 'fwd': (29.14, 37.85, -0.55), 'rev': (-22.31, -28.25, 0.43)},
}

PWM_MIN, PWM_MID, PWM_MAX = 1100.0, 1500.0, 1900.0


def _interp_table(volts: float) -> dict:
    """Blend the two bracketing voltage rows.

    A vehicle does not run at exactly 12, 14 or 16 V, and snapping to the
    nearest row would put a 1 V step in thrust where the battery sags smoothly.
    """
    keys = sorted(T200)
    v = min(max(volts, keys[0]), keys[-1])
    hi = next((k for k in keys if k >= v), keys[-1])
    lo = max((k for k in keys if k <= v), default=keys[0])
    if hi == lo:
        return T200[hi]
    t = (v - lo) / (hi - lo)
    a, b = T200[lo], T200[hi]
    mix = lambda p, q: tuple(x + (y - x) * t for x, y in zip(p, q))  # noqa: E731
    return {'db': mix(a['db'], b['db']),
            'fwd': mix(a['fwd'], b['fwd']),
            'rev': mix(a['rev'], b['rev'])}


def thrust_at_pwm(pwm: float, volts: float = 16.0) -> float:
    """Newtons a T200 makes at this PWM and voltage. Zero inside the deadband."""
    tab = _interp_table(volts)
    lo, hi = tab['db']
    if lo <= pwm <= hi:
        return 0.0
    if pwm > hi:
        u = min((pwm - hi) / (PWM_MAX - hi), 1.0)
        c = tab['fwd']
    else:
        u = min((pwm - lo) / (PWM_MIN - lo), 1.0)
        c = tab['rev']
    return c[0] * u * u + c[1] * u + c[2]


def pwm_from_linear_thrust(thrust: float, max_thrust: float) -> float:
    """Invert the affine map ArduPilotPlugin applied, to recover the PWM.

    The plugin computes `((pwm - 1100)/800 - 0.5) * 2 * max_thrust`, so the PWM
    is recoverable exactly. Going through PWM rather than rescaling the thrust
    directly matters because the deadband and the asymmetry are properties of
    the COMMAND, not of the force.
    """
    frac = max(-1.0, min(1.0, thrust / max_thrust)) if max_thrust else 0.0
    return PWM_MID + frac * (PWM_MAX - PWM_MID)


class T200Curve(Node):
    """Rewrite each thruster's cmd_thrust through the measured T200 curve."""

    def __init__(self) -> None:
        super().__init__('t200_curve')
        # TWO NAMESPACES, and they are not the same one.
        #
        # ArduPilotPlugin's <cmd_topic> is an absolute path baked at generation
        # time from the MODEL name, so it publishes to /model/duburi_heavy/...
        # The Thruster plugin's <namespace> is also the model name, but Gazebo
        # ALSO exposes the joint under the INSTANCE name a course gives it
        # (`duburi`). Subscribing on the wrong one is silent: the node starts,
        # reports ready, shapes nothing, and the vehicle sits inert with the
        # thrusters unfed. Measured that exact failure once.
        #
        # `vehicle` is the model name and matches what the SDF was generated
        # with; it is the one both plugins agree on.
        self.declare_parameter('vehicle', 'duburi_heavy')
        self.declare_parameter('thrusters', 8)
        self.declare_parameter('max_thrust', 50.0)
        self.declare_parameter('voltage', 16.0)
        # First-order spin-up. A T200's rotor and the water around it take a
        # finite time to reach speed; commanding a step does not produce a step.
        # 0.15 s is the order quoted for a T200 under load. Set 0 to disable.
        self.declare_parameter('spinup_tau', 0.15)
        self.declare_parameter('enabled', True)
        # FAULT INJECTION. A dead thruster is a real competition failure -- a
        # flooded penetrator, a cut lead, a cooked ESC -- and ArduSub's
        # allocation matrix has to fly the vehicle on the remaining seven. It
        # cannot be tested any other way in sim, and this node is already the
        # only thing between ArduSub and the propellers.
        self.declare_parameter('dead_thrusters', [0])

        from gz.transport13 import Node as GzNode
        from gz.msgs10.double_pb2 import Double

        self._Double = Double
        self._gz = GzNode()
        self._max = float(self.get_parameter('max_thrust').value)
        self._volts = float(self.get_parameter('voltage').value)
        self._dead = self._parse_dead(
            self.get_parameter('dead_thrusters').value)
        # voltage was READ ONCE and cached, so setting it at runtime did
        # nothing. Battery sag needs it live: the T200 curve is
        # voltage-interpolated, which is the whole mechanism by which a sagging
        # pack costs thrust.
        self.add_on_set_parameters_callback(self._on_params)
        self._tau = float(self.get_parameter('spinup_tau').value)
        self._state = {}
        self._last = {}
        self._pub = {}

        vehicle = self.get_parameter('vehicle').value
        n = int(self.get_parameter('thrusters').value)
        for i in range(1, n + 1):
            base = f'/model/{vehicle}/joint/thruster{i}_joint'
            self._pub[i] = self._gz.advertise(f'{base}/cmd_thrust', Double)
            if not self._gz.subscribe(Double, f'{base}/cmd_thrust_linear',
                                      lambda m, k=i: self._on_cmd(k, m)):
                raise RuntimeError(
                    f'could not subscribe to {base}/cmd_thrust_linear -- '
                    'the thrusters would be silently unfed')
            self._state[i] = 0.0
            self._last[i] = time.monotonic()

        self.get_logger().info(
            f'T200 curve active on {n} thrusters at {self._volts:.1f} V '
            f'(deadband {_interp_table(self._volts)["db"]}, '
            f'spin-up tau {self._tau:.2f} s)')
        self.create_timer(10.0, self._report)
        self._n = 0

    @staticmethod
    def _parse_dead(value) -> set:
        """`[0]` means none -- thrusters are numbered from 1.

        An empty integer_array cannot round-trip through ROS parameters (it
        arrives typeless and the declaration is rejected), so the no-fault
        value has to be a sentinel rather than `[]`.
        """
        return {int(v) for v in (value or []) if int(v) > 0}

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'voltage':
                self._volts = float(p.value)
                self.get_logger().warn(
                    f'[T200 ] battery now {self._volts:.2f} V '
                    f'(deadband {_interp_table(self._volts)["db"]})')
            elif p.name == 'dead_thrusters':
                self._dead = self._parse_dead(p.value)
                self.get_logger().warn(
                    f'[T200 ] dead thrusters: '
                    f'{sorted(self._dead) if self._dead else "none"}')
        return SetParametersResult(successful=True)

    def _on_cmd(self, idx: int, msg) -> None:
        if idx in self._dead:
            # Publish zero rather than simply not publishing: the Thruster
            # plugin LATCHES its last command, so a silenced channel would keep
            # producing whatever thrust it had when it "failed".
            m = self._Double()
            m.data = 0.0
            self._pub[idx].publish(m)
            self._state[idx] = 0.0
            return
        if not self.get_parameter('enabled').value:
            out = msg.data
        else:
            pwm = pwm_from_linear_thrust(msg.data, self._max)
            out = thrust_at_pwm(pwm, self._volts)
            if self._tau > 1e-3:
                # First-order lag on MEASURED elapsed time, not an assumed
                # command rate. A fixed alpha silently rescales the time
                # constant with the command rate: at the 3400 Hz this actually
                # runs at (34200 commands in 10 s across 8 thrusters), a
                # 0.005 s assumption makes the lag ~7x too slow, and it would
                # change again on a different machine. Wall clock is right here
                # because the commands arrive from ArduSub in lock-step with
                # sim time.
                now = time.monotonic()
                dt = now - self._last[idx]
                self._last[idx] = now
                if 0.0 < dt < 1.0:
                    a = 1.0 - math.exp(-dt / self._tau)
                    out = self._state[idx] + (out - self._state[idx]) * a
            self._state[idx] = out
        m = self._Double()
        m.data = float(out)
        self._pub[idx].publish(m)
        self._n += 1

    def _report(self) -> None:
        if self._n:
            self.get_logger().info(f'[T200 ] {self._n} commands shaped')
            self._n = 0


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = T200Curve()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
