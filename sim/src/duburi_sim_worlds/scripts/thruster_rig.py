#!/usr/bin/env python3

"""Shared gz-transport rig for driving the vehicle's thrusters from a script.

Used by step_response.py and thruster_survey.py. Both bypass ArduSub and talk to
the Thruster plugins directly, and both were unreliable until this grew two
pieces of care that are worth keeping:

*Hold the Node.* A gz-transport subscription lives and dies with the Node object
that created it. A Node passed as a bare constructor argument is collected as
soon as the constructor returns, and the subscription goes quiet with no error.

*Wait for discovery, do not sleep for it.* gz-transport silently drops published
messages until the remote subscriber has been discovered, and on a host with
several network interfaces that can take many seconds and does not complete for
all topics at once. A fixed sleep produces the worst possible failure: some
thrusters connect, others do not, and the run yields a plausible-looking but
wrong number rather than an error. `wait_for_thrusters` blocks on the
publishers' own `has_connections`, which reports exactly the thing that matters
and does not itself depend on discovery succeeding in the reverse direction.
"""

import os
import time

# Everything in this simulator runs on one host, and this box has five IPv4
# interfaces (loopback, wifi, and three docker bridges). Left to itself
# gz-transport picks one and discovery becomes slow and partial. Pinning it to
# loopback makes it deterministic. Set GZ_IP yourself to override, which you
# will need to do if the simulator is ever split across machines.
os.environ.setdefault('GZ_IP', '127.0.0.1')

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

THRUSTERS = tuple(range(1, 9))
HORIZONTAL_THRUSTERS = (1, 2, 3, 4)
VERTICAL_THRUSTERS = (5, 6, 7, 8)

COMMAND_RATE_HZ = 20.0


class ThrusterRig:
    """Thruster command publishers plus ground-truth odometry for one vehicle.

    `vehicle` is the instance name from the course YAML, which is what the
    odometry topic is named after. `model` is the model name, which is what the
    Thruster plugin's <namespace> is set to. They are usually different.
    """

    def __init__(self, vehicle: str = 'duburi',
                 model: str = 'duburi_heavy') -> None:
        self.node = Node()
        self.vehicle = vehicle

        self.pose = None
        self.linear = None
        self.angular = None

        self._ang_vel = {}
        self._pub = {}
        for i in THRUSTERS:
            base = f'/model/{model}/joint/thruster{i}_joint'
            self._pub[i] = self.node.advertise(f'{base}/cmd_thrust', Double)
            self.node.subscribe(Double, f'{base}/ang_vel', self._ang_vel_cb(i))

        if not self.node.subscribe(Odometry, f'/model/{vehicle}/odometry',
                                   self._on_odom):
            raise RuntimeError(
                f'could not subscribe to /model/{vehicle}/odometry')

    def _ang_vel_cb(self, index: int):
        def callback(msg: Double) -> None:
            self._ang_vel[index] = msg.data
        return callback

    def _on_odom(self, msg: Odometry) -> None:
        self.pose = (msg.pose.position.x, msg.pose.position.y,
                     msg.pose.position.z)
        self.linear = (msg.twist.linear.x, msg.twist.linear.y,
                       msg.twist.linear.z)
        self.angular = (msg.twist.angular.x, msg.twist.angular.y,
                        msg.twist.angular.z)

    @property
    def depth(self) -> float:
        return self.pose[2]

    @property
    def surge(self) -> float:
        return self.linear[0]

    @property
    def sway(self) -> float:
        return self.linear[1]

    @property
    def heave(self) -> float:
        return self.linear[2]

    @property
    def yaw_rate(self) -> float:
        return self.angular[2]

    def command(self, newtons) -> None:
        """Send thrust. Accepts a scalar for all eight or a {index: N} mapping."""
        if not isinstance(newtons, dict):
            newtons = dict.fromkeys(THRUSTERS, newtons)
        msg = Double()
        for i in THRUSTERS:
            msg.data = float(newtons.get(i, 0.0))
            self._pub[i].publish(msg)

    def hold(self, seconds: float, newtons, on_sample=None,
             sample_period: float = 1.0) -> None:
        """Keep republishing `newtons` for `seconds`.

        Commands are latched by the Thruster plugin, but republishing means a
        subscriber that reconnects mid-run picks the command back up.
        """
        deadline = time.time() + seconds
        next_sample = time.time() + sample_period
        while time.time() < deadline:
            self.command(newtons)
            if on_sample is not None and time.time() >= next_sample:
                on_sample()
                next_sample += sample_period
            time.sleep(1.0 / COMMAND_RATE_HZ)

    def wait_for_odometry(self, timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.linear is not None:
                return True
            time.sleep(0.1)
        return False

    def wait_until_still(self, thrust=0.0, linear_tol: float = 0.01,
                         angular_tol: float = 0.03,
                         timeout: float = 30.0) -> bool:
        """Coast at `thrust` until the vehicle is actually at rest.

        A fixed coast is not enough. The roll and pitch axes are lightly damped
        and the buoyancy righting moment leaves the vehicle wallowing for a long
        time after a pulse, at a rate comparable to the response being measured.
        Waiting on the real velocity instead makes successive measurements
        independent of each other.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.command(thrust)
            if (max(abs(v) for v in self.linear) < linear_tol
                    and max(abs(w) for w in self.angular) < angular_tol):
                return True
            time.sleep(1.0 / COMMAND_RATE_HZ)
        return False

    def wait_for_thrusters(self, timeout: float = 60.0):
        """Block until every cmd_thrust publisher has a live subscriber.

        Returns the list of thrusters still unconnected at the timeout, empty on
        success. Commanding before this returns loses thrust silently.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            pending = [i for i in THRUSTERS
                       if not self._pub[i].has_connections()]
            if not pending:
                return []
            time.sleep(0.2)
        return [i for i in THRUSTERS if not self._pub[i].has_connections()]

    def ang_vel(self, index: int):
        """Last reported propeller speed, or None if never heard from."""
        return self._ang_vel.get(index)
