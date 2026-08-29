#!/usr/bin/env python3

"""Water current: the disturbance that makes station-keeping hard.

WHY. A real pool is not still. Filtration circulates it, other vehicles stir it,
and thermal layers drift. The AUV is pushed off station continuously, and every
station-keeping behaviour in the autonomy stack exists to fight that -- most
explicitly `vision.ki_lat`, a lateral integral term whose entire purpose is to
null a steady current during a hold.

Until now the sim had zero current, so that term had nothing to fight. A hold
that looked perfect in sim was being graded against an easier problem than the
pool poses, which is exactly the direction a training environment must not err.

WHAT IT DRIVES. Gazebo's own Hydrodynamics plugin already accepts a current: it
subscribes to `/model/<model>/ocean_current` (gz.msgs.Vector3d) and folds the
vector into the relative-velocity term of the Fossen model. So the current is
applied by the SAME hydrodynamics that produce drag -- it is not a fudge force
bolted on beside them. The hook existed and nothing was publishing to it.

WHAT THIS IS NOT. It is a flow field the vehicle reacts to, not a fluid
simulation. It reproduces the disturbance, not its cause: there is no
circulation pattern, no wall effect, no wake. Tuning a station-keep or an
integral gain against it transfers to the pool; validating a CFD model against
it does not.

THE MODEL. A steady set plus slow gusting. The gust bandwidth is deliberately
LOW (default 0.08 Hz, ~12 s period): a real pool's disturbance is slow drift,
and a controller can meaningfully fight it. High-frequency noise would be
neither realistic nor useful -- it would just add jitter no controller can
track, and would make the sim harder in a way that teaches nothing.
"""

from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node


class WaterCurrent(Node):
    def __init__(self) -> None:
        super().__init__('water_current')

        self.declare_parameter('vehicle', 'duburi_heavy')
        # Steady set, in WORLD frame, m/s. The hydrodynamics plugin expects a
        # world-frame vector and handles the body-frame projection itself.
        self.declare_parameter('speed', 0.05)
        self.declare_parameter('heading_deg', 0.0)
        self.declare_parameter('vertical', 0.0)
        # Gusting, as a fraction of `speed`, at `gust_hz`.
        self.declare_parameter('gust', 0.4)
        self.declare_parameter('gust_hz', 0.08)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('seed', 0)
        self.declare_parameter('enabled', True)

        from gz.transport13 import Node as GzNode
        from gz.msgs10.vector3d_pb2 import Vector3d

        self._V = Vector3d
        self._gz = GzNode()
        # THE TOPIC IS BARE `/ocean_current`, NOT NAMESPACED.
        #
        # The plugin's own docs and its `<namespace>` element suggest
        # /model/<name>/ocean_current, and that topic even appears in `gz topic
        # -l` because this node was publishing to it. But `gz topic -i` on the
        # two shows the truth: the namespaced one has a publisher and NO
        # subscriber, while bare /ocean_current has a subscriber and no
        # publisher. Measured, after a 0.12 m/s current produced 6 mm of drift
        # in 40 s -- i.e. none.
        #
        # This is the failure mode this simulator keeps producing: everything
        # starts, the topic exists, the numbers look right, and the two halves
        # are not connected. `gz topic -i` is the check that distinguishes
        # "publishing" from "being heard".
        model = self.get_parameter('vehicle').value
        self._topic = '/ocean_current'
        self._pub = self._gz.advertise(self._topic, Vector3d)

        seed = int(self.get_parameter('seed').value)
        self._rng = random.Random(seed if seed else None)
        # Three independent gust phases so the gust is not a pure sinusoid the
        # controller could learn to predict.
        self._phase = [self._rng.uniform(0, 2 * math.pi) for _ in range(3)]
        self._t = 0.0

        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

        spd = float(self.get_parameter('speed').value)
        hdg = float(self.get_parameter('heading_deg').value)
        self.get_logger().info(
            f'water current -> {self._topic}: {spd:.3f} m/s at {hdg:.0f} deg, '
            f'gust {float(self.get_parameter("gust").value) * 100:.0f} % '
            f'@ {float(self.get_parameter("gust_hz").value):.2f} Hz'
            + ('' if spd else '  (STILL WATER -- set speed to enable)'))

    def _tick(self) -> None:
        if not self.get_parameter('enabled').value:
            return
        self._t += self._dt
        speed = float(self.get_parameter('speed').value)
        gust = float(self.get_parameter('gust').value)
        f = float(self.get_parameter('gust_hz').value)

        # Sum of three slow sinusoids at incommensurate rates: quasi-random
        # drift without the sharp edges of white noise.
        w = 2.0 * math.pi * f
        g = sum(math.sin(w * self._t * m + p)
                for m, p in zip((1.0, 1.7, 2.9), self._phase)) / 3.0
        mag = speed * (1.0 + gust * g)

        hdg = math.radians(float(self.get_parameter('heading_deg').value))
        # Let the direction wander a little too; a pool's flow is not a
        # perfectly fixed bearing.
        hdg += math.radians(12.0) * gust * math.sin(w * self._t * 0.6 + self._phase[0])

        m = self._V()
        m.x = mag * math.cos(hdg)
        m.y = mag * math.sin(hdg)
        m.z = float(self.get_parameter('vertical').value)
        self._pub.publish(m)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = WaterCurrent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
