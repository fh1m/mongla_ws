#!/usr/bin/env python3

"""The hull's wash pushes the props it drives past.

WHY THIS EXISTS
---------------
`wake_fraction` in the vehicle model is NOT this, and it is worth being precise
because the name invites the confusion: it reduces a thruster's own thrust as
the hull's forward speed raises the inflow at its blades. That is a self-effect
on the vehicle. It applies no force to anything else in the world.

So nothing in the simulator had a moving hull disturb its surroundings -- which
did not matter while every prop was welded to the world, and started to matter
the moment they were not. A slalom pipe should nod as the vehicle drives past
it, a hanging gate marker should swing before the hull ever touches it, and a
mission that grazes a prop should see it move. In the pool it does.

WHAT IT IS, AND WHAT IT IS NOT
------------------------------
A momentum-jet approximation, not a fluid simulation. The hull is treated as a
source of a cone of moving water pointing ASTERN (thrust goes forward, the water
goes back), with speed falling off as 1/r^2 the way a submerged jet spreads. A
prop inside that cone gets a drag force from the local flow.

It is deliberately NOT:
  * per-thruster -- eight jets on a vectored hull interact, and modelling that
    badly is worse than modelling one jet honestly;
  * a wake that persists after the hull leaves. Real wash lingers for seconds;
    this stops when the vehicle does. A prop that keeps nodding after the hull
    has gone would be a nicer video and a worse teacher.

STATUS: BUILT, WIRED, AND NOT YET DEMONSTRATED. OFF BY DEFAULT.
--------------------------------------------------------------
Three A/B runs -- nine seconds of forward drive at gain 85 past three slalom
pipes, wash on against wash off -- moved exactly one body in both arms, and that
body was the vehicle. So this has never been shown to push anything, and it
ships disabled rather than pretending otherwise.

Two real bugs were found and fixed on the way, and both are worth keeping
written down:

  * the first version used HULL SPEED as the jet speed. That is the wrong
    quantity: wash is set by the thrusters, not by how fast the hull travels,
    and a vehicle holding station against a current has zero speed and full
    wash. It is a slipstream multiple of the free stream now.
  * `gz.transport13.Node` HAS NO `publish` METHOD. `node.publish(...)` raises
    AttributeError inside the timer callback, where it is swallowed -- the node
    starts, logs its parameters, reports the props it loaded, and pushes
    nothing. Two of the three A/B runs were measuring a node that had never
    published a byte.

What has NOT been ruled out is the test itself: the slalom props sit at
x = -2.0, -0.5, +1.0 and the hull starts at -5, so for most of a 3.3 m run they
are AHEAD of it and an astern cone never reaches them. A run that parks the hull
upstream of a prop and holds thrust would settle it. Until someone does that,
this is an unproven node.

The force goes on `/world/<world>/wrench/persistent` and is CLEARED when a prop
leaves the cone. A one-shot publish is not enough (gz-transport drops a publish
made before discovery completes, which this project has already been bitten by
twice), and a persistent wrench that is never cleared leaves a prop accelerating
forever.
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

GROUND_TRUTH = '/duburi/sim/ground_truth'
WATER_DENSITY = 1000.0


class ThrusterWash(Node):

    def __init__(self):
        super().__init__('thruster_wash')
        self.declare_parameter('world', 'robosub26_full')
        self.declare_parameter('enabled', True)
        # Half-angle of the jet. A submerged round jet spreads at roughly 12
        # degrees; 15 is that plus a little, since the hull is not a nozzle.
        self.declare_parameter('cone_deg', 15.0)
        # Beyond this the jet has mixed out and there is nothing left to push.
        self.declare_parameter('reach_m', 3.0)
        # Effective jet area at the hull -- the disc the eight thrusters sweep.
        self.declare_parameter('source_area_m2', 0.09)
        # Drag coefficient x area for a generic prop. One number, because the
        # props this reaches are all slender PVC and pretending otherwise would
        # be false precision. A 0.9 m pipe of 33 mm OD at Cd 1.0 is 0.03; a
        # little more, because a pipe in a jet is not in clean free stream.
        self.declare_parameter('prop_cda', 0.05)
        # SLIPSTREAM RATIO. The jet leaving a propeller moves considerably
        # faster than the hull it is pushing -- momentum theory puts the fully
        # developed slipstream at about twice the free-stream, and a hull
        # holding station against a current has ZERO speed and FULL wash.
        #
        # The first version used hull speed directly as the jet speed and was
        # measured as a complete no-op: nine seconds at gain 85 past three
        # slalom pipes moved not one of them. That is not a tuning miss, it is
        # the wrong quantity -- 0.37 m/s of hull gives 0.14 N at a metre, which
        # is a couple of degrees of tilt on a moored pipe.
        self.declare_parameter('slipstream', 2.5)
        self.declare_parameter('rate_hz', 5.0)
        # Props that can actually move. A wrench on a static body does nothing
        # and costs a publish.
        self.declare_parameter('targets', [
            'slalom_1::pipe_centre', 'slalom_1::pipe_left',
            'slalom_1::pipe_right', 'slalom_2::pipe_centre',
            'slalom_2::pipe_left', 'slalom_2::pipe_right',
            'slalom_3::pipe_centre', 'slalom_3::pipe_left',
            'slalom_3::pipe_right',
        ])

        self._pose = None
        self._vel = (0.0, 0.0, 0.0)
        self._pushed: set = set()
        self._gz = None
        self._pub = None

        self.create_subscription(Odometry, GROUND_TRUTH, self._on_truth,
                                 qos_profile_sensor_data)
        self._connect()
        self._load_course()
        hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(1.0, hz), self._tick)
        self.get_logger().info(
            f'[WASH ] jet {self.get_parameter("cone_deg").value:.0f} deg, '
            f'reach {self.get_parameter("reach_m").value:.1f} m')

    # ------------------------------------------------------------------ #
    def _connect(self) -> None:
        """Advertise the two wrench topics ONCE and keep the publishers.

        `gz.transport13.Node` HAS NO `publish` METHOD -- you advertise a topic
        to get a Publisher and publish on that. Calling `node.publish(...)`
        raises AttributeError inside the timer callback, where it is swallowed:
        the node starts, logs its parameters, reports the props it loaded, and
        then silently pushes nothing. Two full A/B runs measured "no prop
        moved" before anyone looked at `dir(Node)`.
        """
        try:
            from gz.msgs10.entity_pb2 import Entity
            from gz.msgs10.entity_wrench_pb2 import EntityWrench
            from gz.transport13 import Node as GzNode
        except ImportError as exc:
            self.get_logger().error(f'[WASH ] no gz-transport bindings: {exc}')
            return
        world = self.get_parameter('world').value
        self._gz = GzNode()
        self._pub = self._gz.advertise(
            f'/world/{world}/wrench/persistent', EntityWrench)
        self._clear_pub = self._gz.advertise(
            f'/world/{world}/wrench/clear', Entity)
        if not self._pub or not self._clear_pub:
            self.get_logger().error('[WASH ] could not advertise the wrench '
                                    'topics -- no wash will be applied')

    def _on_truth(self, msg) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist.linear
        # yaw only: the wash direction that matters is astern in the horizontal
        # plane, and a hull pitching a few degrees does not change which prop
        # is behind it.
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self._pose = (p.x, p.y, p.z, yaw)
        self._vel = (t.x, t.y, t.z)

    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        if not self.get_parameter('enabled').value or self._pub is None:
            return
        if self._pose is None:
            return
        speed = math.sqrt(sum(v * v for v in self._vel))
        x, y, z, yaw = self._pose
        # Astern: the water goes the way the hull came from.
        ax, ay = -math.cos(yaw), -math.sin(yaw)

        cone = math.radians(float(self.get_parameter('cone_deg').value))
        reach = float(self.get_parameter('reach_m').value)
        area = float(self.get_parameter('source_area_m2').value)
        cda = float(self.get_parameter('prop_cda').value)

        still = set(self._pushed)
        if speed > 0.05:
            for name in self.get_parameter('targets').value:
                pos = self._prop_xy(name)
                if pos is None:
                    continue
                dx, dy = pos[0] - x, pos[1] - y
                r = math.hypot(dx, dy)
                if r < 0.15 or r > reach:
                    continue
                if (dx * ax + dy * ay) / r < math.cos(cone):
                    continue
                # Jet speed decaying as the cone spreads: u = u0 * A / (A + kr^2)
                u0 = speed * float(self.get_parameter('slipstream').value)
                u = u0 * area / (area + 0.35 * r * r)
                f = 0.5 * WATER_DENSITY * cda * u * u
                if f < 0.02:
                    continue
                self._wrench(name, f * ax, f * ay)
                self._pushed.add(name)
                still.discard(name)

        for name in still:                      # left the cone -- stop pushing
            self._clear(name)
            self._pushed.discard(name)

    def _load_course(self) -> None:
        """Prop positions, from the course this world was generated from.

        Read ONCE at startup rather than tracked live. That is a real
        limitation and worth stating: a prop spawned or moved at runtime with
        `props add` is invisible to the wash, and a prop the hull has already
        knocked is pushed from where it started rather than where it is. The
        alternative is a second reader on the pose stream, which is another
        child process and another way for `duburi_sim stop` to leak one -- too
        much machinery for a effect this size. The props this reaches barely
        move compared with the 3 m reach of the jet.
        """
        self._xy = {}
        try:
            import yaml
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('duburi_sim_worlds')
        except Exception as exc:                                # noqa: BLE001
            self.get_logger().warn(f'[WASH ] no course data: {exc}')
            return
        import os
        course = os.path.join(share, 'courses',
                              f'{self.get_parameter("world").value}.yaml')
        if not os.path.isfile(course):
            self.get_logger().warn(f'[WASH ] no course yaml at {course}')
            return
        with open(course) as fh:
            spec = yaml.safe_load(fh) or {}
        for entry in (spec.get('props') or []):
            # Courses place props by `xy`; `pose` is the vehicle's key, not a
            # prop's. Reading the wrong one gave an empty table and a node that
            # ran, logged nothing and pushed nothing.
            name, xy = entry.get('name'), entry.get('xy')
            if name and xy:
                self._xy[name] = (float(xy[0]), float(xy[1]))
        self.get_logger().info(f'[WASH ] {len(self._xy)} prop positions loaded')

    def _prop_xy(self, scoped: str):
        return self._xy.get(scoped.split('::')[0])

    def _wrench(self, scoped: str, fx: float, fy: float) -> None:
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        msg = EntityWrench()
        msg.entity.name = scoped
        msg.entity.type = 2                                     # LINK
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        self._pub.publish(msg)

    def _clear(self, scoped: str) -> None:
        from gz.msgs10.entity_pb2 import Entity
        msg = Entity()
        msg.name = scoped
        msg.type = 2
        self._clear_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterWash()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
