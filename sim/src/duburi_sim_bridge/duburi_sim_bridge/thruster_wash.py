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

JET SPEED COMES FROM THRUST, NOT FROM HULL SPEED
------------------------------------------------
Three earlier bugs, all of which produced a node that ran, logged, and pushed
nothing:

  * the first version used HULL SPEED as the jet speed. Wrong quantity: wash is
    set by the thrusters, not by how fast the hull travels.
  * the second called it a "slipstream multiple of the free stream" -- which is
    still hull speed, wearing a coefficient, and it kept `if speed > 0.05` as
    the gate. So the very experiment the module asked for (park the hull
    upstream of a prop and hold thrust) would have measured NOTHING: parked
    means speed 0 means the gate returns before any force is computed. Two
    contradictory statements sat four lines apart in this file for a round.
  * `gz.transport13.Node` HAS NO `publish` METHOD. `node.publish(...)` raises
    AttributeError inside the timer callback, where it is swallowed. Two of the
    three A/B runs measured a node that had never published a byte.
  * `msg.entity.type = 2  # LINK`. In gz.msgs.Entity, 2 is MODEL and LINK is 3,
    so every wrench addressed a MODEL named "slalom_1::pipe_centre" -- which
    does not exist -- and ApplyLinkWrench dropped it without a word. THIS is
    why the node pushed nothing even once the jet speed was right. It was
    separated from "the prop is stiff" by applying the same wrench to a
    known-free body: the collectible flew, the pipe did not, so the fault was
    in the address and not in the physics.

It now reads the four HORIZONTAL thrusters' shaped commands off gz and uses
momentum theory for the fully developed slipstream:

    v_jet = sqrt(2 |F| / (rho A))

`F` is the net horizontal thrust VECTOR in the world frame, so the jet also
points the right way when the hull is strafing instead of surging, and a hull
holding station against a current -- zero speed, full thrust -- washes at full
strength, which is the case the whole model exists for.

The four vectored thrusters sit at yaw -45 / -135 / +45 / +135 with joint axis
(0, 0, -1) in a child frame posed rpy (-90, 90, yaw). Working that through
(SDF rpy is Rz.Ry.Rx) gives a body-frame thrust direction of
(sin yaw, -cos yaw, 0) per thruster -- see `_AXES`. That derivation is
CHECKED AT RUNTIME, not trusted: `wash_debug` logs the body-frame net thrust,
and a straight forward drive must show it along +x at about 4 x 0.707 x the
per-thruster force. Getting the sign wrong would otherwise be silent, which is
this file's recurring failure mode.

The force goes on `/world/<world>/wrench/persistent` and is CLEARED when a prop
leaves the cone. A one-shot publish is not enough (gz-transport drops a publish
made before discovery completes, which this project has already been bitten by
twice), and a persistent wrench that is never cleared leaves a prop accelerating
forever.

MEASURED, PARKED AND THRUSTING (2026-08-31)
-------------------------------------------
Hull pinned 0.7 m upstream of `slalom_1` on `rs_task_slalom`, held at the node's
own reported +103.52 N of net forward thrust for 25 s:

    wash off   0.001 deg
    wash on    7.331 deg     (all three pipes)

All three move equally because `_prop_xy` returns the MODEL's position for every
one of its links, so they share a cone test and a force. That is the documented
simplification, not a measurement artifact.

DO NOT MEASURE THIS OFF `/world/<w>/pose/info`
----------------------------------------------
It reports a link's pose RELATIVE TO ITS MODEL FRAME, and that frame rides the
model's CANONICAL link -- the first link authored, which for the slalom prop is
`pipe_left`. So the pipe that actually swings reads 0.00 forever while its two
neighbours report its counter-rotation. Push one pipe and two others appear to
move; push all three and every reading collapses to ~0 PRECISELY BECAUSE it is
working. Both symptoms cost this round a long detour, and the tell was that two
different bodies read an IDENTICAL 49.48 deg -- two things reading exactly alike
are usually one thing. Compose the model pose with the link pose, or watch the
model entity's own pose. `dynamic_pose/info` uses the same convention and does
NOT save you.
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

GROUND_TRUTH = '/duburi/sim/ground_truth'
WATER_DENSITY = 1000.0

# Body-frame thrust direction of each vectored thruster, from the model SDF:
# joint axis (0, 0, -1) in a child link posed rpy (-90, 90, yaw), and SDF rpy is
# Rz.Ry.Rx, which reduces to (sin yaw, -cos yaw, 0). Thrusters 5-8 are vertical
# and contribute nothing to a horizontal jet.
_AXES = {
    1: (math.sin(math.radians(-45.0)), -math.cos(math.radians(-45.0))),
    2: (math.sin(math.radians(-135.0)), -math.cos(math.radians(-135.0))),
    3: (math.sin(math.radians(45.0)), -math.cos(math.radians(45.0))),
    4: (math.sin(math.radians(135.0)), -math.cos(math.radians(135.0))),
}


def net_body_thrust(thrust):
    """Net horizontal thrust in the BODY frame, in newtons, as (x, y).

    Summed as a VECTOR, not as magnitudes: four thrusters at EQUAL positive
    command are a pure YAW on a vectored X-frame -- the axes cancel and there is
    no jet at all. That is the right answer, it is the one a scalar sum gets
    wrong, and it is not hypothetical: the A/B rig for this module drove four
    equal commands, and the node correctly reported 0.00 N while the rig's
    author expected a forward drive. Forward is t1/t2 astern and t3/t4 ahead.
    """
    return (sum(thrust[i] * _AXES[i][0] for i in _AXES),
            sum(thrust[i] * _AXES[i][1] for i in _AXES))


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
        self.declare_parameter('rate_hz', 5.0)
        # The model name the thruster topics are namespaced under -- the same
        # trap `t200_curve` documents: ArduPilotPlugin bakes the MODEL name into
        # its topic while a course names the INSTANCE something else, and
        # subscribing to the wrong one is silent.
        self.declare_parameter('vehicle', 'duburi_heavy')
        # Below this there is no jet worth applying. 2 N of net horizontal
        # thrust is trim, not drive.
        self.declare_parameter('min_thrust_n', 2.0)
        # Logs the body-frame net thrust so the _AXES derivation is CHECKED
        # against a straight forward drive rather than assumed.
        self.declare_parameter('wash_debug', False)
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
        self._thrust = {i: 0.0 for i in _AXES}
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

        # THE FOUR HORIZONTAL THRUSTERS, off the SHAPED topic. `cmd_thrust` is
        # what `t200_curve` writes (deadband, reverse asymmetry, spin-up lag);
        # `cmd_thrust_linear` upstream of it is what ArduSub asked for, which is
        # not what the water sees. Reading the wrong one would overstate the jet
        # exactly where the T200 curve costs the most -- near the deadband.
        from gz.msgs10.double_pb2 import Double
        vehicle = self.get_parameter('vehicle').value
        for i in _AXES:
            topic = f'/model/{vehicle}/joint/thruster{i}_joint/cmd_thrust'
            if not self._gz.subscribe(Double, topic,
                                      lambda m, k=i: self._on_thrust(k, m)):
                self.get_logger().error(
                    f'[WASH ] could not subscribe to {topic} -- no wash')

        self._pub = self._gz.advertise(
            f'/world/{world}/wrench/persistent', EntityWrench)
        self._clear_pub = self._gz.advertise(
            f'/world/{world}/wrench/clear', Entity)
        if not self._pub or not self._clear_pub:
            self.get_logger().error('[WASH ] could not advertise the wrench '
                                    'topics -- no wash will be applied')

    def _on_thrust(self, idx: int, msg) -> None:
        self._thrust[idx] = float(msg.data)

    def _net_thrust(self):
        """Net horizontal thrust in the WORLD frame, in newtons."""
        bx, by = net_body_thrust(self._thrust)
        yaw = self._pose[3]
        c, sn = math.cos(yaw), math.sin(yaw)
        return (bx * c - by * sn, bx * sn + by * c, bx, by)

    def _on_truth(self, msg) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # yaw only: the wash direction that matters is astern in the horizontal
        # plane, and a hull pitching a few degrees does not change which prop
        # is behind it.
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self._pose = (p.x, p.y, p.z, yaw)

    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        if not self.get_parameter('enabled').value or self._pub is None:
            return
        if self._pose is None:
            return
        x, y, z, yaw = self._pose
        fx, fy, bx, by = self._net_thrust()
        thrust = math.hypot(fx, fy)
        if self.get_parameter('wash_debug').value:
            self.get_logger().info(
                f'[WASH ] body thrust ({bx:+.2f}, {by:+.2f}) N  '
                f'|F| {thrust:.2f} N  per-thruster '
                + ' '.join(f'{self._thrust[i]:+.1f}' for i in sorted(_AXES)))
        # THE JET GOES OPPOSITE THE THRUST, not merely astern of the hull's
        # heading: a strafing vehicle washes sideways, and the two directions
        # are 90 degrees apart when it does.
        if thrust > 1e-6:
            ax, ay = -fx / thrust, -fy / thrust
        else:
            ax, ay = -math.cos(yaw), -math.sin(yaw)

        cone = math.radians(float(self.get_parameter('cone_deg').value))
        reach = float(self.get_parameter('reach_m').value)
        area = float(self.get_parameter('source_area_m2').value)
        cda = float(self.get_parameter('prop_cda').value)

        still = set(self._pushed)
        # Momentum theory: a disc of area A producing thrust T leaves a fully
        # developed slipstream at sqrt(2T / rho A). This is the line that makes
        # a hull PUSHING but not MOVING wash at full strength -- the case the
        # old `if speed > 0.05` gate silently excluded, and the same case as the
        # experiment this module kept asking someone to run.
        u_jet = math.sqrt(2.0 * thrust / (WATER_DENSITY * area))
        if thrust > float(self.get_parameter('min_thrust_n').value):
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
                u = u_jet * area / (area + 0.35 * r * r)
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
        # `Entity.LINK`, NOT the literal 2. In gz.msgs.Entity the enum is
        # NONE=0 LIGHT=1 MODEL=2 LINK=3 -- so `type = 2  # LINK` addressed a
        # MODEL called "slalom_1::pipe_centre", which does not exist, and
        # ApplyLinkWrench dropped every message in silence. That is the whole
        # reason this node "pushed nothing" through three A/B rounds, and it
        # took applying a wrench to a KNOWN-FREE body (a collectible, which
        # flew) to separate a broken mechanism from a stiff prop. Use the
        # symbol; a magic number with a comment claiming otherwise is exactly
        # how this survived.
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        msg = EntityWrench()
        msg.entity.name = scoped
        msg.entity.type = Entity.LINK
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        self._pub.publish(msg)

    def _clear(self, scoped: str) -> None:
        from gz.msgs10.entity_pb2 import Entity
        msg = Entity()
        msg.name = scoped
        msg.type = Entity.LINK
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
