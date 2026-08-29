#!/usr/bin/env python3

"""A virtual payload board, so `fire()` can run in simulation.

WHY THIS EXISTS. The payload is the only part of the autonomy stack with no
simulated path at all: `PayloadDriver` talks to an ESP32 over a CH340 serial
port, so `duburi.fire()` -- and with it `align(fire=..., fire_t=...)`, the
mid-hold shot with its `is_new_frame` gating and `fire_pass` fallback -- had
never once executed outside the pool.

THE REAL DRIVER CONNECTS TO THIS, UNMODIFIED. `PayloadDriver.connect(port=...)`
already accepts an explicit device path, so a PTY presents a device node that
the existing `pyserial` code opens exactly as it opens the CH340: same
`Serial()` setup, same DTR/RTS handling, same `VERIFY_BYTE` probe, same
single-digit write, same reconnect path. Nothing in `duburi_control` changes,
which is the entire point -- a sim-only fire path would test sim code rather
than the code that flies. Verified against the unmodified driver: `connect()`
returns True, its verify byte arrives as b'0', and `fire(1)`/`fire(3)` arrive
as b'1'/b'3'.

WHAT A SHOT DOES. Each fire publishes on a ROS topic, so a mission or a test
can assert it happened, AND spawns a body in Gazebo:

  ch 1, 2  torpedo   launched forward from the vehicle's nose
  ch 3, 4  dropper   released from underneath, sinks

The projectile is then an ordinary rigid body: it flies, it is dragged, and it
COLLIDES. That is deliberate -- "did the torpedo go through the opening" is
answered by where it ends up, not by a log line saying a byte was written.

TWO GAZEBO CONSTRAINTS SHAPE THIS, and both were found by measurement:

*A spawned model cannot be given an initial velocity.* `EntityFactory` carries
a pose and nothing else. So the launch is a real force applied for a real burn
time through `ApplyLinkWrench` (world plugin, added in the world template),
after which the projectile coasts under physics.

*Buoyancy is a whitelist read once at world load.* A model not named in it
sinks like a stone -- measured, a projectile spawned at 1.0 m was on the 2.1 m
floor inside 0.5 s. Runtime spawns therefore reuse a FIXED pool of names
(`payload_shot_0..N`) that `gen_world.py` bakes into every world. `SHOT_SLOTS`
here must match `PAYLOAD_SHOT_SLOTS` there.
"""

from __future__ import annotations

import math
import os
import pty
import threading
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.odometry_pb2 import Odometry as GzOdometry
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import Int32

# Must match PAYLOAD_SHOT_SLOTS in duburi_sim_worlds/scripts/gen_world.py --
# a name outside the baked whitelist spawns a projectile with no buoyancy.
SHOT_SLOTS = 12

FIRED_TOPIC = '/duburi/sim/payload/fired'

# 1/2 torpedo, 3/4 dropper -- the same mapping as PayloadDriver.CHANNEL_NAMES.
TORPEDO_CHANNELS = (1, 2)
DROPPER_CHANNELS = (3, 4)

# Torpedo: 40 mm x 180 mm, and the mass is set from the DISPLACEMENT rather
# than picked. This cylinder displaces 226.2 g of water, so 228 g is 1.8 g
# negative -- it sinks, but at 0.08 m/s^2, which is ~18 mm of drop over a 1.5 m
# shot.
#
# The margin has to be this fine because THE PROJECTILE HAS NO DRAG. It carries
# no hydrodynamics plugin, so nothing limits its sink rate: an earlier 235 g
# round (9 g negative) accelerated to the pool floor in about a second and the
# shot was meaningless. Measured against this exact geometry: 100 g surfaced,
# 226 g held station (+17 mm in 2 s), 235 g fell 811 mm in 2 s -- which is
# free acceleration at the predicted 0.38 m/s^2, not a buoyancy failure.
_TORPEDO_R, _TORPEDO_L, _TORPEDO_M = 0.02, 0.18, 0.228
# Dropper marker: a 40 mm ball that displaces 34 g and masses 60 g, so it sinks
# and stays where it lands, which is what gets scored.
_DROPPER_R, _DROPPER_M = 0.02, 0.06


def _projectile_sdf(name: str, pose: str, kind: str) -> str:
    if kind == 'torpedo':
        geom = (f'<cylinder><radius>{_TORPEDO_R}</radius>'
                f'<length>{_TORPEDO_L}</length></cylinder>')
        mass, rgba = _TORPEDO_M, '1 0.35 0 1'
    else:
        geom = f'<sphere><radius>{_DROPPER_R}</radius></sphere>'
        mass, rgba = _DROPPER_M, '0.1 0.8 0.3 1'
    # A solid-ish inertia. It only has to be self-consistent and non-zero: the
    # projectile's tumbling is not a scored quantity, where it lands is.
    i = max(1e-5, mass * 0.01 ** 2)
    # DRAG, and it is not decoration. Without it nothing limits the sink rate:
    # a projectile 5.6 g negative accelerated the whole way down and reached
    # the floor mid-flight, which made the shot meaningless and looked exactly
    # like a buoyancy bug. Quadratic coefficients are 0.5*rho*Cd*A for this
    # body -- broadside 0.0036 m^2 at Cd 1.0, axial 0.00126 m^2 at Cd 0.2 --
    # so the same 5.6 g now settles at a terminal 0.17 m/s instead.
    broadside = -0.5 * 1000.0 * 1.0 * (2 * _TORPEDO_R * _TORPEDO_L)
    axial = -0.5 * 1000.0 * 0.1 * (math.pi * _TORPEDO_R ** 2)
    # THE COEFFICIENTS GO ON BODY AXES, AND THE ROUND IS PITCHED 90 deg.
    # An SDF cylinder's length runs along its own z, so a torpedo lying along
    # the flight path is spawned with pitch = pi/2 -- which puts body +z along
    # the flight and body +x pointing DOWN. Written the obvious way round, the
    # streamlined coefficient ended up resisting the sink and the broadside one
    # resisting the flight: measured, the shot travelled 0.14 m in 2 s where it
    # had covered 2.28 m with no drag at all. Body z = flight, body x = depth.
    drag = (f'    <xUabsU>{broadside:.4g}</xUabsU>\n'
            f'    <yVabsV>{broadside:.4g}</yVabsV>\n'
            f'    <zWabsW>{axial:.4g}</zWabsW>\n'
            f'    <kPabsP>-0.01</kPabsP>\n'
            f'    <mQabsQ>-0.01</mQabsQ>\n'
            f'    <nRabsR>-0.01</nRabsR>')
    return f'''<?xml version="1.0"?><sdf version="1.9">
<model name="{name}"><pose>{pose}</pose>
  <link name="body">
    <inertial><mass>{mass}</mass><inertia>
      <ixx>{i:.6g}</ixx><iyy>{i:.6g}</iyy><izz>{i:.6g}</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="collision"><geometry>{geom}</geometry></collision>
    <visual name="visual"><geometry>{geom}</geometry>
      <material><diffuse>{rgba}</diffuse><ambient>{rgba}</ambient></material>
    </visual>
  </link>
  <plugin filename="gz-sim-hydrodynamics-system"
          name="gz::sim::systems::Hydrodynamics">
    <link_name>body</link_name>
{drag}
  </plugin>
</model></sdf>'''


class PayloadSim(Node):
    def __init__(self) -> None:
        super().__init__('payload_sim')
        self.declare_parameter('world', 'sauvc26_final')
        self.declare_parameter('vehicle', 'duburi')
        self.declare_parameter('port_link', '')
        # Launch impulse. 7.6 N for 0.12 s on 228 g is a 4.0 m/s muzzle
        # velocity, which drag then bleeds off over the next couple of metres.
        self.declare_parameter('launch_force_n', 7.6)
        self.declare_parameter('burn_time_s', 0.12)
        # Where a round leaves the hull, in body axes (x forward, z up).
        #
        # THE ROUND MUST CLEAR THE HULL'S COLLISION BOX OR THE VEHICLE PUNTS
        # IT. At 0.30 m the 180 mm round spanned 0.21-0.39 m from the vehicle
        # centre while the hull box reaches 0.229 m, so every shot began with a
        # contact impulse: measured, it dived 0.81 m in 1.25 s instead of the
        # 0.07 m its 1.8 g of negative buoyancy predicts, and it looked exactly
        # like a buoyancy failure. 0.40 m = hull 0.229 + round 0.09 + margin.
        self.declare_parameter('muzzle_forward_m', 0.40)
        self.declare_parameter('muzzle_down_m', 0.05)
        self.declare_parameter('dropper_down_m', 0.25)

        self._world = self.get_parameter('world').value
        self._vehicle = self.get_parameter('vehicle').value

        self._pose = None          # (x, y, z)
        self._yaw = 0.0
        self._slot = 0
        self._lock = threading.Lock()

        self._gz = GzNode()
        self._pub_wrench = self._gz.advertise(
            f'/world/{self._world}/wrench/persistent', EntityWrench)
        self._pub_clear = self._gz.advertise(
            f'/world/{self._world}/wrench/clear', Entity)

        # Vehicle pose comes from GAZEBO, not from the ROS ground-truth topic.
        # That topic only exists when the ros_gz bridge is running, and the
        # first version of this node subscribed to it and then declined to
        # spawn anything under `bridge:=false` -- the fire reached the serial
        # port and the ROS topic, and no projectile appeared. Gazebo publishes
        # the same odometry whenever the world is up, which is exactly when a
        # projectile can be spawned at all.
        if not self._gz.subscribe(GzOdometry, f'/model/{self._vehicle}/odometry',
                                  self._on_odom):
            self.get_logger().error(
                f'[PAYLOAD-SIM] could not subscribe to '
                f'/model/{self._vehicle}/odometry -- fires will be recorded '
                f'but nothing will spawn')
        self._fired_pub = self.create_publisher(Int32, FIRED_TOPIC, 10)

        self._master, slave = pty.openpty()
        self._port = os.ttyname(slave)
        # Hold the slave fd open. Closing it makes every subsequent read on the
        # master raise EIO the moment the client disconnects, so the board would
        # work exactly once.
        self._slave_fd = slave
        self._link = self._make_link(self.get_parameter('port_link').value)

        self.get_logger().info(
            f'[PAYLOAD-SIM] virtual board on {self._port}'
            + (f' (symlink {self._link})' if self._link else '')
            + ' -- start the manager with '
            + f'payload_port:={self._link or self._port}')

        threading.Thread(target=self._read_loop, daemon=True).start()

    def _make_link(self, requested: str) -> str:
        """A stable path, since the PTY number changes every launch."""
        link = requested or os.path.join(
            f'/tmp/duburi-{os.environ.get("USER", "user")}', 'payload')
        try:
            os.makedirs(os.path.dirname(link), exist_ok=True)
            if os.path.islink(link) or os.path.exists(link):
                os.unlink(link)
            os.symlink(self._port, link)
            return link
        except OSError as exc:
            self.get_logger().warn(f'[PAYLOAD-SIM] no symlink at {link}: {exc}')
            return ''

    def _on_odom(self, msg: GzOdometry) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self._pose = (p.x, p.y, p.z)
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

    def _read_loop(self) -> None:
        while True:
            try:
                data = os.read(self._master, 16)
            except OSError:
                time.sleep(0.05)
                continue
            for byte in data:
                # '0' is PayloadDriver.VERIFY_BYTE, the link probe. The real
                # firmware ignores it and so does this.
                if 0x31 <= byte <= 0x34:
                    self._fire(byte - 0x30)

    def _fire(self, channel: int) -> None:
        kind = 'torpedo' if channel in TORPEDO_CHANNELS else 'dropper'
        self.get_logger().info(f'[PAYLOAD-SIM] ch={channel} {kind} FIRED')
        self._fired_pub.publish(Int32(data=channel))
        if self._pose is None:
            self.get_logger().warn(
                '[PAYLOAD-SIM] no ground truth yet -- fire recorded on the '
                'topic but no projectile spawned')
            return
        threading.Thread(target=self._spawn, args=(kind,), daemon=True).start()

    def _spawn(self, kind: str) -> None:
        with self._lock:
            name = f'payload_shot_{self._slot % SHOT_SLOTS}'
            self._slot += 1
        x, y, z = self._pose
        yaw = self._yaw
        cy, sy = math.cos(yaw), math.sin(yaw)

        if kind == 'torpedo':
            fwd = float(self.get_parameter('muzzle_forward_m').value)
            down = float(self.get_parameter('muzzle_down_m').value)
            pose = (f'{x + fwd * cy:.6g} {y + fwd * sy:.6g} {z - down:.6g} '
                    f'0 1.5708 {yaw:.6g}')
        else:
            down = float(self.get_parameter('dropper_down_m').value)
            pose = f'{x:.6g} {y:.6g} {z - down:.6g} 0 0 0'

        # Recycle the slot. A create with a live name is refused, and reusing a
        # name is the only way a spawned body can be buoyant (see module note).
        self._gz.request(f'/world/{self._world}/remove',
                         self._entity(name), Entity, Boolean, 2000)

        req = EntityFactory()
        req.sdf = _projectile_sdf(name, pose, kind)
        req.allow_renaming = False
        ok, rep = self._gz.request(f'/world/{self._world}/create',
                                   req, EntityFactory, Boolean, 4000)
        if not (ok and rep.data):
            self.get_logger().warn(f'[PAYLOAD-SIM] spawn of {name} refused')
            return
        if kind != 'torpedo':
            return

        force = float(self.get_parameter('launch_force_n').value)
        burn = float(self.get_parameter('burn_time_s').value)
        w = EntityWrench()
        w.entity.name = name
        w.entity.type = Entity.MODEL
        w.wrench.force.x = force * cy
        w.wrench.force.y = force * sy
        self._pub_wrench.publish(w)
        time.sleep(burn)
        self._pub_clear.publish(self._entity(name))

    @staticmethod
    def _entity(name: str) -> Entity:
        e = Entity()
        e.name = name
        e.type = Entity.MODEL
        return e

    def destroy_node(self) -> bool:
        if self._link and os.path.islink(self._link):
            try:
                os.unlink(self._link)
            except OSError:
                pass
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = PayloadSim()
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
