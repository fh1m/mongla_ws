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

# TORPEDO, to the rulebook (p. 64): "Each torpedo must fit within a box 2.0"
# square and 6" long (51 x 51 x 152 mm)" and "must weigh no more than 2.0 lbs
# (0.91 kg) in air". So a 51 mm body, 152 mm long.
_TORPEDO_R = 0.0255
_TORPEDO_L = 0.152
_TORPEDO_NOSE = 0.045
# Mass is DERIVED FROM DISPLACEMENT, never picked. This body displaces 311 g of
# water, so 325 g is 14 g negative: it sinks at 0.22 m/s^2 once added mass is
# counted, which is 17 mm of drop across a 1.5 m shot. Well inside the 0.91 kg
# limit. An earlier guess of 790 g was 479 g negative and plummeted -- the
# arithmetic catches that in one line and the eye does not.
_TORPEDO_M = 0.325
# Streamlined nose-on, bluff broadside.
_TORPEDO_CD_AXIAL = 0.12
_TORPEDO_CD_CROSS = 1.0

# DROPPER: same rulebook box, but the shape stays a ball -- it is released, not
# fired, and a sphere is what falls predictably into a bin. Displaces 65 g;
# 118 g sinks and stays where it lands, which is what gets scored.
_DROPPER_R = 0.025
_DROPPER_M = 0.118

WATER_DENSITY = 1000.0

def _torpedo_visual() -> str:
    """A torpedo that LOOKS like one: tapered nose, body, tail fins.

    Not decoration. The operator judges a shot by watching it, and a bare
    cylinder gives no cue which way the round points -- which is exactly what
    you need to see when a shot leaves the tube mis-aimed or tumbles in flight.
    """
    body_l = _TORPEDO_L - _TORPEDO_NOSE
    rgba = '0.95 0.35 0.05 1'
    out = []
    out.append(
        '      <visual name="body">\n'
        f'        <pose>0 0 {-_TORPEDO_NOSE / 2.0:.6g} 0 0 0</pose>\n'
        f'        <geometry><cylinder><radius>{_TORPEDO_R}</radius>'
        f'<length>{body_l:.6g}</length></cylinder></geometry>\n'
        f'        <material><diffuse>{rgba}</diffuse>'
        f'<ambient>{rgba}</ambient></material>\n'
        '      </visual>')
    out.append(
        '      <visual name="nose">\n'
        f'        <pose>0 0 {body_l / 2.0:.6g} 0 0 0</pose>\n'
        f'        <geometry><sphere><radius>{_TORPEDO_R}</radius>'
        '</sphere></geometry>\n'
        '        <material><diffuse>0.95 0.95 0.9 1</diffuse>'
        '<ambient>0.95 0.95 0.9 1</ambient></material>\n'
        '      </visual>')
    for i in range(4):
        ang = i * math.pi / 2.0
        fr = _TORPEDO_R * 0.8
        out.append(
            f'      <visual name="fin{i}">\n'
            f'        <pose>{fr * math.cos(ang):.6g} {fr * math.sin(ang):.6g} '
            f'{-body_l / 2.0 - _TORPEDO_NOSE / 2.0 + 0.025:.6g} '
            f'0 0 {ang:.6g}</pose>\n'
            f'        <geometry><box><size>{_TORPEDO_R:.6g} 0.003 0.04</size>'
            '</box></geometry>\n'
            '        <material><diffuse>0.12 0.12 0.12 1</diffuse>'
            '<ambient>0.12 0.12 0.12 1</ambient></material>\n'
            '      </visual>')
    return '\n'.join(out)


def _projectile_sdf(name: str, pose: str, kind: str) -> str:
    """One fired body, with the hydrodynamics that decide where it lands.

    ADDED MASS IS THE POINT, and it was the stated gap in the first version. A
    body accelerating underwater drags water with it, so it behaves as though
    heavier -- and for a cylinder moving BROADSIDE the added mass is about its
    own displacement, while nose-on it is a tenth of that. Without it a round
    decelerates and turns like an object in air, which is exactly the behaviour
    a vision pipeline would learn to time shots against and then find missing
    in the pool.

    Every coefficient is computed for THIS body: drag as 0.5*rho*Cd*A, added
    mass as a fraction of displaced mass. Nothing is copied from the vehicle.

    THE COEFFICIENTS RIDE BODY AXES AND THE ROUND IS PITCHED 90 deg. An SDF
    cylinder's length runs along its own z, so a torpedo laid along the flight
    path has body +z forward and body +x pointing DOWN. Written the obvious way
    round, the streamlined value resists the sink and the bluff value resists
    the flight: measured, 0.14 m of travel in 2 s where it should cover 2.5 m.
    """
    if kind == 'torpedo':
        r, length, mass = _TORPEDO_R, _TORPEDO_L, _TORPEDO_M
        cd_axial, cd_cross = _TORPEDO_CD_AXIAL, _TORPEDO_CD_CROSS
        collision = (f'<cylinder><radius>{r}</radius>'
                     f'<length>{length:.6g}</length></cylinder>')
        visual = _torpedo_visual()
        area_axial = math.pi * r ** 2
        area_cross = 2.0 * r * length
        displaced = area_axial * length * WATER_DENSITY
        am_axial, am_cross = 0.1 * displaced, 1.0 * displaced
    else:
        r, length, mass = _DROPPER_R, 2 * _DROPPER_R, _DROPPER_M
        cd_axial = cd_cross = 0.47
        collision = f'<sphere><radius>{r}</radius></sphere>'
        rgba = '0.10 0.85 0.30 1'
        visual = ('      <visual name="visual">\n'
                  f'        <geometry><sphere><radius>{r}</radius></sphere>'
                  '</geometry>\n'
                  f'        <material><diffuse>{rgba}</diffuse>'
                  f'<ambient>{rgba}</ambient></material>\n'
                  '      </visual>')
        area_axial = area_cross = math.pi * r ** 2
        displaced = (4.0 / 3.0) * math.pi * r ** 3 * WATER_DENSITY
        # A sphere's added mass is half its displacement, on every axis.
        am_axial = am_cross = 0.5 * displaced

    axial = -0.5 * WATER_DENSITY * cd_axial * area_axial
    cross = -0.5 * WATER_DENSITY * cd_cross * area_cross
    i_long = max(1e-6, mass * r ** 2 / 2.0)
    i_trans = max(1e-6, mass * (3 * r ** 2 + length ** 2) / 12.0)

    return (
        '<?xml version="1.0"?><sdf version="1.9">\n'
        f'<model name="{name}"><pose>{pose}</pose>\n'
        '  <link name="body">\n'
        f'    <inertial><mass>{mass:.6g}</mass><inertia>\n'
        f'      <ixx>{i_trans:.6g}</ixx><iyy>{i_trans:.6g}</iyy>'
        f'<izz>{i_long:.6g}</izz>\n'
        '      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>\n'
        f'    <collision name="collision"><geometry>{collision}</geometry>'
        '</collision>\n'
        f'{visual}\n'
        '  </link>\n'
        '  <plugin filename="gz-sim-hydrodynamics-system"\n'
        '          name="gz::sim::systems::Hydrodynamics">\n'
        '    <link_name>body</link_name>\n'
        f'    <xDotU>{-am_cross:.6g}</xDotU>\n'
        f'    <yDotV>{-am_cross:.6g}</yDotV>\n'
        f'    <zDotW>{-am_axial:.6g}</zDotW>\n'
        f'    <xUabsU>{cross:.6g}</xUabsU>\n'
        f'    <yVabsV>{cross:.6g}</yVabsV>\n'
        f'    <zWabsW>{axial:.6g}</zWabsW>\n'
        '    <kPabsP>-0.002</kPabsP>\n'
        '    <mQabsQ>-0.002</mQabsQ>\n'
        '    <nRabsR>-0.0005</nRabsR>\n'
        '  </plugin>\n'
        '</model></sdf>')


class PayloadSim(Node):
    def __init__(self) -> None:
        super().__init__('payload_sim')
        self.declare_parameter('world', 'sauvc26_final')
        self.declare_parameter('vehicle', 'duburi')
        self.declare_parameter('port_link', '')
        # Launch impulse. 12.2 N for 0.12 s on 325 g is a 4.5 m/s muzzle
        # velocity. The rulebook caps this by feel rather than number --
        # "Torpedoes must travel at a 'safe' speed. A 'safe' speed is one that
        # would not cause a bruise when it strikes a person underwater from
        # close range" (p. 64) -- and 4.5 m/s from a 325 g round is a plausible
        # reading of that. Drag bleeds it to ~1 m/s over 4.7 m, so the useful
        # firing envelope covers the rulebook's 1.0 m and 1.5 m standoffs with
        # margin.
        self.declare_parameter('launch_force_n', 12.2)
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
        self._spawned = set()
        self._lock = threading.Lock()

        self._gz = GzNode()
        # Sim clock, because the launch burn must be measured in SIM TIME.
        # A wall-clock sleep gives the round `burn * RTF` seconds of thrust:
        # measured at RTF 0.23 a 0.12 s burn delivered a quarter of its
        # impulse and the round left the tube at 1.0 m/s instead of 4.5, so
        # every shot fell short and the launcher looked feeble rather than
        # mistimed. It also means the same code fires differently on a fast
        # machine and a slow one.
        self._sim_time = None
        try:
            from gz.msgs10.world_stats_pb2 import WorldStatistics
            self._gz.subscribe(WorldStatistics, f'/world/{self._world}/stats',
                               self._on_stats)
        except ImportError:
            self.get_logger().warn(
                '[PAYLOAD-SIM] no world stats -- burn falls back to wall clock')
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

    def _on_stats(self, msg) -> None:
        self._sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def _sleep_sim(self, seconds: float) -> None:
        """Block until `seconds` of SIM time have passed."""
        if self._sim_time is None:
            time.sleep(seconds)
            return
        target = self._sim_time + seconds
        deadline = time.monotonic() + 30.0     # never hang on a paused sim
        while self._sim_time < target and time.monotonic() < deadline:
            time.sleep(0.002)

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
        # Only remove a slot we have actually filled -- Gazebo logs
        # "Entity named [payload_shot_0] ... not found, so not removed" at ERR
        # level otherwise, so every first shot printed an error that looked
        # like a failure and was not.
        if name in self._spawned:
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
        self._spawned.add(name)
        if kind != 'torpedo':
            return

        force = float(self.get_parameter('launch_force_n').value)
        burn = float(self.get_parameter('burn_time_s').value)
        # WAIT FOR DISCOVERY, DO NOT SLEEP FOR IT. gz-transport silently drops
        # a publish before the remote subscriber has been discovered, so the
        # first shots after startup left the tube with NO launch impulse at
        # all -- the round simply sank away from the muzzle at 0.2 m/s and
        # every one of them was scored a miss. Nothing logs a dropped publish;
        # the shot just looks feeble.
        deadline = time.monotonic() + 3.0
        while (not self._pub_wrench.has_connections()
               and time.monotonic() < deadline):
            time.sleep(0.02)
        if not self._pub_wrench.has_connections():
            self.get_logger().warn(
                '[PAYLOAD-SIM] no subscriber on the wrench topic -- the round '
                'will be released with no launch impulse')
        w = EntityWrench()
        w.entity.name = name
        w.entity.type = Entity.MODEL
        w.wrench.force.x = force * cy
        w.wrench.force.y = force * sy
        self._pub_wrench.publish(w)
        self._sleep_sim(burn)
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
