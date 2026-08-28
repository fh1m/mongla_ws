#!/usr/bin/env python3
"""Simulated hydrophone array: per-ping bearing to an acoustic pinger.

WHAT THIS IS, AND IS NOT. Gazebo has no acoustic sensor and no acoustic
propagation, and neither does this node. It simulates **what a 4-element
hydrophone array reports** -- a bearing, an elevation, a range estimate and an
SNR, once per ping -- by taking ground truth and degrading it the way a real
array degrades it. That is the same bargain the DVL makes.

Tuning a bearing-homing behaviour against this transfers to the pool. Validating
a beamformer against it does not: there is no waveform here, only its output.

Both competitions score on this:
  * SAUVC hides an RJE ULB-362B (45 kHz) in one of four drums, and which drum is
    randomised between attempts, so the bearing is the only thing that finds it.
  * RoboSub uses a Benthos ALP-365, user-selectable 25-40 kHz at 0.5-2 Hz, with
    TWO active pingers at least 2 kHz apart -- one at Deploy, one at Restore.
    The team may ask for a specific one, so this node filters by frequency.

The degradations are the point. A perfect bearing makes homing trivial and
teaches a mission nothing:

  bearing noise    grows with range -- a distant source is a fuzzy source
  dropouts         a real array misses pings; a mission must tolerate silence
  multipath ghosts a wrong bearing off a wall, at the right ping time, which is
                   the failure that actually breaks naive homing: it is not
                   noise around the truth, it is a confident lie
  blind cone       an array sees nothing directly below itself
  SNR              falls with range, and is what a mission should gate on

Publishes:
  <ns>/bearing   duburi_interfaces/... -- no custom msg; see below
  <ns>/ping      geometry_msgs/Vector3Stamped   x=bearing_deg y=elevation_deg
                                                z=snr_db
  <ns>/range     sensor_msgs/Range              slant range estimate

Vector3Stamped rather than a new message type on purpose: this is a simulator-
only diagnostic surface, and adding to duburi_interfaces would put a sim concept
into the package the real vehicle builds against.
"""
from __future__ import annotations

import math
import random

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range

GROUND_TRUTH = '/duburi/sim/ground_truth'

# Speed of sound in fresh water at pool temperature, m/s. Only used to turn a
# range into a time-of-flight for the range estimate's quantisation.
SOUND_SPEED = 1481.0


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


class Hydrophone(Node):
    def __init__(self) -> None:
        super().__init__('hydrophone')

        # Pinger placement. A course puts a `*_pinger` prop somewhere; this node
        # is told where, because it has no way to read prop poses out of Gazebo
        # and inventing a lookup would couple it to the world format.
        self.declare_parameter('pinger_x', 0.0)
        self.declare_parameter('pinger_y', 0.0)
        self.declare_parameter('pinger_z', -1.5)
        self.declare_parameter('freq_khz', 45.0)      # SAUVC ULB-362B default
        self.declare_parameter('pulse_hz', 1.0)

        # What the vehicle is listening FOR. 0 = accept any frequency. RoboSub
        # runs two pingers >= 2 kHz apart and the team may request one, so a
        # mission that sets this should stop hearing the other.
        self.declare_parameter('listen_khz', 0.0)
        self.declare_parameter('listen_bandwidth_khz', 1.0)

        # Degradations.
        self.declare_parameter('bearing_noise_deg', 2.0)   # at 1 m
        self.declare_parameter('noise_growth_per_m', 0.35)
        self.declare_parameter('dropout_prob', 0.12)
        self.declare_parameter('ghost_prob', 0.06)
        self.declare_parameter('blind_cone_deg', 25.0)     # half-angle, downward
        self.declare_parameter('max_range_m', 30.0)
        self.declare_parameter('seed', 0)

        seed = int(self.get_parameter('seed').value)
        self._rng = random.Random(seed if seed else None)

        self._pose = None
        self._pings = 0
        self._heard = 0

        self.create_subscription(Odometry, GROUND_TRUTH, self._on_odom, 10)
        self._pub_ping = self.create_publisher(
            Vector3Stamped, '/duburi/sim/hydrophone/ping', 10)
        self._pub_range = self.create_publisher(
            Range, '/duburi/sim/hydrophone/range', 10)

        pulse = max(0.1, float(self.get_parameter('pulse_hz').value))
        self.create_timer(1.0 / pulse, self._ping)
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f'hydrophone ready  pinger at '
            f'({self.get_parameter("pinger_x").value}, '
            f'{self.get_parameter("pinger_y").value}, '
            f'{self.get_parameter("pinger_z").value})  '
            f'{self.get_parameter("freq_khz").value} kHz @ {pulse} Hz')

    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._pose = ((p.x, p.y, p.z), _quat_to_yaw(q.x, q.y, q.z, q.w))

    def _listening(self) -> bool:
        want = float(self.get_parameter('listen_khz').value)
        if want <= 0.0:
            return True
        bw = float(self.get_parameter('listen_bandwidth_khz').value)
        return abs(float(self.get_parameter('freq_khz').value) - want) <= bw

    def _ping(self) -> None:
        if self._pose is None or not self._listening():
            return
        self._pings += 1
        (vx, vy, vz), yaw = self._pose
        dx = float(self.get_parameter('pinger_x').value) - vx
        dy = float(self.get_parameter('pinger_y').value) - vy
        dz = float(self.get_parameter('pinger_z').value) - vz

        horiz = math.hypot(dx, dy)
        slant = math.sqrt(horiz * horiz + dz * dz)
        if slant > float(self.get_parameter('max_range_m').value):
            return

        # True bearing, relative to the vehicle's heading.
        bearing = _wrap180(math.degrees(math.atan2(dy, dx) - yaw))
        elevation = math.degrees(math.atan2(dz, horiz))

        # An array cannot resolve a source inside its own blind cone.
        if abs(elevation) > (90.0 - float(
                self.get_parameter('blind_cone_deg').value)):
            return

        if self._rng.random() < float(self.get_parameter('dropout_prob').value):
            return

        # A GHOST is not noise around the truth -- it is a confident wrong
        # bearing off a wall, arriving on time and looking exactly as valid as a
        # real one. Homing that averages bearings survives noise and is broken
        # by this, which is why it is modelled separately.
        if self._rng.random() < float(self.get_parameter('ghost_prob').value):
            bearing = _wrap180(bearing + self._rng.choice((-1.0, 1.0))
                               * self._rng.uniform(35.0, 120.0))
            snr = self._rng.uniform(3.0, 8.0)
        else:
            sigma = (float(self.get_parameter('bearing_noise_deg').value)
                     + float(self.get_parameter('noise_growth_per_m').value)
                     * slant)
            bearing = _wrap180(bearing + self._rng.gauss(0.0, sigma))
            elevation += self._rng.gauss(0.0, sigma * 0.5)
            # Spherical spreading: SNR falls ~20*log10(r).
            snr = max(0.0, 42.0 - 20.0 * math.log10(max(slant, 0.3)))

        self._heard += 1
        now = self.get_clock().now().to_msg()

        m = Vector3Stamped()
        m.header.stamp = now
        m.header.frame_id = 'base_link'
        m.vector.x, m.vector.y, m.vector.z = bearing, elevation, snr
        self._pub_ping.publish(m)

        r = Range()
        r.header.stamp = now
        r.header.frame_id = 'base_link'
        r.radiation_type = Range.ULTRASOUND
        r.field_of_view = math.radians(30.0)
        r.min_range = 0.3
        r.max_range = float(self.get_parameter('max_range_m').value)
        # Range from time-of-flight, quantised the way a real correlator is.
        tof = slant / SOUND_SPEED
        r.range = float(round(tof * SOUND_SPEED, 2))
        self._pub_range.publish(r)

    def _report(self) -> None:
        if self._pings:
            self.get_logger().info(
                f'[ACOU ] {self._heard}/{self._pings} pings heard '
                f'({100.0 * self._heard / self._pings:.0f} %)')


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = Hydrophone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
