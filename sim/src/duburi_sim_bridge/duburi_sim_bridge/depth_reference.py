#!/usr/bin/env python3

"""Check that the simulated depth reference is honest, and shout if it is not.

WHY THIS EXISTS
---------------
The stack reads depth from `AHRS2.altitude` -- the pool-verified hardware path
(see CLAUDE.md; do NOT change the depth source). On the real vehicle that reads
true depth because the hull is powered on floating at the surface, so the
reference is captured there.

In SITL the vehicle is SPAWNED SUBMERGED, and each course picks its own z. The
reference is captured at that depth, and every later reading carries a constant
per-course offset. Measured against Gazebo ground truth, steady to four decimal
places for 220 s and identical armed or disarmed:

    sauvc26_qualification (spawn -0.8)   -0.344 m
    robosub26_full        (spawn -0.5)   -0.044 m
    sauvc26_final         (spawn -0.3)   +0.016 m

Two autonomy verbs break as a direct consequence, and they interlock:

  * `surface()` commands 0.0 m. ArduSub takes the hull up correctly -- it
    controls on EKF3, whose altitude IS accurate (-0.007 against a true -0.036)
    -- but our readback plateaus near -0.4, so the verb never confirms.
  * `mission_reset()`'s baro re-zero is REFUSED, because the pre-cal reading
    exceeds `_BARO_SURFACE_BOUND_M` (0.30). That bound catches a real pool baro
    fault and must NOT be widened for a sim artifact.

The fix is in the COURSE (spawn near the surface, like a real launch), not here.
This node exists so a course that spawns too deep says so on every startup
instead of being discovered as two mysteriously broken verbs.

WHAT WAS TRIED AND REJECTED
---------------------------
Correcting the reading with `BARO_ALT_OFFSET`. It zeroes the surface reading --
and then the barometer STOPS TRACKING DEPTH. Measured: readback frozen at
-0.030 m with the hull at -1.206 m, which made `surface()` CONFIRM while
submerged. A false pass is worse than the hang it replaced. Do not reintroduce
it.

Also measured, and worth knowing on its own: ArduSub SITL **ACKs
MAV_CMD_PREFLIGHT_CALIBRATION as ACCEPTED without calibrating** -- the exact
command `calibrate_depth` and QGC's "Calibrate Pressure" send. At best it does
nothing; at worst it re-zeros ground pressure treating water pressure as AIR
(a few centimetres of draft became +20.3 m of apparent altitude). That is why
the sim launch passes `baro_calibration:=false`.
"""

from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

GROUND_TRUTH_TOPIC = '/duburi/sim/ground_truth'

# The hull rises from its spawn depth to a floating equilibrium of about
# -0.036 m. Both series must be flat before the offset means anything: this
# round measured the same quantity four times while it was still settling and
# got four different answers, including one that reversed the conclusion. Gate
# on BOTH the truth and the sensor, never truth alone.
SETTLE_WINDOW_S = 15.0
SETTLE_TOL_M = 0.01
SETTLE_MIN_S = 25.0


class DepthReference(Node):

    def __init__(self):
        super().__init__('depth_reference')
        self.declare_parameter('gcs_port', 14551)
        self.declare_parameter('enabled', True)
        # Beyond this the offset is not a reference artifact -- something is
        # actually wrong (wrong world, hull on the floor) and silently adding a
        # metre of correction would hide it.
        self.declare_parameter('warn_offset_m', 0.15)
        self.declare_parameter('timeout_s', 180.0)
        self._z = None
        self.create_subscription(Odometry, GROUND_TRUTH_TOPIC, self._on_gt,
                                 qos_profile_sensor_data)

    def _on_gt(self, msg):
        self._z = msg.pose.pose.position.z

    # -- helpers ---------------------------------------------------------
    def _spin(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self) -> bool:
        if not self.get_parameter('enabled').value:
            self.get_logger().info('[DEPTH] disabled -- leaving BARO_ALT_OFFSET alone')
            return True
        from pymavlink import mavutil

        port = int(self.get_parameter('gcs_port').value)
        link = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}')
        self.get_logger().info(f'[DEPTH] waiting for ArduSub on udpin:{port}')
        if link.wait_heartbeat(timeout=90) is None:
            self.get_logger().warn('[DEPTH] no heartbeat -- depth reference NOT calibrated')
            return False

        # ArduSub streams AHRS2 slowly on this link by default -- the manager
        # pins its rates on 14550, not here. At the default rate the settle
        # window never collects enough samples and this node waits forever
        # while logging nothing, which looks exactly like a hang.
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 100000, 0, 0, 0, 0, 0)
        self.get_logger().info('[DEPTH] link up, AHRS2 pinned to 10 Hz -- settling')

        got = self._settle(link)
        if got is None:
            self.get_logger().warn('[DEPTH] never settled -- depth reference NOT calibrated')
            return False
        gt, ahrs = got
        offset = ahrs - gt
        self.get_logger().info(
            f'[DEPTH] settled: true={gt:+.3f}m  AHRS2={ahrs:+.3f}m  offset={offset:+.3f}m')

        limit = float(self.get_parameter('warn_offset_m').value)
        if abs(offset) <= limit:
            self.get_logger().info(
                f'[DEPTH] depth reference OK (|{offset:+.3f}| <= {limit:.2f} m)')
            return True

        self.get_logger().error(
            f'[DEPTH] DEPTH REFERENCE IS OFF BY {offset:+.3f} m. The stack '
            f'believes {ahrs:+.3f} m where the truth is {gt:+.3f} m.')
        self.get_logger().error(
            '[DEPTH] Consequences: surface() will not confirm (it commands 0.0 '
            'and the readback plateaus), and mission_reset\'s baro re-zero is '
            'refused once the offset passes 0.30 m.')
        self.get_logger().error(
            f'[DEPTH] Cause: this course spawns the vehicle too deep. The offset '
            f'tracks spawn z; courses spawning at -0.3..-0.5 measure within '
            f'0.05 m. Fix the course, NOT the autopilot -- BARO_ALT_OFFSET '
            f'"corrects" the surface reading and then stops the baro tracking '
            f'depth at all (measured: frozen at -0.03 m with the hull at '
            f'-1.21 m, so surface() CONFIRMED while submerged).')
        return False

    def _sample(self, link, secs):
        """Mean ground truth and AHRS2 over the same window, sampled together."""
        gts, alts = [], []
        end = time.monotonic() + secs
        while time.monotonic() < end:
            msg = link.recv_match(type='AHRS2', blocking=True, timeout=1.0)
            rclpy.spin_once(self, timeout_sec=0.001)
            if msg is not None:
                alts.append(msg.altitude)
            if self._z is not None:
                gts.append(self._z)
        if not gts or not alts:
            return None
        return sum(gts) / len(gts), sum(alts) / len(alts)

    def _settle(self, link):
        deadline = time.monotonic() + float(self.get_parameter('timeout_s').value)
        started = time.monotonic()
        last_note = [0.0]
        hist = []
        while time.monotonic() < deadline:
            msg = link.recv_match(type='AHRS2', blocking=True, timeout=1.0)
            rclpy.spin_once(self, timeout_sec=0.001)
            if msg is None or self._z is None:
                continue
            now = time.monotonic()
            hist.append((now, self._z, msg.altitude))
            hist = [h for h in hist if now - h[0] <= SETTLE_WINDOW_S]
            if now - started - last_note[0] > 20.0:
                last_note[0] = now - started
                self.get_logger().info(
                    f'[DEPTH] settling... t={now - started:.0f}s '
                    f'true={self._z:+.3f} AHRS2={msg.altitude:+.3f} '
                    f'({len(hist)} samples in the window)')
            if now - started < SETTLE_MIN_S or len(hist) < 40:
                continue
            zs = [h[1] for h in hist]
            als = [h[2] for h in hist]
            if (max(zs) - min(zs) < SETTLE_TOL_M
                    and max(als) - min(als) < SETTLE_TOL_M):
                return sum(zs) / len(zs), sum(als) / len(als)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = DepthReference()
    try:
        node.run()
    except Exception as exc:                      # never take the sim down
        node.get_logger().error(f'[DEPTH] {type(exc).__name__}: {exc}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
