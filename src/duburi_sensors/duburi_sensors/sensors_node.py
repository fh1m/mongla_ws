#!/usr/bin/env python3
"""sensors_node — standalone diagnostic for any registered YawSource.

A no-mission, no-thrusters smoke-test you can run on the desk or pool
edge before launching `auv_manager_node`. Picks a yaw source from
ROS params and prints periodic readings + a frame-rate summary so you
can confirm the wire is alive end-to-end.

Examples
--------
# 1) Default (ArduSub AHRS via MAVLink). Mission-equivalent input.
ros2 run duburi_sensors sensors_node

# 2) BNO085 raw — sensor-frame yaw, no Earth reference. Wire smoke-test only.
ros2 run duburi_sensors sensors_node --ros-args \\
    -p yaw_source:=bno085 -p bno085_port:=/dev/ttyACM0

# 3) BNO085 calibrated — captures Pixhawk-mag offset once at boot, then
#    pure-gyro Earth-referenced yaw. Same path the manager uses.
ros2 run duburi_sensors sensors_node --ros-args \\
    -p yaw_source:=bno085 -p calibrate:=true -p bno085_port:=/dev/ttyACM0

Notes
-----
* MAVLink is opened only when needed: for yaw_source=mavlink_ahrs, OR
  for yaw_source=bno085 with calibrate:=true. Pure raw BNO mode runs
  with no autopilot connection at all.
* Do NOT run this alongside auv_manager_node on the same MAVLink URL —
  both processes would bind to udpin:14550 and split incoming packets,
  leaving each side showing STALE telemetry. Stop the manager first, OR
  aim this node at a different MAVLink endpoint via -p mavlink_url:=...
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import logging                                       # noqa: E402
import sys                                           # noqa: E402
import time                                          # noqa: E402

_log = logging.getLogger('duburi_sensors.sensors_node')

import rclpy                                         # noqa: E402
from rclpy.node import Node                          # noqa: E402

from duburi_sensors import make_yaw_source           # noqa: E402


# Imported lazily — only needed for yaw_source=mavlink_ahrs.
def _open_mavlink(connection_string: str, get_logger):
    from pymavlink import mavutil
    from duburi_control import Pixhawk

    get_logger().info(f'[SENS ] Connecting MAVLink -> {connection_string}')
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    get_logger().info(
        f'[SENS ] MAVLink up -- sys={master.target_system}'
        f' comp={master.target_component}')
    return Pixhawk(master, log=get_logger())


class SensorsNode(Node):
    def __init__(self):
        super().__init__('duburi_sensors')

        self.declare_parameter('yaw_source',     'mavlink_ahrs')
        self.declare_parameter('bno085_port',    '/dev/ttyACM0')
        self.declare_parameter('bno085_baud',    115200)
        self.declare_parameter('calibrate',      False)
        self.declare_parameter('mavlink_url',    'udpin:0.0.0.0:14550')
        self.declare_parameter('print_period_s', 0.5)

        name      = str(self.get_parameter('yaw_source').value)
        port      = str(self.get_parameter('bno085_port').value)
        baud      = int(self.get_parameter('bno085_baud').value)
        calibrate = bool(self.get_parameter('calibrate').value)
        url       = str(self.get_parameter('mavlink_url').value)
        self._dt  = float(self.get_parameter('print_period_s').value)

        # Open MAVLink only when something downstream actually needs it.
        needs_mav = (name == 'mavlink_ahrs') or (name == 'bno085' and calibrate)
        pixhawk = _open_mavlink(url, self.get_logger) if needs_mav else None

        try:
            self._src = make_yaw_source(
                name,
                pixhawk=pixhawk, port=port, baud=baud,
                calibrate=calibrate,
                logger=self.get_logger())
        except Exception as exc:
            self.get_logger().fatal(
                f'[SENS ] yaw_source={name!r} failed to init: {exc}')
            raise

        cal_note = ''
        if name == 'bno085':
            cal_note = '  (calibrated)' if calibrate else '  (RAW — sensor-frame)'
        self.get_logger().info(
            f'[SENS ] reading from {self._src.name}{cal_note} '
            f'— print every {self._dt:.1f}s, Ctrl-C to stop')

        self._reads_total  = 0
        self._reads_window = 0
        self._last_print   = time.monotonic()
        self.create_timer(self._dt, self._tick)
        self.create_timer(0.05, self._sample)   # 20 Hz pull, well over print rate

    def _sample(self):
        if self._src.read_yaw() is not None:
            self._reads_total  += 1
            self._reads_window += 1

    def _tick(self):
        now = time.monotonic()
        elapsed = max(now - self._last_print, 1e-3)
        hz      = self._reads_window / elapsed
        yaw     = self._src.read_yaw()
        healthy = self._src.is_healthy()

        if yaw is None:
            self.get_logger().warn(
                f'[SENS ] yaw=STALE  healthy={healthy}  '
                f'rx_hz={hz:5.1f}  total={self._reads_total}')
        else:
            self.get_logger().info(
                f'[SENS ] yaw={yaw:6.2f}°  healthy={healthy}  '
                f'rx_hz={hz:5.1f}  total={self._reads_total}')

        self._reads_window = 0
        self._last_print   = now

    def shutdown(self):
        try:
            self._src.close()
        except Exception as exc:
            self.get_logger().debug(
                f'shutdown: yaw source close() ignored: {exc!r}')


def main():
    rclpy.init()
    node = SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as exc:
            # rclpy.shutdown() can raise if context is already shut down
            # (e.g. signal handler ran first). Surface only at debug —
            # node logger may already be torn down here so use stdlib.
            _log.debug('shutdown: rclpy.shutdown() ignored: %r', exc)


if __name__ == '__main__':
    main()
    sys.exit(0)
