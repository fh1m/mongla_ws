#!/usr/bin/env python3

"""Break things on purpose, so the recovery paths run at least once.

The stack handles a DVL that stops reporting, a camera that goes away, a
MAVLink link that dies, a sagging battery and a dead thruster. Every one of
those paths exists in the code and **none of them has ever executed in
simulation**, because there was no way to cause the fault. They are first
exercised in the pool, on the day, once -- which is the definition of untested.

Each fault is armed by setting a parameter to a duration in seconds and clears
itself when the time is up. `0` clears immediately:

    ros2 param set /faults dvl_dropout_s 6.0      # 6 s without bottom lock
    ros2 param set /faults camera_loss_s 4.0      # front + bottom cameras stop
    ros2 param set /faults mavlink_loss_s 3.0     # the autopilot link drops
    ros2 param set /faults battery_sag_v 13.2     # pack sags; 0 restores
    ros2 param set /faults dvl_dropout_s 0.0      # cancel early

A duration rather than a toggle because a fault you have to remember to clear
is a fault you will leave on and then debug for an hour.

A dead thruster lives on `/t200_curve` instead (`dead_thrusters`), because that
node is already the only thing between ArduSub and the propellers, and routing
it through here would be a facade over a one-line parameter:

    ros2 param set /t200_curve dead_thrusters "[3]"
"""

from __future__ import annotations

import os
import signal
import socket
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String

# Where the DVL is intercepted. The sensor publishes `_RAW` and this node
# republishes the plain topic that SimDvlSource reads -- see model.sdf.in.
DVL_RAW = '/dvl/velocity_raw'
DVL_OUT = '/dvl/velocity'

STATUS_TOPIC = '/duburi/sim/faults'

# The processes whose SIGSTOP takes the cameras away. Matched against the whole
# command line.
CAMERA_PROCESSES = ('ros_gz_image/image_bridge', 'image_bridge')


class FaultInjection(Node):
    def __init__(self) -> None:
        super().__init__('faults')
        self.declare_parameter('dvl_dropout_s', 0.0)
        self.declare_parameter('camera_loss_s', 0.0)
        self.declare_parameter('mavlink_loss_s', 0.0)
        # Volts. 0 = healthy pack (whatever the T200 node was launched with).
        self.declare_parameter('battery_sag_v', 0.0)
        self.declare_parameter('battery_nominal_v', 16.0)
        # MAVLink relay. Off by default: with it off this node never touches
        # the autopilot link, so a bug here cannot cost a pool session.
        self.declare_parameter('mavlink_relay', False)
        self.declare_parameter('relay_listen_port', 14559)
        self.declare_parameter('relay_forward_port', 14550)

        self._until = {'dvl': 0.0, 'camera': 0.0, 'mavlink': 0.0}
        self._camera_stopped = False
        self._status = self.create_publisher(String, STATUS_TOPIC, 10)

        self._gz = None
        self._dvl_pub = None
        self._start_dvl_relay()

        self._relay = None
        if self.get_parameter('mavlink_relay').value:
            self._relay = _MavlinkRelay(
                int(self.get_parameter('relay_listen_port').value),
                int(self.get_parameter('relay_forward_port').value),
                self.get_logger())
            self._relay.start()

        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            '[FAULT] ready -- dvl_dropout_s / camera_loss_s / mavlink_loss_s '
            '/ battery_sag_v (dead thrusters: /t200_curve dead_thrusters)')

    # -- DVL ---------------------------------------------------------------

    def _start_dvl_relay(self) -> None:
        """Republish the DVL, so a dropout is simply not republishing."""
        try:
            from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking
            from gz.transport13 import Node as GzNode
        except ImportError:
            self.get_logger().warn(
                '[FAULT] no gz-transport bindings -- DVL faults unavailable '
                'AND the DVL itself will not reach the stack')
            return
        self._gz = GzNode()
        self._dvl_pub = self._gz.advertise(DVL_OUT, DVLVelocityTracking)
        if not self._gz.subscribe(DVLVelocityTracking, DVL_RAW, self._on_dvl):
            self.get_logger().error(
                f'[FAULT] could not subscribe to {DVL_RAW} -- every *_dist '
                f'verb will refuse, because nothing will publish {DVL_OUT}')

    def _on_dvl(self, msg) -> None:
        if self._until['dvl'] > time.monotonic():
            return
        self._dvl_pub.publish(msg)

    # -- cameras -----------------------------------------------------------

    def _camera_pids(self) -> list:
        pids = []
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmd = f.read().replace(b'\x00', b' ').decode(errors='ignore')
            except OSError:
                continue
            # Never match this process. A fault injector that stops itself is
            # a fault injector you restart the sim to recover from.
            if int(pid) == os.getpid():
                continue
            if any(p in cmd for p in CAMERA_PROCESSES):
                pids.append(int(pid))
        return pids

    def _set_cameras(self, stopped: bool) -> None:
        if stopped == self._camera_stopped:
            return
        sig = signal.SIGSTOP if stopped else signal.SIGCONT
        pids = self._camera_pids()
        for pid in pids:
            try:
                os.kill(pid, sig)
            except OSError as exc:
                self.get_logger().warn(f'[FAULT] pid {pid}: {exc}')
        self._camera_stopped = stopped
        self.get_logger().warn(
            f'[FAULT] cameras {"STOPPED" if stopped else "restored"} '
            f'({len(pids)} bridge process(es))')

    # -- battery -----------------------------------------------------------

    def _set_battery(self, volts: float) -> None:
        """Sag the pack. Thrust falls out of the T200 curve on its own.

        Only the T200 node is told. ArduSub's own SIM_BATT_VOLTAGE would also
        need setting to exercise its battery FAILSAFE, but that failsafe is
        deliberately disabled in duburi_sub.parm (FS_BATT_ENABLE 0, so a long
        experiment is never aborted mid-run) and turning it on from here would
        change the vehicle's behaviour for reasons the operator did not ask
        for. What this models is the part that always bites: less voltage, less
        thrust, a wider deadband.
        """
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        cli = self.create_client(SetParameters, '/t200_curve/set_parameters')
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('[FAULT] /t200_curve is not running')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='voltage',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(volts)))]
        cli.call_async(req)
        self.get_logger().warn(f'[FAULT] battery set to {volts:.2f} V')

    # -- plumbing ----------------------------------------------------------

    def _on_params(self, params) -> SetParametersResult:
        now = time.monotonic()
        for p in params:
            if p.name == 'dvl_dropout_s':
                self._arm('dvl', float(p.value), now)
            elif p.name == 'camera_loss_s':
                self._arm('camera', float(p.value), now)
            elif p.name == 'mavlink_loss_s':
                self._arm('mavlink', float(p.value), now)
            elif p.name == 'battery_sag_v':
                v = float(p.value)
                self._set_battery(
                    v if v > 0.0
                    else float(self.get_parameter('battery_nominal_v').value))
        return SetParametersResult(successful=True)

    def _arm(self, key: str, seconds: float, now: float) -> None:
        if seconds <= 0.0:
            self._until[key] = 0.0
            self.get_logger().warn(f'[FAULT] {key} cleared')
        else:
            self._until[key] = now + seconds
            self.get_logger().warn(f'[FAULT] {key} FAULT for {seconds:.1f} s')

    def _tick(self) -> None:
        now = time.monotonic()
        active = [k for k, t in self._until.items() if t > now]
        self._set_cameras('camera' in active)
        if self._relay is not None:
            self._relay.blocked = 'mavlink' in active
        self._status.publish(String(data=','.join(sorted(active))))

    def destroy_node(self) -> bool:
        # Leave nothing stopped. A SIGSTOPped image bridge survives this node
        # and looks exactly like a broken camera for the rest of the session.
        self._set_cameras(False)
        if self._relay is not None:
            self._relay.stop()
        return super().destroy_node()


class _MavlinkRelay:
    """A gate on the autopilot link, so it can be cut and restored.

    ArduSub is a udpclient and the manager binds the far end, so there is no
    point between them to interrupt without sitting in the middle. With
    `mavlink_relay:=true` the SITL is pointed at `relay_listen_port` and this
    forwards both directions -- and drops everything while `blocked`.

    SIGSTOPping ArduSub would be simpler and is wrong: the FDM link is
    lock-stepped to Gazebo, so freezing the autopilot freezes the whole
    simulation rather than just the telemetry.
    """

    def __init__(self, listen_port: int, forward_port: int, logger) -> None:
        self._listen_port = listen_port
        self._forward_port = forward_port
        self._log = logger
        self.blocked = False
        self._run = True
        self._sock = None
        self._thread = None

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('127.0.0.1', self._listen_port))
        self._sock.settimeout(0.2)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._log.info(
            f'[FAULT] MAVLink relay {self._listen_port} -> '
            f'{self._forward_port}')

    def _loop(self) -> None:
        upstream = None      # where ArduSub talks from
        downstream = None    # where the manager talks from
        forward = ('127.0.0.1', self._forward_port)
        while self._run:
            try:
                data, addr = self._sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            if addr[1] == self._forward_port:
                downstream = addr
                target = upstream
            else:
                upstream = addr
                target = forward if downstream is None else downstream
            if self.blocked or target is None:
                continue
            try:
                self._sock.sendto(data, target)
            except OSError:
                pass

    def stop(self) -> None:
        self._run = False
        if self._sock is not None:
            self._sock.close()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = FaultInjection()
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
