#!/usr/bin/env python3

"""A virtual BNO085 board, so the real yaw source runs in simulation.

`yaw_source` has four options and the sim supported two. The two it did not
are the ones the vehicle actually flies: `bno085` and `bno085_dvl`. So the
heading loop -- `HeadingLock`, `motion_yaw`, every `turn` and every
`lock_heading` -- could only ever be tuned in sim against a sensor the vehicle
does not use, and `BNO085Source` itself (its calibration handshake, its
staleness window, its sign convention) had never executed outside the pool.

THE REAL DRIVER CONNECTS TO THIS, UNMODIFIED, exactly as with `payload_sim`.
The firmware contract is a line of JSON over USB CDC at 115200
(`firmware/esp32c3_bno085.md`), so a PTY presents a device node that
`BNO085Source` opens with its own `Serial()` setup, its own DTR/RTS handling,
its own `readline()` and its own parser. Nothing in `duburi_sensors` changes,
which is the point: a `sim_bno085` yaw *source* would be new code testing
itself, and the calibration handshake against the Pixhawk -- the part most
likely to break -- would never run at all.

    ros2 run duburi_sim_bringup duburi_sim stack --no-vision \\
        --ros-args -p yaw_source:=bno085 \\
        -p bno085_port:=/tmp/duburi-$USER/bno085

CONVENTIONS, both of which are silent if wrong:

*The board emits +CCW, sensor-frame.* `BNO085Source._reader_loop` negates the
raw value once at ingestion because the firmware is ENU-native while the rest
of the stack is compass/NED. Emitting compass yaw here would leave the sign
inverted with no error -- a heading lock that drives away from its target.

*The board emits a BOOT-RELATIVE frame, not Earth.* The real BNO has no
magnetometer enabled, so its zero is wherever the chip happened to be at power
on; the Jetson captures `pixhawk_yaw - bno_raw` once at startup and adds it
forever after. Publishing true heading here would make that handshake a no-op
and leave it untested. `boot_offset_deg` is that arbitrary zero, and it
defaults to a value that is deliberately not 0.

DRIFT IS FROM THE DATASHEET, not invented (BNO08X rev 1.17, Figure 6-14):

    Gaming Rotation Vector Nominal ... Dynamic Heading Drift   0.5 deg/min
    Gyroscope Nominal ................ Dynamic Accuracy        3.1 deg/s

The 0.5 deg/min row is the applicable one because our firmware runs
`SH2_GYRO_INTEGRATED_RV` with the magnetometer disabled, and datasheet section
2.2.6 says the gyro rotation vector is "configured via FRS record to be based
on either the rotation vector (using the magnetometer) or the game rotation
vector (ignoring the magnetometer)" -- ours is the latter. The datasheet gives
no separate drift figure for the gyro-integrated variant, and no Allan-variance
curve, so the random walk below is the datasheet rate applied as a bounded
walk rather than a fitted noise model. That is weaker grounding than the T200
curve, which came from measurements, and the docs say so.
"""

from __future__ import annotations

import json
import math
import os
import pty
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

# BNO08X Datasheet rev 1.17 Figure 6-14, "Gaming Rotation Vector Nominal".
DATASHEET_HEADING_DRIFT_DEG_PER_MIN = 0.5
# Same table, "Gyroscope Nominal Dynamic Accuracy".
DATASHEET_GYRO_ACCURACY_DEG_PER_S = 3.1

# firmware/esp32c3_bno085.md: "~50 Hz sustained (chip runs internal at 500 Hz;
# sketch throttles every 20 ms)".
FIRMWARE_RATE_HZ = 50.0


class Bno085Sim(Node):
    def __init__(self) -> None:
        super().__init__('bno085_sim')
        self.declare_parameter('vehicle', 'duburi')
        self.declare_parameter('rate_hz', FIRMWARE_RATE_HZ)
        self.declare_parameter('port_link', '')
        # Where the chip's arbitrary zero sits relative to true heading. Not 0,
        # so a stack that silently skipped the Pixhawk calibration handshake
        # reads visibly wrong instead of accidentally right.
        self.declare_parameter('boot_offset_deg', 37.0)
        self.declare_parameter('drift_deg_per_min',
                               DATASHEET_HEADING_DRIFT_DEG_PER_MIN)
        # Per-sample angle noise. The datasheet quotes RATE accuracy
        # (3.1 deg/s); over one 20 ms frame that is 0.062 deg of angle, which
        # is what this represents. It is NOT a drift term -- it averages out.
        self.declare_parameter(
            'noise_deg', DATASHEET_GYRO_ACCURACY_DEG_PER_S / FIRMWARE_RATE_HZ)
        # Fault injection, same shape as /faults: a duration that self-clears.
        self.declare_parameter('dropout_s', 0.0)

        # Cached, because the 50 Hz write loop read them through
        # get_parameter() -- three lock-taking calls per frame.
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._noise_deg = float(self.get_parameter('noise_deg').value)
        self._boot_offset = float(self.get_parameter('boot_offset_deg').value)

        self._truth_yaw = None
        self._drift = 0.0
        self._drift_t = time.monotonic()
        self._dropout_until = 0.0
        self._frames = 0
        self._dropped = 0

        import random
        self._rng = random.Random(0xB0)
        # Per-run gyro zero-rate offset, in deg/min of heading error. Drawn
        # from the datasheet figure so a typical run drifts about that much,
        # with the sign and exact size varying run to run as real hardware
        # does. Re-seeded per process, so two runs are not identical.
        self._bias = self._rng.gauss(
            0.0, float(self.get_parameter('drift_deg_per_min').value))

        self._gz_connect()

        self._master, slave = pty.openpty()
        # NON-BLOCKING writes. A PTY write blocks once the buffer fills, and
        # the buffer fills whenever the host end reads slower than 50 Hz --
        # which stalls the whole write loop and drags the stream rate down
        # (measured 50 -> 22.7 Hz the moment the stack attached). A real USB
        # CDC link does not stall the sensor; the frame is simply lost. Drop
        # it here for the same reason, so the board's rate is its own and the
        # host's backlog is the host's problem.
        os.set_blocking(self._master, False)
        self._port = os.ttyname(slave)
        # Hold the slave fd. Closing it makes every later read on the master
        # raise EIO as soon as the client disconnects -- the board would work
        # exactly once.
        self._slave_fd = slave
        self._link = self._make_link(self.get_parameter('port_link').value)

        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            f'[BNO-SIM] virtual BNO085 on {self._port}'
            + (f' (symlink {self._link})' if self._link else '')
            + f' @ {self.get_parameter("rate_hz").value:.0f} Hz, '
            f'drift {self.get_parameter("drift_deg_per_min").value:.2f} deg/min '
            f'(this run: {self._bias:+.3f} deg/min)')
        self.get_logger().info(
            f'[BNO-SIM] start the stack with yaw_source:=bno085 '
            f'bno085_port:={self._link or self._port}')

        threading.Thread(target=self._write_loop, daemon=True).start()
        self.create_timer(10.0, self._report)

    # -- truth -------------------------------------------------------------

    def _gz_connect(self) -> None:
        """Truth from Gazebo, not from the ROS ground-truth topic.

        That topic only exists when the ros_gz bridge is running, and a heading
        sensor that goes silent under `bridge:=false` is a sensor you cannot
        use for a bare control test. `payload_sim` had exactly this bug.
        """
        try:
            from gz.msgs10.odometry_pb2 import Odometry
            from gz.transport13 import Node as GzNode
        except ImportError:
            self.get_logger().error(
                '[BNO-SIM] no gz-transport bindings -- the board will stream '
                'a fixed heading')
            self._gz = None
            return
        self._gz = GzNode()
        vehicle = self.get_parameter('vehicle').value
        if not self._gz.subscribe(Odometry, f'/model/{vehicle}/odometry',
                                  self._on_odom):
            self.get_logger().error(
                f'[BNO-SIM] could not subscribe to /model/{vehicle}/odometry')

    def _on_odom(self, msg) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        self._truth_yaw = math.degrees(yaw)

    # -- the board ---------------------------------------------------------

    def _sensor_yaw(self) -> float:
        """One frame of sensor-frame, +CCW yaw, drift and noise included."""
        now = time.monotonic()
        dt = min(1.0, now - self._drift_t)
        self._drift_t = now

        # A LINEAR RAMP from a fixed per-run bias, not a random walk.
        #
        # The datasheet explains its own drift figure causally: "removal of
        # gyroscope ZRO is critical to reduce heading drift" (section 3.3), and
        # zero-rate offset is essentially constant across a power-up. So the
        # heading error of a gyro-only mode grows LINEARLY with time, and
        # "0.5 deg/min" reads as exactly that -- about 5 degrees after ten
        # minutes, which is why a long mission wants a re-zero.
        #
        # This was a random walk first, and measurement killed it twice over:
        # a walk grows as sqrt(t) rather than t, and the coefficient was
        # divided by 60 as though it were a rate when a walk's scales as
        # sqrt(60) -- together that made the modelled drift 7.7x too weak at
        # one minute. A sim whose heading holds better than the vehicle's is
        # worse than no model at all, because it hides the drift the operator
        # has to plan around.
        #
        # The bias is drawn ONCE per run (self._bias), so a given run drifts
        # steadily one way like real hardware, while runs differ.
        self._drift += self._bias * (dt / 60.0)

        truth = self._truth_yaw if self._truth_yaw is not None else 0.0
        noise = self._rng.gauss(0.0, self._noise_deg)
        boot = self._boot_offset

        # Gazebo yaw is ENU/+CCW already, and so is the firmware, so no
        # negation here -- BNO085Source._reader_loop does that once on the
        # host side. Subtracting the boot offset makes this the SENSOR frame:
        # the driver adds (pixhawk_yaw - bno_raw) back at calibration.
        return (truth - boot + self._drift + noise) % 360.0

    def _write_loop(self) -> None:
        t0 = time.monotonic()
        next_frame = t0
        while True:
            # DEADLINE SCHEDULING, not sleep-per-iteration.
            #
            # `sleep(1/hz)` makes the PERIOD 1/hz PLUS the work, so the rate is
            # always under target and degrades with load. Measured with the
            # Gazebo GUI up, it fell 49.3 -> 23.2 Hz, and that is not cosmetic:
            # BNO085Source._STALE_S is 0.08 s, four frames at 50 Hz, so at
            # 23 Hz the driver holds under two frames per stale window and
            # starts flapping between fresh and stale. Sleeping to the next
            # deadline keeps the rate flat and lets a slow tick catch up.
            hz = max(1.0, self._rate_hz)
            next_frame += 1.0 / hz
            delay = next_frame - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # Fell far behind (a suspended process, a stalled host): resync
                # rather than spin trying to emit a burst of stale frames.
                next_frame = time.monotonic()
            if self._dropout_until > time.monotonic():
                continue
            line = json.dumps({
                'yaw': round(self._sensor_yaw(), 2),
                'ts': int((time.monotonic() - t0) * 1000.0),
            }) + '\n'
            try:
                os.write(self._master, line.encode())
                self._frames += 1
            except BlockingIOError:
                # Host is behind; the frame is gone, exactly as on the wire.
                self._dropped += 1
            except OSError:
                # Nobody has the far end open yet, or it just closed. The real
                # board keeps talking to a disconnected host too.
                time.sleep(0.1)

    # -- plumbing ----------------------------------------------------------

    def _make_link(self, requested: str) -> str:
        link = requested or os.path.join(
            f'/tmp/duburi-{os.environ.get("USER", "user")}', 'bno085')
        try:
            os.makedirs(os.path.dirname(link), exist_ok=True)
            if os.path.islink(link) or os.path.exists(link):
                os.unlink(link)
            os.symlink(self._port, link)
            return link
        except OSError as exc:
            self.get_logger().warn(f'[BNO-SIM] no symlink at {link}: {exc}')
            return ''

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'rate_hz':
                self._rate_hz = float(p.value)
            elif p.name == 'noise_deg':
                self._noise_deg = float(p.value)
            elif p.name == 'boot_offset_deg':
                self._boot_offset = float(p.value)
            elif p.name == 'dropout_s':
                secs = float(p.value)
                if secs <= 0.0:
                    self._dropout_until = 0.0
                    self.get_logger().warn('[BNO-SIM] dropout cleared')
                else:
                    self._dropout_until = time.monotonic() + secs
                    self.get_logger().warn(
                        f'[BNO-SIM] STREAM STOPPED for {secs:.1f} s')
        return SetParametersResult(successful=True)

    def _report(self) -> None:
        if self._frames:
            self.get_logger().info(
                f'[BNO-SIM] {self._frames / 10.0:.1f} Hz, '
                f'accumulated drift {self._drift:+.2f} deg'
                + (f', {self._dropped} frames dropped (host behind)'
                   if self._dropped else ''))
            self._frames = 0
            self._dropped = 0
        self._dropped = 0

    def destroy_node(self) -> bool:
        if self._link and os.path.islink(self._link):
            try:
                os.unlink(self._link)
            except OSError:
                pass
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = Bno085Sim()
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
