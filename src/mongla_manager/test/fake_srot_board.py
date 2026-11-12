"""A SROT board on loopback UDP, speaking real MAVLink (issue #22).

Not a mock of `SrotFC` -- a peer on the wire. The manager connects to it the
way it connects to the vehicle (`mav_device:=udpin:127.0.0.1:<port>`), so a
test drives the WHOLE node: executor, action server, busy gate, abort, the
reader thread and the ACK relay, none of which a unit test with a stubbed FC
can reach.

It models only what the manager relies on, from `srot_protocol.py` and the
board behaviour the docs record:

  * HEARTBEAT (vehicle, autopilot GENERIC) carrying armed and custom_mode;
  * ATTITUDE and VFR_HUD (depth as altitude, negative below the surface);
  * SYS_STATUS with a healthy barometer;
  * AUTOPILOT_VERSION on request, with the firmware behaviour revision in
    `middleware_sw_version`;
  * COMPONENT_ARM_DISARM, DO_SET_MODE;
  * SROT_MOVE: refused while disarmed ("arm first", TEMPORARILY_REJECTED,
    fw rev 13+), otherwise enters AUTO, streams IN_PROGRESS with progress and
    ends ACCEPTED; MOVE_STOP cancels a running move and completes at once.
    AUTO never exits by itself;
  * PARAM_REQUEST_READ -> PARAM_VALUE from a table.

Everything it received is kept in `self.received` so a test can assert on
what actually crossed the wire.
"""
from __future__ import annotations

import math
import os
import threading
import time

# The board speaks MAVLink 2 (COMMAND_ACK.progress is a v2 extension). Set
# before pymavlink picks its dialect -- import this module first.
os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil  # noqa: E402

ML = mavutil.mavlink

CMD_SROT_MOVE = 31000
MOVE_STOP = 6
MOVE_TURN = 4
MOVE_DIVE = 5
MODE_STABILIZE = 0
MODE_AUTO = 23
ACK_ACCEPTED = 0
ACK_TEMPORARILY_REJECTED = 1
ACK_IN_PROGRESS = 5
ACK_CANCELLED = 6


class FakeSrotBoard:
    def __init__(self, port: int, *, behaviour_rev: int = 14,
                 params: dict | None = None, boot_lines: list | None = None,
                 bench_motion: bool = False):
        self.port = port
        # A board on a bench: a slight tremor, a healthy IMU sending
        # SCALED_IMU2, and both BATTERY_STATUS instances, so every Batighor
        # plot moves. Off for the tests, which assert on a board at rest.
        self.bench_motion = bench_motion
        # The boot burst, if this board REPEATS it on REQUEST_MESSAGE(STATUSTEXT).
        # None = today's firmware, which accepts that request and sends nothing.
        # A UDP link has no auto-reset, so the burst is never seen otherwise.
        self.boot_lines = boot_lines
        self.behaviour_rev = int(behaviour_rev)
        # PILOT_YAW_RATE / ATC_RAT_YAW_P: the values measured on the bench board
        # (srot_params.check_yaw_authority). A param the fake does not answer costs
        # the manager's preflight a 3 s timeout, the real board answers them all.
        self.params = {'FRAME_REVERSE': 1.0, 'JS_GAIN_DEFAULT': 1.0,
                       'PILOT_YAW_RATE': 160.0, 'ATC_RAT_YAW_P': 0.18}
        # On the vehicle 1-8 are SERVO (role 1) and 9-16 SWITCH (role 2).
        for n in range(1, 17):
            self.params[f'SERVO{n}_ROLE'] = 1.0 if n <= 8 else 2.0
        self.params.update(params or {})
        # NAMED_VALUE_FLOATs the board streams: a locked yaw reference, dry.
        self.named = {'YAW_REF': 2.0, 'LEAK': 0.0}       # 2 = LOCKED
        self.armed = False
        self.mode = MODE_STABILIZE
        self.depth_m = -0.5
        self.yaw = 0.0
        self.received: list = []
        self._move = None              # (t_end, duration) of the running move
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        self._link = mavutil.mavlink_connection(
            f'udpout:127.0.0.1:{port}', source_system=1, source_component=1)
        self._thread = threading.Thread(target=self._run, daemon=True)

    # -- lifecycle -------------------------------------------------------- #
    def start(self):
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        self._thread.join(2.0)
        try:
            self._link.close()
        except Exception:                  # noqa: BLE001
            pass

    def commands(self, cmd_id):
        with self._lock:
            return [m for m in self.received
                    if m.get_type() == 'COMMAND_LONG' and m.command == cmd_id]

    # -- the board -------------------------------------------------------- #
    def _boot_ms(self):
        return int((time.monotonic() - self._t0) * 1000)

    def _ack(self, command, result, progress=0):
        self._link.mav.command_ack_send(command, result, progress, 0, 0, 0)

    def _heartbeat(self):
        base = ML.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base |= ML.MAV_MODE_FLAG_SAFETY_ARMED
        self._link.mav.heartbeat_send(
            ML.MAV_TYPE_SUBMARINE, ML.MAV_AUTOPILOT_GENERIC, base, self.mode,
            ML.MAV_STATE_ACTIVE if self.armed else ML.MAV_STATE_STANDBY)

    def _rock(self):
        """(roll, pitch, roll rate, pitch rate): a bench tremor, or the board at rest.

        Peak rate 0.011 rad/s, under localization's STILL_GYRO_RAD_S (0.02),
        so the hull still reads as still and the ZUPT holds velocity. With no
        flow and no ZUPT, depth is refused (B-56) and z integrates open loop:
        a 3 deg swell ran the bench's z to 646 m in a few minutes.
        """
        if not self.bench_motion:
            return 0.0, 0.0, 0.0, 0.0
        t = time.monotonic() - self._t0
        wr, wp = 2 * math.pi * 0.2, 2 * math.pi * 0.13
        ar, ap = math.radians(0.5), math.radians(0.4)
        return (ar * math.sin(wr * t), ap * math.sin(wp * t),
                ar * wr * math.cos(wr * t), ap * wp * math.cos(wp * t))

    def _imu(self, roll, pitch, droll, dpitch):
        # Specific force at rest in FRD, then into the sensor's axes the way
        # the real board packs it: (x, y, z) -> (y, x, -z), srot_telemetry.
        g = 1000.0                                                  # mG
        frd = (g * math.sin(pitch), -g * math.sin(roll) * math.cos(pitch),
               -g * math.cos(roll) * math.cos(pitch))
        acc = (frd[1], frd[0], -frd[2])
        self._link.mav.scaled_imu2_send(
            self._boot_ms(), *(int(a) for a in acc),
            int(droll * 1000), int(dpitch * 1000), 0, 0, 0, 0)

    def _battery(self):
        t = time.monotonic() - self._t0
        for bid, volts in ((0, 12.4), (1, 14.8 - 0.002 * (t % 300))):
            cells = [int(volts * 1000)] + [0xFFFF] * 9
            self._link.mav.battery_status_send(bid, 0, 0, 2500, cells,
                                               150 if bid else 80, -1, -1, -1)

    def _telemetry(self):
        roll, pitch, droll, dpitch = self._rock()
        self._link.mav.attitude_send(self._boot_ms(), roll, pitch, self.yaw,
                                     droll, dpitch, 0.0)
        if self.bench_motion:
            self._imu(roll, pitch, droll, dpitch)
        self._link.mav.vfr_hud_send(0.0, 0.0, int(math.degrees(self.yaw)) % 360,
                                    0, self.depth_m, 0.0)
        self._link.mav.ahrs2_send(0.0, 0.0, self.yaw, self.depth_m, 0, 0)

    def _named_values(self):
        for name, value in self.named.items():
            self._link.mav.named_value_float_send(self._boot_ms(),
                                                  name.encode(), float(value))

    def _sys_status(self):
        baro = ML.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        if self.bench_motion:
            # A healthy BNO085; without it the manager withholds every rate.
            baro |= ML.MAV_SYS_STATUS_SENSOR_3D_GYRO | ML.MAV_SYS_STATUS_SENSOR_3D_ACCEL
        self._link.mav.sys_status_send(baro, baro, baro, 100, 16000, -1, -1,
                                       0, 0, 0, 0, 0, 0)

    def _on_command(self, m):
        c = m.command
        if c == ML.MAV_CMD_REQUEST_MESSAGE and int(m.param1) == 148:
            # capabilities, flight_sw, MIDDLEWARE_SW (= behaviour rev), os_sw, ...
            self._link.mav.autopilot_version_send(
                0, 0, self.behaviour_rev, 0, 0, b'\0' * 8, b'\0' * 8,
                b'\0' * 8, 0, 0, 0)
            self._ack(c, ACK_ACCEPTED)
        elif c == ML.MAV_CMD_REQUEST_MESSAGE and int(m.param1) == ML.MAVLINK_MSG_ID_STATUSTEXT:
            for sev, text in self.boot_lines or []:
                self._link.mav.statustext_send(sev, text.encode())
            self._ack(c, ACK_ACCEPTED)
        elif c == ML.MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = m.param1 >= 0.5
            if not self.armed:
                self._move = None
            self._ack(c, ACK_ACCEPTED)
            self._heartbeat()
        elif c == ML.MAV_CMD_DO_SET_MODE:
            self.mode = int(m.param2)
            self._move = None
            self._ack(c, ACK_ACCEPTED)
            self._heartbeat()
        elif c == CMD_SROT_MOVE:
            prim = int(m.param1)
            if prim == MOVE_STOP:
                if self._move is not None:
                    self._move = None
                    self._ack(c, ACK_CANCELLED)
                self._ack(c, ACK_ACCEPTED, 100)
                return
            if not self.armed:
                self._link.mav.statustext_send(
                    ML.MAV_SEVERITY_WARNING, b'SROT_MOVE refused: arm first')
                self._ack(c, ACK_TEMPORARILY_REJECTED)
                return
            if prim in (MOVE_TURN, MOVE_DIVE):
                duration = 0.5
            else:
                duration = max(0.1, float(m.param2))
            self.mode = MODE_AUTO
            self._move = (time.monotonic() + duration, duration)
            self._ack(c, ACK_IN_PROGRESS, 0)
        else:
            self._ack(c, ACK_ACCEPTED)

    def _on_param_read(self, m):
        name = m.param_id if isinstance(m.param_id, str) else m.param_id.decode()
        name = name.rstrip('\0')
        value = float(self.params.get(name, 0.0))
        self._link.mav.param_value_send(name.encode(), value,
                                        ML.MAV_PARAM_TYPE_REAL32, 1, 0)

    def _advance_move(self):
        if self._move is None:
            return
        t_end, duration = self._move
        now = time.monotonic()
        if now >= t_end:
            self._move = None
            self._ack(CMD_SROT_MOVE, ACK_ACCEPTED, 100)
        else:
            frac = 1.0 - (t_end - now) / duration
            self._ack(CMD_SROT_MOVE, ACK_IN_PROGRESS, int(frac * 100))

    def _run(self):
        last_hb = last_tel = last_sys = last_prog = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            while True:
                m = self._link.recv_match(blocking=False)
                if m is None:
                    break
                with self._lock:
                    self.received.append(m)
                t = m.get_type()
                if t == 'COMMAND_LONG':
                    self._on_command(m)
                elif t == 'PARAM_REQUEST_READ':
                    self._on_param_read(m)
                elif t == 'PARAM_SET':
                    name = (m.param_id if isinstance(m.param_id, str)
                            else m.param_id.decode()).rstrip('\0')
                    self.params[name] = float(m.param_value)
                    self._on_param_read(m)
            # 10 Hz: what the manager asks the real board for (SROT_MESSAGE_RATES),
            # and what a finished move waits on -- a move is only believed done on
            # a heartbeat heard after its ACK.
            if now - last_hb >= 0.1:
                self._heartbeat()
                last_hb = now
            if now - last_tel >= 0.05:
                self._telemetry()
                last_tel = now
            if now - last_sys >= 0.5:
                self._sys_status()
                self._named_values()
                if self.bench_motion:
                    self._battery()
                last_sys = now
            if now - last_prog >= 0.1:
                self._advance_move()
                last_prog = now
            time.sleep(0.005)
