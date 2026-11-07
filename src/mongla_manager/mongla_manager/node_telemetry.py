"""What the node publishes about the vehicle: state, IMU, demand, and the console blocks.

Mixed into `AUVManagerNode` (auv_manager_node.py), which owns the board,
the publishers and the state these methods read.
"""
from __future__ import annotations

import math
import time

from mongla_interfaces.msg import MonglaState
from mongla_localization.frames import BODY as _BODY_FRAME

from . import srot_changes as _schg, srot_format as _sfmt
from .dispatch_policy import COMMAND_GATE_STALE_S
from .node_support import (BAT_CHANGE_THRESH, DEPTH_CHANGE_THRESH, FORCE_PRINT_SECONDS,
                           SROT_MESSAGE_RATES, YAW_CHANGE_THRESH, _SROT_DEPTH_OUT_WARN,
                           _kill_text, _quat_from_rpy)


class TelemetryMixin:
    def _publish_static_frames(self) -> None:
        """The tree's fixed edges (issue #19): FRD<->FLU, NED<->ENU, and each
        camera mount that has been MEASURED. Here because this node always
        runs; the moving edge is the localization node's.

        An unmeasured camera gets NO edge, not a zero one, so a lookup into it
        fails loudly -- the `sensor_offsets` health line says why.
        """
        try:
            from tf2_ros import StaticTransformBroadcaster
            from mongla_localization import frames
            self._tf_static = StaticTransformBroadcaster(self)
            stamp = self.get_clock().now().to_msg()
            edges = frames.static_edges(frames.load())
            self._tf_static.sendTransform([frames.to_msg(e, stamp) for e in edges])
            self.get_logger().info(
                '[FRAME] static tree: '
                + ', '.join(f'{e.parent}->{e.child}' for e in edges))
        except Exception as exc:          # noqa: BLE001 -- never fatal
            self.get_logger().error(f'[FRAME] static tree NOT published: {exc!r}')

    def _fast_state_tick(self):
        """Publish heading + depth at 20 Hz for real-time HUD instruments.

        Reads fresh attitude from the reader's cache (ATTITUDE pinned to 50 Hz)
        and reuses the last-known armed/mode/battery from telemetry_tick.
        """
        self._publish_demand()
        attitude = self.fc.get_attitude()
        if attitude is None:
            return
        yaw_deg, _ = self._effective_yaw_deg(attitude)
        msg = MonglaState()
        msg.header.stamp    = self._state_stamp()
        msg.header.frame_id = _BODY_FRAME
        msg.armed           = self._fast_armed
        msg.mode            = self._fast_mode
        msg.yaw_deg         = float(yaw_deg) if yaw_deg is not None else math.nan
        msg.depth_m         = float(attitude['depth'])
        msg.battery_voltage = self._fast_batt_v
        self.state_publisher.publish(msg)

    def _publish_demand(self):
        """20 Hz: the thruster demand in force, or NaN when it is not known."""
        demand = getattr(self.fc, 'demand', None)
        if demand is None:
            return                       # backend has no demand funnel
        d = demand()
        m = self._Vector3Stamped()
        # Informational only. This is REALTIME, which can step (first NTP
        # sync on the Pi did, mid-run). The consumer times the lag on its own
        # arrival monotonic() and must never subtract these stamps.
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = _BODY_FRAME
        m.vector.x, m.vector.y =(math.nan, math.nan) if d is None else d
        self.demand_publisher.publish(m)

    def _imu_rates_tick(self):
        """Publish body-frame angular rates (ATTITUDE) at 50 Hz.

        x=pitch_rate, y=roll_rate, z=yaw_rate (rad/s). The flow node buffers
        these and interpolates to each frame-pair for rotation compensation.
        Skips when ATTITUDE hasn't arrived (no stream / pre-connect).

        The stamp is the sender's capture time, not `now()`. This tick is a
        50 Hz timer polling a cache, so it is ASYNCHRONOUS to arrival: stamping
        `now()` added a uniform 0-20 ms of quantisation on top of transport
        delay, and de-rotation subtracts `f*omega*dt`, so that lands directly
        in the flow residual -- 5 ms is 1.6 px at 0.64 rad/s, and Qin & Shen
        put the whole tolerance at 6 ms.

        Measured on this vehicle: the BOARD's ATTITUDE interval is 20.00 ms
        with sd 0.00 while host ARRIVAL is 20.00 ms with sd 6.67 and p2p 35.12.
        All of the jitter is transport. `ClockMap` maps the board's own
        `time_boot_ms` onto host time from one-way pairs, fitted on the LOWER
        ENVELOPE because transport delay is strictly non-negative -- a fit
        through the middle of the cloud measures the mean delay, not the
        offset. The board does not implement MAVLink TIMESYNC (0 of 12
        requests answered, measured), so this is the available route.

        Falls back to arrival time, loudly, when the mapping is not yet
        established -- an unmapped board clock is not a host clock, and
        publishing it as one would be worse than the jitter it replaces.
        """
        rates = self.fc.get_angular_rates()
        if rates is None:
            return

        stamp_s = None
        board_ms = rates.get('board_ms')
        recv_s = rates.get('host_recv_s')
        if board_ms is not None:
            board_s = board_ms * 1e-3
            if recv_s is not None:
                steps = self._imu_clock.steps
                self._imu_clock.add(board_s, recv_s)
                if self._imu_clock.steps != steps:
                    # The host clock stepped (NTP) or the board rebooted. The
                    # map dropped the other clock's pairs; stamps fall back to
                    # arrival until it refits, and `_imu_clock_ok` re-announces.
                    self._imu_clock_ok = False
                    self.get_logger().warning(
                        f'[SENS ] clock STEP detected (#{self._imu_clock.steps}): '
                        f'host or board time jumped. IMU stamps on arrival '
                        f'until the board clock is re-mapped (~2 s).')
            now = time.monotonic()
            if now - self._imu_clock_fit_t >= 2.0:
                self._imu_clock_fit_t = now
                self._imu_clock.fit()
            if self._imu_clock.ready:
                stamp_s = self._imu_clock.to_host(board_s)
                # Share it with `/mongla/state`, which carries the SAME board
                # sample (yaw, depth) and was stamped on host publish time.
                # Cached here, inside the ready branch, and not after the
                # fallback below: past that point `stamp_s` may be the ARRIVAL
                # time, and caching that as a board stamp would relabel the
                # jitter instead of removing it -- the same lie in a better
                # disguise. Paired with its own monotonic instant so a stale
                # mapping cannot be applied to a fresh state message.
                self._board_stamp = (stamp_s, time.monotonic())
            elif not self._imu_clock_warned:
                self._imu_clock_warned = True
                self.get_logger().info(
                    '[SENS ] imu_rates: board clock not mapped yet, stamping '
                    'on arrival (adds ~6.7 ms sd of transport jitter). '
                    'Mapping needs a few seconds of ATTITUDE.')

        if stamp_s is None:
            stamp_s = recv_s if recv_s is not None else time.time()
        elif not self._imu_clock_ok:
            self._imu_clock_ok = True
            self.get_logger().info(
                f'[SENS ] imu_rates now stamped on the BOARD clock: '
                f'{self._imu_clock}')

        m = self._Vector3Stamped()
        m.header.stamp.sec = int(stamp_s)
        m.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        m.header.frame_id = _BODY_FRAME
        m.vector.x = rates['pitch_rate']
        m.vector.y = rates['roll_rate']
        m.vector.z = rates['yaw_rate']
        self.imu_rates_publisher.publish(m)
        self._drain_imu()

    def _drain_imu(self) -> None:
        """Publish every queued SCALED_IMU2 once, paired with its own ATTITUDE.

        Pairs by `time_boot_ms`: both are packed from one firmware tick, but
        the wire order is not a contract, so the cached ATTITUDE at the moment
        SCALED_IMU2 arrives may be the previous tick's. Each sample is stamped
        from ITS OWN board time, never the tick's.
        """
        q, aq = getattr(self, '_imu_q', None), getattr(self, '_att_q', None)
        if q is None or aq is None:
            return
        if not self._imu_clock.ready:
            q.clear()                    # no board-clock stamp to give them
            return
        atts = {}
        while aq:
            a = aq.popleft()
            atts[getattr(a, 'time_boot_ms', None)] = a
        self._att_by_ms = getattr(self, '_att_by_ms', {})
        self._att_by_ms.update(atts)
        if len(self._att_by_ms) > 64:
            for k in sorted(k for k in self._att_by_ms if k is not None)[:-32]:
                del self._att_by_ms[k]
        while q:
            imu = q.popleft()
            ms = getattr(imu, 'time_boot_ms', None)
            if ms is None or (self._imu_last_ms is not None and ms <= self._imu_last_ms):
                continue
            att = self._att_by_ms.get(ms)
            if att is None:
                # previous tick's attitude, never a later one
                older = [k for k in self._att_by_ms if k is not None and k < ms]
                att = self._att_by_ms[max(older)] if older else None
            self._imu_last_ms = ms
            self._publish_imu(self._imu_clock.to_host(ms * 1e-3),
                              imu_msg=imu, att_msg=att)

    def _publish_imu(self, stamp_s: float, imu_msg=None, att_msg=None) -> None:
        """Publish the 6-DoF sample on `/mongla/imu`, on the SAME mapped stamp.

        FRAME: body FRD, world NED -- MAVLink's and the board's, NOT ROS
        REP-103 (FLU/ENU). Everything that meets this topic is FRD already:
        flow velocity (+y right), the command model (starboard), course
        bearings, MANUAL_CONTROL. One convention end to end beats a REP-103
        conversion at each of those, any one of which can be missed silently.

        Shares `_imu_rates_tick`'s clock work rather than fitting a second
        `ClockMap`: ATTITUDE and SCALED_IMU2 are packed from one `Snap` in the
        same firmware tick (`mav_stream.cpp`), so they are the same instant and
        giving them two independently-fitted mappings would invent a lag
        between two halves of one sample.

        Silently absent when the board says the IMU is unhealthy -- a filter that gets no message coasts,
        which is correct; one that gets zeros integrates a lie.
        """
        getter = getattr(self.fc, 'get_imu', None)
        if getter is None:
            return
        imu = getter() if imu_msg is None else getter(imu_msg=imu_msg, att_msg=att_msg)
        # accel None = the board's accel frame is not proven yet (SrotFC).
        # Publishing without it would hand the filter gravity in the wrong axes.
        if imu is None or imu.get('accel') is None:
            return
        msg = self._Imu()
        msg.header.stamp.sec = int(stamp_s)
        msg.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        msg.header.frame_id = _BODY_FRAME
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = imu['gyro']
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = imu['accel']
        # The board's own attitude, and this is the whole point of the split.
        # It fuses the BNO085 at 500 Hz with the sensor on its own I2C bus and
        # holds heading to under 0.01 deg/min at rest (measured). Propagating
        # attitude again on the companion, from a gyro arriving at 50 Hz over a
        # serial link with nothing aiding it, is strictly worse -- and measured
        # on the vehicle it DIVERGED: an unaided inertial solution grows
        # through the gravity coupling, and the depth update's gain then pumps
        # that error into x and y. 7.1e6 m in 35 s.
        #
        # So the board owns attitude and the companion owns position. -1 in
        # element 0 stays the ROS "no data" convention for a board that cannot
        # supply it; the variance is the BNO's datasheet drift, not a guess
        # dressed as one.
        rpy = imu.get('rpy')
        if rpy is None:
            msg.orientation_covariance[0] = -1.0
            # Say so, once. Without the board's attitude the filter propagates
            # its own from a 50 Hz gyro with nothing aiding it, and that was
            # MEASURED to diverge to 7.1e6 m in 35 s. Publishing a valid
            # message with the orientation quietly marked absent is exactly
            # the failure this stack keeps repeating: the consumer degrades
            # correctly and nobody is told the degraded path is live.
            if not self._imu_rpy_warned:
                self._imu_rpy_warned = True
                self.get_logger().warning(
                    '[SENS ] /mongla/imu carries NO orientation: the backend '
                    'gave no attitude with the inertial sample. Localization '
                    'will propagate attitude itself and WILL drift.')
        else:
            qw, qx, qy, qz = _quat_from_rpy(*rpy)
            msg.orientation.w, msg.orientation.x = qw, qx
            msg.orientation.y, msg.orientation.z = qy, qz
            var = math.radians(0.5) ** 2
            msg.orientation_covariance[0] = var
            msg.orientation_covariance[4] = var
            msg.orientation_covariance[8] = var
        self.imu_publisher.publish(msg)

    def _effective_yaw_deg(self, attitude):
        """Return ``(yaw_deg, label)`` -- the SAME yaw the verbs read:
        ``yaw_source.read_yaw()``, or ``(None, 'N/A')`` when it is stale.

        No fallback to the cached attitude. None is how the source says
        STALE (250 ms). Falling through to ``attitude['yaw']`` undid that gate
        one line later: after a USB drop /mongla/state kept publishing the
        last yaw pymavlink ever cached. Absent is absent -- NaN downstream.
        The board also reports NaN yaw when its IMU is unhealthy; that is
        absence too, and NaN comparisons are silently False in every guard.
        """
        yaw = self.yaw_source.read_yaw()
        if yaw is None or math.isnan(float(yaw)):
            return None, 'N/A'
        return float(yaw), 'AHRS'

    def telemetry_tick(self):
        attitude = self.fc.get_attitude()
        battery  = self.fc.get_battery()
        mode     = self.fc.get_mode()
        armed    = self.fc.is_armed()

        yaw_deg, yaw_label = self._effective_yaw_deg(attitude)

        self._fast_armed  = bool(armed)
        self._fast_mode   = mode or ''
        self._fast_batt_v = float(battery['voltage']) if battery else math.nan

        # A claim that was never confirmed by execute_callback is a leak, and a
        # leaked gate rejects every later verb in silence. Clear it and say so.
        leaked = self._gate.reap_stale()
        if leaked is not None:
            self.get_logger().error(
                f'[ACT  ] gate claim for {leaked!r} was never confirmed by '
                f'execute_callback and has been released after '
                f'{COMMAND_GATE_STALE_S:.0f} s. Goals were being rejected for a '
                f'command that never ran -- this should not happen; capture the log.')

        self._maybe_print_state(attitude, battery, mode, armed, yaw_deg, yaw_label)
        self._publish_state(attitude, battery, mode, armed, yaw_deg)
        self._publish_srot_telemetry()

    def _reapply_srot_config(self) -> None:
        """Re-push the board configuration that a reboot cleared (B43).

        Deliberately only the two things a reboot silently loses and that we set
        ourselves. Not a full re-run of `_srot_preflight_reads`: the behaviour-rev
        and yaw-reference CHECKS are diagnostics whose answers cannot change
        across a reboot of the same firmware, and re-running their round-trips
        from a 2 Hz telemetry tick would add link traffic for no information.
        """
        try:
            for msg_id, hz in SROT_MESSAGE_RATES.items():
                self.fc.set_message_rate(msg_id, hz)
            gain_ok = self.fc.set_default_gain()
        except Exception as exc:                     # noqa: BLE001 -- best-effort
            self.get_logger().error(
                f'[SROT ] reconfigure after reboot FAILED: {exc!r}. The board is '
                f'running compiled defaults: MANUAL_CONTROL at half authority and '
                f'reduced stream rates.')
            return
        if gain_ok:
            self.get_logger().warning(
                '[SROT ] board restarted -- stream rates and JS_GAIN_DEFAULT '
                're-applied. Anything commanded between the reboot and now ran at '
                'half MANUAL_CONTROL authority.')
        else:
            self.get_logger().error(
                '[SROT ] board restarted and JS_GAIN_DEFAULT could NOT be '
                're-applied -- every MANUAL_CONTROL is at HALF authority until it '
                'is. Vision alignment and the arrival brake will under-correct.')

    def _publish_srot_telemetry(self):
        """Surface the parts of SrotFC.telemetry() that nothing else reads.

        `telemetry()` was fully implemented and had no production caller -- the
        manager reads the vehicle through the summary getters
        (get_attitude/get_battery/...), which have no RPM or leak. So the decode
        work, including the ESC_TELEMETRY fallback, went nowhere.
        """
        # An unplanned FC restart, checked here for the same reason `telemetry()`
        # is: `check_for_reboot()` was fully implemented, documented and unit
        # tested, and called by NOTHING outside its own test. A detector nobody
        # runs is not a detector.
        #
        # It matters most exactly where it was missing. After a reboot the board
        # is DISARMED, in its boot mode, with every setpoint cleared and stream
        # rates back to compiled defaults -- while the mission carries on issuing
        # verbs to a vehicle that is no longer the one it configured. Each verb
        # then fails in its own way, none of them naming the cause.
        #
        # So the active command is ABORTED rather than merely logged: a mission
        # step that continues here is steering nothing, and the fail-safe default
        # is to stop and let the operator see why.
        if self.fc.check_for_reboot():          # logs the cause itself
            if self.command_active:
                self.get_logger().error(
                    '[ACT  ] aborting the active command -- the board restarted '
                    'under it, so it is disarmed and no longer configured')
                self.mongla.request_abort()
            # ...and RECONFIGURE it (B43). Aborting the running command was only
            # half the job: the very next verb ran on a board still holding its
            # COMPILED DEFAULTS, and both losses are silent.
            #
            #   JS_GAIN_DEFAULT reverts to 0.5, so every MANUAL_CONTROL -- i.e.
            #   every vision align and every brake -- runs at HALF authority. The
            #   hull just corrects more weakly; nothing reports it.
            #
            #   Stream rates revert to the compiled defaults, so ATTITUDE drops
            #   ~55 Hz -> ~11 Hz. The vision loop's freshness decay then bleeds
            #   translational authority away on a link that looks fine.
            #
            # Neither is visible in any log, and this comment's own predecessor
            # already said the board was "no longer configured" while nothing
            # acted on that.
            self._reapply_srot_config()

        try:
            tel = self.fc.telemetry()
        except Exception as exc:                      # noqa: BLE001 -- telemetry is best-effort
            self.get_logger().debug(f'[TELEM] srot telemetry read failed: {exc!r}')
            return

        # Absence, not zeros. `tel.rpm` is a tuple of eight, so `if tel.rpm`
        # is TRUE even when every slot is 0 -- and the board fills all eight
        # slots whether or not an ESC is attached (measured: 958 CRC-valid
        # frames, no ESCs, every rpm exactly 0). This topic was therefore
        # publishing eight fabricated zeros at 2 Hz on a hull with no thrusters.
        #
        # The discriminator is NOT "are the values zero" -- an idle armed hull
        # with real ESCs also reads zero, and that is genuine data. It is
        # whether the board has ANNOUNCED presence, which is the same source of
        # truth the thrusters health reporter grades. Unannounced publishes
        # nothing, so a consumer sees no message rather than a confident zero.
        present, _lost = getattr(self.fc, 'esc_presence', lambda: (None, set()))()
        if tel.rpm and present and self.esc_rpm_publisher is not None:
            msg = self._Int32MultiArray()
            msg.data = [int(r) for r in tel.rpm]
            self.esc_rpm_publisher.publish(msg)
        self._publish_flare_order()

        # LEAK is edge-latched, not spammed: the board streams it as a
        # NAMED_VALUE_FLOAT and this tick runs at 2 Hz, so an un-latched log would
        # repeat every 500 ms for the rest of the dive.
        #
        # Read it as ADVISORY. SROT multiplexes MV_STATE / LEAK / WTEMP / GAIN onto
        # one msgid and pymavlink's cache keeps only the newest message of a type,
        # so any single poll has roughly a 1-in-N chance of being the one we want --
        # a missed leak here is expected and is NOT a safety mechanism. The board's
        # own leak failsafe surfaces the vehicle regardless; this is for the
        # operator. (Raised upstream: LEAK wants its own message or a SYS_STATUS
        # sensor-health bit.)
        self._maybe_print_srot_block(tel)

        if tel.leak and not self._leak_latched:
            self._leak_latched = True
            self.get_logger().error(
                '[TELEM] LEAK reported by the board -- it surfaces on its own '
                'failsafe; abort the mission and recover the vehicle')
        elif not tel.leak:
            self._leak_latched = False

    @staticmethod
    def _tel(value, fmt='{:.2f}', suffix=''):
        """Render a telemetry numeric, or `--` when absent. NEVER renders absence as 0.

        Delegates to `srot_format` so this log block, `connect`, its dashboard and
        `--json` all format identically. They drifted once already -- the board shows a
        0..360 heading and one path was printing the raw signed value.
        """
        return _sfmt.fmt(value, fmt, suffix)

    def _log_srot_changes(self, tel):
        """One line per meaningful CHANGE, alongside the periodic block.

        The periodic block is a continuous trace you correlate against what the vehicle
        did; this is the opposite view -- what changed while nobody was watching. A mode
        flip or a sensor going absent is a single line here instead of something you have
        to spot by diffing two identical-looking blocks a minute apart.

        Absent <-> present transitions are included on purpose: the board SUPPRESSES
        values it cannot stand behind, so a barometer that stops being reported is the
        board telling you something, and a change log that only watches numbers move
        would never mention it.
        """
        fields = {
            'armed': tel.armed,
            'mode': tel.mode or None,
            'heading_deg': tel.yaw_deg if not math.isnan(tel.yaw_deg) else None,
            'depth_m': tel.depth_m if not math.isnan(tel.depth_m) else None,
            'battery_v': tel.battery_voltage,
            'thruster_v': tel.thruster_voltage,
            'water_temp_c': tel.water_temp_c,
            'depth_out': tel.depth_out,
            'depth_err_m': tel.depth_err_m,
            'mag_accuracy': tel.mag_accuracy,
            'leak': tel.leak,
            'kill': tel.kill_switch,
        }
        for lvl, _field, msg in _schg.diff(self._srot_prev_fields, fields):
            line = f'[SROT ] ~ {msg}'
            if lvl == 'CRIT':
                self.get_logger().error(line)
            elif lvl == 'WARN':
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
        self._srot_prev_fields = fields

    def _maybe_print_srot_block(self, tel):
        """The verbose SROT telemetry block -- everything the summary getters omit.

        Rate-limited by `srot_telemetry_period_s` (0 disables). This is deliberately
        periodic rather than on-change: for the first water test the operator wants a
        continuous trace they can correlate against what the vehicle physically did,
        and an on-change filter hides "nothing is changing", which for a depth loop is
        itself the interesting observation.
        """
        if self._srot_block_period <= 0:
            return
        now = time.time()
        if now - self._srot_block_last < self._srot_block_period:
            return
        self._srot_block_last = now

        self._log_srot_changes(tel)

        rpm = _sfmt.rpm_row(tel.rpm)
        etemp = _sfmt.esc_temp_row(tel.esc_temp_c)
        self.get_logger().info(
            f'[SROT ] BAT main {self._tel(tel.battery_voltage, "{:5.2f}", "V")} | '
            f'thruster {self._tel(tel.thruster_voltage, "{:5.2f}", "V")} | '
            f'DEPTH {self._tel(tel.depth_m, "{:+.2f}", "m")} '
            f'err {self._tel(tel.depth_err_m, "{:+.2f}", "m")} '
            f'out {self._tel(tel.depth_out, "{:+.2f}")} | '
            f'WTEMP {self._tel(tel.water_temp_c, "{:.1f}", "C")} | '
            f'MAGACC {self._tel(tel.mag_accuracy, "{:.0f}")} | '
            f'LEAK {"WET" if tel.leak else "dry"} | '
            f'KILL {_kill_text(tel.kill_switch)}')
        self.get_logger().info(f'[SROT ] RPM  {rpm}')
        if tel.esc_temp_c:
            self.get_logger().info(f'[SROT ] ESC°C{etemp}')

        # A depth loop saturated while DISARMED is the pre-arm tell that arming would
        # command full vertical thrust (mixer throttle column = -1 on all 4 verticals).
        if (not tel.armed and not math.isnan(tel.depth_out)
                and abs(tel.depth_out) >= _SROT_DEPTH_OUT_WARN):
            self.get_logger().error(
                f'[SROT ] DEPTH LOOP SATURATED while disarmed (out='
                f'{tel.depth_out:+.2f}, err={self._tel(tel.depth_err_m, "{:+.2f}", "m")}) '
                f'-- arming would command FULL vertical thrust. Check the barometer.')

    def _maybe_print_state(self, attitude, battery, mode, armed, yaw_deg, yaw_label):
        now  = time.time()
        prev = self.prev_state
        changed = (
            prev.get('arm')  != armed
            or prev.get('mode') != mode
            or abs(prev.get('yaw',   0)
                   - (yaw_deg if yaw_deg is not None else 0)) > YAW_CHANGE_THRESH
            or abs(prev.get('depth', 0)
                   - (attitude['depth'] if attitude else 0)) > DEPTH_CHANGE_THRESH
            or abs(prev.get('bat',   0)
                   - (battery['voltage'] if battery else 0)) > BAT_CHANGE_THRESH
            or (now - self.last_print_time) > FORCE_PRINT_SECONDS
        )
        if not changed:
            return

        arm_str   = 'ARM' if armed  else '---'
        if yaw_deg is not None:
            yaw_str = f'{yaw_deg:6.1f} ({yaw_label})'
        else:
            yaw_str = '   N/A'
        # Through the shared formatter: a NaN depth is ABSENT (the board suppresses
        # what it cannot stand behind), and `+nanm` on an operator's screen is neither
        # a reading nor a legible way to say "no barometer".
        depth_str = (_sfmt.fmt(attitude['depth'], '{:+6.2f}', 'm') if attitude
                     else '   N/A')
        bat_str   = (_sfmt.fmt(battery['voltage'], '{:5.1f}', 'V') if battery
                     else '  N/A')
        self.get_logger().info(
            f'[STATE] {arm_str} | {mode:<10} | '
            f'YAW:{yaw_str} | DEPTH:{depth_str} | BAT:{bat_str}')
        self.prev_state = {
            'arm':   armed, 'mode': mode,
            'yaw':   yaw_deg if yaw_deg is not None else 0,
            'depth': attitude['depth']  if attitude else 0,
            'bat':   battery['voltage'] if battery else 0,
        }
        self.last_print_time = now

    def _state_stamp(self):
        """Board capture time for `/mongla/state`, or host time if unmapped.

        `/mongla/state` carries yaw and depth, which ORIGINATE ON THE BOARD and
        were stamped with the instant the host got round to publishing. The
        board's own interval has sd 0.00 ms where arrival has sd 6.67 and p2p
        35.12 -- so the stamp described the transport, not the measurement.

        `flow_node` reads depth from this topic and differences it over time
        (`_vz_down`), which put that jitter straight into a vertical speed.
        `ClockMap` is already fitted here for `/mongla/imu_rates`; this is the
        same mapping applied to the other stream that needs it.
        """
        bs = self._board_stamp
        if bs is not None and (time.monotonic() - bs[1]) <= self._BOARD_STAMP_MAX_AGE_S:
            sec = int(bs[0])
            stamp = self.get_clock().now().to_msg()
            stamp.sec = sec
            stamp.nanosec = int((bs[0] - sec) * 1e9)
            return stamp
        return self.get_clock().now().to_msg()

    def _publish_state(self, attitude, battery, mode, armed, yaw_deg):
        if attitude is None and battery is None:
            return
        msg = MonglaState()
        msg.header.stamp    = self._state_stamp()
        msg.header.frame_id = _BODY_FRAME
        msg.armed           = bool(armed)
        msg.mode            = mode if mode else ''
        # Publish the yaw the verbs read, so downstream consumers of
        # /mongla/state see the same number as [STATE].
        msg.yaw_deg         = float(yaw_deg) if yaw_deg is not None else math.nan
        msg.depth_m         = float(attitude['depth'])  if attitude else math.nan
        msg.battery_voltage = float(battery['voltage']) if battery  else math.nan
        self.state_publisher.publish(msg)
