"""Reading the vehicle: attitude, IMU, batteries, and the one telemetry snapshot.

Every read is a non-blocking look in pymavlink's message cache, filled by the
manager's reader thread (the threading rule in `srot_fc.py`). A value the board
did not send, or sent too long ago, is NaN or None here, never 0.

Mixed into `SrotFC`.
"""
from __future__ import annotations

import collections
import math
import time

from ._srot_base import _ARMED_FLAG, _LINK_STALE_S, _rx_age, mavutil
from .base import Telemetry
from . import srot_protocol as sp


class SrotTelemetry:
    """The telemetry part of `SrotFC` (see the module docstring)."""

    def telemetry(self) -> Telemetry:
        t = Telemetry()
        hb = self._vehicle_hb()
        if hb is not None:
            t.armed = bool(hb.base_mode & _ARMED_FLAG)
            t.mode = sp.mode_name(hb.custom_mode)
            t.link_alive = _rx_age(hb) <= _LINK_STALE_S
        att = self._fresh('ATTITUDE')
        if att is not None:
            t.yaw_deg = math.degrees(att.yaw) % 360.0
            t.roll_deg = math.degrees(att.roll)
            t.pitch_deg = math.degrees(att.pitch)
        # Depth: NEGATIVE below the surface, as
        # MonglaState.msg documents. VFR_HUD.alt is already in that convention
        # (fw sends alt = -depth), so pass it through -- see get_attitude().
        # Stays NaN until VFR_HUD arrives (5 Hz).
        #
        # Gated on baro health, and that gate is not optional. Unlike
        # SCALED_PRESSURE2 and WTEMP -- which the firmware SUPPRESSES when the
        # barometer is unhealthy or stale -- VFR_HUD is sent unconditionally
        # (fw mav_stream.cpp:258). So a board with NO Bar30 fitted at all still
        # streams `alt = -0.000` forever, and passing that through publishes a
        # confident 0.00 m on /mongla/state, in [STATE], and into every depth guard
        # that compares against a negative constant. Read the health bit instead;
        # the board already refuses DEPTH_HOLD/AUTO on the same condition
        # (fw task_control_loop.cpp:107 and :348), so this stays consistent with
        # what the vehicle will actually let you do.
        vhud = self._fresh('VFR_HUD')
        if vhud is not None and self._baro_healthy() is not False:
            t.depth_m = float(vhud.alt)
        # Read through the per-instance table, NOT `_cache('BATTERY_STATUS')`. The raw
        # slot holds whichever instance landed last, so this line used to claim
        # "id 0 = electronics pack" while actually reporting the THRUSTER pack about
        # half the time -- caught by a live smoke test showing battery_voltage and
        # thruster_voltage identical at 14.63 V when PM1 reads ~1.35 V.
        batteries = self.get_batteries()
        main = batteries.get(sp.BATTERY_ID_MAIN)
        if main is not None:
            t.battery_voltage = main['voltage']
        # Per-thruster RPM (Bluejay bidirectional DShot).
        #
        # This is always empty on pymavlink 2.4.49: upstream MAVLink removed the
        # WIP messages 290/291 from `common`, so ESC_STATUS (291) is in NO dialect we
        # ship (checked common / ardupilotmega / all / development). pymavlink drops
        # any msgid missing from its CRC-extra table SILENTLY -- no error, no callback
        # -- so the board can be reporting RPM perfectly while we read nothing. Bondor
        # lost every RPM packet to exactly this and it looked like an ESC fault.
        #
        # RESOLVED as of fw behaviour rev 2, exactly as this comment proposed: the board
        # now ALSO emits ESC_TELEMETRY_1_TO_4 (11030) + ESC_TELEMETRY_5_TO_8 (11031),
        # which ARE in the ardupilotmega dialect and carry 4 ESCs each -- an exact fit
        # for our 8 thrusters. `/mongla/esc_rpm` is built on that fallback below (see
        # `_publish_srot_telemetry`), NOT on the ESC_STATUS branch, which stays only so
        # the read is free if pymavlink ever regains 291.
        esc = self._cache('ESC_STATUS')
        if esc is not None:
            t.rpm = tuple(int(r) for r in getattr(esc, 'rpm', ()) or ())
        elif t.rpm == ():
            for name in ('ESC_TELEMETRY_1_TO_4', 'ESC_TELEMETRY_5_TO_8'):
                block = self._cache(name)
                if block is not None:
                    t.rpm = t.rpm + tuple(int(r) for r in getattr(block, 'rpm', ()) or ())
        # The latched health bit FIRST, the multiplexed scalar only as a fallback.
        # NAMED_VALUE_FLOAT('LEAK') is deprecated as of rev 3 and is observed
        # roughly one call in fifteen; the bit is deterministic.
        leak_bit = self.sys_status_leak()
        if leak_bit is not None:
            t.leak = leak_bit
        else:
            leak = self._named_value('LEAK')
            if leak is not None:
                t.leak = leak >= 0.5
        wtemp = self._named_value('WTEMP')
        if wtemp is not None:
            t.water_temp_c = wtemp
        # NOT nested under WTEMP. The board SUPPRESSES WTEMP when the barometer is
        # unhealthy, so gating ESC temperatures on it would make thruster temps vanish
        # in exactly the situation where you most want them -- a sick vehicle.
        for name in ('ESC_TELEMETRY_1_TO_4', 'ESC_TELEMETRY_5_TO_8'):
            block = self._cache(name)
            if block is not None:
                t.esc_temp_c = t.esc_temp_c + tuple(
                    int(x) for x in getattr(block, 'temperature', ()) or ())

        # Everything below is board-specific. Each stays NaN
        # (not 0.0) when the board has not said it -- the board SUPPRESSES values it
        # cannot stand behind, so absence is a distinct, meaningful state.
        thr = self.get_batteries().get(sp.BATTERY_ID_THRUSTER)
        if thr is not None:
            t.thruster_voltage = thr['voltage']
        for attr, name in (('depth_err_m', 'DEPTH_ERR'), ('depth_out', 'DEPTH_OUT'),
                           ('mag_accuracy', 'MAGACC')):
            val = self._named_value(name)
            if val is not None:
                setattr(t, attr, val)
        # BARO_HEALTH is truncated to 'BARO_HEALT' on the wire: NAMED_VALUE_FLOAT's
        # name field is 10 chars and the board does not shorten it itself. Reading the
        # untruncated name finds nothing, silently -- which is exactly how this value
        # was once read as a health SCORE rather than a fault CODE (3 = not initialised,
        # not "very healthy").
        baro_h = self._named_value(sp.NAME_BARO_HEALTH)
        if baro_h is not None:
            t.baro_health = baro_h
        # Tri-state: absent stays None, so "the board never said" is not "never seen".
        comp = self._named_value('COMP_SEEN')
        if comp is not None:
            t.companion_seen = comp >= 0.5
        # Kill = 0 is not "clear". The kill switch is a rotary knob on the
        # SECOND board and its state crosses to this one over ESP-NOW; on link
        # loss the firmware reports kill=false on purpose --
        # `kill = f ? s_kill : false;  // link lost -> don't assert kill
        # (display-only)` (espnow_link.cpp:49). But it is NOT display-only: it
        # is packed into NAMED_VALUE_FLOAT "KILL" and read here.
        #
        # So a bare `KILL >= 0.5` reads "power is live" for a vehicle whose
        # second board is unpowered, out of range, or simply not built -- and
        # the operator's status line says `KILL clear` while nothing on the
        # vehicle can see the switch at all.
        #
        # BATTERY_STATUS instance 1 is the disambiguator and costs nothing: the
        # board SUPPRESSES it rather than zeroing it when the ESP-NOW link is
        # stale (`if (s.pm2_present) sendBattery(1, ...)`, and pm2_present is
        # gated on ESPNOW_STALE_MS). Its presence therefore IS the link
        # liveness, already demuxed by id in `get_batteries()`.
        if thr is None:
            t.kill_switch = None            # no 2nd-board link -> nobody knows
        else:
            kill = self._named_value('KILL')
            t.kill_switch = None if kill is None else (kill >= 0.5)
        return t

    # ------------------------------------------------------------------ #
    #  Summary getters -- what the manager's state and HUD paths read      #
    # ------------------------------------------------------------------ #

    def get_attitude(self):
        """{'yaw'(deg 0..360),'roll'(deg),'pitch'(deg),'depth'(m)} or None.

        yaw in degrees (ATTITUDE is radians).

        Depth sign: this stack's convention -- MonglaState.msg, set_depth's input
        -- is NEGATIVE below the surface. The board is positive-DOWN internally
        but already negates
        on the wire (`VFR_HUD.alt = -depth`, fw mav_stream.cpp:210-218), so alt is
        ALREADY in our convention. Pass it straight through: negating here too
        made /mongla/state.depth_m positive when submerged, which silently
        inverted every one of those guards (they all compare against a negative
        constant, so each just stopped firing).

        Stale is absent. pymavlink keeps the last ATTITUDE forever, so after a
        USB drop this kept returning the final yaw/depth the board ever sent, and
        /mongla/state published a frozen, confident vehicle at 2 Hz -- exactly the
        "plausible number standing in for an absent measurement" of safety rule 6.
        ATTITUDE is pinned at 50 Hz, so an ATTITUDE older than `_LINK_STALE_S`
        (the same window `link_alive` uses) means the link, not the vehicle, went
        quiet: None, which every caller already handles as "not yet received"."""
        att = self._cache('ATTITUDE')
        if att is None:
            return None
        age = self.get_attitude_age()
        if age is None or age > _LINK_STALE_S:
            return None
        # Same baro-health gate as telemetry() -- see the long note there. VFR_HUD is
        # NOT suppressed when the barometer is unhealthy, so an ungated read publishes a
        # confident 0.00 m off a board with no Bar30 fitted, and every depth guard here
        # compares against a negative constant (they would simply stop firing).
        depth = math.nan
        vhud = self._fresh('VFR_HUD')
        if vhud is not None and self._baro_healthy() is not False:
            depth = float(vhud.alt)
        # The SAME gate as depth above, on the other sensor. NaN, not a dropped
        # dict: depth is independent of the IMU and is frequently healthy while
        # the BNO is not, so blanking the whole reading would trade one silent
        # failure for another. `MonglaState.msg` documents NaN as the absence
        # sentinel for exactly this.
        if self._ahrs_healthy() is False:
            return {'yaw': math.nan, 'roll': math.nan,
                    'pitch': math.nan, 'depth': depth}
        return {
            'yaw':   math.degrees(att.yaw) % 360.0,
            'roll':  math.degrees(att.roll),
            'pitch': math.degrees(att.pitch),
            'depth': depth,
        }

    def get_attitude_age(self):
        att = self._cache('ATTITUDE')
        return None if att is None else _rx_age(att)

    def get_mode(self):
        """The board's mode, or 'UNKNOWN' -- including when the link has gone quiet.

        A heartbeat older than _LINK_STALE_S says what the mode WAS. Reported as
        current, it let a dead link read "STABILIZE" to every caller that gates
        on the mode. (Cloud audit, 2026-11-07.)"""
        hb = self._vehicle_hb()
        if hb is None or _rx_age(hb) > _LINK_STALE_S:
            return 'UNKNOWN'
        return sp.mode_name(hb.custom_mode)

    def _fresh(self, name):
        """The cached `name` message, or None once it is older than _LINK_STALE_S.

        pymavlink keeps the last message of each type forever. ATTITUDE is
        pinned at 50 Hz and VFR_HUD at 5 Hz, so one older than the link window
        means the stream stopped -- and its last value is a plausible number
        standing in for an absent measurement (safety rule 6). VFR_HUD in
        particular was read with no age at all, beside an ATTITUDE that was
        gated, so depth could freeze while yaw went absent. (Cloud audit,
        2026-11-07.)"""
        msg = self._cache(name)
        if msg is None or _rx_age(msg) > _LINK_STALE_S:
            return None
        return msg

    def note_battery(self, msg):
        """Hook for the manager's MAVLink reader thread -- de-multiplex by instance id.

        Measured on the vehicle (2026-08-02): the board streams BATTERY_STATUS id 0
        (PM1, electronics) and id 1 (PM2, thruster pack) at 2 Hz EACH. pymavlink keys
        its cache by msgid, so `master.messages['BATTERY_STATUS']` alternates between
        them -- a live sample showed the slot flipping between **1.35 V and 14.74 V**.
        Reading that slot is therefore a coin flip on WHICH BATTERY you are reporting,
        and the two differ by an order of magnitude, so it cannot even be averaged away.

        Same failure as NAMED_VALUE_FLOAT, one layer down, and the same fix: key by the
        field that distinguishes them and feed it from the only loop that sees them all.
        """
        bid = int(getattr(msg, 'id', 0))
        volts = list(getattr(msg, 'voltages', []) or [])
        raw_mv = volts[0] if volts else 0xFFFF
        voltage = math.nan if raw_mv in (0, 0xFFFF) else raw_mv / 1000.0
        cur = getattr(msg, 'current_battery', -1)
        current = math.nan if cur == -1 else cur / 100.0
        self._note_battery_step(bid, voltage)
        self._battery_cache[bid] = (voltage, current, time.monotonic())

    def _note_battery_step(self, bid: int, voltage: float) -> None:
        """Track how far this instance moves between samples, so an UNWIRED
        sense line can be told from a battery."""
        prev = self._battery_last.get(bid)
        self._battery_last[bid] = voltage
        if prev is None or not (math.isfinite(prev) and math.isfinite(voltage)):
            return
        ring = self._battery_steps.setdefault(
            bid, collections.deque(maxlen=sp.BATTERY_STEP_WINDOW))
        ring.append(abs(voltage - prev))

    def battery_trusted(self, bid: int) -> bool:
        """Is this instance reporting a battery, or an unwired pin?

        Measured on the vehicle 2026-09-22, exclusive port, 41 frames of
        instance 0, nothing else reading:

            span 1.39 .. 25.05 V, sd 8.72 V
            MEDIAN step between consecutive samples 6.48 V (max 20.42)
            57.5 % of steps exceed 2 V

        `get_batteries()` already documents why: "On this vehicle PM1 reads
        ~1.3 V because nothing is wired to GPIO36." The ADC floats, and the
        stack was promoting that float to `battery_voltage` -- the manager
        printed `BAT main 1.39V` on a healthy bench.

        The cost was not only cosmetic. The manager's state LOG is gated on change
        with a 0.20 V battery threshold, and 67.5 % of these steps cross it: in
        one manager run, **2939 of 2939 state-change log lines were this pin**.
        Real state changes were buried under a floating input.

        CORRECTED: this said "`/mongla/state` publishes ON CHANGE". It does not
        -- `_publish_state` is called unconditionally from `telemetry_tick`, and
        the change threshold belongs to `_maybe_print_state`, a console printer.
        The guard below is unaffected (a floating pin must not reach
        `battery_voltage` either way); only the attribution was wrong.

        The MEDIAN is the statistic, not the max: one glitchy sample from a real
        pack must not blank it, while a pin that is noise every sample cannot
        hide behind a few quiet ones. A real pack sags ~1 V under load
        (`srot_changes.py`), so the bar sits at 2 V -- between the two by a
        factor of three either way.
        """
        ring = self._battery_steps.get(bid)
        if not ring or len(ring) < sp.BATTERY_STEP_MIN_SAMPLES:
            return True                 # not enough evidence to accuse it
        mid = sorted(ring)[len(ring) // 2]
        return mid <= sp.BATTERY_MAX_STEP_V

    def _baro_healthy(self):
        """True / False / None (never reported) from SYS_STATUS's health bitfield.

        Tri-state on purpose: "the board says the barometer is bad" and "we have not
        heard a SYS_STATUS yet" are different, and only the first should suppress a
        depth reading. Treating the second as unhealthy would blank depth for the first
        half-second of every connection.
        """
        msg = self._cache('SYS_STATUS')
        if msg is None:
            return None
        bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        return bool(int(getattr(msg, 'onboard_control_sensors_health', 0)) & bit)

    def _ahrs_healthy(self):
        """True / False / None (never reported) for the BNO085 behind ATTITUDE.

        The exact tri-state shape as `_baro_healthy`, and for the same reason: a
        board that has not sent SYS_STATUS yet has not said the IMU is bad.

        Measured on the vehicle 2026-09-11, which is why this exists. After a
        bad boot the board reported `present=0x80203c0b health=0x2408` -- 3D_GYRO
        (0x01) and 3D_ACCEL (0x02) present and NOT healthy -- with MAGACC 0.0,
        COMP_SEEN 0.0, YAW_REF 0.0, and it kept streaming ATTITUDE at 10 Hz with
        roll/pitch/yaw AND all three rates EXACTLY 0.0. `SCALED_IMU2` was zero in
        every field including accel, which the firmware packs as `s.lx + s.grx`
        (mav_stream.cpp:230) -- so at rest a live sensor owes ~1000 mG on one
        axis and zero is the sensor, not the encoding.

        A confident 0.0 is the worst possible failure here: `_effective_yaw_deg`
        would publish heading 0.0, and `turn` would believe it was already pointing north. Nothing
        anywhere logged a fault, because nothing asked.

        This is the barometer gate's twin. That half was fixed when a failing
        Bar30 saturated the depth PID; the attitude half was left open, in this
        same file, against the same bitfield.
        """
        msg = self._cache('SYS_STATUS')
        if msg is None:
            return None
        health = int(getattr(msg, 'onboard_control_sensors_health', 0))
        bits = (mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
                | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL)
        # BOTH must be healthy. The attitude solution is a fusion of the two, so
        # either one failing makes the quaternion untrustworthy.
        return (health & bits) == bits

    def _drain_battery(self):
        """Fold the currently-cached BATTERY_STATUS in, once per message object.

        The identity guard is load-bearing for the same reason as `_drain_named`'s:
        pymavlink never clears its slot, so re-folding would re-stamp a dead link's
        last reading as fresh forever.
        """
        msg = self._cache('BATTERY_STATUS')
        if msg is not None and msg is not self._last_batt:
            self._last_batt = msg
            self.note_battery(msg)

    def get_batteries(self, max_age_s: float = 5.0):
        """{id: {'voltage','current'}} for every battery seen recently.

        id 0 = PM1 (electronics rail), id 1 = PM2 (thruster pack, via ESP-NOW from the
        2nd board). An id absent here means it has not been heard -- NOT that it reads
        zero. On this vehicle PM1 reads ~1.3 V because nothing is wired to GPIO36.
        """
        self._drain_battery()
        now = time.monotonic()
        out = {}
        for bid, (v, c, stamp) in self._battery_cache.items():
            if (now - stamp) > max_age_s:
                continue
            # An instance that is measurably noise reports NaN, which renders
            # `--`. Absence renders `--`, never a number -- and a pin that moves
            # 6.48 V between samples is absence wearing a value.
            if not self.battery_trusted(bid):
                v = math.nan
            out[bid] = {'voltage': v, 'current': c}
        return out

    def get_battery(self):
        """{'voltage','current'} (V/A) for the MAIN battery (id 0), or None.

        The manager reads battery['voltage'] for /mongla/state. Pinned to id 0 rather than 'whatever arrived
        last' -- see note_battery for why that distinction is not cosmetic.
        """
        self._drain_battery()
        hit = self._battery_cache.get(sp.BATTERY_ID_MAIN)
        if hit is None:
            return None
        voltage, current, _ = hit
        return {'voltage': voltage, 'current': current}

    def get_statustext(self):
        st = self._statustext()
        return st or None

    def get_angular_rates(self):
        """Body-frame angular rates (rad/s) from ATTITUDE:
        {'roll_rate','pitch_rate','yaw_rate','age_s'} or None. The manager's
        _imu_rates_tick reads the *_rate keys, so they MUST match exactly.

        Also returns `board_ms` -- ATTITUDE.time_boot_ms, THE BOARD'S OWN
        Capture time.

        Why that field matters and why it was being wasted. Measured on this
        vehicle at 50 Hz: the BOARD's interval is 20.00 ms with sd 0.00, and
        the HOST's arrival interval is 20.00 ms with sd 6.67 and p2p 35.12.
        The board's clock is exact; every bit of that jitter is transport
        (ESP32 UART FIFO thresholding plus USB-serial scheduling). Anything
        that stamps on arrival inherits all of it, and de-rotation subtracts
        `f*omega*dt`, so 5 ms costs 1.6 px at 0.64 rad/s.

        This field was already parsed and read by exactly one caller -- the
        unplanned-reboot detector -- and thrown away everywhere else. It is the
        jitter-free time base the flow pipeline needs; `flow_timing.ClockMap`
        maps it onto host time (the board does not implement MAVLink TIMESYNC:
        0 of 12 requests answered, measured).

        `board_ms` is None when the field is absent, never 0 -- absence is not
        the boot instant.
        """
        att = self._cache('ATTITUDE')
        if att is None:
            return None
        # ABSENCE, not zeros. A dead BNO streams exact 0.0 rates (measured), and
        # the flow node de-rotates by `f*omega*dt` -- so a zero rate is not a
        # harmless neutral, it is a claim that the hull did not turn.
        if self._ahrs_healthy() is False:
            return None
        age = _rx_age(att) if (getattr(att, '_mono', None) is not None or getattr(att, '_timestamp', 0.0)) else 0.0
        boot = getattr(att, 'time_boot_ms', None)
        return {'roll_rate': float(getattr(att, 'rollspeed', 0.0)),
                'pitch_rate': float(getattr(att, 'pitchspeed', 0.0)),
                'yaw_rate': float(getattr(att, 'yawspeed', 0.0)),
                'age_s': age,
                'board_ms': None if boot is None else int(boot),
                'host_recv_s': float(getattr(att, '_timestamp', 0.0)) or None}

    def get_imu(self, imu_msg=None, att_msg=None):
        """Full 6-DoF inertial sample from SCALED_IMU2, or None.

        `imu_msg` / `att_msg`: a specific SCALED_IMU2 and its ATTITUDE, as the
        manager's reader thread queued them. Omitted, the one-slot cache is read
        -- which a 50 Hz poller of the board's ~44 Hz stream aliases (measured: 384
        of 2501 publishes were repeats, and 42.4 of 44.0 Hz of samples got out).

        {'gyro': (x,y,z) rad/s, 'accel': (x,y,z) m/s^2 INCLUDING gravity,
         'board_ms': int|None, 'host_recv_s': float|None}

        This data was already on the wire and entirely thrown away. The board
        streams SCALED_IMU2 at 50 Hz (60 messages in 6 s, measured) and this
        stack read exactly ONE field off it -- `temperature`, for the water-temp
        readout. Accel and gyro were decoded by pymavlink and discarded every
        single frame. Same shape as the ESC RPM finding: the sensor is paid for
        in bandwidth and never spent.

        The firmware packs accel as `(s.lx + s.grx) / 9.80665 * 1000` in mg and
        gyro as `s.gx * 1000` in mrad/s (mav_stream.cpp:230-240), so accel
        carries gravity and a level hull at rest owes ~1000 mg on one axis.
        Exact zero on all six is the dead-sensor signature, not a neutral
        reading -- which is why this shares `_ahrs_healthy` with the attitude
        path rather than trusting the numbers.

        `board_ms` is the board's own capture time, the jitter-free base
        `flow_timing.ClockMap` maps onto host time. None when absent, never 0.
        """
        imu = self._cache('SCALED_IMU2') if imu_msg is None else imu_msg
        if imu is None:
            return None
        if self._ahrs_healthy() is False:
            return None
        boot = getattr(imu, 'time_boot_ms', None)
        g = 9.80665 / 1000.0          # mg -> m/s^2
        # The board's OWN fused attitude, carried alongside the raw axes.
        # ATTITUDE and SCALED_IMU2 are packed from one `Snap` in one firmware
        # tick, so this is the same instant and needs no second stamp.
        att = self._cache('ATTITUDE') if imu_msg is None else att_msg
        rpy = None if att is None else (float(att.roll), float(att.pitch),
                                        float(att.yaw))
        raw = (float(imu.xacc) * g, float(imu.yacc) * g, float(imu.zacc) * g)
        return {
            'rpy': rpy,
            'gyro': (float(imu.xgyro) * 1e-3,
                     float(imu.ygyro) * 1e-3,
                     float(imu.zgyro) * 1e-3),
            # VEHICLE frame (FRD), or None until the frame is proven -- see
            # `_accel_in_vehicle_frame`. None, never the raw axes: a consumer
            # handed the wrong frame integrates gravity sideways.
            'accel': self._accel_in_vehicle_frame(raw, rpy),
            'accel_frame': getattr(self, '_accel_frame', None),
            'board_ms': None if boot is None else int(boot),
            'host_recv_s': float(getattr(imu, '_timestamp', 0.0)) or None,
        }

    def _accel_in_vehicle_frame(self, raw, rpy):
        """SCALED_IMU2 accel, rotated into the frame its own gyro and ATTITUDE use.

        The board sends them in two different frames. `bno085.cpp` (under
        `BNO_SWAP_ROLL_PITCH`) maps the attitude and the gyro from the sensor's
        axes into the vehicle's FRD frame with (x, y, z) -> (y, x, -z), a proper
        180 deg rotation about x=y. The gravity and linear-acceleration reports
        get NO remap, and `mav_stream.cpp` packs `lx + grx` straight into
        `xacc`. So the accel alone is still in the sensor's axes.

        Measured on the vehicle 2026-09-15, still on the bench:
          ATTITUDE roll +1.57 deg, pitch -15.10 deg
          predicted FRD specific force  (-2.54, -0.26, -9.47) m/s^2
          SCALED_IMU2 accel             (-0.41, -2.56, +9.38)
          ... mapped (y, x, -z)         (-2.56, -0.41, -9.38)
        Fed unmapped to the localization filter, 50 s of that board at rest
        ran to 583 m with 39 of 40 ZUPTs rejected; mapped, 0.036 m and 0 of 40.

        Not hard-coded, proven at runtime. If the firmware ever remaps the
        accel too, applying the swap again would put gravity UP -- 180 deg off,
        and exactly as silent. So both hypotheses are scored against the
        gravity direction the board's own attitude predicts, and the frame is
        latched only after `_ACCEL_FRAME_VOTES` consecutive samples agree
        unambiguously. The two differ by the sign of z, so they are separable
        at any tilt. Until latched this returns None and the filter coasts on
        attitude, depth and flow -- absence, not a guess.
        """
        # Voting never stops. The manager builds one SrotFC per process, so a
        # board reflashed mid-session would otherwise keep a stale latch.
        frame = getattr(self, '_accel_frame', None)
        proven = self._vote_accel_frame(raw, rpy)
        if proven is not None and proven != frame:
            log = getattr(self, '_log', None)
            if log is not None:
                try:
                    (log.info if frame is None else log.warning)(
                        f'[SROT ] SCALED_IMU2 accel frame '
                        f'{"proven" if frame is None else "CHANGED from " + repr(frame) + " to"}: '
                        f'{proven!r} ({self._ACCEL_FRAME_VOTES} consecutive samples '
                        f'against the attitude-predicted gravity)')
                except Exception:   # noqa: BLE001 -- logging must not break telemetry
                    pass
            self._accel_frame = frame = proven
        if frame is None:
            return None
        if frame == 'sensor':
            return (raw[1], raw[0], -raw[2])
        return raw

    _ACCEL_FRAME_VOTES = 25          # 0.5 s at the board's 50 Hz
    _ACCEL_FRAME_AGREE_DEG = 10.0    # level-trim offsets measured at ~1 deg
    _ACCEL_FRAME_REJECT_DEG = 60.0

    def _vote_accel_frame(self, raw, rpy):
        if rpy is None or any(math.isnan(c) for c in rpy):
            return None
        mag = math.sqrt(sum(c * c for c in raw))
        # Only a hull near rest says where gravity is; a hard manoeuvre does not.
        if not (0.8 * 9.80665 < mag < 1.2 * 9.80665):
            self._accel_votes = (None, 0)
            return None
        r, p, _ = rpy
        g = 9.80665
        # Specific force at rest in FRD for aerospace Z-Y-X Euler angles.
        exp = (g * math.sin(p), -g * math.sin(r) * math.cos(p),
               -g * math.cos(r) * math.cos(p))

        def ang(v):
            c = sum(a * b for a, b in zip(v, exp)) / (mag * g)
            return math.degrees(math.acos(max(-1.0, min(1.0, c))))

        e_sensor = ang((raw[1], raw[0], -raw[2]))
        e_vehicle = ang(raw)
        if e_sensor < self._ACCEL_FRAME_AGREE_DEG and e_vehicle > self._ACCEL_FRAME_REJECT_DEG:
            vote = 'sensor'
        elif e_vehicle < self._ACCEL_FRAME_AGREE_DEG and e_sensor > self._ACCEL_FRAME_REJECT_DEG:
            vote = 'vehicle'
        else:
            self._accel_votes = (None, 0)
            return None
        last, n = getattr(self, '_accel_votes', (None, 0))
        n = n + 1 if vote == last else 1
        self._accel_votes = (vote, n)
        return vote if n >= self._ACCEL_FRAME_VOTES else None
