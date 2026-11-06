"""What the board says about itself: its announcements, reboots, thrusters and leak.

The board reports most of its own state as STATUSTEXT sentences rather than as
data -- the boot burst, the per-thruster presence lines, the reboot -- plus
the extended SYS_STATUS bits pymavlink cannot decode. These methods turn
those into answers, and keep "not seen" apart from "seen, nothing wrong".

Mixed into `SrotFC`; it uses the link that class owns (`self._cache`,
`self._command_long`, `self._log`).
"""
from __future__ import annotations

import re
import struct
import time

from ._srot_base import _POLL_S, mavutil
from . import srot_protocol as sp

# SYS_STATUS wire layout, from the firmware's own vendored header
# (lib/mavlink/common/mavlink_msg_sys_status.h): MIN_LEN 31 is the base message,
# LEN 43 includes the three uint32 extended-health fields at 31 / 35 / 39.
# ESC_STATUS (291): removed from upstream `common`, so pymavlink has no entry
# for it and surfaces the frame as UNKNOWN_291 -- unvalidated. crc_extra from
# the message definition; LEN is the full payload before v2 zero-truncation.
_ESC_STATUS_CRC_EXTRA = 10
_ESC_STATUS_LEN = 57


def _decode_esc_status(buf):
    """(index, [rpm x4]) from a raw ESC_STATUS frame, or None if it fails a check.

    Layout from the firmware's own vendored definition: the payload is
    `<Q4i4f4fB` = time_usec, rpm[4] (int32, SIGNED), voltage[4], current[4],
    index. MAVLink v2 truncates trailing zero bytes, so the payload is padded
    back to full length before unpacking -- the same trap as SYS_STATUS's
    extended health, and here the trailing field is `index` itself.
    """
    try:
        if len(buf) < 12 or buf[0] != 0xFD:
            return None                            # not a v2 frame
        n = buf[1]
        payload = buf[10:10 + n]
        if len(payload) != n:
            return None
        # pymavlink hands over an UNKNOWN message WITHOUT checking the checksum,
        # so a corrupt frame would decode into plausible-looking RPM. Check it.
        crc = mavutil.x25crc(buf[1:10 + n])
        crc.accumulate_str(chr(_ESC_STATUS_CRC_EXTRA))
        want = struct.unpack_from('<H', buf, 10 + n)[0]
        if crc.crc != want:
            return None
        payload = payload.ljust(_ESC_STATUS_LEN, b'\x00')
        fields = struct.unpack('<Q4i4f4fB', payload)
        return int(fields[-1]), [int(v) for v in fields[1:5]]
    except Exception:                              # noqa: BLE001
        return None
# The boot burst (task_mavlink.cpp) opens with the reset reason, so this line
# is how we know we saw a boot at all.
_BOOT_MARK = 'RST: '
# The lines in it that change behaviour. Matched by substring, as sent.
_BOOT_WARNINGS = ('Params reset to build defaults', 'NVS reformatted',
                  'CAL DEFAULTS', 'MAG cal MISSING', 'SAVE FAILED',
                  'set but NOT saved')
_SYS_STATUS_BASE_LEN = 31
_SYS_STATUS_EXT_END  = 43


class SrotHealth:
    """The health part of `SrotFC` (see the module docstring)."""

    # ------------------------------------------------------------------ #
    #  Telemetry                                                          #
    # ------------------------------------------------------------------ #
    def note_statustext(self, msg) -> None:
        """Hook for the manager's MAVLink reader thread, like `note_named_value`.

        The reader sees EVERY STATUSTEXT; this object only ever sees whichever
        one last landed in pymavlink's single slot. That matters more here than
        it looks, because the board sends its announcements as a BURST at boot
        (task_mavlink.cpp): the reset reason, a defaults reset, an NVS reformat,
        four migration notices, the calibration state and the config banner --
        13 messages back to back, of which a poller sees one.

        Two of those change how the vehicle behaves and are silent otherwise:
        "Params reset to build defaults" puts JS_GAIN_DEFAULT back to 0.5 and
        LEAK_EN back to 0, and "NVS reformatted" additionally loses the sensor
        calibration.

        Also parses the per-thruster presence lines, which are the only place
        `esc_present` reaches the wire at all -- the board holds the bitmask in
        state and publishes it exclusively as English.
        """
        text = getattr(msg, 'text', '')
        text = text.decode(errors='replace') if isinstance(text, bytes) else str(text)
        text = text.strip('\x00').strip()
        if not text:
            return
        sev = int(getattr(msg, 'severity', 6))
        self._statustext_log.append((time.time(), sev, text))
        if text.startswith(_BOOT_MARK):
            self._boot_lines = []            # a new boot: forget the last one's
        if self._boot_lines is not None and any(k in text for k in _BOOT_WARNINGS):
            self._boot_lines.append((sev, text))
        self._note_esc_line(text)

    def _note_esc_line(self, text: str) -> None:
        """Per-thruster presence, from the board's own announcements.

        The board decides presence from bidirectional-DShot telemetry and holds
        it in `thrusters.esc_present`, but that bitmask NEVER reaches the wire
        as data -- only as these two sentences, once each:

            "Thrusters wired: all 8"            (INFO,  at the first arm)
            "Thrusters wired: 1,2,4 (absent: 3)"(WARN,  at the first arm)
            "Thruster 3 LOST telemetry"         (ERROR, on a regression only)

        The summary is sent ONCE per boot and the LOST lines are edge-triggered,
        so this has to be captured as it goes past. There is no way to ask again.
        """
        m = re.match(r'Thrusters wired: all (\d+)', text)
        if m:
            self._esc_present = set(range(1, int(m.group(1)) + 1))
            return
        m = re.match(r'Thrusters wired: ([\d,]*)', text)
        if m:
            wired = m.group(1)
            self._esc_present = {int(n) for n in wired.split(',') if n.strip().isdigit()}
            return
        m = re.match(r'Thruster (\d+) LOST telemetry', text)
        if m:
            self._esc_lost.add(int(m.group(1)))

    def statustext_log(self, since: float = 0.0):
        """(timestamp, severity, text) newest last, optionally since a time."""
        return [r for r in self._statustext_log if r[0] >= since]

    def boot_warnings(self):
        """The startup announcements that change how the vehicle behaves, or None.

        Returned as (severity, text) so a caller can grade them. These are FAILs,
        not notices: a defaults reset silently reverts the pilot gain to half
        authority and disables the leak failsafe, and the operator's only clue is
        one line in a burst of thirteen.

        None means the boot burst was NEVER SEEN -- the board booted before this
        process was listening -- so nothing is known. It is not the same answer
        as [] ("seen, nothing wrong"), and a caller must not render it as one.
        On USB the port open reboots the board, so the burst normally arrives;
        `request_boot_announcements` asks for it again otherwise.
        """
        return None if self._boot_lines is None else list(self._boot_lines)

    def request_boot_announcements(self, timeout: float = 1.5) -> bool:
        """Ask the board to repeat its boot burst; True once it has been seen.

        `REQUEST_MESSAGE(STATUSTEXT)`. Firmware that does not replay the burst
        ACCEPTS the request and sends nothing (it accepts every REQUEST_MESSAGE),
        so the answer is read off the burst arriving, never off the ACK.
        """
        if self._boot_lines is not None:
            return True
        self._command_long(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                           p1=float(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._boot_lines is not None:
                return True
            time.sleep(_POLL_S)
        return False

    def esc_presence(self):
        """(present:set|None, lost:set) -- per-thruster telemetry presence.

        `present is None` means the board has not announced yet, which is NOT
        the same as "all healthy" and must never be graded as one. The summary
        is emitted at the FIRST ARM, so a disarmed bench session legitimately
        has nothing to report.
        """
        if self._esc_present is None:
            return None, set(self._esc_lost)
        return set(self._esc_present) - self._esc_lost, set(self._esc_lost)

    def _statustext(self) -> str:
        msg = self._cache('STATUSTEXT')
        if msg is None:
            return ''
        text = getattr(msg, 'text', '')
        return text.decode() if isinstance(text, bytes) else str(text)

    # Board uptime can wobble by a sample without meaning anything -- messages
    # are timestamped when built and can be reordered slightly by the tx queue.
    # A REBOOT drops uptime to near zero, so only a large backwards step counts.
    _REBOOT_DROP_MS = 3000

    def check_for_reboot(self) -> bool:
        """True exactly once per unplanned flight-controller restart.

        Opening the serial port reboots this board (see `fc/port_guard.py`), so an
        FC restart mid-session is a REAL and reachable state, not a theoretical
        one: any second process that touches the device causes it. After a reboot
        the board is DISARMED, back in its boot mode, with every setpoint cleared
        and its stream rates reset to the compiled defaults -- so a mission that
        keeps issuing verbs is commanding a vehicle that is no longer the one it
        configured.

        Detected on `ATTITUDE.time_boot_ms` going sharply backwards. That field is
        the board's own clock, so this needs no host timing and survives a stalled
        reader thread.
        """
        att = self._cache('ATTITUDE')
        if att is None:
            return False
        now_ms = int(getattr(att, 'time_boot_ms', 0))
        prev = self._peak_boot_ms
        if prev is None:
            self._peak_boot_ms = now_ms
            return False
        if now_ms < prev - self._REBOOT_DROP_MS:
            self._reboots += 1
            self._peak_boot_ms = now_ms
            if self._log is not None:
                self._log.error(
                    f'[SROT ] FLIGHT CONTROLLER REBOOTED -- uptime fell '
                    f'{prev} -> {now_ms} ms (restart #{self._reboots}). The board is '
                    f'now DISARMED with default stream rates; anything this session '
                    f'configured is gone. Most likely cause: a second process opened '
                    f'the serial port.')
            return True
        self._peak_boot_ms = max(prev, now_ms)
        return False

    def esc_status_rpm(self):
        """SIGNED per-thruster RPM from ESC_STATUS (291), or None.

        Both repos record that pymavlink "silently discards" msgid 291 because
        upstream removed it from `common`. It does not. `MAVLink.decode()` ends
        with:

            if mapkey not in mavlink_map:
                return MAVLink_unknown(msgId, msgbuf)

        -- a message object carrying the WHOLE FRAME, surfaced as UNKNOWN_291.
        Verified against two recorded BENCH sessions: 958 of 958 frames decode,
        CRC-valid, across both index blocks.

        ⛔ THAT IS DECODABILITY, NOT DATA. Every one of those 958 frames carries
        rpm EXACTLY 0 in all eight slots, because no ESC was attached -- the
        board fills the whole block regardless. Read as "the telemetry is real"
        this line would justify a health gate that reports OK on a hull with no
        thrusters. No non-zero RPM has yet crossed this wire on this vehicle.

        Worth the trouble because ESC_STATUS.rpm is `int32` and SIGNED, while
        ESC_TELEMETRY_*.rpm is `uint16` magnitude only -- the firmware says so
        itself ("a reversing thruster reports magnitude; the sign lives in the
        commanded direction, which the companion already knows"). We do know the
        commanded direction, but only for a thruster we commanded: it cannot
        tell a prop spun backwards by the wash from one driven backwards, and it
        says nothing at all when the vehicle is idle. Signed RPM can.

        ⚠ WE MUST CHECK THE CRC OURSELVES. That early return happens BEFORE
        pymavlink validates the checksum, so an unknown message is handed over
        unvalidated -- a corrupted frame would otherwise read as plausible RPM.
        `crc_extra` for ESC_STATUS is 10.

        Returns a tuple of 8 signed ints, or None if nothing valid has arrived.
        Absence, never zeros: a stopped thruster and a missing message are not
        the same fact.
        """
        out = [None] * 8
        got = False
        for key in ('UNKNOWN_291', 'ESC_STATUS'):
            msg = self._cache(key)
            if msg is None:
                continue
            try:
                buf = bytes(msg.get_msgbuf())
            except Exception:                      # noqa: BLE001
                continue
            block = _decode_esc_status(buf)
            if block is None:
                continue
            index, rpm = block
            for i, v in enumerate(rpm):
                if 0 <= index + i < 8:
                    out[index + i] = v
                    got = True
        return tuple(out) if got else None

    def thruster_health(self):
        """(ok, reason) -- is every thruster reporting and turning as commanded?

        `ok is None` means UNKNOWN, and that is the important return value. The
        board announces presence only at the FIRST ARM and only as English, and
        ESC telemetry exists only on ESCs flashed with Bluejay -- stock BLHeli_S
        has no bidirectional DShot at all, so an un-flashed ESC is silent while
        the motor spins perfectly. A gate that answered "healthy" for a vehicle
        nothing has reported on would be worse than no gate, because it reads as
        a check that passed.

        Callers must treat None as "do not know", not as a refusal and not as an
        approval. `mongla_ws` uses it to refuse a PAYLOAD run (where a dead
        thruster means a missed shot and a wasted round) while allowing a
        transit, which is the asymmetry that matters.
        """
        present, lost = self.esc_presence()
        if lost:
            return False, ('thruster(s) '
                           + ', '.join(str(n) for n in sorted(lost))
                           + ' LOST telemetry mid-session')
        if present is None:
            return None, ('thruster telemetry UNKNOWN -- the board announces '
                          'presence at the first arm only, and an ESC without '
                          'Bluejay never reports at all')
        missing = sorted(set(range(1, 9)) - present)
        if missing:
            return False, ('thruster(s) ' + ', '.join(str(n) for n in missing)
                           + ' report no telemetry (unflashed ESC, or unwired)')
        return True, f'all {len(present)} thrusters reporting'

    def thruster_stalled(self, commanded=None, min_rpm: int = 50):
        """Thrusters commanded to turn that are not turning, from SIGNED RPM.

        `commanded` maps 1-based thruster -> commanded sign (-1/0/+1). Without
        it this only reports zero-RPM thrusters, which on an idle hull is every
        thruster -- so it returns None rather than a list nobody can act on.

        The signed reading is what makes this worth having: a thruster turning
        the WRONG WAY is a wiring or direction-calibration fault that unsigned
        magnitude cannot see, and it is exactly the fault that made every axis
        respond backwards on this hull in August.
        """
        rpm = self.esc_status_rpm()
        if rpm is None or commanded is None:
            return None
        bad = []
        for n, want in commanded.items():
            if not want:
                continue
            v = rpm[n - 1] if 1 <= n <= 8 else None
            if v is None:
                continue
            if abs(v) < min_rpm:
                bad.append((n, v, 'not turning'))
            elif (v > 0) != (want > 0):
                bad.append((n, v, 'turning the WRONG WAY'))
        return bad

    def _sys_status_ext_bits(self):
        """`(present, enabled, health)` from SYS_STATUS's extended bitfields, or None.

        THE PADDING IS LOAD-BEARING and this is the only copy of it. MAVLink v2
        truncates trailing zero bytes, so the payload is not its declared 43:
        measured off the live board it arrives at 40, with `health_extended` cut
        to a SINGLE byte. Unpacking four bytes at offset 39 without padding reads
        past the end. A truncated field is zero by definition, which is what the
        `ljust` restores.

        Extracted so `sys_status_leak()` and `leak_state()` cannot drift apart --
        a second hand-written copy of an offset table is how the ESC_STATUS(291)
        trap happened.
        """
        msg = self._cache('SYS_STATUS')
        if msg is None:
            return None
        try:
            buf = bytes(msg.get_msgbuf())
            payload = buf[10:10 + buf[1]]          # v2 header is 10 bytes
            if len(payload) <= _SYS_STATUS_BASE_LEN:
                return None                        # no extension bytes at all
            payload = payload.ljust(_SYS_STATUS_EXT_END, b'\x00')
            return struct.unpack_from('<III', payload, _SYS_STATUS_BASE_LEN)
        except Exception:                          # noqa: BLE001
            return None

    def sys_status_leak(self):
        """Leak from the SYS_STATUS extended health bits, or None if unreadable.

        We ASKED for this (TASKS_FROM_DUBURI_WS.md §3): LEAK rides the multiplexed
        NAMED_VALUE_FLOAT, pymavlink keeps one message per msgid, so observing a
        flooding hull was a lottery on arrival order. The firmware delivered it in
        rev 3 as `MAV_SYS_STATUS_SENSOR_LEAK` on the extended health bitfield --
        a LATCHED state that a temperature reading cannot overwrite -- and we
        never adopted it, because pymavlink 2.4.49's SYS_STATUS schema has 13
        fields and no extensions (verified: it does not parse them).

        It parses them away; it does not throw them away. The bytes are in the
        payload, at the offsets the firmware's own accessors use:

            31  onboard_control_sensors_present_extended
            35  onboard_control_sensors_enabled_extended
            39  onboard_control_sensors_health_extended

        THE PADDING IS LOAD-BEARING. MAVLink v2 truncates trailing zero bytes, so
        the payload is not its declared 43: measured off the live board it arrives
        at 40, with `health_extended` cut to a SINGLE byte. Unpacking four bytes at
        39 without padding reads past the end. A truncated field is zero by
        definition, which is what the pad restores.

        Returns None when the sensor is not present or not enabled -- absence, not
        False. `LEAK_EN = 0` disables the board's own leak failsafe AND its pre-arm
        refusal, and reporting "no leak" for a vehicle that is not looking is the
        exact confusion this whole mechanism exists to remove.
        """
        bits = self._sys_status_ext_bits()
        if bits is None:
            return None
        present, enabled, health = bits
        if not (present & sp.SYS_STATUS_SENSOR_LEAK):
            return None                            # board is not reporting a leak sensor
        if not (enabled & sp.SYS_STATUS_SENSOR_LEAK):
            return None                            # LEAK_EN = 0: nothing is watching
        # Health bit SET means healthy, i.e. dry. Clear means leak.
        return not bool(health & sp.SYS_STATUS_SENSOR_LEAK)

    def leak_state(self):
        """`(enabled, leaking)` -- the two leak facts, kept APART.

        `sys_status_leak()` above answers one question ("are we taking on water?")
        and correctly returns None for every reason it cannot: no SYS_STATUS, no
        sensor, or `LEAK_EN = 0`. That is right for its callers and useless for a
        health verdict, because it collapses "dry" and "nothing is watching" into
        the same absence -- which is the exact confusion its own docstring says
        the mechanism exists to remove.

        `health_reporters.leak_sensor()` was written to render that distinction
        (`LEAK_EN=0 -> FAILED, NOTHING IS WATCHING`) and needs the two signals
        separately. It could not be wired to anything until this existed, which
        is why it sat unregistered.

        Returns:
            (None, None)   -- no SYS_STATUS, or no leak sensor on the board
            (False, None)  -- sensor present but LEAK_EN = 0: nothing is watching
            (True,  bool)  -- enabled; True = leaking, False = dry
        """
        bits = self._sys_status_ext_bits()
        if bits is None:
            return (None, None)
        present, enabled, health = bits
        if not (present & sp.SYS_STATUS_SENSOR_LEAK):
            return (None, None)                    # board reports no leak sensor
        if not (enabled & sp.SYS_STATUS_SENSOR_LEAK):
            return (False, None)                   # LEAK_EN = 0 -- the trap
        return (True, not bool(health & sp.SYS_STATUS_SENSOR_LEAK))
