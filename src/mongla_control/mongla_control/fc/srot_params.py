"""Board parameters and the preflight checks that read them.

PARAM read/set with the board's echo as the acknowledgement, the stream rates,
and the checks the manager runs before the first arm (yaw authority, yaw
reference, cruise limit, depth gain).

Mixed into `SrotFC`.
"""
from __future__ import annotations

import math
import time

from ._srot_base import _POLL_S, mavutil
from . import srot_protocol as sp


def _param_id(pv) -> str:
    """PARAM_VALUE.param_id as a clean str (mavlink sends a padded bytes field)."""
    pid = getattr(pv, 'param_id', '')
    pid = pid.decode() if isinstance(pid, bytes) else str(pid)
    return pid.strip('\x00')


class SrotParams:
    """The params part of `SrotFC` (see the module docstring)."""

    def set_message_rate(self, message_id, hz):
        """MAV_CMD_SET_MESSAGE_INTERVAL (511). Fire-and-forget.

        This was a no-op -- "SROT telemetry rates are fixed on-board" -- which made
        the board's 10 Hz ATTITUDE the hard ceiling on every host loop, un-raisable
        for a control loop and un-lowerable for a slow link. Firmware behaviour rev 2
        implements 511 (and 510), so it is a real call now.

        Board-side rules worth knowing: any interval is clamped to a 20 ms floor
        (50 Hz) so a companion cannot starve the PARAM_VALUE / COMMAND_ACK traffic
        missions depend on, and a request to DISABLE HEARTBEAT is refused with
        DENIED rather than accepted-and-ignored. `hz <= 0` restores the board's
        compiled default for that stream.

        Verified on hardware 2026-09-03 (fw rev 14), and one result is a trap:

            baseline            10.17 Hz  (100.0 ms)   the compiled default
            ask 50 Hz           50.17 Hz  ( 20.0 ms)   ACK ACCEPTED
            ask 100 Hz          50.00 Hz  ( 20.0 ms)   ACK *ACCEPTED*, clamped
            restore (hz<=0)     10.25 Hz  (100.0 ms)   ACK ACCEPTED

        A below-floor request is accepted, not denied. The ACK says nothing about
        the rate you actually got, so a host that asks for 100 Hz and believes the
        ACCEPTED will size its loop for 10 ms of feedback and get 20. The only way
        to know the delivered rate is to measure the arrivals. (DENIED is reserved
        for an unknown msgid and for disabling HEARTBEAT.)

        This is also the first time the 50 Hz in `SROT_MESSAGE_RATES` has been
        confirmed to take effect: the board had only ever been observed at its
        10 Hz default, because nothing had run the manager against it.
        """
        if hz is None:
            return None
        interval_us = 0.0 if hz <= 0 else float(1e6 / float(hz))
        self._command_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                           p1=float(message_id), p2=interval_us)
        return None

    def calibrate_barometer(self, timeout: float = 6.0):
        """Re-zero the depth reference: PREFLIGHT_CALIBRATION (241) p3=1 (baro)."""
        self._clear_ack()
        self._command_long(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                            p1=0.0, p2=0.0, p3=1.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ack = self._cache('COMMAND_ACK')
            if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                return ack.result == sp.ACK_ACCEPTED, 'baro'
            time.sleep(_POLL_S)
        return False, 'NO_ACK'

    # ── parameters: one reply per NAME, fed by the reader (issue #17) ───────
    def note_param_value(self, msg) -> None:
        """Hook for the manager's MAVLink reader: EVERY PARAM_VALUE, by name.

        One slot, several threads. get_param / set_param / set_default_gain
        each POPPED pymavlink's single PARAM_VALUE slot and polled it -- so the
        telemetry timer's `set_default_gain` could erase an action thread's
        `channel_role` reply (or overwrite it before it was read), and the loser
        timed out reading "unreadable" -- which `fire()` turns into a refusal.
        The reader now files every reply under its own name with a monotonic
        receipt stamp, and each caller waits for ITS name received after ITS
        request. Nothing pops, so nothing can be erased.
        """
        self._param_table[_param_id(msg)] = (float(msg.param_value),
                                             time.monotonic())
        self._param_fed = True

    def _send_param_and_wait(self, name: str, timeout: float, value=None):
        """Read (`value is None`) or set `name`; return the echoed value or None.

        Both sends sit LEXICALLY inside `_tx_lock`, which
        `test_srot_tx_lock.py` checks structurally.
        """
        name = str(name)
        wire = name.encode()
        if not getattr(self, '_param_fed', False):
            # No reader feeding the table (a bare tool, a unit test): the old
            # single-slot path, which is safe with one caller.
            self.master.messages.pop('PARAM_VALUE', None)
        t_req = time.monotonic()
        with self._tx_lock:
            if value is None:
                self.master.mav.param_request_read_send(
                    sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, wire, -1)
            else:
                self.master.mav.param_set_send(
                    sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, wire, float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = t_req + float(timeout)
        while time.monotonic() < deadline:
            if getattr(self, '_param_fed', False):
                hit = self._param_table.get(name)
                if hit is not None and hit[1] >= t_req:
                    return hit[0]
            else:
                pv = self._cache('PARAM_VALUE')
                if pv is not None and _param_id(pv) == name:
                    return float(pv.param_value)
            time.sleep(_POLL_S)
        return None

    def get_param(self, name, timeout: float = 2.0):
        name = name.decode() if isinstance(name, bytes) else name
        return self._send_param_and_wait(name, timeout)

    def set_param(self, name, value, timeout: float = 3.0):
        name = name.decode() if isinstance(name, bytes) else name
        got = self._send_param_and_wait(name, timeout, value=float(value))
        return got is not None and abs(got - float(value)) < 1e-3

    # ------------------------------------------------------------------ #
    #  Pilot gain -- MANUAL_CONTROL is halved until GAIN=1.0              #
    # ------------------------------------------------------------------ #
    def read_gain(self):
        """Live pilot gain (NAMED_VALUE_FLOAT 'GAIN', 0.1..1.0), or None if unseen."""
        return self._named_value('GAIN')

    # ------------------------------------------------------------------ #
    #  Thrust-allocator saturation (srot-control-board#20)                 #
    # ------------------------------------------------------------------ #
    #  1.0 = the group is delivering the demand. Below 1.0 the mixer scaled
    #  that group down and the axis is NOT achieving what the controller asked
    #  for. `mixer::mix()` computes this per group and drops it, so until #20
    #  merges these read None -- which is UNKNOWN, not OK. A saturating
    #  allocator and a silent one are indistinguishable from here, and that is
    #  exactly the state #20 exists to end.
    ALLOCATOR_SAT_NAMES = ('MIX_SAT_H', 'MIX_SAT_V')

    def allocator_saturation(self):
        """(horizontal, vertical) delivered fraction, each 0..1 or None."""
        return tuple(self._named_value(n) for n in self.ALLOCATOR_SAT_NAMES)

    def check_move_cruise_max(self, timeout: float = 3.0):
        """Compare the BOARD's speed cap with the host's own clamp (B34).

        `sanitize_speed` clamps every host-issued move to `sp.MOVE_CRUISE_MAX`, a
        hard-coded copy of the firmware DEFAULT. The board's is a runtime param.
        They agree out of the box, and the mismatch is silent in ONE direction:
        RAISE the board's cap for a faster transit and every autonomous move is
        still clamped by us, while a teleop move -- which does not pass through
        `sanitize_speed` -- is not. The operator changes a speed limit, watches
        nothing change, and has no log line to explain it.

        Returns (ok, message). Never raises: a param that will not read is a
        degraded report, not a failed bring-up.
        """
        board = self.get_param('MOVE_CRUISE_MAX', timeout=timeout)
        if board is None:
            return True, ('MOVE_CRUISE_MAX unread -- host clamp '
                          f'{sp.MOVE_CRUISE_MAX:.2f} is the binding one')
        board = float(board)
        if abs(board - sp.MOVE_CRUISE_MAX) < 1e-3:
            return True, f'MOVE_CRUISE_MAX {board:.2f} (host and board agree)'
        if board > sp.MOVE_CRUISE_MAX:
            return False, (
                f'MOVE_CRUISE_MAX: board {board:.2f} > host clamp '
                f'{sp.MOVE_CRUISE_MAX:.2f} -- autonomous moves stay capped at '
                f'{sp.MOVE_CRUISE_MAX:.2f}. Raising the board param does NOT make '
                f'them faster; edit srot_protocol.MOVE_CRUISE_MAX too.')
        return True, (f'MOVE_CRUISE_MAX: board {board:.2f} < host clamp '
                      f'{sp.MOVE_CRUISE_MAX:.2f} -- the board is the binding one')

    def check_yaw_authority(self, timeout: float = 3.0):
        """Can a visual-servo yaw correction produce thrust at all? (ok, message)

        The coupling this guards, measured on `tools/bench/control_bench` against
        this board's own parameters. Two independent thresholds stand between a
        host yaw demand and a turning propeller:

          * `attitude_control.cpp:87` -- `fabsf(yaw_stick) > 0.02f`, compared
            AFTER PILOT_EXPO. Below it the board holds heading instead of taking
            a rate command.
          * `mixer.cpp:102` -- `t < 0.005f` returns neutral outright.

        The first is only USEFUL if crossing it also crosses the second, and
        whether it does depends on two parameters an operator can change:

            0.02 * PILOT_YAW_RATE(rad/s) * ATC_RAT_YAW_P  >  0.005

        This board reads 160 deg/s and 0.18, giving 0.0101 -- comfortable. But
        `config.h` DEFAULTS to 45 deg/s, where the product is 0.0028 and a band
        opens in which the stick is accepted as a rate command and the mixer
        still emits nothing. A demand that is accepted and produces no thrust is
        the failure mode this codebase exists to refuse loudly. Break-even is
        around **80 deg/s** at the shipped rate gain.

        A params reset restores the 45 deg/s default silently, which is
        exactly how this would come back.

        Best-effort, like `check_move_cruise_max`: a parameter that will not read
        degrades the report rather than failing bring-up, because refusing to fly
        on an unread parameter is its own failure mode.
        """
        rate = self.get_param('PILOT_YAW_RATE', timeout=timeout)
        gain = self.get_param('ATC_RAT_YAW_P', timeout=timeout)
        if rate is None or gain is None:
            return True, ('yaw authority UNKNOWN -- PILOT_YAW_RATE / '
                          'ATC_RAT_YAW_P did not read')
        rate, gain = float(rate), float(gain)
        torque = sp.STABILIZE_YAW_STICK_GATE * math.radians(rate) * gain
        if torque > sp.MIXER_CENTRE_EPS:
            return True, (f'yaw authority OK: the stick gate produces torque '
                          f'{torque:.4f} > the mixer centre gap '
                          f'{sp.MIXER_CENTRE_EPS} (PILOT_YAW_RATE {rate:.0f}, '
                          f'ATC_RAT_YAW_P {gain:.3f})')
        return False, (
            f'yaw DEAD BAND: PILOT_YAW_RATE {rate:.0f} deg/s with '
            f'ATC_RAT_YAW_P {gain:.3f} makes the stick gate produce torque '
            f'{torque:.4f}, under the mixer centre gap {sp.MIXER_CENTRE_EPS}. '
            f'Yaw demands above the gate will be ACCEPTED and produce NO '
            f'thrust. Raise PILOT_YAW_RATE above ~80 deg/s (this board shipped '
            f'at 160; a params reset restores the 45 deg/s default).')

    def read_depth_p(self, timeout: float = 3.0):
        """Cache the board's DEPTH_P for `check_depth_loop_settled`. None if unread.

        Best-effort by design: the guard falls back to the firmware default and names
        the gain it used, so a missed read degrades the message rather than the check.
        """
        val = self.get_param('DEPTH_P', timeout=timeout)
        if val is not None:
            self.depth_p = float(val)
        return self.depth_p

    def check_yaw_reference(self):
        """(is_absolute, reason) from YAW_REF (fw rev 9).

        Only `LOCKED` means `ATTITUDE.yaw` is a magnetic heading. Anything else and it
        is relative to wherever the BNO booted -- so an ABSOLUTE `turn` (MOVE_TURN p4=1)
        is aiming at a number that means nothing, and silently: the move completes
        normally, on the wrong heading.

        Do NOT substitute MAGACC. The board's alignment is protected by the |B| band
        and a sample-agreement test, neither of which depends on the sensor's opinion of
        itself, and with a stored calibration the accuracy requirement drops to 0 -- so
        accuracy stops correlating with the outcome entirely. We read MAGACC 2 on a hull
        whose lock state we could not determine at all, which is what prompted the
        firmware to publish this (Round 8 §8.8).

        Absent on fw < 9. Reported as unknown rather than assumed either way.
        """
        raw = self._named_value('YAW_REF')
        if raw is None:
            return False, ('yaw reference UNKNOWN (no YAW_REF -- fw < 9). Absolute '
                           '`turn` may be aiming at a boot-relative heading; prefer '
                           'relative turns')
        state = int(raw)
        if state == sp.YAW_REF_LOCKED:
            return True, 'yaw reference LOCKED -- heading is absolute, `turn` is safe'
        return False, (f'yaw reference NOT LOCKED: {state} = '
                       f'{sp.YAW_REF_NAMES.get(state, "unrecognised")}. ATTITUDE.yaw is '
                       f'relative to boot, so an ABSOLUTE `turn` will aim at a '
                       f'meaningless heading -- prefer relative turns until this reads '
                       f'{sp.YAW_REF_LOCKED}')

    def set_default_gain(self, value: float = sp.GAIN_FOR_AUTONOMY,
                         timeout: float = 3.0) -> bool:
        """Persist JS_GAIN_DEFAULT so programmatic MANUAL_CONTROL is full-scale.

        GAIN is runtime-only and boots at JS_GAIN_DEFAULT (0.5), so setting the
        param is the durable way to make autonomous manual() commands full
        authority. Returns True on a PARAM_VALUE echo confirming the value.
        """
        return self.set_param('JS_GAIN_DEFAULT', value, timeout=timeout)
