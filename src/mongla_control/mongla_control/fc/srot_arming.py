"""The arming interlocks: firmware behaviour revision, thruster power, the depth loop.

Each is a check `arm()` or the bringup gate runs before the hull may move.

Mixed into `SrotFC`.
"""
from __future__ import annotations
import time
from ._srot_base import mavutil, _POLL_S
from . import srot_protocol as sp
from .srot_moves import (  # noqa: F401
    _DEPTH_ARM_TAIL,
)


class SrotArming:
    """Methods of `SrotFC`; see `srot_fc.py` for the node."""

    # ------------------------------------------------------------------ #
    #  Firmware behaviour revision -- the runtime interlock               #
    # ------------------------------------------------------------------ #
    def read_behaviour_rev(self, timeout: float = 2.0, retries: int = 3):
        """Board's `SROT_FW_BEHAVIOUR_REV`, or None if it never answered.

        Carried in `AUTOPILOT_VERSION.middleware_sw_version` (the board has no
        middleware, so the field was free), requested with MAV_CMD_REQUEST_MESSAGE.

        Why this exists at runtime. `test_firmware_behaviour_rev_is_new_enough`
        checks the same number, but it reads the firmware repo off disk and skips
        when that repo is not checked out beside the workspace -- which is exactly
        the situation on the vehicle. As a vehicle-side guarantee it is worth zero.
        The hazard it is supposed to guard is silent in both directions: pre-rev-2
        firmware COASTS on MOVE_STOP and we no longer carry a host brake, so a
        `stop` or an abort simply does not decelerate ten-plus kilograms of hull, with nothing
        in any log to say why. That has to be caught on the wire, before arming.

        A rev of 0 is NOT "unknown" -- it is what firmware older than 2026-08-01
        reports, because that build never populated the field. Fail closed on it.
        """
        if self._behaviour_rev is not None:
            return self._behaviour_rev
        for _ in range(max(1, retries)):
            self._command_long(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                               p1=float(sp.MSG_ID_AUTOPILOT_VERSION))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                msg = self._cache('AUTOPILOT_VERSION')
                if msg is not None:
                    self._behaviour_rev = int(getattr(msg, 'middleware_sw_version', 0))
                    return self._behaviour_rev
                time.sleep(_POLL_S)
        return None

    def check_behaviour_rev(self, for_arm: bool = False):
        """(ok, reason). Fail-closed on a KNOWN-too-old board, and -- when
        ``for_arm`` -- on a board that did not answer at all.

        At connect a silent board only warns: there "board said nothing" is more
        likely a dropped frame than a rev-1 board, and refusing to even talk to
        the vehicle on a comms hiccup is its own hazard.

        At ARM it refuses. That asymmetry was decided when the floor was rev 2
        and the cost of an old board was a `stop` that coasts. The floor is now
        rev 10, because below it the board's yaw sense is INVERTED: every turn
        goes the wrong way and any loop closed on yaw -- the vision verbs --
        diverges, while roll and pitch read correctly so nothing looks wrong.
        `read_behaviour_rev` has already asked three times over six seconds, so
        "no answer" means firmware without REQUEST_MESSAGE (older than the floor)
        or a link too poor to arm on. `allow_fw_behaviour_mismatch:=true` still
        overrides, loudly. Reversed on the cloud audit's finding, 2026-11-07.
        """
        rev = self.read_behaviour_rev()
        need = sp.FW_BEHAVIOUR_REV_REQUIRED
        if rev is None:
            what = (f'could not read SROT_FW_BEHAVIOUR_REV from the board '
                    f'(AUTOPILOT_VERSION unanswered). Host expects >= {need}; below '
                    f'rev 10 the board turns every yaw backwards')
            if not for_arm:
                self._log_warn(f'[SROT ] !! {what}. Connecting anyway; arm will refuse.')
                return True, 'FW_BEHAVIOUR_REV_UNKNOWN'
            if self.allow_fw_behaviour_mismatch:
                self._log_warn(f'[SROT ] !! OVERRIDDEN: {what}.')
                return True, 'FW_BEHAVIOUR_REV_UNKNOWN_OVERRIDDEN'
            self._log_warn(f'[SROT ] !! REFUSING TO ARM: {what}. Set '
                           f'allow_fw_behaviour_mismatch:=true to arm anyway.')
            return False, f'FW_BEHAVIOUR_REV_UNKNOWN: {what}'
        if rev >= need:
            if not self._behaviour_rev_logged:
                self._log_info(f'[SROT ] firmware behaviour rev {rev} (>= {need} required)')
                self._behaviour_rev_logged = True
            return True, f'FW_BEHAVIOUR_REV={rev}'
        detail = (f'board reports SROT_FW_BEHAVIOUR_REV={rev}, host requires >= {need}. '
                  f'Rev < 2 COASTS on MOVE_STOP and this host no longer carries the '
                  f'reverse-leg brake, so stop/abort would not decelerate the hull. '
                  f'Flash rev >= {need}, or set allow_fw_behaviour_mismatch:=true if '
                  f'you accept an un-braked stop.')
        if self.allow_fw_behaviour_mismatch:
            self._log_warn(f'[SROT ] !! OVERRIDDEN: {detail}')
            return True, f'FW_BEHAVIOUR_REV_OVERRIDDEN={rev}'
        self._log_warn(f'[SROT ] !! REFUSING TO ARM: {detail}')
        return False, f'FW_BEHAVIOUR_REV_TOO_OLD: {detail}'

    def check_thruster_power(self):
        """(ok, reason). Refuse to arm when thruster power is KNOWN to be cut.

        The thruster pack is switched by a rotary knob on the SECOND board (an
        AS5600 magnetic encoder driving a MOSFET). Outside its two ON windows the
        propulsion battery is physically disconnected -- the ESCs are unpowered
        and the hull cannot move at all.

        Nothing refuses this today, on either side: the firmware's own `canArm()`
        checks IMU, leak and pack voltage and never looks at the kill state. So
        arming succeeds, every motion verb runs its full profile against dead
        thrusters, the payload fires, and the mission advances to completion on a
        motionless vehicle -- the silent-success shape that has already cost this
        project a round in SURFACE mode and a disarmed `SROT_MOVE`.

        Refuses only on known-engaged, and that asymmetry is deliberate.
        `kill_switch is None` means the second board is not talking (unpowered,
        out of range, `ESPNOW_EN = 0`, or simply not built), which is a perfectly
        ordinary bench configuration. Refusing on unknown would make `arm()`
        unreachable on every vehicle without a second board -- the same trap as
        refusing a torpedo on UNKNOWN thruster health.
        """
        try:
            kill = self.telemetry().kill_switch
        except Exception as exc:                  # a probe fault is not a hull fault
            return True, f'thruster power unreadable ({exc}) -- allowing'
        if kill is True:
            return False, ('thruster power is CUT at the 2nd-board rotary switch -- '
                           'arming would run the whole mission against dead thrusters')
        return True, ''

    def check_depth_loop_settled(self):
        """(ok, reason). Refuse to arm while the depth controller is already saturated.

        Observed on the vehicle, 2026-08-02, disarmed and stationary on a bench:
        `DEPTH_OUT = -1.00` (full scale) with `DEPTH_ERR = -3.1 m`, because the Bar30
        was reporting a phantom depth. Arming into that state is not a subtle risk --
        the mixer's throttle column is **-1 on all four vertical thrusters and 0 on all
        four horizontals** (fw `mixer.cpp`), so a -1.0 heave demand becomes +1.0 on
        every vertical the instant the outputs go live. That is exactly the reported
        blocker: verticals at ~3000 RPM, horizontals idling, nothing commanded.

        In water it is a vehicle that dives or surfaces the moment it arms.

        This reads `DEPTH_CMD`, not `DEPTH_OUT`, and that change is load-bearing.
        It used to read `DEPTH_OUT` (the real controller's last output). From fw rev 8
        that field is SUPPRESSED while the controller is not running -- which is the
        honest-absence fix we asked for, and it made this guard STRUCTURALLY DEAD:
        `arm()` is the only caller, `arm()` only runs while disarmed, and disarmed the
        loop never runs, so `DEPTH_OUT` is now ALWAYS absent here and the old code took
        the `None` -> "not reported" -> PASS branch every single time. A guard that
        cannot fail is worse than no guard, because the preflight still prints a line
        that reads like a check.

        `DEPTH_CMD` is the replacement the firmware team pointed at, and it is a better
        signal for this specific question, not merely an available one:
          * It is `depth::preview(depth, 0.10)` -- computed ON DEMAND against a fixed
            0.10 m target, so it is live while disarmed by construction.
          * Same +/-1.0 clamp, so the 2026-08-02 phantom-baro case (depth -3..-6.7 m,
            `3.0 * -3.1` = -9.3) still pins at -1.00 exactly as `DEPTH_OUT` did.
          * PROPORTIONAL-ONLY, no integrator -- it reflects the CURRENT baro sample
            rather than accumulated windup. For "is the barometer lying right now",
            which is what this guard actually asks, that is the property we want.

        The threshold is expressed in METRES OF DEPTH ERROR and converted through
        `DEPTH_P`, because `DEPTH_CMD = clamp(DEPTH_P * (depth - 0.10))` -- a hardcoded
        0.90 silently means a different physical depth the moment anyone retunes the
        gain. At the default `DEPTH_P = 3.0`, `|DEPTH_CMD| >= 0.90` <=> 0.30 m of error;
        a healthy surface reading (~0.03 m) gives ~-0.22 and is comfortably clear.

        It fails closed and quotes the numbers, because a refusal that does not say what
        it saw cannot be told apart from "not configured" -- the lesson the firmware team
        paid for four times over on the mag reference.

        Bypass with the ROS param `allow_saturated_depth_arm` (deliberately its own
        flag, not the firmware-revision one -- they are different risks).
        """
        cmd = self._named_value('DEPTH_CMD')
        if cmd is None:
            # DEPTH_CMD is gated on `depth_ok`, so absence means the board has declared
            # the barometer unhealthy/stale -- NOT that the check is unavailable. The
            # board then refuses DEPTH_HOLD/AUTO itself, and since SROT_MOVE enters AUTO
            # every move verb is denied, so this fails closed downstream without us
            # blocking the arm. Say which it is; do not report it as "settled".
            return True, 'depth preview absent (barometer unhealthy -- AUTO will refuse)'
        depth_p = self.depth_p or sp.DEPTH_P_DEFAULT
        # Saturation first. While clamped the true depth is beyond the clamp, so the
        # recovery below would report a harmless-looking value for the very case this
        # guard exists to catch -- the 2026-08-02 phantom baro pinned DEPTH_CMD at
        # -1.00, which back-converts to a benign-looking -0.23 m.
        if abs(cmd) >= sp.DEPTH_CMD_SATURATED:
            msg = (f'BAROMETER IMPLAUSIBLE: DEPTH_CMD={cmd:+.2f} is SATURATED, so the '
                   f'board is seeing a depth it cannot even express. '
                   + _DEPTH_ARM_TAIL)
        else:
            # Unsaturated: recover the board's own depth exactly. The preview's 0.10 m
            # target is baked into DEPTH_CMD, so subtracting it is what turns "error
            # against a target" into "how far the barometer is from zero" -- the thing
            # actually being judged.
            depth_m = cmd / depth_p + sp.DEPTH_PREVIEW_TARGET_M
            if abs(depth_m) < sp.DEPTH_ERR_ARM_LIMIT_M:
                return True, (f'barometer sane ({depth_m:+.2f} m at the surface, '
                              f'DEPTH_CMD={cmd:+.2f})')
            msg = (f'BAROMETER IMPLAUSIBLE: the board reads {depth_m:+.2f} m of depth '
                   f'at the surface (limit {sp.DEPTH_ERR_ARM_LIMIT_M:.2f} m, '
                   f'DEPTH_CMD={cmd:+.2f}, DEPTH_P={depth_p:g}). ' + _DEPTH_ARM_TAIL)
        if self.allow_saturated_depth_arm:
            self._log_warn(f'[SROT ] {msg} -- OVERRIDDEN, arming anyway')
            return True, f'OVERRIDDEN: {msg}'
        return False, msg
