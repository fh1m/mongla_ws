"""What the host tells the board it sees: a vision target, a speed, a position.

The vision uplinks (LANDING_TARGET, VISION_SPEED_ESTIMATE,
VISION_POSITION_ESTIMATE). Each is off unless its launch switch is on
(`velocity_uplink`, `position_uplink`).

Mixed into `SrotFC`.
"""
from __future__ import annotations

import math
import time
from typing import Optional

from . import srot_protocol as sp


class SrotUplink:
    """The uplink part of `SrotFC` (see the module docstring)."""

    # LANDING_TARGET (149) -- the vision uplink `VISION_API.md` specifies.
    #
    # The board does not consume this yet. Verified in the firmware source:
    # `mav_commands::handle()` has no MAVLINK_MSG_ID_LANDING_TARGET case, so the
    # message is parsed and silently dropped, and `MAV_CMD_SROT_VISION` (31001)
    # returns MAV_RESULT_UNSUPPORTED. Their spec says so itself: "Status:
    # specification, not yet implemented."
    #
    # It is built now anyway, and default-off, because the firmware side's named
    # blocker was a measured camera FOV -- which we now have. Shipping the
    # producer lets them implement against a stream they can actually watch,
    # rather than against prose.
    _LANDING_TARGET_TYPE_VISION_OTHER = 3
    _MAV_FRAME_BODY_FRD = 12

    def send_landing_target(self, bearing, *, target_num: int = 0,
                            distance_m: float = 0.0,
                            coasted: bool = False,
                            gap_age_s: float = 0.0,
                            board_capture_s: Optional[float] = None) -> None:
        """Publish one selected target as a body-frame bearing. Fire-and-forget.

        `board_capture_s`: When the frame was exposed, on the board's clock.
        A bearing is already tens of ms old when it lands, and the link adds a
        variable delay on top (arrival jitter measured at 6.67 ms sd, 35 ms
        p2p). The board holds its gyro at 500 Hz, so given the capture instant
        in its OWN `time_boot_ms` base it can de-rotate the bearing exactly by
        the angle it turned since -- something no host-side estimate matches.
        The host knows that instant because `ClockMap` already maps board time
        to host time; this is the inverse. When given, `time_usec` carries it
        (microseconds since board boot) and `z` = `UPLINK_TIME_BOARD` says so,
        so a receiver can never mistake it for the old advisory host stamp.
        None (clock map not fitted) keeps the previous bytes exactly.

        `bearing` is a `mongla_control.bearing.Bearing`. One message per frame
        for the ONE currently-selected target: the board never sees candidate
        boxes and never does data association, which is deliberate -- that is
        the host's job and it is where the class filter, the coast and the
        continuity lock live.

        Absence of a message is the only loss signal. There is no "lost" flag in
        the spec: a detector that sees nothing simply stops sending, and the
        board ages the last one out. So a caller must NOT keep re-sending a
        stale bearing to "hold" a target -- that is indistinguishable from a
        live one and defeats the board's staleness timer, which is the entire
        safety mechanism on this path.

        A non-finite field is dropped rather than sent. NaN on this wire would
        reach a 500 Hz control loop.

        `coasted` and `gap_age_s` ride the message's spare x/y fields, which the
        spec leaves undefined and we were sending as literal 0.0. They matter
        because the board can only age a target from ITS OWN receipt time: it
        cannot see that what just arrived is a Kalman prediction of a track
        whose last real detection was 0.35 s ago. Without them a coasted target
        is DOUBLE-DECAYED -- once by our coast authority and again by the
        board's staleness -- roughly twice as fast, so a coast we still consider
        live at 0.6 s is long dead on their side.

        Costs nothing today: the board parses msgid 149 and drops it. It becomes
        a latent bug the moment staleness is implemented, which is exactly when
        nobody would think to look here. Proposed upstream in PR #2 §4.
        """
        vals = (bearing.angle_x, bearing.angle_y, bearing.size_x, bearing.size_y)
        if not all(math.isfinite(float(v)) for v in vals):
            self._log_info('[SROT ] landing_target: non-finite bearing, dropped')
            return
        on_board = (board_capture_s is not None
                    and math.isfinite(board_capture_s) and board_capture_s > 0.0)
        with self._tx_lock:
            self.master.mav.landing_target_send(
                (int(board_capture_s * 1e6) if on_board
                 else int(time.time() * 1e6)),   # advisory host stamp otherwise
                int(target_num) & 0xFF,
                self._MAV_FRAME_BODY_FRD,
                float(bearing.angle_x), float(bearing.angle_y),
                float(distance_m),               # 0 = unknown
                float(bearing.size_x), float(bearing.size_y),
                # x = coasted flag, y = seconds since the last REAL detection.
                # z = which clock time_usec is on (see the docstring).
                1.0 if coasted else 0.0,
                float(max(0.0, gap_age_s)) if math.isfinite(gap_age_s) else 0.0,
                sp.UPLINK_TIME_BOARD if on_board else 0.0, (0.0, 0.0, 0.0, 0.0),
                self._LANDING_TARGET_TYPE_VISION_OTHER,
                0)                               # position_valid = 0, angle-only

    def send_speed_estimate(self, vx: float, vy: float, var_x: float,
                            var_y: float, board_s: Optional[float]) -> bool:
        """Our filtered BODY velocity to the board, as VISION_SPEED_ESTIMATE (103).

        The PI measures, the board closes the loop. A distance move is an
        integral of velocity along the leg. The board can integrate at 500 Hz
        against its own fused heading between our 10 Hz updates; the host
        cannot close that loop through a 115200-baud link with 35 ms p2p
        arrival jitter. So this sends the one thing only the Pi has -- velocity
        over ground, from flow + the command model through the RIEKF -- and
        nothing else.

        FRAME: body FRD (x forward, y right), NOT the local frame the MAVLink
        spec names for 103, stated in the upstream PR. z is unobserved by a
        downward camera and says so with an enormous variance, never a
        confident 0.

        `usec` is the estimate's instant on the BOARD clock. Without a fitted
        `ClockMap` there is no honest stamp, so nothing is sent: an unstamped
        velocity lets the board integrate it at the wrong instant, which is a
        distance error that grows with every update.

        Returns whether it was sent. Non-finite input is refused, not clamped.
        """
        vals = (vx, vy, var_x, var_y)
        if board_s is None or not (math.isfinite(board_s) and board_s > 0.0):
            return False
        if not all(math.isfinite(float(v)) for v in vals) or var_x <= 0.0 or var_y <= 0.0:
            return False
        cov = (float(var_x), 0.0, 0.0,
               0.0, float(var_y), 0.0,
               0.0, 0.0, sp.SPEED_Z_UNOBSERVED_VAR)
        with self._tx_lock:
            self.master.mav.vision_speed_estimate_send(
                int(board_s * 1e6), float(vx), float(vy), 0.0, cov, 0)
        return True

    def send_position_estimate(self, x: float, y: float, z: float, yaw: float,
                               cov6, board_s: Optional[float],
                               reset_counter: int) -> bool:
        """Our fused POSITION to the board, as VISION_POSITION_ESTIMATE (102).

        x, y, z metres in the localization frame (NED: z is +depth), yaw rad.
        `cov6` is the 6x6 (x, y, z, roll, pitch, yaw) covariance as nested
        rows; only the upper triangle is sent, which is the message's layout.
        roll and pitch are sent as 0 with their covariance: the board owns
        attitude and must not take ours.

        `reset_counter` must increase whenever the estimate jumped (a frame
        change, a prop fix): the board integrates deltas, and a delta across a
        reset is a teleport. Stamped on the BOARD clock; nothing is sent without
        one, and non-finite input is refused, not clamped.
        """
        if board_s is None or not (math.isfinite(board_s) and board_s > 0.0):
            return False
        if not all(math.isfinite(float(v)) for v in (x, y, z, yaw)):
            return False
        try:
            c = [[float(cov6[r][k]) for k in range(6)] for r in range(6)]
        except (TypeError, IndexError, ValueError):
            return False
        if any(not math.isfinite(c[i][i]) or c[i][i] <= 0.0 for i in (0, 1, 2, 5)):
            return False
        tri = [c[r][k] for r in range(6) for k in range(r, 6)]
        with self._tx_lock:
            self.master.mav.vision_position_estimate_send(
                int(board_s * 1e6), float(x), float(y), float(z),
                0.0, 0.0, float(yaw), tri, int(reset_counter) & 0xFF)
        return True
