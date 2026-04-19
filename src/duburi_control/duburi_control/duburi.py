#!/usr/bin/env python3
"""Duburi -- the high-level facade over per-axis motion modules.

Every public method is a verb that matches an entry in
`commands.COMMANDS` (and in turn a `Move.Goal.cmd` value). The action
server dispatches by name -- adding a new command means: row in
`commands.py`, method here, that's it.

Every method returns a `duburi_interfaces.action.Move.Result` so the
action server can pass it straight through. Failure modes raise from
the errors module; the action server catches and packages them.

Cross-command isolation contract
--------------------------------
Every command runs under `self.lock` so only one is active at a time.
Between commands, state is reset via `stop()` (or the variant's own
brake+settle for linear moves):

  * RC override channels 1..6  -> 1500 (active hold). Supersedes any
    lingering SET_ATTITUDE_TARGET from a prior yaw.
  * COMMAND_ACK cache  -> cleared per ACK-bearing command in pixhawk.
  * Flight mode        -> persisted (set_depth auto-engages ALT_HOLD).
  * Arm state          -> persisted (explicit arm/disarm only).

Exit semantics are owned by the axis module:

  * drive_constant -> aggressive reverse kick + settle (full-velocity exit)
  * drive_eased    -> settle only (ease-out IS the brake)
  * yaw_*          -> neutral stop for 0.3 s (ArduSub heading-hold latches)
  * hold_depth     -> ArduSub onboard PID drives to absolute setpoint;
                       neutral stop for 0.3 s on exit (ALT_HOLD latches)

stop vs. pause
--------------
Both are "release the goal", but they're not the same:

  * stop()   -> SEND 1500 PWM. The autopilot still treats us as the
                pilot, so its onboard heading-hold and depth-hold latch
                at the current state. Use this between commands.
  * pause(d) -> SEND 65535 (NO_OVERRIDE) for `d` seconds. Tells the
                autopilot we are NOT on the loop -- it falls back to
                its own automation (ALT_HOLD just sits, MANUAL drifts).
                Use this for A/B comparison or to hand off to a future
                higher-level autopilot mode.
"""

import threading
import time

from duburi_interfaces.action import Move

from .errors        import ModeChangeError
from .pixhawk       import Pixhawk
from .motion_yaw    import yaw_snap, yaw_glide
from .motion_linear import drive_constant, drive_eased
from .motion_depth  import hold_depth


# Modes that honour SET_ATTITUDE_TARGET as an *absolute* heading goal.
#
# MANUAL and STABILIZE both fail us:
#   MANUAL    -- attitude target silently dropped, sub doesn't rotate.
#   STABILIZE -- yaw channel is treated as a rate, not absolute heading;
#                also no depth hold, so the sub sinks during the turn.
#
# ALT_HOLD is the smallest mode that does both: holds depth at whatever
# the sub is at when the mode is engaged, and the heading-hold controller
# tracks our absolute target.
YAW_OK_MODES = ('ALT_HOLD', 'POSHOLD', 'GUIDED')


class Duburi:
    """Serialised movement facade.

    Parameters
    ----------
    pixhawk : Pixhawk
        Live MAVLink connection to the autopilot.
    log : logging-style logger
        Anything with `.info(msg, throttle_duration_sec=...)` works
        (rclpy logger does, stdlib logging.Logger ignores the kwarg).
    smooth_yaw : bool
        False -> yaw_snap (bang-bang, ArduSub onboard PID profile).
        True  -> yaw_glide (smootherstep setpoint sweep, no overshoot).
    smooth_linear : bool
        False -> drive_constant (constant gain + reverse-kick brake).
        True  -> drive_eased (trapezoid_ramp envelope, settle-only brake).
    yaw_source : duburi_sensors.YawSource | None
        None -> read yaw from `pixhawk.get_attitude()` (default).
        else -> read from the injected source (e.g. BNO085Source).
    """

    def __init__(self, pixhawk, log, *,
                 smooth_yaw=False,
                 smooth_linear=False,
                 yaw_source=None):
        self.pixhawk       = pixhawk
        self.log           = log
        self.lock          = threading.Lock()
        self.smooth_yaw    = smooth_yaw
        self.smooth_linear = smooth_linear
        self.yaw_source    = yaw_source

    # ================================================================= #
    #  Arm / Disarm / Mode -- ACK-bearing, no axis movement              #
    # ================================================================= #

    def arm(self, timeout=15.0):
        accepted, reason = self.pixhawk.arm(timeout)
        return self._make_result(accepted, f'arm: {reason}')

    def disarm(self, timeout=20.0):
        accepted, reason = self.pixhawk.disarm(timeout)
        return self._make_result(accepted, f'disarm: {reason}')

    def set_mode(self, target_name, timeout=8.0):
        accepted, reason = self.pixhawk.set_mode(target_name, timeout)
        return self._make_result(
            accepted, f'set_mode {target_name}: {reason}')

    # ================================================================= #
    #  Stop / Pause                                                       #
    # ================================================================= #

    def stop(self, settle_time=0.6):
        """Active hold: 1500 PWM on every channel for `settle_time` seconds.

        Used between commands. Internal callers (the per-axis modules
        on entry) ignore the return value; an external `stop` command
        gets the result back through the action server.
        """
        with self.lock:
            self.pixhawk.send_neutral()
            self.log.info('[CMD  ] stop -- stabilising...')
            time.sleep(settle_time)
            return self._make_result(True, 'stop: completed')

    def pause(self, duration=2.0):
        """Release RC override for `duration` seconds.

        65535 on every channel tells ArduSub we are NOT on the loop --
        the autopilot's own automation (ALT_HOLD / POSHOLD) takes over
        for the duration. Use for A/B testing or to hand off to a
        higher-level autopilot mode.
        """
        with self.lock:
            self.log.info(
                f'[CMD  ] pause {duration:.1f}s -- releasing override')
            self.pixhawk.release_rc_override()
            time.sleep(duration)
            self.pixhawk.send_neutral()
            return self._make_result(
                True, f'pause: {duration:.1f}s released')

    # ================================================================= #
    #  Linear translations                                                #
    # ================================================================= #

    def move_forward(self, duration, gain=80.0):
        return self._drive('forward', duration, gain)

    def move_back(self, duration, gain=80.0):
        return self._drive('back', duration, gain)

    def move_left(self, duration, gain=80.0):
        return self._drive('left', duration, gain)

    def move_right(self, duration, gain=80.0):
        return self._drive('right', duration, gain)

    def _drive(self, direction, duration, gain):
        """Execute a linear push.

        No DVL yet, so we report current depth as `final_value` and
        leave `error_value = 0`. When DVL lands this is where the
        odometry-based translation error gets surfaced.
        """
        with self.lock:
            self._send_neutral_and_settle()
            run_drive = drive_eased if self.smooth_linear else drive_constant
            mode      = 'EASED'     if self.smooth_linear else 'CONSTANT'
            self.log.info(
                f'[CMD  ] move_{direction}  {duration:.1f}s  '
                f'gain={gain:.0f}%  ({mode})')
            run_drive(self.pixhawk, direction, duration, int(gain), self.log)
            depth = self._current_depth()
            return self._make_result(
                True, f'move_{direction}: completed',
                final_value=depth, error_value=0.0)

    # ================================================================= #
    #  Yaw                                                                #
    # ================================================================= #

    def yaw_left(self, target, timeout=30.0):
        return self._turn(-abs(target), timeout, 'LEFT')

    def yaw_right(self, target, timeout=30.0):
        return self._turn(+abs(target), timeout, 'RIGHT')

    def _turn(self, signed_degrees, timeout, label):
        """Execute a yaw turn relative to the current heading."""
        with self.lock:
            self._send_neutral_and_settle()
            self._ensure_yaw_capable_mode()
            start_heading  = self._current_heading()
            target_heading = (start_heading + signed_degrees) % 360
            run_yaw        = yaw_glide if self.smooth_yaw else yaw_snap
            run_yaw(self.pixhawk, start_heading, target_heading,
                    timeout, label, self.log,
                    yaw_source=self.yaw_source)
            self._send_neutral_and_settle(settle_time=0.3)
            final_heading = self._current_heading()
            error         = Pixhawk.heading_error(target_heading, final_heading)
            return self._make_result(
                True, f'yaw_{label.lower()}: completed',
                final_value=final_heading, error_value=float(error))

    def _ensure_yaw_capable_mode(self):
        """Engage ALT_HOLD when the autopilot isn't already in a mode
        that tracks absolute yaw. ALT_HOLD also holds the current
        depth, which prevents the slow gravity-sink we'd otherwise get
        during the turn.

        Raises ModeChangeError if the autopilot refuses the switch --
        yaw cannot succeed without an attitude-tracking mode.
        """
        current = self.pixhawk.get_mode()
        if current in YAW_OK_MODES:
            return
        self.log.info(
            f'[CMD  ] yaw needs ALT_HOLD -- switching {current} -> ALT_HOLD')
        accepted, reason = self.pixhawk.set_mode('ALT_HOLD')
        if not accepted:
            self.log.info(f'[YAW  ] !! set_mode ALT_HOLD failed: {reason}')
            raise ModeChangeError(
                f'set_mode ALT_HOLD rejected ({reason}); '
                f'yaw cannot run from {current}')

    def _current_heading(self):
        """Read current heading, preferring the injected source when fresh."""
        if self.yaw_source is not None:
            heading = self.yaw_source.read_yaw()
            if heading is not None:
                return heading
        attitude = self.pixhawk.get_attitude()
        return attitude['yaw'] if attitude else 0.0

    # ================================================================= #
    #  Depth                                                              #
    # ================================================================= #

    def set_depth(self, target, timeout=30.0):
        """Drive to `target` metres (negative = below surface) and hold."""
        with self.lock:
            self._send_neutral_and_settle()
            self.log.info(f'[CMD  ] set_depth  {target:.2f}m')
            accepted, reason = self.pixhawk.set_mode('ALT_HOLD')
            if not accepted:
                self.log.info(
                    f'[DEPTH] !! set_mode ALT_HOLD failed: {reason}')
                raise ModeChangeError(
                    f'set_mode ALT_HOLD rejected ({reason}); '
                    'depth controller will not engage')
            hold_depth(self.pixhawk, target, timeout, self.log)
            self._send_neutral_and_settle(settle_time=0.3)
            depth = self._current_depth()
            return self._make_result(
                True, 'set_depth: completed',
                final_value=depth, error_value=abs(target - depth))

    # ================================================================= #
    #  Internal helpers                                                   #
    # ================================================================= #

    def _send_neutral_and_settle(self, settle_time=0.6):
        """Stop without taking the lock (for use INSIDE a command).

        `stop()` itself takes the lock, but per-axis dispatch already
        holds it -- this is the lock-free version used as the entry
        and exit guard within the public methods.
        """
        self.pixhawk.send_neutral()
        self.log.info('[CMD  ] stop -- stabilising...')
        time.sleep(settle_time)

    def _current_depth(self):
        attitude = self.pixhawk.get_attitude()
        return float(attitude['depth']) if attitude else 0.0

    def _make_result(self, success, message, final_value=None, error_value=0.0):
        """Build a `Move.Result`, defaulting `final_value` to current depth."""
        result = Move.Result()
        result.success     = bool(success)
        result.message     = str(message)
        result.final_value = float(
            final_value if final_value is not None else self._current_depth())
        result.error_value = float(error_value)
        return result
