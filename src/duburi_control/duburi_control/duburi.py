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
Every command runs under `self.lock` so only one is active at a time
(stateful exceptions: `lock_heading` returns immediately and leaves
a daemon thread running; `unlock_heading` joins it). Between
commands, state is reset via `_send_neutral_and_settle()`:

  * RC override channels  -> 1500 (active hold) when no lock active,
                              or "neutral on translation channels +
                              Ch4 released" when a heading-lock is
                              active so the lock thread keeps the
                              SET_ATTITUDE_TARGET stream authoritative.
  * COMMAND_ACK cache     -> cleared per ACK-bearing command in pixhawk.
  * Flight mode           -> persisted (set_depth/yaw_*/lock_heading
                              auto-engage ALT_HOLD).
  * Arm state             -> persisted (explicit arm/disarm only).

Exit semantics are owned by the axis module:

  * drive_forward_constant / drive_lateral_constant
        -> aggressive reverse kick + settle (full-velocity exit)
  * drive_forward_eased / drive_lateral_eased
        -> settle only (ease-out IS the brake)
  * yaw_*                  -> neutral stop for 0.3 s (heading-hold latches)
  * arc                    -> neutral stop for >= 0.6 s
  * hold_depth             -> neutral stop for 0.3 s (ALT_HOLD latches)

stop vs. pause vs. lock_heading
-------------------------------
Three release/hold semantics, all distinct:

  * stop()         -> SEND 1500 PWM. Pilot-still-on-loop. Use between
                      commands for short active hold.
  * pause(d)       -> SEND 65535 (NO_OVERRIDE) for `d` seconds. Pilot
                      OFF the loop -- ArduSub falls back to its own
                      automation (ALT_HOLD just sits, MANUAL drifts).
                      Use for stabilisation between mode changes.
  * lock_heading() -> Spawn a 20 Hz SET_ATTITUDE_TARGET streamer in a
                      background thread. Persists across other
                      commands. Returns immediately.
  * unlock_heading -> Stop the streamer; send_neutral.
"""

import threading
import time
from contextlib import contextmanager

from duburi_interfaces.action import Move

from .errors        import ModeChangeError
from .heading_lock  import HeadingLock
from .motion_common import make_writers
from .motion_depth  import hold_depth
from .motion_forward import (
    arc as motion_arc,
    drive_forward_constant, drive_forward_eased,
)
from .motion_lateral import drive_lateral_constant, drive_lateral_eased
from .motion_yaw    import yaw_glide, yaw_snap
from .pixhawk       import Pixhawk


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
    smooth_translate : bool
        False -> *_constant (constant gain + reverse-kick brake).
        True  -> *_eased    (trapezoid_ramp envelope, settle-only brake).
    yaw_source : duburi_sensors.YawSource | None
        None -> read yaw from `pixhawk.get_attitude()` (default).
        else -> read from the injected source (e.g. BNO085Source).
        The same source is used for `lock_heading` so missions can
        rehearse in Gazebo with `mavlink_ahrs` and run on the real
        sub with `bno085`, no code change.
    """

    def __init__(self, pixhawk, log, *,
                 smooth_yaw=False,
                 smooth_translate=False,
                 yaw_source=None):
        self.pixhawk          = pixhawk
        self.log              = log
        self.lock             = threading.Lock()
        self.smooth_yaw       = smooth_yaw
        self.smooth_translate = smooth_translate
        self.yaw_source       = yaw_source
        self._heading_lock    = None       # HeadingLock thread or None

    # ================================================================== #
    #  Arm / Disarm / Mode -- ACK-bearing, no axis movement              #
    # ================================================================== #

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

    # ================================================================== #
    #  Stop / Pause                                                       #
    # ================================================================== #

    def stop(self, settle_time=0.6):
        """Active hold: 1500 PWM on every channel for `settle_time` seconds.

        Used between commands. Lock-aware: when a heading-lock is
        active, only the translation channels go to 1500 -- Ch4 stays
        released so the lock thread keeps authority.
        """
        with self.lock:
            self._writers().neutral()
            self.log.info('[CMD  ] stop -- stabilising...')
            time.sleep(settle_time)
            return self._make_result(True, 'stop: completed')

    def pause(self, duration=2.0):
        """Release RC override for `duration` seconds.

        65535 on every channel tells ArduSub we are NOT on the loop --
        the autopilot's own automation takes over for the duration.
        Heading-lock is auto-suspended for the pause and resumed after.
        """
        with self.lock:
            self.log.info(
                f'[CMD  ] pause {duration:.1f}s -- releasing override')
            with self._suspend_heading_lock():
                self.pixhawk.release_rc_override()
                time.sleep(duration)
                self._writers().neutral()
            return self._make_result(
                True, f'pause: {duration:.1f}s released')

    # ================================================================== #
    #  Forward / Back  -- Ch5                                            #
    # ================================================================== #

    def move_forward(self, duration, gain=80.0, settle=0.0):
        return self._drive_forward(+1, duration, gain, settle)

    def move_back(self, duration, gain=80.0, settle=0.0):
        return self._drive_forward(-1, duration, gain, settle)

    def _drive_forward(self, signed_dir, duration, gain, settle):
        with self.lock:
            self._send_neutral_and_settle()
            run = (drive_forward_eased if self.smooth_translate
                   else drive_forward_constant)
            mode = 'EASED' if self.smooth_translate else 'CONSTANT'
            label = 'forward' if signed_dir > 0 else 'back'
            self.log.info(
                f'[CMD  ] move_{label}  {duration:.1f}s  '
                f'gain={gain:.0f}%  ({mode})  settle={settle:.1f}s')
            run(self.pixhawk, signed_dir, duration, int(gain), self.log,
                self._writers(), yaw_source=self.yaw_source, settle=settle)
            depth = self._current_depth()
            return self._make_result(
                True, f'move_{label}: completed',
                final_value=depth, error_value=0.0)

    # ================================================================== #
    #  Left / Right  -- Ch6                                              #
    # ================================================================== #

    def move_left(self, duration, gain=80.0, settle=0.0):
        return self._drive_lateral(-1, duration, gain, settle)

    def move_right(self, duration, gain=80.0, settle=0.0):
        return self._drive_lateral(+1, duration, gain, settle)

    def _drive_lateral(self, signed_dir, duration, gain, settle):
        with self.lock:
            self._send_neutral_and_settle()
            run = (drive_lateral_eased if self.smooth_translate
                   else drive_lateral_constant)
            mode = 'EASED' if self.smooth_translate else 'CONSTANT'
            label = 'right' if signed_dir > 0 else 'left'
            self.log.info(
                f'[CMD  ] move_{label}  {duration:.1f}s  '
                f'gain={gain:.0f}%  ({mode})  settle={settle:.1f}s')
            run(self.pixhawk, signed_dir, duration, int(gain), self.log,
                self._writers(), yaw_source=self.yaw_source, settle=settle)
            depth = self._current_depth()
            return self._make_result(
                True, f'move_{label}: completed',
                final_value=depth, error_value=0.0)

    # ================================================================== #
    #  arc -- forward thrust + yaw rate at the same time                  #
    # ================================================================== #

    def arc(self, duration, gain=50.0, yaw_rate_pct=30.0, settle=0.0):
        """Curved car-style motion: Ch5 + Ch4 in the same packet.

        Heading-lock is incompatible by design (`arc` changes heading).
        Auto-suspends the lock during the arc; on exit, retargets the
        lock to the new heading and resumes.
        """
        with self.lock:
            self._send_neutral_and_settle()
            self._ensure_yaw_capable_mode()
            self.log.info(
                f'[CMD  ] arc  {duration:.1f}s  gain={gain:.0f}%  '
                f'yaw_rate={yaw_rate_pct:+.0f}%  settle={settle:.1f}s')
            with self._suspend_heading_lock():
                signed_dir = +1 if gain >= 0 else -1
                motion_arc(self.pixhawk, signed_dir, duration, abs(int(gain)),
                           yaw_rate_pct, self.log,
                           yaw_source=self.yaw_source, settle=settle)
            new_heading = self._current_heading()
            self._retarget_heading_lock(new_heading)
            return self._make_result(
                True, 'arc: completed',
                final_value=new_heading, error_value=0.0)

    # ================================================================== #
    #  Yaw  -- sharp pivots                                              #
    # ================================================================== #

    def yaw_left(self, target, timeout=30.0, settle=0.0):
        return self._turn(-abs(target), timeout, 'LEFT', settle)

    def yaw_right(self, target, timeout=30.0, settle=0.0):
        return self._turn(+abs(target), timeout, 'RIGHT', settle)

    def _turn(self, signed_degrees, timeout, label, settle):
        """Execute a yaw turn relative to the current heading."""
        with self.lock:
            self._send_neutral_and_settle()
            self._ensure_yaw_capable_mode()
            start_heading  = self._current_heading()
            target_heading = (start_heading + signed_degrees) % 360
            run_yaw = yaw_glide if self.smooth_yaw else yaw_snap
            with self._suspend_heading_lock():
                run_yaw(self.pixhawk, start_heading, target_heading,
                        timeout, label, self.log,
                        yaw_source=self.yaw_source)
                self._send_neutral_and_settle(settle_time=0.3 + settle)
            final_heading = self._current_heading()
            self._retarget_heading_lock(final_heading)
            error = Pixhawk.heading_error(target_heading, final_heading)
            return self._make_result(
                True, f'yaw_{label.lower()}: completed',
                final_value=final_heading, error_value=float(error))

    def _ensure_yaw_capable_mode(self):
        """Engage ALT_HOLD when the autopilot isn't already in a mode
        that tracks absolute yaw. ALT_HOLD also holds the current
        depth, which prevents the slow gravity-sink we'd otherwise get
        during the turn.
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

    # ================================================================== #
    #  Depth                                                              #
    # ================================================================== #

    def set_depth(self, target, timeout=30.0, settle=0.0):
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
            hold_depth(self.pixhawk, target, timeout, self.log,
                       neutral_writer=self._writers().neutral)
            self._send_neutral_and_settle(settle_time=0.3 + settle)
            depth = self._current_depth()
            return self._make_result(
                True, 'set_depth: completed',
                final_value=depth, error_value=abs(target - depth))

    # ================================================================== #
    #  Heading lock -- depth-hold's yaw cousin                            #
    # ================================================================== #

    def lock_heading(self, target=0.0, timeout=300.0):
        """Engage continuous heading-hold using the configured yaw_source.

        `target=0` (the rosidl unset default) means "lock at current
        heading right now". Returns immediately -- the actual streaming
        runs on a daemon thread so subsequent motion commands stack on
        top with active heading correction.

        Source-agnostic: works with `mavlink_ahrs` (Gazebo / bench),
        `bno085` (real sub), or any future YawSource. Uniform plug.
        """
        with self.lock:
            self._ensure_yaw_capable_mode()
            current = self._current_heading()
            actual_target = current if abs(target) < 1e-3 else float(target) % 360.0

            if self._heading_lock is not None:
                self._heading_lock.stop()
                self._heading_lock = None

            self._heading_lock = HeadingLock(
                pixhawk=self.pixhawk,
                target_deg=actual_target,
                yaw_source=self.yaw_source,
                log=self.log,
                timeout=timeout,
            )
            self._heading_lock.start()

            return self._make_result(
                True,
                f'lock_heading: locked at {actual_target:.1f} deg '
                f'(timeout {timeout:.0f}s)',
                final_value=actual_target, error_value=0.0)

    def unlock_heading(self):
        """Stop the heading-lock streamer and send neutral."""
        with self.lock:
            if self._heading_lock is None:
                return self._make_result(True, 'unlock_heading: no-op')
            self._heading_lock.stop()
            self._heading_lock = None
            self.pixhawk.send_neutral()
            return self._make_result(True, 'unlock_heading: released')

    # ================================================================== #
    #  Internal helpers                                                   #
    # ================================================================== #

    def _writers(self):
        """Build a `Writers` matching the current heading-lock state."""
        return make_writers(self.pixhawk, release_yaw=self._lock_active())

    def _lock_active(self):
        return self._heading_lock is not None

    @contextmanager
    def _suspend_heading_lock(self):
        """Pause the lock thread for the duration of the block.

        Used by yaw_left / yaw_right / arc / pause -- commands whose
        intent is to either change heading or release the override.
        Re-arms `resume()` even if the body raises so a failed yaw
        doesn't leave the lock paused forever.
        """
        active = self._heading_lock is not None
        if active:
            self._heading_lock.suspend()
        try:
            yield
        finally:
            if active and self._heading_lock is not None:
                self._heading_lock.resume()

    def _retarget_heading_lock(self, new_heading_deg):
        """Update the lock target to follow yaw_left/yaw_right/arc exit."""
        if self._heading_lock is not None:
            self._heading_lock.retarget(new_heading_deg)

    def _send_neutral_and_settle(self, settle_time=0.6):
        """Stop without taking the lock (for use INSIDE a command).

        Lock-aware: uses `_writers().neutral()` so Ch4 stays released
        when a heading-lock is active.
        """
        self._writers().neutral()
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
