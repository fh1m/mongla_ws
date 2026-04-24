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
(stateful exception: `lock_heading` returns immediately and leaves a
daemon thread running; `unlock_heading` joins it). Between commands,
state is reset via `_send_neutral_and_settle()`:

  * RC override channels  -> 1500 (active hold) when no lock active,
                              or "neutral on translation channels +
                              Ch4 released" when a heading-lock is
                              active so the lock thread keeps the
                              Ch4 rate-override stream authoritative.
  * COMMAND_ACK cache     -> cleared per ACK-bearing command in pixhawk.
  * Flight mode           -> persisted (set_depth / yaw_* / lock_heading
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

Depth strategy (no DepthLock daemon)
------------------------------------
``set_depth`` engages ALT_HOLD and runs a single ``hold_depth`` drive
loop until the target is reached, then hands depth back to ArduSub's
onboard ALT_HOLD controller. ALT_HOLD closes the depth loop at 400 Hz
internally and keeps holding the last setpoint forever -- we do NOT
need a 5 Hz Python streamer to refresh it. Subsequent translations,
yaws, vision verbs etc. all run with depth held automatically as long
as we stay in ALT_HOLD.

stop vs. pause vs. lock_heading
-------------------------------
Three release/hold semantics, all distinct:

  * stop()         -> SEND 1500 PWM. Pilot-still-on-loop. Use between
                      commands for short active hold.
  * pause(d)       -> SEND 65535 (NO_OVERRIDE) for `d` seconds. Pilot
                      OFF the loop -- ArduSub falls back to its own
                      automation (ALT_HOLD just sits, MANUAL drifts).
                      Use for stabilisation between mode changes.
  * lock_heading() -> Spawn a 20 Hz Ch4 rate-override streamer in a
                      background thread driven by yaw_source. Persists
                      across other commands. Returns immediately.
  * unlock_heading -> Stop the streamer; send_neutral.

Heartbeat
---------
A separate ``Heartbeat`` daemon (owned by the manager, injected here)
streams an all-neutral RC override at 5 Hz whenever no other writer
is active. That keeps ``FS_PILOT_INPUT`` from disarming the sub during
long idle gaps between commands. Every command pauses the heartbeat
on entry (via ``with self.lock:`` -> ``_heartbeat_hold()``) and resumes
it on exit; ``lock_heading`` keeps it paused for the entire lock
lifetime because the lock thread is itself writing the wire.
"""

import threading
import time
from contextlib import contextmanager

from duburi_interfaces.action import Move

from .vision_verbs import VisionVerbs
from .errors        import ModeChangeError, NotArmedError
from .heading_lock  import HeadingLock
from .motion_writers import make_writers
from .motion_depth  import hold_depth
from .motion_forward import (
    arc as motion_arc,
    drive_forward_constant, drive_forward_eased, drive_forward_dist,
)
from .motion_lateral import (
    drive_lateral_constant, drive_lateral_eased, drive_lateral_dist,
)
from .motion_yaw    import yaw_glide, yaw_snap
from .pixhawk       import Pixhawk
from .tracing       import command_scope


# Verbs that may run while the AUV is disarmed. Every other verb that
# enters _command_scope requires arm() first: ArduSub silently drops all
# RC_CHANNELS_OVERRIDE and SET_POSITION_TARGET_GLOBAL_INT frames while
# disarmed, so motion commands would appear to succeed but move nothing.
#
# arm / disarm / set_mode are NOT listed here because they use the
# tracing-only `command_scope` directly and never enter _command_scope.
_UNARM_SAFE = frozenset({'stop', 'pause', 'unlock_heading', 'dvl_connect'})


# Modes whose ALT_HOLD-style onboard automation honours BOTH our depth
# setpoint (via SET_POSITION_TARGET_GLOBAL_INT) AND our Ch4 rate input
# (via RC_CHANNELS_OVERRIDE) as the heading-hold rate command.
#
# MANUAL and STABILIZE both fail us:
#   MANUAL    -- depth setpoint silently dropped; sub sinks during turns.
#   STABILIZE -- no depth hold, only attitude self-levelling; sub still
#                sinks. Yaw rate input is honoured but you lose the dive.
#
# ALT_HOLD is the smallest mode that does both: holds depth at whatever
# the sub is at when the mode is engaged, and accepts the lock thread's
# Ch4 rate-override as the heading-hold rate target.
YAW_OK_MODES = ('ALT_HOLD', 'POSHOLD', 'GUIDED')


class Duburi(VisionVerbs):
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
                 yaw_source=None,
                 vision_state_provider=None,
                 heartbeat=None,
                 quick_settle=False):
        """vision_state_provider(camera_name) -> VisionState | None.

        Injected by the manager so the facade can stay rclpy-free and
        the vision verbs can ask "give me state for camera X" without
        knowing how the subscriptions were set up. Same pattern as
        `yaw_source` -- the facade never imports rclpy.

        heartbeat : Heartbeat | None
            Optional. When supplied, every command body pauses the
            heartbeat for the duration of the command (so its 1500 us
            packets don't race the command's per-axis writes), and
            ``lock_heading`` / ``unlock_heading`` toggle a longer-lived
            pause spanning the whole lock lifetime. None (the default,
            used by unit tests) skips the cooperation entirely.

        quick_settle : bool
            False (default) -> every command pre-flight calls
                ``_send_neutral_and_settle`` which writes 1500 PWM and
                sleeps 0.6 s. Conservative; matches the historical
                Move-action contract so a stale Ch5/Ch6 from the
                previous command can't bleed into the next one.
            True            -> the pre-flight pause is SKIPPED when the
                next command shares the same axis set as the previous
                one AND no heading lock is active. Cuts ~0.6 s off
                chained ``move_forward; move_forward; move_left;
                move_left`` sequences in scripted demos. The pause is
                still honoured around lock-bearing or mode-changing
                commands so the autopilot has time to latch.
        """
        self.pixhawk           = pixhawk
        self.log               = log
        self.lock              = threading.Lock()
        self.smooth_yaw        = smooth_yaw
        self.smooth_translate  = smooth_translate
        self.yaw_source        = yaw_source
        self.vision_state_provider = vision_state_provider
        self._heartbeat        = heartbeat
        self.quick_settle      = bool(quick_settle)
        self._heading_lock     = None      # HeadingLock thread or None
        # Tracks which channel set the last in-command write touched, so
        # the pre-flight pause can be skipped when the next command uses
        # the same axes. None = no recent write / lock state changed.
        self._last_axes        = None

    # ================================================================== #
    #  Arm / Disarm / Mode -- ACK-bearing, no axis movement              #
    # ================================================================== #

    def arm(self, timeout=15.0):
        """impl: pixhawk.py:arm (COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM, p1=1)."""
        with command_scope('arm'):
            accepted, reason = self.pixhawk.arm(timeout)
        return self._make_result(accepted, f'arm: {reason}')

    def disarm(self, timeout=20.0):
        """impl: pixhawk.py:disarm (set_mode MANUAL -> send_neutral -> COMMAND_LONG p1=0)."""
        with command_scope('disarm'):
            accepted, reason = self.pixhawk.disarm(timeout)
        return self._make_result(accepted, f'disarm: {reason}')

    def set_mode(self, target_name, timeout=8.0):
        """impl: pixhawk.py:set_mode (legacy SET_MODE retried until heartbeat reflects)."""
        with command_scope('set_mode'):
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

        impl: pixhawk.py:send_rc_override via motion_writers.neutral.
        """
        with self._command_scope('stop'):
            self._writers().neutral()
            self.log.info('[CMD  ] stop -- stabilising...')
            time.sleep(settle_time)
            return self._make_result(True, 'stop: completed')

    def pause(self, duration=2.0):
        """Release RC override for `duration` seconds.

        65535 on every channel tells ArduSub we are NOT on the loop --
        the autopilot's own automation takes over for the duration.
        Heading-lock is auto-suspended for the pause and resumed
        after; the heartbeat is also paused (the whole point of pause
        is that nobody is writing the wire).

        impl: pixhawk.py:release_rc_override (all channels = 65535).
        """
        with self._command_scope('pause'):
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
        """impl: motion_forward.drive_forward_constant/_eased -> pixhawk.send_rc_translation."""
        return self._drive_forward(+1, duration, gain, settle)

    def move_back(self, duration, gain=80.0, settle=0.0):
        """impl: motion_forward.drive_forward_constant/_eased -> pixhawk.send_rc_translation."""
        return self._drive_forward(-1, duration, gain, settle)

    def _drive_forward(self, signed_dir, duration, gain, settle):
        verb = 'move_forward' if signed_dir > 0 else 'move_back'
        with self._command_scope(verb):
            self._send_neutral_and_settle(axes=frozenset({'forward'}))
            run = (drive_forward_eased if self.smooth_translate
                   else drive_forward_constant)
            mode = 'EASED' if self.smooth_translate else 'CONSTANT'
            label = 'forward' if signed_dir > 0 else 'back'
            self.log.info(
                f'[CMD  ] move_{label}  {duration:.1f}s  '
                f'gain={gain:.0f}%  ({mode})  settle={settle:.1f}s')
            with self._suspend_heading_lock():
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
        """impl: motion_lateral.drive_lateral_constant/_eased -> pixhawk.send_rc_translation."""
        return self._drive_lateral(-1, duration, gain, settle)

    def move_right(self, duration, gain=80.0, settle=0.0):
        """impl: motion_lateral.drive_lateral_constant/_eased -> pixhawk.send_rc_translation."""
        return self._drive_lateral(+1, duration, gain, settle)

    def _drive_lateral(self, signed_dir, duration, gain, settle):
        verb = 'move_right' if signed_dir > 0 else 'move_left'
        with self._command_scope(verb):
            self._send_neutral_and_settle(axes=frozenset({'lateral'}))
            run = (drive_lateral_eased if self.smooth_translate
                   else drive_lateral_constant)
            mode = 'EASED' if self.smooth_translate else 'CONSTANT'
            label = 'right' if signed_dir > 0 else 'left'
            self.log.info(
                f'[CMD  ] move_{label}  {duration:.1f}s  '
                f'gain={gain:.0f}%  ({mode})  settle={settle:.1f}s')
            with self._suspend_heading_lock():
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

        impl: motion_forward.arc -> pixhawk.send_rc_override (Ch5+Ch4 same packet).
        """
        with self._command_scope('arc'):
            self._send_neutral_and_settle(axes=frozenset({'forward', 'yaw'}))
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
        """impl: motion_yaw.yaw_snap/yaw_glide -> pixhawk.send_rc_override (Ch4 rate)."""
        return self._turn(-abs(target), timeout, 'LEFT', settle)

    def yaw_right(self, target, timeout=30.0, settle=0.0):
        """impl: motion_yaw.yaw_snap/yaw_glide -> pixhawk.send_rc_override (Ch4 rate)."""
        return self._turn(+abs(target), timeout, 'RIGHT', settle)

    def _turn(self, signed_degrees, timeout, label, settle):
        """Execute a yaw turn relative to the current heading."""
        verb = 'yaw_right' if signed_degrees > 0 else 'yaw_left'
        with self._command_scope(verb):
            self._send_neutral_and_settle(axes=frozenset({'yaw'}))
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
        """Drive to `target` metres (negative = below surface) and hold.

        Engages ALT_HOLD (so ArduSub's onboard 400 Hz depth controller
        owns the loop), drives ``hold_depth`` until the target is
        reached, then exits. ALT_HOLD continues to hold the achieved
        setpoint forever -- subsequent commands run with depth held
        automatically as long as the mode is preserved. No background
        streamer required.

        impl: pixhawk.set_target_depth (SET_POSITION_TARGET_GLOBAL_INT) +
        motion_depth.hold_depth control loop.
        """
        with self._command_scope('set_depth'):
            self._send_neutral_and_settle(axes=frozenset({'depth'}))
            self.log.info(f'[CMD  ] set_depth  {target:.2f}m')
            self._ensure_alt_hold('set_depth')
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

        While the lock is engaged the heartbeat is held (the lock
        thread is itself writing the wire). ``unlock_heading`` releases
        that hold so the heartbeat resumes between later commands.

        Source-agnostic: works with `mavlink_ahrs` (Gazebo / bench),
        `bno085` (real sub), or any future YawSource. Uniform plug.

        impl: heading_lock.HeadingLock daemon -> pixhawk.send_rc_override (Ch4 rate, 20 Hz).
        """
        with self._command_scope('lock_heading'):
            self._ensure_yaw_capable_mode()
            current = self._current_heading()
            actual_target = current if abs(target) < 1e-3 else float(target) % 360.0

            if self._heading_lock is not None:
                self._heading_lock.stop()
                self._heading_lock = None
                self._release_heartbeat_for_lock()

            self._heading_lock = HeadingLock(
                pixhawk=self.pixhawk,
                target_deg=actual_target,
                yaw_source=self.yaw_source,
                log=self.log,
                timeout=timeout,
            )
            self._heading_lock.start()
            self._hold_heartbeat_for_lock()

            return self._make_result(
                True,
                f'lock_heading: locked at {actual_target:.1f} deg '
                f'(timeout {timeout:.0f}s)',
                final_value=actual_target, error_value=0.0)

    def unlock_heading(self):
        """Stop the heading-lock streamer and send neutral.

        impl: HeadingLock.stop -> pixhawk.send_neutral.
        """
        with self._command_scope('unlock_heading'):
            if self._heading_lock is None:
                return self._make_result(True, 'unlock_heading: no-op')
            self._heading_lock.stop()
            self._heading_lock = None
            self._release_heartbeat_for_lock()
            self.pixhawk.send_neutral()
            return self._make_result(True, 'unlock_heading: released')

    # ================================================================== #
    #  DVL                                                                #
    # ================================================================== #

    def dvl_connect(self):
        """Connect to the Nortek Nucleus 1000 DVL and begin streaming.

        Calls `yaw_source.connect()` when the active source supports it
        (i.e. NucleusDVLSource). Safe no-op if the source has no connect
        method (e.g. mavlink_ahrs, bno085).

        impl: NucleusDVLSource.connect() -> TCP 192.168.2.201:9000, auth, START.
        """
        with command_scope('dvl_connect'):
            src = self.yaw_source
            if src is not None and hasattr(src, 'connect'):
                self.log.info('[DVL  ] dvl_connect: connecting...')
                src.connect()  # type: ignore[union-attr]
                self.log.info('[DVL  ] dvl_connect: connected')
                return self._make_result(True, 'dvl_connect: streaming')
            self.log.info('[DVL  ] dvl_connect: yaw_source has no connect() -- no-op')
            return self._make_result(True, 'dvl_connect: no-op (source has no connect)')

    # ================================================================== #
    #  DVL distance-based motion                                          #
    # ================================================================== #

    def move_forward_dist(self, distance_m, gain=60.0, dvl_tolerance=0.1,
                          settle=0.0):
        """Drive forward `distance_m` metres using DVL position feedback.

        impl: motion_forward.drive_forward_dist -> NucleusDVLSource position loop.
        Falls back to open-loop timed estimate if DVL not available.
        """
        return self._drive_forward_dist(+1, distance_m, gain, dvl_tolerance, settle)

    def _drive_forward_dist(self, signed_dir, distance_m, gain,
                            dvl_tolerance, settle):
        verb = 'move_forward_dist' if signed_dir > 0 else 'move_back_dist'
        with self._command_scope(verb):
            self._send_neutral_and_settle(axes=frozenset({'forward'}))
            self.log.info(
                f'[CMD  ] {verb}  {distance_m:.2f}m  '
                f'gain={gain:.0f}%  tol={dvl_tolerance:.3f}m  settle={settle:.1f}s')
            with self._suspend_heading_lock():
                drive_forward_dist(
                    self.pixhawk, signed_dir, distance_m, int(gain),
                    dvl_tolerance, self.log, self._writers(),
                    yaw_source=self.yaw_source, settle=settle)
            depth = self._current_depth()
            return self._make_result(
                True, f'{verb}: completed',
                final_value=depth, error_value=0.0)

    def move_lateral_dist(self, distance_m, gain=36.0, dvl_tolerance=0.1,
                          settle=0.0):
        """Strafe `distance_m` metres (positive=right, negative=left) using DVL.

        impl: motion_lateral.drive_lateral_dist -> NucleusDVLSource position loop.
        Falls back to open-loop timed estimate if DVL not available.
        """
        signed_dir = +1 if distance_m >= 0 else -1
        with self._command_scope('move_lateral_dist'):
            self._send_neutral_and_settle(axes=frozenset({'lateral'}))
            self.log.info(
                f'[CMD  ] move_lateral_dist  {distance_m:.2f}m  '
                f'gain={gain:.0f}%  tol={dvl_tolerance:.3f}m  settle={settle:.1f}s')
            with self._suspend_heading_lock():
                drive_lateral_dist(
                    self.pixhawk, signed_dir, distance_m, int(gain),
                    dvl_tolerance, self.log, self._writers(),
                    yaw_source=self.yaw_source, settle=settle)
            depth = self._current_depth()
            return self._make_result(
                True, 'move_lateral_dist: completed',
                final_value=depth, error_value=0.0)

    # ================================================================== #
    #  Vision verbs                                                       #
    # ================================================================== #
    #
    # Live in `VisionVerbs` (mixed in via the class declaration
    # above) so this file stays focused on motion-axis verbs. The
    # mixin uses only the helpers defined below + base attributes
    # set in __init__, no rclpy.

    # ================================================================== #
    #  Internal helpers                                                   #
    # ================================================================== #

    def _writers(self):
        """Build a `Writers` matching the current heading-lock state."""
        return make_writers(self.pixhawk, release_yaw=self._lock_active())

    def _lock_active(self):
        lock = self._heading_lock
        return lock is not None and not lock.is_suspended

    @contextmanager
    def _command_scope(self, verb):
        """Wrap a command body: serial lock + heartbeat pause + cmd= tag.

        Combining all three into one context manager keeps every command
        body's ``with`` line uniform and guarantees:

          * the heartbeat is always resumed -- even if the command
            raises -- so the next command starts from a known wire
            state;
          * the per-command ``cmd=<verb>`` trace tag (see
            ``tracing.command_scope``) wraps every MAVLink frame the
            verb emits, so ``rg "cmd=<verb>"`` over a debug log
            returns every frame the verb produced.

        ``verb`` is the public method name (``'yaw_right'``,
        ``'vision_align_yaw'``, ...) and shows up verbatim in the
        ``[MAV <fn> cmd=<verb>] ...`` line.
        """
        with self.lock, command_scope(verb):
            if verb not in _UNARM_SAFE and not self.pixhawk.is_armed():
                raise NotArmedError(
                    f'{verb}: AUV is disarmed -- call arm() first')
            if self._heartbeat is not None:
                self._heartbeat.pause()
            try:
                yield
            finally:
                if self._heartbeat is not None:
                    self._heartbeat.resume()

    def _hold_heartbeat_for_lock(self):
        """Mark the heartbeat held for the entire heading-lock lifetime.

        Released by ``_release_heartbeat_for_lock`` from
        ``unlock_heading`` (or when ``lock_heading`` swaps locks).
        """
        if self._heartbeat is not None:
            self._heartbeat.pause()

    def _release_heartbeat_for_lock(self):
        if self._heartbeat is not None:
            self._heartbeat.resume()

    @contextmanager
    def _suspend_heading_lock(self):
        """Pause the lock thread for the duration of the block.

        Used by yaw_left / yaw_right / arc / pause / linear movement --
        commands whose intent is to change heading or take sole authority
        over all RC channels. Re-arms resume() even on exception so a
        failed command never leaves the lock paused forever.
        """
        lock = self._heading_lock
        if lock is not None:
            lock.suspend()
        try:
            yield
        finally:
            lock = self._heading_lock
            if lock is not None and lock.is_suspended:
                lock.resume()

    def _retarget_heading_lock(self, new_heading_deg):
        """Update the lock target to follow yaw_left/yaw_right/arc exit."""
        if self._heading_lock is not None:
            self._heading_lock.retarget(new_heading_deg)

    def _ensure_alt_hold(self, reason):
        """Engage ALT_HOLD when the autopilot isn't already in a mode
        that honours streamed depth setpoints. Used by every verb
        that writes a depth target (``set_depth`` and any vision verb
        touching the depth axis).

        ALT_HOLD, POSHOLD and GUIDED all close the loop on the
        Python-supplied depth setpoint via ArduSub's onboard 400 Hz
        depth controller. MANUAL / STABILIZE silently drop it.
        """
        current = self.pixhawk.get_mode()
        if current in YAW_OK_MODES:        # same set: ALT_HOLD/POSHOLD/GUIDED
            return
        self.log.info(
            f'[CMD  ] {reason} needs ALT_HOLD -- '
            f'switching {current} -> ALT_HOLD')
        accepted, ack_reason = self.pixhawk.set_mode('ALT_HOLD')
        if not accepted:
            self.log.info(f'[DEPTH] !! set_mode ALT_HOLD failed: {ack_reason}')
            raise ModeChangeError(
                f'set_mode ALT_HOLD rejected ({ack_reason}); '
                f'depth setpoint will not engage from {current}')

    def _send_neutral_and_settle(self, settle_time=0.6, axes=None):
        """Stop without taking the lock (for use INSIDE a command).

        Lock-aware: uses ``_writers().neutral()`` so Ch4 stays released
        when a heading-lock is active.

        ``axes`` (optional) tells the quick-settle guard which channel
        set the *next* command will write. When ``self.quick_settle``
        is True AND the same set was written immediately before AND no
        active heading lock is mid-stream, the 0.6 s pause is skipped --
        the previous command's channels are already at the values the
        next command wants to overwrite, so the brake is redundant.

        The neutral write itself is always issued; only the ``sleep``
        is conditional. That keeps RC override fresh (ArduSub treats
        > 1 s of silence as a pilot dropout and reverts).
        """
        self._writers().neutral()
        skip = (
            self.quick_settle
            and axes is not None
            and self._last_axes == axes
            and self._heading_lock is None
        )
        if skip:
            self._last_axes = axes
            return
        self.log.info('[CMD  ] stop -- stabilising...')
        time.sleep(settle_time)
        self._last_axes = axes

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
