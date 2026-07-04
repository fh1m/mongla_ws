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

import math
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
_UNARM_SAFE = frozenset({'stop', 'pause', 'unlock_heading', 'dvl_connect',
                         'mission_reset', 'lock_heading'})

# Verbs that do NOT engage a deferred heading lock. lock_heading called while
# disarmed captures the heading but holds correction suspended until the first
# *actuating* (control/vision) command runs while armed. Everything not listed
# here (set_depth, move_*, yaw_*, turn, arc, style_*, vision_align/move)
# activates the correction. set_depth is intentionally an activator: the depth
# descent is the operator's first commanded motion.
_LOCK_PASSIVE_VERBS = frozenset({'lock_heading', 'unlock_heading', 'stop',
                                 'pause', 'surface', 'head', 'mission_reset',
                                 'dvl_connect', 'fire'})


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


# style_roll tuning. RoboSub rule: surfacing during a run ends the run, so
# depth is actively guarded during the ACRO roll phase instead of being left
# fully open-loop.
STYLE_ROLL_SURFACE_GUARD_M = -0.15  # AHRS2 depth (negative=below surface);
                                     # abort the flip if shallower than this
STYLE_ROLL_DEPTH_KP        = 150.0  # Ch3 PWM offset per metre of depth error
STYLE_ROLL_DEPTH_CORR_MAX  = 150.0  # clamp on |Ch3 PWM offset|

# |accum| (deg) before a flip's rotation direction is locked for delta-unwrap.
# BNO085/AHRS2 report roll in -180..180; once the per-tick rotation exceeds
# 180 deg the naive shortest-path unwrap aliases (picks the wrong delta sign),
# under-counting accum and requiring several extra physical flips to satisfy
# the target. Locking the rotation direction early lets each subsequent tick
# pick the same-direction candidate (delta, delta-360, delta+360) closest to
# zero, which resolves correctly even for >180 deg/tick steps.
STYLE_ROLL_DIRECTION_LOCK_DEG = 10.0


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
                 quick_settle=False,
                 payload=None):
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
        # Reentrant so a safety verb can call another scoped verb without
        # self-deadlocking: surface() runs inside _command_scope('surface')
        # and then calls set_depth() (its own _command_scope). A plain Lock
        # would block the second acquire on the same thread forever. Commands
        # are still serialized one-at-a-time across threads (RLock only
        # re-admits the thread that already holds it).
        self.lock              = threading.RLock()
        self.smooth_yaw        = smooth_yaw
        self.smooth_translate  = smooth_translate
        self.yaw_source        = yaw_source
        self.vision_state_provider = vision_state_provider
        self._heartbeat        = heartbeat
        self.quick_settle      = bool(quick_settle)
        self._payload          = payload
        self._heading_lock     = None      # HeadingLock thread or None
        # Deferred-activation state: lock_heading called while disarmed
        # captures the heading immediately but suspends the thread (no Ch4
        # correction) until the first armed actuating command resumes it.
        self._lock_deferred    = False
        # Heartbeat-pause REF COUNT (was a bool -> CTRL-6 bug: with a heading lock
        # already holding the pause, style_roll's own hold/release would resume the
        # heartbeat on its release while the lock still needed it paused, letting the
        # 5 Hz neutral writer race the lock's 50 Hz Ch4). A counter pauses on the
        # first holder and resumes only when the LAST holder releases; clamped at 0
        # so the deferred-lock path (release without a matching hold) can't underflow.
        self._heartbeat_hold_count = 0
        # Tracks which channel set the last in-command write touched, so
        # the pre-flight pause can be skipped when the next command uses
        # the same axes. None = no recent write / lock state changed.
        self._last_axes        = None
        # Cooperative abort: set by request_abort(), checked every loop tick.
        # Cleared at the START of each command scope so each command starts fresh.
        self._abort_event      = threading.Event()

        # Live vision telemetry slot: the active align/move loop writes the
        # signed from-centre target px here every tick (report_vision); the
        # manager's feedback pump reads it (vision_telemetry) to stream live
        # per-tick error. (x, y, monotonic stamp). ponytail: plain float writes
        # are GIL-atomic and this is telemetry, not control -- no lock needed.
        self._vis_x = math.nan
        self._vis_y = math.nan
        self._vis_stamp = 0.0

    def report_vision(self, x_px, y_px):
        """Loop callback: record the live signed target-from-centre px + stamp."""
        self._vis_x = float(x_px)
        self._vis_y = float(y_px)
        self._vis_stamp = time.monotonic()

    def vision_telemetry(self, fresh_s=0.5):
        """Latest ``(x_px, y_px)`` if reported within ``fresh_s``, else None.

        The feedback pump calls this each publish; None (no recent vision tick,
        or a non-vision command) maps to NaN feedback.
        """
        if (time.monotonic() - self._vis_stamp) <= fresh_s:
            return (self._vis_x, self._vis_y)
        return None

    def request_abort(self):
        """Signal all running motion loops to exit at their next tick.

        Called from ``auv_manager_node.cancel_callback`` (a different thread)
        when the operator cancels the active action goal. Each loop checks
        ``abort_fn()`` once per tick; on True it breaks cleanly and returns,
        which releases ``self.lock`` so queued safety commands (disarm, stop)
        can proceed immediately.
        """
        self._abort_event.set()

    @property
    def _abort_fn(self):
        """Callable returned to motion loops -- True when abort requested."""
        return self._abort_event.is_set

    # ================================================================== #
    #  Arm / Disarm / Mode -- ACK-bearing, no axis movement              #
    # ================================================================== #

    def arm(self, timeout=15.0):
        """impl: pixhawk.py:arm (COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM, p1=1)."""
        with command_scope('arm'):
            accepted, reason = self.pixhawk.arm(timeout)
        return self._make_result(accepted, f'arm: {reason}')

    def disarm(self, timeout=20.0):
        """impl: pixhawk.py:disarm (set_mode MANUAL -> send_neutral -> COMMAND_LONG p1=0).

        Stops any active heading lock first: its daemon streams Ch4 yaw-rate
        overrides, which must not keep firing after the vehicle drops to
        MANUAL (and leaving the lock handle set would keep the heartbeat
        paused). Mirrors the cleanup in unlock_heading / mission_reset.
        """
        with command_scope('disarm'):
            if self._heading_lock is not None:
                self._heading_lock.stop()
                self._heading_lock = None
                self._lock_deferred = False
                self._release_heartbeat_for_lock()
            accepted, reason = self.pixhawk.disarm(timeout)
        return self._make_result(accepted, f'disarm: {reason}')

    def set_mode(self, target_name, timeout=8.0):
        """impl: pixhawk.py:set_mode (legacy SET_MODE retried until heartbeat reflects)."""
        with command_scope('set_mode'):
            accepted, reason = self.pixhawk.set_mode(target_name, timeout)
        return self._make_result(
            accepted, f'set_mode {target_name}: {reason}')

    # ================================================================== #
    #  Payload actuation                                                  #
    # ================================================================== #

    def fire(self, fire_channel: float):
        """Fire ESP32 payload channel (1/2 = torpedo, 3/4 = dropper).

        ``fire_channel`` is float from Move.Goal (0.0 = unset/stub).
        Returns a command result so the generic COMMANDS dispatcher works.

        Also callable internally as ``self._fire_payload(channel)`` for a
        mission's "align then fire" pattern (no command scope needed since
        it runs inside an already-scoped motion verb).
        """
        ch = int(fire_channel)
        with self._command_scope('fire'):
            ok = self._fire_payload(ch)
        _name = {1: 'torpedo_1', 2: 'torpedo_2', 3: 'dropper_1', 4: 'dropper_2'}.get(ch, '?')
        return self._make_result(ok, f'fire: ch={ch} ({_name}) {"FIRED" if ok else "stub/fail"}')

    def _fire_payload(self, channel: int) -> bool:
        """Raw payload fire — no command scope. Use inside vision verbs."""
        if self._payload is None or not self._payload.is_ready:
            self.log.warning(f'[FIRE ] payload not ready ch={channel} -- stub only')
            return False
        return self._payload.fire(channel)

    @property
    def payload_ready(self) -> bool:
        """True when the ESP32 payload board is connected and port is open."""
        return self._payload is not None and self._payload.is_ready

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

    def surface(self):
        """Emergency surface: ascend to 0 m and hold.

        Sets depth setpoint to 0 m and waits up to 60 s for the AUV to
        reach the surface.  The COMMANDS registry lists it as a safety
        verb so goal_callback bypasses the command_active gate — it can
        run even while another mission command is executing.
        """
        with self._command_scope('surface'):
            self.log.info('[CMD  ] surface -- ascending to 0 m')
            return self.set_depth(0.0, timeout=60.0)

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
            # Heading lock stays ACTIVE during timed forward/back moves.
            # _writers() already releases Ch4 when lock is running so the
            # HeadingLock thread remains the sole Ch4 author.
            run(self.pixhawk, signed_dir, duration, int(gain), self.log,
                self._writers(), yaw_source=self.yaw_source, settle=settle,
                abort_fn=self._abort_fn)
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
            # Heading lock stays ACTIVE during timed lateral moves.
            # _writers() already releases Ch4 when lock is running.
            run(self.pixhawk, signed_dir, duration, int(gain), self.log,
                self._writers(), yaw_source=self.yaw_source, settle=settle,
                abort_fn=self._abort_fn)
            depth = self._current_depth()
            return self._make_result(
                True, f'move_{label}: completed',
                final_value=depth, error_value=0.0)

    # ================================================================== #
    #  arc -- forward thrust + yaw rate at the same time                  #
    # ================================================================== #

    def arc(self, duration, gain=50.0, target_yaw=0.0, settle=0.0):
        """Curved motion to an ABSOLUTE heading: Ch5 forward + Ch4 heading-loop.

        Drives forward at `gain`% for `duration` s while a _YawPID closes Ch4 to
        reach and hold `target_yaw` (absolute degrees) -- the hull curves onto
        the heading then straightens. The turn direction is auto-computed; there
        is no yaw-rate stick to set.

        Heading-lock is incompatible by design (`arc` changes heading).
        Auto-suspends the lock during the arc; on exit, retargets the lock to
        the ACTUAL measured heading (not `target_yaw`, so a partial arc doesn't
        yank the resumed lock toward a heading the hull isn't at) and resumes.

        impl: motion_forward.arc -> pixhawk.send_rc_override (Ch5+Ch4 same packet).
        """
        with self._command_scope('arc'):
            self._send_neutral_and_settle(axes=frozenset({'forward', 'yaw'}))
            self._ensure_yaw_capable_mode()
            self.log.info(
                f'[CMD  ] arc  {duration:.1f}s  gain={gain:.0f}%  '
                f'-> {target_yaw:.0f}deg  settle={settle:.1f}s')
            with self._suspend_heading_lock():
                signed_dir = +1 if gain >= 0 else -1
                motion_arc(self.pixhawk, signed_dir, duration, abs(int(gain)),
                           float(target_yaw), self.log,
                           yaw_source=self.yaw_source, settle=settle,
                           abort_fn=self._abort_fn)
            new_heading = self._current_heading()
            self._retarget_heading_lock(new_heading)
            return self._make_result(
                True, 'arc: completed',
                final_value=new_heading, error_value=0.0)

    # ================================================================== #
    #  Style maneuvers  — 360° rotation on roll, pitch, or yaw axis    #
    # ================================================================== #

    def style_roll(self, gain=60.0, timeout=20.0, flips=1, headroom=1.0):
        """Style: N × 360° roll (Ch2 axis) in ACRO mode, BNO-confirmed.

        Per-flip loop:
          1. Zero ACRO_BAL_ROLL + ACRO_TRAINER so ACRO doesn't auto-level.
          2. Capture origin depth (depth at launch) once — every flip
             returns to this depth, not the headroom pre-dive depth.
          3. For each flip:
             - Optional pre-dive by `headroom` m (ALT_HOLD).
             - Enter ACRO, drive Ch2 until BNO (or AHRS2) accumulates one
               full 360° (direction-locked unwrap — see
               STYLE_ROLL_DIRECTION_LOCK_DEG) or `timeout` elapses. A hard
               surface guard aborts immediately if depth would breach
               STYLE_ROLL_SURFACE_GUARD_M; a cos(roll)-modulated Ch3
               correction tries to hold the origin depth throughout.
             - Return to ALT_HOLD, recover origin depth, continue.
          4. Restore ACRO params, resume heading lock.

        Ctrl-C (goal cancel) mid-flip: ALT_HOLD is restored and the AUV
        disarms immediately — safer than leaving it armed unattended.

        impl: pixhawk.send_rc_override(roll=pwm, throttle=corr) in ACRO — Ch2/Ch3.
        """
        import math as _math
        import time as _t
        with self._command_scope('style_roll'):
            self._ensure_yaw_capable_mode()
            att          = self.pixhawk.get_attitude()
            origin_depth = att['depth'] if att else -0.5

            bno        = self.yaw_source
            use_bno    = bno is not None and hasattr(bno, 'read_roll')
            n_flips    = max(1, int(flips))

            self.log.info(
                f'[CMD  ] style_roll  gain={gain:.0f}%  timeout={timeout:.1f}s/flip'
                f'  flips={n_flips}  headroom={headroom:.1f}m'
                f'  origin_depth={origin_depth:.2f}m  src={"bno" if use_bno else "ahrs2"}')

            orig_bal = self.pixhawk.get_param('ACRO_BAL_ROLL') or 1.0
            orig_trn = self.pixhawk.get_param('ACRO_TRAINER')  or 2.0
            if not (self.pixhawk.set_param('ACRO_BAL_ROLL', 0.0) and
                    self.pixhawk.set_param('ACRO_TRAINER',  0.0)):
                self.log.warn('[CMD  ] style_roll: PARAM_SET timeout — aborting')
                return self._make_result(False, 'style_roll: param set failed')

            pwm         = Pixhawk.percent_to_pwm(gain)
            flip_accums = []
            surfaced    = False

            # Pause heartbeat (ref-counted; safe even if lock already holds it).
            # Spans every flip — without this the 5 Hz neutral writer clobbers
            # Ch2/Ch3 every 200 ms.
            self._hold_heartbeat_for_lock()
            try:
                with self._suspend_heading_lock():
                    for flip_num in range(1, n_flips + 1):
                        if headroom > 0.0:
                            dive_depth = origin_depth - headroom
                            self.log.info(
                                f'[CMD  ] style_roll: flip {flip_num}/{n_flips}'
                                f' pre-diving to {dive_depth:.2f}m')
                            hold_depth(self.pixhawk, dive_depth, 15.0, self.log,
                                       neutral_writer=self._writers().neutral,
                                       abort_fn=self._abort_fn)

                        accum     = 0.0
                        direction = 0   # +1/-1 once |accum| >= STYLE_ROLL_DIRECTION_LOCK_DEG
                        try:
                            self.pixhawk.set_mode('ACRO')
                            _t.sleep(0.2)

                            att = self.pixhawk.get_attitude()
                            if use_bno:
                                last_roll = bno.read_roll()  # type: ignore[union-attr]
                            else:
                                last_roll = att.get('roll', 0.0) if att else 0.0
                            if last_roll is None:
                                last_roll = 0.0
                            last_depth = att['depth'] if att else origin_depth

                            deadline = _t.monotonic() + timeout
                            tick     = 0
                            while abs(accum) < 360.0 and _t.monotonic() < deadline:
                                if self._abort_fn():
                                    break

                                att = self.pixhawk.get_attitude()
                                if att:
                                    last_depth = att['depth']

                                if last_depth > STYLE_ROLL_SURFACE_GUARD_M:
                                    self.log.warn(
                                        f'[CMD  ] style_roll: SURFACE GUARD —'
                                        f' flip {flip_num}/{n_flips}'
                                        f' depth={last_depth:.2f}m — aborting')
                                    surfaced = True
                                    break

                                depth_err = origin_depth - last_depth
                                corr = depth_err * STYLE_ROLL_DEPTH_KP * _math.cos(_math.radians(last_roll))
                                corr = max(-STYLE_ROLL_DEPTH_CORR_MAX,
                                           min(STYLE_ROLL_DEPTH_CORR_MAX, corr))
                                throttle_pwm = int(1500 + corr)

                                # Stream RC every tick — ACRO rate command must be
                                # continuously refreshed; one-shot send is not reliable.
                                self.pixhawk.send_rc_override(roll=pwm, throttle=throttle_pwm)
                                _t.sleep(0.05)    # 20 Hz

                                if use_bno:
                                    cur = bno.read_roll()  # type: ignore[union-attr]
                                else:
                                    cur = att.get('roll', 0.0) if att else None
                                if cur is None:
                                    tick += 1
                                    continue      # stale frame — don't corrupt accum

                                delta = cur - last_roll
                                if delta != 0.0:
                                    if direction == 0:
                                        if delta >  180.0: delta -= 360.0
                                        if delta < -180.0: delta += 360.0
                                    else:
                                        candidates = (delta, delta - 360.0, delta + 360.0)
                                        same_dir = [d for d in candidates if d * direction > 0]
                                        if same_dir:
                                            delta = min(same_dir, key=abs)
                                    accum += delta
                                    if direction == 0 and abs(accum) >= STYLE_ROLL_DIRECTION_LOCK_DEG:
                                        direction = 1 if accum > 0 else -1
                                last_roll = cur

                                tick += 1
                                if tick % 10 == 0:
                                    self.log.info(
                                        f'[CMD  ] style_roll: flip={flip_num}/{n_flips}'
                                        f' roll={cur:+.1f}° accum={accum:+.1f}°'
                                        f' depth={last_depth:.2f}m corr={corr:+.0f}')

                            self.pixhawk.send_rc_override(roll=1500, throttle=1500)
                            self.pixhawk.send_neutral()
                        finally:
                            self.pixhawk.set_mode('ALT_HOLD')

                        flip_accums.append(accum)

                        if not self._abort_fn():
                            hold_depth(self.pixhawk, origin_depth, 15.0, self.log,
                                       neutral_writer=self._writers().neutral,
                                       abort_fn=self._abort_fn)

                        if surfaced or self._abort_fn():
                            break
                # Heading lock resumes here in ALT_HOLD.
                new_heading = self._current_heading()
                self._retarget_heading_lock(new_heading)
            finally:
                # Resume heartbeat FIRST so the 5 Hz neutral RC stream covers
                # the blocking param-restore calls below. FS_PILOT_INPUT cannot
                # fire while heartbeat is active.
                self._release_heartbeat_for_lock()
                self.pixhawk.set_param('ACRO_BAL_ROLL', orig_bal)
                self.pixhawk.set_param('ACRO_TRAINER',  orig_trn)

            completed_flips = sum(1 for a in flip_accums if abs(a) >= 359.0)
            total_accum     = sum(flip_accums)

            if self._abort_fn():
                self.pixhawk.disarm()
                return self._make_result(
                    False,
                    f'style_roll: cancelled after {completed_flips}/{n_flips} flip(s)'
                    f' — ALT_HOLD restored, disarmed',
                    final_value=total_accum)
            if surfaced:
                return self._make_result(
                    False,
                    f'style_roll: ABORTED — surface guard tripped after'
                    f' {completed_flips}/{n_flips} flip(s)',
                    final_value=total_accum)
            return self._make_result(
                True,
                f'style_roll: done {completed_flips}/{n_flips} flip(s)'
                f' ({total_accum:+.0f}°) src={"bno" if use_bno else "ahrs2"}',
                final_value=total_accum)

    def style_yaw(self, flips=1, deg_per_step=90.0, settle=1.0):
        """Style: N×360° yaw spin in ALT_HOLD.

        Depth, roll, and pitch are held automatically by ArduSub (ALT_HOLD) —
        no mode change or pre-dive required. Each flip is split into
        (360/deg_per_step) yaw snaps with settle between.

        impl: steps × yaw_snap/yaw_glide inside a single _suspend_heading_lock.
        """
        n_flips  = max(1, int(flips))
        step_deg = max(1.0, abs(float(deg_per_step)))
        steps    = n_flips * max(1, int(round(360.0 / step_deg)))
        with self._command_scope('style_yaw'):
            self._ensure_yaw_capable_mode()
            self.log.info(
                f'[CMD  ] style_yaw  {n_flips} flip(s)  '
                f'{steps}×{step_deg:.0f}°  settle={settle:.1f}s')
            run_yaw = yaw_glide if self.smooth_yaw else yaw_snap
            abort   = self._abort_fn
            completed = 0
            with self._suspend_heading_lock():
                for completed in range(1, steps + 1):
                    if abort():
                        completed -= 1
                        break
                    start  = self._current_heading()
                    target = (start + step_deg) % 360.0
                    run_yaw(self.pixhawk, start, target, 30.0, 'STYLE_YAW',
                            self.log, yaw_source=self.yaw_source,
                            abort_fn=self._abort_fn)
                    inter_settle = settle if completed < steps else 0.3
                    self._send_neutral_and_settle(settle_time=0.3 + inter_settle)
            new_heading = self._current_heading()
            self._retarget_heading_lock(new_heading)
            return self._make_result(
                True,
                f'style_yaw: done  {completed}/{steps} steps  {n_flips} flip(s)',
                final_value=new_heading)

    # ================================================================== #
    #  Yaw  -- sharp pivots                                              #
    # ================================================================== #

    def yaw_left(self, target, timeout=30.0, settle=0.0):
        """impl: motion_yaw.yaw_snap/yaw_glide -> pixhawk.send_rc_override (Ch4 rate)."""
        return self._turn(-abs(target), timeout, 'LEFT', settle)

    def yaw_right(self, target, timeout=30.0, settle=0.0):
        """impl: motion_yaw.yaw_snap/yaw_glide -> pixhawk.send_rc_override (Ch4 rate)."""
        return self._turn(+abs(target), timeout, 'RIGHT', settle)

    def turn(self, target, timeout=30.0, settle=0.0):
        """Rotate to absolute heading `target` degrees (0-360) via shortest arc.

        Direction (left/right) is determined automatically from the signed
        heading error. Uses the same yaw_snap/yaw_glide loop as yaw_left/right.

        impl: motion_yaw.yaw_snap/yaw_glide -> pixhawk.send_rc_override (Ch4 rate).
        """
        with self._command_scope('turn'):
            self._send_neutral_and_settle(axes=frozenset({'yaw'}))
            self._ensure_yaw_capable_mode()
            target_heading = float(target) % 360.0
            start_heading  = self._current_heading()
            delta          = Pixhawk.heading_error(target_heading, start_heading)
            label          = 'RIGHT' if delta >= 0 else 'LEFT'
            run_yaw = yaw_glide if self.smooth_yaw else yaw_snap
            self.log.info(
                f'[CMD  ] turn -> {target_heading:.1f}°  '
                f'from {start_heading:.1f}°  delta={delta:+.1f}°  ({label})')
            with self._suspend_heading_lock():
                run_yaw(self.pixhawk, start_heading, target_heading,
                        timeout, label, self.log,
                        yaw_source=self.yaw_source,
                        abort_fn=self._abort_fn)
                self._send_neutral_and_settle(settle_time=0.3 + settle)
            final_heading = self._current_heading()
            self._retarget_heading_lock(final_heading)
            error = Pixhawk.heading_error(target_heading, final_heading)
            return self._make_result(
                True, 'turn: completed',
                final_value=final_heading, error_value=float(error))

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
                        yaw_source=self.yaw_source,
                        abort_fn=self._abort_fn)
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

    def head(self):
        """Return the most-recent heading reading without moving anything.

        Use in missions or CLI to capture the exact heading at execution
        time. Value lands in Move.Result.final_value.
        """
        with self._command_scope('head'):
            heading = self._current_heading()
            return self._make_result(True, f'heading={heading:.1f}°',
                                     final_value=heading, error_value=0.0)

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
                       neutral_writer=self._writers().neutral,
                       abort_fn=self._abort_fn)
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

        Heading lock NEVER changes flight mode: the Ch4 rate-override is
        honoured in whatever mode the operator has selected, and depth hold
        is the operator's responsibility (``set_depth`` engages ALT_HOLD).

        Deferred activation: when called while DISARMED the heading is
        captured now but correction is held suspended (no Ch4, no thruster
        kick) until the first armed actuating command resumes it -- so
        surface holders aren't fought during arm + descent. Called while
        ARMED it activates immediately (legacy in-mission behaviour).

        impl: heading_lock.HeadingLock daemon -> pixhawk.send_rc_override (Ch4 rate, 50 Hz LOCK_STREAM_HZ).
        """
        with self._command_scope('lock_heading'):
            current = self._current_heading()
            actual_target = current if abs(target) < 1e-3 else float(target) % 360.0

            if self._heading_lock is not None:
                self._heading_lock.stop()
                self._heading_lock = None
                self._lock_deferred = False
                self._release_heartbeat_for_lock()

            self._heading_lock = HeadingLock(
                pixhawk=self.pixhawk,
                target_deg=actual_target,
                yaw_source=self.yaw_source,
                log=self.log,
                timeout=timeout,
                on_exit=self._on_lock_timeout,
            )

            armed = self.pixhawk.is_armed()
            if not armed:
                # Suspend BEFORE start so the daemon never emits a single Ch4
                # write before the first armed command -- no thruster kick
                # while surface holders steady the hull.
                self._heading_lock.suspend()
            self._heading_lock.start()

            if armed:
                # Active immediately: the lock authors Ch4, so hold the
                # neutral heartbeat for its lifetime.
                self._hold_heartbeat_for_lock()
                self._lock_deferred = False
                lock_msg = f'lock_heading: locked at {actual_target:.1f} deg'
            else:
                # Capture now, correct later. Keep the neutral heartbeat
                # streaming (1500) so once armed there's no FS_PILOT_INPUT
                # failsafe; do NOT hold it for the (suspended) lock yet.
                self._lock_deferred = True
                self.log.info(
                    f'[LOCK ] captured {actual_target:.1f} deg -- correction '
                    f'deferred until armed + first command')
                lock_msg = (f'lock_heading: captured {actual_target:.1f} deg '
                            f'(deferred until armed + first command)')

            # Signal BNO085 to log current heading to OLED for post-run deviation audit.
            if self.yaw_source is not None and hasattr(self.yaw_source, 'send_command'):
                self.yaw_source.send_command('L\n')  # type: ignore[union-attr]

            return self._make_result(
                True,
                f'{lock_msg} (timeout {timeout:.0f}s)',
                final_value=actual_target, error_value=0.0)

    def mission_reset(self):
        """Clear all cross-goal state before a new mission starts.

        Stops heading lock, clears a stale abort event, and sends RC
        neutral.  Call this as the FIRST line of every mission run() so
        state from a previous run (headed to wrong heading, lingering
        abort flag) does not carry forward.

        impl: unlock_heading (direct, not via _command_scope) + abort clear
              + neutral RC.
        """
        with self._command_scope('mission_reset'):
            if self._heading_lock is not None:
                self._heading_lock.stop()
                self._heading_lock = None
                self._lock_deferred = False
                self._release_heartbeat_for_lock()
            self._abort_event.clear()
            # Forget the previous run's axis history so the first command's
            # pre-flight settle pause is never skipped on stale state.
            self._last_axes = None
            self._writers().neutral()
            self.log.info('[CMD  ] mission_reset — heading lock stopped, abort cleared, RC neutral')
            return self._make_result(True, 'mission_reset: completed')

    def unlock_heading(self):
        """Stop the heading-lock streamer and send neutral.

        impl: HeadingLock.stop -> pixhawk.send_neutral.
        """
        with self._command_scope('unlock_heading'):
            if self._heading_lock is None:
                return self._make_result(True, 'unlock_heading: no-op')
            self._heading_lock.stop()
            self._heading_lock = None
            self._lock_deferred = False
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

    def move_back_dist(self, distance_m, gain=60.0, dvl_tolerance=0.1,
                       settle=0.0):
        """Drive backward `distance_m` metres using DVL position feedback.

        Mirrors move_forward_dist with signed_dir=-1; DVL closed-loop keeps
        the heading lock active on Ch4 while Ch5 is driven in reverse.
        Falls back to open-loop timed estimate if DVL not available.
        """
        return self._drive_forward_dist(-1, distance_m, gain, dvl_tolerance, settle)

    def _drive_forward_dist(self, signed_dir, distance_m, gain,
                            dvl_tolerance, settle):
        verb = 'move_forward_dist' if signed_dir > 0 else 'move_back_dist'
        with self._command_scope(verb):
            self._send_neutral_and_settle(axes=frozenset({'forward'}))
            self.log.info(
                f'[CMD  ] {verb}  {distance_m:.2f}m  '
                f'gain={gain:.0f}%  tol={dvl_tolerance:.3f}m  settle={settle:.1f}s')
            # Heading lock stays ACTIVE during DVL distance moves.
            # The lock owns Ch4 (yaw rate); DVL drives Ch5 (forward) only.
            # _writers() already releases Ch4 when lock is live.
            drive_forward_dist(
                self.pixhawk, signed_dir, distance_m, int(gain),
                dvl_tolerance, self.log, self._writers(),
                yaw_source=self.yaw_source, settle=settle,
                abort_fn=self._abort_fn)
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
            # Heading lock stays ACTIVE during DVL lateral moves (owns Ch4 only).
            drive_lateral_dist(
                self.pixhawk, signed_dir, distance_m, int(gain),
                dvl_tolerance, self.log, self._writers(),
                yaw_source=self.yaw_source, settle=settle,
                abort_fn=self._abort_fn)
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

    def _set_lock_hold(self, on):
        """Widen/restore the heading-lock deadband for a fire-window hold.

        Used by vision_align(hold_heading=True) so a terminal fire-lock holds a
        steady launcher heading instead of micro-correcting sub-deg noise. Safe
        no-op when no lock is active/live (local ref -- another thread may null
        self._heading_lock)."""
        lock = self._heading_lock
        if lock is not None and not lock.is_suspended:
            lock.set_hold_mode(bool(on))

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
        ``'vision_align'``, ...) and shows up verbatim in the
        ``[MAV <fn> cmd=<verb>] ...`` line.
        """
        with self.lock, command_scope(verb):
            self._abort_event.clear()   # each command starts with a clean abort slate
            if verb not in _UNARM_SAFE and not self.pixhawk.is_armed():
                raise NotArmedError(
                    f'{verb}: AUV is disarmed -- call arm() first')
            # Engage a deferred heading lock on the first armed actuating
            # command. Runs before the body builds its (now lock-aware)
            # writers, so Ch4 is released to the lock for this command too.
            # Capture the handle in a local: disarm()/_on_lock_timeout() can
            # null _heading_lock from another thread (they bypass self.lock),
            # so guard on the local and undo our heartbeat hold if the lock
            # vanished mid-activation -- never strand a hold on a dead lock.
            lock = self._heading_lock
            if (self._lock_deferred and lock is not None
                    and verb not in _LOCK_PASSIVE_VERBS
                    and self.pixhawk.is_armed()):
                self._lock_deferred = False
                lock.resume()
                self._hold_heartbeat_for_lock()
                if self._heading_lock is None:
                    self._release_heartbeat_for_lock()   # disarm raced us
                else:
                    self.log.info(
                        '[LOCK ] heading correction engaged (first armed command)')
            if self._heartbeat is not None:
                self._heartbeat.pause()
            try:
                yield
            finally:
                if self._heartbeat is not None:
                    self._heartbeat.resume()

    def _hold_heartbeat_for_lock(self):
        """Take a heartbeat-pause hold (ref-counted).

        Pauses the heartbeat on the FIRST holder (count 0 -> 1); further holders
        (e.g. style_roll while a heading lock already holds it) just bump the
        count so the pause spans the union of their lifetimes.
        """
        if self._heartbeat is not None:
            if self._heartbeat_hold_count == 0:
                self._heartbeat.pause()
            self._heartbeat_hold_count += 1

    def _release_heartbeat_for_lock(self):
        """Drop one heartbeat-pause hold; resume only when the LAST holder leaves.

        Clamped at 0 so the deferred path (a lock captured but never activated,
        which still calls release at teardown without a matching hold) is a safe
        no-op instead of underflowing / prematurely resuming.
        """
        if self._heartbeat is not None and self._heartbeat_hold_count > 0:
            self._heartbeat_hold_count -= 1
            if self._heartbeat_hold_count == 0:
                self._heartbeat.resume()

    def _on_lock_timeout(self, lock):
        """Heading lock auto-released on its own timeout (runs in the lock thread).

        Clear our handle and resume the heartbeat so a timed-out lock isn't
        left looking "active" (which would keep the heartbeat paused and make
        translation verbs release Ch4 to a thread that no longer exists). The
        identity guard means a stale callback can't clobber a freshly engaged
        lock that already replaced the timed-out one.
        """
        if self._heading_lock is lock:
            self._heading_lock = None
            self._lock_deferred = False
            self._release_heartbeat_for_lock()

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

    def _make_result(self, success, message, final_value=None, error_value=0.0,
                     end_x_px=math.nan, end_y_px=math.nan,
                     fill_frac=0.0, elapsed_s=0.0):
        """Build a `Move.Result`, defaulting `final_value` to current depth.

        ``end_x_px``/``end_y_px``/``fill_frac``/``elapsed_s`` are the vision
        end-state fields; non-vision callers leave the NaN/0 defaults.
        """
        result = Move.Result()
        result.success     = bool(success)
        result.message     = str(message)
        result.final_value = float(
            final_value if final_value is not None else self._current_depth())
        result.error_value = float(error_value)
        result.end_x_px    = float(end_x_px)
        result.end_y_px    = float(end_y_px)
        result.fill_frac   = float(fill_frac)
        result.elapsed_s   = float(elapsed_s)
        return result
