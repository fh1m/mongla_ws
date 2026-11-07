#!/usr/bin/env python3
"""Mongla -- one method per verb.

Every public method is a verb in `commands.COMMANDS` (and so a `Move.Goal.cmd`
value); the action server dispatches by name. Adding a verb is a row in
`commands.py` and a method here.

Two kinds of verb, and where each runs:

  * BOARD verbs -- move_*, yaw_*, turn, set_depth, stop, pause, style_roll.
    One `SROT_MOVE` to the board, which runs and brakes the motion in its own
    500 Hz loop. The host only waits, relays progress, and brakes on cancel.
    `surface` puts the board in its SURFACE mode.
  * HOST verbs -- arm, disarm, set_mode, head, fire, motor_test,
    mission_reset, calibrate_depth, calc_distance, vision_align, vision_move.
    A loop or a single message from here.

Every method returns a `Move.Result`. Every verb runs under `self.lock`, so one
runs at a time; `stop`, `surface` and `disarm` are let through the action
server's busy gate and signal abort first, so a running loop exits and frees
the lock within one tick.
"""

import math
import threading
from contextlib import contextmanager

from mongla_interfaces.action import Move

from .vision_verbs import VisionVerbs
from .fc.base      import FireResult, FIRE_NOT_READY, FIRE_THRUSTER_FAULT
from .errors       import NotArmedError
from .motion_writers import make_writers, _interruptible_sleep


# Verbs that may run while disarmed. Every other verb refuses: the board drops
# motion demands while disarmed, so a motion verb would report success and
# move nothing.
_UNARM_SAFE = frozenset({'stop', 'pause', 'mission_reset', 'calibrate_depth',
                         'calc_distance'})

# Board verbs and the axis each one moves, for calc_distance's projection.
_BOARD_AXIS = {
    'move_forward': 'axial', 'move_back': 'axial',
    'move_left': 'lateral', 'move_right': 'lateral',
    'yaw_left': None, 'yaw_right': None, 'turn': None, 'set_depth': None,
    'stop': None, 'pause': None, 'style_roll': None,
}

# Verbs whose result reports the heading after; every other board verb
# reports the depth after.
_HEADING_VERBS = frozenset({'yaw_left', 'yaw_right', 'turn'})

# Depth readings older than this are not "the depth now". 2.0 s catches a real
# freeze with ~8x margin over 4 Hz jitter and never trips on normal telemetry.
_ATTITUDE_STALE_S = 2.0


def _fresh_depth(fc):
    """Current depth in metres (negative below the surface), or None if stale."""
    attitude = fc.get_attitude()
    if attitude is None:
        return None
    age = fc.get_attitude_age()
    if age is not None and age > _ATTITUDE_STALE_S:
        return None
    return attitude['depth']


class Mongla(VisionVerbs):
    """Serialised verb facade over the board driver.

    fc                    -- the board driver (`SrotFC`)
    log                   -- anything with .info/.warning/.error
    yaw_source            -- `BoardYaw`, or None to read `fc.get_attitude()`
    vision_state_provider -- camera name -> VisionState (injected by the manager
                             so this package stays rclpy-free)
    distance_provider     -- () -> DistanceState, for calc_distance
    payload               -- `SrotPayload`, or None when the payload is disabled
    """

    def __init__(self, fc, log, *, yaw_source=None, vision_state_provider=None,
                 distance_provider=None, payload=None):
        self.fc                    = fc
        self.log                   = log
        # Reentrant, so a verb can call another scoped verb on the same thread.
        # Commands are still one at a time across threads.
        self.lock                  = threading.RLock()
        self.yaw_source            = yaw_source
        self.vision_state_provider = vision_state_provider
        self._distance_provider    = distance_provider
        self._payload              = payload
        # calc_distance's projection axis: set by the LAST move_* verb.
        self._distance_axis        = 'axial'
        # Cooperative abort: set by request_abort(), checked every loop tick,
        # cleared at the start of each verb so each starts fresh.
        self._abort_event          = threading.Event()
        # Live vision telemetry: the align/move loop writes the signed
        # from-centre px here each tick; the feedback pump reads it.
        self._vis_x = math.nan
        self._vis_y = math.nan
        self._vis_stamp = 0.0

    # ================================================================== #
    #  Abort and live telemetry                                           #
    # ================================================================== #

    def report_vision(self, x_px, y_px):
        """Loop callback: record the live signed target-from-centre px."""
        import time
        self._vis_x, self._vis_y = float(x_px), float(y_px)
        self._vis_stamp = time.monotonic()

    def vision_telemetry(self, fresh_s=0.5):
        """Latest ``(x_px, y_px)`` if reported within ``fresh_s``, else None."""
        import time
        if (time.monotonic() - self._vis_stamp) <= fresh_s:
            return (self._vis_x, self._vis_y)
        return None

    def request_abort(self):
        """Make every running loop exit at its next tick (called from the
        action server's cancel path, on another thread)."""
        self._abort_event.set()

    @property
    def _abort_fn(self):
        """The callable loops poll -- True once abort is requested."""
        return self._abort_event.is_set

    # ================================================================== #
    #  Board verbs -- one SROT_MOVE each                                  #
    # ================================================================== #

    def _board(self, cmd, on_progress=None, **kwargs):
        """Run one board verb: the board executes and brakes it.

        `on_progress(fraction)` relays the board's progress (the manager turns
        it into action feedback). The abort hook brakes the board on cancel.
        """
        with self._command_scope(cmd):
            axis = _BOARD_AXIS.get(cmd)
            if axis is not None:
                self._distance_axis = axis
            res = self.fc.move(cmd, on_progress=on_progress,
                               abort_fn=self._abort_fn, **kwargs)
            final = self._current_heading() if cmd in _HEADING_VERBS else None
            return self._make_result(res.ok, res.reason, final_value=final)

    def move_forward(self, duration, gain=80.0, settle=0.0, on_progress=None):
        return self._board('move_forward', on_progress, duration=duration,
                           gain=gain, settle=settle)

    def move_back(self, duration, gain=80.0, settle=0.0, on_progress=None):
        return self._board('move_back', on_progress, duration=duration,
                           gain=gain, settle=settle)

    def move_left(self, duration, gain=80.0, settle=0.0, on_progress=None):
        return self._board('move_left', on_progress, duration=duration,
                           gain=gain, settle=settle)

    def move_right(self, duration, gain=80.0, settle=0.0, on_progress=None):
        return self._board('move_right', on_progress, duration=duration,
                           gain=gain, settle=settle)

    def yaw_left(self, target, timeout=30.0, settle=0.0, on_progress=None):
        return self._board('yaw_left', on_progress, target=target,
                           timeout=timeout, settle=settle)

    def yaw_right(self, target, timeout=30.0, settle=0.0, on_progress=None):
        return self._board('yaw_right', on_progress, target=target,
                           timeout=timeout, settle=settle)

    def turn(self, target, timeout=30.0, settle=0.0, on_progress=None):
        return self._board('turn', on_progress, target=target,
                           timeout=timeout, settle=settle)

    def set_depth(self, target, timeout=30.0, settle=0.0, on_progress=None):
        return self._board('set_depth', on_progress, target=target,
                           timeout=timeout, settle=settle)

    def stop(self, on_progress=None):
        return self._board('stop', on_progress)

    def pause(self, duration=2.0, on_progress=None):
        return self._board('pause', on_progress, duration=duration)

    def style_roll(self, flips=1, timeout=20.0, on_progress=None):
        return self._board('style_roll', on_progress, flips=flips, timeout=timeout)

    def surface(self, timeout=60.0, wait=True):
        """Emergency surface: brake, then the board's SURFACE mode, then wait
        (up to `timeout` s) until the depth reads within SURFACED_M of zero.

        SURFACE is the board's own ascend-and-hold failsafe state -- the depth
        PID to 0 m, or an open-loop ascent when there is no depth sensor -- and
        the ONE mode the board never blocks for a missing barometer. Braked
        first: SURFACE alone does not abort a running movement primitive.

        It WAITS because a caller's next step is usually `disarm`, and a
        surface that returned the instant the mode took left the vehicle at
        depth: the disarm then cut thrust and buoyancy did the rest, slowly.
        `wait=False` is for the watchdog, which must not block its timer.

        Not under `self.lock`, and the abort is LEFT SET: SURFACE *is* the
        abort. The action server signals abort when surface arrives during
        another verb; clearing it here would let a vision loop keep driving
        and a queued shot still fire after the operator said "surface". The
        next verb clears it at its own entry. So the wait does not poll the
        abort flag -- it is bounded by `timeout` alone.
        """
        import time
        self.fc.stop_motion()
        ok, reason = self.fc.set_mode('SURFACE')
        if not ok:
            self.log.error(f'[ACT  ] surface FAILED: {reason}')
            return self._make_result(False, f'surface: could not engage SURFACE -- {reason}')
        if not wait:
            return self._make_result(True, 'surface: SURFACE engaged (not waiting)')
        deadline = time.monotonic() + max(0.0, float(timeout))
        depth = _fresh_depth(self.fc)
        while time.monotonic() < deadline:
            depth = _fresh_depth(self.fc)
            if depth is not None and depth >= -self.SURFACED_M:
                return self._make_result(True, f'surface: at {depth:+.2f} m',
                                         final_value=depth)
            time.sleep(0.2)
        seen = 'depth unreadable' if depth is None else f'still at {depth:+.2f} m'
        self.log.warning(f'[ACT  ] surface: SURFACE engaged but {seen} after '
                         f'{float(timeout):.0f} s')
        # SURFACE is engaged and the board keeps ascending -- but the surface was
        # not SEEN, so this is not a success (safety rule 6: never claim a verb
        # worked without seeing the value it produced). It used to return True
        # here, and a mission's next step -- usually disarm, or "we are up, open
        # the hatch" -- ran on a hull still at depth. Nothing is undone by
        # failing: the board stays in SURFACE. (Cloud audit, 2026-11-07.)
        return self._make_result(
            False, f'surface: SURFACE engaged, but {seen} after '
                   f'{float(timeout):.0f} s -- the board is still ascending',
            final_value=math.nan if depth is None else depth)

    # Depth within this of zero counts as surfaced. UNMEASURED: 0.20 m is a
    # margin for the sensor sitting below the waterline of a floating hull, not
    # a measurement of where it sits. Read the depth on the first surfacing and
    # replace this with that number.
    SURFACED_M = 0.20

    # ================================================================== #
    #  Arm / disarm / mode                                                #
    # ================================================================== #

    def arm(self, timeout=15.0):
        # Clean abort slate: a stale abort from a cancelled verb would make
        # the arm poll abort and disarm this fresh arm.
        self._abort_event.clear()
        accepted, reason = self.fc.arm(timeout, abort=self._abort_fn)
        return self._make_result(accepted, f'arm: {reason}')

    def disarm(self, timeout=20.0):
        """Signals abort first. Without it a `vision_align` kept streaming
        MANUAL_CONTROL into a board that would re-arm under it, and a queued
        shot still left the tube -- the one thing a disarm must prevent."""
        self.request_abort()
        accepted, reason = self.fc.disarm(timeout)
        return self._make_result(accepted, f'disarm: {reason}')

    def set_mode(self, target_name, timeout=8.0):
        accepted, reason = self.fc.set_mode(target_name, timeout)
        return self._make_result(accepted, f'set_mode {target_name}: {reason}')

    # ================================================================== #
    #  Thrusters and payload                                              #
    # ================================================================== #

    def motor_test(self, target: float = 1.0, gain: float = 20.0,
                   duration: float = 2.0):
        """Spin ONE thruster, by index 1..8, at `gain` percent (negative =
        reverse) for `duration` s.

        This turns a thruster. Props off or the vehicle restrained, and a
        human on the kill switch.

        It is the primitive under every per-thruster question: the thrust
        curve, which thruster is dead (an ESC frame count proves nothing --
        958 frames read 0 rpm with nothing attached), and whether a motor turns
        the way the mixer believes.

        The safety mechanism is the board's: the test expires when keep-alives
        stop, and expiry auto-disarms, so a caller that wedges cannot hold a
        motor spinning. `SrotFC.motor_test` blocks and pumps from inside, so
        every exit path stops resending.
        """
        motor = int(target)
        with self._command_scope('motor_test'):
            ok, reason = self.fc.motor_test(motor, float(gain), float(duration),
                                            abort_fn=self._abort_fn)
            return self._make_result(ok, f'motor_test: {reason}',
                                     final_value=float(motor))

    def fire(self, fire_channel: float):
        """Activate BOARD channel `fire_channel` (1..16) -- `DO_SET_SERVO
        param1`, the same n as `SERVO{n}_ROLE`. There is no host-side map:
        whether the channel is a payload switch or the on-board arm is read
        from the board, and a channel it calls PWM/arm is refused.

        The outcome code lands in `final_value`, so a mission can tell "fired"
        from "that channel is the arm" from "the link is down".
        """
        ch = int(fire_channel)
        with self._command_scope('fire'):
            res = self._fire_payload(ch)
        label = self._payload.label(ch) if self._payload is not None else ''
        who = f'ch={ch}' + (f' ({label})' if label else '')
        return self._make_result(res.ok, f'fire: {who} {res.code_name}: {res.reason}',
                                 final_value=float(res.code))

    def _thruster_fault(self):
        """The reason to REFUSE a shot, or None to allow it.

        Refuse only on a known-bad thruster. `thruster_health()` returns None
        -- UNKNOWN -- whenever the board has not announced presence, which is
        every disarmed session. Refusing on UNKNOWN would make `fire()` a silent
        no-op at competition. UNKNOWN is logged loudly and allowed through; a
        torpedo that might miss beats one that never leaves the tube. A
        thruster that reported and then STOPPED, or turns the wrong way, means
        the hull cannot hold a firing solution: that is what this refuses.
        """
        try:
            ok_flag, reason = self.fc.thruster_health()
        except Exception as exc:          # a probe fault is not a hull fault
            self.log.warning(f'[FIRE ] thruster health unreadable ({exc}) -- allowing')
            return None
        if ok_flag is False:
            return reason
        if ok_flag is None:
            self.log.warning(f'[FIRE ] thruster health UNKNOWN -- firing anyway: {reason}')
        return None

    def _fire_payload(self, channel: int) -> FireResult:
        """Raw payload fire, no command scope -- for use inside vision verbs.
        Always a FireResult, so every caller can say WHY a shot did not happen."""
        fault = self._thruster_fault()
        if fault is not None:
            self.log.error(f'[FIRE ] REFUSED ch={channel} -- {fault}')
            return FireResult(FIRE_THRUSTER_FAULT, int(channel), fault)
        if not self.payload_ready:
            self.log.warning(f'[FIRE ] payload not ready ch={channel}')
            return FireResult(FIRE_NOT_READY, int(channel),
                              'payload disabled or link down')
        return self._payload.fire(channel)

    @property
    def payload_ready(self) -> bool:
        return self._payload is not None and self._payload.is_ready

    # ================================================================== #
    #  Reads                                                              #
    # ================================================================== #

    def _current_heading(self):
        """Heading in degrees, or NaN when there is no fresh one.

        Absence is NaN, never 0.0: 0.0 is north, and a mission that branches on
        a heading would act on it.
        """
        if self.yaw_source is not None:
            heading = self.yaw_source.read_yaw()
        else:
            attitude = self.fc.get_attitude()
            heading = attitude['yaw'] if attitude else None
        return math.nan if heading is None else float(heading)

    def head(self):
        """The heading now, in `final_value`. Moves nothing. Fails, with NaN,
        when the board has given no fresh heading."""
        with self._command_scope('head'):
            heading = self._current_heading()
            if math.isnan(heading):
                return self._make_result(False, 'heading unavailable (no fresh attitude)',
                                         final_value=heading)
            return self._make_result(True, f'heading={heading:.1f}°',
                                     final_value=heading, error_value=0.0)

    # ================================================================== #
    #  Mission state, depth zero, distance                                #
    # ================================================================== #

    def mission_reset(self):
        """Clear cross-goal state: the abort flag, any streamed demand, and
        re-zero the barometer if disarmed at the surface. First line of every
        mission."""
        with self._command_scope('mission_reset'):
            self._abort_event.clear()
            self._writers().neutral()
            self.log.info('[CMD  ] mission_reset -- abort cleared, demand neutral')
            # Best-effort: a failed re-zero logs loudly but must never break
            # the reset itself, so it does not raise here.
            try:
                self._run_baro_calibration(settle_s=2.5, strict=False)
            except Exception as exc:   # noqa: BLE001 -- reset must always complete
                self.log.warning(f'[BARO ] mission_reset calibration skipped: {exc!r}')
            return self._make_result(True, 'mission_reset: completed')

    def calibrate_depth(self, settle_s=2.5):
        """Re-zero the barometer at the surface so depth reads 0 before a dive.

        The depth reading drifts +0.01..0.1 m between power-on and dive
        (temperature, weather). Sequence, each step checked: disarmed ->
        fresh pre-depth near the surface -> MAV_CMD_PREFLIGHT_CALIBRATION ->
        settle -> fresh post-depth near 0.
        """
        with self._command_scope('calibrate_depth'):
            return self._run_baro_calibration(settle_s=settle_s, strict=True)

    # Pre-cal depth beyond the surface bound means "not surfaced, or a baro
    # fault" -- normal drift is 0.01..0.1 m -- so it is refused rather than
    # zeroed at a bad reference (set_depth would then drive to the wrong
    # depth). Post-cal must land within the tolerance or the re-zero failed.
    _BARO_SURFACE_BOUND_M = 0.30
    _BARO_ZERO_TOL_M      = 0.05

    def _run_baro_calibration(self, *, settle_s, strict):
        """`strict` fails on any problem (the standalone verb); non-strict logs
        and returns (mission_reset must never break)."""
        # Arm state is the surface proxy. NEVER calibrate a diving hull.
        if self.fc.is_armed():
            self.log.warning('[BARO ] skip depth calibration -- vehicle ARMED '
                             '(calibrate only DISARMED at the surface)')
            return self._make_result(not strict, 'calibrate_depth: skipped (armed)')

        pre = _fresh_depth(self.fc)
        if pre is None:
            self.log.warning('[BARO ] skip -- no fresh depth reading')
            return self._make_result(not strict, 'calibrate_depth: no depth telemetry')
        if abs(pre) > self._BARO_SURFACE_BOUND_M:
            self.log.warning(
                f'[BARO ] REFUSE depth calibration -- pre-cal depth {pre:+.2f}m '
                f'exceeds surface bound {self._BARO_SURFACE_BOUND_M:.2f}m '
                f'(not surfaced or baro fault). NOT re-zeroing.')
            return self._make_result(
                not strict, 'calibrate_depth: not at surface', final_value=pre)

        self.log.info(f'[BARO ] depth re-zero: pre={pre:+.3f}m -- calibrating')
        ok, reason = self.fc.calibrate_barometer()
        if not ok:
            self.log.warning(f'[BARO ] calibration command failed: {reason}')
            return self._make_result(not strict,
                                     f'calibrate_depth: cmd {reason}', final_value=pre)

        _interruptible_sleep(max(0.0, settle_s), self._abort_fn)

        post = _fresh_depth(self.fc)
        if post is None:
            self.log.warning('[BARO ] post-cal depth unavailable -- cannot verify')
            return self._make_result(not strict,
                                     'calibrate_depth: unverified', final_value=pre)
        verified = abs(post) <= self._BARO_ZERO_TOL_M
        tag = 'OK' if verified else 'FAILED (post not ~0)'
        (self.log.info if verified else self.log.warning)(
            f'[BARO ] depth re-zero: pre={pre:+.3f}m -> post={post:+.3f}m  {tag}')
        return self._make_result(verified or not strict,
                                 f'calibrate_depth: {tag}', final_value=post)

    def calc_distance(self, phase='start'):
        """Downward optical-flow distance bracket.

        `phase='start'` latches the projection axis (from the last move_*
        verb) and resets the accumulator; `'stop'` freezes it and returns the
        metres in `final_value`. `stop` reports a FAILURE when the estimator
        is not publishing -- never a phantom 0 m.
        """
        with self._command_scope('calc_distance'):
            prov = self._distance_provider
            state = prov() if callable(prov) else prov
            if state is None:
                return self._make_result(
                    False, 'calc_distance: no distance node (launch flow:=true)',
                    final_value=0.0)
            if str(phase or 'start').strip().lower() == 'start':
                ok, msg = state.start(lateral=(self._distance_axis == 'lateral'))
                self.log.info(f'[DIST ] calc_distance start ({self._distance_axis}): {msg}')
                return self._make_result(bool(ok), f'calc_distance: {msg}', final_value=0.0)
            ok, msg, dist = state.stop()
            self.log.info(f'[DIST ] calc_distance stop: {dist:+.3f}m ({msg})')
            return self._make_result(bool(ok), f'calc_distance: {msg}',
                                     final_value=float(dist))

    # ================================================================== #
    #  Internals                                                          #
    # ================================================================== #

    def _writers(self):
        return make_writers(self.fc)

    @contextmanager
    def _command_scope(self, verb):
        """One verb at a time, a clean abort slate, and the disarmed gate."""
        with self.lock:
            self._abort_event.clear()
            if verb not in _UNARM_SAFE and not self.fc.is_armed():
                raise NotArmedError(f'{verb}: AUV is disarmed -- call arm() first')
            yield

    def _send_neutral_and_settle(self, settle_time=0.6):
        """Zero the demand and let the hull settle (inside a verb)."""
        self._writers().neutral()
        _interruptible_sleep(settle_time, self._abort_fn)

    def _current_depth(self):
        attitude = self.fc.get_attitude()
        return float(attitude['depth']) if attitude else math.nan

    def _make_result(self, success, message, final_value=None, error_value=0.0,
                     end_x_px=math.nan, end_y_px=math.nan,
                     fill_frac=0.0, elapsed_s=0.0):
        """A `Move.Result`; `final_value` defaults to the depth now (NaN when
        there is none -- absence is never 0.0, which is the surface)."""
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
