#!/usr/bin/env python3
"""Continuous heading-lock streamer (depth-hold's yaw cousin).

A ``HeadingLock`` is a daemon thread that runs a proportional
Ch4-rate-override loop at 50 Hz against the heading read from the
configured ``YawSource`` (BNO085, AHRS, etc.). While the lock is
active the vehicle's yaw rate is authored entirely by this loop;
ArduSub's own compass/AHRS does not close the heading loop.

Why this exists
---------------
ArduSub can hold heading itself in ALT_HOLD/POSHOLD/STABILIZE when
Ch4 is at 1500, using its internal compass. Two physical realities
make that unreliable on a competition AUV:

  1. Magnetic interference. Thruster ESCs draw 10A peaks; the field
     they produce dwarfs the geomagnetic vector. ArduSub's compass
     drifts. This is *the* reason we have a BNO085 on the vehicle.
  2. Asymmetric forward thrust. No four-thruster vectored frame is
     perfectly through CG, so every ``move_forward`` injects a small
     yaw moment. Without active correction the body weather-cocks.

How the loop works (same pattern as ``motion_yaw`` and ``motion_vision``)
------------------------------------------------------------------------
Each tick:

  1. Read ``current`` from the yaw_source (or AHRS if no source given).
  2. ``error = Pixhawk.heading_error(target, current)`` in [-180, 180].
  3. Deadband inside ``LOCK_DEADBAND_DEG`` so noise does not twitch
     the sub; otherwise ``yaw_pct = error * LOCK_KP_PCT_PER_DEG``,
     clamped to ``+/-LOCK_PCT_MAX`` so we cannot saturate the bus.
  4. ``pixhawk.send_rc_yaw_only(percent_to_pwm(yaw_pct))``.
     Sets ONLY Ch4; all other channels stay at NO_OVERRIDE (65535).
     This lets concurrent DVL forward/lateral moves run without
     their Ch5/Ch6 thrust being overwritten every lock tick.
     ArduSub treats any Ch4 override != 1500 as a pilot yaw-rate
     command, so the Python-side yaw_source is the sole feedback
     closing the loop.

Source-agnostic by design
-------------------------
Same ``YawSource`` plug the rest of motion uses:

  * ``mavlink_ahrs`` -> heading from Pixhawk's AHRS2 (works in Gazebo
                        SITL, on the bench, when BNO085 isn't wired)
  * ``bno085``       -> external 9-DOF IMU (preferred on the real sub)
  * future Gazebo / DVL / vision yaw -> drop-in YawSource subclass

A pool team can rehearse heading-lock missions in Gazebo with
``mavlink_ahrs``, then switch to ``bno085`` for the run with zero code
changes; in both cases the lock closes on that exact source.

Failure modes
-------------
* Source returns None for a tick (stale BNO sample, parse error)
  -> loop holds and waits. If the source stays silent longer than
     ``SOURCE_DEAD_S`` the lock releases Ch4 (yaw -> 1500, safe stop)
     and logs a WARN so the operator sees it. When samples resume,
     an INFO log records the recovery and the loop closes again.
* Operator forgets to unlock -> ``timeout`` (default 300 s) auto-stops
  the thread with a [LOCK ] log line and releases Ch4.
* Manager process exits -> daemon=True kills the thread; the
  manager's atexit also calls ``pixhawk.send_neutral()``.
"""

import math
import threading
import time

from .motion_writers import read_heading
from .pixhawk       import Pixhawk


# 50 Hz (LOCK_STREAM_HZ) matches the BNO085 firmware output so the rate
# command is refreshed comfortably inside ArduSub's RC timeout.
# Sourced from motion_rates so a single edit retunes every loop.
from .motion_rates import LOCK_STREAM_HZ as STREAM_HZ

DRIFT_LOG_SEC  = 1.0     # how often to print the [LOCK ] heartbeat
SOURCE_DEAD_S  = 0.5     # release Ch4 after this many seconds with no fresh sample

# Rate-loop tunables. Gentle but with a minimum floor so T200s actually
# spin when correcting small drifts (pure proportional at 2° gives ~1.2%
# = ~5 PWM, below thruster spin-up threshold).
LOCK_KP_PCT_PER_DEG = 1.2    # increased from 0.6 for reliable small corrections
LOCK_SPEED_MIN_PCT  = 5.0    # floor: minimum thrust to spin T200 above deadband
LOCK_PCT_MAX        = 22.5   # ceiling: matches yaw_snap max so corrective bursts are consistent
LOCK_DEADBAND_DEG   = 1.0
# Floor taper band. A HARD min-PWM floor on Ch4 (a yaw RATE) is a relay feeding
# ArduSub's yaw integrator: holding a still hull it barely fires (error stays in
# the deadband), but rejecting a CONTINUOUS lateral-thrust yaw moment (a lat-only
# vision_align strafing Ch6 on an off-CG vectored frame) it kicks >=floor every
# tick -> overshoot -> sign flip -> limit-cycle = the align-yaw jitter. Mirror the
# motion_yaw `ab2014f` fix: taper the floor to 0 across an approach band so the
# command decays into the deadband instead of relay-bouncing. Only 1-6deg softens;
# >=band keeps the full stiction-break floor, <=deadband still commands 0. Pure-P
# (no integral) so under a sustained disturbance this settles to a small bounded
# heading offset rather than wobbling -- raise LOCK_KP_PCT_PER_DEG or add a
# LOCK_KI follow-up if that droop is too large (pool-tunable).
LOCK_APPROACH_BAND_DEG = 6.0

# "Fire-window quiet mode" deadband. During a terminal vision fire-lock (a lat/depth
# align that hands Ch4 to this lock and holds while a torpedo fires) the operator
# wants a STEADY launcher heading, not a lock chasing every 1-2deg of BNO / thrust-
# coupling noise -- that micro-correction is the residual left-right wobble. A wider
# deadband makes the lock HOLD and only correct real drift, killing the limit-cycle
# for the fire window. Engaged per-call via HeadingLock.set_hold_mode(True); the
# default deadband (LOCK_DEADBAND_DEG) is untouched otherwise. At a ~0.4 m standoff a
# few deg of hull yaw is a small linear error and the align's lat axis still centres
# the shot. Pool-tunable. Must stay < LOCK_APPROACH_BAND_DEG.
LOCK_HOLD_DEADBAND_DEG = 3.0

# B15: the "must stay <" above was a COMMENT, and nothing enforced it. Raise the
# deadband to or past the band and `span` goes <= 0; the runtime guard below then
# returns the full LOCK_SPEED_MIN_PCT everywhere, which silently reinstates the
# hard min-PWM floor -- i.e. exactly the relay limit-cycle the taper was added to
# remove (the align-yaw jitter). No error, no log, just a hull that hunts again.
# A module-level check makes a bad tune impossible to load rather than subtle to
# fly.
assert 0.0 <= LOCK_HOLD_DEADBAND_DEG < LOCK_APPROACH_BAND_DEG, (
    f'LOCK_HOLD_DEADBAND_DEG ({LOCK_HOLD_DEADBAND_DEG}) must be >= 0 and strictly '
    f'less than LOCK_APPROACH_BAND_DEG ({LOCK_APPROACH_BAND_DEG}) -- otherwise the '
    f'approach taper collapses to a hard floor and the heading lock limit-cycles')


def _lock_floor(abs_error_deg: float,
                deadband_deg: float = LOCK_DEADBAND_DEG) -> float:
    """Stiction-breaking speed floor (%), tapered across the approach band.

    Full ``LOCK_SPEED_MIN_PCT`` at/above ``LOCK_APPROACH_BAND_DEG`` (brisk
    correction, break T200 stiction), then linearly to **0 at the deadband
    edge** so the command can decay and the hull eases back to heading instead
    of being driven across the deadband at a hard floor (the relay limit-cycle).
    Pure / side-effect-free. ``deadband_deg`` is the active deadband (widened in
    fire-window quiet mode). Only meaningful for ``abs_error_deg > deadband_deg``
    (inside the deadband the loop commands 0, never calls this).
    """
    if abs_error_deg >= LOCK_APPROACH_BAND_DEG:
        return LOCK_SPEED_MIN_PCT
    span = LOCK_APPROACH_BAND_DEG - deadband_deg
    if span <= 0.0:
        # Unreachable with the module constants (asserted at import), but a
        # caller may pass its own deadband. Full floor, i.e. no taper.
        return LOCK_SPEED_MIN_PCT
    frac = (abs_error_deg - deadband_deg) / span   # 1.0 at band edge -> 0 at deadband
    return LOCK_SPEED_MIN_PCT * max(0.0, frac)


def _lock_command(error_deg: float,
                  deadband_deg: float = LOCK_DEADBAND_DEG) -> float:
    """Signed Ch4 yaw-rate command (%) for a heading error -- pure, the law.

    0 inside ``deadband_deg`` (don't twitch on noise; widened in quiet mode).
    Outside it, proportional ``LOCK_KP_PCT_PER_DEG`` capped at ``LOCK_PCT_MAX``,
    with the TAPERED stiction floor (``_lock_floor``) underneath so corrections
    actually spin the T200s without the hard-floor relay that limit-cycles the
    hull under a sustained lateral-thrust yaw moment. Sign follows the error.
    """
    if abs(error_deg) <= deadband_deg:
        return 0.0
    mag   = min(LOCK_PCT_MAX, abs(error_deg) * LOCK_KP_PCT_PER_DEG)
    speed = max(_lock_floor(abs(error_deg), deadband_deg), mag)
    return math.copysign(speed, error_deg)


class HeadingLock:
    """Background streamer of Ch4 rate-overrides that holds a fixed yaw.

    Lifecycle:
        lock = HeadingLock(pixhawk, target_deg=90, yaw_source=bno, log=...)
        lock.start()
        ...                       # other commands run; lock corrects drift
        lock.retarget(180)        # yaw_left/yaw_right call this
        ...
        lock.stop()               # joins thread, sends neutral

    Re-targeting is thread-safe and instantaneous -- the next streamed
    packet uses the new value. Useful for `yaw_left`/`yaw_right` which
    intentionally change heading; they call `retarget(new_heading)` on
    exit so the lock follows the most recent commanded heading.
    """

    def __init__(self, pixhawk, target_deg, yaw_source, log,
                 timeout=300.0, on_exit=None):
        self._pixhawk     = pixhawk
        self._yaw_source  = yaw_source
        self._log         = log
        self._timeout     = float(timeout)
        # Called with (self) when the loop auto-releases on timeout (NOT on
        # an explicit stop(), which the caller already cleans up after). Lets
        # the owner clear its handle and resume the heartbeat so a timed-out
        # lock doesn't leave a zombie that still looks "active".
        self._on_exit     = on_exit

        self._target_lock = threading.Lock()
        self._target_deg  = float(target_deg) % 360.0

        self._stop_event  = threading.Event()
        # J01: a COUNTER, not a boolean Event -- reentrant, exactly as
        # `Heartbeat.pause/resume` already is one module over. A boolean makes
        # nesting lossy: an inner resume cancels an outer suspend. That was
        # recorded as "latent, not live" on the reasoning that Duburi.lock
        # serialises verbs so two scoped blocks cannot nest. REPRODUCED as LIVE
        # (2026-09-08), because the collision is not scoped-vs-scoped:
        #
        #   1. `lock_heading` while DISARMED latches a suspend that is
        #      DELIBERATELY unpaired (duburi.py "Suspend BEFORE start so the
        #      daemon never emits a single Ch4 write before the first armed
        #      command -- no thruster kick while surface holders steady the
        #      hull"), and sets _lock_deferred.
        #   2. `pause` is in _LOCK_PASSIVE_VERBS, so the deferred-activation
        #      hook does NOT clear that latch -- but pause's BODY uses the
        #      scoped `_suspend_heading_lock()`.
        #   3. Its `finally` sees is_suspended and calls resume(), CLEARING a
        #      latch it never owned.
        #
        # The lock then streams Ch4 while disarmed -- the exact kick the latch
        # exists to prevent -- with nothing logged. A counter makes step 3
        # decrement to 1 and stay suspended, which is the intended semantics.
        self._suspend_count = 0
        self._suspend_lock  = threading.Lock()
        # Active heading deadband (deg). Widened to LOCK_HOLD_DEADBAND_DEG for a
        # terminal fire-lock via set_hold_mode(True) so the lock holds steady
        # instead of chasing sub-degree noise; restored on set_hold_mode(False).
        self._deadband_deg = LOCK_DEADBAND_DEG
        self._thread      = threading.Thread(
            target=self._run, daemon=True, name='HeadingLock')

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                          #
    # ------------------------------------------------------------------ #

    def start(self):
        source_name = (
            getattr(self._yaw_source, 'name', 'external')
            if self._yaw_source is not None else 'mavlink_ahrs')
        self._log.info(
            f'[LOCK ] start  target={self._target_deg:.1f}deg  '
            f'source={source_name}  timeout={self._timeout:.0f}s')
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        # `join()` on a thread that was never started raises RuntimeError (B17).
        # stop() is reached from abort/cleanup paths, which is exactly where a
        # lock may have been constructed and not started -- a deferred lock whose
        # first armed command never arrived, or an aborted mission_reset. Raising
        # out of cleanup skips the Ch4 release below and whatever the caller
        # meant to do next.
        if self._thread.is_alive() or self._thread.ident is not None:
            self._thread.join(timeout=1.0)
        # Release Ch4 so the yaw channel is not left driving after stop().
        # send_neutral() in the manager's shutdown hook handles this too,
        # but being explicit avoids a brief hang if stop() is called from
        # a mission without going through shutdown.
        self._release_ch4()
        self._log.info('[LOCK ] stopped')

    # ------------------------------------------------------------------ #
    #  Mutators (thread-safe)                                            #
    # ------------------------------------------------------------------ #

    def retarget(self, new_target_deg):
        """Atomically swap the locked target. Streams the new value on
        the next tick (worst case 1 / STREAM_HZ later)."""
        new = float(new_target_deg) % 360.0
        with self._target_lock:
            self._target_deg = new
        self._log.info(f'[LOCK ] retarget -> {new:.1f}deg')

    def suspend(self):
        """Pause streaming (used while yaw_left/yaw_right/arc runs).

        Reentrant: nests correctly with the deferred-lock latch. See the
        counter's rationale where it is declared.
        """
        with self._suspend_lock:
            self._suspend_count += 1

    def resume(self):
        """Undo ONE suspension. Streaming resumes when the last one is undone."""
        with self._suspend_lock:
            if self._suspend_count > 0:          # underflow guard, as Heartbeat
                self._suspend_count -= 1

    def set_hold_mode(self, on: bool):
        """Widen (on) / restore (off) the heading deadband for a fire-window hold.

        In hold mode the lock uses LOCK_HOLD_DEADBAND_DEG so it holds a steady
        launcher heading instead of micro-correcting sub-degree noise (the
        terminal yaw wobble). Single float write -> no lock needed; the 50 Hz
        stream picks it up next tick. Idempotent."""
        self._deadband_deg = LOCK_HOLD_DEADBAND_DEG if on else LOCK_DEADBAND_DEG
        self._log.info(f'[LOCK ] hold-mode {"ON" if on else "off"} '
                       f'(deadband {self._deadband_deg:.1f}deg)')

    @property
    def target_deg(self):
        with self._target_lock:
            return self._target_deg

    @property
    def is_suspended(self) -> bool:
        with self._suspend_lock:
            return self._suspend_count > 0

    # ------------------------------------------------------------------ #
    #  Thread body                                                       #
    # ------------------------------------------------------------------ #

    def _run(self):
        period       = 1.0 / STREAM_HZ
        started_at   = time.monotonic()
        last_log     = 0.0
        last_fresh   = time.monotonic()
        warned_dead  = False

        while not self._stop_event.is_set():
            now = time.monotonic()

            if now - started_at > self._timeout:
                self._release_ch4()
                self._log.warn(
                    f'[LOCK ] timeout {self._timeout:.0f}s reached -- '
                    f'auto-releasing')
                self._fire_on_exit()
                break

            if self.is_suspended:
                self._stop_event.wait(timeout=period)
                continue

            with self._target_lock:
                target = self._target_deg

            current = read_heading(self._pixhawk, self._yaw_source)

            if current is not None:
                last_fresh = now
                if warned_dead:
                    self._log.info('[LOCK ] yaw source recovered')
                    warned_dead = False

                error   = Pixhawk.heading_error(target, current)
                # Tapered-floor P law (see _lock_command/_lock_floor): the floor
                # decays to 0 at the deadband edge so the hull eases in instead
                # of relay-bouncing under a sustained yaw disturbance (the jitter).
                yaw_pct = _lock_command(error, self._deadband_deg)

                try:
                    # send_rc_yaw_only leaves Ch5/Ch6 at NO_OVERRIDE (65535)
                    # so concurrent DVL forward/lateral moves are not
                    # interrupted by this tick.
                    self._pixhawk.send_rc_yaw_only(
                        Pixhawk.percent_to_pwm(yaw_pct))
                except Exception as exc:
                    self._log.warn(f'[LOCK ] send_rc_yaw_only raised: {exc}')

                if now - last_log >= DRIFT_LOG_SEC:
                    self._log.info(
                        f'[LOCK ] tgt:{target:6.1f}  cur:{current:6.1f}  '
                        f'err:{error:+5.1f}  pct:{yaw_pct:+5.1f}')
                    last_log = now

            elif (now - last_fresh) > SOURCE_DEAD_S:
                # Source silent too long -> release Ch4 (safe stop) so the
                # sub does not keep spinning on a stale error reading.
                self._release_ch4()
                if not warned_dead:
                    self._log.warn(
                        f'[LOCK ] yaw source silent for {now-last_fresh:.1f}s '
                        f'-- releasing Ch4')
                    warned_dead = True

            self._stop_event.wait(timeout=period)

    def _release_ch4(self) -> None:
        """Park the yaw channel at 1500 us (zero pilot rate), no other channels."""
        try:
            self._pixhawk.send_rc_yaw_only(Pixhawk.percent_to_pwm(0.0))
        except Exception:
            pass

    def _fire_on_exit(self) -> None:
        """Notify the owner that the lock auto-released (timeout). Never raises."""
        cb = self._on_exit
        if cb is None:
            return
        try:
            cb(self)
        except Exception as exc:   # noqa: BLE001 -- cleanup must never crash the thread
            self._log.warn(f'[LOCK ] on_exit callback raised: {exc}')
