#!/usr/bin/env python3
"""Continuous heading-lock streamer (depth-hold's yaw cousin).

A `HeadingLock` is a daemon thread that streams `SET_ATTITUDE_TARGET`
at 20 Hz with a fixed yaw target while reading the current heading
from any `YawSource` (or AHRS as fallback) for drift logging.

Why this exists
---------------
ArduSub already holds heading in ALT_HOLD/POSHOLD/STABILIZE when Ch4
RC is at 1500 -- using its internal compass + AHRS yaw. Two physical
realities make that unreliable on a competition AUV:

  1. Magnetic interference.  Thruster ESCs draw 10A peaks; the field
     they produce dwarfs the geomagnetic vector. ArduSub's compass
     drifts.  This is *the* reason we have a BNO085 on the vehicle.
  2. Asymmetric forward thrust.  No four-thruster vectored frame is
     perfectly through CG, so every `move_forward` injects a small
     yaw moment. Without active correction the body weather-cocks.

`lock_heading` puts our chosen yaw_source in charge: the lock thread
streams `SET_ATTITUDE_TARGET` continuously, ArduSub's onboard 400 Hz
attitude stabiliser drives to the target, and our motion commands
switch from `send_rc_override` to `send_rc_translation` so Ch4
stays released and the attitude target keeps authority.

Source-agnostic by design
-------------------------
The yaw_source plug is the same one motion_yaw uses:

  * `mavlink_ahrs`  -> heading from Pixhawk's AHRS2 (works in Gazebo
                       SITL, on the bench, when BNO085 isn't wired)
  * `bno085`        -> external 9-DOF IMU (preferred on the real sub)
  * future Gazebo / DVL / vision yaw -> drop-in YawSource subclass

This means a pool team can rehearse heading-lock missions in Gazebo
with `mavlink_ahrs`, then switch to `bno085` for the actual run with
zero code changes.

Failure modes
-------------
* Source returns None for a tick (stale BNO sample, parse error)
  -> last valid heading is held; lock target is unchanged; drift log
  shows '--'. Never blocks, never crashes the loop.
* Both sources dead          -> loop keeps streaming the static
                                target and emits a [LOCK] WARN log.
                                ArduSub's onboard stabiliser still
                                tries to hold even without our drift
                                visibility.
* Operator forgets to unlock -> `timeout` (default 300 s) auto-stops
                                the thread with a [LOCK] log line.
* Manager process exits      -> daemon=True kills the thread; the
                                manager's atexit also calls
                                `pixhawk.send_neutral()`.
"""

import threading
import time

from .pixhawk import Pixhawk


# Stream rate. ArduSub drops SET_ATTITUDE_TARGET after ~1 s of silence
# and falls back to RC, so anything >= 5 Hz keeps the target latched.
# 20 Hz matches the motion-loop rate so logs interleave cleanly.
STREAM_HZ      = 20.0
DRIFT_LOG_SEC  = 1.0     # how often to print the [LOCK] heartbeat
SOURCE_DEAD_S  = 2.0     # warn after this many seconds with no fresh sample


class HeadingLock:
    """Background streamer for `SET_ATTITUDE_TARGET` at a fixed yaw.

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
                 timeout=300.0):
        self._pixhawk     = pixhawk
        self._yaw_source  = yaw_source
        self._log         = log
        self._timeout     = float(timeout)

        self._target_lock = threading.Lock()
        self._target_deg  = float(target_deg) % 360.0

        self._stop_event  = threading.Event()
        self._suspended   = threading.Event()
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
        self._thread.join(timeout=1.0)
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
        """Pause streaming (used while yaw_left/yaw_right/arc runs)."""
        self._suspended.set()

    def resume(self):
        """Resume streaming after a suspension."""
        self._suspended.clear()

    @property
    def target_deg(self):
        with self._target_lock:
            return self._target_deg

    # ------------------------------------------------------------------ #
    #  Thread body                                                       #
    # ------------------------------------------------------------------ #

    def _run(self):
        period       = 1.0 / STREAM_HZ
        started_at   = time.time()
        last_log     = 0.0
        last_fresh   = time.time()
        warned_dead  = False

        while not self._stop_event.is_set():
            now = time.time()

            if now - started_at > self._timeout:
                self._log.warn(
                    f'[LOCK ] timeout {self._timeout:.0f}s reached -- '
                    f'auto-releasing')
                break

            if self._suspended.is_set():
                self._stop_event.wait(timeout=period)
                continue

            with self._target_lock:
                target = self._target_deg

            # Stream the attitude target. ArduSub's 400 Hz stabiliser
            # does the actual correction; we just keep the target latched.
            try:
                self._pixhawk.set_attitude_setpoint(yaw_deg=target)
            except Exception as exc:
                self._log.warn(f'[LOCK ] set_attitude_setpoint raised: {exc}')

            # Read current heading from the source for drift visibility.
            current = _read_heading(self._pixhawk, self._yaw_source)
            if current is not None:
                last_fresh = now
                if warned_dead:
                    self._log.info('[LOCK ] yaw source recovered')
                    warned_dead = False
                error = Pixhawk.heading_error(target, current)
                if now - last_log >= DRIFT_LOG_SEC:
                    self._log.info(
                        f'[LOCK ] tgt:{target:6.1f}  cur:{current:6.1f}  '
                        f'err:{error:+5.1f}')
                    last_log = now
            elif not warned_dead and (now - last_fresh) > SOURCE_DEAD_S:
                self._log.warn(
                    f'[LOCK ] yaw source silent for {now-last_fresh:.1f}s '
                    f'-- holding last target')
                warned_dead = True

            self._stop_event.wait(timeout=period)


def _read_heading(pixhawk, yaw_source):
    """Same switchpoint the rest of the motion code uses."""
    if yaw_source is not None:
        return yaw_source.read_yaw()
    attitude = pixhawk.get_attitude()
    return None if attitude is None else attitude['yaw']
