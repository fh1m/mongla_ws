#!/usr/bin/env python3
"""Continuous heading-lock streamer (depth-hold's yaw cousin).

A ``HeadingLock`` is a daemon thread that runs a proportional
Ch4-rate-override loop at 20 Hz against the heading read from the
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
  4. ``pixhawk.send_rc_override(yaw=percent_to_pwm(yaw_pct))``.
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
  the thread with a [LOCK] log line and releases Ch4.
* Manager process exits -> daemon=True kills the thread; the
  manager's atexit also calls ``pixhawk.send_neutral()``.
"""

import threading
import time

from .pixhawk import Pixhawk


# 20 Hz matches the motion-loop rate so logs interleave cleanly and the
# rate command is refreshed comfortably inside ArduSub's RC timeout.
STREAM_HZ      = 20.0
DRIFT_LOG_SEC  = 1.0     # how often to print the [LOCK] heartbeat
SOURCE_DEAD_S  = 2.0     # warn after this many seconds with no fresh sample

# Proportional rate-loop tunables. Gentler than yaw_snap because
# lock_heading is correcting small drifts, not executing 90 deg turns.
LOCK_KP_PCT_PER_DEG = 0.6
LOCK_PCT_MAX        = 25.0
LOCK_DEADBAND_DEG   = 1.5


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
                self._release_ch4()
                self._log.warn(
                    f'[LOCK ] timeout {self._timeout:.0f}s reached -- '
                    f'auto-releasing')
                break

            if self._suspended.is_set():
                self._stop_event.wait(timeout=period)
                continue

            with self._target_lock:
                target = self._target_deg

            current = _read_heading(self._pixhawk, self._yaw_source)

            if current is not None:
                last_fresh = now
                if warned_dead:
                    self._log.info('[LOCK ] yaw source recovered')
                    warned_dead = False

                error = Pixhawk.heading_error(target, current)
                if abs(error) <= LOCK_DEADBAND_DEG:
                    yaw_pct = 0.0
                else:
                    yaw_pct = max(-LOCK_PCT_MAX,
                                  min(LOCK_PCT_MAX,
                                      error * LOCK_KP_PCT_PER_DEG))

                try:
                    self._pixhawk.send_rc_override(
                        yaw=Pixhawk.percent_to_pwm(yaw_pct))
                except Exception as exc:
                    self._log.warn(f'[LOCK ] send_rc_override raised: {exc}')

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
        """Park the yaw channel at 1500 us (zero pilot rate)."""
        try:
            self._pixhawk.send_rc_override(yaw=Pixhawk.percent_to_pwm(0.0))
        except Exception:
            pass


def _read_heading(pixhawk, yaw_source):
    """Same switchpoint the rest of the motion code uses."""
    if yaw_source is not None:
        return yaw_source.read_yaw()
    attitude = pixhawk.get_attitude()
    return None if attitude is None else attitude['yaw']
