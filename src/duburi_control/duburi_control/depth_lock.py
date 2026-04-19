#!/usr/bin/env python3
"""Continuous depth-lock streamer (heading-lock's depth cousin).

A ``DepthLock`` is a daemon thread that streams an absolute depth
setpoint to ArduSub at 5 Hz via ``pixhawk.set_target_depth(target_m)``.
ArduSub's onboard ALT_HOLD controller (running at 400 Hz) closes the
loop on the throttle channel; the lock just keeps the setpoint *fresh*
so ArduSub does not fall back to the last-known value if the link
chokes. Caller MUST already be in ALT_HOLD before ``start()``.

Why this exists -- the symmetry with HeadingLock
------------------------------------------------
``HeadingLock`` writes Ch4 rate-overrides at 20 Hz because we don't
trust ArduSub's compass (thruster ESC interference) -- the Python loop
is the *whole* heading controller. ``DepthLock`` is the opposite: the
pressure sensor IS reliable and ArduSub's onboard depth PID is well
tuned, so we delegate the loop and just stream the setpoint. That keeps
the depth axis robust under translations / vision verbs / heading-lock
running in parallel because each thread writes a different MAVLink
message family:

  * HeadingLock                 -> RC_CHANNELS_OVERRIDE  (Ch4 only)
  * motion_forward / lateral    -> RC_CHANNELS_OVERRIDE  (Ch3/5/6 only)
  * DepthLock                   -> SET_POSITION_TARGET_GLOBAL_INT
  * motion_vision (depth axis)  -> SET_POSITION_TARGET_GLOBAL_INT

So the setpoint stream and the rate-override stream live on different
ArduSub inputs and never clobber each other. The only conflict is when
*two* authors stream depth setpoints simultaneously (e.g. a vision
verb that nudges depth while DepthLock is also streaming) -- that
collision is resolved at the facade by ``_suspend_depth_lock`` around
the verb body, exactly like ``_suspend_heading_lock`` does for yaw.

Lifecycle
---------
::

    lock = DepthLock(pixhawk, target_m=-0.5, log=...)
    lock.start()
    ...                       # other commands run; ArduSub holds depth
    lock.retarget(-1.2)       # set_depth() calls this on exit
    ...
    lock.stop()               # joins thread

Failure modes
-------------
* AHRS depth telemetry goes silent for ``> SOURCE_DEAD_S`` seconds
  -> log WARN and stop streaming. ArduSub's ALT_HOLD onboard controller
     keeps holding *some* depth (its last-known); we just stop pushing
     fresh setpoints because we can't see the result. When telemetry
     resumes, the lock has already exited; the operator must
     ``release_depth`` and ``lock_depth`` again.
* Operator forgets to release -> ``timeout`` (default 600 s) auto-stops.
* Manager process exits -> daemon=True kills the thread; the manager's
  shutdown hook also calls ``send_neutral`` and the autopilot drops
  the override.
"""

import threading
import time


# 5 Hz matches motion_depth.SETPOINT_HZ. ArduSub's depth loop runs at
# 400 Hz internally so 5 Hz from us is comfortably above the 1 Hz floor
# below which it stops trusting the setpoint.
STREAM_HZ      = 5.0
DRIFT_LOG_SEC  = 1.0     # how often to print the [DEPTH] heartbeat
SOURCE_DEAD_S  = 2.0     # warn after this many seconds with no fresh AHRS depth


class DepthLock:
    """Background streamer for ``set_target_depth`` at a fixed depth.

    Thread-safe: ``retarget`` can be called from any thread and the
    next streamed packet uses the new value. ``suspend`` / ``resume``
    are used by the facade to step out of the way while another author
    (vision_align_depth, vision_align_3d with depth axis) writes the
    depth setpoint directly.
    """

    def __init__(self, pixhawk, target_m, log, timeout=600.0):
        self._pixhawk     = pixhawk
        self._log         = log
        self._timeout     = float(timeout)

        self._target_lock = threading.Lock()
        self._target_m    = float(target_m)

        self._stop_event  = threading.Event()
        self._suspended   = threading.Event()
        self._thread      = threading.Thread(
            target=self._run, daemon=True, name='DepthLock')

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                          #
    # ------------------------------------------------------------------ #

    def start(self):
        self._log.info(
            f'[DEPTH] lock start  target={self._target_m:+.2f}m  '
            f'timeout={self._timeout:.0f}s')
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._log.info('[DEPTH] lock stopped')

    # ------------------------------------------------------------------ #
    #  Mutators (thread-safe)                                            #
    # ------------------------------------------------------------------ #

    def retarget(self, new_target_m):
        """Atomically swap the locked depth. Streams the new value on
        the next tick (worst case 1 / STREAM_HZ later)."""
        new = float(new_target_m)
        with self._target_lock:
            self._target_m = new
        self._log.info(f'[DEPTH] lock retarget -> {new:+.2f}m')

    def suspend(self):
        """Pause streaming (used while a vision-depth verb runs)."""
        self._suspended.set()

    def resume(self):
        """Resume streaming after a suspension."""
        self._suspended.clear()

    @property
    def target_m(self):
        with self._target_lock:
            return self._target_m

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
                    f'[DEPTH] lock timeout {self._timeout:.0f}s reached -- '
                    f'auto-releasing')
                break

            if self._suspended.is_set():
                self._stop_event.wait(timeout=period)
                continue

            with self._target_lock:
                target = self._target_m

            try:
                self._pixhawk.set_target_depth(target)
            except Exception as exc:
                self._log.warn(f'[DEPTH] set_target_depth raised: {exc}')

            # AHRS depth gives us a "did we actually reach it" signal,
            # but ArduSub closes the loop -- so this is just for the
            # operator's drift log + the silent-source watchdog.
            attitude = self._pixhawk.get_attitude()
            if attitude is not None:
                last_fresh = now
                if warned_dead:
                    self._log.info('[DEPTH] AHRS recovered -- resuming drift log')
                    warned_dead = False
                if now - last_log >= DRIFT_LOG_SEC:
                    current = float(attitude['depth'])
                    error   = target - current
                    self._log.info(
                        f'[DEPTH] lock tgt:{target:+.2f}m  '
                        f'cur:{current:+.2f}m  err:{error:+.2f}m')
                    last_log = now
            elif (now - last_fresh) > SOURCE_DEAD_S:
                # Pressure-source telemetry silent too long -> stop
                # pushing setpoints. ALT_HOLD will fall back to its
                # last-known onboard target.
                if not warned_dead:
                    self._log.warn(
                        f'[DEPTH] AHRS silent for {now-last_fresh:.1f}s -- '
                        f'stopping setpoint stream')
                    warned_dead = True
                break

            self._stop_event.wait(timeout=period)
