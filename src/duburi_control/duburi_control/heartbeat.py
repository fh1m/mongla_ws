#!/usr/bin/env python3
"""Heartbeat -- keep the RC override stream warm so ArduSub never disarms.

Why this exists
---------------
ArduSub treats prolonged silence on ``RC_CHANNELS_OVERRIDE`` as the pilot
disconnecting. The behaviour is governed by ``FS_PILOT_TIMEOUT`` (range
0.1 .. 3.0 s, default 3.0 s) and ``FS_PILOT_INPUT`` (default action:
disarm). If we send a one-shot command (``set_depth``, ``arm``,
``set_mode`` etc.) and then sit idle for more than the timeout, the
autopilot believes the link dropped, runs the failsafe, and the next
verb arrives at a disarmed sub.

Today this gap is masked by ``HeadingLock`` -- when the lock is engaged
its 20 Hz Ch4 stream incidentally counts as pilot input. But missions
that never engage ``lock_heading`` have no such cover, and any pause
between commands longer than ~3 s would trip the failsafe.

``Heartbeat`` closes the gap explicitly: a tiny daemon that writes a
neutral (1500 us) RC override on Ch1..Ch6 at ``HEARTBEAT_HZ`` (default
5 Hz, well inside the 100 ms .. 3 s window). Neutral PWM means "pilot
present, not commanding any axis" -- ArduSub's onboard automation
(ALT_HOLD depth, MANUAL drift, etc.) keeps doing its job and the
failsafe stays disarmed.

Why not just use HeadingLock for this
-------------------------------------
HeadingLock is a *control* loop: it commands a yaw rate, which is a
real movement. Heartbeat is *not* a control loop -- it commands no
movement, just keeps the wire warm. Conflating the two means
"failsafe-quiet" depends on someone remembering to engage a heading
lock; this way it is automatic and free.

Cooperation with other writers
------------------------------
Anything that owns the RC channels for a stretch (a Duburi command, a
HeadingLock thread, etc.) calls ``Heartbeat.pause()`` on entry and
``Heartbeat.resume()`` on exit so the two writers never race. The
pause counter is reentrant, so nested holds are fine.

When the heartbeat is paused the daemon thread sleeps without writing,
so the active writer's packets are the only ones on the wire.

Lifecycle
---------
::

    hb = Heartbeat(pixhawk, log)
    hb.start()
    ...
    with hb.hold():
        # active command writes here; heartbeat skipped
        ...
    ...
    hb.stop()
"""

import threading
import time
from contextlib import contextmanager

from .motion_rates import HEARTBEAT_HZ as STREAM_HZ


class Heartbeat:
    """Background streamer of all-neutral RC overrides.

    Thread-safe. ``pause`` / ``resume`` use a counter so multiple
    callers can stack their holds without one's ``resume`` accidentally
    re-enabling the stream while another caller still wants it quiet.
    """

    def __init__(self, pixhawk, log, hz: float = STREAM_HZ):
        self._pixhawk = pixhawk
        self._log     = log
        self._period  = 1.0 / float(hz)

        self._stop_event = threading.Event()
        self._hold_count = 0
        self._hold_lock  = threading.Lock()
        self._thread     = threading.Thread(
            target=self._run, daemon=True, name='Heartbeat')

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                          #
    # ------------------------------------------------------------------ #

    def start(self):
        self._log.info(
            f'[HBEAT] start  rate={1.0/self._period:.1f}Hz  '
            f'(neutral RC override on Ch1..Ch6)')
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._log.info('[HBEAT] stopped')

    # ------------------------------------------------------------------ #
    #  Hold (suspend / resume) -- reentrant via a counter                 #
    # ------------------------------------------------------------------ #

    def pause(self):
        with self._hold_lock:
            self._hold_count += 1

    def resume(self):
        with self._hold_lock:
            if self._hold_count > 0:
                self._hold_count -= 1

    @contextmanager
    def hold(self):
        """Suspend the heartbeat for the duration of the with-block."""
        self.pause()
        try:
            yield
        finally:
            self.resume()

    # ------------------------------------------------------------------ #
    #  Thread body                                                       #
    # ------------------------------------------------------------------ #

    def _is_held(self) -> bool:
        with self._hold_lock:
            return self._hold_count > 0

    def _run(self):
        while not self._stop_event.is_set():
            if not self._is_held():
                try:
                    self._pixhawk.send_neutral()
                except Exception as exc:
                    self._log.warn(f'[HBEAT] send_neutral raised: {exc}')
            self._stop_event.wait(timeout=self._period)
