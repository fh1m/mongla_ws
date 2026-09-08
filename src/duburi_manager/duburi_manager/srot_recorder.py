"""Record the raw MAVLink stream so a dive can be replayed after the fact.

WHY RAW BYTES, AND ONLY RAW BYTES
---------------------------------
Everything the live console shows -- attitude, depth, the 24 multiplexed named
values, ESC telemetry, link rates, the filters -- is DERIVED from this stream.
Logging the derived values too would put a second copy of every number in the
file, and a second copy of a number is exactly how the sim's scorer came to
grade against a board that no longer existed. Log the source; derive the rest
at replay, with the same code the live path uses.

It also makes a decoder bug visible. A decoded log cannot show you a message
its decoder dropped, and we have one: pymavlink silently discards ESC_STATUS
(291), so it is absent from every decoded view and present in every raw byte.

WHY .tlog AND NOT MCAP
----------------------
The plan said MCAP, on the grounds that `pool_record.sh` already writes it and
Lichtblick reads it. That reasoning does not survive contact with the content:
`ros2 bag` records ROS TOPICS, and these are raw MAVLink frames, which
Lichtblick would render as opaque blobs whatever container they arrive in.

`.tlog` is MAVLink's own log format -- `[8-byte big-endian microseconds][frame]`
repeated -- and it costs nothing to gain a lot:

  * pymavlink opens one with `mavutil.mavlink_connection(path)`, so the replay
    tool is the same decoder as the live path with a different source
  * `mavlogdump.py`, MAVProxy and QGroundControl read it, which is what makes
    "decode it with a DIFFERENT decoder and get the same messages" an actual
    check rather than a wish
  * no new dependency on a control host

THE RULE THIS FILE EXISTS TO NOT BREAK
--------------------------------------
Whoever consumes must not set the pace of whoever produces. That has now
appeared three times in this stack -- the JPEG encoder pacing the control loop
at 31.9 Hz against a 50 Hz request, the view pacing the console's data plane,
and here. The reader thread is the ONLY thing draining the link, and the only
place the multiplexed NAMED_VALUE_FLOAT burst can be de-multiplexed, so it must
never wait on a disk. `write()` is an append to a bounded deque and nothing
else; a separate thread does the I/O.

When the writer falls behind, frames are DROPPED and COUNTED rather than
buffered without limit or blocking the reader. A recorder that stalls the
reader to save a log has broken the vehicle to record it; and a drop that is
not counted turns a lossy log into a log that lies.
"""
from __future__ import annotations

import collections
import os
import struct
import threading
import time
from pathlib import Path

# ~30 s of the full stream at the measured ~70 msg/s, so a disk hiccup is
# absorbed rather than lost. Bounded because unbounded is just a slower crash.
_QUEUE_MAX = 4096
_FLUSH_S = 1.0


class SrotRecorder:
    """Append-only .tlog writer fed from the MAVLink reader thread."""

    def __init__(self, path, log=None, queue_max: int = _QUEUE_MAX):
        self.path = Path(path)
        self._log = log
        self._q = collections.deque(maxlen=queue_max)
        self._lock = threading.Lock()
        self.dropped = 0
        self.write_errors = 0          # B44: flushes that raised (disk full, etc)
        self.last_write_error = ''
        self._consec_write_fail = 0
        self.written = 0
        self._stop = threading.Event()
        self._fh = None
        self._thread = None

    # -- producer side: called from the reader thread, must not block ------ #
    def write(self, msg) -> None:
        """Queue one message's raw bytes. Never raises, never waits.

        A message with no buffer (synthesised, or a decode failure) is skipped
        rather than logged as empty -- an empty frame in a tlog desynchronises
        every reader after it.
        """
        try:
            buf = msg.get_msgbuf()
        except Exception:                        # noqa: BLE001
            return
        if not buf:
            return
        rec = struct.pack('>Q', int(time.time() * 1e6)) + bytes(buf)
        with self._lock:
            if len(self._q) == self._q.maxlen:
                self.dropped += 1
                return
            self._q.append(rec)

    # -- lifecycle --------------------------------------------------------- #
    def start(self) -> 'SrotRecorder':
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, 'wb')
        self._thread = threading.Thread(target=self._drain, daemon=True,
                                        name='srot_recorder')
        self._thread.start()
        if self._log is not None:
            self._log.info(f'[REC  ] recording raw MAVLink to {self.path}')
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._flush()
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        if self._log is not None:
            self._log.info(
                f'[REC  ] {self.written} frames to {self.path.name}'
                + (f' -- {self.dropped} DROPPED (writer fell behind)'
                   if self.dropped else ''))

    # -- consumer side ----------------------------------------------------- #
    def _flush(self) -> None:
        with self._lock:
            batch, self._q = list(self._q), collections.deque(
                maxlen=self._q.maxlen)
        if not batch or self._fh is None:
            return
        self._fh.write(b''.join(batch))
        self._fh.flush()
        self.written += len(batch)

    def _drain(self) -> None:
        """Writer thread. A WRITE FAILURE MUST NOT KILL IT SILENTLY (B44).

        This is a bare `threading.Thread` target and `_flush()` does real disk
        I/O. A full card -- `OSError: [Errno 28] No space left on device`, the
        ordinary end state of a Pi after a few pool sessions -- used to propagate
        straight out of the thread. Recording then stopped with nothing logged,
        and the operator found out when they went looking for the tlog after the
        run that went wrong, which is exactly the run they needed it for.

        Note `_flush()` swaps the queue out BEFORE writing, so a failing batch is
        also lost. That is deliberate and stays: the alternative is holding a
        growing batch against a disk that is not coming back, which turns a lost
        log into a lost mission. What changes is that it is now COUNTED and SAID.
        """
        while not self._stop.is_set():
            self._safe_flush()
            self._stop.wait(_FLUSH_S)
        self._safe_flush()

    def _safe_flush(self) -> None:
        try:
            self._flush()
            self._consec_write_fail = 0
        except Exception as exc:                  # noqa: BLE001 -- see _drain
            self.write_errors += 1
            self._consec_write_fail = getattr(self, '_consec_write_fail', 0) + 1
            self.last_write_error = f'{type(exc).__name__}: {exc}'
            # First, then rarely: a full disk fails every flush, and at 1/_FLUSH_S
            # that would bury the line it is trying to be found in.
            if self._log is not None and (self._consec_write_fail == 1
                                          or self._consec_write_fail % 60 == 0):
                self._log.error(
                    f'[REC  ] !! tlog write FAILED ({self.write_errors} total): '
                    f'{self.last_write_error}. Recording is DEGRADED -- the '
                    f'session log will be incomplete. Check free space on '
                    f'{self.path}.')


def run_dir() -> Path:
    """Where scorecards already go, so a run's log and its card sit together."""
    return Path(os.environ.get('DUBURI_RUN_DIR',
                               str(Path.home() / 'duburi_runs')))


def default_path(tag: str = 'session') -> Path:
    stamp = time.strftime('%Y%m%d_%H%M%S')
    return run_dir() / f'{tag}_{stamp}.tlog'
