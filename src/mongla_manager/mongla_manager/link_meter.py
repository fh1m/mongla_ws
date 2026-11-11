"""What the SROT link is carrying now: bytes and frames per second, per type.

A count divided by uptime answers "how much since boot" and hides a stream
that stopped; a sliding window answers "what is arriving now". The reader
thread, the one place every frame passes, feeds it with `note`; a 1 Hz timer
reads it into `/mongla/board_link`.

The line is 115200 baud, 8N1: ten bits a byte, so 11520 bytes/s is all it can
carry. The board's own view of the link (SYS_STATUS drop rate and error
count) is kept beside ours, as it last arrived; absent until it does.
"""
from __future__ import annotations

import collections
import math
import threading

WINDOW_S = 4.0
LINE_BYTES_PER_S = 115200 / 10


class LinkMeter:
    def __init__(self, window_s: float = WINDOW_S):
        self.window_s = window_s
        self._hits = collections.deque()          # (monotonic, type, bytes)
        self._lock = threading.Lock()
        self.drop_pct = math.nan
        self.errors = math.nan

    def note(self, msg, now: float) -> None:
        """One frame off the wire. Called on the reader thread: never raises."""
        kind = msg.get_type()
        try:
            nbytes = len(msg.get_msgbuf())
        except Exception:                          # noqa: BLE001 -- a bad frame has no buffer
            nbytes = 0
        if kind == 'SYS_STATUS':
            self.drop_pct = getattr(msg, 'drop_rate_comm', math.nan) / 100.0
            self.errors = float(getattr(msg, 'errors_comm', math.nan))
        with self._lock:
            self._hits.append((now, kind, nbytes))
            self._trim(now)

    def read(self, now: float) -> tuple[float, dict]:
        """(bytes per second, {message type: frames per second}) over the window."""
        with self._lock:
            self._trim(now)
            hits = list(self._hits)
        rates = collections.Counter(kind for _, kind, _ in hits)
        return (sum(n for _, _, n in hits) / self.window_s,
                {kind: n / self.window_s for kind, n in sorted(rates.items())})

    def _trim(self, now: float) -> None:
        cut = now - self.window_s
        while self._hits and self._hits[0][0] < cut:
            self._hits.popleft()
