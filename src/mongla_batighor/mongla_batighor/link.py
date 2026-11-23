"""How fast the viewers' video runs, set by the slowest browser's lag.

foxglove_bridge sends one client its messages in arrival order. When the
pictures need more than the link carries, everything waits behind them, the
vehicle's armed state included: three captures on 2026-11-23 showed it
seconds old with the stack healthy (the pictures ~1.5 MB/s, the link measured
0.96 MB/s that day). Each browser measures its own queueing lag from
`/mongla/state`'s stamp and reports it to the helm (`op: link`); this decides
what the cameras' `stream` topics run at. Only the viewers' copy: detection
reads `image_raw`, so perception never slows for a slow laptop.

The policy is the one a congested network has used for forty years: halve at
once on lag, add back a step only after a calm spell. Halving is held for
`HOLD_S` so a queue that is already full has time to drain before it is
judged again; a step up waits `CALM_S` after the last lag any browser
reported, so one fast browser cannot undo what a slow one needed.
"""
from __future__ import annotations

import math

LAG_HIGH_S = 0.5      # a state this late is one an operator would act on wrongly
LAG_LOW_S = 0.15      # below this the queue is empty to within a frame or two
HOLD_S = 2.0
CALM_S = 10.0
STEP = 0.25
FLOOR = 0.25          # 30 Hz -> 7.5 Hz: still a moving picture
MIN_HZ = 5.0


class StreamGovernor:
    def __init__(self):
        self.scale = 1.0
        self.floor = FLOOR
        self._changed = -math.inf
        self._unsettled = -math.inf

    def report(self, lag_s, now: float):
        """One browser's lag; the new scale when it changes, else None."""
        if isinstance(lag_s, bool) or not isinstance(lag_s, (int, float)) \
                or not math.isfinite(lag_s) or lag_s < 0:
            return None
        if lag_s >= LAG_LOW_S:
            self._unsettled = now
        if lag_s > LAG_HIGH_S:
            if self.scale > self.floor and now - self._changed >= HOLD_S:
                return self._set(max(self.floor, self.scale / 2), now)
            return None
        if lag_s < LAG_LOW_S and self.scale < 1.0 \
                and now - self._unsettled >= CALM_S and now - self._changed >= CALM_S:
            return self._set(min(1.0, self.scale + STEP), now)
        return None

    def _set(self, scale, now):
        self.scale = scale
        self._changed = now
        return scale

    def rate(self, top_hz: float) -> float:
        """A camera's stream rate at this scale; never under MIN_HZ unless its own is."""
        return max(min(top_hz, MIN_HZ), top_hz * self.scale)
