"""LocalizationBridge -- the manager's view of `/mongla/localization/state`.

Mirrors `DistanceState`: a manager-process subscription injected into the
`Mongla` facade (as `localization_provider`) so the control package stays
rclpy-free. `latest()` is the newest state while it is fresh by host arrival,
else None: a localiser that stopped publishing reads as absent, never as its
last word. The bound is the one the odometry consumers use (`ODOM_MAX_AGE_S`).
"""

from __future__ import annotations

import threading
import time

from mongla_interfaces.msg import LocalizationState
from mongla_localization.localization_publish import ODOM_MAX_AGE_S


class LocalizationBridge:
    def __init__(self, node, *, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._msg = None
        self._rx = 0.0
        node.create_subscription(LocalizationState, '/mongla/localization/state',
                                 self._on_state, 10)

    def _on_state(self, msg) -> None:
        with self._lock:
            self._msg, self._rx = msg, self._clock()

    def latest(self):
        with self._lock:
            if self._msg is None or self._clock() - self._rx > ODOM_MAX_AGE_S:
                return None
            return self._msg
