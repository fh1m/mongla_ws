"""When a camera that has stopped delivering is closed and opened again.

A USB camera can drop off the bus mid-run (a cable flexing in a hull, a hub
brown-out). The source does not recover by itself: the v4l2 mailbox keeps
polling a dead descriptor, `is_healthy()` turns False two seconds after the
last frame, and the node publishes nothing for the rest of the run.

Restarting the process would reopen it, but both cameras and both detectors
share one process on the vehicle (`detector_dual_node`), so that would blind
the healthy camera too. A lost camera is reopened in place instead, by its own
capture thread, which owns the source (`camera_node._capture_loop`).

Owner policy (2026-11-10), the same as for a dead process (`launch_respawn`):
three attempts, then the camera stays down and says so loudly. Nothing else
acts on it: the detector stops seeing frames, and the manager's `detector`
health row reads FAIL.

Only device sources are reopened. A video file at its end and a ROS topic
with no publisher are not lost hardware, and reopening them would hide that.

Measured on the vehicle with the forward camera (2026-11-10, measured-bars
"The restart policy"): an unplugged lead spent all three attempts in 6 s while
the device node did not exist, and the camera stayed LOST after it was plugged
back in. So an attempt counts only while the device node exists: with the
lead out there is nothing to try, and the watch waits for it to come back.
"""
from __future__ import annotations

from mongla_localization.launch_respawn import MAX_RESTARTS

#: Source kinds that are a physical device and can come back after a reopen.
DEVICE_KINDS = frozenset({'v4l2_mailbox', 'webcam'})

#: How long a camera must stay unhealthy before it is reopened. `is_healthy()`
#: already waits 2 s after the last frame, so a lost device is reopened about
#: 5 s after its last frame; a camera just reopened gets this long to deliver.
LOST_AFTER_S = 3.0

OK, WAITING, REOPEN, GAVE_UP = 'ok', 'waiting', 'reopen', 'gave_up'
#: Lost, and the device node is gone too (a lead out, a hub re-enumerating).
ABSENT = 'absent'


class CameraWatch:
    """Says when to reopen, given the camera's health over time.

    Pure: the caller passes health and the time, and does the reopening. Call
    `step` every so often from the thread that owns the source.
    """

    def __init__(self, source_kind: str, *, lost_after_s: float = LOST_AFTER_S,
                 max_reopens: int = MAX_RESTARTS):
        self.enabled = source_kind in DEVICE_KINDS
        self.lost_after_s = lost_after_s
        self.max_reopens = max_reopens
        self.reopens = 0
        self._unhealthy_since: float | None = None
        self._gave_up = False

    def step(self, healthy: bool, now: float, present: bool = True) -> str:
        """`present`: the device node exists. Absent time is never an attempt."""
        if not self.enabled or self._gave_up:
            return OK if healthy or not self.enabled else GAVE_UP
        if healthy:
            self._unhealthy_since = None
            return OK
        if not present:
            # Nothing to open. When it reappears it gets the full grace
            # period, as a freshly opened device would.
            self._unhealthy_since = None
            return ABSENT
        if self._unhealthy_since is None:
            self._unhealthy_since = now
        if now - self._unhealthy_since < self.lost_after_s:
            return WAITING
        if self.reopens >= self.max_reopens:
            self._gave_up = True
            return GAVE_UP
        self.reopens += 1
        # The fresh source gets the full grace period before it is judged.
        self._unhealthy_since = now
        return REOPEN
