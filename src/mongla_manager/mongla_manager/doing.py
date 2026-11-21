"""What the vehicle is doing: every verb the manager takes, as it starts and as it ends.

Every goal reaches the board through `/mongla/move`, whoever sent it: a
mission, the CLI, the deck or a browser through the helm. An operator could
see a goal's feedback but not the goal itself: the bridge relays topics, not
actions, so Batighor showed a pixel error with no verb, no target and no
outcome behind it. The manager publishes this record on `/mongla/doing`
(`std_msgs/String`, JSON, latched): the verb running now, the fields its
sender set, and the last few verbs with how each ended, refusals included.

Pure: no ROS here, so the record is tested without a node.
"""
from __future__ import annotations

import json
import math
import time

TOPIC = '/mongla/doing'
KEEP = 12                     # verbs remembered: a minute of a busy mission

RUNNING, DONE, CANCELED, FAILED, REFUSED = 'running', 'done', 'canceled', 'failed', 'refused'


def fields_of(request) -> dict:
    """The fields the sender set, with their values; a NaN becomes None (JSON has none).

    `set_fields` names them when the sender used our clients. A goal sent
    without it (`ros2 action send_goal`, a test) shows every field that
    differs from a fresh goal's, so no sender's request reads as empty.
    """
    names = list(getattr(request, 'set_fields', None) or ())
    if not names and hasattr(request, 'get_fields_and_field_types'):
        blank = type(request)()
        names = [n for n in request.get_fields_and_field_types()
                 if n not in ('cmd', 'set_fields') and getattr(request, n) != getattr(blank, n)]
    out = {}
    for name in names:
        v = getattr(request, name, None)
        if isinstance(v, float) and not math.isfinite(v):
            v = None
        elif not isinstance(v, (bool, int, float, str, type(None))):
            v = str(v)
        out[str(name)] = v
    return out


class Doing:
    """The running verbs and the recent ones, as one JSON document.

    Keyed by goal: a safety verb pre-empts a running one, so two can be
    running for the moment the first winds down, and the first one's end
    must not close the second.
    """

    def __init__(self, clock=time.time):
        self._clock = clock
        self.running: dict = {}          # goal -> entry, in start order
        self.recent: list = []

    def start(self, cmd: str, request, goal: str) -> str:
        self.running[goal] = {'verb': cmd, 'fields': fields_of(request), 'goal': goal,
                              'state': RUNNING, 'since': self._clock()}
        return self.text()

    def end(self, goal: str, state: str, message: str = '', final_value=None) -> str:
        entry = self.running.pop(goal, None) or {'verb': '?', 'fields': {}, 'goal': goal,
                                                 'since': self._clock()}
        return self._close(entry, state, message, final_value)

    def refused(self, cmd: str, request, why: str) -> str:
        """A goal refused before it ran: recorded, and nothing running changes."""
        entry = {'verb': cmd, 'fields': fields_of(request), 'goal': '', 'since': self._clock()}
        return self._close(entry, REFUSED, why, None)

    def _close(self, entry, state, message, final_value) -> str:
        if isinstance(final_value, (int, float)) and math.isfinite(final_value):
            entry['final_value'] = float(final_value)
        entry.update(state=state, message=str(message)[:240], ended=self._clock())
        self.recent = ([entry] + self.recent)[:KEEP]
        return self.text()

    @property
    def now(self):
        """The verb started last that is still running, or None."""
        return next(reversed(self.running.values()), None)

    def text(self) -> str:
        return json.dumps({'now': self.now, 'running': len(self.running),
                           'recent': self.recent, 'stamp': self._clock()})
