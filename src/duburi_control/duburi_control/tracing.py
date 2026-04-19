"""Per-command MAVLink trace tag (lean, off-by-default).

ONE responsibility: when an operator runs the manager with `debug:=true`,
every MAVLink frame Pixhawk emits gets a tag in its DEBUG log line that
names the high-level Duburi verb that caused it. That turns this:

    [MAV pixhawk.py:send_rc_override] RC_OVERRIDE ... yaw=1430
    [MAV pixhawk.py:send_rc_override] RC_OVERRIDE ... yaw=1500

into this:

    [MAV pixhawk.py:send_rc_override cmd=lock_heading] RC_OVERRIDE ... yaw=1430
    [MAV pixhawk.py:send_rc_override cmd=yaw_right]    RC_OVERRIDE ... yaw=1500

so a single `rg "cmd=yaw_right"` over a session log shows EVERY frame
the verb produced, across files, without any per-call boilerplate.

Design constraints (set by the user):

* No counters, no UUIDs -- just the verb name. `cmd=yaw_right`, never
  `cmd=yaw_right#42`. Multiple invocations look identical and that is
  fine; we trace by ordering, not by id.
* Off by default. Production runs stay quiet. The manager flips it on
  via `set_enabled(True)` only when the `debug` ROS-param is true.
* Background daemons (Heartbeat, HeadingLock) live in their own
  threads. ContextVar values do NOT propagate across `threading.Thread`
  boundaries, so their MAVLink frames will simply lack the `cmd=` tag
  -- they'll still show `pixhawk.py:send_rc_override`, which is
  enough to identify them. This is intentional: we trade tag breadth
  for keeping the daemons free of tracing imports.
"""

import contextvars
from contextlib import contextmanager


_cmd_id  = contextvars.ContextVar('duburi_cmd_id',  default='')
_enabled = contextvars.ContextVar('duburi_tracing', default=False)


@contextmanager
def command(verb: str):
    """Open a `cmd=<verb>` tag scope for the duration of one Duburi call.

    Use exactly once per public verb body (`Duburi.yaw_right`,
    `Duburi.vision_align_yaw`, ...). Nested scopes are allowed -- the
    inner one wins for the duration it is active and the outer scope
    is restored on exit. When tracing is disabled this is a no-op
    yield, so the cost in production is one branch.
    """
    if not _enabled.get():
        yield
        return
    token = _cmd_id.set(verb)
    try:
        yield
    finally:
        _cmd_id.reset(token)


def tag() -> str:
    """Return ` cmd=<verb>` (note leading space) or '' if no scope is open.

    `Pixhawk._mav` interpolates this directly into the log prefix so
    the formatting stays in one place.
    """
    cid = _cmd_id.get()
    return f' cmd={cid}' if cid else ''


def set_enabled(value: bool) -> None:
    """Globally enable/disable tag emission for new `command()` scopes.

    Intended to be called once at process startup from the manager
    (when `debug:=true`). Calling this from inside a `command()`
    scope is allowed but only affects scopes opened AFTER the call.
    """
    _enabled.set(bool(value))


def is_enabled() -> bool:
    """Cheap read for tests / introspection."""
    return _enabled.get()
