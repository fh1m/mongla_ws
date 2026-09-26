"""Per-command MAVLink trace tag (lean, off-by-default).

ONE responsibility: when an operator runs the manager with `debug:=true`,
every MAVLink frame Pixhawk emits gets a tag in its DEBUG log line that
names the high-level Mongla verb that caused it. That turns this:

    [MAV send_rc_override] yaw=1430
    [MAV send_rc_override] yaw=1500

into this:

    [MAV send_rc_override cmd=lock_heading] yaw=1430
    [MAV send_rc_override cmd=yaw_right]    yaw=1500

so a single `rg "cmd=yaw_right"` over a session log shows EVERY frame
the verb produced, across files, without any per-call boilerplate.

Public surface (only three things):

    command_scope(verb)  -- context manager wrapping ONE Mongla call
    current_tag()        -- 'cmd=<verb>' or '' (read by Pixhawk._log_mavlink)
    set_enabled(bool)    -- flip tracing on (manager calls when debug:=true)

Design constraints (set by the user):

* No counters, no UUIDs -- just the verb name. `cmd=yaw_right`, never
  `cmd=yaw_right#42`. Multiple invocations look identical and that is
  fine; we trace by ordering, not by id.
* Off by default. Production runs stay quiet. The manager flips it on
  via `set_enabled(True)` only when the `debug` ROS-param is true.
* Background daemons (Heartbeat, HeadingLock) live in their own threads and
  open no scope, so their frames carry no `cmd=` tag. They still show their
  `_log_mavlink` callsite (e.g. `send_rc_override`), which identifies them.
  That is the intended trade: tag breadth for keeping the daemons free of
  tracing imports.

⛔ THE SWITCH IS A PLAIN GLOBAL, AND THAT IS THE BUG FIX. It used to be a
ContextVar, and a new thread in CPython starts with an EMPTY context rather than
a copy of its parent's. The manager runs a `MultiThreadedExecutor`, so every
action callback -- which is where every `command_scope()` in the facade and in
`_run_srot_move` is entered -- ran on a worker that read the DEFAULT, `False`.
The scope took its no-op branch and `debug:=true` produced no `cmd=` tag on any
frame, ever, while looking like it worked: the parameter was read, the logger
went to DEBUG, and `[MAV ...]` lines appeared without the one thing this module
exists for.

The two values are different kinds of thing and now have different types:

    _cmd_id    WHICH verb is in flight -- per scope, per thread, nested-safe.
               A ContextVar, correctly.
    _enabled   whether tracing runs AT ALL -- one process-wide switch, set once
               at startup. A module global, which every thread sees.

`test_tracing_crosses_thread_boundaries.py` fails if the switch becomes a
ContextVar again.
"""

import contextvars
from contextlib import contextmanager


_cmd_id = contextvars.ContextVar('mongla_cmd_id', default='')

# Process-wide, not per-context. A bare bool needs no lock: it is written once
# at startup and read on every frame, and CPython's attribute store is atomic.
_enabled = False


@contextmanager
def command_scope(verb: str):
    """Open a `cmd=<verb>` tag scope for the duration of one Mongla call.

    Use exactly once per public verb body (`Mongla.yaw_right`,
    `Mongla.vision_align`, ...). Nested scopes are allowed -- the
    inner one wins for the duration it is active and the outer scope
    is restored on exit. When tracing is disabled this is a no-op
    yield, so the cost in production is one branch.
    """
    if not _enabled:
        yield
        return
    token = _cmd_id.set(verb)
    try:
        yield
    finally:
        _cmd_id.reset(token)


def current_tag() -> str:
    """Return ``'cmd=<verb>'`` or ``''`` if no scope is open.

    `Pixhawk._log_mavlink` reads this and decides whether to splice
    ``' cmd=<verb>'`` into the log prefix, so the formatting (and the
    leading-space rule) lives in one place.
    """
    cid = _cmd_id.get()
    return f'cmd={cid}' if cid else ''


def set_enabled(value: bool) -> None:
    """Globally enable/disable tag emission for new `command_scope()` scopes.

    Called once at process startup from the manager (when `debug:=true`), and
    it reaches EVERY thread -- including the executor workers where the verbs
    actually run. Calling it from inside a `command_scope()` affects scopes
    opened after it, not the one in flight.
    """
    global _enabled
    _enabled = bool(value)


def is_enabled() -> bool:
    """Whether tagging is on, from any thread."""
    return _enabled
