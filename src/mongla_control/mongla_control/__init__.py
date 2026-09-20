"""mongla_control -- MAVLink layer + high-level Mongla facade.

Public surface:
  Pixhawk          -- raw MAVLink wrapper (one instance per process)
  Mongla           -- serialised, high-level command facade
  HeadingLock      -- background Ch4 rate-override streamer (used by Mongla)
  Heartbeat        -- background neutral-RC-override streamer (FS_PILOT_INPUT guard)
  COMMANDS         -- single source of truth for /mongla/move commands
  fields_for       -- pull kwargs from a Move.Goal for COMMANDS dispatch
  MovementError    -- base exception, plus MovementTimeout / ModeChangeError
  tracing          -- per-command MAVLink-trace tag (off by default).
                      Manager flips it on via ``tracing.set_enabled(True)``
                      when the ``debug:=true`` ROS-param is set.
"""

from .commands     import COMMANDS, fields_for
from .mongla       import Mongla
from .errors       import ModeChangeError, MovementError, MovementTimeout
from .heading_lock import HeadingLock
from .heartbeat    import Heartbeat
from .pixhawk      import Pixhawk
from . import tracing

__all__ = [
    'COMMANDS', 'fields_for',
    'Mongla', 'HeadingLock', 'Heartbeat', 'Pixhawk',
    'MovementError', 'MovementTimeout', 'ModeChangeError',
    'tracing',
]
