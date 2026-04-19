"""duburi_control -- MAVLink layer + high-level Duburi facade.

Public surface:
  Pixhawk          -- raw MAVLink wrapper (one instance per process)
  Duburi           -- serialised, high-level command facade
  HeadingLock      -- background Ch4 rate-override streamer (used by Duburi)
  Heartbeat        -- background neutral-RC-override streamer (FS_PILOT_INPUT guard)
  COMMANDS         -- single source of truth for /duburi/move commands
  fields_for       -- pull kwargs from a Move.Goal for COMMANDS dispatch
  MovementError    -- base exception, plus MovementTimeout / ModeChangeError
"""

from .commands     import COMMANDS, fields_for
from .duburi       import Duburi
from .errors       import ModeChangeError, MovementError, MovementTimeout
from .heading_lock import HeadingLock
from .heartbeat    import Heartbeat
from .pixhawk      import Pixhawk

__all__ = [
    'COMMANDS', 'fields_for',
    'Duburi', 'HeadingLock', 'Heartbeat', 'Pixhawk',
    'MovementError', 'MovementTimeout', 'ModeChangeError',
]
