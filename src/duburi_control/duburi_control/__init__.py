"""duburi_control -- MAVLink layer + high-level Duburi facade.

Public surface:
  Pixhawk          -- raw MAVLink wrapper (one instance per process)
  Duburi           -- serialised, high-level command facade
  HeadingLock      -- background SET_ATTITUDE_TARGET streamer (used by Duburi)
  COMMANDS         -- single source of truth for /duburi/move commands
  fields_for       -- pull kwargs from a Move.Goal for COMMANDS dispatch
  MovementError    -- base exception, plus MovementTimeout / ModeChangeError
"""

from .commands     import COMMANDS, fields_for
from .duburi       import Duburi
from .errors       import ModeChangeError, MovementError, MovementTimeout
from .heading_lock import HeadingLock
from .pixhawk      import Pixhawk

__all__ = [
    'COMMANDS', 'fields_for',
    'Duburi', 'HeadingLock', 'Pixhawk',
    'MovementError', 'MovementTimeout', 'ModeChangeError',
]
