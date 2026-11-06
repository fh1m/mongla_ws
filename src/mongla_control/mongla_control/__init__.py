"""mongla_control -- the verbs, and the boundary to the SROT board.

Public surface:
  Mongla           -- serialised, high-level command facade (the host verbs)
  COMMANDS         -- single source of truth for /mongla/move commands
  fields_for       -- pull kwargs from a Move.Goal for COMMANDS dispatch
  MovementError    -- a verb that cannot do what it was asked; NotArmedError

The board driver is `mongla_control.fc.srot_fc.SrotFC`.
"""

from .commands     import COMMANDS, fields_for
from .mongla       import Mongla
from .errors       import MovementError, NotArmedError

__all__ = [
    'COMMANDS', 'fields_for', 'Mongla',
    'MovementError', 'NotArmedError',
]
