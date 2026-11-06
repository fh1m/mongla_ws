"""Verb-layer exceptions.

Raised by `Mongla` and the vision verbs to fail loudly instead of returning a
success that moved nothing. The action server catches them, sets
`Move.Result.success = False` with the message, and aborts the goal, so the
CLI and mission scripts see the failure.

  MovementError  -- base class: a verb that cannot do what it was asked
                    (a refused mode, an axis the board does not take).
  NotArmedError  -- a motion verb on a disarmed vehicle.
"""


class MovementError(Exception):
    """A verb that cannot do what it was asked. The message says why."""


class NotArmedError(MovementError):
    """Command rejected because the AUV is not armed.

    The board ignores motion demands while disarmed, so a motion command
    would appear to succeed but produce no thrust. Call arm() first.
    """
