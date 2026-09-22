"""mongla_planner -- Mongla mission planner.

Public surface (re-exported for ergonomic imports):
  MonglaClient    -- blocking Python API over /mongla/move
  MonglaMission   -- human-readable mission DSL wrapping MonglaClient
  MoveRejected    -- raised when the action server REJECTs a goal
  MoveFailed      -- raised when the goal completes with success=False
  MoveTimeout     -- raised (a MoveFailed) when the client's own deadline
                     elapses before the server returns -- bounds a wedged server

Submodules:
  client          -- MonglaClient implementation
  mongla_dsl      -- MonglaMission + mongla.vision sub-namespace
  cli             -- `mongla` argparse CLI (auto-built from COMMANDS)
  mission         -- `mission` runner that dispatches into missions/<name>.run
  missions/       -- one Python file per mission (run(mongla, log) entry)

Why this package exists
-----------------------
`mongla_manager` owns the action SERVER and the manager node lifecycle
(MAVLink + sensors + dispatcher). Everything that talks TO the server
lives here -- separating "the AUV is running" from "what should it do"
keeps imports one-way and lets us swap planners without touching the
manager.
"""

from .client     import MonglaClient, MoveFailed, MoveRejected, MoveTimeout
from .mongla_dsl import MonglaMission

__all__ = ['MonglaClient', 'MonglaMission', 'MoveFailed', 'MoveRejected',
           'MoveTimeout']
