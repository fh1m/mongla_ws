"""duburi_planner -- Mongla mission planner.

Public surface (re-exported for ergonomic imports):
  DuburiClient    -- blocking Python API over /duburi/move
  DuburiMission   -- human-readable mission DSL wrapping DuburiClient
  MoveRejected    -- raised when the action server REJECTs a goal
  MoveFailed      -- raised when the goal completes with success=False

Submodules:
  client          -- DuburiClient implementation
  duburi_dsl      -- DuburiMission + duburi.vision sub-namespace
  cli             -- `duburi` argparse CLI (auto-built from COMMANDS)
  mission         -- `mission` runner that dispatches into missions/<name>.run
  missions/       -- one Python file per mission (run(duburi, log) entry)
  state_machines/ -- reserved for future YASMIN-based plans

Why this package exists
-----------------------
`duburi_manager` owns the action SERVER and the manager node lifecycle
(MAVLink + sensors + dispatcher). Everything that talks TO the server
lives here -- separating "the AUV is running" from "what should it do"
keeps imports one-way and lets us swap planners without touching the
manager.
"""

from .client     import DuburiClient, MoveFailed, MoveRejected
from .duburi_dsl import DuburiMission

__all__ = ['DuburiClient', 'DuburiMission', 'MoveFailed', 'MoveRejected']
