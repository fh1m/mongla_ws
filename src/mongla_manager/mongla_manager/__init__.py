"""mongla_manager -- the ROS 2 action server that owns the SROT board.

The Python `MonglaClient`, the `mongla` CLI and the mission scripts that USE
the action server live in `mongla_planner`. This package is the server side:
the manager node, how it finds the board, and the operator tools.
"""
import contextlib

# Refuses a release that is not the active one (B-153). Tools that run this
# package from source alone have no runtime beside it; the refusal is a
# SystemExit, which this does not catch.
with contextlib.suppress(ModuleNotFoundError):
    import mongla_runtime  # noqa: F401
