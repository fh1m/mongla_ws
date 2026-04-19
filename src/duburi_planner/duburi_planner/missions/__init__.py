"""Mission registry -- single source of truth for `mission` runner.

Add a new mission:
  1. Create `missions/<name>.py` with `def run(client, log): ...`
  2. Import it here and add it to NAMES.
  3. `colcon build` and `ros2 run duburi_planner mission <name>`.
"""

from . import arc_demo, heading_lock_demo, square_pattern

NAMES = {
    'square_pattern':    square_pattern.run,
    'arc_demo':          arc_demo.run,
    'heading_lock_demo': heading_lock_demo.run,
}

__all__ = ['NAMES']
