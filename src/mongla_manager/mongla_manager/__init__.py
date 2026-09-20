"""mongla_manager -- ROS2 ActionServer + connection profiles.

The Python `MonglaClient`, `mongla` CLI, and mission scripts that USE
the action server have moved to the `mongla_planner` package. This
package now contains ONLY the server side: the manager node, its
connection profiles, and the dispatcher.
"""

from .connection_config import DEFAULT_MODE, PROFILES

__all__ = ['PROFILES', 'DEFAULT_MODE']
