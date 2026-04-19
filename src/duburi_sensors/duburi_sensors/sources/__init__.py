from .base         import YawSource
from .mavlink_ahrs import MavlinkAhrsSource

# BNO085Source is imported lazily by the factory so that workspaces
# without pyserial can still use the default mavlink_ahrs source.
# Import it directly only if you need the class symbol:
#   from duburi_sensors.sources.bno085 import BNO085Source

__all__ = ['YawSource', 'MavlinkAhrsSource']
