"""duburi_sensors -- sensor sources + helpers for duburi_control.

Exports:
  YawSource           abstract base every source implements
  MavlinkAhrsSource   wraps duburi_control.Pixhawk.get_attitude
  BNO085Source        ESP32-C3 + BNO085 over USB CDC
  make_yaw_source     factory: name -> YawSource
"""

from .sources.base         import YawSource
from .sources.mavlink_ahrs import MavlinkAhrsSource
from .factory              import make_yaw_source

__all__ = ['YawSource', 'MavlinkAhrsSource', 'make_yaw_source']
