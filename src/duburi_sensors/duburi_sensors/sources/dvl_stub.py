"""DVLSource — placeholder for future Nortek Nucleus1000 yaw integration.

The DVL exposes heading via its NMEA / JSON UDP stream on the ROV LAN
(192.168.2.201). When we wire it up:

  1. Implement DVLSource(host, port) that opens a UDP socket, parses the
     bottom-track JSON record, and updates self._latest_yaw.
  2. Mirror the BNO085Source threading + stale-detection pattern.
  3. Register in factory._BUILDERS as 'dvl'.

Until then, asking for yaw_source='dvl' should fail loudly so nobody runs
a mission thinking they have heading data.
"""

from .base import YawSource


class DVLSource(YawSource):
    name = 'DVL'

    def __init__(self, *_, **__):
        raise NotImplementedError(
            "DVL yaw source not implemented yet. "
            "Use yaw_source='mavlink_ahrs' or 'bno085'.")
