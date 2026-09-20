"""WitMotionSource — placeholder for future WitMotion HWT905 / WT901C
serial yaw integration.

WitMotion sensors ship a binary protocol (0x55 0x53 ... + checksum) over
USB-serial. When we wire it up:

  1. Implement WitMotionSource(port, baud) that mirrors BNO085Source's
     threading + stale-detection pattern but parses the binary frame.
  2. Register in factory._BUILDERS as 'witmotion'.

Until then, asking for yaw_source='witmotion' should fail loudly so
nobody runs a mission thinking they have heading data.
"""

from .base import YawSource


class WitMotionSource(YawSource):
    name = 'WITMOTION'

    def __init__(self, *_, **__):
        raise NotImplementedError(
            "WitMotion yaw source not implemented yet. "
            "Use yaw_source='mavlink_ahrs' or 'bno085'.")
