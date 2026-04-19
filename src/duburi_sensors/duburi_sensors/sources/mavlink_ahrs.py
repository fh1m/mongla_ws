"""MavlinkAhrsSource -- default yaw source.

Wraps `Pixhawk.get_attitude()` so the rest of the system treats
ArduSub's onboard AHRS as just another YawSource. No new MAVLink
calls; no extra threads. As long as a fresh AHRS2 message has arrived
in the last `STALE_SECONDS` seconds, the source is healthy.

Staleness gate
--------------
The `.claude/context/sensors-pipeline.md` contract says any YawSource
must withhold a sample older than 250 ms instead of returning a stale
one -- otherwise yaw_snap / yaw_glide will lock onto a yaw value the
sub already moved past. We implement that gate via
`Pixhawk.get_attitude_age()` rather than reaching into pymavlink
internals here.
"""

from .base import YawSource

STALE_SECONDS = 0.25  # matches sensors-pipeline.md staleness contract


class MavlinkAhrsSource(YawSource):
    name = 'MAVLINK_AHRS'

    def __init__(self, pixhawk):
        self.pixhawk = pixhawk

    def _is_fresh(self):
        age = self.pixhawk.get_attitude_age()
        return age is not None and age <= STALE_SECONDS

    def read_yaw(self):
        if not self._is_fresh():
            return None
        attitude = self.pixhawk.get_attitude()
        return None if attitude is None else attitude['yaw']

    def is_healthy(self):
        return self._is_fresh()
