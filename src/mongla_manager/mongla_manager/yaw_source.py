"""Heading, read off the board's own fused attitude.

The BNO085 is on the SROT board (I2C0) and the board fuses it at 500 Hz, so
the manager's heading is the board's ATTITUDE message -- there is no second
heading sensor on this vehicle.

read_yaw() -> degrees in [0, 360), NED (0 = magnetic north, clockwise from
above), or None when the newest attitude is older than STALE_SECONDS. A
caller holds its last valid value for that tick; it never crashes or blocks
on a None. A stale heading must not be returned as if it were current: a
turn that locks onto a heading the hull has already rotated past overshoots.
"""

STALE_SECONDS = 0.25


class BoardYaw:
    name = 'board_ahrs'

    def __init__(self, fc):
        self.fc = fc

    def _is_fresh(self) -> bool:
        age = self.fc.get_attitude_age()
        return age is not None and age <= STALE_SECONDS

    def read_yaw(self):
        if not self._is_fresh():
            return None
        attitude = self.fc.get_attitude()
        return None if attitude is None else attitude['yaw']

    def is_healthy(self) -> bool:
        return self._is_fresh()

    def close(self) -> None:
        pass
