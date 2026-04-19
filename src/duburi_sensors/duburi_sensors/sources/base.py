"""Abstract YawSource — every yaw-providing sensor implements this.

Contract
--------
read_yaw() -> float | None
    Latest yaw in **degrees**, range [0, 360), NED frame
    (0 = magnetic north, +CW from above). Returns None when no fresh
    sample is available (sensor not yet warmed up, stale, parse error,
    failed calibration gate, etc). Callers MUST hold the last valid
    value for that tick rather than crash or block.

is_healthy() -> bool
    True when read_yaw() is currently producing usable values. Used
    only for startup banners + diagnostic logging — never for
    runtime fallback (single source per launch is the design).

close()
    Release any underlying resources (serial port, threads). Safe to
    call multiple times.
"""


class YawSource:
    name: str = 'base'

    def read_yaw(self) -> float | None:
        raise NotImplementedError

    def is_healthy(self) -> bool:
        return False

    def close(self) -> None:
        pass

    def __repr__(self) -> str:
        return f'<{self.__class__.__name__} name={self.name}>'
