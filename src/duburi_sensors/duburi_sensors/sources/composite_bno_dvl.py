"""CompositeBnoDvlSource -- BNO085 heading + Nucleus DVL position.

Use this when you want the BNO085's gyro-fused heading (robust against
magnetic interference) for yaw turns AND the Nucleus 1000's DVL bottom-
track for closed-loop distance moves.

  read_yaw()         -> BNO085 heading
  get_position()     -> Nucleus DVL integrated position
  reset_position()   -> zero DVL integrator
  connect()          -> open DVL TCP connection
  is_healthy()       -> BNO healthy AND DVL streaming
  close()            -> shut down both sources

Factory key: 'bno085_dvl'

Startup sequence (pool day):
    ros2 launch duburi_manager bringup.launch.py yaw_source:=bno085_dvl

DVL connects automatically if dvl_auto_connect:=true (default).
Manual fallback: ros2 run duburi_planner duburi dvl_connect
"""
from __future__ import annotations

from .base import YawSource


class CompositeBnoDvlSource(YawSource):
    """BNO085 yaw + Nucleus DVL position in one YawSource-compatible object."""

    name: str = 'bno085_dvl'

    def __init__(self, bno_source, dvl_source, logger=None):
        """
        bno_source: BNO085Source (provides read_yaw, is_healthy, close)
        dvl_source: NucleusDVLSource (provides connect, get_position,
                    reset_position, is_healthy, close)
        """
        self._bno = bno_source
        self._dvl = dvl_source
        self._log = logger

    # ------------------------------------------------------------------
    #  YawSource ABC (heading from BNO085)
    # ------------------------------------------------------------------

    def read_yaw(self) -> float | None:
        return self._bno.read_yaw()

    def is_healthy(self) -> bool:
        return self._bno.is_healthy()

    def close(self) -> None:
        self._bno.close()
        self._dvl.close()

    # ------------------------------------------------------------------
    #  DVL extensions (position from Nucleus DVL)
    # ------------------------------------------------------------------

    def get_position(self) -> tuple[float, float]:
        """Return (x_m, y_m) integrated body-frame position since last reset."""
        return self._dvl.get_position()

    def reset_position(self) -> None:
        """Zero the DVL body-frame position integrator."""
        self._dvl.reset_position()

    def connect(self) -> None:
        """Open DVL TCP connection. Called by dvl_connect verb or auto-connect."""
        self._dvl.connect()

    def dvl_is_healthy(self) -> bool:
        """True when DVL is streaming (separate from BNO health)."""
        return self._dvl.is_healthy()
