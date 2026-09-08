"""CompositeBnoDvlSource -- heading from one source, position from another.

Named for its first user (BNO085 + Nucleus) but the split is generic: the two
arguments are only ever used as "the thing that knows heading" and "the thing
that knows position". `sim_dvl` reuses it unchanged with MAVLink AHRS heading
and the Gazebo DVL, which is why `name` is a constructor argument -- the log
line has to say which pairing is actually running.

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

    def __init__(self, bno_source, dvl_source, logger=None, name=None):
        """
        bno_source: heading source (provides read_yaw, is_healthy, close)
        dvl_source: position source (provides connect, get_position,
                    reset_position, is_healthy, close)
        name:       overrides the class-level name so logs identify the pairing
        """
        self._bno = bno_source
        self._dvl = dvl_source
        self._log = logger
        if name:
            self.name = name

    # ------------------------------------------------------------------
    #  YawSource ABC (heading from BNO085)
    # ------------------------------------------------------------------

    def read_yaw(self) -> float | None:
        return self._bno.read_yaw()

    def is_healthy(self) -> bool:
        """BOTH sources, as the class docstring has always claimed (B11).

        This returned only the BNO. A DVL that had died -- or never connected --
        left the composite reporting healthy, so `*_dist` verbs ran against a
        position integrator that was not integrating. `dvl_is_healthy()` stays
        for callers that need the halves apart.
        """
        return bool(self._bno.is_healthy()) and bool(self._dvl.is_healthy())

    def close(self) -> None:
        """Close BOTH, even if the first raises (B19).

        Without the try/finally an exception from the BNO -- a serial port
        already gone, which is exactly when close() runs -- skipped the DVL
        entirely, leaking its socket and reader thread.
        """
        try:
            self._bno.close()
        finally:
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
