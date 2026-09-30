"""Frames: the names, the sensor offsets, and the conversions -- one owner.

⛔ WHY THIS FILE EXISTS (issues #19 and #27). The stack is FRD body / NED world
throughout, deliberately, and that stays. What it lacked was any statement of
WHERE its sensors are relative to one another, and any transform at the ROS
boundary. So:

* the flow DVL fused the camera's velocity as the IMU's (no lever arm), and
* every external tool read FRD/NED data under names that REP-103 reserves for
  FLU/ENU, with no tree to say otherwise.

This module is the single place the frame names and the sensor offsets live.
The offsets come from `frames.yaml` beside it, and `None` means NOT MEASURED --
never zero (see that file).
"""
from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import Optional, Tuple

import yaml

# ── frame names ──────────────────────────────────────────────────────────────
# `mongla` and `pool` collide with no REP-103 name and keep theirs. `odom` DID
# collide: REP-103 reserves it for ENU, and ours is NED, so a tool reading
# `/mongla/odom` under `odom` drew the hull climbing as it dived.
BODY = 'mongla'            # body FRD, origin at the board's IMU
BODY_FLU = 'base_link'     # REP-103 body, for tools
ODOM = 'odom_ned'          # the filter's world frame before a pool fix, NED
ODOM_ENU = 'odom'          # REP-103 odom, for tools
POOL = 'pool'              # the anchored pool frame, NED
MAP_ENU = 'map'            # REP-103 map, parent of `pool`

Vec3 = Tuple[float, float, float]

_YAML = pathlib.Path(__file__).with_name('frames.yaml')


@dataclass(frozen=True)
class Offsets:
    """IMU -> sensor, body FRD, metres. `None` = not measured."""
    downward_cam: Optional[Vec3]
    forward_cam: Optional[Vec3]
    baro: Optional[Vec3]

    def unmeasured(self) -> list:
        return [k for k in ('downward_cam', 'forward_cam', 'baro')
                if getattr(self, k) is None]

    def baro_to_downward_cam_z(self) -> Optional[float]:
        """How far BELOW the baro port the downward lens sits, metres (FRD z).

        Needs both offsets. A partial measurement is not a measurement.
        """
        if self.baro is None or self.downward_cam is None:
            return None
        return float(self.downward_cam[2]) - float(self.baro[2])


def _vec(v, name: str) -> Optional[Vec3]:
    if v is None:
        return None
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f'frames.yaml: imu_to.{name} must be null or [x, y, z]'
                         f' in metres, got {v!r}')
    out = tuple(float(c) for c in v)
    # A lever arm longer than the hull is a unit error (mm typed as m), not a
    # vehicle: the hull is 0.702 m long.
    if any(abs(c) > 1.0 for c in out):
        raise ValueError(f'frames.yaml: imu_to.{name} = {out} has a component '
                         f'over 1 m on a 0.70 m hull -- millimetres typed as '
                         f'metres?')
    return out


def load(path: Optional[pathlib.Path] = None) -> Offsets:
    """Read the offsets. Raises on a malformed entry rather than guessing."""
    data = yaml.safe_load(pathlib.Path(path or _YAML).read_text()) or {}
    imu_to = data.get('imu_to') or {}
    return Offsets(**{k: _vec(imu_to.get(k), k)
                      for k in ('downward_cam', 'forward_cam', 'baro')})
