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

import math
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


# ── the depth sign: the one place it flips ────────────────────────────────
#
# Two conventions, each correct where it lives, and ONE conversion between them:
#
#   ALTITUDE  negative below the surface.  `MonglaState.depth_m`, the facade,
#             every mission constant (-0.6 = 0.6 m down), the board's VFR_HUD.
#   NED z     positive DOWN.               the filter state, `/mongla/odom`
#             position.z, and VISION_POSITION_ESTIMATE, which MAVLink defines
#             in NED.
#
# ⛔ NO INTERFACE CHANGES SIGN HERE. Changing either would silently invert
# every depth guard on the other side (it has happened: `SrotFC.get_attitude`
# once negated an already-negative depth and every guard stopped firing).
# What changes is that the flip has a NAME, and `test_depth_sign.py` walks one
# depth through all three interfaces.


def ned_z_from_altitude(altitude_m: float) -> float:
    """-0.6 m (0.6 m below the surface) -> +0.6 NED z."""
    return -float(altitude_m)


def altitude_from_ned_z(z_ned: float) -> float:
    """+0.6 NED z -> -0.6 m, the MonglaState convention."""
    return -float(z_ned)


# ── the tree ─────────────────────────────────────────────────────────────────
#
#   map (ENU) ── pool (NED)                          static
#    └─ odom (ENU)                                   identity, ONLY once anchored
#        └─ odom_ned (NED)                           static
#            └─ mongla (FRD, at the IMU)             dynamic, the filter
#                ├─ base_link (FLU)                  static
#                ├─ downward_cam  (FRD axes)         static, ONLY when measured
#                └─ forward_cam   (FRD axes)         static, ONLY when measured
#
# One parent per frame. Before a pool fix `map` and `pool` are not connected to
# the hull, so a lookup into them FAILS -- which is the truth: there is no pool
# position yet. After the anchor, `rotate_world_yaw` has re-expressed the
# filter in pool axes, so odom_ned coordinates ARE pool coordinates and
# `map -> odom` is the identity.
#
# ⚠ `odom_ned` IS NOT CONTINUOUS ACROSS THE ANCHOR, and REP-105 says odom
# should be. The heading anchor ROTATES the filter state (`rotate_world_yaw`),
# so `odom_ned -> mongla` jumps in yaw -- and in x/y by the same rotation about
# the origin -- exactly once, at the anchor. Anyone adding `robot_localization`
# or another consumer that integrates odom must treat that instant as a reset
# (the uplink already counts it: `_send_position_uplink` bumps its reset counter
# on the frame change). The REP-shaped alternative, an unrotated odom plus a
# `map -> odom` correction, would mean carrying the anchor as a transform
# instead of in the state; not done, because every consumer today wants the
# pool-axis estimate itself.
#
# ⚠ `{cam}_cam` HAS BODY AXES, NOT OPTICAL ONES. It is the frame flow publishes
# in, and flow's velocity is already remapped to body x/y by `flow_node`. An
# optical frame (z out of the lens) belongs to the IMAGE and needs the measured
# image->body signs -- `flow_node.py` records four wrong answers reached
# deriving them -- so it is owed, not guessed.
#
# Quaternions are (x, y, z, w), ROS order.

# FLU axes written in FRD: x = x, y = -y, z = -z. 180 deg about x.
Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)
# NED axes written in ENU: north = y, east = x, down = -z. 180 deg about the
# (1, 1, 0) diagonal.
Q_ENU_TO_NED = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
Q_IDENTITY = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Edge:
    parent: str
    child: str
    xyz: Vec3
    q: Tuple[float, float, float, float]


def cam_frame(cam: str) -> str:
    """The body-aligned frame at camera `cam`'s lens -- what flow publishes in."""
    return f'{cam}_cam'


def static_edges(offsets: Offsets) -> list:
    """Every edge that does not move. A camera edge exists only if measured."""
    edges = [
        Edge(ODOM_ENU, ODOM, (0.0, 0.0, 0.0), Q_ENU_TO_NED),
        Edge(MAP_ENU, POOL, (0.0, 0.0, 0.0), Q_ENU_TO_NED),
        Edge(BODY, BODY_FLU, (0.0, 0.0, 0.0), Q_FRD_TO_FLU),
    ]
    for cam in ('downward', 'forward'):
        r = getattr(offsets, f'{cam}_cam')
        if r is not None:
            edges.append(Edge(BODY, cam_frame(cam), r, Q_IDENTITY))
    return edges


def anchored_edge() -> Edge:
    """`map -> odom`, published only once the pool anchor has been applied."""
    return Edge(MAP_ENU, ODOM_ENU, (0.0, 0.0, 0.0), Q_IDENTITY)


def to_msg(edge: Edge, stamp):
    """A `geometry_msgs/TransformStamped` for `edge`, imported lazily so this
    module stays importable without a ROS environment."""
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = edge.parent
    t.child_frame_id = edge.child
    t.transform.translation.x, t.transform.translation.y, \
        t.transform.translation.z = (float(c) for c in edge.xyz)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = edge.q
    return t
