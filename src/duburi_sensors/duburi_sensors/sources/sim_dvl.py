"""SimDvlSource -- Gazebo's native DVL as a duburi_ws position source.

Gives the simulator the same bottom-track feedback the Nortek Nucleus1000 gives
the real hull, so `move_forward_dist` and friends close their loop in sim
instead of dead-reckoning. Before this existed those verbs fell back to an
open-loop timed estimate at a hardcoded 0.3 m/s and reported success anyway --
measured 2.361 m for a 1.0 m command.

  read_yaw()        -> delegated (the DVL is position-only; see below)
  get_position()    -> integrated (forward_m, starboard_m) since last reset
  reset_position()  -> zero the integrator

Factory keys: 'sim_dvl' (position only, yaw delegated to MAVLink AHRS)

WHY THE INTEGRATION LIVES HERE
------------------------------
The Gazebo DVL publishes VELOCITY, not position -- exactly like the Nucleus,
whose source integrates at `nucleus_dvl.py:297`. This mirrors that, including
the `_DT_MAX` stale-sample guard: a gap larger than that means we cannot know
what happened in between, and integrating across it would invent distance.

WHY THE gz IMPORT IS LAZY
-------------------------
`duburi_sensors` sources carry no ROS dependency by design, and the Jetson has
no Gazebo install at all. Importing gz-transport at module scope would make
`import duburi_sensors.sources` fail on the vehicle. It is imported inside
`connect()` instead, so this module is safe to import anywhere and only a
caller that actually asks for `yaw_source=sim_dvl` needs Gazebo present.
"""
from __future__ import annotations

import math
import threading
import time

from .base import YawSource

_GZ_TOPIC = '/dvl/velocity'
_STALE = 3.0     # unhealthy after this long with no bottom-locked sample
_DT_MAX = 0.5    # discard the step if the gap exceeds this (same as Nucleus)

# gz.msgs10.dvl_tracking_target_pb2.DVLTrackingTarget.TargetType
_TARGET_BOTTOM = 1
# gz.msgs10.dvl_kinematic_estimate_pb2.DVLKinematicEstimate.ReferenceType
_REFERENCE_SHIP = 2


class SimDvlSource(YawSource):
    """Gazebo DVL over gz-transport. Lazy-connect; call connect() before use."""

    name: str = 'sim_dvl'

    def __init__(self, topic: str = _GZ_TOPIC, logger=None):
        self._topic = topic
        self._log = logger
        self._lock = threading.Lock()
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._last_t = None
        self._last_good = None
        self._node = None          # held so gz-transport does not drop the sub

    # ------------------------------------------------------------------
    #  lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Subscribe to the Gazebo DVL. Safe to call more than once."""
        if self._node is not None:
            return
        try:
            from gz.transport13 import Node as GzNode
            from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking
        except ImportError as exc:
            raise RuntimeError(
                'yaw_source=sim_dvl needs the Gazebo python bindings '
                '(gz.transport13 / gz.msgs10), which are part of a Gazebo '
                f'Harmonic install. Not available here: {exc}') from exc

        node = GzNode()
        if not node.subscribe(DVLVelocityTracking, self._topic, self._on_msg):
            raise RuntimeError(
                f'could not subscribe to {self._topic}. Is the sim running, and '
                f'does the world load gz::sim::systems::DopplerVelocityLogSystem? '
                f'Without that system the DVL sensor loads and never publishes.')
        self._node = node
        if self._log:
            self._log.info(f'[DVL  ] sim DVL subscribed to {self._topic}')

    def close(self) -> None:
        self._node = None

    # ------------------------------------------------------------------
    #  sampling
    # ------------------------------------------------------------------

    def _on_msg(self, msg) -> None:
        # Drop anything that is not a live bottom lock. The sensor keeps
        # publishing when it loses the floor, and integrating a water-mass or
        # unspecified reading would silently invent position -- the exact
        # failure mode this whole source exists to remove.
        if msg.target.type != _TARGET_BOTTOM or msg.target.range.mean <= 0.0:
            return
        if msg.velocity.reference != _REFERENCE_SHIP:
            return                      # not body frame; we do not rotate it

        # AXIS MAPPING -- measured, not derived. The sensor's <reference_frame>
        # carries a -90 deg z rotation ("ENU to SFM"), so the DVL frame is NOT
        # the body frame and a naive .x/.y read is silently wrong: it reports
        # ~0.005 m/s while the hull does 0.7, which looks exactly like a broken
        # sensor. Measured against Gazebo odometry at ~0.68 m/s:
        #
        #     forward  (+x body)   -> dvl y = -0.687
        #     back     (-x body)   -> dvl y = +0.685
        #     starboard            -> dvl x = -0.678
        #
        # hence  forward = -dvl.y   and   starboard = -dvl.x.
        # `get_position()` returns (forward, starboard) because that is what
        # motion_forward reads as x and motion_lateral reads as y, with
        # move_right = +1. Change <reference_frame> in model.sdf.in and this
        # mapping must be re-measured -- there is no runtime check that can
        # catch it, only ground truth.
        fwd = -msg.velocity.mean.y
        stbd = -msg.velocity.mean.x

        # dt comes from the message's SIM-TIME stamp, never from wall clock.
        # Gazebo does not run at real time -- measured RTF here was 0.056 -- so
        # `time.monotonic()` deltas are unrelated to how far the vehicle
        # actually moved between two samples. Integrating v_sim against
        # dt_wall inflates distance by roughly 1/RTF, and because RTF drifts
        # with scene load the error grows with the length of the move: measured
        # 0.89 m of real travel reported as 0.93, then 1.58 as 2.04, then 2.18
        # as 3.00. Sim time makes the integrator RTF-independent.
        #
        # This is sim-only by nature. On the vehicle NucleusDVLSource uses wall
        # clock correctly, because there real time IS the clock.
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nsec * 1e-9

        with self._lock:
            if self._last_t is not None:
                dt = now - self._last_t
                if 0 < dt < _DT_MAX:
                    self._pos_x += fwd * dt
                    self._pos_y += stbd * dt
            self._last_t = now
            self._last_good = time.monotonic()   # health is a WALL-clock question

    # ------------------------------------------------------------------
    #  YawSource ABC
    # ------------------------------------------------------------------

    def read_yaw(self) -> float | None:
        """A DVL is not a heading source. Always None; pair it with one.

        Returning None rather than 0.0 is deliberate -- 0.0 is a legal heading
        and would be indistinguishable from 'pointing north'. The factory pairs
        this with MAVLink AHRS for heading.
        """
        return None

    def is_healthy(self) -> bool:
        with self._lock:
            last = self._last_good
        return last is not None and (time.monotonic() - last) < _STALE

    # ------------------------------------------------------------------
    #  DVL extensions -- the duck-typed contract motion_forward.py checks
    # ------------------------------------------------------------------

    def get_position(self) -> tuple[float, float]:
        """(x_m, y_m) integrated body-frame metres since the last reset."""
        with self._lock:
            return self._pos_x, self._pos_y

    def reset_position(self) -> None:
        with self._lock:
            self._pos_x = 0.0
            self._pos_y = 0.0
            self._last_t = None
