"""anchor_state -- manager-side reader for the anchor_node topics.

Mirrors :class:`VisionState`: owns the ROS subscriptions to one camera's
anchor topics, caches the latest sample with a monotonic stamp, and exposes a
small read API the rclpy-free control loop (`anchor_align_loop`) consumes via a
provider -- exactly like the control loop reads detections through VisionState.

Keeps `motion_vision` free of ROS: the loop calls ``pose()`` and never touches
a subscription.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Vector3
from std_msgs.msg      import String, Float32

STATE_IDLE   = 'IDLE'
STATE_LOCKED = 'LOCKED'
STATE_LOST   = 'LOST'


@dataclass
class AnchorSample:
    """One read of the anchor pose error + lock state.

    tx_px / ty_px -- pixel pose error (+tx = reference right, +ty = below).
    theta_rad     -- in-plane rotation error.
    state         -- IDLE | LOCKED | LOST (the node's min_inliers verdict).
    conf          -- RANSAC inlier count.
    age_s         -- seconds since this error sample arrived (freshness).
    """
    tx_px:     float
    ty_px:     float
    theta_rad: float
    state:     str
    conf:      float
    age_s:     float


class AnchorState:
    """Subscribes one camera's anchor_{error,state,conf} and caches the latest."""

    def __init__(self, node: Node, *, camera: str = 'forward', logger=None):
        self._node   = node
        self._camera = camera
        self._log    = logger or node.get_logger()

        self._lock = threading.Lock()
        self._tx = self._ty = self._theta = 0.0
        self._state = STATE_IDLE
        self._conf  = 0.0
        self._err_stamp: float = 0.0          # monotonic of last anchor_error
        self._state_stamp: float = 0.0        # monotonic of last anchor_state

        ns  = f'/duburi/vision/{camera}'
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._sub_err = node.create_subscription(
            Vector3, f'{ns}/anchor_error', self._on_error, qos)
        self._sub_st  = node.create_subscription(
            String,  f'{ns}/anchor_state', self._on_state, qos)
        self._sub_cf  = node.create_subscription(
            Float32, f'{ns}/anchor_conf',  self._on_conf,  qos)

        self._log.info(f'[ANCH ] subscribed camera={camera!r} -> {ns}/anchor_error '
                       f'(+anchor_state, +anchor_conf)')

    # ---- callbacks ----------------------------------------------------- #
    def _on_error(self, msg: Vector3) -> None:
        with self._lock:
            self._tx = float(msg.x); self._ty = float(msg.y); self._theta = float(msg.z)
            self._err_stamp = time.monotonic()

    def _on_state(self, msg: String) -> None:
        with self._lock:
            self._state = str(msg.data) or STATE_IDLE
            self._state_stamp = time.monotonic()

    def _on_conf(self, msg: Float32) -> None:
        with self._lock:
            self._conf = float(msg.data)

    # ---- read API (control loop) --------------------------------------- #
    def pose(self) -> Optional[AnchorSample]:
        """Latest anchor sample, or None if no anchor_error has arrived yet."""
        with self._lock:
            if self._err_stamp == 0.0:
                return None
            return AnchorSample(
                tx_px=self._tx, ty_px=self._ty, theta_rad=self._theta,
                state=self._state, conf=self._conf,
                age_s=time.monotonic() - self._err_stamp)

    def state(self) -> str:
        with self._lock:
            return self._state

    def is_streaming(self) -> bool:
        """True once any anchor_error or anchor_state has been seen (node alive)."""
        with self._lock:
            return self._err_stamp > 0.0 or self._state_stamp > 0.0
