"""Subscribe single-frame poses, publish the one the support agrees on.

A SEPARATE topic, deliberately. `/target_pose` stays exactly what it is -- one
frame's answer, published by whichever estimator produced it -- and the fused
answer lands beside it on `/target_pose_fused`. A consumer that wants the
latest reading and one that wants the settled reading are different consumers,
and collapsing them would remove the ability to compare an estimator against
its own fused output, which is how a regression in either becomes visible.

The decision rule lives in `pose_cluster`, tested against constructed bimodal
distributions with no ROS. This node is the wiring: subscribe, stamp, fuse,
publish.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from duburi_interfaces.msg import TargetPose

from duburi_vision.pose_cluster import (
    CLUSTER_TOL_DEG, MIN_POSES, WINDOW_S, PoseCluster, PoseSample,
)
from duburi_vision.stamps import capture_monotonic


class PoseFuseNode(Node):
    def __init__(self, **kw):
        super().__init__('duburi_pose_fuse', **kw)
        cam = str(self.declare_parameter('camera', 'forward').value)
        ns = f'/duburi/vision/{cam}'
        # Every knob exposed: re-fusing a bag with a different window or floor
        # is the whole reason the per-frame evidence is published at all, and a
        # mission can reach these through `set_node('posefuse', ...)`.
        self._cluster = PoseCluster(
            window_s=float(self.declare_parameter('window_s', WINDOW_S).value),
            tol_deg=float(self.declare_parameter('tol_deg', CLUSTER_TOL_DEG).value),
            min_poses=int(self.declare_parameter('min_poses', MIN_POSES).value),
            max_ambiguity=float(self.declare_parameter('max_ambiguity', 0.9).value),
            max_reproj_px=float(self.declare_parameter('max_reproj_px', 10.0).value),
        )
        self._pub = self.create_publisher(TargetPose, f'{ns}/target_pose_fused', 10)
        self.create_subscription(TargetPose, f'{ns}/target_pose', self._on_pose, 10)
        self.get_logger().info(
            f'[FUSE ] {cam}: {ns}/target_pose -> {ns}/target_pose_fused '
            f'(window {self._cluster.window_s:.0f}s, '
            f'min {self._cluster.min_poses} agreeing)')

    def _on_pose(self, msg: TargetPose) -> None:
        # A pose the solver already disowned carries no vote. Publishing the
        # refusal onward would let a consumer count it as evidence of a fork.
        if not msg.ok:
            return
        t, _ = capture_monotonic(getattr(msg, 'header', None))
        self._cluster.add(PoseSample(
            t=t, yaw_deg=float(msg.yaw_deg), range_m=float(msg.range_m),
            ambiguity=float(msg.ambiguity), reproj_px=float(msg.reproj_px),
            n_points=int(msg.n_points)))

        out = TargetPose()
        out.header = msg.header
        fused = self._cluster.fuse(now=t)
        out.ok = bool(fused.decided)
        if fused.decided:
            out.yaw_deg = float(fused.yaw_deg)
            out.range_m = float(fused.range_m)
            # `n_points` is the support count here, not RANSAC inliers: a fused
            # pose is fitted from FRAMES, and a consumer reading "how much
            # evidence" wants that number. `yaw_spread_deg` keeps its meaning --
            # the width of the answer, now across frames instead of branches.
            out.n_points = int(fused.support)
            out.yaw_spread_deg = float(fused.spread_deg)
        else:
            out.reason = fused.reason
        self._pub.publish(out)


def main():
    rclpy.init()
    node = PoseFuseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
