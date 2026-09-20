"""One PnP solver for every evidence source. Subscribe correspondences, publish a pose.

WHY A SEPARATE NODE AND NOT A FUNCTION CALL IN EACH PRODUCER. `lock_node` used
to match, solve and publish in one place, so the pose could only ever come from
the anchor's patch. Splitting the evidence (`TargetCorrespondences`) from the
solve makes three things possible that were not:

  * A board layout, a set of opening centres, or a segmentation contour can
    produce a pose without any of them re-implementing IPPE-vs-SQPnP, the flip
    interval, or the reprojection gates. There is ONE solver
    (`anchor/pose.py::solve_pnp`) and one place its thresholds live.
  * A recorded run is RE-SOLVABLE. Play the bag back into this node with a
    different `max_reproj_px` and see whether a missed shot was bad evidence or
    a bad solve -- a question the pose alone can never answer.
  * Two estimators can solve the same target from different evidence and
    publish to distinct topics, which is what item 2.7 needs and what a single
    fused publisher structurally prevents.

⛔ THE TWO SILENT SCALE ERRORS THIS NODE EXISTS TO NOT MAKE. Both produce a
plausible number and no warning, and both were already documented in
`lock_node` before this node existed:

  1. **Optics.** Underwater a flat port is not a pinhole. Rectified points must
     be solved against the RECTIFIED K -- mixing them leaves a clean 1/n scale
     error, every range 33 % short. The message states which optics its pixels
     are in and an unset value is REFUSED, never assumed.
  2. **Resolution.** The anchor matches in its backend's pixels, not the
     camera's full frame, while CameraInfo describes the full frame. The
     message states its own pixel frame and K is scaled to it.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from mongla_interfaces.msg import TargetCorrespondences, TargetPose


class PnPNode(Node):
    def __init__(self, **kw):
        super().__init__('mongla_pnp', **kw)
        cam = str(self.declare_parameter('camera', 'forward').value)
        ns = f'/mongla/vision/{cam}'
        self._cam = cam
        self._medium = str(
            self.declare_parameter('medium', 'water').value or 'water').lower()
        if self._medium not in ('water', 'air'):
            raise ValueError(
                f"medium must be 'water' or 'air', got {self._medium!r}")
        # Deliberately exposed: re-solving a bag with a different gate is half
        # the reason the evidence is published at all.
        from mongla_vision.anchor.pose import AMBIGUITY_MAX, MAX_REPROJ_PX
        self._ambiguity_max = float(
            self.declare_parameter('ambiguity_max', AMBIGUITY_MAX).value)
        self._max_reproj_px = float(
            self.declare_parameter('max_reproj_px', MAX_REPROJ_PX).value)

        # ⛔ REDUNDANCY IS CHEAPER THAN PICKING THE RIGHT NUMBER. A
        # reprojection gate that is tight enough to reject a bad solve at 3 m
        # also rejects a good one at 0.4 m, where the board fills the frame and
        # a few pixels of contour error are ordinary. BumblebeeAS run exactly
        # this pair on slalom -- a normal estimator and a `_near` variant at
        # max_reprojection_error 100.0 against 10.0 -- and let the consumer
        # choose, instead of tuning one threshold and losing the other regime.
        #
        # A variant publishes to its OWN topic. Merging them here would destroy
        # the thing that makes redundancy useful: a consumer cannot prefer one
        # estimator over another if both arrive on the same topic, and a
        # disagreement between them is evidence rather than noise.
        variant = str(self.declare_parameter('variant', '').value or '').strip()
        topic = f'{ns}/target_pose' + (f'_{variant}' if variant else '')
        self._variant = variant

        self._info = None
        self._K_full = None
        self._cache = {}            # (w,h) -> (rect, K_rect)
        self._pub = self.create_publisher(TargetPose, topic, 10)
        self.create_subscription(CameraInfo, f'{ns}/camera_info',
                                 self._on_info, 10)
        self.create_subscription(TargetCorrespondences, f'{ns}/correspondences',
                                 self._on_corr, 10)
        self.get_logger().info(
            f'[PNP  ] {cam}: medium={self._medium} '
            f'reproj<={self._max_reproj_px:.1f}px  {ns}/correspondences -> '
            f'{topic}')

    def _on_info(self, msg):
        k = list(msg.k)
        if len(k) < 9 or k[0] <= 0.0:
            return
        K = np.array([[k[0], 0.0, k[2]],
                      [0.0, k[4], k[5]],
                      [0.0, 0.0, 1.0]], np.float64)
        if self._K_full is not None and np.array_equal(K, self._K_full):
            return                  # CameraInfo arrives with every frame
        self._K_full = K
        self._info = (int(msg.width or 0), int(msg.height or 0))
        self._cache.clear()
        self.get_logger().info(
            f'[PNP  ] camera_info {self._info[0]}x{self._info[1]} fx={k[0]:.1f}')

    def _optics_for(self, w, h):
        """(rectifier, K) for points expressed in a `w`x`h` frame.

        Cached per frame size: building the rectifier is not free and a
        producer's resolution does not change between messages.
        """
        key = (w, h)
        if key in self._cache:
            return self._cache[key]
        fw, fh = self._info or (w, h)
        sx = (w / float(fw)) if fw else 1.0
        sy = (h / float(fh)) if fh else 1.0
        f = self._K_full
        K = np.array([[f[0][0] * sx, 0.0, f[0][2] * sx],
                      [0.0, f[1][1] * sy, f[1][2] * sy],
                      [0.0, 0.0, 1.0]], np.float64)
        from mongla_vision.optics import rectifier_for
        rect, K_rect, note = rectifier_for(K, self._medium)
        self.get_logger().info(f'[PNP  ] {w}x{h}: {note}')
        self._cache[key] = (rect, K_rect)
        return rect, K_rect

    def _refuse(self, msg, reason):
        out = TargetPose()
        out.header = msg.header
        out.ok = False
        out.reason = reason
        self._pub.publish(out)

    def _on_corr(self, msg):
        from mongla_vision.anchor.pose import solve_pnp
        if msg.camera and msg.camera != self._cam:
            # Crossed wiring is not hypothetical here: the two cameras' udev
            # names were bound backwards once, and nothing noticed.
            return self._refuse(
                msg, f'camera {msg.camera!r} is not {self._cam!r}')
        if self._K_full is None:
            return self._refuse(msg, 'no camera_info')
        if msg.optics == TargetCorrespondences.OPTICS_UNSET:
            return self._refuse(msg, 'optics unset')
        n = len(msg.object_points) // 3
        if len(msg.object_points) != n * 3 or len(msg.image_points) != n * 2:
            return self._refuse(msg, 'ragged correspondence arrays')
        if n < 4:
            return self._refuse(msg, 'too few points')

        obj = np.asarray(msg.object_points, np.float64).reshape(n, 3)
        img = np.asarray(msg.image_points, np.float64).reshape(n, 2)
        w = int(msg.image_width) or (self._info or (0, 0))[0]
        h = int(msg.image_height) or (self._info or (0, 0))[1]
        rect, K = self._optics_for(w, h)
        if msg.optics == TargetCorrespondences.OPTICS_RAW and rect is not None:
            img = np.asarray(rect.rectify(img), np.float64).reshape(-1, 2)
        # OPTICS_RECTIFIED: the producer already did it, so rectifying again
        # would apply the map twice -- the reason the field is not a guess.

        tp = solve_pnp(obj, img, K, ambiguity_max=self._ambiguity_max,
                       max_reproj_px=self._max_reproj_px)
        self._pub.publish(to_msg(tp, msg.header))


def to_msg(tp, header=None) -> TargetPose:
    """A `TargetPose` dataclass as the wire message.

    NaN is scrubbed to 0.0 on the fields a consumer might compare without
    checking `ok` first: every comparison against NaN is silently False, so
    `range_m < tol` reads as out of tolerance while a caller testing the other
    way round reads as in it. `pose.py` refuses non-finite poses for the same
    reason at the other end of the pipeline.
    """
    m = TargetPose()
    if header is not None:
        m.header = header
    m.ok = bool(tp.ok)
    m.reason = tp.reason
    m.n_points = int(min(tp.n_points, 65535))

    def f(v):
        v = float(v)
        return v if v == v else 0.0

    m.reproj_px, m.ambiguity = f(tp.reproj_px), f(tp.ambiguity)
    if tp.ok:
        m.yaw_deg, m.pitch_deg, m.roll_deg = (f(tp.yaw_deg), f(tp.pitch_deg),
                                              f(tp.roll_deg))
        m.range_m = f(tp.range_m)
        m.yaw_spread_deg = f(tp.yaw_spread_deg)
        m.pitch_spread_deg = f(tp.pitch_spread_deg)
        m.off_axis_deg = f(tp.off_axis_deg)
    return m


def main(argv=None):
    rclpy.init(args=argv)
    node = PnPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
