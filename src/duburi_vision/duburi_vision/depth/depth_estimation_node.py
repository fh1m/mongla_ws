#!/usr/bin/env python3
"""depth_estimation_node -- monocular depth estimation for detected objects.

Topics:
  in    /duburi/vision/<cam>/image_raw       sensor_msgs/Image
  in    /duburi/vision/<cam>/detections      vision_msgs/Detection2DArray
  out   /duburi/vision/<cam>/vis_range       std_msgs/Float32MultiArray  (one float per detection, 0=far 1=close)
  out   /duburi/vision/<cam>/vis_range_map   sensor_msgs/Image  (float32 depth map, debug only)
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray

_DA2_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_DA2_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
_DA2_SIZE = 364


def _bbox_area_fallback(cx, cy, w, h, img_w, img_h):
    """Estimate proximity from normalised bbox area; no model needed."""
    w_frac = w / max(img_w, 1)
    h_frac = h / max(img_h, 1)
    return float(np.sqrt(w_frac * h_frac))


def _preprocess(frame_bgr):
    """Resize + ImageNet-normalise a BGR frame → (1, 3, H, W) float32 NCHW."""
    rgb     = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (_DA2_SIZE, _DA2_SIZE), interpolation=cv2.INTER_LINEAR)
    arr     = resized.astype(np.float32) / 255.0
    arr     = (arr - _DA2_MEAN) / _DA2_STD
    return arr.transpose(2, 0, 1)[np.newaxis]  # (1, 3, H, W)


class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('duburi_depth_estimator')

        self.declare_parameter('camera',             'forward')
        self.declare_parameter('model_path',         '')
        self.declare_parameter('run_every_n_frames',  3)
        self.declare_parameter('publish_depth_map',  False)
        self.declare_parameter('invert_depth',       True)
        self.declare_parameter('use_tracks',         False)

        cam        = str(self.get_parameter('camera').value or 'forward').strip()
        model_path = str(self.get_parameter('model_path').value or '').strip()
        every_n    = self.get_parameter('run_every_n_frames').value
        self._every_n = max(1, int(every_n) if every_n is not None else 3)
        self._pub_map = bool(self.get_parameter('publish_depth_map').value)
        self._invert  = bool(self.get_parameter('invert_depth').value)
        use_tracks    = self.get_parameter('use_tracks').get_parameter_value().bool_value

        ns = f'/duburi/vision/{cam}'

        img_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._sub_img = self.create_subscription(
            Image, f'{ns}/image_raw', self._on_image, img_qos)
        det_topic     = f'{ns}/tracks' if use_tracks else f'{ns}/detections'
        self._sub_det = self.create_subscription(
            Detection2DArray, det_topic, self._on_detections, 10)

        self._pub_range = self.create_publisher(Float32MultiArray, f'{ns}/vis_range', 10)
        self._pub_depth_map = (
            self.create_publisher(Image, f'{ns}/vis_range_map', 2)
            if self._pub_map else None
        )

        self._bridge      = CvBridge()
        self._frame_count = 0
        self._depth_map: np.ndarray | None = None   # (H, W) float32, 0=far 1=close
        self._depth_shape: tuple | None = None       # (orig_h, orig_w)
        self._ort_session = None
        self._input_name  = ''
        self._output_name = ''
        self._fallback    = True
        self._last_log: float = 0.0
        self._smooth_ranges: list[float] = []       # EMA-smoothed vis_range per detection

        self._try_load_model(model_path)

        mode_tag = f'model={model_path!r}' if not self._fallback else "model='fallback'"
        self.get_logger().info(
            f'[DEPTH] camera={cam!r} {mode_tag} every_n={self._every_n}')

    # ── model loading ──────────────────────────────────────────────────────────

    def _try_load_model(self, model_path: str) -> None:
        if not model_path:
            self.get_logger().warning('[DEPTH] no model_path set — running bbox-area fallback')
            return
        try:
            import onnxruntime as ort   # type: ignore[import-untyped]
            opts = ort.SessionOptions()
            opts.inter_op_num_threads = 2
            opts.intra_op_num_threads = 2
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self._ort_session  = ort.InferenceSession(model_path, sess_options=opts,
                                                      providers=providers)
            self._input_name   = self._ort_session.get_inputs()[0].name
            self._output_name  = self._ort_session.get_outputs()[0].name
            self._fallback     = False
            self.get_logger().info(f'[DEPTH] ONNX session ready: {model_path!r}')
        except ImportError:
            self.get_logger().warning('[DEPTH] onnxruntime not installed — running bbox-area fallback')
        except Exception as exc:
            self.get_logger().warning(
                f'[DEPTH] ONNX load failed ({exc!r}) — bbox-area fallback active')

    # ── image callback ─────────────────────────────────────────────────────────

    def _on_image(self, msg: Image) -> None:
        self._frame_count += 1
        if self._frame_count % self._every_n != 0:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'[DEPTH] cv_bridge decode failed: {exc!r}')
            return

        self._depth_shape = (frame.shape[0], frame.shape[1])

        if self._fallback or self._ort_session is None:
            return

        depth = self._run_onnx(frame)
        if depth is None:
            return

        orig_h, orig_w = frame.shape[:2]
        depth_full       = cv2.resize(depth, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        self._depth_map  = depth_full

        if self._pub_map and self._pub_depth_map is not None:
            try:
                map_msg        = self._bridge.cv2_to_imgmsg(depth_full, encoding='32FC1')
                map_msg.header = msg.header
                self._pub_depth_map.publish(map_msg)
            except Exception:
                pass

    def _run_onnx(self, frame_bgr: np.ndarray) -> np.ndarray | None:
        """Run DA V2-Small ONNX inference. Returns normalised depth (H×W, 0=far 1=close)."""
        try:
            inp  = _preprocess(frame_bgr)
            raw  = self._ort_session.run(  # type: ignore[union-attr]
                [self._output_name], {self._input_name: inp})[0]
            depth = raw.squeeze().astype(np.float32)
            if self._invert:
                depth = -depth
            d_min, d_max = depth.min(), depth.max()
            if d_max - d_min > 1e-6:
                depth = (depth - d_min) / (d_max - d_min)
            else:
                depth[:] = 0.5
            return depth
        except Exception as exc:
            self.get_logger().error(f'[DEPTH] ONNX inference error: {exc!r}')
            return None

    # ── detections callback ────────────────────────────────────────────────────

    def _on_detections(self, msg: Detection2DArray) -> None:
        ranges = []
        for det in msg.detections:
            bbox   = det.bbox
            cx, cy = float(bbox.center.position.x), float(bbox.center.position.y)
            w,  h  = float(bbox.size_x), float(bbox.size_y)

            if self._fallback or self._depth_map is None:
                img_w = self._depth_shape[1] if self._depth_shape else max(int(cx * 2), 1)
                img_h = self._depth_shape[0] if self._depth_shape else max(int(cy * 2), 1)
                val   = _bbox_area_fallback(cx, cy, w, h, img_w, img_h)
            else:
                val = self._median_depth_in_bbox(cx, cy, w, h)

            ranges.append(float(np.clip(val, 0.0, 1.0)))

        # Temporal EMA smoothing: alpha=0.40, reset on detection count change
        _alpha = 0.40
        if len(ranges) == len(self._smooth_ranges):
            self._smooth_ranges = [
                _alpha * r + (1.0 - _alpha) * s
                for r, s in zip(ranges, self._smooth_ranges)
            ]
        else:
            self._smooth_ranges = list(ranges)

        out      = Float32MultiArray()
        out.data = list(self._smooth_ranges)
        now = time.monotonic()
        if now - self._last_log >= 2.0:
            self._last_log = now
            mode = 'ONNX' if not self._fallback else 'fallback'
            vals = ', '.join(f'{r:.2f}' for r in ranges)
            self.get_logger().info(f'[VISRNG] {mode}  {len(ranges)} dets  [{vals}]')
        self._pub_range.publish(out)

    def _median_depth_in_bbox(self, cx: float, cy: float, w: float, h: float) -> float:
        dmap    = self._depth_map
        assert dmap is not None  # caller guards with `self._depth_map is None` check
        dh, dw  = dmap.shape[:2]
        x1 = int(np.clip(cx - w / 2, 0, dw - 1))
        x2 = int(np.clip(cx + w / 2, 0, dw - 1))
        y1 = int(np.clip(cy - h / 2, 0, dh - 1))
        y2 = int(np.clip(cy + h / 2, 0, dh - 1))
        if x2 <= x1 or y2 <= y1:
            return float(dmap[int(np.clip(cy, 0, dh - 1)), int(np.clip(cx, 0, dw - 1))])
        return float(np.median(dmap[y1:y2, x1:x2]))


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
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


if __name__ == '__main__':
    main()
    sys.exit(0)
