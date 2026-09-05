#!/usr/bin/env python3
"""lock_node -- run the ladder, publish one target position.

    detections ─┐
    image_raw ──┼─→ follower (LK, fast) ─┐
                └─→ anchor  (XFeat)  ────┼─→ arbitrate ─→ /lock
                                          │
                                    authority decays from the
                                    last REAL detection

Publishes a `Detection2DArray` on `<ns>/lock` carrying ONE box: whatever rung
can still stand behind an answer. `class_id` names the rung, `score` is the
arbiter's confidence (rung trust x time decay), so a consumer that already
knows how to read detections needs no new message type and no new units.

WHY A SEPARATE TOPIC AND NOT `/detections`. Publishing a followed or anchored
box onto the detector's topic would make it indistinguishable from something
the detector actually saw -- every downstream consumer, the HUD included, would
report a detection that never happened. The rung name and the separate topic
are what keep "the vehicle has a position" from becoming "the vehicle saw the
target".

The heavy rungs run on a WORKER THREAD off the subscription callbacks: the
anchor is 33 ms at 320x240 and blocking a callback with it would back the
executor up behind the very frames it is meant to bridge.
"""
import os

os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from duburi_vision import qos as _qos
from duburi_vision.detection.detector import Detection
from duburi_vision.detection.messages import detections_to_array
from duburi_vision.tracking.follower import Follower
from duburi_vision.tracking.lock_state import (
    FULL_AUTHORITY_S, ZERO_AUTHORITY_S, Rung, arbitrate)


class LockNode(Node):
    def __init__(self, node_name: str = 'duburi_lock', *,
                 parameter_overrides=None):
        super().__init__(node_name,
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter('camera', 'forward')
        self.declare_parameter('target_class', '')
        self.declare_parameter('follow', True)
        self.declare_parameter('anchor', False)
        self.declare_parameter('anchor_model', '')
        self.declare_parameter('anchor_hz', 8.0)
        self.declare_parameter('full_authority_s', FULL_AUTHORITY_S)
        self.declare_parameter('zero_authority_s', ZERO_AUTHORITY_S)

        cam = str(self.get_parameter('camera').value)
        ns = f'/duburi/vision/{cam}'
        self._cls = str(self.get_parameter('target_class').value).strip()
        self._full = float(self.get_parameter('full_authority_s').value)
        self._zero = float(self.get_parameter('zero_authority_s').value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._gray = None
        self._header = None
        self._det_box = None
        self._det_conf = 0.0
        self._det_t = 0.0
        self._fresh = threading.Event()

        self._follower = Follower() if bool(
            self.get_parameter('follow').value) else None
        self._anchor = None
        self._anchor_pose = None
        self._anchor_next = 0.0
        self._anchor_period = 1.0 / max(
            float(self.get_parameter('anchor_hz').value), 0.5)
        if bool(self.get_parameter('anchor').value):
            self._build_anchor()

        self._pub = self.create_publisher(Detection2DArray, f'{ns}/lock',
                                          _qos.DETECTIONS)
        # Shared profiles, never a hand-rolled QoS: a RELIABLE/BEST_EFFORT
        # mismatch is answered by rclpy with one WARN and then silence --
        # clean launch, healthy nodes, zero frames. `duburi_vision.qos` owns
        # these topics and a contract test enforces it.
        self.create_subscription(Detection2DArray, f'{ns}/detections',
                                 self._on_det, _qos.DETECTIONS)
        self.create_subscription(Image, f'{ns}/image_raw',
                                 self._on_img, _qos.IMAGE)
        threading.Thread(target=self._loop, daemon=True).start()
        self.create_timer(5.0, self._log_health)
        self._n_by_rung = {r: 0 for r in Rung}

        self.get_logger().info(
            f'[LOCK ] {cam}: follow={self._follower is not None} '
            f'anchor={self._anchor is not None} '
            f'authority {self._full:.2f}->{self._zero:.2f}s  -> {ns}/lock')

    def _build_anchor(self):
        """Optional by design: a missing ONNX must cost the anchor rung, not
        the node -- the follower is still worth running without it."""
        try:
            from duburi_vision.anchor.anchor import Anchor
            from duburi_vision.anchor.xfeat_onnx import XFeatONNX
            import glob
            p = str(self.get_parameter('anchor_model').value).strip()
            if not p:
                c = sorted(glob.glob(os.path.expanduser(
                    '~/hailo_models/xfeat_*.onnx')))
                p = c[0] if c else ''
            if not p:
                raise FileNotFoundError('no xfeat_*.onnx found')
            self._anchor = Anchor(XFeatONNX(p, top_k=1024))
            self.get_logger().info(f'[LOCK ] anchor backend {os.path.basename(p)}')
        except Exception as exc:
            self.get_logger().warning(
                f'[LOCK ] anchor DISABLED: {type(exc).__name__}: {exc} '
                f'-- the follower rung still runs')

    # -- inputs ------------------------------------------------------------- #
    def _on_img(self, msg):
        try:
            g = cv2.cvtColor(self._bridge.imgmsg_to_cv2(msg, 'bgr8'),
                             cv2.COLOR_BGR2GRAY)
        except Exception:
            return
        with self._lock:
            self._gray = g
            self._header = msg.header
        self._fresh.set()

    def _on_det(self, msg):
        best = None
        for d in msg.detections:
            if not d.results:
                continue
            h = d.results[0]
            score = float(h.hypothesis.score if hasattr(h, 'hypothesis')
                          else h.score)
            name = str(h.hypothesis.class_id if hasattr(h, 'hypothesis')
                       else h.id)
            if self._cls and name != self._cls:
                continue
            if score <= 0.0:          # a COASTED track, not an observation
                continue
            if best is None or score > best[0]:
                b = d.bbox
                hw, hh = b.size_x * 0.5, b.size_y * 0.5
                best = (score, (b.center.position.x - hw,
                                b.center.position.y - hh,
                                b.center.position.x + hw,
                                b.center.position.y + hh))
        with self._lock:
            if best is None:
                self._det_box, self._det_conf = None, 0.0
            else:
                self._det_conf, self._det_box = best
                self._det_t = time.monotonic()

    # -- the ladder --------------------------------------------------------- #
    def _loop(self):
        while rclpy.ok():
            if not self._fresh.wait(0.5):
                continue
            self._fresh.clear()
            with self._lock:
                gray = self._gray
                header = self._header
                det_box, det_conf, det_t = (self._det_box, self._det_conf,
                                            self._det_t)
            if gray is None:
                continue
            now = time.monotonic()

            fb = fc = None
            if self._follower is not None:
                if det_box is not None:
                    # Reseed on every accepted detection: this is what keeps
                    # the follower's drift bounded to a single gap.
                    self._follower.reset(gray, det_box)
                elif self._follower.active:
                    r = self._follower.step(gray)
                    if r.ok:
                        fb, fc = r.xyxy, r.confidence

            ab = ac = None
            if self._anchor is not None:
                if det_box is not None and not self._anchor.has_reference:
                    self._anchor.snap(gray, roi=det_box)
                elif (self._anchor.has_reference
                      and now >= self._anchor_next):
                    self._anchor_next = now + self._anchor_period
                    self._anchor_pose = self._anchor.locate(gray)
                p = self._anchor_pose
                if p is not None and p.ok and p.corners is not None:
                    q = np.asarray(p.corners, np.float32)
                    ab = (float(q[:, 0].min()), float(q[:, 1].min()),
                          float(q[:, 0].max()), float(q[:, 1].max()))
                    ac = p.confidence

            st = arbitrate(now=now, last_detection_t=det_t,
                           detection=det_box, detection_conf=det_conf,
                           follow=fb, follow_conf=fc or 0.0,
                           anchor=ab, anchor_conf=ac or 0.0,
                           full_s=self._full, zero_s=self._zero)
            self._n_by_rung[st.rung] = self._n_by_rung.get(st.rung, 0) + 1
            self._publish(st, header)

    def _publish(self, st, header):
        dets = []
        if st.have_target:
            x1, y1, x2, y2 = st.xyxy
            dets.append(Detection(class_id=0, class_name=st.rung.value,
                                  score=float(st.confidence),
                                  xyxy=(x1, y1, x2, y2)))
        if header is not None:
            self._pub.publish(detections_to_array(dets, header))

    def _log_health(self):
        n = self._n_by_rung
        tot = max(sum(n.values()), 1)
        self.get_logger().info(
            '[LOCK ] ' + '  '.join(
                f'{r.value}={100.0 * n.get(r, 0) / tot:.0f}%' for r in Rung))
        for r in Rung:
            n[r] = 0


def main():
    rclpy.init()
    node = LockNode()
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
