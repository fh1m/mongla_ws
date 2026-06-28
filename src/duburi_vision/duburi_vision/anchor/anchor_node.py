#!/usr/bin/env python3
"""anchor_node -- XFeat/LighterGlue geometric lock against a snapped reference.

Topics
  in    /duburi/vision/<cam>/image_raw       sensor_msgs/Image
  out   /duburi/vision/<cam>/anchor_error     geometry_msgs/Vector3  (x=tx,y=ty,z=theta)
  out   /duburi/vision/<cam>/anchor_state     std_msgs/String        (IDLE|LOCKED|LOST)
  out   /duburi/vision/<cam>/anchor_conf      std_msgs/Float32       (RANSAC inliers)
  out   /duburi/vision/<cam>/anchor_ref       sensor_msgs/Image      (stored reference, debug)

Services (std_srvs/Trigger)
  ~/snap   -- capture the NEXT received frame as the reference
  ~/clear  -- drop the reference (next snap starts fresh)

Mirrors detector_node: subscriber stays light, the heavy match runs on a
background worker thread off the ROS executor (single-slot, drop-stale), and
every publish is gated on the matcher being loaded + a reference being set.
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import queue as _queue
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg   import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg      import String, Float32
from std_srvs.srv      import Trigger
from duburi_interfaces.srv import AnchorRef
from vision_msgs.msg   import Detection2DArray
from cv_bridge         import CvBridge

from duburi_vision.anchor.references import save_reference, load_reference
from duburi_vision.anchor.crop_gate import qualifying_bbox
from duburi_vision.anchor.overlay import draw_match_overlay

# How long to wait for a qualifying detection before falling back to a
# whole-frame snap (seconds).
_SNAP_DETECT_TIMEOUT_S = 3.0

STATE_IDLE   = 'IDLE'
STATE_LOCKED = 'LOCKED'
STATE_LOST   = 'LOST'

# Rate-limit for the (large) reference-image debug publish.
_REF_PUB_MIN_DT = 0.5


class AnchorNode(Node):
    def __init__(self):
        super().__init__('duburi_anchor')

        self.declare_parameter('cam',         'forward')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('min_inliers', 12)
        self.declare_parameter('top_k',       2048)
        self.declare_parameter('device',      'cuda:0')
        self.declare_parameter('skip_frames', 3)
        self.declare_parameter('min_conf',    0.1)

        self._cam = str(self.get_parameter('cam').value).strip() or 'forward'
        ns_in  = (str(self.get_parameter('image_topic').value).strip()
                  or f'/duburi/vision/{self._cam}/image_raw')
        ns_out = f'/duburi/vision/{self._cam}'

        self._min_inliers = int(self.get_parameter('min_inliers').value)
        self._skip        = max(int(self.get_parameter('skip_frames').value), 0)
        self._bridge      = CvBridge()

        # Publishers.
        self._pub_err   = self.create_publisher(Vector3, f'{ns_out}/anchor_error', 10)
        self._pub_state = self.create_publisher(String,  f'{ns_out}/anchor_state', 10)
        self._pub_conf  = self.create_publisher(Float32, f'{ns_out}/anchor_conf',  10)
        self._pub_ref   = self.create_publisher(Image,   f'{ns_out}/anchor_ref',   2)

        # Services (absolute, camera-namespaced so the manager client name is
        # deterministic regardless of this node's runtime name). snap = AnchorRef
        # (name/load fields for disk save+load); clear = bare Trigger.
        self.create_service(AnchorRef, f'{ns_out}/anchor_snap',  self._handle_snap)
        self.create_service(Trigger,   f'{ns_out}/anchor_clear', self._handle_clear)

        # Matcher loads async so the subscriber + services come up immediately
        # (mirrors detector_node's async single-model load).
        self._matcher = None
        self._load_error = None              # str once the model load FAILED (vs None=loading)
        self._snap_pending = threading.Event()
        self._pending_name = ''              # disk name to save the captured ref under
        self._snap_target  = ''              # detection-gated snap: class to wait for
        self._snap_conf    = 0.0             #   min score
        self._snap_err     = 0.0             #   centring tolerance px (<=0 = no gate)
        self._snap_deadline = 0.0            #   monotonic 3 s fallback deadline
        self._ref_frame    = None            # last captured reference (full BGR) for debug pub
        self._ref_bbox     = None            # crop bbox of the reference (drawn on the inset)
        self._last_ref_pub = 0.0
        threading.Thread(target=self._load_matcher_async, daemon=True).start()

        self._sub = self.create_subscription(Image, ns_in, self._on_image, 5)
        # Detections (for snap-at-detection cropping). Cached latest only.
        self._det_lock  = threading.Lock()
        self._det_array = None
        self.create_subscription(
            Detection2DArray, f'{ns_out}/detections', self._on_detections, 5)

        # Single-slot worker: subscriber drops stale, worker always matches the
        # newest frame -- same decouple-and-drop pattern as detector_node.
        self._frame_q: _queue.SimpleQueue = _queue.SimpleQueue()
        self._frame_i = 0
        threading.Thread(target=self._match_loop, daemon=True).start()

        self._frames = 0
        self._locks  = 0
        self.create_timer(2.0, self._log_health)
        self._last_log = time.monotonic()

        self.get_logger().info(
            f'[ANCHOR] subscribed {ns_in!r} -> {ns_out}/anchor_error '
            f'(state/conf/ref) cam={self._cam!r} min_inliers={self._min_inliers} '
            f'skip={self._skip}')
        self._publish_state(STATE_IDLE)

    # ---- async model load ---------------------------------------------- #
    def _load_matcher_async(self):
        from duburi_vision.anchor.xfeat import XFeatMatcher
        try:
            self._matcher = XFeatMatcher(
                top_k=int(self.get_parameter('top_k').value),
                device=str(self.get_parameter('device').value),
                min_conf=float(self.get_parameter('min_conf').value),
                logger=self.get_logger())
        except Exception as exc:   # noqa: BLE001
            self._load_error = str(exc)
            self.get_logger().fatal(
                f'[ANCHOR] XFeatMatcher init FAILED: {exc} -- pre-download the '
                f'XFeat/LighterGlue weights on the Jetson (no pool internet)')

    # ---- services ------------------------------------------------------ #
    def _handle_snap(self, req, resp):
        if self._matcher is None or not self._matcher.is_loaded():
            resp.success = False
            resp.message = (f'anchor model FAILED to load: {self._load_error} '
                            f'(pre-download XFeat weights on the Jetson)'
                            if self._load_error else 'matcher still loading')
            return resp

        name = str(getattr(req, 'name', '') or '').strip()

        # load=true: read the named PNG (+bbox sidecar) from disk and set it as
        # the reference NOW -- e.g. anchor_align('hole') reloading a pre-run
        # snapshot at competition. A crop reference keeps its bbox geometry.
        if getattr(req, 'load', False):
            if not name:
                resp.success = False
                resp.message = 'load requested but no name given'
                return resp
            img, bbox = load_reference(name)
            if img is None:
                resp.success = False
                resp.message = f'reference {name!r} not found on disk'
                return resp
            ok = self._matcher.set_reference(img, bbox=bbox)
            if ok:
                self._ref_frame = img.copy()
                self._ref_bbox  = bbox
                self._publish_reference(self._make_header())
            resp.success = bool(ok)
            resp.message = (f'loaded reference {name!r}' if ok
                            else f'reference {name!r} rejected (too few keypoints / '
                                 f'low texture)')
            self.get_logger().info(f'[ANCHOR] load {name!r} -> {resp.success}')
            return resp

        # load=false: capture the next live frame (and save it if a name was given).
        self._pending_name = name
        self._snap_target  = str(getattr(req, 'target_class', '') or '').strip()
        self._snap_conf    = float(getattr(req, 'conf', 0.0) or 0.0)
        self._snap_err     = float(getattr(req, 'err_px', 0.0) or 0.0)
        self._snap_deadline = time.monotonic() + _SNAP_DETECT_TIMEOUT_S
        self._snap_pending.set()
        resp.success = True
        if self._snap_target:
            resp.message = (f'waiting <=3s for {self._snap_target!r} (conf>={self._snap_conf}) '
                            f'to crop' + (f' -> references/{name}.png' if name else ''))
        else:
            resp.message = (f'capturing next frame -> references/{name}.png'
                            if name else 'capturing next frame (in-memory)')
        self.get_logger().info('[ANCHOR] snap requested -- ' + resp.message)
        return resp

    def _handle_clear(self, _req, resp):
        if self._matcher is not None:
            self._matcher.clear_reference()
        self._ref_frame = None
        self._ref_bbox  = None
        self._publish_state(STATE_IDLE)
        resp.success = True
        resp.message = 'reference cleared'
        self.get_logger().info('[ANCHOR] reference cleared')
        return resp

    # ---- detection-gated crop snap ------------------------------------- #
    def _on_detections(self, msg: Detection2DArray):
        with self._det_lock:
            self._det_array = msg

    def _qualifying_bbox(self, frame):
        """The padded crop bbox for the pending snap target, or None this tick."""
        with self._det_lock:
            arr = self._det_array
        if arr is None:
            return None
        h, w = frame.shape[:2]
        return qualifying_bbox(arr.detections, w, h, self._snap_target,
                               self._snap_conf, self._snap_err)

    # ---- image path ---------------------------------------------------- #
    def _on_image(self, msg: Image):
        # Frame-skip to hold ~image_rate / (skip+1); single-slot drop-stale.
        self._frame_i += 1
        if self._skip and (self._frame_i % (self._skip + 1)) != 0:
            return
        while not self._frame_q.empty():
            try:
                self._frame_q.get_nowait()
            except _queue.Empty:
                break
        self._frame_q.put_nowait(msg)

    def _match_loop(self):
        while rclpy.ok():
            try:
                msg = self._frame_q.get(timeout=0.5)
            except _queue.Empty:
                continue
            matcher = self._matcher
            if matcher is None or not matcher.is_loaded():
                continue
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:   # noqa: BLE001
                self.get_logger().warning(f'[ANCHOR] cv_bridge decode failed: {exc!r}')
                continue

            # Pending snap consumes a frame as the reference.
            if self._snap_pending.is_set():
                bbox = None
                if self._snap_target:
                    # Detection-gated crop snap: wait (up to 3 s) for a qualifying
                    # detection on THIS frame; crop it. Fall back to whole frame
                    # at the deadline. Keep waiting otherwise (don't clear).
                    bbox = self._qualifying_bbox(frame)
                    if bbox is None and time.monotonic() < self._snap_deadline:
                        continue   # no match yet, still within 3 s -> next frame
                    if bbox is None:
                        self.get_logger().info(
                            f'[ANCHOR] no {self._snap_target!r} in 3s -- whole-frame snap')
                self._snap_pending.clear()
                name = self._pending_name
                self._pending_name = ''
                self._snap_target = ''
                if matcher.set_reference(frame, bbox=bbox):
                    self._ref_frame = frame.copy()
                    self._ref_bbox  = bbox
                    if name:
                        path = save_reference(name, frame, bbox=bbox)
                        self.get_logger().info(
                            f'[ANCHOR] reference saved -> {path}' if path
                            else f'[ANCHOR] WARN: failed to save reference {name!r}')
                    self._publish_reference(msg.header)
                else:
                    # set_reference rejected the frame (too few keypoints) -- say so
                    # instead of silently leaving the old/empty reference in place.
                    self.get_logger().warning(
                        '[ANCHOR] snap rejected -- low-texture frame/crop '
                        '(aim at a textured target / get closer)')
                continue

            if not matcher.has_reference():
                continue

            err = matcher.match(frame)
            self._frames += 1
            if err is None:
                self._publish_state(STATE_LOST)
                continue

            self._publish_error(err)
            if err.n_inliers >= self._min_inliers:
                self._locks += 1
                self._publish_state(STATE_LOCKED)
            else:
                self._publish_state(STATE_LOST)

            # Keep the reference inset fresh on the HUD even while locked.
            if self._ref_frame is not None and \
                    (time.monotonic() - self._last_ref_pub) >= _REF_PUB_MIN_DT:
                self._publish_reference(msg.header)

    # ---- publishers (gated) -------------------------------------------- #
    def _publish_error(self, err):
        v = Vector3()
        v.x = float(err.tx_px); v.y = float(err.ty_px); v.z = float(err.theta_rad)
        if rclpy.ok():
            self._pub_err.publish(v)
            c = Float32(); c.data = float(err.n_inliers)
            self._pub_conf.publish(c)

    def _publish_state(self, state: str):
        if rclpy.ok():
            s = String(); s.data = state
            self._pub_state.publish(s)

    def _make_header(self):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self._cam
        return h

    def _publish_reference(self, header):
        if self._ref_frame is None or self._pub_ref.get_subscription_count() == 0:
            return
        last = self._matcher.last_match() if self._matcher is not None else None
        overlaid = draw_match_overlay(
            self._ref_frame, last, self._ref_bbox, self._min_inliers)
        try:
            img = self._bridge.cv2_to_imgmsg(overlaid, encoding='bgr8')
            img.header = header
            self._pub_ref.publish(img)
            self._last_ref_pub = time.monotonic()
        except Exception as exc:   # noqa: BLE001
            if rclpy.ok():
                self.get_logger().warning(f'[ANCHOR] ref publish failed: {exc!r}')

    def _log_health(self):
        now = time.monotonic()
        dt  = max(now - self._last_log, 1e-3)
        hz  = self._frames / dt
        lock_pct = 100.0 * self._locks / max(self._frames, 1)
        has_ref = self._matcher is not None and self._matcher.has_reference()
        self.get_logger().debug(
            f'[ANCHOR] match_hz={hz:4.1f}  locked={lock_pct:3.0f}%  ref={has_ref}')
        self._frames = 0; self._locks = 0; self._last_log = now


def main():
    rclpy.init()
    node = AnchorNode()
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
