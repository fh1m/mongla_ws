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

from duburi_vision.optics import RefractiveRectifier
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

try:                                   # noqa: SIM105
    from duburi_interfaces.msg import TargetPose
except ImportError:                    # message not built in this workspace
    # The LADDER must survive it. A pose is an addition to what this node does,
    # not a precondition, so an unbuilt interface costs the 6-DoF topic and
    # nothing else -- the follower and anchor rungs still publish `/lock`.
    TargetPose = None

from duburi_vision import qos as _qos
from duburi_vision.stamps import capture_monotonic
from duburi_vision.detection.detector import Detection
from duburi_vision.detection.messages import detections_to_array
from duburi_vision.tracking.follower import Follower
from duburi_vision.tracking.lock_state import (
    FULL_AUTHORITY_S, ZERO_AUTHORITY_S, Rung, arbitrate)


def header_for(rung, *, detection, frame, anchor):
    """The stamp of the frame the WINNING RUNG actually observed.

    Each rung answers from a different instant. The follower ran on this frame;
    the anchor ran on whichever frame it last evaluated, up to its own period
    ago (0.333 s at 3 Hz); the detector finished on whichever frame it last
    got through. Publishing all three under the newest header makes every rung
    claim the freshness of the fastest one -- and it does so ONLY on the lower
    rungs, so it looks correct exactly while a detection is present and lies
    exactly when the ladder is doing its job.

    Falls back to the current frame when a rung has no header yet: a stamp that
    is merely too NEW makes the consumer act with less authority than it could,
    which is the safe direction. There is no correct answer to hand it, and
    `None` would suppress the publish entirely.
    """
    return {Rung.DETECTION: detection,
            Rung.FOLLOW: frame,
            Rung.ANCHOR: anchor}.get(rung, frame) or frame


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
        # 3 Hz, not 8. Measured on the Pi with the full stack live, and the
        # cost is LATENCY rather than throughput -- detection rate is unchanged
        # at every setting, but the anchor's 33 ms bursts delay the image
        # pipeline:
        #
        #     ladder OFF          det 49.3 Hz   frame age 16.1 ms   idle 82.4 %
        #     follower only       det 50.6 Hz             18.7 ms        75.3 %
        #     + anchor @ 8 Hz     det 50.7 Hz             22.8 ms        65.5 %
        #     + anchor @ 3 Hz     det 50.6 Hz             18.4 ms        66.8 %
        #
        # 8 Hz costs +42 % of frame age for NO extra coverage: the anchor is the
        # long-horizon rung, called on when the follower has already given up,
        # and the p99 gap it exists to cover is 2.418 s. 3 Hz samples that
        # seven times over.
        # PUBLISH RATE -- OFF BY DEFAULT, and the default is the point.
        #
        # The ladder tracks on every camera frame (LK wants the smallest
        # inter-frame motion) and publishes on every frame: MEASURED 132.4 Hz
        # on the vehicle against detections at 30.5 Hz.
        #
        # ⛔ THAT IS NOT WASTE, and an earlier version of this comment said it
        # was. The reasoning "nobody samples above 20-50 Hz" describes the
        # HOST loop, which is a limitation of the un-ported vision uplink --
        # not the architecture. THE CONTROL LOOP IS 500 Hz ON THE SROT BOARD.
        # Once LANDING_TARGET ingest lands the board closes the visual loop at
        # that rate and wants bearings as fresh and as often as we can send
        # them, so capping this publisher would throttle the exact stream the
        # uplink exists to feed.
        #
        # The knob stays for a CPU-bound bench (the ladder costs 16 points of
        # idle: 53.4 % -> 37.4 %), but 0 = publish every frame is the default
        # and the design intent. A RUNG CHANGE is never delayed even when a
        # limit is set -- that transition is the information.
        self.declare_parameter('publish_hz', 0.0)
        self.declare_parameter('anchor_hz', 3.0)
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
        self._det_header = None
        self._anchor_header = None
        self._stamp_warned = False
        self._pose_was_ok = None
        self._pose_pub_t = 0.0
        # 6-DoF: needs K at the BACKEND's resolution (the frame H is fitted in)
        # and the target's true width. Both absent by default -- no width means
        # no metric answer, and guessing one would make every range wrong by a
        # constant nobody could see.
        self._K = None
        self._target_w_m = float(
            self.declare_parameter('target_width_m', 0.0).value or 0.0)
        # An explicit parameter WINS -- a measured prop beats a rulebook
        # nominal, which is what SAUVC's +/- 5 % tolerance exists to allow for.
        # Otherwise fall back to the committed table, keyed by the class we are
        # locking onto. Without this the default is 0.0 and the 6-DoF branch
        # publishes `ok=false, reason='target_width_m unset'` forever: the
        # metric path has never once run on the vehicle.
        if self._target_w_m <= 0.0 and self._cls:
            from duburi_vision.target_geometry import width_for, describe
            w = width_for(self._cls)
            if w > 0.0:
                self._target_w_m = w
                d = describe(self._cls)
                self.get_logger().info(
                    f"[LOCK ] target width {w:.4f} m for {self._cls!r} "
                    f"({d.get('boxes', '?')}; {d.get('source', 'no source')})")
            else:
                self.get_logger().warn(
                    f'[LOCK ] no committed width for {self._cls!r}, and '
                    f'target_width_m is unset -- the 6-DoF pose will refuse. '
                    f'Add it to config/target_geometry.yaml or pass '
                    f'target_width_m.')
        # MEDIUM. A flat port is not a pinhole: the ray from a point at water
        # angle tw leaves at air angle ta with sin(ta) = n*sin(tw), so the
        # effective focal length grows with field angle -- 10.6 % centre to
        # corner on this camera, and the pose path was reading the AIR K with
        # raw pixels. Ray-traced against the shipped model, the error at the
        # frame corner is +34 px at 0.3 m and +46 px at 2 m. Default 'water'
        # matches `flow_node`'s `flow_medium`, and the choice is LOGGED at
        # startup because it silently rescales every range this node publishes.
        #
        # Placed AFTER the width lookup on purpose: `test_target_geometry`
        # asserts `width_for(self._cls)` lands within 1500 chars of
        # `target_width_m`, and splitting that block pushed it out of range.
        self._medium = str(
            self.declare_parameter('medium', 'water').value or 'water').lower()
        if self._medium not in ('water', 'air'):
            raise ValueError(
                f"medium must be 'water' or 'air', got {self._medium!r}")
        self._rect = None
        self._K_rect = None
        self._pub_min_dt = 0.0
        self._last_pub_t = 0.0
        self._last_pub_rung = None
        self._pub_pose = (
            self.create_publisher(TargetPose, f'{ns}/target_pose',
                                  _qos.DETECTIONS)
            if TargetPose is not None else None)
        if TargetPose is None:
            self.get_logger().warn(
                '[LOCK ] duburi_interfaces/TargetPose not built -- the 6-DoF '
                'target pose will not be published (the ladder is unaffected). '
                'Rebuild duburi_interfaces to enable it.')
        self.create_subscription(CameraInfo, f'{ns}/camera_info',
                                 self._on_info, 10)
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
        # Read it, or the parameter is decoration -- the "declared and unread"
        # defect this package has produced four times. `/target_pose` already
        # got this treatment (see the note at the anchor publish: 96.5 Hz for a
        # 3 Hz quantity); this is the same fix for `/lock`.
        _phz = float(self.get_parameter('publish_hz').value)
        self._pub_min_dt = (1.0 / _phz) if _phz > 0.0 else 0.0
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
                # DUBURI_HEF_DIR FIRST, then the historical default. The
                # detector already resolves its models that way
                # (`detection/yolo.py`), and hardcoding one of the two paths
                # here made the model directory a truth with two copies: point
                # DUBURI_HEF_DIR somewhere else and the detector follows while
                # the anchor silently does not, losing the rung with one WARN
                # and no error.
                roots = [d for d in (os.environ.get('DUBURI_HEF_DIR', '').strip(),
                                     '~/hailo_models') if d]
                c = []
                for r in roots:
                    c = sorted(glob.glob(os.path.join(
                        os.path.expanduser(r), 'xfeat_*.onnx')))
                    if c:
                        break
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

    def _on_info(self, msg):
        """K, scaled to the anchor backend's resolution.

        The homography and the correspondences live in backend pixels, so a
        full-resolution K would scale every recovered angle -- silently, with
        no error and a plausible number. Same trap as the anchor's ROI.
        """
        k = list(msg.k)
        if len(k) < 9 or k[0] <= 0.0 or self._anchor is None:
            return
        sx = self._anchor._be.w / float(msg.width or 1)
        sy = self._anchor._be.h / float(msg.height or 1)
        K = np.array([[k[0] * sx, 0.0, k[2] * sx],
                      [0.0, k[4] * sy, k[5] * sy],
                      [0.0, 0.0, 1.0]], np.float64)
        # CameraInfo arrives with EVERY FRAME, so rebuilding here would
        # reallocate the rectifier and re-log at camera rate. Only a genuine
        # change of intrinsics is an event; anything else is the same K again.
        if self._K is not None and np.array_equal(K, self._K):
            return
        self._K = K

        # ⛔ THE RECTIFIED POINTS NEED THE RECTIFIED K. `rectify` re-projects
        # each ray through `f_ref`, which defaults to `fx * n` -- so a point at
        # water angle tw lands at radius `f_ref * tan(tw)`, a true pinhole of
        # focal `f_ref`. Feeding those points to PnP with the AIR K would leave
        # a clean 1/n scale error: every range 33 % short, with a plausible
        # number and no warning. Same class as the fx!=fy aspect defect the
        # rectifier's own docstring records.
        if self._medium == 'water':
            fx, fy = float(self._K[0][0]), float(self._K[1][1])
            cx, cy = float(self._K[0][2]), float(self._K[1][2])
            self._rect = RefractiveRectifier(fx, fy, cx, cy)
            self._K_rect = np.array(
                [[self._rect.f_ref, 0.0, cx],
                 [0.0, self._rect.f_ref_y, cy],
                 [0.0, 0.0, 1.0]], np.float64)
            self.get_logger().info(
                f'[LOCK ] medium=water: flat-port rectification ON, '
                f'f_air={fx:.1f} -> f_ref={self._rect.f_ref:.1f} px '
                f'(centre f_eff {self._rect.local_focal_px(0.0):.1f}, '
                f'corner {self._rect.local_focal_px(min(cx, cy)):.1f})')
        else:
            self._rect, self._K_rect = None, self._K
            self.get_logger().info(
                '[LOCK ] medium=air: no refraction correction (identity). '
                'Ranges are only valid OUT of water.')

    def _publish_pose(self, pose, header):
        """6-DoF from the anchor's inliers, when calibrated and sized.

        Published on its OWN topic, never folded into `/lock`: the ladder's box
        is what the control loop steers on, and a pose is a different claim
        with a different failure mode. Absence of this message means no pose --
        there is no "invalid" flag to misread.
        """
        if self._pub_pose is None:
            return
        from duburi_vision.anchor.pose import target_pose
        m = TargetPose()
        if header is not None:
            m.header = header
        if (self._K is None or self._target_w_m <= 0.0 or pose is None
                or not pose.ok):
            m.ok = False
            m.reason = ('no camera_info' if self._K is None else
                        'target_width_m unset' if self._target_w_m <= 0.0
                        else 'no anchor')
            self._pub_pose.publish(m)
            return
        roi = self._anchor.reference_roi
        wh = ((roi[2] - roi[0], roi[3] - roi[1]) if roi
              else (self._anchor._be.w, self._anchor._be.h))
        # Both point sets come from the SAME camera, so both are rectified --
        # rectifying only the live side would compare water geometry against
        # air geometry and put the whole error into the pose.
        ref_pts, live_pts = pose.ref_pts, pose.live_pts
        if self._rect is not None:
            ref_pts = self._rect.rectify(ref_pts)
            live_pts = self._rect.rectify(live_pts)
        tp = target_pose(ref_pts, live_pts, self._K_rect,
                         width_m=self._target_w_m, ref_size_px=wh)
        m.ok = bool(tp.ok)
        m.reason = tp.reason
        m.n_points = int(min(tp.n_points, 65535))
        m.reproj_px = float(tp.reproj_px if tp.reproj_px == tp.reproj_px else 0.0)
        m.ambiguity = float(tp.ambiguity if tp.ambiguity == tp.ambiguity else 0.0)
        if tp.ok:
            m.yaw_deg, m.pitch_deg, m.roll_deg = (float(tp.yaw_deg),
                                                  float(tp.pitch_deg),
                                                  float(tp.roll_deg))
            m.range_m = float(tp.range_m)
            m.yaw_spread_deg = float(tp.yaw_spread_deg)
            m.pitch_spread_deg = float(tp.pitch_spread_deg)
            m.off_axis_deg = float(tp.off_axis_deg)
        self._pub_pose.publish(m)

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
        # THE DECAY CLOCK. `_det_t` is what `arbitrate` measures authority
        # against, so it must be when the frame was SEEN, not when the message
        # landed here. Arrival time under-reports every gap by the whole
        # capture->inference->transport chain -- measured at 32 ms median,
        # 48 p95 -- always in the flattering direction, so the ladder holds
        # full authority slightly past the point the measurement justifies.
        t, why = capture_monotonic(msg.header)
        if why and not self._stamp_warned:
            self._stamp_warned = True
            self.get_logger().warn(
                f'[LOCK ] detection {why} -- the authority decay is measuring '
                f'age since ARRIVAL, not since capture. Check use_sim_time.')
        with self._lock:
            if best is None:
                self._det_box, self._det_conf = None, 0.0
            else:
                self._det_conf, self._det_box = best
                self._det_t = t
                self._det_header = msg.header

    # -- the ladder --------------------------------------------------------- #
    def _loop(self):
        """The lock ladder. GUARDED (B44 class).

        A bare Thread target running XFeat/LK on live frames. One malformed
        frame or ONNX hiccup killed it, after which the ladder silently never
        locked again while every node looked healthy. Faults are counted and
        reported; the ladder degrades to "not locked", which consumers already
        handle, instead of vanishing.
        """
        faults = 0
        while rclpy.ok():
            try:
              if not self._fresh.wait(0.5):
                  continue
              self._fresh.clear()
              with self._lock:
                  gray = self._gray
                  header = self._header
                  det_header = self._det_header
                  det_box, det_conf, det_t = (self._det_box, self._det_conf,
                                              self._det_t)
              if gray is None:
                  continue
              now = time.monotonic()

              anchor_ran = False
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
                      anchor_ran = True
                      # The frame this pose was fitted to. The anchor runs at
                      # 3 Hz while this loop runs at frame rate, so between
                      # evaluations the pose below is REUSED -- up to 333 ms old
                      # -- and publishing it under the current frame's stamp
                      # would report a third of a second of staleness as ~18 ms.
                      # That defeats the freshness machinery precisely on the
                      # rung it exists to protect, and only on that rung, so it
                      # looks correct whenever a detection is present.
                      self._anchor_header = header
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
              self._publish(st, header_for(st.rung, detection=det_header,
                                           frame=header,
                                           anchor=self._anchor_header))
              # The pose rides the ANCHOR's header: it is derived from that
              # frame's correspondences, not from whichever frame just arrived.
              #
              # Published only when the anchor actually RE-EVALUATED, or when its
              # ok/not-ok state changed. The loop runs at frame rate and the
              # anchor at `anchor_hz` (3), so publishing every tick sent ~32
              # duplicates for every real evaluation -- measured at 96.5 Hz on
              # the vehicle for a 3 Hz quantity. Each message now corresponds to
              # one evaluation, which is also what makes its stamp meaningful.
              if self._anchor is not None:
                  ok_now = bool(self._anchor_pose is not None
                                and self._anchor_pose.ok)
                  # ...OR on a slow heartbeat. Without one this went SILENT:
                  # with no reference snapped the anchor never re-evaluates, so
                  # `anchor_ran` stays False and the state never changes, and
                  # after the first message nothing was published again. A
                  # consumer then cannot tell "no target" from "lock_node is not
                  # running", and the health board cannot age what it never sees.
                  # Absence of a TARGET is carried by `ok=False`, not by absence
                  # of the message.
                  due = (now - self._pose_pub_t) >= self._anchor_period
                  if anchor_ran or due or ok_now != self._pose_was_ok:
                      self._pose_was_ok = ok_now
                      self._pose_pub_t = now
                      self._publish_pose(self._anchor_pose,
                                         self._anchor_header or header)
            except Exception as exc:        # noqa: BLE001 -- B44 class
                faults += 1
                if faults in (1, 50):
                    self.get_logger().error(
                        f'[LOCK ] ladder fault #{faults} (thread kept alive): '
                        f'{type(exc).__name__}: {exc}. The lock reports NOT LOCKED '
                        f'while this persists, which consumers already handle.')
                time.sleep(0.05)

    def _publish(self, st, header):
        """`header` is the frame the WINNING RUNG observed, not the newest one.

        Each rung answers from a different instant -- the follower from this
        frame, the anchor from whichever frame it last ran on, the detector
        from whichever frame it last finished. One shared stamp would make all
        three claim the freshest of them.
        """
        # Rate limit when one is configured, EXCEPT on a rung change. Default
        # is no limit: the board-side loop is 500 Hz and wants every frame.
        rung = st.rung.value if st.have_target else None
        if self._pub_min_dt > 0.0 and rung == self._last_pub_rung:
            now = time.monotonic()
            if now - self._last_pub_t < self._pub_min_dt:
                return
            self._last_pub_t = now
        else:
            self._last_pub_t = time.monotonic()
        self._last_pub_rung = rung

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
