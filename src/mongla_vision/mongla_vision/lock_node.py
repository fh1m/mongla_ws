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
import collections
import math
import os

os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import threading
import time

import cv2
import numpy as np

from mongla_vision.optics import RefractiveRectifier
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

try:                                   # noqa: SIM105
    from mongla_interfaces.msg import TargetCorrespondences
except ImportError:                    # message not built in this workspace
    # The LADDER must survive it. Correspondences are an addition to what this
    # node does, not a precondition, so an unbuilt interface costs the 6-DoF
    # path and nothing else -- the follower and anchor rungs still publish
    # `/lock`, which is what the control loop steers on.
    TargetCorrespondences = None

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Range
from mongla_vision.anchor import loop_closure as _lc
from mongla_vision.anchor import health as _health

from mongla_vision import qos as _qos
from mongla_vision.stamps import capture_monotonic
from mongla_vision.detection.detector import Detection
from mongla_vision.detection.messages import detections_to_array
from mongla_vision.tracking.follower import Follower
from mongla_vision.tracking.lock_state import (
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
    def __init__(self, node_name: str = 'mongla_lock', *,
                 parameter_overrides=None):
        super().__init__(node_name,
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter('camera', 'forward')
        self.declare_parameter('target_class', '')
        self.declare_parameter('follow', True)
        self.declare_parameter('anchor', False)
        self.declare_parameter('anchor_model', '')
        # A bank built BEFORE the run, from practice footage or stills:
        # `tools/build_practice_bank.py`. Measured 2026-09-24 -- references
        # from one run clear the trust bar on 92-100 % of frames of a DIFFERENT
        # run of the same prop, so the anchor need not wait for a confident
        # live detection to have something to match against. Which matters
        # because the moment it is needed is the moment the detector is failing.
        #
        # ⛔ A preloaded reference is trusted exactly like a live one: it clears
        # MIN_INLIERS or it does not answer, and `recognise()` still wants the
        # detector to agree before any identity is asserted.
        self.declare_parameter('anchor_bank', '')
        # ⛔ LOOP CLOSURE IS OFF BY DEFAULT, like every other capability here.
        # It is the only path in this node that can reach the LOCALISATION
        # FILTER, and a wrong position fix is not merely wrong -- the filter
        # shrinks its covariance around it and becomes confident. Off until a
        # pool day says otherwise.
        self.declare_parameter('loop_closure',
                               _lc.defaults()['enabled'])
        self.declare_parameter('place_period_s',
                               _lc.defaults()['place_period_s'])
        self.declare_parameter('place_travel_m',
                               _lc.defaults()['place_travel_m'])
        # Height above the floor = pool_depth_m - |depth|. Without it pixels
        # cannot become metres and every closure is refused, by design.
        self.declare_parameter('pool_depth_m',
                               _lc.defaults()['pool_depth_m'])
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
        self._cam = cam
        ns = f'/mongla/vision/{cam}'
        # AN EXPLICIT target_class WINS and freezes the aim. Anything else and
        # the ladder aims ITSELF at whatever the mission told the detector to
        # look for -- see `_on_classes_filter`.
        self._cls_pinned = str(self.get_parameter('target_class').value).strip()
        self._cls = self._cls_pinned
        self._full = float(self.get_parameter('full_authority_s').value)
        self._zero = float(self.get_parameter('zero_authority_s').value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._gray = None
        self._header = None
        self._recent = collections.deque(maxlen=4)
        self._snap_skew_warned = False
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
        # Ungated on purpose: what the table IGNORED must be reported whether
        # or not this node needs a lookup. Inside the `if` below it would stay
        # silent exactly when `target_width_m` was passed explicitly -- which is
        # the case where a broken override matters most, because the operator
        # believes the file is doing something.
        self._report_geometry_problems()
        if self._target_w_m <= 0.0 and self._cls:
            from mongla_vision.target_geometry import width_for, describe
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
                    f'Fix on the deck without a rebuild: put the width in '
                    f'~/.mongla/target_geometry.yaml (or set '
                    f'MONGLA_TARGET_GEOMETRY), or pass target_width_m.')
        # MEDIUM. A flat port is not a pinhole: the ray from a point at water
        # angle tw leaves at air angle ta with sin(ta) = n*sin(tw), so the
        # effective focal length grows with field angle -- 10.6 % centre to
        # corner on this camera, and the pose path was reading the AIR K with
        # raw pixels. Ray-traced against the shipped model, the error at the
        # frame corner is +34 px at 0.3 m and +46 px at 2 m. Default 'water'
        # matches `flow_node` and `pnp_node` -- one `medium:=` launch
        # argument feeds all three -- and the choice is LOGGED at
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
        # Evidence, not a pose: `pnp_node` subscribes to this and publishes
        # `{ns}/target_pose`. Solving here as well would put two publishers on
        # one claim, which is the defect this split exists to remove.
        self._pub_corr = (
            self.create_publisher(TargetCorrespondences,
                                  f'{ns}/correspondences', _qos.DETECTIONS)
            if TargetCorrespondences is not None else None)
        if TargetCorrespondences is None:
            self.get_logger().warn(
                '[LOCK ] mongla_interfaces/TargetCorrespondences not built -- '
                'no 6-DoF pose will be published (the ladder is unaffected). '
                'Rebuild mongla_interfaces to enable it.')
        self.create_subscription(CameraInfo, f'{ns}/camera_info',
                                 self._on_info, 10)
        self._det_box = None
        self._det_conf = 0.0
        self._det_cls = ''
        self._det_t = 0.0
        self._fresh = threading.Event()

        self._follower = Follower() if bool(
            self.get_parameter('follow').value) else None
        self._anchor = None
        # Inliers the last bank evaluation produced, and the reference that
        # produced them. Starts at 0, which cannot read as stale because
        # staleness is gated on `has_reference` -- an empty bank is not a
        # decayed one, and the two need different answers.
        self._anchor_inliers = 0
        self._anchor_best = None
        self._anchor_enrolled = 0
        # What the bank is holding. One bank can carry several kinds of thing,
        # so a checkpoint is tagged with the class it was enrolled from and the
        # search is restricted to it -- otherwise "where is the gate" could be
        # answered by a torpedo board that happened to match better.
        self._anchor_label = ''
        # Earliest monotonic time another enrolment may be ATTEMPTED. See the
        # throttle note at the enrol site.
        self._anchor_enrol_next = 0.0
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

        # ---- loop closure ------------------------------------------------
        # ⛔ CONFIGURED BY THE NODE, NOT BY LAUNCH. A switch threaded through
        # launch has to be declared in every launch file that starts this
        # node, and this package has already shipped capabilities reachable
        # from one launch path and not the one `bringup` includes. Loop
        # closure nearly became the next: it went into vision_pi.launch.py
        # while bringup includes vision.launch.py.
        #
        # Precedence: an explicitly-set ROS parameter wins (so `ros2 param
        # set` and any launch that does pass one still work), otherwise
        # ~/.mongla/loop_closure.yaml, otherwise the defaults. The deck file
        # is the same pattern the course priors already use.
        cfg = _lc.load_config(log=self.get_logger())

        def _cfg(name, key):
            p = self.get_parameter(name)
            if p.type_ != rclpy.parameter.Parameter.Type.NOT_SET:
                v = p.value
                # A parameter left at its declared default is not a decision;
                # the file is. Only a value that DIFFERS from the default
                # overrides the file, so an operator's edit is not silently
                # outvoted by a default nobody set.
                if v != _lc.defaults()[key]:
                    return v
            return cfg[key]

        self._closer_on = bool(_cfg('loop_closure', 'enabled'))
        self._places = _lc.PlaceLog(
            float(_cfg('place_period_s', 'place_period_s')),
            float(_cfg('place_travel_m', 'place_travel_m')))
        self._pool_depth = float(_cfg('pool_depth_m', 'pool_depth_m'))
        self._closure_cfg = cfg
        self._odom_xy = None          # (x, y) m, pool frame
        self._odom_sigma = float('nan')
        self._odom_z = float('nan')   # filter depth, negative below surface
        self._odom_t = 0.0
        self._floor_h = float('nan')
        self._floor_h_t = 0.0
        self._pool_depth_warned = False
        self._last_health = _health.UNKNOWN
        self._fix_pub = None
        self._closures = 0
        self._closure_refusals = collections.Counter()
        if self._closer_on:
            # ⛔ DOWNWARD ONLY. A forward bank holds crops of TARGETS -- today's
            # fixture is literally a person walking about -- and matching a
            # moving object is not a vehicle position. The floor does not move.
            if self._camera != 'downward':
                self.get_logger().warn(
                    '[LOCK ] loop_closure requested on the '
                    f'{self._camera} camera and REFUSED. A forward bank holds '
                    'targets, and a target that moves is not a place. Run it '
                    'on the downward camera.')
                self._closer_on = False
            else:
                self.create_subscription(Odometry, '/mongla/odom',
                                         self._on_odom, _qos.sensor())
                # ⭐ THE FLOOR'S OWN HEIGHT, from the tile grating in
                # `flow_node`. That is a MEASUREMENT of the floor; flow_node
                # declines to publish the pool_depth arithmetic under this
                # name precisely because it is not one.
                self.create_subscription(Range, f'{ns}/floor_height',
                                         self._on_floor_height, 10)
                self._fix_pub = self.create_publisher(
                    PointStamped, '/mongla/localization/fix', 10)
                self.get_logger().info(
                    '[LOCK ] loop closure ON: places remembered every '
                    f'{self._places.period_s:.0f} s or '
                    f'{self._places.travel_m:.1f} m, offered as a fix at '
                    f'>= {_lc.CLOSURE_INLIERS} inliers.')

        self._pub = self.create_publisher(Detection2DArray, f'{ns}/lock',
                                          _qos.DETECTIONS)
        # Shared profiles, never a hand-rolled QoS: a RELIABLE/BEST_EFFORT
        # mismatch is answered by rclpy with one WARN and then silence --
        # clean launch, healthy nodes, zero frames. `mongla_vision.qos` owns
        # these topics and a contract test enforces it.
        self.create_subscription(Detection2DArray, f'{ns}/detections',
                                 self._on_det, _qos.DETECTIONS)
        self.create_subscription(Image, f'{ns}/image_raw',
                                 self._on_img, _qos.IMAGE)
        # SELF-AIMING. `lock_class` is a LAUNCH argument, so a mission that
        # switches target mid-run (gate -> rescue -> red_pipe, which the DSL
        # does on every vision verb via `set_classes`) left the ladder still
        # following the FIRST class -- silently, and only on the rung that is
        # supposed to save the lock. At the pool that reads as "the ladder is
        # broken" when it is merely aimed somewhere else.
        #
        # `classes_filter` is LATCHED, so this also works when the ladder
        # starts after the detector has already been told what to look for.
        self.create_subscription(String, f'{ns}/classes_filter',
                                 self._on_classes_filter, _qos.LATCHED)
        threading.Thread(target=self._loop, daemon=True).start()
        # LIVE, like the detector and the tracker. The ladder was aimable only
        # at launch, so a mission could not point it at the object it was about
        # to steer on -- it had to be relaunched. `target_class` and
        # `publish_hz` now take effect on the next tick.
        self.add_on_set_parameters_callback(self._on_param_change)
        self.create_timer(5.0, self._log_health)
        self._n_by_rung = {r: 0 for r in Rung}

        self.get_logger().info(
            f'[LOCK ] {cam}: follow={self._follower is not None} '
            f'anchor={self._anchor is not None} '
            f'authority {self._full:.2f}->{self._zero:.2f}s  -> {ns}/lock')


    def _report_geometry_problems(self):
        """Say what the target table IGNORED, before saying what it used.

        A dropped override and an override that was never read look identical
        from the deck, and a table that failed to parse reads as 'no committed
        width' for every class at once -- which is exactly how a stray indent
        in the committed YAML presented while this was being written.

        Out of line on purpose: `test_target_geometry` asserts the width lookup
        and its assignment stay within one window of source, and inlining this
        pushed them apart.
        """
        from mongla_vision.target_geometry import load_errors, rejected_overrides
        for path, err in load_errors():
            self.get_logger().error(
                f'[LOCK ] target geometry {path} FAILED TO PARSE ({err}) -- '
                f'widths from it are MISSING, not zero')
        for name, w_bad, origin in rejected_overrides():
            self.get_logger().warn(
                f'[LOCK ] IGNORED override {name}={w_bad} m from {origin}: '
                f'outside the plausible band, likely cm typed as m')

    def _build_anchor(self):
        """Optional by design: a missing ONNX must cost the anchor rung, not
        the node -- the follower is still worth running without it."""
        try:
            from mongla_vision.anchor.bank import CheckpointBank
            from mongla_vision.anchor.xfeat_onnx import XFeatONNX
            import glob
            p = str(self.get_parameter('anchor_model').value).strip()
            if not p:
                # MONGLA_HEF_DIR FIRST, then the historical default. The
                # detector already resolves its models that way
                # (`detection/yolo.py`), and hardcoding one of the two paths
                # here made the model directory a truth with two copies: point
                # MONGLA_HEF_DIR somewhere else and the detector follows while
                # the anchor silently does not, losing the rung with one WARN
                # and no error.
                roots = [d for d in (os.environ.get('MONGLA_HEF_DIR', '').strip(),
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
            # `period_s` is what lets the bank size its own shortlist: it
            # measures one match on THIS machine and spends a fixed fraction of
            # the evaluation period. The dev box and the Pi differ by 2.2x on
            # the identical work (14.4 vs 31.4 ms), so a constant width would
            # be wrong on one of them.
            self._anchor = CheckpointBank(XFeatONNX(p, top_k=1024),
                                          period_s=self._anchor_period)
            self.get_logger().info(
                f'[LOCK ] anchor backend {os.path.basename(p)}, '
                f'bank capacity {self._anchor._cap}, '
                f'shortlist <= {self._anchor.shortlist_k()}')
            self._load_bank()
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
            # A SHORT HISTORY, so a detection can be paired with the frame it
            # was actually computed on. See `_frame_for`. Four frames at the
            # anchor backend's resolution is ~300 kB; the detector's own
            # capture->arrival latency is 32 ms median / 48 p95, which is one
            # to two frames, so four is the p95 with headroom and no more.
            self._recent.append((_stamp_key(msg.header), g))
        self._fresh.set()

    def _load_bank(self):
        """Preload a practice bank, if one was named.

        A missing or unreadable bank costs the PRELOAD, never the rung: the
        anchor still learns live checkpoints from detections. But it WARNs
        rather than logging quietly, because an operator who passed
        `anchor_bank:=` and silently got nothing would believe the vehicle had
        a memory it does not have, and would find out during the run.
        """
        path = str(self.get_parameter('anchor_bank').value or '').strip()
        if not path:
            return
        path = os.path.expanduser(path)
        try:
            n = self._anchor.load(path)
            self.get_logger().info(
                f'[LOCK ] preloaded {n} checkpoints from '
                f'{os.path.basename(path)}: {sorted(set(self._anchor.labels))}')
            if self._anchor.labels:
                self._anchor_label = self._anchor.labels[-1]
        except Exception as exc:
            self.get_logger().warn(
                f'[LOCK ] anchor_bank {path} NOT loaded: '
                f'{type(exc).__name__}: {exc} -- the rung still runs and will '
                f'learn checkpoints from live detections')

    def _frame_for(self, det_header):
        """The frame a detection was computed on, not merely the newest one.

        ⛔ WHAT THIS FIXES. `snap` defines the anchor's reference patch, and
        that patch IS the object model: its width is taken to be the target's
        true width, so every later range is scaled by it. It was snapped from
        the CURRENT frame using the LAST detection's box, with nothing
        comparing the two stamps -- and the detector's box is 32 ms median /
        48 p95 behind the frame in hand. On a moving hull that box no longer
        bounds the target in the frame it is applied to, so the reference patch
        clips the target or swallows background, and the error is baked into
        the model for the whole lock.

        Matching the stamp removes the error rather than bounding it, which is
        affordable HERE and only here: snapping happens once, so waiting for
        the right frame costs nothing, while the steering rungs must act on
        whatever they have and use freshness decay instead.
        """
        key = _stamp_key(det_header)
        if key is None:
            return None
        for k, g in self._recent:
            if k == key:
                return g
        return None

    def _on_odom(self, msg) -> None:
        """The filter's own estimate: where it thinks it is, and how sure.

        ⛔ SIGMA IS READ, NOT ASSUMED. It gates the bank search radius and it
        is half of every closure's own sigma, so a zero covariance here (the
        ROS default for "not filled in") would claim perfect knowledge. Absent
        is NaN, which every gate downstream refuses.
        """
        pp = msg.pose.pose.position
        self._odom_xy = (float(pp.x), float(pp.y))
        self._odom_z = float(pp.z)
        cov = msg.pose.covariance
        vx, vy = float(cov[0]), float(cov[7])
        self._odom_sigma = (math.sqrt(vx + vy)
                            if (vx > 0.0 and vy > 0.0) else float('nan'))
        self._odom_t = time.monotonic()

    def _note_anchor_health(self) -> None:
        """Say WHICH failure this is, when the bank stops matching.

        ⚠ Reported, not acted on. The thresholds in `anchor/health.py` are
        DECLARED -- no port has been fouled on purpose -- so this logs a
        verdict for an operator and changes no control path. Wiring it to
        behaviour needs a measured bar first.
        """
        try:
            cur, best = self._anchor.last_lookup_scores()
        except Exception:                      # noqa: BLE001 -- never fatal
            return
        v = _health.assess(cur, best)
        if v.state == self._last_health:
            return
        self._last_health = v.state
        if v.state == _health.CAMERA:
            self.get_logger().warning(
                f'[LOCK ] the CAMERA looks degraded, not the scene: '
                f'{v.reason}. Searching for a better view cannot help.')
        elif v.state == _health.WORLD:
            self.get_logger().info(
                f'[LOCK ] the VIEW changed, the camera is fine: {v.reason}')

    def _on_floor_height(self, msg) -> None:
        """Measured height above the floor, from the tile grating."""
        r = float(msg.range)
        if r > 0.05 and math.isfinite(r):
            self._floor_h = r
            self._floor_h_t = time.monotonic()

    # A floor height older than this is not this vehicle's altitude any more.
    _FLOOR_H_MAX_AGE_S = 2.0

    def _altitude_m(self) -> float:
        """Height above the floor. NaN when it cannot be known.

        ⭐ THE MEASURED FLOOR FIRST. `flow_node` publishes `floor_height` from
        the tile grating, and refuses to publish the pool_depth arithmetic
        under that name because `pool_depth_m - |depth|` is a typed constant
        minus a depth, not a measurement of the floor. This agrees: the
        grating wins, and it must be FRESH -- a stale height is a different
        altitude wearing the right units.

        ⚠ The pool_depth fallback stays, because an operator who has measured
        their pool may legitimately supply it. It says what it is, once.

        ⛔ AND THERE IS NO THIRD OPTION. Pixels become metres only with this
        number; a default would put a plausible value where a measurement is
        missing, the recurring defect in this codebase. The bench barometer
        read "not initialised" (section 37), so with neither source this
        returns NaN and every closure is refused.
        """
        if (self._floor_h == self._floor_h
                and (time.monotonic() - self._floor_h_t)
                <= self._FLOOR_H_MAX_AGE_S):
            return self._floor_h
        if not (self._pool_depth > 0.0):
            return float('nan')
        if self._odom_z != self._odom_z:
            return float('nan')
        alt = self._pool_depth - abs(self._odom_z)
        if alt <= 0.05:
            return float('nan')
        if not self._pool_depth_warned:
            self._pool_depth_warned = True
            self.get_logger().warn(
                f'[LOCK ] no floor_height; using pool_depth_m '
                f'({self._pool_depth:.2f} m) minus depth. That is a TYPED '
                f'CONSTANT minus a measurement, not a measured altitude, and '
                f'every closure offset is scaled by it.')
        return alt

    def _place_tick(self, gray, header, now: float) -> None:
        """Remember where we are, and recognise where we have been.

        Recognition FIRST: enrolling before asking would add the current frame
        to the bank and then find it, which is the self-closure this whole
        path is built to avoid.
        """
        if not self._closer_on or gray is None or self._anchor is None:
            return
        xy = self._odom_xy

        # ---- recognise ---------------------------------------------------
        if self._anchor.size:
            # ⭐ THE FILTER'S OWN UNCERTAINTY IS THE SEARCH RADIUS. A confident
            # filter refuses to consider a place 20 m away; a drifting one
            # widens until it gates on nothing, which is exactly right -- the
            # prior should stop helping precisely when it stops being one.
            radius = (3.0 * self._odom_sigma
                      if self._odom_sigma == self._odom_sigma else 0.0)
            bp = self._anchor.locate(gray, label=_lc.PLACE_LABEL,
                                     near=xy if radius > 0.0 else None,
                                     radius_m=radius)
            c = _lc.consider(
                bp,
                ref_age_s=(now - (self._places._last_t or now)),
                travel_m=self._places.travel_since(xy),
                ref_sigma_m=(self._odom_sigma
                             if self._odom_sigma == self._odom_sigma else 1.0),
                m_per_px=self._m_per_px(),
                bank_size=self._anchor.size,
                min_inliers=int(self._closure_cfg['min_inliers']),
                min_age_s=float(self._closure_cfg['min_age_s']),
                min_travel_m=float(self._closure_cfg['min_travel_m']),
                max_offset_m=float(self._closure_cfg['max_offset_m']))
            if c.ok:
                self._closures += 1
                m = PointStamped()
                # THE MATCHED FRAME'S header, on the board clock -- not now().
                # A fix stamped on arrival is replayed at the wrong instant by
                # the filter's retrodiction, which is the one thing that makes
                # a late measurement safe.
                m.header = header
                m.point.x, m.point.y = float(c.xy[0]), float(c.xy[1])
                m.point.z = float(c.sigma)      # the producer's own sigma
                self._fix_pub.publish(m)
                self.get_logger().info(
                    f'[LOCK ] loop closure {self._closures}: ref {c.index}, '
                    f'{c.reason}, sigma {c.sigma:.2f} m')
            else:
                # Counted, not dropped. A closure path that silently never
                # fires is indistinguishable from one that was never wired up.
                key = c.reason.split(' -- ')[0].split('(')[0].strip()
                self._closure_refusals[key] += 1
                n = sum(self._closure_refusals.values())
                if n % 300 == 0:
                    top = ', '.join(f'{k} x{v}' for k, v in
                                    self._closure_refusals.most_common(3))
                    self.get_logger().info(
                        f'[LOCK ] no closure in {n} looks: {top}')

        # ---- remember ----------------------------------------------------
        if self._places.should_enrol(now, xy):
            # ⛔ WHOLE FRAME, no ROI. The ~100-inlier bar was derived on
            # whole-frame downward references (section 24); cropping would
            # repeat section 25, where a bar measured on one configuration was
            # shipped against another and silently never fired.
            r = self._anchor.enrol(gray, roi=None, det_conf=1.0,
                                   label=_lc.PLACE_LABEL, position=xy)
            if r.accepted:
                self._places.note(now, xy)
                self.get_logger().info(
                    f'[LOCK ] place {self._anchor.size}: {r.keypoints} kp '
                    f'at ({xy[0]:.2f}, {xy[1]:.2f})')

    def _m_per_px(self) -> float:
        """Metres per BACKEND pixel on the floor. NaN without an altitude.

        ⭐ THE RECTIFIED K, NOT THE AIR ONE. `_on_info` already scales the
        intrinsics to the backend's own grid -- which is the grid the
        homography is defined on -- and `rectifier_for` returns the K that
        matches the flat-port correction. Using the air focal length underwater
        leaves a clean 1/n scale error: every offset 33 % short, with a
        plausible number and no warning. That is the exact defect
        `optics.py` exists to prevent, and a loop closure is precisely where a
        silent 33 % would be believed.
        """
        alt = self._altitude_m()
        if alt != alt:
            return float('nan')
        K = self._K_rect if getattr(self, '_K_rect', None) is not None \
            else self._K
        if K is None:
            return float('nan')
        f = float(K[0, 0])
        return alt / f if f > 0.0 else float('nan')

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
        # One place builds the (rectifier, matching K) pair -- see
        # `optics.rectifier_for`. `pnp_node` reads the same helper, so the two
        # cannot disagree about what optics a point set is in.
        from mongla_vision.optics import rectifier_for
        self._rect, self._K_rect, note = rectifier_for(self._K, self._medium)
        self.get_logger().info(f'[LOCK ] {note}')

    def _publish_correspondences(self, pose, header):
        """The anchor's inliers as EVIDENCE, for `pnp_node` to solve.

        This node used to solve the pose here. It no longer does, and the split
        is the point: a pose is a lossy summary of the points it was fitted to,
        so publishing only the pose made a missed shot unexplainable -- bad
        evidence and a bad solve look identical afterwards. Correspondences on
        a bag are re-solvable with a different gate, months later.

        ⛔ THE OBJECT SIDE IS RECTIFIED HERE; THE IMAGE SIDE IS NOT. Object
        points are reference PIXELS scaled to metres, so they must go through
        the flat-port map or the object model itself is distorted -- that is
        this node's business, because this node owns the patch. The live pixels
        are sent RAW and marked `OPTICS_RAW`, because rectifying the image side
        is the SOLVER's business and doing it in both places is how the two
        would come to disagree. The message states which, so neither has to
        guess and nothing can apply the map twice.
        """
        if self._pub_corr is None or pose is None or not pose.ok:
            return
        if self._K is None or self._target_w_m <= 0.0:
            return          # `_report_geometry_problems` already said why
        roi = self._anchor.reference_roi
        wh = ((roi[2] - roi[0], roi[3] - roi[1]) if roi
              else (self._anchor._be.w, self._anchor._be.h))
        w_px, h_px = float(wh[0]), float(wh[1])
        if w_px <= 0 or h_px <= 0:
            return
        ref_pts = pose.ref_pts
        if self._rect is not None:
            ref_pts = self._rect.rectify(ref_pts)
        ref = np.asarray(ref_pts, np.float64).reshape(-1, 2)
        live = np.asarray(pose.live_pts, np.float64).reshape(-1, 2)
        if len(ref) < 4 or len(ref) != len(live):
            return

        # Reference pixels -> object-plane metres, origin at the patch centre.
        # The patch is planar BY CONSTRUCTION (one snapped view), so z = 0.
        m_per_px = float(self._target_w_m) / w_px
        obj = np.zeros((len(ref), 3), np.float64)
        obj[:, 0] = (ref[:, 0] - w_px * 0.5) * m_per_px
        obj[:, 1] = (ref[:, 1] - h_px * 0.5) * m_per_px

        m = TargetCorrespondences()
        if header is not None:
            m.header = header
        m.camera = self._cam
        m.target_label = self._cls or ''
        m.source = 'anchor'
        m.optics = TargetCorrespondences.OPTICS_RAW
        # The anchor matches in its BACKEND's pixels, not the camera's full
        # frame. Saying so is what lets the solver scale K instead of silently
        # scaling every angle and range it recovers.
        m.image_width = int(self._anchor._be.w)
        m.image_height = int(self._anchor._be.h)
        m.object_points = obj.reshape(-1).tolist()
        m.image_points = live.reshape(-1).tolist()
        self._pub_corr.publish(m)

    def _on_param_change(self, params):
        """Aim (or un-aim) the ladder while a mission is running.

        `target_class` non-empty PINS the aim, exactly as the launch argument
        does. Setting it back to '' releases the pin and the ladder resumes
        following `classes_filter` -- so a mission can take manual control of
        one leg and hand it back, without a relaunch.
        """
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == 'target_class':
                new = str(prm.value or '').strip()
                if new == self._cls_pinned:
                    continue
                self._cls_pinned = new
                if new:
                    self._cls = new
                    self._retarget_width()
                    self.get_logger().info(
                        f'[LOCK ] target_class pinned to {new!r}')
                else:
                    self.get_logger().info(
                        '[LOCK ] target_class released -- following '
                        'classes_filter again')
            elif prm.name == 'publish_hz':
                hz = float(prm.value or 0.0)
                if hz < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_hz must be >= 0 (0 = no limit)')
                self._pub_min_dt = (1.0 / hz) if hz > 0.0 else 0.0
                self.get_logger().info(
                    f'[LOCK ] publish_hz {"unlimited" if hz <= 0 else hz}')
        return SetParametersResult(successful=True)

    def _on_classes_filter(self, msg):
        """Follow the mission's current target class.

        Only when the filter names EXACTLY ONE class. An empty or multi-class
        filter is not an aim -- steering the ladder at one of several would be
        a guess, and the ladder's whole value is that it does not guess.
        """
        if self._cls_pinned:
            return                        # operator pinned it; do not fight
        names = [n.strip() for n in str(msg.data).split(',') if n.strip()]
        if len(names) != 1 or names[0] == self._cls:
            return
        self._cls = names[0]
        self._retarget_width()
        self.get_logger().info(
            f'[LOCK ] following {self._cls!r} (from classes_filter)')

    def _retarget_width(self):
        """Re-resolve the committed width for the class we now follow.

        Without this the 6-DoF pose keeps the FIRST class's width and reports a
        confident range for the wrong object -- worse than refusing. An
        explicit `target_width_m` still wins, exactly as at startup.
        """
        if float(self.get_parameter('target_width_m').value or 0.0) > 0.0:
            return
        try:
            from mongla_vision.target_geometry import width_for
        except ImportError:
            return
        w = width_for(self._cls) if self._cls else 0.0
        self._target_w_m = float(w or 0.0)
        if self._target_w_m <= 0.0:
            self.get_logger().warn(
                f'[LOCK ] no committed width for {self._cls!r} -- the 6-DoF '
                f'pose will refuse while this class is the target.')

    def _on_det(self, msg):
        best = None
        best_name = ''
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
                best_name = name
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
                self._det_cls = best_name
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
                  det_cls = self._det_cls
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
                  # ⛔ THE DEFECT THIS REPLACED. The guard here used to be
                  # `not self._anchor.has_reference`, so the reference was
                  # snapped ONCE and never refreshed. Measured 2026-09-24 on
                  # the archive clips, inliers against that frozen reference:
                  #
                  #   clip              +1 s  +3 s  +5 s  +8 s
                  #   mirpur_torpedo     189    65   134    24
                  #   mirpur_torpedo_1    69    43    51    12  <- under the bar
                  #   mirpur_gate        195    36   237    31
                  #
                  # Every clip's minimum is its +8 s column. A reference decays,
                  # and the same frames against a FRESH reference are the +1 s
                  # column -- so this was never a limit of the descriptor.
                  #
                  # The bank takes a new checkpoint when the best stored one has
                  # decayed toward the trust bar, judged on the inlier count the
                  # 3 Hz `locate()` below already produced. Asking
                  # `wants_refresh()` here instead would run a SECOND full match
                  # per frame to learn what we just measured.
                  stale = (self._anchor.has_reference
                           and self._anchor_inliers < self._anchor._refresh)
                  # ⭐ AND the criterion that matters more, per §21.2: the bank
                  # buys VIEWPOINT DIVERSITY, not freshness. A target whose
                  # apparent size is outside the band the bank covers earns a
                  # slot even while the existing references are matching fine
                  # -- because they will stop matching when the vehicle closes
                  # in, and that is the frame a torpedo actually fires from.
                  # Seen on footage: a bank enrolled at mid-range refused every
                  # close-range frame at 7 inliers against a board filling the
                  # screen.
                  uncovered = (det_box is not None
                               and not self._anchor.covers_scale(gray, det_box))
                  # ⛔ AND THE REASON THE BANK WAS STAYING EMPTY.
                  # `stale` only fires when the match is BAD -- but when it is
                  # bad the target may already be lost, and when it is good
                  # nothing is ever added, so the bank never grows. Measured on
                  # Mirpur torpedo footage: the stale-only policy enrolled ONE
                  # reference and asserted identity on 0 % of later frames,
                  # while a bank of 14 asserted on 3 of 8.
                  #
                  # Hoarding is free here and we built the machinery that makes
                  # it so: §20 measured search cost FLAT in bank size (100
                  # references cost what 10 do, via the signature shortlist) and
                  # 64 references is 16 MB. Eviction by inlier yield keeps the
                  # quality bar. So while there is room, every confident
                  # detection earns a slot -- diversity then emerges from the
                  # run instead of being predicted by a heuristic.
                  room = self._anchor.size < self._anchor._cap
                  # ⛔ THROTTLED, AND NOT FOR TIDINESS. `enrol()` runs a full
                  # `locate()` to decide whether a candidate agrees with the
                  # bank, and a candidate that keeps being REFUSED leaves
                  # `stale` true -- so an unthrottled path retries that match on
                  # every detection frame. Measured cost of one match on the Pi
                  # is 31.4 ms and the shortlist is several, which lands
                  # ~124 ms of matching inside the loop that publishes /lock,
                  # against a measured 53.6 Hz floor. The refusal case is
                  # exactly when the vehicle can least afford it.
                  #
                  # One attempt per anchor period: enrolling faster than the
                  # bank is evaluated cannot help, because nothing has changed
                  # its mind in between.
                  may_try = now >= self._anchor_enrol_next
                  if det_box is not None and may_try and (
                          not self._anchor.has_reference or stale or uncovered
                          or room):
                      self._anchor_enrol_next = now + self._anchor_period
                      # THE FRAME THE BOX BELONGS TO, not merely the newest.
                      # This patch becomes the object model, so pairing it with
                      # the wrong frame scales every range that follows.
                      snap_gray = self._frame_for(det_header)
                      if snap_gray is None:
                          # The matching frame has aged out of the ring, or a
                          # stamp is missing. Snapping anyway would bake in the
                          # skew silently; waiting costs one detection, because
                          # the next one arrives with its frame still in hand.
                          if not self._snap_skew_warned:
                              self._snap_skew_warned = True
                              self.get_logger().warn(
                                  '[LOCK ] anchor snap deferred: no frame '
                                  'matching the detection stamp. Check '
                                  'use_sim_time and the detector latency -- '
                                  'the reference patch defines the object '
                                  'model, so it is not snapped against a '
                                  'frame it does not belong to.')
                      else:
                          r = self._anchor.enrol(snap_gray, roi=det_box,
                                                 det_conf=det_conf,
                                                 label=det_cls)
                          if r.accepted:
                              # A fresh checkpoint resets the staleness measure:
                              # the number that triggered this refresh described
                              # the reference we have just replaced.
                              self._anchor_inliers = self._anchor._refresh
                              self._anchor_enrolled += 1
                              self._anchor_label = det_cls
                              # Logged because a bank that silently stops
                              # enrolling looks exactly like one that never
                              # needed to, and those are opposite faults.
                              self.get_logger().info(
                                  f'[LOCK ] checkpoint {self._anchor_enrolled}: '
                                  f'{r.keypoints} kp, bank {self._anchor.size}/'
                                  f'{self._anchor._cap}, '
                                  f'{"scale" if uncovered else ("room" if room else "quality")}')
                          elif r.reason != 'confidence':
                              self.get_logger().debug(
                                  f'[LOCK ] checkpoint refused: {r.reason}')
                  elif (self._anchor.has_reference
                        and now >= self._anchor_next):
                      self._anchor_next = now + self._anchor_period
                      bp = self._anchor.locate(
                          gray, label=self._anchor_label or None)
                      # The bank answers best-of-bank; `pose` is the ordinary
                      # AnchorPose the rest of this node already consumes, so
                      # nothing downstream learns that there is now more than
                      # one reference.
                      # ⭐ THE PER-REFERENCE YIELDS, WHICH USED TO BE THROWN
                      # AWAY. A fouled port collapses the match against EVERY
                      # reference at once; a target that left collapses only
                      # the one that used to win. Both arrive at the ladder as
                      # "the anchor is not matching" and need OPPOSITE
                      # responses -- stop trusting vision, or go and search.
                      # `bp` alone cannot carry that: it is a statement about
                      # the whole shortlist.
                      self._note_anchor_health()
                      self._anchor_pose = bp.pose
                      self._anchor_inliers = int(bp.inliers)
                      self._anchor_best = bp.index
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

              # Places ride the same frame the ladder just used. Kept out
              # of the ladder itself on purpose: a place is evidence about the
              # VEHICLE, and the ladder is evidence about a TARGET. Blending
              # the two is how a lock starts following the floor.
              if self._closer_on:
                  self._place_tick(gray, header, now)

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
                      self._publish_correspondences(
                          self._anchor_pose, self._anchor_header or header)
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



def _stamp_key(header):
    """A hashable identity for a header stamp, or None when it carries none.

    Compared as (sec, nanosec) integers rather than a float: a float seconds
    value loses nanosecond resolution at ROS epoch magnitudes, so two different
    frames can compare equal -- which would silently pair the wrong ones, the
    exact failure this exists to prevent.
    """
    if header is None:
        return None
    st = header.stamp
    if st.sec == 0 and st.nanosec == 0:
        return None
    return (int(st.sec), int(st.nanosec))

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
