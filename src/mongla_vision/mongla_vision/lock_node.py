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
import os

os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import threading
import time
from typing import NamedTuple

import cv2
import numpy as np

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
from geometry_msgs.msg import Vector3Stamped
# ⚠ OPTIONAL, like TargetCorrespondences above. A workspace that has not
# rebuilt mongla_interfaces must still get a working ladder: losing vehicle
# state costs the object-permanence rung, not the lock.
try:
    from mongla_interfaces.msg import MonglaState
except ImportError:                                              # noqa: BLE001
    MonglaState = None
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Range
from mongla_vision.anchor import loop_closure as _lc
from mongla_vision.anchor import health as _health

from mongla_vision import qos as _qos
from mongla_vision.stamps import capture_monotonic
from mongla_vision.tracking.follower import Follower
from mongla_vision.tracking.lock_state import Rung, arbitrate, live_window
from .lock_setup import LockSetupMixin
from .lock_inputs import LockInputsMixin
from .lock_publish import LockPublishMixin

# ⛔ THE BAR FOR HEALING THE FOLLOWER FROM THE ANCHOR. Deliberately ABOVE the
# anchor's own trust bar (100): a box good enough to report is not
# automatically good enough to restart the fast rung on, because a reseeded
# follower then carries FULL authority for the rest of the gap. Set at the
# identity bar the re-ID work uses for "this is the same object", which is the
# strictest claim any rung in this package makes.
_ANCHOR_RESEED_INLIERS = 120



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


class _Tick(NamedTuple):
    """What one ladder tick works on: the newest frame and detection, snapped
    together under the lock so every stage sees the same instant."""
    gray: object
    header: object
    det_header: object
    det_box: object
    det_conf: float
    det_t: float
    det_cls: object
    live_s: float
    now: float


class LockNode(LockSetupMixin, LockInputsMixin, LockPublishMixin, Node):
    def __init__(self, node_name: str = 'mongla_lock', *,
                 parameter_overrides=None):
        super().__init__(node_name,
                         parameter_overrides=parameter_overrides or [])
        self._declare_parameters()

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
        # A decoded BGR frame handed over by reference (`submit_frame`), not
        # yet turned grey. Converted on the ladder's own thread, once per
        # tick it is actually used -- see `submit_frame`.
        self._pending = None
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
        # ⭐ COMPOSED: FRAMES BY REFERENCE, NO TOPICS (`direct_feed`).
        # Measured on the vehicle, 2026-10-03: composing this ladder into the
        # detector's process took both detectors from 71 Hz to 21.6 Hz
        # combined, with the anchor IDLE. py-spy put the cost in rclpy intake,
        # not in the ladder: image_raw and camera_info arriving at 134 Hz
        # each across two cameras, deserialised, dispatched by the executor
        # and copied by cv_bridge -- all under the GIL the detector workers
        # need. In-process, the camera already holds the decoded frame; a
        # reference costs nothing. Same switch, same name, as the detector's.
        self.declare_parameter('direct_feed', False)
        self._direct = bool(self.get_parameter('direct_feed').value)
        if not self._direct:
            self.create_subscription(CameraInfo, f'{ns}/camera_info',
                                     self._on_info, 10)
        # ⚠ SET BESIDE THE STATE IT GUARDS, NOT WHERE THE OTHER PARAMETERS ARE
        # READ. `_on_det` runs as soon as the subscription exists, which is
        # earlier in __init__ than the parameter block below, so reading it
        # there left a window where a detection arriving mid-construction
        # raised AttributeError inside the callback -- caught by
        # test_stamps.py, but it is a real startup race and not a test
        # artifact.
        # ⭐ THE DECK FILE WINS OVER THE DEFAULT, AND A LAUNCH ARGUMENT WINS
        # OVER BOTH. One file (`~/.mongla/ladder.yaml`) changes the ladder
        # whichever launch started the node -- the pattern `loop_closure`
        # already proved and has a guard test for. An explicitly-passed launch
        # parameter still overrides it, so a mission can pin a value without
        # editing the deck.
        from .ladder_config import defaults as _ld_defaults, load_config as _ld_load
        self._ladder_cfg = _ld_load(log=self.get_logger())
        _ld_def = _ld_defaults()

        # ⚠ NOT named `_cfg`: a DIFFERENT local `_cfg` exists further down
        # for the loop-closure block, and two closures with one name in one
        # __init__ is a shadowing bug waiting to be written.
        def _ladder(name):
            """An explicitly-set launch/param value wins; otherwise the deck
            file's. Equality with the shipped default is how "not explicitly
            set" is detected -- ROS does not distinguish a default from a
            value that happens to equal it, so a mission wanting the default
            AND a deck override of it must edit the deck."""
            v = self.get_parameter(name).value
            return v if v != _ld_def[name] else self._ladder_cfg[name]

        self._ladder = _ladder
        self._act_conf = float(_ladder('act_conf'))
        self._roll_rad = float('nan')
        self._imu_t = 0.0
        self._yaw_deg = float('nan')
        self._depth_m = float('nan')
        self._armed = False
        self._state_t = 0.0
        # ⭐ OBJECT PERMANENCE, downward camera only -- see `_remember_world`
        # for why the forward camera cannot have it without a measured range.
        try:
            from .tracking.visibility import WorldTarget
            self._world = WorldTarget()
        except Exception:                                        # noqa: BLE001
            self._world = None
        self._reseeds = 0
        self._vis_state = None
        self._vis_last = None
        self._det_box = None
        self._det_conf = 0.0
        self._det_cls = ''
        self._det_t = 0.0
        # The detector's own cadence, from capture stamps of EVERY detection
        # message (a frame with no match still proves the detector ran). It
        # sizes how long a detection counts as live -- see `live_window`.
        self._det_msg_t = 0.0
        self._det_dt = None
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
        # Recent bank yields, oldest first. Short on purpose: this answers
        # "is it getting worse NOW", not "what has the run been like".
        self._anchor_trend = collections.deque(maxlen=8)
        self._prepared = 0
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
                # ⭐ THE VEHICLE'S OWN STATE. The ladder ran without ever
                # knowing which way the hull was pointing: it had odometry
                # (where) and floor height (how high) but no yaw, so a target
                # leaving the frame could not be turned into a world bearing
                # and `visibility.WorldTarget` -- the object-permanence rung --
                # was unreachable for want of one number the board already
                # publishes.
                #
                # ⚠ `yaw_deg` is NaN when the board has no AHRS sample, and
                # every consumer must gate on that rather than treat NaN as
                # zero. A heading of "0 because we do not know" points the
                # memory due north, which is worse than having no memory.
                if MonglaState is not None:
                    self._state_sub = self.create_subscription(
                        MonglaState, '/mongla/state', self._on_state, 10)
                else:
                    self.get_logger().warn(
                        '[LOCK ] mongla_interfaces/MonglaState not built -- '
                        'the ladder runs without vehicle yaw, so any '
                        'world-frame target memory stays disabled. Rebuild '
                        'mongla_interfaces to enable it.')
                self._fix_pub = self.create_publisher(
                    PointStamped, '/mongla/localization/fix', 10)
                self.get_logger().info(
                    '[LOCK ] loop closure ON: places remembered every '
                    f'{self._places.period_s:.0f} s or '
                    f'{self._places.travel_m:.1f} m, offered as a fix at '
                    f'>= {_lc.CLOSURE_INLIERS} inliers.')

        # ⭐ THE DECOMPOSITION, PUBLISHED (Round 11, half-built until now).
        # Every anchor fit produces tx, ty, theta and scale, and until this
        # the ladder read `corners` alone -- the only consumer of theta/scale
        # in the whole tree was a debug text label. Two real signals were
        # computed and discarded on every evaluation.
        #
        # ⚠ PUBLISHED, NOT ACTED ON. Bumblebee reflex 1 is "redundancy beats
        # optimisation" -- their gate runs five pose sources. This is a SECOND
        # range estimate beside the follower's, fitted frame-to-REFERENCE
        # rather than frame-to-frame, and a second is worth more than a
        # replacement. A consumer that wants to fuse them can; nothing is
        # forced to.
        self._decomp_pub = self.create_publisher(
            Vector3Stamped, f'{ns}/anchor_decomposition', 10)
        self._pub = self.create_publisher(Detection2DArray, f'{ns}/lock',
                                          _qos.DETECTIONS)
        # Shared profiles, never a hand-rolled QoS: a RELIABLE/BEST_EFFORT
        # mismatch is answered by rclpy with one WARN and then silence --
        # clean launch, healthy nodes, zero frames. `mongla_vision.qos` owns
        # these topics and a contract test enforces it.
        self.create_subscription(Detection2DArray, f'{ns}/detections',
                                 self._on_det, _qos.DETECTIONS)
        if not self._direct:
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
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
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

    # -- inputs ------------------------------------------------------------- #
    def _on_img(self, msg):
        try:
            g = cv2.cvtColor(self._bridge.imgmsg_to_cv2(msg, 'bgr8'),
                             cv2.COLOR_BGR2GRAY)
        except Exception as exc:                          # noqa: BLE001
            # Counted and said, never swallowed: a camera publishing an
            # encoding this cannot convert starves the whole ladder, and the
            # only symptom was a lock that never formed. (Cloud audit, 2026-11-07.)
            self._n_bad_frames = getattr(self, '_n_bad_frames', 0) + 1
            if self._n_bad_frames in (1, 10, 100) or self._n_bad_frames % 1000 == 0:
                self.get_logger().error(
                    f'[LOCK ] frame {self._n_bad_frames} undecodable '
                    f'({msg.encoding!r}: {exc}) -- the ladder gets no image')
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

    # -- composed intake (`direct_feed`) ------------------------------------ #
    def wants_frame(self) -> bool:
        """Every frame: the stamp history must hold the frame a detection was
        computed on, or `_frame_for` refuses the snap. The camera decodes
        every frame for its own publish slot anyway."""
        return True

    def submit_frame(self, frame_bgr, header) -> None:
        """A decoded BGR frame from the camera in this process, by reference.

        No copy and no grey conversion here: this runs on the camera's capture
        thread, at camera rate. The ladder converts on its own thread, once,
        only for the frame it actually uses; `_frame_for` converts a history
        entry only when a snap needs it. The camera decodes a NEW array per
        frame, so holding the reference is safe.
        """
        with self._lock:
            self._pending = frame_bgr
            self._header = header
            self._recent.append((_stamp_key(header), frame_bgr))
        self._fresh.set()

    def _gray_now(self):
        """The newest frame in grey, converting a pending one on demand.

        For readers outside the ladder's tick (timers). The ladder converts
        only on ticks that read the frame, so `self._gray` alone can be a
        frame or more old.
        """
        with self._lock:
            pending, self._pending = self._pending, None
        if pending is not None:
            self._gray = cv2.cvtColor(pending, cv2.COLOR_BGR2GRAY)
        return self._gray

    def submit_info(self, info) -> None:
        """CameraInfo straight from the camera; `_on_info` drops repeats."""
        self._on_info(info)

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
                # Composed intake stores BGR; it turns grey only here, for
                # the one frame a snap needs.
                return g if g.ndim == 2 else cv2.cvtColor(
                    g, cv2.COLOR_BGR2GRAY)
        return None

    # A floor height older than this is not this vehicle's altitude any more.
    _FLOOR_H_MAX_AGE_S = 2.0

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
            # ⛔ AN ACTING BAR, NOT MERELY "NOT ZERO" (B-59). This accepted the
            # highest-scoring box with score > 0 as THE target, at full
            # authority. Measured on real Mirpur footage, the now-RETIRED
            # `gate_rescue_repair` graph (⚠ not `gate_sharks`, which replaced
            # it on 2026-09-28 and against which this bar has NOT been
            # re-derived -- see measured-bars section 59)
            # claims a gate on 90.8 % of GATE-FREE frames at conf 0.15 --
            # half-frame boxes over empty turquoise at 0.44-0.56, confirmed by
            # rendering them. Separation between gate-present and gate-absent
            # footage is +0.0 points at 0.15 and at 0.30, and +74.0 at 0.60.
            #
            # With no bar here the ladder locked onto open water: the follower
            # seeded on it, the anchor enrolled against it, and the vision
            # verbs drove the hull at it -- section 8.6's failure exactly.
            #
            # ⚠ The bar lives HERE and not only in the detector because the
            # detector's `conf` is deliberately permissive: the tracker needs
            # low-scoring boxes to maintain association across a gap. What may
            # be ASSOCIATED and what may be ACTED ON are different questions,
            # and conflating them turned a 0.15 detector bar into a 0.15
            # control bar.
            if score < self._act_conf:
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
            if 0.0 < self._det_msg_t < t:
                dt = t - self._det_msg_t
                # EMA over ~10 messages: one dropped frame should not swing
                # the window, a model swap to a slower detector should.
                self._det_dt = (dt if self._det_dt is None
                                else 0.9 * self._det_dt + 0.1 * dt)
            self._det_msg_t = t
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
        while rclpy.ok() and not self._stopping.is_set():
            try:
                self._ladder_tick()
            except Exception as exc:        # noqa: BLE001 -- B44 class
                faults += 1
                if faults in (1, 50):
                    self.get_logger().error(
                        f'[LOCK ] ladder fault #{faults} (thread kept alive): '
                        f'{type(exc).__name__}: {exc}. The lock reports NOT LOCKED '
                        f'while this persists, which consumers already handle.')
                time.sleep(0.05)

    # ── one tick of the ladder, stage by stage ─────────────────────────────
    # Each stage below was a span of `_loop` until 2026-11-07; the code and its
    # comments moved verbatim. The order is the order they run in.

    def _ladder_tick(self) -> None:
        """Take the newest frame, run each rung on it, arbitrate, publish."""
        f = self._take_frame()
        if f is None:
            return
        fb, fc = self._follow_rung(f)
        ab = ac = None
        anchor_ran = False
        if self._anchor is not None:
            anchor_ran = self._enrol_or_locate(f)
            ab, ac = self._anchor_box(f.gray, anchor_ran)
        # Places ride the same frame the ladder just used. Kept out
        # of the ladder itself on purpose: a place is evidence about the
        # VEHICLE, and the ladder is evidence about a TARGET. Blending
        # the two is how a lock starts following the floor.
        if self._closer_on:
            self._place_tick(f.gray, f.header, f.now)
        self._arbitrate_and_publish(f, fb, fc, ab, ac, anchor_ran)

    def _take_frame(self):
        """The newest frame and detection, or None when there is nothing to do.

        Grey is converted only when a later stage will read it."""
        if not self._fresh.wait(0.5):
            return None
        self._fresh.clear()
        with self._lock:
            pending, self._pending = self._pending, None
            gray = self._gray
            header = self._header
            det_header = self._det_header
            det_box, det_conf, det_t = (self._det_box, self._det_conf,
                                        self._det_t)
            det_cls = self._det_cls
            live_s = live_window(self._det_dt)
        now = time.monotonic()
        # ⭐ GREY ONLY WHEN THIS TICK READS IT. Every use below is
        # behind one of these: a detection to reseed on, a follower to
        # step, an anchor evaluation that is due, or place recognition.
        # Lost with none of them -- the common case between targets --
        # the downward camera's 104 frames a second were each converted
        # for nothing, on the thread that shares a GIL with the
        # detectors (26 % of the ladder's wall time, measured).
        need = (det_box is not None
                or (self._follower is not None
                    and self._follower.active)
                or (self._anchor is not None
                    and self._anchor.has_reference
                    and now >= self._anchor_next)
                or self._closer_on)
        if pending is not None and need:
            # Outside the lock: cvtColor releases the GIL, and the
            # camera thread must never wait on the ladder.
            gray = cv2.cvtColor(pending, cv2.COLOR_BGR2GRAY)
            self._gray = gray
        elif pending is not None:
            # Not read this tick; kept so a later reader (a timer) can
            # still convert the newest frame.
            with self._lock:
                if self._pending is None:
                    self._pending = pending
        if gray is None and need:
            return None
        return _Tick(gray=gray, header=header, det_header=det_header,
                     det_box=det_box, det_conf=det_conf, det_t=det_t,
                     det_cls=det_cls, live_s=live_s, now=now)

    def _follow_rung(self, f):
        """The fast rung: reseed LK on a detection, else step it. -> (box, conf)."""
        fb = fc = None
        if self._follower is not None:
            gray, det_box = f.gray, f.det_box
            if det_box is not None:
                # Reseed on every accepted detection: this is what keeps
                # the follower's drift bounded to a single gap.
                self._follower.reset(gray, det_box)
            elif self._follower.active:
                r = self._follower.step(gray)
                if r.ok:
                    fb, fc = r.xyxy, r.confidence
        return fb, fc

    def _enrol_or_locate(self, f) -> bool:
        """The anchor bank: enrol a checkpoint, or locate against the bank.

        Returns True when a locate ran this tick (the pose is fresh)."""
        gray, header, now = f.gray, f.header, f.now
        det_header, det_box, det_conf, det_cls = (f.det_header, f.det_box,
                                                  f.det_conf, f.det_cls)
        anchor_ran = False
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
        # ⭐ A FALLING TREND IS A REASON TO ENROL, alongside stale /
        # uncovered / room. It is the only one of the four that is
        # PREDICTIVE: the others describe the bank, this describes
        # where the view is heading. It licenses nothing else --
        # it never claims the target is gone.
        falling = False
        if len(self._anchor_trend) >= _health.TREND_MIN_SAMPLES:
            tr = _health.trend(self._anchor_trend)
            if tr.prepare:
                falling = True
                self._prepared += 1
                if self._prepared == 1 or self._prepared % 20 == 0:
                    self.get_logger().info(
                        f'[LOCK ] anticipating a loss: {tr.reason}')
        if det_box is not None and may_try and (
                falling or
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
            # ⭐ ANTICIPATE, do not merely react. A falling match
            # quality predicts the next seconds will be worse, and
            # the reference worth having is the one from BEFORE the
            # detection is lost. Snapping then is cheap and
            # reversible; snapping after costs the very view it
            # needed.
            self._anchor_trend.append(int(bp.inliers))
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
        return anchor_ran

    def _anchor_box(self, gray, anchor_ran: bool):
        """The anchor's box and confidence, and the follower reseeded from it."""
        ab = ac = None
        p = self._anchor_pose
        # ⭐ EVERY fit publishes its decomposition, including the
        # ones with no usable corner quad -- theta and scale are
        # fitted from the same inliers and are valid whether or not
        # the warped quad is. Gating them on `corners` would discard
        # the signal exactly when the box is least trustworthy.
        if anchor_ran:
            self._publish_decomposition(p, self._anchor_header)
        if p is not None and p.ok and p.corners is not None:
            q = np.asarray(p.corners, np.float32)
            ab = (float(q[:, 0].min()), float(q[:, 1].min()),
                  float(q[:, 0].max()), float(q[:, 1].max()))
            ac = p.confidence
            # ⭐⭐ THE LADDER HEALS UPWARD, NOT ONLY DOWNWARD.
            #
            # Until this, `_follower.reset()` was called on an
            # accepted DETECTION and nowhere else. So when LK's
            # points died mid-gap -- which is exactly what the
            # survival and forward-backward gates exist to detect --
            # the fast ~50 Hz rung stayed dead until a detection
            # came back, even though the anchor was still finding
            # the target every 222 ms. The ladder had a one-way
            # ratchet: every rung could fall, none could climb.
            #
            # The anchor's box is a legitimate seed. It is fitted
            # frame-to-REFERENCE, so unlike a follower box it has
            # not drifted, and `goodFeaturesToTrack` inside it finds
            # the same corners LK wanted. Reseeding costs one GFTT
            # call (measured 8.0 ms for the whole LK step) and buys
            # back the 50 Hz rung for the rest of the gap.
            #
            # ⛔ ONLY WHEN THE FOLLOWER IS DEAD AND THE ANCHOR IS
            # CONFIDENT. Reseeding a LIVE follower would replace a
            # frame-fresh box with one up to 222 ms old, and
            # reseeding from a weak fit would restart the fast rung
            # on a bad box and give it full authority -- exactly the
            # failure the acting bar exists to prevent, arriving by
            # another door.
            if (self._follower is not None
                    and not self._follower.active
                    and anchor_ran
                    and int(self._anchor_inliers)
                    >= _ANCHOR_RESEED_INLIERS):
                n = self._follower.reset(gray, ab)
                if n:
                    self._reseeds += 1
                    if self._reseeds in (1, 10, 100):
                        self.get_logger().info(
                            f'[LOCK ] follower reseeded from the '
                            f'anchor ({n} pts, {self._anchor_inliers} '
                            f'inliers) -- reseed #{self._reseeds}')
        return ab, ac

    def _arbitrate_and_publish(self, f, fb, fc, ab, ac, anchor_ran: bool) -> None:
        """Pick the rung, publish the lock, and the anchor's correspondences."""
        now, header, live_s = f.now, f.header, f.live_s
        det_t, det_box, det_conf, det_header = (f.det_t, f.det_box, f.det_conf,
                                                f.det_header)
        st = arbitrate(now=now, last_detection_t=det_t,
                       detection=det_box, detection_conf=det_conf,
                       follow=fb, follow_conf=fc or 0.0,
                       anchor=ab, anchor_conf=ac or 0.0,
                       live_s=min(live_s, self._full),
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

    def destroy_node(self):
        """Stop the ladder thread BEFORE the publishers go.

        ⛔ Found by tools/composed_lock_harness.py: destroyed while its thread
        was mid-tick, the ladder published on a destroyed handle and logged
        `ladder fault #1 ... InvalidHandle` on every shutdown -- an ERROR on
        every Ctrl-C, which teaches an operator to ignore that line.
        """
        if hasattr(self, '_stopping'):     # a half-built node has no thread
            self._stopping.set()
            self._fresh.set()              # wake a tick waiting for a frame
        t = getattr(self, '_thread', None)
        if t is not None and t is not threading.current_thread():
            t.join(2.0)
        return super().destroy_node()

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
    from mongla_localization.orphan import die_with_parent
    die_with_parent()        # an orphan holds devices and duplicates topics
    rclpy.init()
    node = LockNode()
    from mongla_localization.params_file import refuse_unknown
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
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
