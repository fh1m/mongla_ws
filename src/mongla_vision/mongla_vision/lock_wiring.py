"""The steps LockNode.__init__ runs, in order: what it locks onto, the inputs,
the ladder's state, the rungs, loop closure, and the outputs.

Each step only creates state, publishers and subscriptions; nothing here
runs per frame. The order is __init__'s, and two comments below depend on it.
"""
import collections
import threading

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Range
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
try:                                   # noqa: SIM105
    from mongla_interfaces.msg import TargetCorrespondences
except ImportError:                    # message not built in this workspace
    # The LADDER must survive it. Correspondences are an addition to what this
    # node does, not a precondition, so an unbuilt interface costs the 6-DoF
    # path and nothing else -- the follower and anchor rungs still publish
    # `/lock`, which is what the control loop steers on.
    TargetCorrespondences = None
# OPTIONAL, like TargetCorrespondences above. A workspace that has not
# rebuilt mongla_interfaces must still get a working ladder: losing vehicle
# state costs the object-permanence rung, not the lock.
try:
    from mongla_interfaces.msg import MonglaState
except ImportError:                                              # noqa: BLE001
    MonglaState = None

from mongla_vision import qos as _qos
from mongla_vision.anchor import health as _health
from mongla_vision.anchor import loop_closure as _lc
from mongla_vision.tracking.follower import Follower


class LockWiringMixin:
    """Methods of `LockNode`; see `lock_node.py` for the node."""

    def _setup_target(self) -> None:
        """What to lock onto, the target's true width, and the frame buffers
        the callbacks fill.
        """
        # An explicit target_class wins and freezes the aim. Anything else and
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

    def _setup_pose_inputs(self, ns) -> None:
        """The medium the camera looks through, the correspondences publisher
        for the 6-DoF path, and camera_info unless frames arrive by reference.
        """
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
        # Composed: Frames by reference, no topics (`direct_feed`).
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

    def _setup_ladder_state(self) -> None:
        """The ladder's tunables (deck file, then launch), the vehicle state it
        reads, and the last detection.
        """
        # Set beside the state it guards, not where the other parameters are
        # READ. `_on_det` runs as soon as the subscription exists, which is
        # earlier in __init__ than the parameter block below, so reading it
        # there left a window where a detection arriving mid-construction
        # raised AttributeError inside the callback -- caught by
        # test_stamps.py, but it is a real startup race and not a test
        # artifact.
        # The deck file wins over the default, and a launch argument wins
        # Over both. One file (`~/.mongla/ladder.yaml`) changes the ladder
        # whichever launch started the node -- the pattern `loop_closure`
        # already proved and has a guard test for. An explicitly-passed launch
        # parameter still overrides it, so a mission can pin a value without
        # editing the deck.
        from .ladder_config import defaults as _ld_defaults, load_config as _ld_load
        self._ladder_cfg = _ld_load(log=self.get_logger())
        _ld_def = _ld_defaults()

        # NOT named `_cfg`: a DIFFERENT local `_cfg` exists further down
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
        # OBJECT PERMANENCE, downward camera only -- see `_remember_world`
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
        # HELD (see `_on_det`): a weak box where the lock already is, and the
        # centre of the last published lock it was judged against.
        self._held_box, self._held_conf, self._held_t = None, 0.0, 0.0
        self._held_header = None
        self._lock_centre = None
        # Bumped on every change of target class (#75). A ladder tick that
        # started under an older one publishes nothing.
        self._gen = 0
        # The detector's own cadence, from capture stamps of EVERY detection
        # message (a frame with no match still proves the detector ran). It
        # sizes how long a detection counts as live -- see `live_window`.
        self._det_msg_t = 0.0
        self._det_dt = None
        self._fresh = threading.Event()

    def _setup_rungs(self) -> None:
        """The follower and the anchor rungs, and the /lock publish rate.
        """
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

    def _setup_loop_closure(self, ns) -> None:
        """Loop closure on the downward camera: odometry, floor height and
        vehicle state in, position fixes out. Refused on the forward camera.
        """
        # ---- loop closure ------------------------------------------------
        # Configured by the node, not by launch. A switch threaded through
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
            # Downward only. A forward bank holds crops of TARGETS -- today's
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
                # The floor's own height, from the tile grating in
                # `flow_node`. That is a MEASUREMENT of the floor; flow_node
                # declines to publish the pool_depth arithmetic under this
                # name precisely because it is not one.
                self.create_subscription(Range, f'{ns}/floor_height',
                                         self._on_floor_height, 10)
                # The vehicle's own state. The ladder ran without ever
                # knowing which way the hull was pointing: it had odometry
                # (where) and floor height (how high) but no yaw, so a target
                # leaving the frame could not be turned into a world bearing
                # and `visibility.WorldTarget` -- the object-permanence rung --
                # was unreachable for want of one number the board already
                # publishes.
                #
                # `yaw_deg` is NaN when the board has no AHRS sample, and
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

    def _setup_outputs(self, ns) -> None:
        """/lock and the anchor decomposition out; detections, images and the
        mission's class filter in.
        """
        # Every anchor fit produces tx, ty, theta and scale. The ladder itself
        # reads only the corners, so theta and scale go out here rather than
        # being computed and dropped on every evaluation.
        #
        # Published, not acted on. Bumblebee reflex 1 is "redundancy beats
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
        # `candidates`, not `detections`: every box down to the detector's
        # association floor. The acting bar is applied HERE (`act_conf`), and
        # the boxes under it can only HOLD a target the bar acquired. With no
        # association floor set, the two topics carry the same boxes.
        self.create_subscription(Detection2DArray, f'{ns}/candidates',
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
