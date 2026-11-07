#!/usr/bin/env python3
"""MonglaMission -- the mission DSL.

Two namespaces, one mental model:

Open-loop motion verbs sit directly on `mongla`:

    mongla.arm() / mongla.disarm()
    mongla.set_depth(meters)
    mongla.move_forward(seconds, gain=60)
    mongla.move_back(seconds)
    mongla.move_left(seconds) / mongla.move_right(seconds)
    mongla.yaw_left(degrees) / mongla.yaw_right(degrees)
    mongla.turn(heading_deg)        -- absolute heading, direction auto-selected
    mongla.pause(seconds) / mongla.stop()

Each of these is ONE move the SROT board runs and brakes itself; the board
holds depth and heading at 500 Hz between them.

Closed-loop vision lives under `mongla.vision` as exactly two verbs:

    mongla.vision.align(target, *, lat=None, yaw=None, depth=None,
                        err=40, duration=20, gain=30, fallback=None, camera=None)
        Centre the target on the selected axes. Each of lat/yaw/depth is
        None (axis off) or a signed pixel offset from centre (0 = centre,
        +=right/below, -=left/above). lat strafes and yaw rotates; depth is
        image-Y, and on the downward camera it drives SURGE (the board holds
        the real depth). At least one axis is required.

    mongla.vision.move(target, *, fwd=95, mode='area', maintain=None,
                       hold=None, err=40, duration=20, gain=30,
                       fallback=None, camera=None)
        Drive forward until the bbox fills `fwd` % of the frame. mode is
        'area' | 'width' | 'height' (slalom uses 'height'). `maintain` holds
        a px lateral offset while driving; `hold` station-keeps after reach.
        Never re-centres; depth is held by the board.

Both return a `VisionResult(ok, reason, code, last_err_px, fill)` and
NEVER raise on a miss — on timeout/loss they log "not aligned/reached"
and the mission continues. `gain` is a hard max-speed cap (% thrust).

`fallback` is a mission-authored search function called on target loss:

    def creep_forward(mongla):          # one short maneuver, then return
        mongla.move_forward(0.6, gain=35)

    def sweep_yaw(mongla, should_stop):  # longer self-polling sweep
        for ang in (15, -30, 30):
            mongla.turn(mongla.head() + ang)
            if should_stop():
                return

    mongla.vision.align('gate', yaw=0, lat=0, fallback=creep_forward)

Firing replaces the old lock-fire verb with align + the `fire` control verb:

    if mongla.vision.align('hole', yaw=0, lat=0, depth=0, err=12).ok:
        mongla.fire(1)

Vision queries -- detected() / wait_for() / where():

    # branch ONCE on what's visible now (an `if` runs once -- it does NOT loop)
    if mongla.detected('gate'):
        mongla.vision.align('gate', yaw=0, lat=0)

    # search WHILE MOVING -- this needs a `while`, not an `if`
    while not mongla.detected('red_pipe'):
        mongla.move_left(2)
    mongla.move_forward(3)

    # acquire while stationary, no busy-loop: wait_for blocks until seen/timeout
    if mongla.wait_for('gate', timeout=8):
        mongla.vision.align('gate', yaw=0, lat=0)
    else:
        sweep_yaw(mongla, should_stop)   # the fallback defined above

    # steer by bearing: 'left' | 'center' | 'right' | 'unknown'
    {'left':  lambda: mongla.yaw_left(20),
     'right': lambda: mongla.yaw_right(20),
    }.get(mongla.where('gate'), lambda: mongla.move_forward(1))()

    # ClassRef handles + camera / freshness overrides work everywhere:
    mongla.detected(mongla.models.gate.gate)
    mongla.detected('flare', camera='downward', stale_after=2.0)

These are client-side reads of the same `/detections` stream the control
loop acts on. Each pumps the node so the answer reflects the CURRENT frame
(not a stale cache); the default camera is subscribed eagerly so the first
query never false-negates on DDS discovery. They run between goals (safe in
search loops and inside a vision `fallback`), never during one.

Model context (multi-model missions):

    mongla.models(
        gate   = 'gate_flare_medium_100ep',
        slalom = 'slalom_combined',
    )
    mongla.vision.align(mongla.models.gate.gate, yaw=0, lat=0)
    mongla.vision.move(mongla.models.gate.gate, fwd=80, mode='height')

    # Strict class list (validates attribute access at handle time):
    mongla.models(gate=('gate_flare_medium_100ep', ['gate', 'flare']))
    mongla.vision.align(mongla.models.gate.gate, yaw=0)   # OK
    # mongla.vision.align(mongla.models.gate.typo, yaw=0)  # → AttributeError

When a ClassRef is passed as target, the DSL automatically calls
set_model() + set_classes() before sending the goal — no explicit
mongla.set_classes() or mongla.use() needed per verb.

Canonical competition task pattern (gate pass):

    mongla.models(gate='gate_flare_medium_100ep')
    mongla.set_depth(-1.2)
    while not mongla.detected(mongla.models.gate.gate):
        mongla.move_forward(0.6, gain=35)
    mongla.vision.align(mongla.models.gate.gate, yaw=0, lat=0,
                        fallback=creep_forward)
    mongla.vision.move(mongla.models.gate.gate, fwd=80, mode='area',
                       fallback=creep_forward)

Detector control (manual — ClassRef targets do this automatically):
    mongla.set_classes('gate')         # only gate detections
    mongla.set_classes('gate,flare')   # gate + flare
    mongla.set_classes('')             # all classes
    mongla.set_model('combined')       # switch model in registry
    mongla.use('combined', 'gate')     # switch model + class in one call

Tunable live (between runs, no rebuild):
    ros2 param set /mongla_manager vision.kp_yaw 80.0
    ros2 param set /mongla_manager vision.kp_lat 60.0
    ros2 param set /mongla_manager vision.lost_grace_s 1.0

Where each part lives. `MonglaMission` is one class assembled from one file per
concern, so a mission author reads one class and a bug report names a file:

  mongla_dsl.py      the verbs, arm/disarm, set_mode, fire
  dsl_plan.py        task(), use_budget(), worth_attempting(), step(), run_plan()
  dsl_report.py      vision evidence, the countdown, the scoreboard, the run folder
  dsl_detections.py  what the cameras see: detected(), wait_for(), where(), outline()
  dsl_position.py    where the vehicle is: bearing_to(), range_to(), fix_position(),
                     pose(), anchor_heading()
  dsl_cameras.py     which camera is live (one detector at a time on the Hailo)
  dsl_detector.py    set_model(), set_classes(), set_conf(), set_vision_param()
  dsl_support.py     parsing detection messages, and the small shared helpers
  vision_dsl.py      `mongla.vision.*`, the closed-loop vision verbs
"""

from __future__ import annotations

from dataclasses import dataclass

import math
import os
import time as _time

from .model_context import ModelRegistry
from .dsl_cameras import CameraMixin
from .dsl_detections import SeeMixin
from .dsl_detector import DetectorMixin
from .dsl_position import WhereMixin
from .dsl_support import _to_float
from .vision_dsl import _VisionDSL  # noqa: F401 -- re-exported; used by MonglaMission

from .client import MoveFailed, MoveRejected
from .dsl_plan import PlanMixin
from .dsl_report import ReportMixin


def _format_outcome(cmd: str, result) -> str:
    return (f'  {cmd:<22s} final={result.final_value:+.3f} '
            f'err={result.error_value:+.3f}  ({result.message})')


# --------------------------------------------------------------------------- #
#  Per-run artifact folder (scorecards live here; rosbags land alongside)      #
# --------------------------------------------------------------------------- #
# One place to grab everything after a pool session. Override the parent with
# MONGLA_RUN_DIR (pool_record.sh writes bags into the same tree). Default
# ~/mongla_runs so a scorecard never litters the CWD the operator launched from.







@dataclass(frozen=True)
class FireOutcome:
    """What `mongla.fire()` did. `if mongla.fire(ch):` is True only if it fired."""
    fired: bool
    channel: int
    reason: str = ''

    def __bool__(self) -> bool:
        return self.fired


class MonglaMission(PlanMixin, ReportMixin, SeeMixin, WhereMixin, CameraMixin, DetectorMixin):
    """Mission-author API. Wraps MonglaClient with human verbs + sticky context.

    Parameters
    ----------
    client : MonglaClient
        The blocking action client.
    log : rclpy logger
        Anything with `.info(...)`. One outcome line is printed per verb call.
    camera, target : str
        Sticky defaults. Override per call with `camera=` / `target=`.
    """

    def __init__(self, client, log, *, camera: str = 'forward',
                 target: str = 'person'):
        self.client = client
        self.log    = log
        self.camera = camera
        self.target = target
        self.vision = _VisionDSL(self)
        self.models = ModelRegistry()
        # Detection cache: camera -> (monotonic_stamp, [DetRecord, ...]).
        # Refreshed by ROS callbacks; detected()/where() actively pump the node
        # so the cache is fresh at the call instant (not just during a send()).
        self._det_cache: dict[str, tuple[float, list]] = {}
        # Per-class last-seen monotonic stamp, per camera: {cam: {class: stamp}}.
        # detected()/wait_for() read this (NOT the latest frame) so a class that
        # flickers out of individual raw frames at low FPS still counts as present
        # within a recency window -- the reacquire-side analogue of the control
        # loop's lost_grace_s. where()/where_offset() deliberately stay on the
        # latest frame (bearing must be current, never a stale remembered spot).
        self._det_seen:  dict[str, dict[str, float]] = {}
        self._det_subs:  dict[str, object] = {}   # detection subs (kept alive)
        self._info_subs: dict[str, object] = {}   # camera_info subs (kept alive)
        self._img_size:  dict[str, tuple] = {}    # camera -> (width, height)
        self._cam_k:     dict[str, tuple] = {}    # camera -> (fx, fy, cx, cy), in AIR, at _img_size
        # 'water' (default) rectifies pixels through the flat port before any
        # metric use; 'air' uses the calibration K as a plain pinhole (bench).
        # Same parameter, same default as lock_node / pnp_node / flow_node.
        self.medium: str = str(os.environ.get('MONGLA_MEDIUM', 'water')).strip().lower()
        self._det_warm:  set[str] = set()         # cameras that have produced a frame
        # Detector parameter control (in-process, reliable -- replaces flaky
        # subprocess `ros2 param set`). node-name -> SetParameters client; the
        # set of nodes whose existence has been confirmed (so the loud preflight
        # probes each detector node at most once).
        self._param_clients: dict[str, object] = {}
        self._detector_ok:   set[str] = set()
        # Single-live-detector orchestration (the Hailo holds one model's
        # context at a time: never run two detectors at once). `_live_camera` is the camera whose detector is currently
        # resumed; a vision verb / use_camera on a DIFFERENT camera pauses the old
        # one, resumes the new, points the HUD at it, and settles -- see
        # _activate_camera. None = nothing resumed yet (launch starts both paused).
        self._live_camera: str | None = None
        # Cameras whose detector we have already probed for the first-query
        # resume. Probing is not free (DDS discovery settle), and a repeat
        # probe cannot tell us anything new inside one mission.
        self._resume_probed: set[str] = set()
        # camera -> is its detector on the graph. Cached: a detector that
        # is up stays up, and one that never came up will not appear.
        self._camera_available: dict[str, bool] = {}
        # None until a landmark anchors the heading. Never assumed.
        self._heading_offset: float | None = None
        self._active_cam_pub = None   # lazily-created latched String publisher (HUD follow)
        self._fix_pub = None          # lazily-created PointStamped publisher (pool fix -> filter)
        self._odom_sub = None         # lazily-created Odometry subscription (filter -> mission)
        self._odom = None             # latest Odometry, or None if the filter is not running
        self._heading_pub = None      # lazily-created latched Float32 (anchored heading)
        self._pose_frame_warned = False
        # Scoreboard: ordered list of (cmd, success, elapsed_s, message)
        self._scoreboard: list[dict] = []
        # OPT-IN run clock (`use_budget`). None = no rationing: every
        # `worth_attempting` says attempt, exactly as before it existed.
        self._budget = None
        self._mission_start: float = _time.monotonic()
        # Wall clock of the same instant: the board's latched flare order is stamped in
        # wall clock, and an order from before this mission must be refused.
        self._mission_start_wall: float = _time.time()
        # Eager-subscribe the default camera so DDS discovery completes before
        # the first detected()/where() -- otherwise the first poll false-negates
        # (discovery takes 50-500 ms) and a `while not detected()` loop hangs.
        self._subscribe_detections(self.camera)

    # ================================================================== #
    #  Single send + log helper                                           #
    # ================================================================== #

    def _send(self, cmd: str, **fields):
        fields = {k: _to_float(v) for k, v in fields.items()}
        t0     = _time.monotonic()
        result = self.client.send(cmd, **fields)
        elapsed = _time.monotonic() - t0
        # Level by outcome. Every verb used to log at INFO, so a failure and a
        # success were visually identical in a pool scrollback and differed only
        # in the message text -- with the commonest failure (a move stall) buried
        # among the successes. The operator's eye is the last line of defence
        # during a run; give it something to catch.
        ok = bool(getattr(result, 'success', False))
        (self.log.info if ok else self.log.warning)(
            _format_outcome(cmd, result) if ok
            else '!! ' + _format_outcome(cmd, result).lstrip())
        self._scoreboard.append({
            'cmd':     cmd,
            'success': bool(getattr(result, 'success', False)),
            'elapsed': round(elapsed, 2),
            'msg':     str(getattr(result, 'message', '')),
            # First 8 hex of the action goal UUID -- the manager logs the same
            # 8 on its [ACT] line, so a row can be found in the vehicle log.
            'goal':    str(getattr(self.client, 'last_goal_id', '') or '')[:8],
        })
        return result

    # ================================================================== #
    #  Vision queries -- detected() / wait_for() / where()                #
    # ================================================================== #
    #
    # These read the raw /detections stream the control loop and HUD act on,
    # so a query agrees with what the AUV is steering toward. They are
    # client-side cache checks (NOT action goals), and each actively pumps the
    # node so the cache is fresh at the call instant. Safe ONLY between goals
    # (the mission is single-threaded; a query never runs during a send()).
    #
    # First-frame discovery: the default camera is subscribed eagerly in
    # __init__; other cameras subscribe on first query and use a longer pump
    # window until they have produced a frame (see _pump_detections).

    # Timeouts (seconds) for the per-call pump. _PUMP_WARM_S must be >= ~2 real
    # frame-periods or the pump returns having seen no frame newer than `start`,
    # and the query falls back to the per-class last-seen window (see _present).
    # Sized for a slow detector (3-4 Hz -> ~0.25-0.33 s/frame), so 0.40 s covers
    # ~1.5 frames even there; the Hailo path runs far faster (14.7-27 Hz,
    # cameras.yaml). The pump
    # early-returns the instant a fresh frame lands, so this only costs latency
    # on a miss. Raise it if your real FPS is lower.
    _PUMP_WARM_S = 0.40   # ~1.5 frame-periods at 3-4 Hz; early-returns when fresh
    _PUMP_COLD_S = 0.60   # first frame after subscribe: covers DDS discovery
    _PUMP_SLICE_S = 0.02  # spin_once granularity inside the pump

    # Detector warm-up gate (cold-detector-after-switch guard, see
    # _wait_detector_warm). Right after a camera/model/class switch the detector
    # needs a moment to produce its first frame under the new config; a vision
    # verb that starts before then can drop into an autonomous fallback SEARCH
    # within its lost_grace. WARMUP_S bounds the wait; FRESH_S is how recent a
    # /detections frame must be to count the detector "producing".
    _DETECTOR_WARMUP_S = 2.5
    _DETECTOR_FRESH_S  = 1.0




















    # ================================================================== #
    #  Power / mode                                                        #
    # ================================================================== #

    def arm(self, *, timeout: float = 15.0, required: bool = True):
        """Arm the vehicle. RAISES on failure unless `required=False`.

        This is the one verb where continuing after failure is always wrong.
        Every subsequent move is refused by the board ("SROT_MOVE refused: arm
        first"), so a mission that arms unsuccessfully and carries on runs its
        entire sequence disarmed: the hull sits still, every verb logs a failure,
        the run ends "complete", and a pool slot is gone. Measured before this
        change: 16 of 16 missions called `arm()` and discarded the result --
        including `task_full_2026`, the full competition run.

        That is NASA JPL Power-of-10 rule 7 (Holzmann, IEEE Computer 2006: "the
        return value of non-void functions must be checked by each calling
        function") and it is the same shape as J02 on the register -- the one
        call a step exists to make, with its failure swallowed.

        Raising is SAFE here: `mission.py`'s runner calls `_safe_shutdown` on
        both the success and the unhandled-exception path, so the vehicle is
        still released, stopped and disarmed.

        `required=False` restores the old behaviour for a diagnostic that
        genuinely wants to observe a refusal and keep going.
        """
        result = self._send('arm', timeout=timeout)
        if required and not bool(getattr(result, 'success', False)):
            reason = str(getattr(result, 'message', '')) or 'no reason given'
            self.log.error(
                f'[MISS ] ARM FAILED -- aborting instead of running the mission '
                f'disarmed. Reason: {reason}')
            raise MoveFailed(f'arm failed: {reason}')
        budget = getattr(self, '_budget', None)
        if budget is not None and not budget.started and bool(getattr(result, 'success', False)):
            budget.start()
            self.log.info(f'[BUDG ] run clock started on arm: '
                          f'{budget.remaining_s():.0f} s usable '
                          f'(+{budget.reserve_s:.0f} s reserve)')
        return result

    # ================================================================== #
    #  Run budget -- OPT-IN (Tier 5.1)                                    #
    # ================================================================== #

    def disarm(self, *, timeout: float = 20.0):
        return self._send('disarm', timeout=timeout)

    def set_mode(self, name: str, *, timeout: float = 8.0):
        return self._send('set_mode', target_name=name, timeout=timeout)

    # ================================================================== #
    #  Stop / pause                                                        #
    # ================================================================== #

    def stop(self):
        return self._send('stop')

    def surface(self, timeout: float = 60.0):
        """Surface: the board's SURFACE mode, waiting up to `timeout` s for the
        depth to read ~0. Safe to call during a running mission."""
        return self._send('surface', timeout=float(timeout))

    def head(self) -> float:
        """Return live heading (degrees) at call time; NaN if none came back.

            h = mongla.head()
            mongla.turn(h + 90)

        The server REFUSES (MoveFailed) when the board has no fresh heading,
        so a turn is never computed from an absent one.
        """
        result = self._send('head')
        return result.final_value if result is not None else math.nan

    def pause(self, seconds: float):
        return self._send('pause', duration=float(seconds))

    def fire(self, channel: int):
        """Activate payload BOARD channel `channel` (1..16).

        On srot this is the board's own PCA9685 channel -- the same n as
        `SERVO{n}_ROLE` -- with no host-side map. The board's role config decides:
        a SWITCH channel fires, a PWM channel is REFUSED (it is the on-board arm).
        `ros2 run mongla_manager connect` lists which channels are fireable.

        Returns a `FireOutcome`: truthy ONLY when the board ACCEPTED the
        activation -- it energised the channel -- with the server's reason either
        way. That is not "the round left": nothing on the vehicle senses a
        launcher or dropper actually releasing, so a jammed one still reads
        truthy. Score a shot on what the camera sees afterwards, not on this.

        A refusal is a task outcome, not a mission failure. A refused shot
        sets success=False, which the client raises as MoveFailed -- so the
        docstring's promise that "a mission can tell refused from link-down"
        was never true: the mission saw an exception, and a practice run with
        an unassigned channel (B53) aborted the whole run at the torpedo. The
        vision verbs never raise on a miss; neither does this. Ctrl-C still
        propagates (it is not a MoveFailed).
        """
        try:
            r = self._send('fire', fire_channel=float(channel))
        except (MoveFailed, MoveRejected) as exc:
            self.log.error(f'[FIRE ] ch={channel} NOT FIRED -- {exc}')
            return FireOutcome(False, int(channel), str(exc))
        return FireOutcome(True, int(channel), str(getattr(r, 'message', '') or ''))

    # ================================================================== #
    #  Open-loop motion                                                    #
    # ================================================================== #

    def set_depth(self, meters: float, *, timeout: float = 30.0,
                  settle: float = 0.0):
        return self._send('set_depth',
                          target=float(meters),
                          timeout=timeout, settle=settle)

    def move_forward(self, seconds: float, *, gain: float = 80.0,
                     settle: float = 0.0):
        return self._send('move_forward',
                          duration=float(seconds),
                          gain=gain, settle=settle)

    def move_back(self, seconds: float, *, gain: float = 80.0,
                  settle: float = 0.0):
        return self._send('move_back',
                          duration=float(seconds),
                          gain=gain, settle=settle)

    def move_left(self, seconds: float, *, gain: float = 80.0,
                  settle: float = 0.0):
        return self._send('move_left',
                          duration=float(seconds),
                          gain=gain, settle=settle)

    def move_right(self, seconds: float, *, gain: float = 80.0,
                   settle: float = 0.0):
        return self._send('move_right',
                          duration=float(seconds),
                          gain=gain, settle=settle)

    def yaw_left(self, degrees: float, *, timeout: float = 30.0,
                 settle: float = 0.0):
        return self._send('yaw_left',
                          target=float(degrees),
                          timeout=timeout, settle=settle)

    def yaw_right(self, degrees: float, *, timeout: float = 30.0,
                  settle: float = 0.0):
        return self._send('yaw_right',
                          target=float(degrees),
                          timeout=timeout, settle=settle)

    def turn(self, degrees: float, *, timeout: float = 30.0,
             settle: float = 0.0):
        """Rotate to absolute heading `degrees` (0-360) via shortest arc.

        Direction is chosen automatically — no left/right prefix needed.
        Absolute headings need the board's magnetic reference (YAW_REF LOCKED).

        Examples::

            mongla.turn(90)          # face east, from any current heading
            mongla.turn(0)           # face north (shortest path)
            mongla.turn(270)         # face west
        """
        return self._send('turn',
                          target=float(degrees),
                          timeout=timeout, settle=settle)

    def style_roll(self, *, flips: int = 1, timeout: float = 20.0):
        """`flips` x 360 deg roll, run by the board. `timeout` bounds the whole manoeuvre."""
        return self._send('style_roll', flips=flips, timeout=timeout)

    # ================================================================== #

    # Settle after a live-detector switch so the resumed detector's first frames
    # land (and the paused one's queue drains) before the loop steers on them --
    # otherwise the first align tick acquires on a cold/empty detector. Bringing a
    # camera + its detector + the HUD onto a fresh target takes real time (stream
    # re-latch + first inference tick + tracker warm-up), so give generous headroom:
    # a switch happens once per task, not per loop, so an extra second is free
    # insurance against acquiring on a stale/empty frame. Override per-mission via
    # ``mongla.cam_switch_settle_s = <seconds>`` before the switch if a run needs
    # it snappier or slower.
    _CAM_SWITCH_SETTLE_S = 1.5

    # Known dual-camera detector cameras. On every switch we pause EVERY one of
    # these except the target (not just the previously-live one), so exclusivity
    # holds from the FIRST use_camera even if the node was launched paused:=false
    # (both detectors inferring from t=0 -> the concurrent-inference OOM). Pausing
    # an absent one is a quiet no-op, so single-camera runs are unaffected.
    _KNOWN_CAMERAS = ('forward', 'downward')

    # ------------------------------------------------------------------ #
    #  Model provenance -- `vision_info` (OPT-IN: subscribed on first use) #
    # ------------------------------------------------------------------ #



    # ------------------------------------------------------------------ #
    #  Outlines -- `contours` (OPT-IN: subscribed on first use)            #
    # ------------------------------------------------------------------ #






















    # ================================================================== #
    #  Vision detector control                                             #
    # ================================================================== #


    # How long `_detector_present` may spin waiting for DDS discovery. Only the
    # FIRST query pays it, and only when the node has not been seen yet.
    _DISCOVERY_SETTLE_S = 0.4










    # Per-camera node names, one rule for the whole stack. A subsystem the
    # mission can tune is a subsystem it can adapt mid-run; one that is
    # launch-only is frozen at the value someone typed before the water.
    _NODE_SUFFIX = {
        'detector': 'mongla_detector_{cam}',
        'camera':   'mongla_camera_{cam}',
        'tracker':  'mongla_tracker_{cam}',
        'lock':     'mongla_lock_{cam}',
        'pnp':      'mongla_pnp_{cam}',
        'posefuse': 'mongla_pose_fuse_{cam}',
        'flow':     'mongla_flow_velocity',      # one node, not per camera
        'manager':  'mongla_manager',            # ditto
    }





    # Manager node that owns the vision.* tunables (declares/re-snapshots them).
    _MANAGER_NODE = '/mongla_manager'





    # ================================================================== #
    #  Mission countdown                                                   #
    # ================================================================== #

    # ================================================================== #
    #  Mission scoreboard                                                  #
    # ================================================================== #

    # ================================================================== #
    #  Escape hatch -- unknown verbs fall through to raw client           #
    # ================================================================== #

    def __getattr__(self, name: str):
        if name == 'send':
            def _send_with_log(cmd, **fields):
                return self._send(cmd, **fields)
            return _send_with_log

        attr = getattr(self.client, name)
        if not callable(attr):
            return attr

        def _wrapped(*args, **kwargs):
            result = attr(*args, **kwargs)
            if hasattr(result, 'final_value') and hasattr(result, 'message'):
                self.log.info(_format_outcome(name, result))
            return result

        return _wrapped
