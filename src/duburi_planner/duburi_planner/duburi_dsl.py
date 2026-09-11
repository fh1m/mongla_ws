#!/usr/bin/env python3
"""DuburiMission -- the mission DSL.

Two namespaces, one mental model:

Open-loop motion verbs sit directly on `duburi`:

    duburi.arm() / duburi.disarm()
    duburi.set_depth(meters)
    duburi.move_forward(seconds, gain=60)
    duburi.move_back(seconds)
    duburi.move_left(seconds) / duburi.move_right(seconds)
    duburi.yaw_left(degrees) / duburi.yaw_right(degrees)
    duburi.turn(heading_deg)        -- absolute heading, direction auto-selected
    duburi.arc(target_yaw, seconds=4, gain=50)   # curve onto an absolute heading
    duburi.lock_heading(degrees)  / duburi.release_heading()
    duburi.pause(seconds) / duburi.stop()

Depth is held automatically by ArduSub's onboard ALT_HOLD. `set_depth`
engages the mode and drives to the target; the autopilot holds it afterwards.

Closed-loop vision lives under `duburi.vision` as exactly two verbs:

    duburi.vision.align(target, *, lat=None, yaw=None, depth=None,
                        err=40, duration=20, gain=30, fallback=None, camera=None)
        Centre the target on the selected axes. Each of lat/yaw/depth is
        None (axis off) or a signed pixel offset from centre (0 = centre,
        +=right/below, -=left/above). lat+yaw are horizontal (Ch6 strafe /
        Ch4 rotate); depth is vertical. At least one axis is required.

    duburi.vision.move(target, *, fwd=95, mode='area', maintain=None,
                       hold=None, err=40, duration=20, gain=30,
                       fallback=None, camera=None)
        Drive forward until the bbox fills `fwd` % of the frame. mode is
        'area' | 'width' | 'height' (slalom uses 'height'). `maintain` holds
        a px lateral offset while driving; `hold` station-keeps after reach.
        Never re-centres; depth is left to ArduSub's depth-hold.

Both return a `VisionResult(ok, reason, code, last_err_px, fill)` and
NEVER raise on a miss — on timeout/loss they log "not aligned/reached"
and the mission continues. `gain` is a hard max-speed cap (% thrust).

`fallback` is a mission-authored search function called on target loss:

    def creep_forward(duburi):          # one short maneuver, then return
        duburi.move_forward(0.6, gain=35)

    def sweep_yaw(duburi, should_stop):  # longer self-polling sweep
        for ang in (15, -30, 30):
            duburi.turn(duburi.head() + ang)
            if should_stop():
                return

    duburi.vision.align('gate', yaw=0, lat=0, fallback=creep_forward)

Firing replaces the old lock-fire verb with align + the `fire` control verb:

    if duburi.vision.align('hole', yaw=0, lat=0, depth=0, err=12).ok:
        duburi.fire(1)

Vision queries -- detected() / wait_for() / where():

    # branch ONCE on what's visible now (an `if` runs once -- it does NOT loop)
    if duburi.detected('gate'):
        duburi.vision.align('gate', yaw=0, lat=0)

    # search WHILE MOVING -- this needs a `while`, not an `if`
    while not duburi.detected('red_pipe'):
        duburi.move_left(2)
    duburi.move_forward(3)

    # acquire while stationary, no busy-loop: wait_for blocks until seen/timeout
    if duburi.wait_for('gate', timeout=8):
        duburi.vision.align('gate', yaw=0, lat=0)
    else:
        duburi.recover()

    # steer by bearing: 'left' | 'center' | 'right' | 'unknown'
    {'left':  lambda: duburi.yaw_left(20),
     'right': lambda: duburi.yaw_right(20),
    }.get(duburi.where('gate'), lambda: duburi.move_forward(1))()

    # ClassRef handles + camera / freshness overrides work everywhere:
    duburi.detected(duburi.models.gate.gate)
    duburi.detected('flare', camera='downward', stale_after=2.0)

These are client-side reads of the same `/detections` stream the control
loop acts on. Each pumps the node so the answer reflects the CURRENT frame
(not a stale cache); the default camera is subscribed eagerly so the first
query never false-negates on DDS discovery. They run between goals (safe in
search loops and inside a vision `fallback`), never during one.

Model context (multi-model missions):

    duburi.models(
        gate   = 'gate_flare_medium_100ep',
        slalom = 'slalom_combined',
    )
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0)
    duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='height')

    # Strict class list (validates attribute access at handle time):
    duburi.models(gate=('gate_flare_medium_100ep', ['gate', 'flare']))
    duburi.vision.align(duburi.models.gate.gate, yaw=0)   # OK
    # duburi.vision.align(duburi.models.gate.typo, yaw=0)  # → AttributeError

When a ClassRef is passed as target, the DSL automatically calls
set_model() + set_classes() before sending the goal — no explicit
duburi.set_classes() or duburi.use() needed per verb.

Canonical competition task pattern (gate pass):

    duburi.models(gate='gate_flare_medium_100ep')
    duburi.set_depth(-1.2)
    while not duburi.detected(duburi.models.gate.gate):
        duburi.move_forward(0.6, gain=35)
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                        fallback=creep_forward)
    duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='area',
                       fallback=creep_forward)

Detector control (manual — ClassRef targets do this automatically):
    duburi.set_classes('gate')         # only gate detections
    duburi.set_classes('gate,flare')   # gate + flare
    duburi.set_classes('')             # all classes
    duburi.set_model('combined')       # switch model in registry
    duburi.use('combined', 'gate')     # switch model + class in one call

Tunable live (between runs, no rebuild):
    ros2 param set /duburi_manager vision.kp_yaw 80.0
    ros2 param set /duburi_manager vision.kp_lat 60.0
    ros2 param set /duburi_manager vision.lost_grace_s 1.0
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time as _time

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from .model_context import ClassRef, ModelRegistry
from .vision_dsl import _VisionDSL  # noqa: F401 -- re-exported; used by DuburiMission

from .client import MoveFailed          # arm() raises this on refusal


def _format_outcome(cmd: str, result) -> str:
    return (f'  {cmd:<22s} final={result.final_value:+.3f} '
            f'err={result.error_value:+.3f}  ({result.message})')


# --------------------------------------------------------------------------- #
#  Per-run artifact folder (scorecards live here; rosbags land alongside)      #
# --------------------------------------------------------------------------- #
# One place to grab everything after a pool session. Override the parent with
# DUBURI_RUN_DIR (pool_record.sh writes bags into the same tree). Default
# ~/duburi_runs so a scorecard never litters the CWD the operator launched from.

_RUN_DIR_ENV     = 'DUBURI_RUN_DIR'
_DEFAULT_RUN_DIR = '~/duburi_runs'


def _run_dir() -> str:
    """Return the per-run artifact folder, creating it if absent."""
    base = os.path.expanduser(os.environ.get(_RUN_DIR_ENV) or _DEFAULT_RUN_DIR)
    os.makedirs(base, exist_ok=True)
    return base


def _git_sha() -> str:
    """Best-effort short git SHA of the workspace (which code ran this run).

    Returns '' if git/the repo is unavailable -- traceability is a nice-to-have,
    never a reason to raise on pool day.
    """
    try:
        out = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            capture_output=True, text=True, timeout=2.0)
        return out.stdout.strip() if out.returncode == 0 else ''
    except Exception:
        return ''


# --------------------------------------------------------------------------- #
#  Detection-stream parsing helpers (defensive vs ROS distro field layout)    #
# --------------------------------------------------------------------------- #
# A parsed detection record: lowercased class + bbox geometry in pixels + conf.
# Tuple, not a class, so it copies trivially off the (reused) ROS message.
#   (class_lower, cx_px, cy_px, w_px, h_px, conf)

def _det_class_id(det) -> str:
    """Class id of a Detection2D's top hypothesis -- handles both layouts.

    Iron+ nests the label under ``results[0].hypothesis.class_id``; Humble's
    older message has a flat ``results[0].id``. Mirrors VisionState's
    ``_hypothesis_class_id`` so detected()/where() agree with the control path.
    """
    if not det.results:
        return ''
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'class_id'):
        return str(hyp.hypothesis.class_id)
    if hasattr(hyp, 'id'):
        return str(hyp.id)
    return ''


def _det_score(det) -> float:
    """Confidence of a Detection2D's top hypothesis (both layouts)."""
    if not det.results:
        return 0.0
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'score'):
        return float(hyp.hypothesis.score)
    if hasattr(hyp, 'score'):
        return float(hyp.score)
    return 0.0


def _det_center(bbox):
    """(cx, cy) of a BoundingBox2D -- Iron+ Pose2D w/ Point2D, or Humble flat."""
    centre = bbox.center
    if hasattr(centre, 'position'):       # Iron+: Pose2D with Point2D position
        return float(centre.position.x), float(centre.position.y)
    return float(centre.x), float(centre.y)   # Humble: flat Pose2D


def _parse_detections(msg) -> list:
    """Detection2DArray -> [DetRecord, ...], copied to plain Python.

    Copy eagerly: rclpy may reuse the underlying C++ buffer across callbacks,
    so holding the message objects past the callback would corrupt the cache.
    """
    out = []
    for det in msg.detections:
        cls = _det_class_id(det).strip().lower()
        if not cls:
            continue
        cx, cy = _det_center(det.bbox)
        out.append((cls, cx, cy,
                    float(det.bbox.size_x), float(det.bbox.size_y),
                    _det_score(det)))
    return out


def _eval_detected(records: list, needle: str) -> bool:
    """True iff any record matches ``needle`` (case-insensitive)."""
    n = str(needle).strip().lower()
    return any(rec[0] == n for rec in records)


def _eval_where(records: list, needle: str, width: float,
                band: float) -> tuple:
    """Bearing of the largest matching detection.

    Returns ``(label, offset)`` where ``label`` is 'left' | 'center' |
    'right' | 'unknown' and ``offset`` is the signed normalized horizontal
    offset in [-1, +1] (negative = left of centre, positive = right), or
    ``None`` when unknown. 'unknown' = class absent or image width not yet
    known. ``band`` is the centre dead-zone half-width (normalized).
    """
    n = str(needle).strip().lower()
    if not width or width <= 0.0:
        return ('unknown', None)
    best = None
    best_area = 0.0
    for rec in records:
        if rec[0] != n:
            continue
        area = rec[3] * rec[4]
        if area > best_area:
            best_area = area
            best = rec
    if best is None:
        return ('unknown', None)
    offset = (best[1] - width * 0.5) / (width * 0.5)   # [-1, +1]
    if offset < -band:
        return ('left', offset)
    if offset > band:
        return ('right', offset)
    return ('center', offset)


def _to_float(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    return value


def _param_value(value) -> ParameterValue:
    """Wrap a Python value in a typed rcl_interfaces ParameterValue.

    bool is checked before int (bool is a subclass of int in Python).
    """
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))


class DuburiMission:
    """Mission-author API. Wraps DuburiClient with human verbs + sticky context.

    Parameters
    ----------
    client : DuburiClient
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
        self._det_warm:  set[str] = set()         # cameras that have produced a frame
        # Detector parameter control (in-process, reliable -- replaces flaky
        # subprocess `ros2 param set`). node-name -> SetParameters client; the
        # set of nodes whose existence has been confirmed (so the loud preflight
        # probes each detector node at most once).
        self._param_clients: dict[str, object] = {}
        self._detector_ok:   set[str] = set()
        # Single-live-detector orchestration (Jetson VRAM: never run two detectors
        # at once). `_live_camera` is the camera whose detector is currently
        # resumed; a vision verb / use_camera on a DIFFERENT camera pauses the old
        # one, resumes the new, points the HUD at it, and settles -- see
        # _activate_camera. None = nothing resumed yet (launch starts both paused).
        self._live_camera: str | None = None
        self._active_cam_pub = None   # lazily-created latched String publisher (HUD follow)
        # Scoreboard: ordered list of (cmd, success, elapsed_s, message)
        self._scoreboard: list[dict] = []
        self._mission_start: float = _time.monotonic()
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
        # LEVEL BY OUTCOME. Every verb used to log at INFO, so a failure and a
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
    # The detector can run slow (medium model on the Orin Nano, sometimes 3-4 Hz
    # -> ~0.25-0.33 s/frame), so 0.40 s covers ~1.5 frames at 3-4 Hz. The pump
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

    def _wait_detector_warm(self, camera: str, timeout: float | None = None) -> bool:
        """Block until the detector for `camera` is PRODUCING /detections frames.

        Gates on the detector being ALIVE (publishing any frame -- the detector
        emits a frame every inference tick, empty or not), NOT on the target being
        visible. So a just-switched / cold detector is given time to warm up before
        a vision verb's acquire clock starts, while a genuine "target simply absent"
        search is NOT delayed: a warm detector is already publishing empty frames,
        so this returns on the first pump. Returns True once producing, False on
        timeout (detector never came alive -- caller proceeds + likely falls back).
        """
        budget = self._DETECTOR_WARMUP_S if timeout is None else float(timeout)
        self._subscribe_detections(camera)
        deadline = _time.monotonic() + max(budget, 0.0)
        while _time.monotonic() < deadline:
            self._pump_detections(camera)
            entry = self._det_cache.get(camera)
            if entry is not None and (_time.monotonic() - entry[0]) <= self._DETECTOR_FRESH_S:
                return True
        return False

    def _subscribe_detections(self, camera: str) -> None:
        """Subscribe a camera's /detections + /camera_info (idempotent)."""
        if camera in self._det_subs:
            return
        node = self.client.node
        # Match VisionState's QoS so we connect to the same publishers.
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        ns = f'/duburi/vision/{camera}'
        self._det_subs[camera] = node.create_subscription(
            Detection2DArray, f'{ns}/detections',
            lambda msg, cam=camera: self._on_detections(cam, msg), qos)
        self._info_subs[camera] = node.create_subscription(
            CameraInfo, f'{ns}/camera_info',
            lambda msg, cam=camera: self._on_info(cam, msg), qos)

    def _on_detections(self, camera: str, msg) -> None:
        # Copy to plain Python immediately (rclpy reuses the C++ buffer across
        # callbacks; holding the message would corrupt the cache).
        now     = _time.monotonic()
        records = _parse_detections(msg)
        self._det_cache[camera] = (now, records)        # latest frame: where() reads this
        seen = self._det_seen.setdefault(camera, {})    # per-class last-seen: detected() reads this
        for rec in records:
            seen[rec[0]] = now                          # rec[0] = lowercased class
        self._det_warm.add(camera)

    def _on_info(self, camera: str, msg) -> None:
        if msg.width and msg.height:
            self._img_size[camera] = (float(msg.width), float(msg.height))

    def _pump_detections(self, camera: str) -> None:
        """Spin the node until a /detections frame newer than now arrives.

        The detector publishes every frame, so a live pipeline lands a fresh
        frame within one frame period; a stalled or just-subscribed pipeline
        times out and the caller reads no (or stale) data -> correctly absent.
        """
        node = self.client.node
        start = _time.monotonic()
        budget = self._PUMP_WARM_S if camera in self._det_warm else self._PUMP_COLD_S
        deadline = start + budget
        while _time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=self._PUMP_SLICE_S)
            entry = self._det_cache.get(camera)
            if entry is not None and entry[0] >= start:
                return   # a frame stamped after we started -> cache is current

    def _records(self, camera: str, stale_after: float) -> list:
        """Latest-frame detection records for `camera`, or [] when stale/absent.

        Used by where()/where_offset() -- bearing must reflect the CURRENT frame.
        detected()/wait_for() use _present() instead (per-class recency window).
        """
        entry = self._det_cache.get(camera)
        if entry is None:
            return []
        stamp, records = entry
        if _time.monotonic() - stamp > stale_after:
            return []
        return records

    def _present(self, camera: str, needle: str, within: float) -> bool:
        """True iff `needle` was seen on `camera` within the last `within` s.

        Flicker-tolerant: a class that drops from individual raw frames (low
        FPS, motion blur) still counts as present until `within` elapses since
        its last sighting -- so a `while not detected()` search reacquires the
        instant the target reappears instead of chasing per-frame dropouts.
        """
        n    = str(needle).strip().lower()
        last = self._det_seen.get(camera, {}).get(n)
        return last is not None and (_time.monotonic() - last) <= within

    def _seen_since(self, camera: str, needle: str, t: float) -> bool:
        """True iff `needle` was seen on `camera` in a frame that arrived at/after `t`.

        PURE cache read (no pump) -- the caller's own spin refreshes `_det_seen`.
        Used by the vision-verb fallback interrupt: keying on "seen AFTER the search
        began" makes a stale pre-loss sighting unable to trip it, so the interrupt
        fires only on a genuine reacquisition mid-search.
        """
        last = self._det_seen.get(camera, {}).get(str(needle).strip().lower())
        return last is not None and last >= t

    def detected(self, target_class, *,
                 camera: str | None = None,
                 stale_after: float = 1.0) -> bool:
        """Was `target_class` seen within the last `stale_after` s on `camera`?

        A recency check: it pumps the detection stream so the cache is current,
        then returns True iff the class was seen within the `stale_after` window
        (per-class last-seen, NOT just the single latest frame -- so it tolerates
        the per-frame flicker raw `/detections` shows at low FPS). Use it to
        branch (``if``) or to drive a moving search (``while``):

            # branch once on what is currently visible
            if duburi.detected('gate'):
                duburi.vision.align('gate', yaw=0, lat=0)

            # search WHILE MOVING -- needs a loop (an `if` runs once!)
            while not duburi.detected('red_pipe'):
                duburi.move_left(2)
            duburi.move_forward(3)

        To wait for a target while holding station, prefer ``wait_for`` (one
        call, no busy-loop). Works inside a vision ``fallback`` too -- the
        search re-enters the verb the moment the target reappears.

        Parameters
        ----------
        target_class : str | ClassRef
            Class name (``'gate'``) or a ``duburi.models.<m>.<c>`` handle.
        camera : str | None
            Camera to query. Defaults to ``duburi.camera``.
        stale_after : float
            Recency window: the class counts as present until this many seconds
            after its last sighting (bridges per-frame flicker). A class not seen
            for longer than this counts as absent. Lower it where you need the
            False edge to be prompt (e.g. gating an irreversible fire/drop).
        """
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        return self._present(cam, target_class, stale_after)

    def wait_for(self, target_class, *,
                 timeout: float = 10.0,
                 camera: str | None = None,
                 stale_after: float = 1.0) -> bool:
        """Block until `target_class` is seen on `camera`, or `timeout` elapses.

        The loop-free way to (re)acquire a target while stationary -- returns
        True the moment it appears, False if `timeout` passes first. Each poll
        pumps the stream, so this keeps the cache fresh without a busy-loop::

            if duburi.wait_for('gate', timeout=8):
                duburi.vision.align('gate', yaw=0, lat=0)
            else:
                duburi.recover()           # never showed up

        For a search that should KEEP MOVING while looking, use a
        ``while not duburi.detected(...): <small move>`` loop instead.
        """
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        deadline = _time.monotonic() + max(float(timeout), 0.0)
        while _time.monotonic() < deadline:
            self._pump_detections(cam)
            if self._present(cam, target_class, stale_after):
                return True
        return False

    def where(self, target_class, *,
              camera: str | None = None,
              stale_after: float = 1.0,
              band: float = 0.15) -> str:
        """Bearing of `target_class`: 'left' | 'center' | 'right' | 'unknown'.

        Picks the largest matching detection and reports which side of frame
        centre it sits on (``band`` = centre dead-zone half-width, normalized).
        ``'unknown'`` = not visible (or camera_info not seen yet). Image-frame
        semantics: ``'left'`` means the target is on the left, so yaw left to
        face it (same polarity as the vision-yaw axis)::

            {'left': lambda: duburi.yaw_left(20),
             'right': lambda: duburi.yaw_right(20),
            }.get(duburi.where('gate'), lambda: duburi.move_forward(1))()

        Use ``where_offset`` for the raw signed offset (fine steering).
        """
        label, _ = self._where_eval(target_class, camera, stale_after, band)
        return label

    def where_offset(self, target_class, *,
                     camera: str | None = None,
                     stale_after: float = 1.0):
        """Signed normalized horizontal offset [-1,+1] of `target_class`.

        Negative = left of centre, positive = right; ``None`` when not visible
        or image width unknown. The continuous companion to ``where``.
        """
        _, offset = self._where_eval(target_class, camera, stale_after, 0.0)
        return offset

    def _where_eval(self, target_class, camera, stale_after, band) -> tuple:
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        width = self._img_size.get(cam, (0.0, 0.0))[0]
        return _eval_where(self._records(cam, stale_after),
                           target_class, width, band)

    # ================================================================== #
    #  Power / mode                                                        #
    # ================================================================== #

    def arm(self, *, timeout: float = 15.0, required: bool = True):
        """Arm the vehicle. RAISES on failure unless `required=False`.

        ⛔ THIS IS THE ONE VERB WHERE CONTINUING AFTER FAILURE IS ALWAYS WRONG.
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
        return result

    def disarm(self, *, timeout: float = 20.0):
        return self._send('disarm', timeout=timeout)

    def set_mode(self, name: str, *, timeout: float = 8.0):
        return self._send('set_mode', target_name=name, timeout=timeout)

    # ================================================================== #
    #  Stop / pause                                                        #
    # ================================================================== #

    def stop(self):
        return self._send('stop')

    def surface(self):
        """Emergency surface: ascend to 0 m depth. Safe to call during a running mission."""
        return self._send('surface')

    def head(self) -> float:
        """Return live heading (degrees) at call time.

            h = duburi.head()
            duburi.lock_heading(target=h)
        """
        result = self._send('head')
        return result.final_value if result is not None else 0.0

    def pause(self, seconds: float):
        return self._send('pause', duration=float(seconds))

    def fire(self, channel: int):
        """Activate payload BOARD channel `channel` (1..16).

        On srot this is the board's own PCA9685 channel -- the same n as
        `SERVO{n}_ROLE` -- with no host-side map. The board's role config decides:
        a SWITCH channel fires, a PWM channel is REFUSED (it is the on-board arm).
        `ros2 run duburi_manager connect` lists which channels are fireable.

        `result.final_value` carries the outcome code (`FIRE_*` in
        `duburi_control.fc.base`), so a mission can tell "refused, that channel is
        the arm" from "the link is down" instead of just seeing success=False.
        """
        return self._send('fire', fire_channel=float(channel))

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
        Internally calls yaw_snap or yaw_glide depending on smooth_yaw.

        Examples::

            duburi.turn(90)          # face east, from any current heading
            duburi.turn(0)           # face north (shortest path)
            duburi.turn(270)         # face west
        """
        return self._send('turn',
                          target=float(degrees),
                          timeout=timeout, settle=settle)

    def arc(self, target_yaw: float, seconds: float = 4.0, *,
            gain: float = 50.0, settle: float = 0.0):
        """Curve onto an ABSOLUTE heading while driving forward.

        Drives forward at ``gain`` %% for ``seconds`` while a PID turns the hull
        to ``target_yaw`` (absolute degrees) and holds it -- the trajectory
        curves onto the heading then straightens (a sweeping "turn-and-go").
        Turn direction is auto-computed; ``seconds`` sets how far it travels.
        E.g. ``duburi.arc(90, 4)`` sweeps onto heading 90° over a 4 s forward run.
        """
        return self._send('arc',
                          duration=float(seconds), gain=gain,
                          target_yaw=float(target_yaw), settle=settle)

    def style_roll(self, *, gain: float = 60.0, timeout: float = 20.0,
                  flips: int = 1, headroom: float = 1.0):
        """N×360° roll in ACRO mode (timeout is per flip). Pre-dives headroom m per flip to avoid surfacing."""
        return self._send('style_roll', gain=gain, timeout=timeout,
                          flips=flips, headroom=headroom)

    def style_yaw(self, *, flips: int = 1, deg_per_step: float = 90.0,
                  settle: float = 1.0):
        """N×360° yaw spin in ALT_HOLD. flips full rotations via deg_per_step yaw snaps."""
        return self._send('style_yaw', flips=flips, deg_per_step=deg_per_step,
                          settle=settle)

    def lock_heading(self, degrees: float = 0.0, *, timeout: float = 300.0):
        return self._send('lock_heading',
                          target=float(degrees), timeout=timeout)

    def release_heading(self):
        return self._send('unlock_heading')

    # ================================================================== #
    #  DVL                                                                 #
    # ================================================================== #

    # Settle after a live-detector switch so the resumed detector's first frames
    # land (and the paused one's queue drains) before the loop steers on them --
    # otherwise the first align tick acquires on a cold/empty detector. Bringing a
    # camera + its detector + the HUD onto a fresh target takes real time (stream
    # re-latch + first inference tick + tracker warm-up), so give generous headroom:
    # a switch happens once per task, not per loop, so an extra second is free
    # insurance against acquiring on a stale/empty frame. Override per-mission via
    # ``duburi.cam_switch_settle_s = <seconds>`` before the switch if a run needs
    # it snappier or slower.
    _CAM_SWITCH_SETTLE_S = 1.5

    # Known dual-camera detector cameras. On every switch we pause EVERY one of
    # these except the target (not just the previously-live one), so exclusivity
    # holds from the FIRST use_camera even if the node was launched paused:=false
    # (both detectors inferring from t=0 -> the concurrent-inference OOM). Pausing
    # an absent one is a quiet no-op, so single-camera runs are unaffected.
    _KNOWN_CAMERAS = ('forward', 'downward')

    def use_camera(self, name: str) -> None:
        """Switch the sticky camera for all subsequent vision verbs AND make it the
        single live detector (pause the other, resume this one, point the HUD at it).

        Logs the switch so pool-side operators see the transition. Idempotent on the
        detector switch (no-op when already live). Best-effort: a missing detector
        node warns rather than crashing, so pure-control / single-camera runs are
        unaffected.

        Example::

            duburi.use_camera('downward')            # downward detector live, HUD flips
            duburi.vision.align('fire', lat=0, depth=0, err=30)
            duburi.use_camera('forward')             # back to forward
        """
        self.log.info(f'[MISSION] camera → {name!r}')
        self.camera = name
        # Eager-subscribe so discovery warms before the first detected()/where()
        # on the new camera (otherwise that first query false-negates).
        self._subscribe_detections(name)
        self._activate_camera(name)

    def _activate_camera(self, name: str) -> None:
        """Make ``name`` the ONLY live detector + point the HUD at it. Idempotent.

        Called by ``use_camera`` and automatically by a vision verb whose camera
        differs from the live one, so ``camera='downward'`` in a verb "just works"
        (never two detectors at once on the Jetson). Best-effort on the detector
        params: a missing node warns, doesn't raise -- a pure-control sim has no
        detectors and must not crash here."""
        if name == self._live_camera:
            return
        self._publish_active_camera(name)   # HUD follows (latched topic; always safe)
        # Pause EVERY known detector except the target -- not just the previously
        # live one. This makes the invariant "exactly one detector infers" hold
        # from the first switch regardless of the launch `paused` state, so a
        # stray `paused:=false` (both inferring, the OOM config) is corrected the
        # moment the mission calls use_camera. Gated on a FAST graph existence
        # check (get_node_names) so an absent counterpart is skipped instantly --
        # otherwise pause_detector -> _ensure_detector would eat its 5 s
        # wait_for_service on every switch of a single-camera run.
        for other in self._KNOWN_CAMERAS:
            if other == name:
                continue
            if not self._detector_present(other):
                continue                     # not up -> nothing to pause (no 5 s wait)
            try:
                self.pause_detector(other)   # exclusivity: only one detector runs
            except Exception as exc:         # noqa: BLE001 -- best-effort
                self.log.debug(f'[CAM  ] pause {other!r} detector skipped: {exc}')
        switched = False
        try:
            self.resume_detector(name)
            switched = True
        except Exception as exc:            # noqa: BLE001 -- best-effort (no detector = sim)
            self.log.warning(f'[CAM  ] resume {name!r} detector skipped: {exc}')
        if switched:
            # Per-mission override wins; else the class default headroom.
            settle = getattr(self, 'cam_switch_settle_s', None)
            _time.sleep(self._CAM_SWITCH_SETTLE_S if settle is None else float(settle))
        # let first live frames land before the next verb steers on them
        self._live_camera = name

    def _publish_active_camera(self, name: str) -> None:
        """Publish the active camera on a LATCHED topic so the HUD auto-follows the
        mission (manual f/d keys remain an override). Lazily create the publisher."""
        try:
            if self._active_cam_pub is None:
                qos = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)   # latched
                self._active_cam_pub = self.client.node.create_publisher(
                    String, '/duburi/vision/active_camera', qos)
            self._active_cam_pub.publish(String(data=str(name)))
        except Exception as exc:            # noqa: BLE001 -- HUD follow is best-effort
            self.log.warning(f'[CAM  ] active_camera publish skipped: {exc}')

    def calc_distance(self, phase: str):
        """Downward optical-flow distance bracket (DVL-free). OWNS the camera switch.

        ``calc_distance('start')`` enters distance mode: pause BOTH YOLO detectors
        (LK flow is CPU -- no detector runs, freeing the GPU + honouring "not
        simultaneous with YOLO"), point the HUD at the downward camera, then latch
        the axis + reset the accumulator (facade verb). Run your axis-locked move
        between start and stop. ``calc_distance('stop')`` freezes + returns the
        accumulated METRES, then RESTORES the camera/detector that was live before
        (so forward vision missions resume as usual). move_* verbs are never touched.

            duburi.calc_distance('start')
            duburi.move_forward(4.0, gain=60)      # blind timed move, measured
            metres = duburi.calc_distance('stop')

        Returns the accumulated distance (float, metres) on 'stop'; the raw
        Move.Result on 'start'.
        """
        p = str(phase or '').strip().lower()
        if p == 'start':
            # Remember what to restore, then enter distance mode (both detectors
            # paused = no YOLO; HUD -> downward). Best-effort: absent detectors are
            # quiet no-ops (single-camera / control-only runs unaffected).
            self._pre_distance_camera = getattr(self, '_live_camera', None) or self.camera
            for cam in self._KNOWN_CAMERAS:
                try:
                    self.pause_detector(cam)
                except Exception as exc:          # noqa: BLE001
                    self.log.debug(f'[DIST ] pause {cam!r} skipped: {exc}')
            self._publish_active_camera('downward')
            self.camera = 'downward'
            self.log.info('[MISSION] distance mode ON (downward, detectors paused)')
            return self._send('calc_distance', phase='start')

        # stop: freeze + read metres, then restore the prior camera/detector.
        result = self._send('calc_distance', phase='stop')
        metres = float(getattr(result, 'final_value', 0.0))
        prior  = getattr(self, '_pre_distance_camera', None) or 'forward'
        try:
            self.use_camera(prior)                # resumes the prior detector + HUD
        except Exception as exc:                  # noqa: BLE001
            self.log.warning(f'[DIST ] restore camera {prior!r} skipped: {exc}')
        self.log.info(f'[MISSION] distance mode OFF -> {metres:+.3f}m  (camera -> {prior!r})')
        return metres

    def dvl_connect(self):
        """Connect Nortek Nucleus 1000 DVL over TCP."""
        return self._send('dvl_connect')

    def move_forward_dist(self, metres: float, *, gain: float = 60.0,
                          tolerance: float = 0.1, settle: float = 0.0):
        """Drive forward `metres` metres using DVL closed-loop feedback."""
        return self._send('move_forward_dist',
                          distance_m=float(metres),
                          gain=gain, dvl_tolerance=tolerance, settle=settle)

    def move_back_dist(self, metres: float, *, gain: float = 60.0,
                       tolerance: float = 0.1, settle: float = 0.0):
        """Drive backward `metres` metres using DVL closed-loop feedback."""
        return self._send('move_back_dist',
                          distance_m=float(metres),
                          gain=gain, dvl_tolerance=tolerance, settle=settle)

    def move_lateral_dist(self, metres: float, *, gain: float = 36.0,
                          tolerance: float = 0.1, settle: float = 0.0):
        """Strafe `metres` metres (positive=right) using DVL feedback."""
        return self._send('move_lateral_dist',
                          distance_m=float(metres),
                          gain=gain, dvl_tolerance=tolerance, settle=settle)

    # ================================================================== #
    #  Vision detector control                                             #
    # ================================================================== #

    def _detector_node(self, camera: str | None = None,
                       node: str | None = None) -> str:
        """Resolve the detector node name for a camera.

        Single naming rule for the whole stack: ``/duburi_detector_<camera>``
        (e.g. ``/duburi_detector_forward``, ``/duburi_detector_downward``),
        matching the names the vision launch files give the detector nodes.
        ``camera`` defaults to the mission's sticky camera. An explicit
        ``node`` always wins (escape hatch for non-standard setups).
        """
        if node:
            return node
        return f'/duburi_detector_{camera or self.camera}'

    def _detector_present(self, camera: str) -> bool:
        """Fast graph check: is the ``camera`` detector node currently up?

        Reads the discovery graph (``get_node_names``) -- instant, no service
        wait -- so the exclusivity loop can SKIP an absent counterpart instead of
        paying ``_ensure_detector``'s 5 s ``wait_for_service`` on every switch of a
        single-camera run. Best-effort: any error -> treat as absent (skip).
        """
        node = self._detector_node(camera)
        want = node.lstrip('/')
        try:
            names = self.client.node.get_node_names()
        except Exception:   # noqa: BLE001 -- graph read is best-effort
            return False
        return want in names or node in names

    def _ensure_detector(self, node: str, *, timeout: float = 5.0) -> None:
        """Abort the mission LOUDLY if detector ``node`` is not on the graph.

        Probed at most once per node (cached in ``_detector_ok``). This is the
        single guard that turns "someone forgot to start the vision stack" from
        a silent warning + a mission that idles on err=+inf (the pool-test
        failure) into an immediate, actionable abort. Called from every detector
        param op AND from the vision verbs (align/move) so a mission that
        skips set_model still can't run blind.
        """
        if node in self._detector_ok:
            return
        ros_node = self.client.node
        cli = ros_node.create_client(GetParameters, f'{node}/get_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                cam = node.rsplit('duburi_detector_', 1)[-1]
                raise RuntimeError(
                    f"Detector node {node} NOT FOUND after {timeout:.0f}s -- the "
                    f"vision stack is not running, but this mission uses vision. "
                    f"Start it, e.g.:\n"
                    f"  ros2 launch duburi_vision vision.launch.py camera:={cam} "
                    f"model:=<stem> classes:=<csv>\n"
                    f"(aborting loudly so a missing detector can't cost a run)")
        finally:
            ros_node.destroy_client(cli)
        self._detector_ok.add(node)

    def _set_detector_param(self, node: str, name: str, value) -> None:
        """Set one detector parameter in-process via SetParameters (reliable).

        Replaces the old ``subprocess('ros2 param set')`` which spun up a fresh
        CLI node that had to re-discover the detector every call (the flaky,
        silent "Node not found" source). Raises on absence (via _ensure_detector)
        or rejection so failures are loud, not swallowed warnings.
        """
        self._ensure_detector(node)
        ros_node = self.client.node
        cli = self._param_clients.get(node)
        if cli is None:
            cli = ros_node.create_client(SetParameters, f'{node}/set_parameters')
            self._param_clients[node] = cli
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{node}/set_parameters unavailable')
        req = SetParameters.Request(
            parameters=[Parameter(name=name, value=_param_value(value))])
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(ros_node, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            raise RuntimeError(f'set {node}.{name} timed out (no response)')
        res = resp.results[0]
        if not res.successful:
            raise RuntimeError(f'set {node}.{name}={value!r} rejected: {res.reason}')

    def set_model(self, name, *,
                  camera: str | None = None, node: str | None = None) -> None:
        """Switch the active detector model, or run SEVERAL at once.

        ``name`` is the model **stem** (e.g. ``'gate_rescue_repair'``) or a
        registry key. Works on BOTH launch styles: a single-model launch
        (``model:=<stem>``) accepts ``set_model('<that stem>')`` as a no-op and
        rejects any *other* name; a registry launch (``models:=``) accepts the
        key OR the stem. Targets ``/duburi_detector_<camera>`` (camera defaults to
        the mission's). Raises if the node is absent or the switch is rejected.

        **Pass a list or a comma-separated string to run more than one model on
        every frame** -- a detector and a segmentation model together, say::

            duburi.use('gate_rescue_repair')                  # detection only
            duburi.use('gate_seg')                            # segmentation only
            duburi.use(['gate_rescue_repair', 'gate_seg'])    # both, merged

        The FIRST name stays "the" model: it owns the class filter, the
        published `vision_info`, and the alignment line. The rest only
        contribute detections, which arrive merged in the same
        `Detection2DArray` and outlined in the same `TargetContours`.

        ⚠ Two models is not free and is not a default. The accelerator runs one
        graph at a time and each handover costs ~4 ms, so the frame rate is the
        sum plus the swaps -- measured 37.5 Hz for a pair against 95 Hz for one
        model. Ask for both when a task needs both, not for the whole mission.

        Why a widened argument rather than a new verb: the node takes ONE
        `active_model` parameter, and one `SetParameters` call changes the whole
        selection atomically. A separate "extra models" verb would need two
        calls, and between them a frame would be detected with a pair nobody
        asked for.
        """
        if isinstance(name, (list, tuple)):
            name = ','.join(str(n).strip() for n in name if str(n).strip())
        node = self._detector_node(camera, node)
        try:
            self._set_detector_param(node, 'active_model', str(name))
        except RuntimeError as exc:
            if 'registry' in str(exc).lower():
                raise RuntimeError(
                    f'set_model({name!r}): {exc} -- launch with models:="..." '
                    f'to enable hot model switching') from None
            raise
        self.log.info(f'[DSL  ] {node} active_model → {name!r}')

    def use(self, model: str, classes: str | list | None = None, *,
            camera: str | None = None, node: str | None = None) -> None:
        """Switch active detector model and optionally its class filter.

        Parameters
        ----------
        model : str
            Registry key (e.g. ``'gate'``, ``'combined'``).
        classes : str | list | None
            Class filter to apply. ``None`` leaves current filter untouched.
            ``''`` enables all classes.

        Example::

            duburi.use('gate', 'gate')    # switch model and filter together
            duburi.use('combined', '')    # combined model, all classes visible
        """
        self.set_model(model, camera=camera, node=node)
        if classes is not None:
            self.set_classes(classes, camera=camera, node=node)

    def set_classes(self, classes: str | list, *,
                    camera: str | None = None, node: str | None = None) -> None:
        """Switch the detector's class filter without restarting the node.

        Example::

            duburi.set_classes('gate')
            duburi.set_classes(['gate', 'flare'])
            duburi.set_classes('')   # all classes
        """
        node = self._detector_node(camera, node)
        if isinstance(classes, list):
            classes_str = ','.join(str(c).strip() for c in classes)
        else:
            classes_str = str(classes).strip()
        self._set_detector_param(node, 'classes', classes_str)
        self.log.info(f"[DSL  ] {node} classes → {classes_str!r}")

    def set_conf(self, conf: float, *, model: str | None = None,
                 camera: str | None = None, node: str | None = None) -> None:
        """Set the YOLO confidence threshold live (next inference tick).

        ``model=None`` (default) sets the threshold for EVERY model on the
        detector (survives model switches). ``model='<registry name>'`` sets it
        for that ONE model only -- e.g. run the torpedo model tight and the gate
        model loose on the same forward detector::

            duburi.set_conf(0.35)                              # all models
            duburi.set_conf(0.55, model='torpedo_blood_hole')  # torpedo only

        Per-model needs a ``models`` registry (multi-model launch); the override
        persists across ``set_model`` switches (it lives on the model itself).
        """
        node = self._detector_node(camera, node)
        if model is None:
            self._set_detector_param(node, 'conf', float(conf))
            self.log.info(f"[DSL  ] {node} conf → {float(conf):.3f} (all models)")
        else:
            self._set_detector_param(node, 'model_conf', f'{model}={float(conf)}')
            self.log.info(f"[DSL  ] {node} conf[{model!r}] → {float(conf):.3f}")

    def lock_class(self, target: str = '', *, camera: str | None = None,
                   timeout: float = 2.0) -> bool:
        """Aim the continuity ladder MID-MISSION. Returns True if it took.

        The ladder (`lock_node`: follower + XFeat anchor) normally aims itself
        at whatever `set_classes` last told the detector, via the latched
        `classes_filter`. Use this only to PIN it somewhere else -- e.g. hold
        the gate while the detector is already hunting the next prop::

            duburi.lock_class('gate')        # pin
            duburi.lock_class('')            # release; follow the mission again

        Unlike the detector helpers this does NOT abort when the node is
        missing. The ladder is a fallback rung: a mission must run without it,
        and a hard failure here would turn an enhancement into a dependency.
        A clear warning is logged instead, and False returned.
        """
        node = f'/duburi_lock_{camera or self.camera}'
        ros_node = self.client.node
        cli = ros_node.create_client(SetParameters, f'{node}/set_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                self.log.warning(
                    f'[DSL  ] {node} not on the graph -- the ladder is not '
                    f'running, so it cannot be aimed at {target!r}. The '
                    f'mission continues on live detections. Start it with '
                    f'`lock:=true` on the vision launch.')
                return False
            from rcl_interfaces.msg import Parameter as _P, ParameterValue as _PV
            from rcl_interfaces.msg import ParameterType as _PT
            req = SetParameters.Request(parameters=[_P(
                name='target_class',
                value=_PV(type=_PT.PARAMETER_STRING,
                          string_value=str(target).strip()))])
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(ros_node, fut, timeout_sec=timeout)
            res = fut.result()
            ok = bool(res and res.results and res.results[0].successful)
            if ok:
                self.log.info(
                    f"[DSL  ] {node} target_class -> {target!r}"
                    + ('' if target else '  (released; follows the mission)'))
            else:
                self.log.warning(f'[DSL  ] {node} refused target_class {target!r}')
            return ok
        finally:
            ros_node.destroy_client(cli)

    # Manager node that owns the vision.* tunables (declares/re-snapshots them).
    _MANAGER_NODE = '/duburi_manager'

    def set_vision_param(self, name: str, value: float) -> None:
        """Set a manager ``vision.*`` tunable live (applies to the NEXT vision goal).

        For per-MISSION control of a deck tunable without touching every verb call
        -- e.g. a downward bin run fixes its depth floor/ceiling ONCE at the top of
        the mission instead of passing ``max_depth_m=`` / ``depth_ceiling=`` on each
        ``vision.align``::

            duburi.set_vision_param('max_depth_m', -1.6)   # enables+bounds the descent
            duburi.set_vision_param('depth_ceiling', -0.4) # surface guard

        ``name`` is the stem with or without the ``vision.`` prefix. The manager
        re-snapshots ``vision.*`` per goal, so the next align picks it up. Raises if
        the manager is absent or the set is rejected (loud, not silent).
        """
        param = name if name.startswith('vision.') else f'vision.{name}'
        self._set_manager_param(param, float(value))
        self.log.info(f'[DSL  ] {self._MANAGER_NODE} {param} → {float(value):.3f}')

    def _set_manager_param(self, name: str, value) -> None:
        """Set one parameter on the manager node via SetParameters (in-process)."""
        node_name = self._MANAGER_NODE
        ros_node = self.client.node
        cli = self._param_clients.get(node_name)
        if cli is None:
            cli = ros_node.create_client(SetParameters, f'{node_name}/set_parameters')
            self._param_clients[node_name] = cli
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{node_name}/set_parameters unavailable')
        req = SetParameters.Request(
            parameters=[Parameter(name=name, value=_param_value(value))])
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(ros_node, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            raise RuntimeError(f'set {node_name}.{name} timed out (no response)')
        res = resp.results[0]
        if not res.successful:
            raise RuntimeError(f'set {node_name}.{name}={value!r} rejected: {res.reason}')

    def pause_detector(self, camera: str | None = None, *,
                       node: str | None = None) -> None:
        """Pause inference on a detector node (frame still consumed from queue)."""
        cam  = camera or self.camera
        node = self._detector_node(camera, node)
        self._set_detector_param(node, 'paused', True)
        # Keep the exclusivity tracker honest: if we just paused the live detector,
        # nothing is live now, so the next _activate_camera re-resumes it.
        if self._live_camera == cam:
            self._live_camera = None
        self.log.info(f"[DSL  ] {node} paused")

    def resume_detector(self, camera: str | None = None, *,
                        node: str | None = None) -> None:
        """Resume inference on a detector node."""
        cam  = camera or self.camera
        node = self._detector_node(camera, node)
        self._set_detector_param(node, 'paused', False)
        # This camera is now the live detector. Recording it means a later switch
        # to a DIFFERENT camera pauses THIS one (never two detectors on the Jetson),
        # even when a mission resumed it manually rather than via _activate_camera.
        self._live_camera = cam
        self.log.info(f"[DSL  ] {node} resumed")

    # ================================================================== #
    #  Mission countdown                                                   #
    # ================================================================== #

    def countdown(self, seconds: int = 10, *,
                  message: str = "Wire removed  --  Duburi is now autonomous. Good luck."):
        """Print a tether-removal countdown and return."""
        width = 66
        border_h = '━' * width
        tl, tr, bl, br = '┏', '┓', '┗', '┛'
        vb = '┃'

        def _box(lines):
            print(f'{tl}{border_h}{tr}')
            for line in lines:
                pad = width - len(line)
                lp  = pad // 2
                rp  = pad - lp
                print(f'{vb}{" " * lp}{line}{" " * rp}{vb}')
            print(f'{bl}{border_h}{br}')

        print()
        _box([
            '',
            'TETHER REMOVAL WINDOW',
            '',
            'Disconnect the tether now.',
            f'Mission starts in {seconds} seconds.',
            '',
        ])
        print()

        for remaining in range(seconds, 0, -1):
            bar_total = 40
            filled    = int(bar_total * (seconds - remaining) / seconds)
            bar       = '█' * filled + '░' * (bar_total - filled)
            sys.stdout.write(f'\r  T-{remaining:3d}s  [{bar}]  ')
            sys.stdout.flush()
            _time.sleep(1)

        sys.stdout.write('\r' + ' ' * 60 + '\r')
        sys.stdout.flush()
        print()
        _box(['', message, ''])
        print()

    # ================================================================== #
    #  Mission scoreboard                                                  #
    # ================================================================== #

    def log_scoreboard(self, *, json_path: str | None = None,
                       mission: str | None = None) -> None:
        """Print a structured per-verb mission summary and optionally write JSON.

        Called automatically by `mission.py` on exit (normal or exception).
        Can also be called manually at any point during a mission.

        Parameters
        ----------
        json_path : str | None
            If given, write the scoreboard JSON to this path in addition to
            printing to stdout.  Pass ``'auto'`` to write a timestamped
            ``<mission>_<ts>.json`` into the dedicated run folder (``_run_dir``,
            default ``~/duburi_runs``, override with ``DUBURI_RUN_DIR``).
        mission : str | None
            Mission name, recorded in the JSON and used in the ``'auto'``
            filename so a pool session's scorecards are traceable per run.

        Example output::

            ╔══════════════════════════════════════════════════════════════════╗
            ║  MISSION SCOREBOARD                         total: 47.3 s       ║
            ╠══════════╦══════════╦═══════╦═══════════════════════════════════╣
            ║  #  verb ║ success  ║  time ║  message                          ║
            ╠══════════╬══════════╬═══════╬═══════════════════════════════════╣
            ║  1  arm  ║    ✓     ║  2.1s ║  armed                            ║
            ║  ...                                                             ║
            ╚══════════════════════════════════════════════════════════════════╝
        """
        total_s = round(_time.monotonic() - self._mission_start, 1)
        width   = 68
        hb      = '═' * width

        def _row(n, entry):
            tick = '✓' if entry['success'] else '✗'
            cmd  = entry['cmd'][:18]
            t    = f"{entry['elapsed']:.1f}s"
            msg  = entry['msg'][:30]
            return f"  {n:>2d}  {cmd:<18s}  {tick}   {t:>5s}   {msg}"

        print(f'\n╔{hb}╗')
        print(f'║  MISSION SCOREBOARD{" " * (width - 20 - len(str(total_s)) - 12)}total: {total_s} s  ║')
        print(f'╠{hb}╣')
        print(f'║  {"#":>2s}  {"verb":<18s}  {"ok"}   {"time":>5s}   {"message":<30s}  ║')
        print(f'╠{hb}╣')
        for i, entry in enumerate(self._scoreboard, 1):
            row = _row(i, entry)
            pad = width - len(row)
            print(f'║{row}{" " * pad}║')
        print(f'╚{hb}╝\n')

        if json_path:
            payload = {
                'mission':       mission or '',
                'timestamp':     _time.strftime('%Y-%m-%dT%H:%M:%S'),
                'git_sha':       _git_sha(),
                'total_s':       total_s,
                'success_count': sum(1 for e in self._scoreboard if e['success']),
                'fail_count':    sum(1 for e in self._scoreboard if not e['success']),
                'phases':        self._scoreboard,
            }
            # Whole write is best-effort: log_scoreboard runs in mission.py's
            # finally, so a run-dir mkdir / write failure must never mask the
            # mission outcome. Resolve the auto path INSIDE the guard too
            # (_run_dir mkdir can raise on a read-only / full disk).
            try:
                if json_path == 'auto':
                    ts   = _time.strftime('%Y%m%d_%H%M%S')
                    name = (mission or 'mission').replace('/', '_')
                    json_path = os.path.join(_run_dir(), f'{name}_{ts}.json')
                with open(json_path, 'w') as fh:
                    json.dump(payload, fh, indent=2)
                self.log.info(f'[DSL  ] scoreboard → {json_path}')
            except OSError as exc:
                self.log.warning(f'[DSL  ] scoreboard write failed ({exc})')

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

