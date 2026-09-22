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
    mongla.arc(target_yaw, seconds=4, gain=50)   # curve onto an absolute heading
    mongla.lock_heading(degrees)  / mongla.release_heading()
    mongla.pause(seconds) / mongla.stop()

Depth is held automatically by ArduSub's onboard ALT_HOLD. `set_depth`
engages the mode and drives to the target; the autopilot holds it afterwards.

Closed-loop vision lives under `mongla.vision` as exactly two verbs:

    mongla.vision.align(target, *, lat=None, yaw=None, depth=None,
                        err=40, duration=20, gain=30, fallback=None, camera=None)
        Centre the target on the selected axes. Each of lat/yaw/depth is
        None (axis off) or a signed pixel offset from centre (0 = centre,
        +=right/below, -=left/above). lat+yaw are horizontal (Ch6 strafe /
        Ch4 rotate); depth is vertical. At least one axis is required.

    mongla.vision.move(target, *, fwd=95, mode='area', maintain=None,
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
        mongla.recover()

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
"""

from __future__ import annotations

import contextlib
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
from .vision_dsl import _VisionDSL  # noqa: F401 -- re-exported; used by MonglaMission

from .client import MoveFailed, TaskAbandoned, MissionRefused  # arm() raises MoveFailed; task() raises TaskAbandoned; require() raises MissionRefused


def _format_outcome(cmd: str, result) -> str:
    return (f'  {cmd:<22s} final={result.final_value:+.3f} '
            f'err={result.error_value:+.3f}  ({result.message})')


# --------------------------------------------------------------------------- #
#  Per-run artifact folder (scorecards live here; rosbags land alongside)      #
# --------------------------------------------------------------------------- #
# One place to grab everything after a pool session. Override the parent with
# MONGLA_RUN_DIR (pool_record.sh writes bags into the same tree). Default
# ~/mongla_runs so a scorecard never litters the CWD the operator launched from.

_RUN_DIR_ENV     = 'MONGLA_RUN_DIR'
_DEFAULT_RUN_DIR = '~/mongla_runs'


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


class _DetView:
    """A cached record `(cls, cx, cy, w, h, score)` seen as a Detection.

    `mongla_vision.identity` works on anything with `class_name`, `score` and
    `xyxy`, so the adapter lives here rather than changing either side: the
    cache stays a plain tuple (copied eagerly off the C++ buffer) and the
    geometry module stays free of the DSL's storage choices.
    """

    __slots__ = ('class_name', 'score', 'xyxy', 'mask')

    def __init__(self, rec):
        cls, cx, cy, w, h, score = rec
        self.class_name = cls
        self.score = float(score)
        self.xyxy = (cx - w * 0.5, cy - h * 0.5, cx + w * 0.5, cy + h * 0.5)
        self.mask = None


def _outline_view(outline):
    """An `Outline` polygon as a Detection-shaped view with a filled mask.

    The mask is the polygon rasterised at its integer bounding box -- the same
    anchoring `Detection.mask` uses, so `identity` reads both the same way.
    None for a degenerate polygon.
    """
    import numpy as np
    pts = np.asarray(outline.points, dtype=np.int32).reshape(-1, 2)
    if len(pts) < 3:
        return None
    x1, y1 = pts.min(axis=0)
    x2, y2 = pts.max(axis=0)
    w, h = int(x2 - x1 + 1), int(y2 - y1 + 1)
    try:
        import cv2
        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [pts - [x1, y1]], 1)
    except Exception:                   # noqa: BLE001 -- no cv2: box only
        return None
    v = _DetView((outline.class_name, (x1 + x2) / 2.0, (y1 + y2) / 2.0,
                  float(x2 - x1), float(y2 - y1), outline.score))
    v.xyxy = (float(x1), float(y1), float(x2 + 1), float(y2 + 1))
    v.mask = mask
    return v


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


Outline = None


def _pick_outline(msg, target_class: str):
    """Largest-area outline of `target_class` in a TargetContours message, or None."""
    global Outline
    if Outline is None:
        from collections import namedtuple
        Outline = namedtuple('Outline', 'class_name score angle_deg area_px points')
    want = target_class.strip().lower()
    names = list(msg.class_name)
    offs, pts = list(msg.offset), list(msg.points)
    best = None
    for i, n in enumerate(names):
        if str(n).strip().lower() != want or i + 1 >= len(offs):
            continue
        a, b = int(offs[i]), int(offs[i + 1])
        poly = [(int(pts[2 * k]), int(pts[2 * k + 1])) for k in range(a, b)
                if 2 * k + 1 < len(pts)]
        area = int(msg.area_px[i]) if i < len(msg.area_px) else 0
        if best is None or area > best.area_px:
            best = Outline(str(n), float(msg.score[i]), int(msg.angle_deg[i]),
                           area, poly)
    return best


def _write_ppm(img, folder: str, stem: str):
    """sensor_msgs/Image (rgb8 | bgr8) -> binary PPM. None for other encodings."""
    enc = str(img.encoding).lower()
    if enc not in ('rgb8', 'bgr8'):
        return None
    w, h, step = int(img.width), int(img.height), int(img.step)
    data = bytes(img.data)
    rows = [data[r * step:r * step + 3 * w] for r in range(h)]
    if enc == 'bgr8':
        rows = [bytes(b for i in range(0, 3 * w, 3) for b in (row[i + 2], row[i + 1], row[i]))
                for row in rows]
    path = os.path.join(folder, f'{stem}_{_time.strftime("%Y%m%d_%H%M%S")}.ppm')
    with open(path, 'wb') as fh:
        fh.write(f'P6 {w} {h} 255\n'.encode())
        fh.write(b''.join(rows))
    return path


class MonglaMission:
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
        # Single-live-detector orchestration (Jetson VRAM: never run two detectors
        # at once). `_live_camera` is the camera whose detector is currently
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
        ns = f'/mongla/vision/{camera}'
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
        # The frame's CAPTURE stamp, so a fix computed from it can say when it
        # was true (the filter applies it there with `retrodict:=true`).
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        if stamp is not None:
            caps = self.__dict__.setdefault('_det_capture', {})
            caps[camera] = (int(stamp.sec), int(stamp.nanosec))
        seen = self._det_seen.setdefault(camera, {})    # per-class last-seen: detected() reads this
        for rec in records:
            seen[rec[0]] = now                          # rec[0] = lowercased class
        self._det_warm.add(camera)

    def _on_info(self, camera: str, msg) -> None:
        if msg.width and msg.height:
            self._img_size[camera] = (float(msg.width), float(msg.height))
            k = [float(x) for x in msg.k]
            # A zeroed K is what CameraInfo carries before a calibration loads:
            # present is not usable, so only a positive focal is kept.
            if len(k) >= 9 and k[0] > 0.0 and k[4] > 0.0:
                self._cam_k[camera] = (k[0], k[4], k[2], k[5])

    def _resume_default_detector(self, camera: str) -> None:
        """Resume the default camera's detector, once, at first use.

        ⛔ WHY THIS IS NOT INSIDE THE QUERY. Every query funnels through
        `_pump_detections`, and none of them reached `_activate_camera` -- only
        `use_camera` and the vision verbs did. So against a launch that starts
        both detectors paused (which is the correct default: two live models
        alternate on one Hailo at 37.5 Hz per pair against 95.3 Hz for one),
        `while not mongla.detected('gate')` polls a detector that will never
        infer: an empty cache, forever, no error and no timeout.

        ⛔ AND WHY THE SETTLE IS SHORTER THAN THE RECENCY WINDOW. Finding the
        detector means waiting for DDS discovery, and the wait happens INSIDE
        the caller's query. At 1.5 s it made `detected(stale_after=1.0)` --
        "seen within the last second" -- meaningless: frames that arrived just
        before the call aged out DURING it, and `wait_for('red_pipe',
        timeout=3.0)` returned False against five frames published moments
        earlier. `_DISCOVERY_SETTLE_S` must stay well under the 1.0 s default
        `stale_after`, and it is paid at most ONCE PER CAMERA -- a repeat probe
        cannot learn anything new inside one mission, and paying it per query
        would make a `while not detected(...)` search crawl.

        Best-effort by construction: no detector on the graph (a pure-control
        mission, a bag replay) means there is nothing to resume, and the
        mission proceeds exactly as it did before.
        """
        if self._live_camera is not None or camera in self._resume_probed:
            return
        # Frames already in hand beat any graph probe: a cache entry for this
        # camera IS the detector, alive and publishing. Skipping here keeps the
        # settle out of every query after the first frame lands -- which is
        # what stops it eating the recency window a later query measures.
        if self._det_cache.get(camera) is not None:
            self._resume_probed.add(camera)
            return
        self._resume_probed.add(camera)             # once per camera, win or lose
        if self._detector_present(camera, settle=self._DISCOVERY_SETTLE_S):
            self._activate_camera(camera)

    def _pump_detections(self, camera: str) -> None:
        """Spin the node until a /detections frame newer than now arrives.

        The detector publishes every frame, so a live pipeline lands a fresh
        frame within one frame period; a stalled or just-subscribed pipeline
        times out and the caller reads no (or stale) data -> correctly absent.
        """
        # BEFORE the timing reference below: see `_resume_default_detector`.
        self._resume_default_detector(camera)
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
            if mongla.detected('gate'):
                mongla.vision.align('gate', yaw=0, lat=0)

            # search WHILE MOVING -- needs a loop (an `if` runs once!)
            while not mongla.detected('red_pipe'):
                mongla.move_left(2)
            mongla.move_forward(3)

        To wait for a target while holding station, prefer ``wait_for`` (one
        call, no busy-loop). Works inside a vision ``fallback`` too -- the
        search re-enters the verb the moment the target reappears.

        Parameters
        ----------
        target_class : str | ClassRef
            Class name (``'gate'``) or a ``mongla.models.<m>.<c>`` handle.
        camera : str | None
            Camera to query. Defaults to ``mongla.camera``.
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

    def can_see(self, camera: str | None = None, *, timeout: float = 1.0):
        """Can `camera` see at all? `'ok'`, a blind reason, or None if unknown.

        Blind reasons: `'covered'`, `'washout'`, `'glare'`, `'frozen'`. Use it
        to tell "the target is not there" from "the camera cannot show it":

            while not mongla.detected('gate'):
                if mongla.can_see() not in ('ok', None):
                    break          # searching blind wastes the whole budget
                mongla.yaw_right(20)

        ⛔ `'ok'` MEANS NONE OF THOSE FAULTS, NOT A GOOD IMAGE. Defocus and mild
        fog are NOT detectable from the image alone without firing on ordinary
        murky footage -- measured on the real 2025 archive, see seeing.py.
        None = the detector has not published (paused camera, not running).
        """
        import time as _t
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String
        cam = camera or self.camera
        node = self.client.node
        subs = self.__dict__.setdefault('_seeing_subs', {})
        vals = self.__dict__.setdefault('_seeing_vals', {})
        if cam not in subs:
            def _keep(msg, cam=cam):
                vals[cam] = str(msg.data)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            subs[cam] = node.create_subscription(
                String, f'/mongla/vision/{cam}/seeing', _keep, latched)
        deadline = _t.monotonic() + float(timeout)
        while cam not in vals and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(node, timeout_sec=0.0)
        return vals.get(cam)

    def wait_for(self, target_class, *,
                 timeout: float = 10.0,
                 camera: str | None = None,
                 stale_after: float = 1.0) -> bool:
        """Block until `target_class` is seen on `camera`, or `timeout` elapses.

        The loop-free way to (re)acquire a target while stationary -- returns
        True the moment it appears, False if `timeout` passes first. Each poll
        pumps the stream, so this keeps the cache fresh without a busy-loop::

            if mongla.wait_for('gate', timeout=8):
                mongla.vision.align('gate', yaw=0, lat=0)
            else:
                mongla.recover()           # never showed up

        For a search that should KEEP MOVING while looking, use a
        ``while not mongla.detected(...): <small move>`` loop instead.
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

            {'left': lambda: mongla.yaw_left(20),
             'right': lambda: mongla.yaw_right(20),
            }.get(mongla.where('gate'), lambda: mongla.move_forward(1))()

        Use ``where_offset`` for the raw signed offset (fine steering).
        """
        label, _ = self._where_eval(target_class, camera, stale_after, band)
        return label

    def side_on(self, symbol: str, *, structure: str = 'gate',
                camera: str | None = None, stale_after: float = 1.0,
                use_outline: bool = False) -> str:
        """Which side OF THE STRUCTURE the symbol is on: left/right/centre.

        ⛔ NOT `where()`, AND THE DIFFERENCE DECIDES THE GATE. `where()` answers
        "left or right of the FRAME", which is where the camera is pointing.
        The gate's divider is a property of the gate, so a hull sitting off to
        one side reads a correctly-placed placard as the wrong side and flies
        under the wrong half -- with the detector, the pose and the control loop
        all working. This measures against the structure's own midline, so it
        survives being off-axis::

            if mongla.side_on('rescue') == 'left':
                mongla.move_left(1.2)
            mongla.vision.move('gate', fwd=None)      # pass through

        Returns ``'unknown'`` when the structure is not visible or the symbol
        is not on it -- two states a mission branches differently on, so check
        `detected(structure)` to tell them apart. Geometry comes from the
        structure and identity from the symbol, never the reverse: a placard is
        occluded from oblique angles and makes a poor geometric object.
        """
        cam = self._resolve_camera(camera) if hasattr(self, '_resolve_camera') \
            else (camera or self.camera)
        self._pump_detections(cam)
        records = self._records(cam, stale_after)
        if not records:
            return 'unknown'
        from mongla_vision.identity import identify, pick_structure

        dets = [_DetView(r) for r in records]
        struct = pick_structure(dets, structure)
        if struct is None:
            return 'unknown'
        want = str(symbol).strip().lower()
        symbols = [d for d in dets if d.class_name == want]
        if use_outline:
            # OPT-IN: the symbol's segmentation OUTLINE (contours topic) instead
            # of its box -- its visible pixels decide overlap and side, which a
            # half-occluded placard's box cannot. Falls back to boxes silently
            # when no outline is available (box model, contours off).
            o = self.outline(want, camera=cam, stale_after=stale_after)
            view = _outline_view(o) if o is not None else None
            if view is not None:
                symbols = [view]
        got = identify(struct, symbols)
        return got.side if got.label else 'unknown'

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

    def _record_vision(self, verb: str, target: str, camera: str, res) -> None:
        """Put WHERE and HOW a vision verb ended on its scoreboard row.

        The scorecard used to carry only cmd/success/elapsed/msg, and the vision
        server always reports success -- so a run that never saw the gate and
        one that centred it looked identical afterwards. Updates the verb's own
        row (the last `vision_<verb>`); adds one if the verb never reached the
        server.
        """
        import math as _math
        info = self.__dict__.get('_vinfo', {}).get(camera)
        evidence = {
            'target': str(target), 'camera': str(camera),
            'outcome': res.reason, 'saw_target': bool(res.saw_target),
            'x_px': None if _math.isnan(res.x_px) else round(res.x_px, 1),
            'y_px': None if _math.isnan(res.y_px) else round(res.y_px, 1),
            'fill': round(float(res.fill), 3), 'fired': res.fired,
            'model': list(info[0]) if info else None,
        }
        board = self.__dict__.setdefault('_scoreboard', [])
        for row in reversed(board):
            if row.get('cmd') == f'vision_{verb}' and 'vision' not in row:
                row['vision'] = evidence
                return
        board.append({'cmd': f'vision_{verb}', 'success': bool(res.ok),
                      'elapsed': round(float(res.elapsed_s), 2),
                      'msg': res.reason, 'vision': evidence})

    def save_evidence(self, camera: str | None = None, tag: str = 'evidence', *,
                      timeout: float = 2.0):
        """Save the next `image_debug` frame (boxes drawn) into the run folder.

        OPT-IN. Returns the file path, or None when no frame arrived. Written as
        binary PPM: no OpenCV or cv_bridge needed (both fail under numpy 2), and
        any viewer opens it. The path goes on the scoreboard so the scorecard row
        and the picture of how the task ended travel together.
        """
        import time as _t
        from sensor_msgs.msg import Image
        cam = camera or self.camera
        node = self.client.node
        got = {}
        sub = node.create_subscription(
            Image, f'/mongla/vision/{cam}/image_debug',
            lambda m: got.setdefault('img', m), 1)
        try:
            deadline = _t.monotonic() + float(timeout)
            while 'img' not in got and _t.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_subscription(sub)
        img = got.get('img')
        path = _write_ppm(img, _run_dir(), f'{tag}_{cam}') if img is not None else None
        self.__dict__.setdefault('_scoreboard', []).append(
            {'cmd': f'evidence:{tag}', 'success': path is not None,
             'elapsed': 0.0, 'msg': path or f'no image_debug frame from {cam}'})
        return path

    @contextlib.contextmanager
    def task(self, name: str, *, deadline_s: float | None = None):
        """Bound a block of verbs by time, and abandon it cleanly when it runs out.

        OPT-IN. When the deadline passes, the goal in flight is CANCELLED (the
        manager brakes and neutralises as for any cancel) and `TaskAbandoned`
        is raised out of the block, so the mission takes its fallback::

            try:
                with mongla.task('torpedo', deadline_s=90):
                    mongla.vision.align('hole', ...)
            except TaskAbandoned:
                mongla.fire(1)                      # blind shot, keep the points

        With no `deadline_s` and a budget on (`use_budget`), the deadline is what
        the budget has left. With neither, the block only records its outcome.
        Nested tasks keep the EARLIER deadline. `stop`/`surface`/`disarm`/`pause`
        are never blocked by a passed deadline.
        """
        import time as _t
        from mongla_planner.client import TaskAbandoned
        budget = getattr(self, '_budget', None)
        if deadline_s is None and budget is not None and budget.started:
            deadline_s = budget.remaining_s()
        start = _t.monotonic()
        outer = getattr(self.client, '_task_deadline', None)
        mine = start + float(deadline_s) if deadline_s is not None else None
        eff = mine if outer is None else (outer if mine is None else min(outer, mine))
        self.client._task_deadline = eff
        outcome = 'done'
        try:
            yield
        except TaskAbandoned:
            outcome = 'abandoned'
            self.log.warning(f'[TASK ] {name}: ABANDONED at deadline '
                             f'({_t.monotonic() - start:.1f} s)')
            raise
        except Exception as exc:
            outcome = f'failed: {exc}'
            raise
        finally:
            self.client._task_deadline = outer
            self.__dict__.setdefault('_scoreboard', []).append({
                'cmd': f'task:{name}', 'success': outcome == 'done',
                'elapsed': round(_t.monotonic() - start, 2), 'msg': outcome})

    def use_budget(self, total_s: float = 900.0, *, reserve_s: float = 45.0):
        """Ration the run by the clock. OPT-IN: nothing is rationed until this is called.

        The clock starts on a SUCCESSFUL `arm()`, not on script start -- time
        spent waiting for a tether to come off is not run time. `reserve_s` is
        the surface-and-disarm allowance and is never offered to a task.
        Returns the `RunBudget` so a mission can also call `plan()` on it.

            mongla.use_budget(900, reserve_s=60)
            v = mongla.worth_attempting('torpedo', points=300, worst_case_s=120,
                                        fallback_s=25, fallback_points=100)
            if v.mode == 'full': ...
            elif v.mode == 'fallback': ...          # e.g. blind fire
        """
        from mongla_planner.run_budget import RunBudget
        self._budget = RunBudget(float(total_s), reserve_s=float(reserve_s))
        self.log.info(f'[BUDG ] run budget ON: {total_s:.0f} s total, '
                      f'{reserve_s:.0f} s reserve, clock starts on arm')
        return self._budget

    def note(self, name: str, msg: str, *, success: bool = True) -> None:
        """Put a mission-level fact on the scoreboard that no verb reports.

        e.g. `note('navigation', 'unconfirmed: blind transit', success=False)` --
        a run that proceeds on an attempt must not read as a confirmed pass.
        """
        (self.log.info if success else self.log.warning)(f'[NOTE ] {name}: {msg}')
        self._scoreboard.append({'cmd': f'note:{name}', 'success': bool(success),
                                 'elapsed': 0.0, 'msg': str(msg)})

    def worth_attempting(self, name: str, *, points: int, worst_case_s: float,
                         fallback_s: float = 0.0, fallback_points: int = 0):
        """`Verdict(attempt, mode='full'|'fallback'|'skip', reason, remaining_s)`.

        With no `use_budget()` the answer is always attempt/full -- the opt-out
        IS not calling `use_budget`. The verdict goes on the scoreboard, so a
        skipped task explains itself after the run.
        """
        from mongla_planner.run_budget import Task, Verdict
        budget = getattr(self, '_budget', None)
        if budget is None:
            v = Verdict(True, 'full', 'no run budget configured', float('inf'))
        else:
            v = budget.verdict(Task(str(name), int(points), float(worst_case_s),
                                    fallback_s=float(fallback_s),
                                    fallback_points=int(fallback_points)))
            (self.log.info if v.attempt else self.log.warning)(
                f'[BUDG ] {name}: {v.mode} -- {v.reason}')
        self._scoreboard.append({'cmd': f'budget:{name}', 'success': bool(v.attempt),
                                 'elapsed': 0.0, 'msg': f'{v.mode}: {v.reason}'})
        return v

    def step(self, name: str, *, points: int, worst_case_s: float,
             run, fallback=None, fallback_s: float = 0.0,
             fallback_points: int = 0, needs: str = ''):
        """One declared piece of a run. Pairs with `run_plan()`.

        `run` and `fallback` are CALLABLES taking `(mongla)` -- never verb
        names, the same rule `resilience.py` holds: a mechanism that accepts a
        verb name can be pointed at `disarm` by a config edit.

        `needs` names a verb the step cannot work without; the plan skips the
        step when the backend refuses it, instead of discovering that mid-dive.
        """
        return {'name': str(name), 'points': int(points),
                'worst_case_s': float(worst_case_s), 'run': run,
                'fallback': fallback, 'fallback_s': float(fallback_s),
                'fallback_points': int(fallback_points), 'needs': str(needs)}

    def run_plan(self, steps, *, order: str = 'as_written'):
        """Execute a declared plan, re-deciding from the LIVE clock each time.

        This is the whole point: a mission stops being a hand-ordered script
        that hopes it fits, and becomes a declaration of what each task is worth
        and what it costs. Between steps the clock has moved and the world has
        changed, so the verdict is asked again -- never precomputed once.

            mongla.use_budget(900, reserve_s=60)
            mongla.run_plan([
                mongla.step('gate',   points=100, worst_case_s=120,
                            run=gate, fallback=blind_transit,
                            fallback_s=25, fallback_points=40),
                mongla.step('bins',   points=200, worst_case_s=180, run=bins),
                mongla.step('flares', points=50,  worst_case_s=90,
                            run=flares, needs='fire'),
            ])

        What it handles so the author does not:
          * the budget verdict per step (full / fallback / skip),
          * a deadline per step, so an overrun is abandoned rather than eating
            the run -- `TaskAbandoned` is caught here and the plan continues,
          * a verb the backend refuses (`needs=`), skipped with a reason,
          * every outcome on the scorecard.

        `order='by_value'` sorts by points per second first. ⚠ It ignores where
        the props are -- `by_points_per_second` says so itself -- so it is for
        deciding what to DROP, not the order to swim. Default is as written.

        Returns the list of `(name, outcome)` actually executed.
        """
        from mongla_planner.run_budget import Task, by_points_per_second

        steps = list(steps)
        if order == 'by_value':
            ranked = by_points_per_second([
                Task(s['name'], s['points'], s['worst_case_s'],
                     fallback_s=s['fallback_s'],
                     fallback_points=s['fallback_points']) for s in steps])
            by_name = {s['name']: s for s in steps}
            steps = [by_name[t.name] for t in ranked]
            self.log.info('[PLAN ] order by value: '
                          + ' > '.join(s['name'] for s in steps))

        results = []
        for s in steps:
            name = s['name']

            if s['needs'] and not self.can(s['needs']):
                self.note(name, f"skipped: backend refuses {s['needs']!r}",
                          success=False)
                results.append((name, 'unsupported'))
                continue

            verdict = self.worth_attempting(
                name, points=s['points'], worst_case_s=s['worst_case_s'],
                fallback_s=s['fallback_s'], fallback_points=s['fallback_points'])

            if not verdict.attempt:
                results.append((name, 'skipped'))
                continue

            use_fallback = verdict.mode == 'fallback'
            body = s['fallback'] if use_fallback else s['run']
            if body is None:
                self.note(name, 'no fallback authored for this step',
                          success=False)
                results.append((name, 'skipped'))
                continue

            deadline = s['fallback_s'] if use_fallback else s['worst_case_s']
            try:
                with self.task(name, deadline_s=deadline):
                    body(self)
                results.append((name, verdict.mode))
            except TaskAbandoned:
                # A task that cannot be finished must not cost the run. The
                # deadline already cancelled the goal in flight; keep swimming.
                results.append((name, 'abandoned'))

        done = [n for n, o in results if o in ('full', 'fallback')]
        self.log.info(f'[PLAN ] {len(done)}/{len(steps)} steps carried out: '
                      + ', '.join(f'{n}={o}' for n, o in results))
        return results

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

            h = mongla.head()
            mongla.lock_heading(target=h)
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
        `ros2 run mongla_manager connect` lists which channels are fireable.

        `result.final_value` carries the outcome code (`FIRE_*` in
        `mongla_control.fc.base`), so a mission can tell "refused, that channel is
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

            mongla.turn(90)          # face east, from any current heading
            mongla.turn(0)           # face north (shortest path)
            mongla.turn(270)         # face west
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
        E.g. ``mongla.arc(90, 4)`` sweeps onto heading 90° over a 4 s forward run.
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
    # ``mongla.cam_switch_settle_s = <seconds>`` before the switch if a run needs
    # it snappier or slower.
    _CAM_SWITCH_SETTLE_S = 1.5

    # Known dual-camera detector cameras. On every switch we pause EVERY one of
    # these except the target (not just the previously-live one), so exclusivity
    # holds from the FIRST use_camera even if the node was launched paused:=false
    # (both detectors inferring from t=0 -> the concurrent-inference OOM). Pausing
    # an absent one is a quiet no-op, so single-camera runs are unaffected.
    _KNOWN_CAMERAS = ('forward', 'downward')

    # In-water horizontal FOV per camera, used ONLY when that camera has not
    # published a calibrated CameraInfo. Derived from the committed calibration
    # files through the flat port (`optics.fov_air_to_water`):
    #   downward  Microdia global shutter, fx 1027.87 @1280: 63.8 air -> 46.7 water
    #   forward   Fantech Luminous C30,    fx  851.23 @1280: 73.9 air -> 53.6 water
    # ⛔ This was ONE constant, 46.7, measured on the global-shutter unit when it
    # was still called "forward". The cameras were re-assigned on 2026-09-07 and
    # the constant stayed, so every forward-camera range read 17 % long and every
    # forward bearing 13 % short, with nothing logging a fault.
    HFOV_WATER_DEG_BY_CAMERA = {'forward': 53.6, 'downward': 46.7}

    def _optics(self, camera):
        """(fx, fy, cx, cy, width, height) in AIR from CameraInfo, or None."""
        ks = getattr(self, '_cam_k', None)
        sizes = getattr(self, '_img_size', None)
        if not isinstance(ks, dict) or not isinstance(sizes, dict):
            return None
        k, wh = ks.get(camera), sizes.get(camera)
        if not k or not wh or not wh[0]:
            return None
        return (*k, float(wh[0]), float(wh[1]))

    def _water_camera(self, camera):
        """(rectifier, K_rect) for `camera` from its live CameraInfo, or None.

        ⛔ A FLAT PORT HAS NO SINGLE FOCAL LENGTH. The in-water focal grows with
        field angle (forward camera at 640 px: ~567 px at the centre, ~634 at the
        edge), so ANY one focal -- the old shared 46.7 deg, or a per-camera FOV --
        is wrong by up to ~12 % somewhere in the frame. Rectified pixels ARE a
        pinhole at `K_rect`, which is the model `lock_node` and `pnp_node` use;
        the DSL now uses the same one instead of a second copy.
        """
        o = MonglaMission._optics(self, camera)
        if o is None:
            return None
        from mongla_vision.optics import rectifier_for
        fx, fy, cx, cy, _w, _h = o
        medium = getattr(self, 'medium', 'water')
        medium = medium if isinstance(medium, str) else 'water'
        rect, K_rect, _note = rectifier_for(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], medium)
        if rect is None:                          # air: K is already the pinhole
            class _Identity:
                @staticmethod
                def rectify(pts):
                    return pts
            rect = _Identity()
        return rect, K_rect

    def hfov_water_deg(self, camera: str | None = None) -> float:
        """In-water horizontal FOV of `camera`: its live calibration, else its fallback."""
        import math as _math
        from mongla_vision.optics import fov_air_to_water
        cam = camera or getattr(self, 'camera', 'forward')
        o = MonglaMission._optics(self, cam)
        if o is not None:
            fx, _fy, _cx, _cy, w, _h = o
            return fov_air_to_water(2.0 * _math.degrees(_math.atan(w / 2.0 / fx)))
        return float(MonglaMission.HFOV_WATER_DEG_BY_CAMERA.get(
            str(cam), MonglaMission.HFOV_WATER_DEG_BY_CAMERA['forward']))

    def bearing_to(self, prop_class: str, *, camera: str | None = None,
                   hfov_deg: float | None = None) -> float | None:
        """Bearing from the hull to a visible prop, in WORLD terms. None if unseen.

        Pixel offset to angle, then relative to absolute through the anchored
        heading. Uses `where_offset`, which is the bbox centre as a fraction of
        half the frame, so this is the small-angle pinhole reading of a
        calibrated lens rather than a full unprojection through K. Good to
        about a degree near the centre and worse at the edge, which is fine for
        a resection whose geometry gate wants 12 deg of separation.

        Unanchored, the number returned is relative to boot, so a fix built
        from it is in a rotated frame. `fix_position` says so rather than
        letting the frames silently mix.
        """
        off = self.where_offset(prop_class, camera=camera)
        if off is None:
            return None
        cam = camera or getattr(self, 'camera', 'forward')
        wc = None if hfov_deg is not None else MonglaMission._water_camera(self, cam)
        if wc is not None:
            import math as _math
            rect, K_rect = wc
            w = MonglaMission._optics(self, cam)[4]
            u = (float(off) + 1.0) * w / 2.0
            u_r = float(rect.rectify([[u, K_rect[1][2]]])[0][0])
            rel = _math.degrees(_math.atan((u_r - K_rect[0][2]) / K_rect[0][0]))
            return (self.absolute_heading() + rel) % 360.0
        half = float(hfov_deg if hfov_deg is not None
                     else MonglaMission.hfov_water_deg(self, camera)) / 2.0
        return (self.absolute_heading() + float(off) * half) % 360.0

    def fix_position(self, *, props: list | None = None,
                     camera: str | None = None) -> object:
        """Where we are in the pool, from bearings to props we can see.

        ⛔ THE COURSE IS THE LANDMARK FIELD. A team that could not localise
        underwater added obstacles to their pool to make features, and wrote
        the approach off because a venue will not let you. That is backwards:
        the competition course is the densest set of surveyed, known-size,
        already-detected objects we will ever operate in. The thing they had to
        fake is what the venue hands us.

        Needs a loaded course for the positions, two visible props whose
        positions were measured, and enough angle between them. Returns a
        `Fix` whose `ok` is False with a stated reason -- a thin geometry is
        refused rather than reported with a big residual, because two nearly
        parallel sight lines still cross, just far away and wrongly.
        """
        from mongla_localization.resection import Fix, fix_from_bearings

        course = getattr(self, '_course', None)
        if course is None:
            return Fix(False, reason='no course loaded: call use_course(<name>)')
        if getattr(self, '_heading_offset', None) is None:
            return Fix(False, reason='heading is not anchored, so bearings are '
                                     'relative to boot and a fix would be in a '
                                     'rotated frame. anchor_on(<prop>) first.')
        positions, sightings = {}, {}
        for name, prop in course.props.items():
            if props is not None and name not in props:
                continue
            if not prop.has_position:
                continue
            cls = prop.detect_class or name
            bearing = self.bearing_to(cls, camera=camera)
            if bearing is None:
                continue
            positions[name] = (float(prop.x_m), float(prop.y_m))
            sightings[name] = bearing
        got = fix_from_bearings(sightings, positions)
        if got.ok:
            # FEED THE FILTER, not just the log. `update_position` is the only
            # channel that bounds horizontal drift, and until this line the fix
            # was computed, printed, and thrown away -- the filter had no
            # caller for the one measurement it cannot replace. Best-effort:
            # a missing localization node must not fail a mission step.
            self._publish_fix(got.x_m, got.y_m, camera=camera or self.camera)
            self.log.info(
                f'[FIX  ] pool position ({got.x_m:+.2f}, {got.y_m:+.2f}) m from '
                f'{got.used} props, residual {got.residual_m:.3f} m, '
                f'spread {got.separation_deg:.0f} deg')
        else:
            self.log.warning(f'[FIX  ] no position fix: {got.reason}')
        return got

    def pose(self, *, timeout: float = 2.0, any_frame: bool = False):
        """Where the filter thinks we are: `(x_m, y_m, depth_m, yaw_deg)` or None.

        The mission-facing read of `/mongla/odom`. None when the localization
        node is not running, has not published yet, or -- the part that
        matters -- when its answer is not yet in the POOL frame.

            here = mongla.pose()
            if here and here[0] > 4.0:
                mongla.turn(180)

        ⛔ NONE IS ALSO THE ANSWER FOR A POSITION THAT EXISTS AND IS WRONG.
        Two ways that happens, and neither looks any different from a good
        reading at this interface:

          * BEFORE THE HEADING IS ANCHORED the estimate is in the board's boot
            frame, so comparing it against a course coordinate is a comparison
            in a rotated frame. The node says so in `header.frame_id` -- it
            publishes `odom` until an anchor arrives and `pool` after -- and
            this reads that field rather than trusting the numbers.
          * WITH NO VELOCITY AIDING the position runs away. Measured on the
            vehicle with flow absent and ZUPT off: 635 m in 95 s, while the
            node published a healthy-looking pose at 10 Hz throughout. The
            node warns in ITS log; a mission holding a bare tuple cannot hear
            that, which is this repo's recurring defect with the roles
            reversed -- an honest producer and a deaf consumer.

        Returning None rather than a tuple is the same rule already applied to
        a NaN yaw in `_effective_yaw_deg`: absence is safe, a plausible wrong
        number is not.

        `any_frame=True` returns the unanchored reading anyway, for a
        diagnostic that genuinely wants to watch the filter converge. It is
        opt-in precisely so it cannot be the thing a mission does by accident.

        Pairs with `fix_position()` / `fix_from_prop()`, which push a resected
        position INTO the filter; this reads the fused result back out.
        """
        import math
        import time as _t
        from nav_msgs.msg import Odometry
        node = self.client.node
        if self._odom_sub is None:
            def _keep(msg):
                self._odom = msg
            self._odom_sub = node.create_subscription(
                Odometry, '/mongla/odom', _keep, 10)
        deadline = _t.monotonic() + float(timeout)
        # Pump rather than sleep: the mission thread owns this executor, so a
        # bare sleep here would spin the clock and receive nothing.
        while self._odom is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if self._odom is None:
            return None
        frame = str(self._odom.header.frame_id)
        if frame != 'pool' and not any_frame:
            if not self._pose_frame_warned:
                self._pose_frame_warned = True
                self.log.warning(
                    f'[LOCAL] pose() withheld: the filter is publishing in '
                    f'{frame!r}, not the pool frame. anchor_on(<prop>) makes '
                    f'it absolute. (pose(any_frame=True) to read it anyway.)')
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        # /mongla/odom is NED: z is +depth, and this API returns depth negative.
        return (float(p.x), float(p.y), -float(p.z), yaw)

    def motion(self, *, timeout: float = 1.0):
        """Is the hull moving the way it is being told to?

        `'ok'` | `'blocked'` | `'unknown'`, or None when localization is not
        running. `'blocked'` means thrust is commanded and the floor is not
        moving: against a prop, snagged, or a thruster is dead.

            mongla.vision.move('gate', fwd=80)
            if mongla.motion() == 'blocked':
                mongla.move_back(duration=2)

        ⛔ A TIMED MOVE REPORTS SUCCESS WHETHER OR NOT THE HULL WENT ANYWHERE.
        This is the only reading that can tell them apart without a detection.
        `'unknown'` is common and honest: no flow, no learned model, or a board
        primitive running (the host does not know its demand). Treat unknown as
        "no evidence", never as "ok".
        """
        import time as _t
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String
        node = self.client.node
        if getattr(self, '_motion_sub', None) is None:
            self._motion = None

            def _keep(msg):
                self._motion = str(msg.data)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._motion_sub = node.create_subscription(
                String, '/mongla/localization/motion', _keep, latched)
        deadline = _t.monotonic() + float(timeout)
        while self._motion is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        # Pump once more so a change since the last call is seen.
        rclpy.spin_once(node, timeout_sec=0.0)
        return self._motion

    # ------------------------------------------------------------------ #
    #  Model provenance -- `vision_info` (OPT-IN: subscribed on first use) #
    # ------------------------------------------------------------------ #

    def active_models(self, camera: str | None = None, *, timeout: float = 1.0):
        """`(model_stems, epoch)` the detector says is live on `camera`, or None.

        Read from the detector's latched `vision_info`: `database_location` is the
        comma list of live stems (primary first) and `database_version` an epoch
        bumped on every switch. None when the detector has not announced one.
        """
        import time as _t
        from vision_msgs.msg import VisionInfo
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        cam = camera or self.camera
        subs = self.__dict__.setdefault('_vinfo_subs', {})
        latest = self.__dict__.setdefault('_vinfo', {})
        node = self.client.node
        if cam not in subs:
            def _keep(msg, c=cam):
                stems = tuple(x for x in str(msg.database_location).split(',') if x)
                latest[c] = (stems, int(msg.database_version))
            subs[cam] = node.create_subscription(
                VisionInfo, f'/mongla/vision/{cam}/vision_info', _keep,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = _t.monotonic() + float(timeout)
        rclpy.spin_once(node, timeout_sec=0.0)
        while cam not in latest and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return latest.get(cam)

    def _confirm_model(self, camera: str, name: str, before, timeout_s: float) -> None:
        """Block until `vision_info` shows `name` live with a NEW epoch, then drop
        every detection cached before it. Raises if the detector never confirms.

        ⛔ WHY: the parameter set returning is not the model running. Until the
        detector republishes `vision_info`, frames in the cache came from the OLD
        model, and a `detected()` right after a switch can answer about a class
        the new model does not even have.
        """
        import time as _t
        want = [x.strip() for x in name.split(',') if x.strip()]
        if before and list(before[0][:len(want)]) == want:
            # Already the live selection: the detector does not re-announce a
            # no-op, and nothing cached came from a different model.
            self.log.info(f'[DSL  ] {camera}: model {name!r} was already live')
            return
        # Names, not the epoch: the epoch resets to 1 when a detector restarts,
        # so "newer epoch" would refuse a real switch on a respawned node.
        deadline = _t.monotonic() + timeout_s
        got = None
        while _t.monotonic() < deadline:
            got = self.active_models(camera, timeout=0.05)
            if got and list(got[0][:len(want)]) == want:
                self._det_cache.pop(camera, None)
                self._det_seen.pop(camera, None)
                self.log.info(f'[DSL  ] {camera}: model {name!r} confirmed live '
                              f'(epoch {got[1]}); pre-switch detections dropped')
                return
        raise RuntimeError(f'set_model({name!r}): detector on {camera!r} did not confirm '
                           f'within {timeout_s:.1f} s (last vision_info: {got})')

    # ------------------------------------------------------------------ #
    #  Outlines -- `contours` (OPT-IN: subscribed on first use)            #
    # ------------------------------------------------------------------ #

    def outline(self, target_class, *, camera: str | None = None,
                stale_after: float = 1.0, timeout: float = 1.0):
        """The largest outline of `target_class` in the latest contours frame, or None.

        Returns `Outline(class_name, score, angle_deg, area_px, points)`, `points`
        a list of (x, y) image pixels. A box model gives 4 corners, a segmentation
        model the mask outline -- same call either way. `angle_deg` is the
        oriented-box angle (a path marker's direction). Needs the detector's
        `publish_contours` on (`contours:=true`, the vehicle launch default).
        """
        import time as _t
        from mongla_interfaces.msg import TargetContours
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        subs = self.__dict__.setdefault('_contour_subs', {})
        frames = self.__dict__.setdefault('_contours', {})
        node = self.client.node
        if cam not in subs:
            def _keep(msg, c=cam):
                frames[c] = (_t.monotonic(), msg)
            subs[cam] = node.create_subscription(
                TargetContours, f'/mongla/vision/{cam}/contours', _keep,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        deadline = _t.monotonic() + float(timeout)
        rclpy.spin_once(node, timeout_sec=0.0)
        while cam not in frames and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        entry = frames.get(cam)
        if entry is None or _t.monotonic() - entry[0] > float(stale_after):
            return None
        return _pick_outline(entry[1], str(target_class))

    def floor_height(self, *, max_age_s: float = 1.0, timeout: float = 1.0):
        """Height above the floor in metres, from the floor's own tiles, or None.

        Measured by the downward camera off the tile grating (`tile_m` must be
        set for the venue), NOT depth minus a typed pool depth. None when no
        tiles are visible or the last reading is older than `max_age_s`:
        a stale height reads as a fresh one to every comparison.

            h = mongla.floor_height()
            if h is None:
                ...                     # no tiles in view: do not guess
            elif h < 0.6:
                ...                     # too close to the floor for the drop
        """
        import time as _t
        from sensor_msgs.msg import Range
        node = self.client.node
        if getattr(self, '_floor_h_sub', None) is None:
            self._floor_h = None

            def _keep(msg):
                self._floor_h = (float(msg.range), _t.time())
            self._floor_h_sub = node.create_subscription(
                Range, '/mongla/vision/downward/floor_height', _keep, 10)
        deadline = _t.monotonic() + float(timeout)
        while self._floor_h is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(node, timeout_sec=0.0)
        if self._floor_h is None:
            return None
        h, rx = self._floor_h
        return h if _t.time() - rx <= float(max_age_s) else None

    def flare_order(self, *, timeout: float = 0.0, max_age_s: float = 3.0):
        """The flare order the operator sent over LoRa this mission, or None.

        Returns a tuple of colour names, e.g. ('red', 'blue', 'yellow'). Blocks up to
        `timeout` seconds for one to arrive (0 = just look). None when no order has been
        received, when the manager stopped republishing it more than `max_age_s` ago,
        or when the board latched it BEFORE this mission started -- a practice order
        still in the board's RAM must never stand in for this run's.

            order = mongla.flare_order(timeout=45.0)   # hold at the listen station
            if order is None:
                ...                                    # no order: bump all, earn 20 each
        """
        import json
        import time as _t
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        node = self.client.node
        if getattr(self, '_flare_sub', None) is None:
            self._flare = None

            def _keep(msg):
                try:
                    d = json.loads(msg.data)
                    self._flare = (tuple(d['colours']), float(d['latched_at']), _t.time())
                except (ValueError, KeyError, TypeError):
                    pass
            self._flare_sub = node.create_subscription(
                String, '/mongla/flare_order', _keep,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def _current():
            if self._flare is None:
                return None
            colours, latched_at, rx = self._flare
            if latched_at < self._mission_start_wall or _t.time() - rx > float(max_age_s):
                return None
            return colours

        deadline = _t.monotonic() + float(timeout)
        rclpy.spin_once(node, timeout_sec=0.0)
        while _current() is None and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return _current()

    def range_to(self, prop_class: str, *, camera: str | None = None,
                 stale_after: float = 1.0):
        """Metres to a visible prop of known width, or None. Also its sigma.

        Returns `(range_m, sigma_m)`. The pinhole relation `Z = f W / w` needs
        nothing but the prop's real width and its box width in pixels, both of
        which we already have -- `target_geometry` is the measured width table
        and the detector gives the box.

        ⛔ THE SIGMA IS DERIVED, NOT CHOSEN. Differentiating gives
        `dZ/dw = -Z^2 / (f W)`, so range error grows with the SQUARE of range:
        one pixel of box-width noise is 6 mm at 1 m and 15 cm at 5 m for a gate.
        A constant sigma would tell the filter a far reading is as good as a
        near one, which is the single most common way a landmark fix poisons a
        position estimate. This matches the published analytical models for
        planar-marker pose variance, which are functions of range and angle.
        """
        from mongla_vision.target_geometry import width_for

        if isinstance(prop_class, ClassRef):
            prop_class = prop_class.class_name
        real_w = width_for(str(prop_class))
        if real_w <= 0.0:
            return None
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        target = str(prop_class).strip().lower()
        boxes = [r for r in self._records(cam, stale_after) if r[0] == target]
        if not boxes:
            return None
        # The LARGEST box, not the most confident -- size is what the range is
        # read off, and a small confident box is a worse range than a large
        # doubtful one. Same rule as `identity.pick_structure`.
        big = max(boxes, key=lambda r: float(r[3]))
        w_px = float(big[3])
        if w_px <= 0.0:
            return None
        wc = MonglaMission._water_camera(self, cam)
        if wc is not None:
            # Both EDGES through the port, then the pinhole at K_rect: the box
            # width compresses by the refraction at ITS field angle.
            rect, K_rect = wc
            e = rect.rectify([[big[1] - w_px / 2.0, big[2]], [big[1] + w_px / 2.0, big[2]]])
            w_px = abs(float(e[1][0]) - float(e[0][0]))
            f_px = float(K_rect[0][0])
            if w_px <= 0.0:
                return None
            z = f_px * real_w / w_px
            return (z, (z * z) * 1.0 / (f_px * real_w))
        img_w = self._img_size.get(cam, (0.0, 0.0))[0] or 640.0
        f_px = self.focal_px(img_w, camera=cam)
        z = f_px * real_w / w_px
        # One pixel of box-width noise, which is optimistic for a YOLO box and
        # stated as such rather than padded with an invented factor.
        sigma = (z * z) * 1.0 / (f_px * real_w)
        return (z, sigma)

    def floor_range(self, target: str, *, plane_below_m: float, pitch_deg: float,
                    camera: str | None = None,
                    plane_sigma_m: float = 0.0, stale_after: float = 1.0):
        """Metres to a target standing on (or hanging from) a known plane.

        Returns `(range_m, sigma_m)` like `range_to`, or None. Needs NO prop
        width: the ray through the box's foot meets the floor at one point.
        `plane_below_m` = floor depth under the target minus the camera's
        depth (positive); pass a negative value for a surface-hung target and
        its TOP edge is used instead. Use when `range_to` has nothing to say --
        unknown width, or a box cut by the frame side -- and take whichever
        sigma is smaller when both answer. Detail and failure modes:
        `mongla_localization.floor_plane`.

        ⛔ BOTH GEOMETRY ARGUMENTS ARE REQUIRED, ON PURPOSE. Pitch is the
        dominant error term (17 cm per degree at h 1 m, R 3 m), and the bench
        hull read +1.77 deg nose-up, so a silent level-camera default would be
        a ~30 cm bias no test sees. `pitch_deg` is the optical axis BELOW
        horizontal: mount pitch minus the hull's nose-up pitch (forward camera
        on a hull pitched 1.77 deg nose-up: `pitch_deg=-1.77`).

        ⛔ SIGNS. Depths here are negative below the surface, and getting the
        subtraction backwards yields a NEGATIVE plane, which is read as a
        surface-hung target -- a confident wrong answer, not a refusal:

            hull = mongla.pose()[2]                   # e.g. -0.60
            plane_below_m = abs(-1.60) - abs(hull)    # floor 1.60 m -> 1.00

        Pixels are assumed square (`focal_px` is fx and is passed as fy too),
        which holds for the cameras we fly.

        ⛔ A box touching the frame edge on the side we read is REFUSED: the
        foot is outside the image and the box edge is not the foot, which would
        return a confident range to the frame border.
        """
        from mongla_localization.floor_plane import intersect

        if isinstance(target, ClassRef):
            target = target.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        name = str(target).strip().lower()
        boxes = [r for r in self._records(cam, stale_after) if r[0] == name]
        if not boxes:
            return None
        _, cx, cy, w, h, _ = max(boxes, key=lambda r: r[3] * r[4])
        img_w, img_h = self._img_size.get(cam, (0.0, 0.0))
        img_w, img_h = (img_w or 640.0), (img_h or 480.0)
        v = cy + h / 2.0 if plane_below_m > 0.0 else cy - h / 2.0
        if v >= img_h - 2.0 or v <= 2.0:
            return None
        wc = MonglaMission._water_camera(self, cam)
        if wc is not None:
            # The foot through the port, then the pinhole at K_rect -- which also
            # carries the calibrated principal point: the forward camera's cy sits
            # 70 px above centre at 720 p, ~6 deg of pitch, about a metre at 3 m.
            rect, K_rect = wc
            cx, v = (float(x) for x in rect.rectify([[cx, v]])[0])
            f_px, fy_px = float(K_rect[0][0]), float(K_rect[1][1])
            pcx, pcy = float(K_rect[0][2]), float(K_rect[1][2])
        else:
            f_px = self.focal_px(img_w, camera=cam)
            pcx, pcy, fy_px = img_w / 2.0, img_h / 2.0, f_px
        hit = intersect(cx, v, fx=f_px, fy=fy_px, cx=pcx, cy=pcy,
                        plane_below_m=float(plane_below_m),
                        pitch_deg=float(pitch_deg),
                        plane_sigma_m=float(plane_sigma_m))
        return None if hit is None else (hit.range_m, hit.sigma_m)

    def fix_from_prop(self, prop: str, *, camera: str | None = None):
        """Pool position from ONE surveyed prop: range and bearing together.

        ⛔ THIS IS THE DVL WE DO NOT HAVE. `fix_position()` needs TWO props
        with 12 degrees between them, because bearings alone cannot fix a
        range -- and two props are visible together far less often than one.
        Adding the range collapses that requirement: a single prop of known
        width at a surveyed position pins the hull completely.

        Everything it needs already existed separately and was never combined:
        the surveyed position from the course map, the absolute bearing from
        the anchored heading, and the range from the width table. The
        arithmetic is one line; the capability is the difference between
        localising when the course cooperates and localising whenever anything
        is in view.

        Returns a `Fix`. Publishes to the filter on success, with the
        range-derived sigma rather than a constant.
        """
        import math as _m
        from mongla_localization.resection import Fix

        course = getattr(self, '_course', None)
        if course is None:
            return Fix(False, reason='no course loaded: call use_course(<name>)')
        if getattr(self, '_heading_offset', None) is None:
            return Fix(False, reason='heading is not anchored, so the bearing '
                                     'is relative to boot and the fix would be '
                                     'in a rotated frame. anchor_on(<prop>) first.')
        p = course.props.get(prop)
        if p is None:
            return Fix(False, reason=f'{prop!r} is not in course {course.name!r}')
        if not p.has_position:
            return Fix(False, reason=f'{prop!r} has no measured position; a '
                                     f'prop we cannot place cannot place us')
        cls = p.detect_class or prop
        bearing = self.bearing_to(cls, camera=camera)
        if bearing is None:
            return Fix(False, reason=f'{cls!r} is not visible right now')
        got = self.range_to(cls, camera=camera)
        if got is None:
            return Fix(False, reason=f'no width for {cls!r}, so no range')
        rng, sigma = got
        # The hull sits one range BACK along the bearing from the prop.
        rad = _m.radians(bearing)
        x = float(p.x_m) - rng * _m.cos(rad)
        y = float(p.y_m) - rng * _m.sin(rad)
        self._publish_fix(x, y, sigma=sigma, camera=camera or self.camera)
        self.log.info(
            f'[FIX  ] pool position ({x:+.2f}, {y:+.2f}) m from {prop!r} at '
            f'{rng:.2f} m bearing {bearing:.0f} deg (sigma {sigma:.2f} m)')
        return Fix(True, x_m=x, y_m=y, used=1, residual_m=0.0,
                   separation_deg=0.0)

    def _publish_fix(self, x_m: float, y_m: float,
                     sigma: float | None = None, camera: str | None = None) -> None:
        """Hand a resected pool position to the invariant filter.

        A topic rather than a direct call, because the filter runs in its own
        process on its own thread and the mission is a client. Lazily created,
        same pattern as `_publish_active_camera`.
        """
        try:
            from geometry_msgs.msg import PointStamped
            if getattr(self, '_fix_pub', None) is None:
                self._fix_pub = self.client.node.create_publisher(
                    PointStamped, '/mongla/localization/fix', 10)
            m = PointStamped()
            m.header.stamp = self.client.node.get_clock().now().to_msg()
            # When the fix came from a camera frame, stamp it with THAT frame's
            # capture time: the hull was there then, not at publish time.
            cap = self.__dict__.get('_det_capture', {}).get(camera) if camera else None
            if cap and (cap[0] or cap[1]):
                m.header.stamp.sec, m.header.stamp.nanosec = cap
            m.header.frame_id = 'pool'
            m.point.x = float(x_m)
            m.point.y = float(y_m)
            # z carries the sigma. PointStamped has no covariance field and
            # inventing a message type for one float is worse than documenting
            # this; the filter reads it back the same way. <= 0 means "use the
            # node's default", so a caller that has no sigma is not forced to
            # invent one.
            m.point.z = float(sigma if sigma is not None else 0.0)
            self._fix_pub.publish(m)
        except Exception as exc:            # noqa: BLE001 -- best-effort
            self.log.warning(f'[FIX  ] fix not published to the filter: {exc}')

    def focal_px(self, width_px: float = 640.0, *, camera: str | None = None) -> float:
        """In-water focal length of `camera`, DERIVED from its field of view.

        `fx = (W/2) / tan(HFOV/2)` with the camera's own in-water FOV
        (`hfov_water_deg`). For the downward camera at 640 px that is 741 px,
        which the flow node independently reports; the forward camera is 634 px.
        """
        import math as _math
        return (float(width_px) / 2.0) / _math.tan(
            _math.radians(MonglaMission.hfov_water_deg(self, camera) / 2.0))

    def standoff_for_prop(self, prop_class: str, *, visibility_m: float | None = None,
                          width_px: float = 640.0, camera: str | None = None) -> float:
        """How close we must be for `prop_class` to be comfortably detectable.

        Arithmetic, not a habit: a prop of known width projects to a box of
        `f * w / Z` pixels, the detector was MEASURED to fall off a cliff at
        about 10 px of box width, and the answer is clamped by how far the
        water lets us see. Which term binds depends on the prop -- pixels for a
        slalom pipe at 1.7 m, water for a gate whose pixel-limited range is
        154 m.
        """
        from mongla_vision.acquire import VISIBILITY_M, standoff_for
        from mongla_vision.target_geometry import width_for

        w = width_for(str(prop_class))
        if w <= 0.0:
            return float(visibility_m if visibility_m is not None else VISIBILITY_M)
        return standoff_for(
            w, self.focal_px(width_px, camera=camera),
            visibility_m=(VISIBILITY_M if visibility_m is None else visibility_m))

    def acquire(self, prop: str, *, camera: str | None = None,
                visibility_m: float | None = None,
                stale_after: float = 1.0, max_legs: int = 8):
        """Get the prop ON CAMERA: approach to a computed standoff, then search.

        ⛔ THE DEAD ZONE THIS CLOSES. `goto_prop` stops short by design, and an
        ARBITRARY standoff can leave the hull where the prop is present and
        undetectable -- too far for the box to survive the detector, with the
        mission unable to tell "not there" from "too far to see". So the
        standoff is computed from the prop's committed width, the measured
        detector floor and the water, and only then does a search make sense.

        ⛔ AND THE SEARCH IS A RACE, WHICH IS THEIR IDIOM AND NOT A TIMEOUT.
        Each leg is run in short segments with a detection check between them,
        so the moment the prop appears the leg is abandoned mid-flight. A
        search that completes its pattern before looking has already swum past
        the answer.

        The pattern is an expanding square sized from the error we actually
        carry -- the fix residual plus what a heading error throws a leg of
        this length off its line -- so a confident fix searches a small box and
        a shaky one searches a big one, with nobody choosing a number. Legs are
        closed with the distance verb, never timed.

        Returns an `Attempt`: `branch` says whether the approach alone found
        it, which leg did, or that the pattern ran out.
        """
        from mongla_planner.resilience import Attempt
        from mongla_vision.acquire import expanding_box, search_radius_m, total_path_m

        cls = str(prop)
        course = getattr(self, '_course', None)
        detect_class = cls
        if course is not None and cls in course.props:
            detect_class = course.props[cls].detect_class or cls

        standoff = self.standoff_for_prop(detect_class, visibility_m=visibility_m,
                                          camera=camera)
        self.log.info(f'[ACQ  ] {cls!r}: computed standoff {standoff:.2f} m '
                      f'(class {detect_class!r})')

        leg = self.goto_prop(cls, standoff_m=standoff)
        if self.detected(detect_class, camera=camera, stale_after=stale_after):
            self.log.info(f'[ACQ  ] {cls!r} on camera after the approach')
            return Attempt(f'acquire:{cls}', True, branch='approach')

        residual = 0.0
        fix = self.fix_position()
        if fix.ok:
            residual = float(fix.residual_m)
        reach = search_radius_m(residual, standoff)
        legs = expanding_box(max(0.5, reach / 2.0), reach_m=reach * 2.0,
                             max_legs=max_legs)
        self.log.warning(
            f'[ACQ  ] {cls!r} not visible at the standoff. Searching an '
            f'expanding box: reach {reach:.1f} m, {len(legs)} legs, '
            f'{total_path_m(legs):.1f} m of swimming.')

        for l in legs:
            self.yaw_right(l.turn_deg)
            if self._run_watching(l.run_m, detect_class, camera, stale_after):
                self.log.info(f'[ACQ  ] {cls!r} found on search leg {l.index}')
                return Attempt(f'acquire:{cls}', True, branch=f'leg{l.index}')
        return Attempt(f'acquire:{cls}', False, tries=len(legs),
                       error=f'{cls!r} not found within {reach * 2.0:.1f} m of '
                             f'the prior. It is not where the map says.')

    def _run_watching(self, metres: float, detect_class: str,
                      camera: str | None, stale_after: float,
                      segment_m: float = 0.5) -> bool:
        """Run `metres`, checking for the class between short segments.

        The imperative form of their `Parallel(SuccessOnOne)[motion, seen]`:
        a detection abandons the leg mid-flight instead of after it. Segment
        length is the granularity of that preemption, and shorter is not free
        -- each segment is a separate closed-loop command.
        """
        remaining = float(metres)
        while remaining > 1e-3:
            step = min(float(segment_m), remaining)
            self.move_forward_dist(step)
            remaining -= step
            if self.detected(detect_class, camera=camera, stale_after=stale_after):
                return True
        return False

    def goto_prop(self, prop: str, *, standoff_m: float = 2.0,
                  gain: float = 50.0, max_leg_m: float = 12.0):
        """Dead-reckon to within `standoff_m` of a prop, then hand to perception.

        ⛔ THE WHOLE POINT IS THAT IT STOPS SHORT. This does not arrive at the
        prop, it arrives in DETECTION RANGE of it, which is all a prior map can
        honestly deliver: the map is a rulebook and a tape measure, and the last
        couple of metres belong to the camera. That is how the team that wins
        does waypoints -- and on the task where perception was hardest they
        shipped plain waypoints and never ran the filter they had built.

        Everything it needs is refused loudly rather than guessed:

          * a loaded course with a MEASURED position for the prop;
          * an anchored heading, or the turn would be in a boot-relative frame;
          * a position fix, which needs two visible props and real geometry;
          * a closed-loop distance verb. ⛔ It never falls back to a timed
            guess -- that fallback once drove 2.361 m for a 1.0 m command and
            reported success, which is worse than refusing because the mission
            believes it arrived.

        Returns the `Attempt` of the leg, or a refusal naming the missing part.
        """
        from mongla_planner.resilience import Attempt
        from mongla_localization.resection import _wrap180
        from mongla_localization.course_map import bearing_to, range_to
        import math as _math

        course = getattr(self, '_course', None)
        if course is None:
            return Attempt(f'goto:{prop}', False,
                           error='no course loaded: use_course(<name>) first')
        try:
            target = course.position_of(prop)       # raises when unmeasured
        except (KeyError, ValueError) as exc:
            return Attempt(f'goto:{prop}', False, error=str(exc))

        fix = self.fix_position()
        if not fix.ok:
            return Attempt(f'goto:{prop}', False,
                           error=f'no position fix, so nothing to reckon from: '
                                 f'{fix.reason}')

        here = (fix.x_m, fix.y_m)
        bearing = bearing_to(here, target)
        leg = range_to(here, target) - float(standoff_m)
        if leg <= 0.0:
            self.log.info(f'[GOTO ] already inside the {standoff_m:.1f} m '
                          f'standoff of {prop!r}; perception takes it from here')
            return Attempt(f'goto:{prop}', True, branch='already_there')
        if leg > float(max_leg_m):
            return Attempt(
                f'goto:{prop}', False,
                error=f'{leg:.1f} m leg exceeds max_leg_m={max_leg_m:.0f}; a '
                      f'dead-reckoned run that long accumulates more error than '
                      f'the standoff it is aiming for')

        self.log.info(
            f'[GOTO ] {prop!r} at ({target[0]:+.1f}, {target[1]:+.1f}) is '
            f'{bearing:.0f} deg and {leg + standoff_m:.1f} m away; turning and '
            f'running {leg:.1f} m to a {standoff_m:.1f} m standoff')
        self.turn(self.absolute_to_relative(bearing))
        res = self.move_forward_dist(leg, gain=gain)
        ok = bool(res)
        if not ok:
            self.log.warning(
                f'[GOTO ] the distance leg did not close. There is no timed '
                f'fallback here on purpose: search for {prop!r} instead of '
                f'assuming the vehicle arrived.')
        return Attempt(f'goto:{prop}', ok, branch='dead_reckon')

    def use_course(self, name: str):
        """Load the prop priors for a course. Returns the `Course`.

        The prior half of perceive-then-move: dead-reckon to roughly where a
        prop should be and let perception take the last few metres. A mission
        that searches for every prop from scratch spends its run turning.

        Deck-first search (`MONGLA_COURSE_DIR`, then `~/.mongla/courses`, then
        the package), so a course re-measured at the venue beats one committed
        months earlier without a rebuild. Unmeasured props are named in the log
        rather than left to be discovered by driving to them::

            mongla.use_course('robosub26')
            mongla.anchor_on('gate')          # the course supplies the bearing
        """
        from mongla_localization.course_map import load_course

        course = self._course = load_course(name)
        unmeasured = course.unmeasured()
        self.log.info(f'[CRSE ] {course.name} loaded from {course.source} '
                      f'({len(course.props)} props)')
        if unmeasured:
            self.log.warning(
                f'[CRSE ] NOT measured for this pool: {", ".join(unmeasured)}. '
                f'Those props refuse a dead-reckon and the mission must search '
                f'for them.')
        return course

    def anchor_on(self, prop: str, *, camera: str | None = None,
                  timeout: float = 6.0):
        """Anchor the heading on a course prop, taking its bearing from the map.

        ⛔ ONE NUMBER, ONE PLACE. A prop's face bearing is what an absolute
        heading needs AND what approaching it from the front needs. Passing it
        by hand at each call site is how the two come to disagree, so this
        reads it from the loaded course and refuses if it was never measured --
        a guessed bearing writes a wrong heading zero, and every later turn
        inherits it.
        """
        course = getattr(self, '_course', None)
        if course is None:
            raise RuntimeError(
                'anchor_on() needs a course: call use_course(<name>) first')
        bearing = course.bearing_of(prop)      # raises when unmeasured
        return self.anchor_heading(bearing_deg=bearing, camera=camera,
                                   timeout=timeout)

    def anchor_heading(self, *, bearing_deg: float, camera: str | None = None,
                       timeout: float = 6.0) -> object:
        """Re-zero the heading against a prop whose world bearing is known.

        ⛔ WHAT THIS BUYS. The BNO is deliberately magnetometer-free, so its yaw
        is relative to wherever the board booted: drift under 0.01 deg/min, and
        an arbitrary zero. Every mission that says `turn(90)` means 90 from
        boot, so a hull powered on at a different angle flies a different course
        from the same file. A prop the rulebook fixes the orientation of is a
        heading reference already in the pool.

        `bearing_deg` is the compass bearing the prop's FACE points along, which
        is per-COURSE knowledge -- where this gate was installed -- and not a
        property of the class, so it is passed in rather than read from the
        committed geometry table::

            a = mongla.anchor_heading(bearing_deg=270)   # gate faces west
            if a.ok:
                mongla.turn(mongla.absolute_to_relative(90))   # true east

        Consumes the FUSED pose, never a raw one: the planar flip would write a
        heading wrong by twice the off-axis angle, and every later turn would
        inherit it. Returns an `Anchor` whose `ok` is False with a stated
        reason rather than a number, and does NOT change what `turn()` means --
        a mission opts in by converting explicitly.
        """
        from mongla_localization.heading_anchor import anchor_from

        cam = str(camera or self.camera).strip().lower()
        fused = self._wait_fused_pose(cam, timeout=timeout)
        if fused is None:
            from mongla_localization.heading_anchor import Anchor
            return Anchor(False, reason=f'no fused pose on {cam} within '
                                        f'{timeout:.0f}s')
        got = anchor_from(fused, self.head(), float(bearing_deg))
        if got.ok:
            self._heading_offset = got.offset_deg
            # TELL THE FILTER. Until this line the anchor was a mission-local
            # variable: `absolute_heading()` used it, `bearing_to()` used it,
            # and the estimator -- the one component whose whole output is
            # labelled with a frame -- never heard about it. Its attitude came
            # from the board, which is boot-relative or magnetic, so
            # `/mongla/odom` claimed a pool frame it had no way to be in, and
            # flow would have dead-reckoned off along that unmeasured offset.
            # Same no-caller defect as the filter itself, one layer up.
            self._publish_heading(got.absolute_deg)
            self.log.info(
                f'[ANCH ] heading anchored: relative {self.head():.1f} is truly '
                f'{got.absolute_deg:.1f} (offset {got.offset_deg:+.1f}, '
                f'{got.support} frames, rule={got.rule})')
        else:
            self.log.warning(f'[ANCH ] heading NOT anchored: {got.reason}')
        return got

    def _publish_heading(self, absolute_deg: float) -> None:
        """Hand the anchored world heading to the invariant filter.

        Latched, because the anchor is a one-shot event and the filter may
        start after the mission does -- a VOLATILE publish would be heard by
        nobody and leave the estimator in the boot frame while every log line
        said it was anchored.
        """
        try:
            from std_msgs.msg import Float32
            if getattr(self, '_heading_pub', None) is None:
                qos = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
                self._heading_pub = self.client.node.create_publisher(
                    Float32, '/mongla/localization/heading', qos)
            self._heading_pub.publish(Float32(data=float(absolute_deg)))
        except Exception as exc:            # noqa: BLE001 -- best-effort
            self.log.warning(f'[ANCH ] heading not published to the filter: {exc}')

    def absolute_heading(self) -> float:
        """The hull's heading in WORLD terms, or the relative one if unanchored.

        Passing through unanchored is deliberate: a mission written against
        boot-relative headings keeps working exactly as before, and silently
        changing what every existing `turn()` means would be far more dangerous
        than making the conversion explicit.
        """
        from mongla_localization.heading_anchor import apply_offset
        return apply_offset(self.head(), getattr(self, '_heading_offset', None))

    def absolute_to_relative(self, absolute_deg: float) -> float:
        """A WORLD heading -> the number to hand `turn()`.

        `turn()` speaks the hull's own relative frame and this does not change
        that. Unanchored, it is the identity -- so a mission can be written in
        world headings and still run on a hull that never saw its landmark,
        just without the correction.
        """
        off = getattr(self, '_heading_offset', None)
        if off is None or off != off:
            return float(absolute_deg)
        return float(absolute_deg) - float(off)

    def _wait_fused_pose(self, camera: str, *, timeout: float):
        """Latest DECIDED fused pose for `camera`, or None.

        Subscribes lazily: a mission that never anchors pays nothing, and a
        stack launched without `lock:=true` has no such topic to subscribe to.
        """
        from mongla_interfaces.msg import TargetPose

        subs = getattr(self, '_fused_subs', None)
        if subs is None:
            subs = self._fused_subs = {}
            self._fused_pose = {}
        if camera not in subs:
            topic = f'/mongla/vision/{camera}/target_pose_fused'
            subs[camera] = self.client.node.create_subscription(
                TargetPose, topic,
                lambda msg, c=camera: self._fused_pose.__setitem__(c, msg), 10)
            self.log.info(f'[ANCH ] listening on {topic}')
        deadline = _time.monotonic() + float(timeout)
        while _time.monotonic() < deadline:
            rclpy.spin_once(self.client.node, timeout_sec=0.05)
            msg = self._fused_pose.get(camera)
            if msg is not None and msg.ok:
                from mongla_localization.pose_cluster import Fused
                return Fused(decided=True, yaw_deg=float(msg.yaw_deg),
                             range_m=float(msg.range_m),
                             support=int(msg.n_points),
                             spread_deg=float(msg.yaw_spread_deg),
                             rule=str(msg.reason))
        return None

    def camera_available(self, name: str | None = None) -> bool:
        """Is ``name``'s detector actually on the graph? Cached per camera.

        THE BRANCH A MISSION NEEDS BEFORE IT COMMITS TO AN EYE. A camera that
        fails to open no longer kills the vision stack -- the other camera keeps
        running and that camera's detector is simply absent (see
        `detector_dual_node`). That is the right behaviour for the vehicle and
        it hands the mission a decision: a bin task that steers on `downward`
        must SKIP rather than drive blind, and it can only skip if it can ask.

        Without this the mission has two bad options. Call the vision verb and
        `_ensure_detector` aborts the whole run over one dead camera -- losing
        every later task with it. Or do not call it, and the mission cannot
        tell "camera dead" from "target not in view".

        Use it as the fallback selector the task trees are built from::

            if mongla.camera_available('downward'):
                mongla.vision.align('fire', camera='downward', lat=0, fwd=0)
                mongla.fire(3)
            else:
                mongla.log('downward camera absent -- skipping the bin drop')

        Cheap after the first call per camera: the answer is cached, because a
        detector that is up stays up for the run and one that never came up is
        not going to appear mid-mission.
        """
        cam = str(name or self.camera).strip().lower()
        if cam in self._camera_available:
            return self._camera_available[cam]
        ok = self._detector_present(cam, settle=self._DISCOVERY_SETTLE_S)
        self._camera_available[cam] = ok
        if not ok:
            self.log.warning(
                f'[CAM  ] {cam} detector is NOT on the graph. A task that '
                f'steers on {cam} will find nothing -- branch on '
                f'camera_available({cam!r}) rather than calling a vision verb.')
        return ok

    def use_camera(self, name: str) -> None:
        """Switch the sticky camera for all subsequent vision verbs AND make it the
        single live detector (pause the other, resume this one, point the HUD at it).

        Logs the switch so pool-side operators see the transition. Idempotent on the
        detector switch (no-op when already live). Best-effort: a missing detector
        node warns rather than crashing, so pure-control / single-camera runs are
        unaffected.

        Example::

            mongla.use_camera('downward')            # downward detector live, HUD flips
            mongla.vision.align('fire', lat=0, depth=0, err=30)
            mongla.use_camera('forward')             # back to forward
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
                    String, '/mongla/vision/active_camera', qos)
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

            mongla.calc_distance('start')
            mongla.move_forward(4.0, gain=60)      # blind timed move, measured
            metres = mongla.calc_distance('stop')

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

        Single naming rule for the whole stack: ``/mongla_detector_<camera>``
        (e.g. ``/mongla_detector_forward``, ``/mongla_detector_downward``),
        matching the names the vision launch files give the detector nodes.
        ``camera`` defaults to the mission's sticky camera. An explicit
        ``node`` always wins (escape hatch for non-standard setups).
        """
        if node:
            return node
        return f'/mongla_detector_{camera or self.camera}'

    # How long `_detector_present` may spin waiting for DDS discovery. Only the
    # FIRST query pays it, and only when the node has not been seen yet.
    _DISCOVERY_SETTLE_S = 0.4

    def _detector_present(self, camera: str, *, settle: float = 0.0) -> bool:
        """Graph check: is the ``camera`` detector node currently up?

        Reads the discovery graph (``get_node_names``) -- no service wait -- so
        the exclusivity loop can SKIP an absent counterpart instead of paying
        ``_ensure_detector``'s 5 s ``wait_for_service`` on every switch of a
        single-camera run. Best-effort: any error -> treat as absent (skip).

        ⛔ `settle` EXISTS BECAUSE AN INSTANT GRAPH READ ANSWERS "ABSENT" FOR A
        NODE THAT IS RUNNING. `get_node_names` reports what DDS discovery has
        found SO FAR, and a mission's first query runs a fraction of a second
        after its own node is created -- before the detector has been
        discovered. Measured on the vehicle: `detected('gate')` returned in
        0.61 s with `live_camera=None` while `ros2 param get
        /mongla_detector_forward paused` answered fine from the same shell.
        A caller that must not act on a false "absent" passes `settle` and
        spins until the node appears or the budget runs out. The exclusivity
        loop keeps the instant read: there, a false absent only skips a pause
        it can redo, and the 5 s service wait is the thing being avoided.
        """
        node = self._detector_node(camera)
        want = node.lstrip('/')
        deadline = _time.monotonic() + max(0.0, float(settle))
        while True:
            try:
                names = self.client.node.get_node_names()
            except Exception:   # noqa: BLE001 -- graph read is best-effort
                return False
            if want in names or node in names:
                return True
            if _time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self.client.node, timeout_sec=0.05)

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
                cam = node.rsplit('mongla_detector_', 1)[-1]
                raise RuntimeError(
                    f"Detector node {node} NOT FOUND after {timeout:.0f}s -- the "
                    f"vision stack is not running, but this mission uses vision. "
                    f"Start it, e.g.:\n"
                    f"  ros2 launch mongla_vision vision.launch.py camera:={cam} "
                    f"model:=<stem> classes:=<csv>\n"
                    f"(aborting loudly so a missing detector can't cost a run)")
        finally:
            ros_node.destroy_client(cli)
        self._detector_ok.add(node)

    def _refuse_duplicate_node(self, node: str) -> None:
        """Raise if `node`'s name is on the graph more than once.

        Best-effort on the graph read itself: a discovery hiccup must not block
        a legitimate write, so an unreadable graph is treated as "one".
        """
        want = node.lstrip('/')
        try:
            names = list(self.client.node.get_node_names())
        except Exception:   # noqa: BLE001 -- graph read is best-effort
            return
        n = sum(1 for x in names if x.lstrip('/') == want)
        if n > 1:
            raise RuntimeError(
                f'{n} nodes named {node!r} are on the graph -- a parameter '
                f'write would land on an arbitrary one. Stop the duplicate '
                f'stack (orphaned launch?) before tuning.')

    def _set_detector_param(self, node: str, name: str, value) -> None:
        """Set one DETECTOR parameter, aborting loudly if the node is absent.

        `_ensure_detector` is the detector-specific half: "someone forgot to
        start the vision stack" must stop the mission rather than warn, because
        every vision verb after it would idle on err=+inf. The wire work is
        shared with every other node -- see `_set_node_param`.
        """
        self._ensure_detector(node)
        self._set_node_param(node, name, value)

    def _set_node_param(self, node: str, name: str, value) -> None:
        """Set one parameter on ANY node, in-process via SetParameters.

        Replaces the old ``subprocess('ros2 param set')`` which spun up a fresh
        CLI node that had to re-discover the target every call (the flaky,
        silent "Node not found" source). Raises on rejection or timeout so a
        failure is loud rather than a swallowed warning: these are the knobs a
        task's behaviour depends on.
        """
        ros_node = self.client.node
        # ⛔ A DUPLICATE NODE NAME MAKES THIS WRITE A COIN FLIP. Two processes
        # can hold the same node name -- an orphaned launch, a second stack
        # started by mistake -- and ROS does not arbitrate: the service call
        # lands on whichever answers, which may be the one NOT driving the
        # vehicle. The setting then reads back correctly from one instance and
        # does nothing to the pipeline, which is indistinguishable from a knob
        # that was never wired. Measured on the vehicle: ten
        # `mongla_tracker_forward` nodes on one graph after repeated launches.
        #
        # Refuse instead. A tuning call that might not take is worse than one
        # that says it cannot.
        self._refuse_duplicate_node(node)
        cli = self._param_clients.get(node)
        if cli is None:
            cli = ros_node.create_client(SetParameters, f'{node}/set_parameters')
            self._param_clients[node] = cli
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(
                f'{node}/set_parameters unavailable -- is that node running?')
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
                  camera: str | None = None, node: str | None = None,
                  confirm_s: float = 0.0) -> None:
        """Switch the active detector model, or run SEVERAL at once.

        ``name`` is the model **stem** (e.g. ``'gate_rescue_repair'``) or a
        registry key. Works on BOTH launch styles: a single-model launch
        (``model:=<stem>``) accepts ``set_model('<that stem>')`` as a no-op and
        rejects any *other* name; a registry launch (``models:=``) accepts the
        key OR the stem. Targets ``/mongla_detector_<camera>`` (camera defaults to
        the mission's). Raises if the node is absent or the switch is rejected.

        **Pass a list or a comma-separated string to run more than one model on
        every frame** -- a detector and a segmentation model together, say::

            mongla.use('gate_rescue_repair')                  # detection only
            mongla.use('gate_seg')                            # segmentation only
            mongla.use(['gate_rescue_repair', 'gate_seg'])    # both, merged

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
        cam = camera or self.camera
        before = self.active_models(cam, timeout=0.0) if confirm_s > 0.0 else None
        try:
            self._set_detector_param(node, 'active_model', str(name))
        except RuntimeError as exc:
            if 'registry' in str(exc).lower():
                raise RuntimeError(
                    f'set_model({name!r}): {exc} -- launch with models:="..." '
                    f'to enable hot model switching') from None
            raise
        self.log.info(f'[DSL  ] {node} active_model → {name!r}')
        if confirm_s > 0.0:
            self._confirm_model(cam, str(name), before, float(confirm_s))

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

            mongla.use('gate', 'gate')    # switch model and filter together
            mongla.use('combined', '')    # combined model, all classes visible
        """
        self.set_model(model, camera=camera, node=node)
        if classes is not None:
            self.set_classes(classes, camera=camera, node=node)

    def set_classes(self, classes: str | list, *,
                    camera: str | None = None, node: str | None = None) -> None:
        """Switch the detector's class filter without restarting the node.

        Example::

            mongla.set_classes('gate')
            mongla.set_classes(['gate', 'flare'])
            mongla.set_classes('')   # all classes
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

            mongla.set_conf(0.35)                              # all models
            mongla.set_conf(0.55, model='torpedo_blood_hole')  # torpedo only

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

    def _subsystem_node(self, kind: str, camera: str | None = None) -> str:
        try:
            pattern = self._NODE_SUFFIX[kind]
        except KeyError:
            raise ValueError(
                f'unknown subsystem {kind!r}; known: '
                f'{sorted(self._NODE_SUFFIX)}') from None
        cam = str(camera or self.camera).strip().lower()
        return '/' + pattern.format(cam=cam)

    def set_node(self, kind: str, *, camera: str | None = None, **params) -> None:
        """Set parameters on ANY node in the stack, by subsystem name.

        ⛔ WHY THIS EXISTS. A census of the tree found 164 declared parameters
        across 13 nodes and a DSL that could write two groups of them: the
        detector, and the manager's ``vision.*``. Everything else -- the
        tracker's coast and Kalman noise, the camera's exposure and rate, the
        lock ladder's anchor and authority windows, the flow front end's 31
        knobs, PnP's reprojection gate -- was launch-only, which means frozen at
        whatever someone typed before the vehicle went in the water. A
        capability the mission cannot reach is a capability the mission does not
        have.

        One method, seven subsystems, no new layer::

            mongla.set_node('tracker', coast_s=1.2)      # hold a flickering lock
            mongla.set_node('camera',  exposure_us=4000) # kill motion blur
            mongla.set_node('lock',    full_authority_s=3.0)
            mongla.set_node('pnp',     max_reproj_px=15.0)
            mongla.set_node('flow',    max_baseline_s=0.4)
            mongla.set_node('manager', **{'vision.kp_yaw': 70.0})

        ``camera`` selects which per-camera instance; ``flow`` and ``manager``
        are single nodes and ignore it. Raises on a rejected or undeclared
        parameter, for the same reason ``set_detector`` does: a silent no-op is
        a mission believing it changed something.
        """
        if not params:
            raise ValueError('set_node() needs at least one parameter')
        node = self._subsystem_node(kind, camera)
        for name, value in params.items():
            self._set_node_param(node, name, value)
            self.log.info(f'[DSL  ] {node} {name} → {value!r}')

    def set_detector(self, *, camera: str | None = None,
                     node: str | None = None, **params) -> None:
        """Set ANY detector parameter live. The escape hatch, not a shortcut.

        ``set_model`` / ``set_conf`` / ``set_classes`` exist because they do
        more than write a value -- they resolve a registry name, scope a
        threshold to one model, validate a class against the loaded allowlist.
        Everything else the detector node declares is reachable here without a
        new DSL method per knob, so a parameter added to the node is usable
        from a mission the same day::

            mongla.set_detector(masks=False)             # boxes only on transit
            mongla.set_detector(masks=True)              # outlines at the board
            mongla.set_detector(publish_contours=False)  # silence the topic
            mongla.set_detector(max_det=10, iou=0.5)     # several at once

        Order within one call is not guaranteed; make two calls if one setting
        must land before another. Raises on a rejected or undeclared parameter
        -- a silent no-op here would be a mission believing it had changed
        something. That is deliberately the opposite of best-effort: these are
        the knobs a task's behaviour depends on.
        """
        if not params:
            raise ValueError('set_detector() needs at least one parameter')
        target = self._detector_node(camera, node)
        for name, value in params.items():
            self._set_detector_param(target, name, value)
            self.log.info(f'[DSL  ] {target} {name} → {value!r}')

    def lock_class(self, target: str = '', *, camera: str | None = None,
                   timeout: float = 2.0) -> bool:
        """Aim the continuity ladder MID-MISSION. Returns True if it took.

        The ladder (`lock_node`: follower + XFeat anchor) normally aims itself
        at whatever `set_classes` last told the detector, via the latched
        `classes_filter`. Use this only to PIN it somewhere else -- e.g. hold
        the gate while the detector is already hunting the next prop::

            mongla.lock_class('gate')        # pin
            mongla.lock_class('')            # release; follow the mission again

        Unlike the detector helpers this does NOT abort when the node is
        missing. The ladder is a fallback rung: a mission must run without it,
        and a hard failure here would turn an enhancement into a dependency.
        A clear warning is logged instead, and False returned.
        """
        node = f'/mongla_lock_{camera or self.camera}'
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
    _MANAGER_NODE = '/mongla_manager'

    def set_vision_param(self, name: str, value: float) -> None:
        """Set a manager ``vision.*`` tunable live (applies to the NEXT vision goal).

        For per-MISSION control of a deck tunable without touching every verb call
        -- e.g. a downward bin run fixes its depth floor/ceiling ONCE at the top of
        the mission instead of passing ``max_depth_m=`` / ``depth_ceiling=`` on each
        ``vision.align``::

            mongla.set_vision_param('max_depth_m', -1.6)   # enables+bounds the descent
            mongla.set_vision_param('depth_ceiling', -0.4) # surface guard

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

    _backend_cache = None

    @property
    def backend(self) -> str:
        """The manager's `flight_controller` ('srot' or 'pixhawk'), read once.

        Missions branch on it for verbs one backend refuses (`lock_heading` on
        srot, where the board holds heading itself). Raises when the manager
        cannot be read: guessing a backend would send a refused verb or skip a
        needed one, and neither is safe to discover mid-run.
        """
        if self._backend_cache is None:
            node_name = self._MANAGER_NODE
            ros_node = self.client.node
            cli = ros_node.create_client(GetParameters, f'{node_name}/get_parameters')
            try:
                if not cli.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError(f'{node_name}/get_parameters unavailable')
                fut = cli.call_async(GetParameters.Request(names=['flight_controller']))
                rclpy.spin_until_future_complete(ros_node, fut, timeout_sec=5.0)
                resp = fut.result()
                if resp is None or not resp.values:
                    raise RuntimeError(f'{node_name}.flight_controller unreadable')
                kind = str(resp.values[0].string_value).strip().lower()
                if kind not in ('srot', 'pixhawk'):
                    raise RuntimeError(f'{node_name}.flight_controller={kind!r} unknown')
            finally:
                ros_node.destroy_client(cli)
            self._backend_cache = kind
        return self._backend_cache

    def can(self, verb: str) -> bool:
        """Will this verb actually do something on the backend we are flying?

        The capability oracle the retired FSM layer had and never consulted: it
        carried a `VehicleProfile.has_heading_lock` that was False on srot and
        had **zero callers**, so every plan dispatched `lock_heading` anyway, the
        refusal raised, and the run ended one state after DIVE (J04). A mission
        branches on this instead:

            if mongla.can('lock_heading'):
                mongla.lock_heading(90)

        ONE TRUTH, not a second copy: the answer is read out of
        `srot_fc.UNSUPPORTED_VERBS`, the same frozenset the manager checks before
        dispatch. A hand-maintained list here would be the second place to
        disagree -- which is exactly how J04 happened.
        """
        if self.backend != 'srot':
            return True
        try:
            from mongla_control.fc.srot_fc import UNSUPPORTED_VERBS
        except Exception:                       # control package not importable
            self.log.warn(f"[DSL  ] can({verb!r}): srot_fc unreadable, assuming yes")
            return True
        return verb not in UNSUPPORTED_VERBS

    def require(self, verb: str, *, why: str = '') -> None:
        """Refuse the mission NOW if a verb it depends on cannot run.

        For the case where there is no sensible branch: better to fail on the
        deck, loudly, than to discover it underwater. The opposite of what the
        FSM did, which was to convert the refusal into a silent surface.
        """
        if self.can(verb):
            return
        reason = f' ({why})' if why else ''
        raise MissionRefused(
            f"{verb!r} is refused on backend {self.backend!r}{reason} -- "
            f"branch on mongla.can({verb!r}) or author a fallback")

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
                  message: str = "Wire removed  --  Mongla is now autonomous. Good luck."):
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
            default ``~/mongla_runs``, override with ``MONGLA_RUN_DIR``).
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

