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
    duburi.arc(seconds, gain=50, yaw_rate_pct=30)
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

Detection guards (non-blocking, safe in tight loops):

    # Move forward until the gate is detected, then align
    while not duburi.detected('gate'):
        duburi.move_forward(1.0, gain=30)
    duburi.vision.align('gate', yaw=0, lat=0)

    # Works with ClassRef model handles (no model switching side-effect):
    duburi.models(gate='gate_flare_medium_100ep')
    while not duburi.detected(duburi.models.gate.gate):
        duburi.move_forward(0.5, gain=25)
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0)

    # Override camera or freshness window:
    duburi.detected('flare', camera='downward', stale_after=2.0)

`detected()` subscribes to `/duburi/vision/<camera>/detections` on
first call (lazy, per-camera). It is a zero-wait cache check — every
blocking DSL verb keeps the cache warm via ROS callbacks processed
during the action spin.

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
import subprocess
import sys
import time as _time

import rclpy
from vision_msgs.msg import Detection2DArray

from .model_context import ClassRef, ModelRegistry
from .vision_dsl import _VisionDSL  # noqa: F401 -- re-exported; used by DuburiMission


def _format_outcome(cmd: str, result) -> str:
    return (f'  {cmd:<22s} final={result.final_value:+.3f} '
            f'err={result.error_value:+.3f}  ({result.message})')


def _to_float(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    return value


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
        # Detection cache: camera -> (monotonic_stamp, {class_name, ...})
        # Populated by lazy per-camera subscriptions; refreshed automatically
        # during every blocking send() via spin_until_future_complete.
        self._det_cache: dict[str, tuple[float, set[str]]] = {}
        self._det_subs:  dict[str, object] = {}  # keeps subscriptions alive
        # Scoreboard: ordered list of (cmd, success, elapsed_s, message)
        self._scoreboard: list[dict] = []
        self._mission_start: float = _time.monotonic()

    # ================================================================== #
    #  Single send + log helper                                           #
    # ================================================================== #

    def _send(self, cmd: str, **fields):
        fields = {k: _to_float(v) for k, v in fields.items()}
        t0     = _time.monotonic()
        result = self.client.send(cmd, **fields)
        elapsed = _time.monotonic() - t0
        self.log.info(_format_outcome(cmd, result))
        self._scoreboard.append({
            'cmd':     cmd,
            'success': bool(getattr(result, 'success', False)),
            'elapsed': round(elapsed, 2),
            'msg':     str(getattr(result, 'message', '')),
        })
        return result

    # ================================================================== #
    #  Detection guard -- non-blocking cache check                        #
    # ================================================================== #

    def _subscribe_detections(self, camera: str) -> None:
        # Always read /detections -- the same raw detector topic the control
        # loop (VisionState) and the HUD use, so detected() agrees with what
        # the AUV actually acts on. The tracker stays display-only.
        topic = f'/duburi/vision/{camera}/detections'
        sub = self.client.node.create_subscription(
            Detection2DArray, topic,
            lambda msg, cam=camera: self._on_detections(cam, msg), 10)
        self._det_subs[camera] = sub

    def _on_detections(self, camera: str, msg: Detection2DArray) -> None:
        # Extract class names to plain strings immediately — do not store ROS
        # message objects because rclpy may reuse the underlying C++ memory
        # across callbacks, which would corrupt cached data read later.
        names: set[str] = set()
        for d in msg.detections:
            if not d.results:
                continue
            hyp = d.results[0]
            if hasattr(hyp, 'hypothesis'):
                names.add(str(hyp.hypothesis.class_id))
            else:
                names.add(str(getattr(hyp, 'id', '')))
        self._det_cache[camera] = (_time.monotonic(), names)

    def detected(self, target_class, *,
                 camera: str | None = None,
                 stale_after: float = 1.0) -> bool:
        """Return True if `target_class` was recently detected on `camera`.

        Non-blocking: reads a local cache updated by ROS callbacks. Safe to
        call between DSL verbs in tight loops. Lazily subscribes to the
        `/duburi/vision/<camera>/detections` topic on first call.

        Parameters
        ----------
        target_class : str | ClassRef
            Class name to look for (e.g. ``'gate'``, ``duburi.models.gate.gate``).
        camera : str | None
            Camera to query. Defaults to ``duburi.camera``.
        stale_after : float
            Detections older than this many seconds are treated as absent.

        Examples::

            while not duburi.detected('gate'):
                duburi.move_forward(1.0, gain=30)

            if duburi.detected(duburi.models.gate.flare, stale_after=2.0):
                duburi.vision.align('flare', yaw=0)
        """
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        if cam not in self._det_subs:
            self._subscribe_detections(cam)
        # Wait briefly so the subscriber callback can fire (detector ~15-25 Hz → one frame in 40-66 ms).
        rclpy.spin_once(self.client.node, timeout_sec=0.05)
        entry = self._det_cache.get(cam)
        if entry is None:
            return False
        stamp, class_names = entry
        if _time.monotonic() - stamp > stale_after:
            return False
        # Case-insensitive match, consistent with the control path
        # (VisionState._hypothesis_matches lowercases both sides) so a
        # fallback's should_stop() fires the moment the target reappears.
        needle = str(target_class).strip().lower()
        return any(needle == name.strip().lower() for name in class_names)

    # ================================================================== #
    #  Power / mode                                                        #
    # ================================================================== #

    def arm(self, *, timeout: float = 15.0):
        return self._send('arm', timeout=timeout)

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
        """Fire payload channel via ESP32 serial. 1/2 = torpedo, 3/4 = dropper."""
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

    def arc(self, seconds: float, *, gain: float = 50.0,
            yaw_rate_pct: float = 30.0, settle: float = 0.0):
        return self._send('arc',
                          duration=float(seconds), gain=gain,
                          yaw_rate_pct=yaw_rate_pct, settle=settle)

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

    def use_camera(self, name: str) -> None:
        """Switch the sticky camera for all subsequent vision verbs.

        Logs the switch so pool-side operators see the transition in the
        console. Does not affect already-running vision verbs.

        Example::

            duburi.use_camera('downward')
            duburi.vision.align('bin_marker', lat=0, depth=0, err=30)
            duburi.use_camera('forward')
        """
        self.log.info(f'[MISSION] camera → {name!r}')
        self.camera = name

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

    def set_model(self, name: str, *,
                  node: str = '/duburi_detector') -> None:
        """Switch active detector model by registry name (hot, no restart).

        Requires the detector to have been launched with a ``models`` registry.
        """
        result = subprocess.run(
            ['ros2', 'param', 'set', node, 'active_model', name],
            capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            stderr = result.stderr.strip()
            if 'no registry' in stderr or 'not in registry' in stderr:
                self.log.warning(
                    f'[DSL  ] set_model({name!r}): {stderr}  '
                    f'-- launch with models:="..." to enable hot switching')
            else:
                self.log.warning(
                    f'[DSL  ] set_model({name!r}) failed: {stderr!r}')
        else:
            self.log.info(f'[DSL  ] active_model → {name!r}')

    def use(self, model: str, classes: str | list | None = None, *,
            node: str = '/duburi_detector') -> None:
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
        self.set_model(model, node=node)
        if classes is not None:
            self.set_classes(classes, node=node)

    def set_classes(self, classes: str | list, *,
                    node: str = '/duburi_detector') -> None:
        """Switch the detector's class filter without restarting the node.

        Example::

            duburi.set_classes('gate')
            duburi.set_classes(['gate', 'flare'])
            duburi.set_classes('')   # all classes
        """
        if isinstance(classes, list):
            classes_str = ','.join(str(c).strip() for c in classes)
        else:
            classes_str = str(classes).strip()
        result = subprocess.run(
            ['ros2', 'param', 'set', node, 'classes', classes_str],
            capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            self.log.warning(
                f"[DSL  ] set_classes failed: {result.stderr.strip()!r}")
        else:
            self.log.info(f"[DSL  ] detector classes → {classes_str!r}")

    def set_conf(self, conf: float, *, camera: str = 'forward') -> None:
        """Set YOLO confidence threshold live. Takes effect on next inference tick."""
        node = f'/duburi_detector_{"fwd" if camera == "forward" else "dwn"}'
        result = subprocess.run(
            ['ros2', 'param', 'set', node, 'conf', str(float(conf))],
            capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            self.log.warning(f"[DSL  ] set_conf({conf}) failed: {result.stderr.strip()!r}")
        else:
            self.log.info(f"[DSL  ] {node} conf → {conf:.3f}")

    def pause_detector(self, camera: str = 'forward') -> None:
        """Pause inference on a detector node (frame still consumed from queue)."""
        node = f'/duburi_detector_{"fwd" if camera == "forward" else "dwn"}'
        result = subprocess.run(
            ['ros2', 'param', 'set', node, 'paused', 'true'],
            capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            self.log.warning(
                f"[DSL  ] pause_detector({camera!r}) failed: {result.stderr.strip()!r}")
        else:
            self.log.info(f"[DSL  ] {node} paused")

    def resume_detector(self, camera: str = 'forward') -> None:
        """Resume inference on a detector node."""
        node = f'/duburi_detector_{"fwd" if camera == "forward" else "dwn"}'
        result = subprocess.run(
            ['ros2', 'param', 'set', node, 'paused', 'false'],
            capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            self.log.warning(
                f"[DSL  ] resume_detector({camera!r}) failed: {result.stderr.strip()!r}")
        else:
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

    def log_scoreboard(self, *, json_path: str | None = None) -> None:
        """Print a structured per-verb mission summary and optionally write JSON.

        Called automatically by `mission.py` on exit (normal or exception).
        Can also be called manually at any point during a mission.

        Parameters
        ----------
        json_path : str | None
            If given, write the scoreboard JSON to this path in addition to
            printing to stdout.  Pass ``'auto'`` to generate a timestamped
            filename in the current directory.

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
            if json_path == 'auto':
                ts = _time.strftime('%Y%m%d_%H%M%S')
                json_path = f'mission_scoreboard_{ts}.json'
            payload = {
                'total_s':    total_s,
                'phases':     self._scoreboard,
                'success_count': sum(1 for e in self._scoreboard if e['success']),
                'fail_count':    sum(1 for e in self._scoreboard if not e['success']),
            }
            with open(json_path, 'w') as fh:
                json.dump(payload, fh, indent=2)
            self.log.info(f'[DSL  ] scoreboard written → {json_path}')

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

