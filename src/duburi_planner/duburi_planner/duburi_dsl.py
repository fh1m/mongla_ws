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

Closed-loop vision verbs sit under `duburi.vision`. Each verb corresponds
to one AUV physical motion and maps to one future YASMIN state:

    duburi.vision.find(target, move='still')   -- watch until target seen
    duburi.vision.turn(target, duration)        -- yaw to centre horizontally
    duburi.vision.slide(target, duration)       -- slide laterally to centre
    duburi.vision.hover(target, duration)       -- rise/sink to centre vertically
    duburi.vision.approach(target, dist, metric)-- approach to standoff distance
    duburi.vision.home(target,                  -- multi-axis convergence
                       yaw=True, lat=True,
                       dist=0.42, metric='area',
                       gate_guard=True,
                       pass_at=0.35)
    duburi.vision.track(target, duration)       -- track continuously
    duburi.vision.scan(target, step=20, dwell=1.5) -- orbit-scan for target

Detection guards (non-blocking, safe in tight loops):

    # Move forward until the gate is detected, then align
    while not duburi.detected('gate'):
        duburi.move_forward(1.0, gain=30)
    duburi.vision.home(target='gate', yaw=True, lat=True)

    # Branch on action result (.success is True on any successful verb)
    result = duburi.vision.find(target='gate', timeout=30)
    if result.success:
        duburi.vision.home(target='gate', yaw=True)

    # Works with ClassRef model handles (no model switching side-effect):
    duburi.models(gate='gate_flare_medium_100ep')
    while not duburi.detected(duburi.models.gate.gate):
        duburi.move_forward(0.5, gain=25)
    duburi.vision.home(target=duburi.models.gate.gate)

    # Override camera or freshness window:
    duburi.detected('flare', camera='downward', stale_after=2.0)

`detected()` subscribes to `/duburi/vision/<camera>/detections` on
first call (lazy, per-camera). It is a zero-wait cache check — every
blocking DSL verb keeps the cache warm via ROS callbacks processed
during the action spin.

All DSL verbs return `Move.Result` with `result.success`, `result.final_value`,
and `result.error_value`. Use `result.success` to gate state transitions:

    result = duburi.vision.scan(target='flare', duration=60)
    if result.success:
        duburi.vision.home(target='flare', yaw=True, depth=True)
    else:
        duburi.move_forward(3.0)   # fallback: advance and retry

Model context (multi-model missions):

    duburi.models(
        gate   = 'gate_flare_medium_100ep',
        slalom = 'slalom_combined',
    )
    duburi.vision.find(target=duburi.models.gate.gate,       move='forward', gain=35)
    duburi.vision.home(target=duburi.models.gate.gate,       yaw=True, lat=True)
    duburi.vision.turn(target=duburi.models.slalom.slalom_red)

    # Strict class list (validates attribute access at handle time):
    duburi.models(gate=('gate_flare_medium_100ep', ['gate', 'flare']))
    duburi.vision.find(target=duburi.models.gate.gate)   # OK
    # duburi.vision.find(target=duburi.models.gate.typo) # → AttributeError

When a ClassRef is passed as target, the DSL automatically calls
set_model() + set_classes() before sending the goal — no explicit
duburi.set_classes() or duburi.use() needed per verb.

Canonical competition task pattern (gate pass):

    duburi.models(gate='gate_flare_medium_100ep')
    duburi.set_depth(-1.2)
    duburi.vision.find(target=duburi.models.gate.gate, move='forward', gain=35, timeout=45)
    duburi.vision.turn(target=duburi.models.gate.gate, duration=6)
    duburi.vision.slide(target=duburi.models.gate.gate, duration=5)
    duburi.vision.approach(target=duburi.models.gate.gate, dist=0.42, metric='area', duration=10)
    duburi.vision.home(target=duburi.models.gate.gate, yaw=True, lat=True,
                       gate_guard=True, duration=8)
    duburi.move_forward(3.5, gain=55)

Each DSL line becomes one YASMIN state when the state machine layer is added.

Vision verb quick reference
---------------------------
find(target, move='still', gain=25, yaw_rate_pct=22, timeout=25)
    move: 'still' | 'forward' | 'yaw_right' | 'yaw_left' | 'arc'
    Blocks until target is detected. Optionally moves while searching.

turn(target, duration=8)
    Yaw (Ch4) left/right to horizontally centre target in frame.

slide(target, duration=8)
    Slide (Ch6) laterally to horizontally centre target without changing heading.

hover(target, duration=8)
    Nudge depth setpoint up/down to vertically centre target in frame.

approach(target, dist=0.55, metric='height', duration=12, pass_at=0.0, pass_at_gain=50)
    Drive forward/back to reach standoff distance.
    metric: 'height' (tall objects) | 'width' (wide objects) |
            'area' (gates) | 'diagonal' (all-rounder)
    pass_at: once size >= pass_at, freeze lat+depth and drive straight.

home(target, yaw=True, lat=False, depth=False, forward=False,
     dist=0.55, metric='height',
     gate_guard=False, gate_guard_min_w_frac=0.35,
     pass_at=0.0, pass_at_gain=50, duration=15)
    Multi-axis convergence. Boolean flags select which axes run simultaneously.
    gate_guard: suppress forward when gate appears angled (experimental).
    pass_at: commit to straight-through pass at threshold (experimental).

track(target, yaw=True, forward=True, lat=False, depth=False,
      dist=0.55, duration=60)
    Track continuously until duration expires (never exits on settle).

Detector control (manual — ClassRef targets do this automatically):
    duburi.set_classes('gate')         # only gate detections
    duburi.set_classes('gate,flare')   # gate + flare
    duburi.set_classes('')             # all classes
    duburi.set_model('combined')       # switch model in registry
    duburi.use('combined', 'gate')     # switch model + class in one call

Tunable live (between runs, no rebuild):
    ros2 param set /duburi_manager vision.kp_yaw 80.0
    ros2 param set /duburi_manager vision.kp_forward 150.0
    ros2 param set /duburi_manager vision.deadband 0.10
"""

from __future__ import annotations

import json
import subprocess
import sys
import time as _time

import rclpy
from vision_msgs.msg import Detection2DArray

from .model_context import ClassRef, ModelRegistry


def _format_outcome(cmd: str, result) -> str:
    return (f'  {cmd:<22s} final={result.final_value:+.3f} '
            f'err={result.error_value:+.3f}  ({result.message})')


def _to_float(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    return value


# Maps the `move=` param on `find()` to the internal drive verb.
_MOVE_TO_DRIVER: dict[str, str] = {
    'still':     '',
    'none':      '',
    'forward':   'move_forward',
    'yaw_right': 'yaw_right',
    'yaw_left':  'yaw_left',
    'arc':       'arc',
}


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

    def __init__(self, client, log, *, camera: str = 'laptop',
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
                duburi.vision.home(target='flare', yaw=True)
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
        return target_class in class_names

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

    def lock_heading(self, degrees: float = 0.0, *, timeout: float = 300.0):
        return self._send('lock_heading',
                          target=float(degrees), timeout=timeout)

    def release_heading(self):
        return self._send('unlock_heading')

    # ================================================================== #
    #  DVL                                                                 #
    # ================================================================== #

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


# ---------------------------------------------------------------------- #
#  duburi.vision -- closed-loop sub-namespace                             #
# ---------------------------------------------------------------------- #

class _VisionDSL:
    """duburi.vision.* -- the closed-loop sub-namespace.

    Every verb is a blocking call. On completion it prints one outcome line.
    Gains (kp_*, deadband, on_lost, stale_after) fall back to live ROS params
    when not passed — use `ros2 param set /duburi_manager vision.kp_yaw ...`
    to tune between runs without touching mission code.
    """

    def __init__(self, mission: 'DuburiMission'):
        self._dsl = mission

    # ---- target / camera resolution ----------------------------------- #

    def _resolve_camera(self, camera) -> str:
        return camera if camera else self._dsl.camera

    def _resolve_target(self, target) -> str:
        """Resolve target: string → use as-is; ClassRef → switch detector first."""
        if isinstance(target, ClassRef):
            self._dsl.set_model(target.model_name)
            self._dsl.set_classes(target.class_name)
            return target.class_name
        return target if target else self._dsl.target

    def _send(self, cmd: str, **fields):
        return self._dsl._send(cmd, **fields)

    # ================================================================== #
    #  Vision verbs                                                        #
    # ================================================================== #

    def find(self, target=None, *,
             camera=None,
             move: str = 'still',
             gain: float = 25.0,
             yaw_rate_pct: float = 22.0,
             timeout: float = 25.0,
             stale_after: float = 0.0):
        """Search until target is detected. Optionally move while searching.

        Parameters
        ----------
        move : str
            What the AUV does while searching:
            'still'     -- wait in place (good when location is roughly known)
            'forward'   -- drive forward (search a pool lane)
            'yaw_right' -- rotate right
            'yaw_left'  -- rotate left
            'arc'       -- forward + yaw simultaneously
        gain : float
            Forward or arc drive thrust % (used when move='forward'/'arc').
        yaw_rate_pct : float
            Yaw rate % during rotational sweep (move='yaw_right'/'yaw_left'/'arc').
        timeout : float
            Abort with MoveFailed after this many seconds.
        stale_after : float
            0.0 = use live ROS param (vision.stale_after, default 1.5 s).
        """
        move_key = (move or 'still').strip().lower()
        if move_key not in _MOVE_TO_DRIVER:
            raise ValueError(
                f"vision.find: unknown move={move!r}; "
                f"valid: {sorted(_MOVE_TO_DRIVER)}")
        target_str = self._resolve_target(target)
        return self._send(
            'vision_acquire',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            target_name=_MOVE_TO_DRIVER[move_key],
            timeout=float(timeout),
            gain=float(gain),
            yaw_rate_pct=float(yaw_rate_pct),
            stale_after=float(stale_after))

    def turn(self, target=None, *,
             camera=None,
             duration: float = 8.0,
             **overrides):
        """Yaw (Ch4) left/right to horizontally centre target in frame.

        Exits when target is within deadband for 0.1 s, or duration expires.
        """
        target_str = self._resolve_target(target)
        return self._send(
            'vision_align_yaw',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            duration=float(duration),
            **overrides)

    def slide(self, target=None, *,
              camera=None,
              duration: float = 8.0,
              **overrides):
        """Slide laterally (Ch6) to horizontally centre target without changing heading.

        Best used when heading should stay fixed (e.g. gate-aligned approach).
        """
        target_str = self._resolve_target(target)
        return self._send(
            'vision_align_lat',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            duration=float(duration),
            **overrides)

    def hover(self, target=None, *,
              camera=None,
              duration: float = 8.0,
              **overrides):
        """Nudge depth setpoint to vertically centre target in frame.

        Adjusts the ArduSub depth setpoint in small steps so the autopilot
        moves smoothly. Use depth_anchor_frac=0.2 for tall objects (people,
        poles) to align toward the top of the bbox instead of the centre.
        """
        target_str = self._resolve_target(target)
        return self._send(
            'vision_align_depth',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            duration=float(duration),
            **overrides)

    def approach(self, target=None, *,
                 camera=None,
                 dist: float = 0.55,
                 metric: str = 'height',
                 duration: float = 12.0,
                 pass_at: float = 0.0,
                 pass_at_gain: float = 50.0,
                 **overrides):
        """Drive forward/back to reach standoff distance measured by metric.

        Parameters
        ----------
        dist : float
            Target bbox fraction to stop at (0..1).
            0.20 = far away, 0.55 = medium, 0.80 = very close.
        metric : str
            Distance proxy to use:
            'height'   -- bbox height fraction (default; tall objects: buoy, pole, flare)
            'width'    -- bbox width fraction (wide horizontal objects)
            'area'     -- sqrt(w × h); robust for gates and variable-shape objects
            'diagonal' -- normalised diagonal; best all-rounder
        pass_at : float
            Position-lock trigger (0.0 = disabled). Once size >= pass_at, the
            sub freezes lateral + depth corrections and drives straight at
            pass_at_gain% until duration expires. Use for a committed gate pass.
        pass_at_gain : float
            Forward thrust % during pass-through phase (default 50%).
        """
        target_str = self._resolve_target(target)
        return self._send(
            'vision_hold_distance',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            target_bbox_h_frac=float(dist),
            distance_metric=metric,
            pass_at=float(pass_at),
            pass_at_gain=float(pass_at_gain),
            duration=float(duration),
            **overrides)

    def home(self, target=None, *,
             camera=None,
             yaw: bool = True,
             lat: bool = False,
             depth: bool = False,
             forward: bool = False,
             dist: float = 0.55,
             metric: str = 'height',
             gate_guard: bool = False,
             gate_guard_min_w_frac: float = 0.35,
             pass_at: float = 0.0,
             pass_at_gain: float = 50.0,
             duration: float = 15.0,
             **overrides):
        """Home in on target using selected axes simultaneously.

        Boolean flags select which axes are active. All active axes run in the
        same 20 Hz loop and fire in one ArduSub command packet. The verb exits
        when all active axes are centred within deadband, or duration expires.

        Parameters
        ----------
        yaw : bool
            Steer heading (Ch4) to horizontally centre. Default True.
        lat : bool
            Slide laterally (Ch6) to horizontally centre without yawing. Default False.
        depth : bool
            Nudge depth setpoint to vertically centre. Default False.
        forward : bool
            Approach/back off (Ch5) to reach `dist` fraction. Default False.
        dist : float
            Bbox fraction to stop at when forward=True.
        metric : str
            Distance proxy: 'height' | 'width' | 'area' | 'diagonal'.
        gate_guard : bool
            Experimental: suppresses forward thrust when the gate bbox appears
            angled (w_frac/h_frac < gate_guard_min_w_frac). Lateral + yaw
            corrections keep running to realign the sub to perpendicular.
        gate_guard_min_w_frac : float
            Aspect ratio threshold for gate_guard (default 0.35 = pool-calibrated).
        pass_at : float
            Commit to straight-through pass once size >= this fraction (0 = disabled).
        pass_at_gain : float
            Forward % during the committed pass phase (default 50%).
        duration : float
            Time limit in seconds.

        Examples::

            # Gate pass: yaw + lateral + gate_guard
            duburi.vision.home(target=m.gate.gate, yaw=True, lat=True,
                               gate_guard=True, duration=8)

            # Full 3-axis lock on flare
            duburi.vision.home(target=m.gate.flare, yaw=True, depth=True, forward=True,
                               dist=0.38, metric='height', duration=20,
                               on_lost='hold')

            # Committed pass: align then drive through
            duburi.vision.home(target=m.gate.gate, yaw=True, lat=True, forward=True,
                               dist=0.42, metric='area',
                               gate_guard=True, pass_at=0.35, pass_at_gain=55,
                               duration=12)
        """
        axes_parts = [a for a, flag in
                      [('yaw', yaw), ('forward', forward),
                       ('lat', lat), ('depth', depth)]
                      if flag]
        if not axes_parts:
            raise ValueError(
                "vision.home: at least one axis must be True "
                "(yaw, lat, depth, or forward)")
        target_str = self._resolve_target(target)
        return self._send(
            'vision_align_3d',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            axes=','.join(axes_parts),
            target_bbox_h_frac=float(dist),
            distance_metric=metric,
            gate_guard=bool(gate_guard),
            gate_guard_min_w_frac=float(gate_guard_min_w_frac),
            pass_at=float(pass_at),
            pass_at_gain=float(pass_at_gain),
            duration=float(duration),
            **overrides)

    def track(self, target=None, *,
              camera=None,
              yaw: bool = True,
              forward: bool = True,
              lat: bool = False,
              depth: bool = False,
              dist: float = 0.55,
              duration: float = 60.0,
              **overrides):
        """Track target continuously until duration expires.

        Never exits on settle — runs until duration regardless of how well
        centred the target is. Good for following a moving target (swimmer,
        diver, moving buoy) for a fixed number of seconds.

        Parameters
        ----------
        yaw, forward, lat, depth : bool
            Which axes to use. Defaults: yaw=True, forward=True.
        dist : float
            Standoff distance fraction for the forward axis.
        duration : float
            How long to track (seconds).
        """
        axes_parts = [a for a, flag in
                      [('yaw', yaw), ('forward', forward),
                       ('lat', lat), ('depth', depth)]
                      if flag]
        if not axes_parts:
            raise ValueError(
                "vision.track: at least one axis must be True")
        target_str = self._resolve_target(target)
        return self._send(
            'vision_align_3d',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            axes=','.join(axes_parts),
            target_bbox_h_frac=float(dist),
            lock_mode='follow',
            duration=float(duration),
            **overrides)

    def scan(self, target=None, *,
             camera=None,
             step: float = 20.0,
             speed: float = 40.0,
             dwell: float = 1.5,
             duration: float = 60.0,
             start_yaw: float = 0.0,
             **overrides):
        """Rotate on the spot searching for target. Uses POSHOLD for position hold.

        Takes incremental yaw steps, pausing at each position to observe.
        Exits immediately when target is detected. Completes a full orbit
        if duration allows without finding the target.

        Parameters
        ----------
        step : float
            Yaw increment in degrees per look (positive=CW/right, negative=CCW/left).
            20.0 gives 18 stops for a full 360° orbit.
        speed : float
            Turn speed percent (0–100).
        dwell : float
            Seconds to observe at each yaw stop.
        duration : float
            Total time budget in seconds.
        start_yaw : float
            Snap to this heading before starting (0.0 = use current heading).

        Example::

            # Orbit right, 15° steps, stop when gate is found
            result = duburi.vision.scan(target=m.gate.gate, step=15, duration=90)
            if result.success:
                duburi.vision.home(target=m.gate.gate, yaw=True, duration=10)
        """
        target_str = self._resolve_target(target)
        return self._send(
            'look_around',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            duration=float(duration),
            gain=float(speed),
            yaw_rate_pct=float(step),
            settle=float(dwell),
            target=float(start_yaw),
            **overrides)
