#!/usr/bin/env python3
"""DuburiMission -- the human-readable mission DSL.

Two namespaces, one mental model:

  Open-loop motion verbs sit directly on `duburi`:

      duburi.arm() / duburi.disarm()
      duburi.set_depth(meters)
      duburi.move_forward(seconds, gain=60)
      duburi.move_back(seconds)
      duburi.move_left(seconds) / duburi.move_right(seconds)
      duburi.yaw_left(degrees) / duburi.yaw_right(degrees)
      duburi.arc(seconds, gain=50, yaw_rate_pct=30)
      duburi.lock_heading(degrees)  / duburi.release_heading()
      duburi.pause(seconds) / duburi.stop()

  Depth is held automatically by ArduSub's onboard ALT_HOLD whenever
  the mode is ALT_HOLD/POSHOLD/GUIDED. ``set_depth`` engages the mode
  and drives to the target; once it returns the autopilot keeps the
  depth indefinitely without a Python streamer.

  Closed-loop vision verbs mirror the SAME axis names under
  `duburi.vision`, so it is impossible to confuse them with their
  open-loop twin:

      duburi.vision.find(target)               # drive + watch until target seen
      duburi.vision.yaw(target)                # steer left/right toward horizontal centre
      duburi.vision.lateral(target)            # strafe left/right toward horizontal centre
      duburi.vision.depth(target)              # nudge depth up/down toward vertical centre
      duburi.vision.forward(target,            # approach / back off by bbox fill fraction
                            distance=0.55)
      duburi.vision.lock(target,               # all of the above, simultaneously
                         axes='yaw,forward',
                         distance=0.55)

Same axis isolation contract as control: every verb owns one MAVLink
channel; `vision.lock` owns the union of the axes listed in `axes`.

Sticky context
--------------
`duburi.camera` and `duburi.target` are set ONCE per mission (defaults
'laptop' and 'person'). Every vision verb reads them; pass `target=...`
or `camera=...` to override per call.

Tunable from ROS during pool tests
----------------------------------
Vision gains (kp_yaw, kp_lat, kp_depth, kp_forward), `deadband`,
`target_bbox_h_frac`, `stale_after`, and `on_lost` all have ROS-param
fallbacks declared on `auv_manager_node`. When a vision verb leaves
those fields at their rosidl zero default, the manager substitutes the
live `vision.*` ROS-param value -- so operators can `ros2 param set
/duburi_manager vision.kp_yaw 80.0` between goals without touching
mission code.

Mission-author contract
-----------------------
Every `duburi.<verb>(...)` and `duburi.vision.<verb>(...)` call is a
blocking call that prints ONE result line on completion -- mission files
do not need a `_step` wrapper, banner code, or per-call logging. Just
write the verbs in order:

    def run(duburi, log):
        duburi.arm()
        duburi.set_depth(-0.5)
        duburi.vision.find()
        duburi.vision.lock(axes='yaw,forward', distance=0.55, duration=10)
        duburi.set_depth(0.0)
        duburi.disarm()

Quick reference -- what each verb does at the hardware level
------------------------------------------------------------
OPEN-LOOP VERBS (no sensor feedback during the move):
  arm()               Power the thrusters. Wait for autopilot confirmation.
  disarm()            Cut thruster power.
  set_mode(name)      Switch ArduSub flight mode (e.g. 'ALT_HOLD', 'MANUAL').
  set_depth(m)        Drive to m metres (negative = below surface). The
                      autopilot holds that depth until the next set_depth.
  move_forward(s)     Push the forward thrust channel at gain% for s seconds.
  move_back(s)        Same, reverse.
  move_left(s)        Push the lateral thrust channel at gain% for s seconds.
  move_right(s)       Same, opposite direction.
  yaw_left(deg)       Spin left N degrees via the heading PID. Returns when done.
  yaw_right(deg)      Same, right.
  arc(s, yaw_rate_pct)  Forward thrust + continuous yaw simultaneously (curved path).
  lock_heading(deg)   Start a background heading-hold thread. 0.0 = current heading.
  release_heading()   Stop the background heading-hold thread.
  pause(s)            Send neutral signals for s seconds (hold depth + heading).
  stop()              Send neutral signals once (immediate hold).

CLOSED-LOOP VISION VERBS (use camera feedback each step):
  vision.find(target, sweep)
      Rotate slowly while watching the camera. Return once the target
      appears. Fails after timeout seconds if nothing is seen.
  vision.yaw(target, duration)
      Steer left/right to bring the target's horizontal centre into the
      middle of the frame. Uses heading PID. Exits when centred or time up.
  vision.lateral(target, duration)
      Strafe left/right to centre the target horizontally. Uses the lateral
      thrust channel directly (does not change heading).
  vision.depth(target, duration)
      Nudge depth setpoint up/down to bring the target to the vertical
      centre of the frame. Exits when centred or time up.
  vision.forward(target, distance, duration)
      Approach or back off until the target's bounding box fills `distance`
      fraction of the frame height. 0.55 = stop when bbox is 55% of frame.
  vision.lock(target, axes, distance, duration)
      Run multiple axes at once. All axes run in the same loop and the
      vehicle settles when ALL are centred at the same time.
      axes can be any combination of: 'yaw', 'lat', 'depth', 'forward'

DETECTOR CONTROL (within a mission):
  duburi.set_classes('gate')          # only publish gate detections
  duburi.set_classes('gate,flare')    # publish gate + flare detections
  duburi.set_classes('')              # publish ALL model classes
  # Takes effect next inference frame. Model reload requires node restart:
  #   ros2 launch duburi_vision cameras_.launch.py model:=gate_flare_medium_100ep

TETHER REMOVAL COUNTDOWN (before mission start):
  duburi.countdown(10)        # 10-second window to disconnect tether
  duburi.countdown(15, message="Starting run. Stand clear.")

LIVE TUNING (between runs, no rebuild needed):
  ros2 param set /duburi_manager vision.kp_yaw 80.0       # faster/slower yaw
  ros2 param set /duburi_manager vision.kp_forward 150.0  # faster/slower approach
  ros2 param set /duburi_manager vision.deadband 0.10     # tighter centering

Why a class (not just module-level functions)
---------------------------------------------
We need to carry sticky context (camera, target) and we want
`__getattr__` so missions never get blocked from raw verbs:
`duburi.move_forward(...)` falls through to the underlying client when
the wrapper hasn't defined it. The class is named `DuburiMission` so it
doesn't shadow `duburi_control.duburi.Duburi` (the facade); the
variable in every mission is just `duburi`.
"""

from __future__ import annotations

import subprocess
import sys
import time as _time


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
        The blocking action client. The DSL is a thin wrapper -- no extra
        threading, no extra state. Every verb still goes through one
        `client.send(...)` call so the action server's single-flight lock
        keeps every command serialised.
    log : rclpy logger
        Anything with `.info(...)`. Used to print one outcome line per
        verb call so mission files stay free of logging boilerplate.
    camera, target : str
        Sticky defaults. Override per call if needed.
    """

    def __init__(self, client, log, *, camera: str = 'laptop',
                 target: str = 'person'):
        self.client = client
        self.log    = log
        self.camera = camera
        self.target = target
        self.vision = _VisionDSL(self)

    # ================================================================== #
    #  Single send + log helper -- every verb funnels through here        #
    # ================================================================== #

    def _send(self, cmd: str, **fields):
        fields = {k: _to_float(v) for k, v in fields.items()}
        result = self.client.send(cmd, **fields)
        self.log.info(_format_outcome(cmd, result))
        return result

    # ================================================================== #
    #  Power / mode                                                       #
    # ================================================================== #

    def arm(self, *, timeout: float = 15.0):
        return self._send('arm', timeout=timeout)

    def disarm(self, *, timeout: float = 20.0):
        return self._send('disarm', timeout=timeout)

    def set_mode(self, name: str, *, timeout: float = 8.0):
        return self._send('set_mode', target_name=name, timeout=timeout)

    # ================================================================== #
    #  Stop / pause                                                       #
    # ================================================================== #

    def stop(self):
        return self._send('stop')

    def head(self) -> float:
        """Return the live heading (degrees) at the moment this call executes.

            h = duburi.head()
            duburi.lock_heading(target=h)
        """
        result = self._send('head')
        return result.final_value if result is not None else 0.0

    def pause(self, seconds: float):
        return self._send('pause', duration=float(seconds))

    # ================================================================== #
    #  Open-loop motion -- names match COMMANDS keys exactly              #
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
    #  DVL                                                                #
    # ================================================================== #

    def dvl_connect(self):
        """Connect Nortek Nucleus 1000 DVL over TCP. Must be called before
        any move_*_dist command when yaw_source is 'dvl' or 'nucleus_dvl'."""
        return self._send('dvl_connect')

    def move_forward_dist(self, metres: float, *, gain: float = 60.0,
                          tolerance: float = 0.1, settle: float = 0.0):
        """Drive forward `metres` metres using DVL closed-loop position feedback.

        Falls back to open-loop timed estimate if DVL position is unavailable.
        Requires dvl_connect() first when yaw_source='dvl'.
        """
        return self._send('move_forward_dist',
                          distance_m=float(metres),
                          gain=gain,
                          dvl_tolerance=tolerance,
                          settle=settle)

    def move_lateral_dist(self, metres: float, *, gain: float = 36.0,
                          tolerance: float = 0.1, settle: float = 0.0):
        """Strafe `metres` metres (positive=right, negative=left) using DVL.

        Falls back to open-loop timed estimate if DVL position is unavailable.
        Requires dvl_connect() first when yaw_source='dvl'.
        """
        return self._send('move_lateral_dist',
                          distance_m=float(metres),
                          gain=gain,
                          dvl_tolerance=tolerance,
                          settle=settle)

    # ================================================================== #
    #  Vision detector control                                            #
    # ================================================================== #

    def set_classes(self, classes: str | list, *,
                    node: str = '/duburi_detector') -> None:
        """Switch the detector's class filter without restarting the node.

        Changes take effect on the next inference frame (~16 ms on GPU).

        Parameters
        ----------
        classes : str or list
            CSV string or list of class names to keep.
            Examples::
                duburi.set_classes('gate')
                duburi.set_classes('gate,flare')
                duburi.set_classes(['gate', 'flare'])
                duburi.set_classes('')          # keep ALL classes
        node : str
            ROS2 node name. Override if you renamed the detector.

        Use case: gate+flare combined model — start with 'gate' to pass
        the gate, then switch to 'flare' for the flare orbit phase.
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
            self.log.info(
                f"[DSL  ] detector classes → {classes_str!r}")

    # ================================================================== #
    #  Mission start / countdown                                          #
    # ================================================================== #

    def countdown(self, seconds: int = 10, *,
                  message: str = "Wire removed  --  Duburi is now autonomous. Good luck."):
        """Print an aesthetic tether-removal countdown and return.

        The operator should disconnect the tether during this window.
        All code AFTER the countdown runs without any tether or external
        connection -- Jetson, Pi, DVL, and Pixhawk are all on-board.

        Parameters
        ----------
        seconds : int
            Countdown duration. 10 s is enough for a quick disconnect.
        message : str
            Banner text shown at T=0.

        Example::

            def run(duburi, log):
                duburi.countdown(10)   # disconnect tether now
                duburi.arm()
                ...
        """
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
            sys.stdout.write(
                f'\r  T-{remaining:3d}s  [{bar}]  ')
            sys.stdout.flush()
            _time.sleep(1)

        sys.stdout.write('\r' + ' ' * 60 + '\r')
        sys.stdout.flush()
        print()
        _box([
            '',
            message,
            '',
        ])
        print()

    # ================================================================== #
    #  Escape hatch -- unknown verbs fall through to the raw client BUT   #
    #  still get the one-line outcome log.                                #
    # ================================================================== #

    def __getattr__(self, name: str):
        # Anything we did not define as a verb falls through to the raw
        # client. So `duburi.send('whatever', **fields)` works, and any
        # future COMMANDS row is reachable as `duburi.<name>(...)` even
        # before someone wires it explicitly above. We wrap the dispatch
        # so the outcome line still gets printed.
        if name == 'send':
            def _send_with_log(cmd, **fields):
                return self._send(cmd, **fields)
            return _send_with_log

        attr = getattr(self.client, name)
        if not callable(attr):
            return attr

        def _wrapped(*args, **kwargs):
            result = attr(*args, **kwargs)
            # Only log if the result looks like a Move.Result (has the
            # fields we care about). Otherwise (e.g. wait_for_connection)
            # just pass it through silently.
            if hasattr(result, 'final_value') and hasattr(result, 'message'):
                self.log.info(_format_outcome(name, result))
            return result

        return _wrapped


# ---------------------------------------------------------------------- #
#  duburi.vision -- closed-loop sub-namespace                            #
# ---------------------------------------------------------------------- #

# Map the human "sweep direction" knob on `vision.find` to the underlying
# `target_name` field that motion_vision.vision_acquire understands.
_FIND_SWEEP_DRIVERS = {
    'left':    'yaw_left',
    'right':   'yaw_right',
    'forward': 'move_forward',
    'arc':     'arc',
    'still':   '',
    'none':    '',
}


class _VisionDSL:
    """duburi.vision.* -- the closed-loop sub-namespace.

    Same axis names as control verbs, but vision-driven. All verbs send
    the same field shape to `/duburi/move`; only `cmd` differs.
    Sticky camera + target come from the parent `DuburiMission`
    instance; pass them explicitly to override per call.

    Pass `kp_*`, `deadband`, `on_lost`, `stale_after`, etc. as kwargs
    only when you want to override the live ROS-param defaults declared
    on `auv_manager_node`. Otherwise leave them out and let the manager
    use whatever `vision.*` value is set on the deck.

    Outcome is auto-logged by the parent `DuburiMission._send` helper --
    mission files just call `duburi.vision.<verb>(...)` and read one
    line of feedback per call.
    """

    def __init__(self, mission: 'DuburiMission'):
        self._mission = mission

    # ---- helpers ------------------------------------------------------ #

    def _resolved_camera(self, camera):
        return camera if camera else self._mission.camera

    def _resolved_target(self, target):
        return target if target else self._mission.target

    def _send(self, cmd: str, **fields):
        # Forward through DuburiMission._send so we get the outcome line
        # for free. Vision goals have rosidl-zero defaults for every
        # override (kp_*, deadband, target_bbox_h_frac, stale_after,
        # on_lost). The manager treats those zeros as "use the live ROS
        # param value". So callers only pass what they want to PIN
        # per-call.
        return self._mission._send(cmd, **fields)

    # ---- verbs (mirror the control verbs, vision-driven) ------------- #

    def find(self, target: str | None = None, *,
             camera: str | None = None,
             sweep: str = 'right',
             timeout: float = 25.0,
             gain: float = 25.0,
             yaw_rate_pct: float = 22.0,
             stale_after: float = 0.0):
        """Rotate slowly while watching the camera. Return once the target is seen.

        sweep     -- which way to rotate while searching: 'right' (default),
                     'left', 'forward' (drive ahead), 'arc', 'still' (wait in place).
        timeout   -- give up and abort the mission after this many seconds.
        gain      -- forward thrust percentage used during 'forward' or 'arc' sweep.
        yaw_rate_pct -- how fast to rotate during the sweep (default 22% yaw stick).
        stale_after  -- how old a detection can be before it is ignored (seconds).
                        0.0 = use the live ROS param (vision.stale_after, default 1.5 s).

        Tip: if the vehicle rotates past the target before detecting it,
        lower yaw_rate_pct. If detection is too slow to confirm, lower stale_after.
        """
        sweep_key = (sweep or 'still').strip().lower()
        if sweep_key not in _FIND_SWEEP_DRIVERS:
            raise ValueError(
                f"vision.find: unknown sweep={sweep!r}; "
                f"valid: {sorted(_FIND_SWEEP_DRIVERS)}")
        return self._send(
            'vision_acquire',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            target_name=_FIND_SWEEP_DRIVERS[sweep_key],
            timeout=float(timeout),
            gain=float(gain),
            yaw_rate_pct=float(yaw_rate_pct),
            stale_after=float(stale_after))

    def yaw(self, target: str | None = None, *,
            camera: str | None = None,
            duration: float = 8.0,
            **overrides):
        """Steer left or right to bring the target horizontally into the centre of frame.

        Uses ArduSub's heading PID -- a smooth correction that gets
        smaller as it closes in. Only one axis moves; depth and distance
        are untouched.

        duration   -- stop after this many seconds even if not centred.
        kp_yaw     -- correction speed: 60 = 60% yaw at full-frame offset.
                      Raise → faster response; lower → less overshoot.
        deadband   -- how close to centre counts as "done" (fraction of frame width).
                      0.18 = within 18% of centre. Smaller = tighter.
        on_lost    -- 'fail' (abort if target disappears) or 'hold' (keep last heading).
        stale_after-- seconds before a detection is considered too old to use.

        Exits: target centred within deadband for 0.1 s, OR duration runs out.
        """
        return self._send(
            'vision_align_yaw',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            duration=float(duration),
            **overrides)

    def lateral(self, target: str | None = None, *,
                camera: str | None = None,
                duration: float = 8.0,
                **overrides):
        """Strafe left or right to bring the target horizontally into the centre of frame.

        Unlike vision.yaw, this does NOT change the vehicle's heading --
        it slides the body sideways. Best used when you need to face a
        fixed direction (e.g. aligned with a gate) and just shift position.

        kp_lat    -- strafe speed: 60 = 60% lateral thrust at full offset.
        All other tuning knobs are the same as vision.yaw.
        """
        return self._send(
            'vision_align_lat',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            duration=float(duration),
            **overrides)

    def depth(self, target: str | None = None, *,
              camera: str | None = None,
              duration: float = 8.0,
              **overrides):
        """Nudge depth up or down to bring the target to the vertical centre of frame.

        Instead of a direct thrust command, this adjusts the ArduSub depth
        setpoint in small steps. The autopilot's depth PID then drives the
        thrusters to reach the new setpoint. This keeps depth changes smooth
        and prevents fighting the autopilot.

        kp_depth  -- depth step size per correction (metres per unit error).
                     Default 0.05 m/step. Raise cautiously -- too large causes
                     overshooting and oscillation.
        deadband  -- vertical "close enough" zone (fraction of frame height).

        Tip: if the target has a very tall bounding box (person upright, pole),
        use depth_anchor_frac=0.2 to align toward the top of the box instead
        of the centre -- prevents the controller from stalling when the bbox
        centre is already near the middle of the frame.
        """
        return self._send(
            'vision_align_depth',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            duration=float(duration),
            **overrides)

    def forward(self, target: str | None = None, *,
                camera: str | None = None,
                distance: float = 0.55,
                duration: float = 12.0,
                **overrides):
        """Approach or back away to reach the requested standoff distance.

        `distance` is how much of the frame the target's bounding box should
        fill (by height), used as a distance proxy:
          0.20 = target is small / far away
          0.55 = medium close-up (default)
          0.80 = very close (nearly filling the frame)

        The vehicle drives forward if the target looks too small, and backs
        off if it looks too big. Exits when the bbox fraction matches
        `distance` within deadband, or `duration` seconds pass.

        kp_forward    -- approach speed: 200 = 50% thrust at 25% size error.
                         Raise for faster approach; lower to prevent overshoot.
        distance_metric -- how to measure "distance" from the bounding box:
                         'height' (default), 'area' (width × height), 'diagonal'.
                         Use 'area' for targets that are wider than tall (gates).
        on_lost       -- 'hold' is useful here to maintain the last thrust
                         if the detection drops briefly.
        """
        return self._send(
            'vision_hold_distance',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            target_bbox_h_frac=float(distance),
            duration=float(duration),
            **overrides)

    def follow(self, target: str | None = None, *,
               camera: str | None = None,
               axes: str = 'yaw,forward',
               distance: float = 0.55,
               duration: float = 60.0,
               **overrides):
        """Track the target continuously until duration expires -- never exits on settle.

        Same as vision.lock(..., lock_mode='follow'). Designed for following
        a moving target (swimmer, diver, moving buoy) for a fixed number of
        seconds without stopping the moment it's centred.

        For a stationary target you want to just monitor, set on_lost='hold'
        so brief detection gaps don't abort the tracking.

        All tuning knobs from vision.lock apply (kp_*, deadband, on_lost, etc.).
        """
        return self.lock(target, camera=camera, axes=axes,
                         distance=distance, duration=duration,
                         lock_mode='follow', **overrides)

    def lock(self, target: str | None = None, *,
             camera: str | None = None,
             axes: str = 'yaw,forward',
             distance: float = 0.55,
             duration: float = 15.0,
             **overrides):
        """Hold multiple axes at the same time on `target`.

        All requested axes run in the same loop and fire in the same
        ArduSub command packet. The verb exits when ALL axes are centred
        at the same time within deadband, or `duration` seconds pass.

        axes     -- which axes to control (comma-separated string):
                    'yaw'     = steer left/right to centre horizontally
                    'lat'     = strafe left/right to centre horizontally
                    'depth'   = nudge depth setpoint for vertical centre
                    'forward' = approach/back off to maintain `distance`
                    Examples: 'yaw,forward'  'yaw,lat,depth'  'yaw,lat,depth,forward'
        distance -- target bbox fill fraction (used only when 'forward' in axes).
        duration -- time limit in seconds. The lock continues until settled
                    OR this limit is hit.

        lock_mode (kwarg) -- controls exit behaviour:
                    'settle' (default) = exit as soon as all axes are centred.
                    'follow'  = never exit on settle; track indefinitely until
                                duration runs out. Good for moving targets.
                    'pursue'  = keep driving forward until the target fills
                                `distance` fraction (no backing off). Good for
                                torpedo / ramming approaches.

        kp_yaw, kp_lat, kp_depth, kp_forward -- per-axis speed gains.
        deadband  -- centering tolerance for all axes.
        on_lost   -- 'fail' (abort on lost target) or 'hold' (freeze setpoints).
        """
        return self._send(
            'vision_align_3d',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            axes=str(axes),
            target_bbox_h_frac=float(distance),
            duration=float(duration),
            **overrides)
