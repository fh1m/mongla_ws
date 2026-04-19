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

  Closed-loop vision verbs mirror the SAME axis names under
  `duburi.vision`, so it is impossible to confuse them with their
  open-loop twin:

      duburi.vision.find(target)               # search until target seen
      duburi.vision.yaw(target)                # centre on target via Ch4
      duburi.vision.lateral(target)            # centre on target via Ch6
      duburi.vision.depth(target)              # centre on target via depth setpoint
      duburi.vision.forward(target,            # close to / back from target via Ch5
                            distance=0.55)     # bbox-height fraction = distance proxy
      duburi.vision.lock(target,               # multi-axis simultaneous hold
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
blocking RPC that prints ONE line on completion -- mission files do not
need a `_step` wrapper, banner code, or per-call logging. Just write
the verbs in order:

    def run(duburi, log):
        duburi.arm()
        duburi.set_depth(-0.5)
        duburi.vision.find()
        duburi.vision.lock(axes='yaw,forward', distance=0.55, duration=10)
        duburi.set_depth(0.0)
        duburi.disarm()

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


def _format_outcome(cmd: str, result) -> str:
    return (f'  {cmd:<22s} final={result.final_value:+.3f} '
            f'err={result.error_value:+.3f}  ({result.message})')


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
        self.vision = _VisionAxes(self)

    # ================================================================== #
    #  Single send + log helper -- every verb funnels through here        #
    # ================================================================== #

    def _send(self, cmd: str, **fields):
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
_FIND_SWEEP_TO_DRIVE = {
    'left':    'yaw_left',
    'right':   'yaw_right',
    'forward': 'move_forward',
    'arc':     'arc',
    'still':   '',
    'none':    '',
}


class _VisionAxes:
    """duburi.vision.* -- same axis names as control verbs, but vision-driven.

    All verbs send the same field shape to `/duburi/move`; only `cmd`
    differs. Sticky camera + target come from the parent `DuburiMission`
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
        """Search until at least one fresh detection of `target` arrives.

        sweep: 'right' (default) | 'left' | 'forward' | 'arc' | 'still'
        """
        sweep_key = (sweep or 'still').strip().lower()
        if sweep_key not in _FIND_SWEEP_TO_DRIVE:
            raise ValueError(
                f"vision.find: unknown sweep={sweep!r}; "
                f"valid: {sorted(_FIND_SWEEP_TO_DRIVE)}")
        return self._send(
            'vision_acquire',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            target_name=_FIND_SWEEP_TO_DRIVE[sweep_key],
            timeout=float(timeout),
            gain=float(gain),
            yaw_rate_pct=float(yaw_rate_pct),
            stale_after=float(stale_after))

    def yaw(self, target: str | None = None, *,
            camera: str | None = None,
            duration: float = 8.0,
            **overrides):
        """Centre the largest `target` bbox horizontally via Ch4 (yaw).

        Override knobs (only pass if you want to PIN, not use the live
        ROS-param default): `deadband`, `kp_yaw`, `on_lost`,
        `stale_after`.
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
        """Centre via lateral strafe (Ch6). Same overrides as `yaw` plus `kp_lat`."""
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
        """Centre vertically via incremental depth setpoint nudges. Overrides include `kp_depth`."""
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
        """Drive Ch5 so `target`'s bbox-height fraction matches `distance`.

        `distance` IS the bbox height as a fraction of image height (0..1).
        Bigger fraction = closer target. Override knobs: `kp_forward`,
        `deadband`, `on_lost`, `stale_after`.
        """
        return self._send(
            'vision_hold_distance',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            target_bbox_h_frac=float(distance),
            duration=float(duration),
            **overrides)

    def lock(self, target: str | None = None, *,
             camera: str | None = None,
             axes: str = 'yaw,forward',
             distance: float = 0.55,
             duration: float = 15.0,
             **overrides):
        """Hold multiple axes simultaneously on `target`.

        `axes` is a CSV: any subset of 'yaw,lat,depth,forward'. The same
        overrides apply for every axis (`kp_yaw`, `kp_lat`, `kp_depth`,
        `kp_forward`, `deadband`, `target_bbox_h_frac`, `on_lost`,
        `stale_after`, `visual_pid`).
        """
        return self._send(
            'vision_align_3d',
            camera=self._resolved_camera(camera),
            target_class=self._resolved_target(target),
            axes=str(axes),
            target_bbox_h_frac=float(distance),
            duration=float(duration),
            **overrides)
