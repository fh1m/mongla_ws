"""vision_dsl -- duburi.vision.* closed-loop sub-namespace.

Extracted from duburi_dsl.py so motion verbs (DuburiMission) and
vision verbs (_VisionDSL) can evolve independently. DuburiMission
instantiates _VisionDSL as ``self.vision`` and passes itself in.
"""
from __future__ import annotations

import time as _time
from typing import TYPE_CHECKING

import rclpy

from .model_context import ClassRef

if TYPE_CHECKING:
    from .duburi_dsl import DuburiMission

# Maps the `move=` param on `find()` to the internal drive verb.
_MOVE_TO_DRIVER: dict[str, str] = {
    'still':     '',
    'none':      '',
    'forward':   'move_forward',
    'yaw_right': 'yaw_right',
    'yaw_left':  'yaw_left',
    'arc':       'arc',
}


class _VisionDSL:
    """duburi.vision.* -- the closed-loop sub-namespace.

    Every verb is a blocking call. On completion it prints one outcome line.
    Gains (kp_*, deadband, on_lost, stale_after) fall back to live ROS params
    when not passed -- use `ros2 param set /duburi_manager vision.kp_yaw ...`
    to tune between runs without touching mission code.
    """

    def __init__(self, mission: 'DuburiMission'):
        self._dsl = mission

    # ---- target / camera resolution ----------------------------------- #

    def _resolve_camera(self, camera) -> str:
        return camera if camera else self._dsl.camera

    def _resolve_target(self, target) -> str:
        """Resolve target: string -> use as-is; ClassRef -> switch detector first."""
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
            'area'     -- sqrt(w x h); robust for gates and variable-shape objects
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

    def vis_approach(self, target=None, *,
                     camera=None,
                     threshold: float = 0.65,
                     duration: float = 30.0,
                     lock_mode: str = '',
                     **overrides):
        """Drive forward using monocular depth from depth_estimation_node.

        Parameters
        ----------
        threshold : float
            Target vis_range to stop at (0=far, 1=close). 0.65 ~= gate-pass range.
        duration : float
            Time budget in seconds.

        Requires depth_estimation_node running on the same camera.
        Fallback: if depth_estimation_node not running, vis_range = bbox-area proxy
        (same as approach(metric='area')) -- so the verb degrades gracefully.

        Example::

            duburi.vision.vis_approach(target='gate', threshold=0.65, duration=20)
        """
        target_str = self._resolve_target(target)
        return self._send(
            'vis_approach',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            target_vis_range=float(threshold),
            duration=float(duration),
            lock_mode=lock_mode,
            **overrides)

    def vis_range(self, target=None, *, camera=None, stale_after: float = 1.0) -> float:
        """Return the latest vis_range estimate for target_class (0=far, 1=close).

        Non-blocking cache check -- reads the most recent Float32MultiArray published
        by depth_estimation_node. Returns 0.0 when no data is available.

        Example::

            r = duburi.vision.vis_range('gate')
            if r > 0.6:
                duburi.vision.vis_approach('gate', threshold=0.8)
        """
        if isinstance(target, ClassRef):
            target = target.class_name
        cam = self._resolve_camera(camera) or self._dsl.camera

        if not hasattr(self, '_vr_subs'):
            self._vr_subs:  dict = {}
            self._vr_cache: dict = {}

        if cam not in self._vr_subs:
            from std_msgs.msg import Float32MultiArray as _FA
            topic = f'/duburi/vision/{cam}/vis_range'
            sub   = self._dsl.client.node.create_subscription(
                _FA, topic,
                lambda msg, c=cam: self._on_vis_range(c, msg), 10)
            self._vr_subs[cam] = sub

        rclpy.spin_once(self._dsl.client.node, timeout_sec=0.05)
        entry = self._vr_cache.get(cam)
        if entry is None:
            return 0.0
        stamp, vals = entry
        if _time.monotonic() - stamp > stale_after:
            return 0.0
        return float(vals[0]) if vals else 0.0

    def _on_vis_range(self, camera: str, msg) -> None:
        if not hasattr(self, '_vr_cache'):
            self._vr_cache: dict = {}
        self._vr_cache[camera] = (_time.monotonic(), list(msg.data))

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
             offset_x: float = 0.0,
             offset_y: float = 0.0,
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
            offset_x=float(offset_x),
            offset_y=float(offset_y),
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

        Never exits on settle -- runs until duration regardless of how well
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

    def hold(self, target=None, *,
             camera=None,
             yaw: bool = True,
             lat: bool = True,
             depth: bool = False,
             forward: bool = False,
             duration: float = 60.0,
             offset_x: float = 0.0,
             offset_y: float = 0.0,
             **overrides):
        """Maintain PID position-lock on target for duration without exiting on settle.

        Unlike home() which exits once aligned, hold() keeps running the PID
        loops for the full duration. Use for timed holds, waiting for external
        triggers, or as the lock phase before a fire sequence.

        Default axes (yaw + lat) maintain 2D centering -- add depth=True to
        hold a specific vertical position against the bbox, or forward=True to
        maintain standoff distance.

        The trilogy:
          home()             -- align, exit when settled
          hold()             -- align, maintain indefinitely (lock_mode='follow')
          vision_lock_fire() -- align, maintain, fire when stable
        """
        axes_parts = [a for a, flag in
                      [('yaw', yaw), ('lat', lat),
                       ('depth', depth), ('forward', forward)]
                      if flag]
        if not axes_parts:
            raise ValueError("vision.hold: at least one axis must be True")
        target_str = self._resolve_target(target)
        return self._send(
            'vision_align_3d',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            axes=','.join(axes_parts),
            lock_mode='follow',
            duration=float(duration),
            offset_x=float(offset_x),
            offset_y=float(offset_y),
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
            20.0 gives 18 stops for a full 360 orbit.
        speed : float
            Turn speed percent (0-100).
        dwell : float
            Seconds to observe at each yaw stop.
        duration : float
            Total time budget in seconds.
        start_yaw : float
            Snap to this heading before starting (0.0 = use current heading).

        Example::

            # Orbit right, 15 deg steps, stop when gate is found
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

    def vision_lock_fire(self, target=None, *,
                         camera=None,
                         yaw: bool = True,
                         lat: bool = True,
                         depth: bool = True,
                         forward: bool = False,
                         dist: float = 0.0,
                         metric: str = '',
                         stable_lock_s: float = 3.0,
                         max_attempts: int = 3,
                         attempt_timeout: float = 15.0,
                         fire_channel: int = 1,
                         offset_x: float = 0.0,
                         offset_y: float = 0.0,
                         duration: float = 60.0,
                         **overrides):
        """Lock 3D on target, hold stable, fire via ESP32 serial.

        Aligns on the selected axes; once all axes stay within deadband for
        ``stable_lock_s`` seconds ``fire_channel`` is sent over serial.
        Retries up to ``max_attempts``; fires at last pose as fallback.

        ``fire_channel``: 1/2 = torpedo, 3/4 = dropper. 0 = log-only stub.

        Example::

            duburi.vision.vision_lock_fire(
                target=m.torpedo.hole,
                fire_channel=1,
                yaw=True, lat=True, depth=True,
                stable_lock_s=3.0, max_attempts=2,
                duration=60)
        """
        axes_parts = [a for a, flag in
                      [('yaw', yaw), ('lat', lat),
                       ('depth', depth), ('forward', forward)]
                      if flag]
        if not axes_parts:
            raise ValueError(
                "vision.vision_lock_fire: at least one axis must be True")
        target_str = self._resolve_target(target)
        return self._send(
            'vision_lock_fire',
            camera=self._resolve_camera(camera),
            target_class=target_str,
            axes=','.join(axes_parts),
            target_bbox_h_frac=float(dist),
            distance_metric=metric,
            stable_lock_s=float(stable_lock_s),
            max_attempts=float(max_attempts),
            attempt_timeout=float(attempt_timeout),
            fire_channel=float(fire_channel),
            offset_x=float(offset_x),
            offset_y=float(offset_y),
            duration=float(duration),
            **overrides)
