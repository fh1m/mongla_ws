#!/usr/bin/env python3
"""Vision verbs for the Duburi facade.

Split out of ``duburi.py`` so the motion-axis side of the facade
(forward/lateral/yaw/depth/lock/pause/stop) lives in one file and the
camera-driven side lives in another. Mixed into ``Duburi`` via
multiple inheritance:

    class Duburi(VisionVerbs):
        ...

The mixin only references attributes the base ``Duburi`` provides
(``self._command_scope(verb)``, ``self.pixhawk``, ``self.log``,
``self._writers()``, ``self._send_neutral_and_settle()``,
``self._ensure_alt_hold()``, ``self._current_depth()``,
``self._current_heading()``, ``self._make_result()``,
``self.vision_state_provider``, ``self._suspend_heading_lock()``,
``self._retarget_heading_lock()``) -- nothing rclpy-aware, identical
serialisation contract as the rest of the
facade. Each verb passes its own name into ``_command_scope`` so the
``[MAV <fn> cmd=vision_align_yaw] ...`` trace tag (see ``tracing.py``)
attributes every camera-driven MAVLink frame to its high-level verb.

All six verbs share the same pipeline:
  1. Resolve VisionState for ``camera`` (lazy preflight in the manager).
  2. Build a VisionGains from the operator-supplied ``kp_*``.
  3. Hand off to motion_vision.vision_track_axes / vision_acquire.
  4. Wrap the VisionTrackResult in a Move.Result.

The single-axis convenience verbs (``vision_align_yaw``, ``_lat``,
``_depth``) are just ``vision_align_3d`` with ``axes`` pinned -- one
canonical loop, no copy-paste.
"""

import time
from contextlib import nullcontext

from .motion_vision import (
    VisionGains, vision_acquire as run_vision_acquire,
    vision_track_axes,
)
from .motion_yaw import yaw_snap
from .pixhawk import Pixhawk


def _parse_axes(csv: str):
    """``'yaw,forward'`` -> ``{'yaw','forward'}``. Whitespace + case tolerant."""
    out = set()
    for token in (csv or '').split(','):
        name = token.strip().lower()
        if name:
            out.add(name)
    return out


class VisionVerbs:
    """Camera-driven verbs for the Duburi facade.

    Provides the ``vision_*`` methods plus their three private helpers
    (``_run_vision_track``, ``_resolve_vision_state``,
    ``_build_acquire_drive``). Designed to be mixed in -- never
    instantiated on its own.
    """

    # ================================================================== #
    #  Vision verbs  -- closed-loop, multi-axis, P-only (PI hook in v2)   #
    # ================================================================== #

    def vision_align_3d(self, camera, target_class, axes, duration,
                        deadband, kp_yaw, kp_lat, kp_depth, kp_forward,
                        target_bbox_h_frac, visual_pid, on_lost,
                        stale_after, depth_anchor_frac=0.0,
                        lock_mode='', distance_metric='',
                        gate_guard=False, gate_guard_min_w_frac=0.35,
                        pass_at=0.0, pass_at_gain=50.0,
                        offset_x=0.0, offset_y=0.0,
                        lost_patience_s=0.0):
        """Centre + maintain distance on the largest ``target_class`` bbox.

        ``axes`` is a CSV: any subset of ``'yaw,lat,depth,forward'``.
        This verb is the everything-on tool; the per-axis verbs below
        are pinned wrappers for missions that want to be explicit
        about intent.

        gate_guard=True suppresses forward when gate appears angled (w/h aspect
        below gate_guard_min_w_frac). Experimental — requires pool calibration.

        pass_at: once size metric >= pass_at, freezes lat+depth and drives
        straight at pass_at_gain%. Set to 0.0 to disable.

        impl: motion_vision.vision_track_axes -> pixhawk.send_rc_override
        (heading/lateral/forward channels) and set_target_depth when
        'depth' is in axes.
        """
        axis_set = _parse_axes(axes)
        gains = VisionGains(kp_yaw=float(kp_yaw), kp_lat=float(kp_lat),
                            kp_depth=float(kp_depth),
                            kp_forward=float(kp_forward))
        return self._run_vision_track(
            verb='vision_align_3d', label='align_3d',
            camera=camera, target_class=target_class,
            axes=axis_set, duration=float(duration),
            gains=gains, deadband=float(deadband),
            target_h_frac=float(target_bbox_h_frac),
            visual_pid=bool(visual_pid),
            on_lost=str(on_lost), stale_after=float(stale_after),
            depth_anchor_frac=float(depth_anchor_frac),
            lock_mode=str(lock_mode),
            distance_metric=str(distance_metric),
            gate_guard=bool(gate_guard),
            gate_guard_min_w_frac=float(gate_guard_min_w_frac),
            pass_at=float(pass_at),
            pass_at_gain=float(pass_at_gain),
            offset_x=float(offset_x),
            offset_y=float(offset_y),
            lost_patience_s=float(lost_patience_s))

    def vision_align_yaw(self, camera, target_class, duration, deadband,
                         kp_yaw, on_lost, stale_after, lock_mode='',
                         offset_x=0.0, offset_y=0.0, lost_patience_s=0.0):
        """Steer toward horizontal centre via heading channel. lock_mode: 'settle'/'follow'."""
        gains = VisionGains(kp_yaw=float(kp_yaw))
        return self._run_vision_track(
            verb='vision_align_yaw', label='align_yaw',
            camera=camera, target_class=target_class,
            axes={'yaw'}, duration=float(duration), gains=gains,
            deadband=float(deadband), target_h_frac=0.0,
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after),
            lock_mode=str(lock_mode),
            offset_x=float(offset_x), offset_y=float(offset_y),
            lost_patience_s=float(lost_patience_s))

    def vision_align_lat(self, camera, target_class, duration, deadband,
                         kp_lat, on_lost, stale_after, lock_mode='',
                         offset_x=0.0, offset_y=0.0, lost_patience_s=0.0):
        """Strafe toward horizontal centre (or offset) via lateral channel.

        offset_x: keep target this many pixels to the RIGHT of frame center.
        Negative = keep target left. Use for slalom pipe passing.
        offset_y: keep target this many pixels BELOW frame center.
        """
        gains = VisionGains(kp_lat=float(kp_lat))
        return self._run_vision_track(
            verb='vision_align_lat', label='align_lat',
            camera=camera, target_class=target_class,
            axes={'lat'}, duration=float(duration), gains=gains,
            deadband=float(deadband), target_h_frac=0.0,
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after),
            lock_mode=str(lock_mode),
            offset_x=float(offset_x), offset_y=float(offset_y),
            lost_patience_s=float(lost_patience_s))

    def vision_align_depth(self, camera, target_class, duration, deadband,
                           kp_depth, on_lost, stale_after,
                           depth_anchor_frac=0.0, lock_mode='',
                           offset_x=0.0, offset_y=0.0, lost_patience_s=0.0):
        """Nudge depth setpoint to centre vertically.

        depth_anchor_frac: which vertical point on the bbox to align (0=top,
        0.5=centre, 1=bottom). 0.2 works well for tall objects like people.
        offset_y: keep target this many pixels BELOW frame center.
        """
        gains = VisionGains(kp_depth=float(kp_depth))
        return self._run_vision_track(
            verb='vision_align_depth', label='align_depth',
            camera=camera, target_class=target_class,
            axes={'depth'}, duration=float(duration), gains=gains,
            deadband=float(deadband), target_h_frac=0.0,
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after),
            depth_anchor_frac=float(depth_anchor_frac),
            lock_mode=str(lock_mode),
            offset_x=float(offset_x), offset_y=float(offset_y),
            lost_patience_s=float(lost_patience_s))

    def vision_hold_distance(self, camera, target_class, duration, deadband,
                             kp_forward, target_bbox_h_frac, on_lost,
                             stale_after, lock_mode='', distance_metric='',
                             gate_guard=False, gate_guard_min_w_frac=0.35,
                             pass_at=0.0, pass_at_gain=50.0,
                             offset_x=0.0, offset_y=0.0, lost_patience_s=0.0):
        """Approach / back off to maintain standoff distance by bbox fill fraction.

        lock_mode: 'settle' (exit when at distance), 'follow' (track until
        duration), 'pursue' (only approach, exit when close enough).
        distance_metric: 'height' (default), 'area', 'width', 'diagonal'.
        pass_at: once size >= pass_at, drive straight at pass_at_gain% (0=disabled).
        """
        gains = VisionGains(kp_forward=float(kp_forward))
        return self._run_vision_track(
            verb='vision_hold_distance', label='hold_distance',
            camera=camera, target_class=target_class,
            axes={'forward'}, duration=float(duration), gains=gains,
            deadband=float(deadband),
            target_h_frac=float(target_bbox_h_frac),
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after),
            lock_mode=str(lock_mode),
            distance_metric=str(distance_metric),
            gate_guard=bool(gate_guard),
            gate_guard_min_w_frac=float(gate_guard_min_w_frac),
            pass_at=float(pass_at),
            pass_at_gain=float(pass_at_gain),
            offset_x=float(offset_x), offset_y=float(offset_y),
            lost_patience_s=float(lost_patience_s))

    def vis_approach(self, camera, target_class, duration, deadband,
                     kp_forward, target_vis_range, on_lost,
                     stale_after, lock_mode='',
                     offset_x=0.0, offset_y=0.0, lost_patience_s=0.0):
        """Drive forward using monocular depth (vis_range) as the distance proxy.

        target_vis_range 0..1: 0=far, 1=close. Requires depth_estimation_node
        running on the same camera namespace.
        distance_metric='vis_range' routes through _distance_size → sample.vis_range.
        """
        gains = VisionGains(kp_forward=float(kp_forward))
        return self._run_vision_track(
            verb='vis_approach', label='vis_approach',
            camera=camera, target_class=target_class,
            axes={'forward'}, duration=float(duration), gains=gains,
            deadband=float(deadband),
            target_h_frac=float(target_vis_range),
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after),
            lock_mode=str(lock_mode),
            distance_metric='vis_range',
            offset_x=float(offset_x), offset_y=float(offset_y),
            lost_patience_s=float(lost_patience_s))

    def vision_acquire(self, camera, target_class, target_name, timeout,
                       gain, yaw_rate_pct, stale_after):
        """Block until ``target_class`` appears.

        ``target_name`` picks an OPTIONAL drive verb to use while
        waiting (``''`` = wait in place). ``'arc'`` uses both ``gain``
        (forward thrust) and ``yaw_rate_pct`` so you can sweep an area.

        impl: motion_vision.vision_acquire + per-axis drive closure
        from _build_acquire_drive (calls pixhawk.send_rc_override).
        """
        with self._command_scope('vision_acquire'):
            self._send_neutral_and_settle()
            vstate = self._resolve_vision_state(camera)
            drive_writer = self._build_acquire_drive(
                target_name, gain=float(gain),
                yaw_rate_pct=float(yaw_rate_pct))
            self.log.info(
                f'[CMD  ] vision_acquire camera={camera!r} '
                f'class={target_class!r} drive={target_name or "none"} '
                f'timeout={timeout:.1f}s')
            outcome = run_vision_acquire(
                pixhawk=self.pixhawk, vision_state=vstate,
                target_class=target_class, timeout=float(timeout),
                drive_writer=drive_writer,
                stale_after=float(stale_after),
                writers=self._writers(), log=self.log,
                abort_fn=self._abort_fn)
            self._send_neutral_and_settle()
            return self._make_result(
                outcome.success, f'vision_acquire: {outcome.reason}',
                final_value=outcome.elapsed_s,
                error_value=(0.0 if outcome.success else float(timeout)))

    def look_around(self, camera, target_class, duration, gain,
                    yaw_rate_pct, settle, target, stale_after):
        """Rotate on the spot looking for target_class.

        Switches to POSHOLD (position hold via DVL) then yaws in
        incremental steps of ``yaw_rate_pct`` degrees, dwelling
        ``settle`` seconds at each stop to observe. Exits immediately
        when target_class is detected. Completes a full 360° orbit if
        duration allows.

        Parameters
        ----------
        gain         -- yaw turn speed percent (Ch4)
        yaw_rate_pct -- step size in degrees, signed
                        positive = clockwise (right), negative = CCW (left)
        settle       -- dwell time at each yaw stop to observe (seconds)
        target       -- override starting yaw in degrees (0.0 = current)
        stale_after  -- detection freshness tolerance (seconds)
        """
        with self._command_scope('look_around'):
            self._send_neutral_and_settle()
            vstate = self._resolve_vision_state(camera)

            # Try POSHOLD for true position hold. Fall back to ALT_HOLD
            # gracefully — position may drift but the scan still works.
            accepted, _ = self.pixhawk.set_mode('POSHOLD')
            if not accepted:
                self.log.warning(
                    '[CMD  ] look_around: POSHOLD rejected — '
                    'falling back to ALT_HOLD (position may drift)')
                self._ensure_alt_hold('look_around')

            step_deg  = float(yaw_rate_pct) or 20.0   # degrees per step
            turn_spd  = abs(float(gain)) or 40.0
            dwell     = max(float(settle), 0.3)
            # Negative step_deg → left (CCW); positive → right (CW).
            # Use the same sign for the yaw turn calls.
            turn_deg  = abs(step_deg) * (-1 if step_deg < 0 else 1)

            # Snap to starting yaw if the caller requests one.
            start_yaw = float(target)
            if start_yaw != 0.0:
                yaw_snap(self.pixhawk, self._current_heading(), start_yaw,
                         timeout=10.0, label='LOOK_INIT', log=self.log,
                         yaw_source=self.yaw_source)

            deadline  = time.monotonic() + max(float(duration), 1.0)
            swept_deg = 0.0
            found     = False

            self.log.info(
                f'[CMD  ] look_around camera={camera!r} class={target_class!r} '
                f'step={step_deg:+.0f}° speed={turn_spd:.0f}% '
                f'dwell={dwell:.1f}s budget={duration:.0f}s')

            with self._suspend_heading_lock():
                while time.monotonic() < deadline:
                    if self._abort_fn():
                        break
                    # Observe at current yaw position.
                    t_obs = time.monotonic()
                    while time.monotonic() - t_obs < dwell:
                        sample = vstate.bbox_error(target_class)
                        if sample is not None and sample.age_s <= stale_after:
                            found = True
                            break
                        time.sleep(0.05)

                    if found:
                        break

                    # Exit after a full orbit even if duration remains.
                    swept_deg += abs(turn_deg)
                    if swept_deg >= 360.0:
                        break

                    if time.monotonic() >= deadline:
                        break

                    # Yaw one step.
                    cur = self._current_heading()
                    tgt = (cur + turn_deg) % 360.0
                    yaw_snap(self.pixhawk, cur, tgt,
                             timeout=max(dwell * 2, 6.0),
                             label='LOOK', log=self.log,
                             yaw_source=self.yaw_source)

            self._retarget_heading_lock(self._current_heading())
            self._send_neutral_and_settle()

            reason = (f'found {target_class!r} after {swept_deg:.0f}° scan'
                      if found else
                      f'no {target_class!r} detected after full scan ({swept_deg:.0f}°)')
            self.log.info(f'[CMD  ] look_around: {reason}')
            return self._make_result(found, f'look_around: {reason}',
                                     final_value=swept_deg,
                                     error_value=0.0)

    # ---- vision helpers (private) ----------------------------------- #

    def _run_vision_track(self, *, verb, label, camera, target_class, axes,
                          duration, gains, deadband, target_h_frac,
                          visual_pid, on_lost, stale_after,
                          depth_anchor_frac=0.5, lock_mode='settle',
                          distance_metric='height',
                          gate_guard=False, gate_guard_min_w_frac=0.35,
                          pass_at=0.0, pass_at_gain=50.0,
                          offset_x=0.0, offset_y=0.0,
                          stable_lock_s=0.0, on_stable=None,
                          lost_patience_s=0.0):
        """Common path for every vision_align_* / vision_hold_distance verb.

        ``verb`` is the public method name (``'vision_align_yaw'``, ...)
        and flows into ``_command_scope`` so the MAVLink trace line
        carries ``cmd=<verb>``. ``label`` is the shorter human log token.

        When ``'depth'`` is in the axis set we ensure ALT_HOLD is engaged
        so ArduSub honours our streamed depth setpoints. The verb is the
        sole author of depth packets while it runs; on exit the autopilot
        keeps holding the new depth without any background streamer.
        """
        with self._command_scope(verb):
            self._send_neutral_and_settle()
            vstate = self._resolve_vision_state(camera)
            is_downward     = camera in ('downward',)
            depth_sign      = -1 if is_downward else +1
            # Downward cam: ey (vertical in image) maps to forward/back axis.
            forward_uses_ey = is_downward and 'forward' in axes
            touches_depth   = 'depth' in axes
            touches_yaw   = 'yaw' in axes
            if touches_depth:
                self._ensure_alt_hold(f'vision_{label}')
            self.log.info(
                f'[CMD  ] vision_{label}  camera={camera!r}  '
                f'class={target_class!r}  axes={sorted(axes)}  '
                f'duration={duration:.1f}s  on_lost={on_lost}  '
                f'lock={lock_mode or "settle"}  anchor={depth_anchor_frac:.2f}  '
                f'dist_metric={distance_metric or "height"}  '
                f'gate_guard={gate_guard}  pass_at={pass_at:.2f}')
            # When yaw is in axes, vision_track_axes writes Ch4 directly.
            # Suspend HeadingLock for the duration to avoid a Ch4 race,
            # then retarget to the new heading on exit (same as arc).
            with self._suspend_heading_lock() if touches_yaw else nullcontext():
                outcome = vision_track_axes(
                    pixhawk=self.pixhawk, vision_state=vstate,
                    target_class=target_class, axes=axes,
                    duration=duration, gains=gains,
                    target_h_frac=target_h_frac,
                    deadband=deadband, stale_after=stale_after,
                    on_lost=on_lost, depth_sign=depth_sign,
                    depth_anchor_frac=depth_anchor_frac,
                    lock_mode=lock_mode, distance_metric=distance_metric,
                    gate_guard=gate_guard,
                    gate_guard_min_w_frac=gate_guard_min_w_frac,
                    pass_at=pass_at, pass_at_gain=pass_at_gain,
                    offset_x=offset_x, offset_y=offset_y,
                    forward_uses_ey=forward_uses_ey,
                    stable_lock_s=stable_lock_s, on_stable=on_stable,
                    **({'lost_patience_s': lost_patience_s}
                       if lost_patience_s > 0.0 else {}),
                    log=self.log, writers=self._writers(),
                    visual_pid=visual_pid,
                    abort_fn=self._abort_fn)
            if touches_yaw:
                self._retarget_heading_lock(self._current_heading())
            self._send_neutral_and_settle()
            return self._make_result(
                outcome.success,
                f'vision_{label}: {outcome.reason}',
                final_value=outcome.composite_error,
                error_value=outcome.last_age_s)

    def _resolve_vision_state(self, camera):
        if self.vision_state_provider is None:
            raise RuntimeError(
                'vision verbs require a vision_state_provider; '
                'launch via auv_manager_node so the manager can wire '
                'VisionState into Duburi.')
        vstate = self.vision_state_provider(camera)
        if vstate is None:
            raise RuntimeError(
                f"vision_state_provider({camera!r}) returned None; "
                f"check the camera name matches a running detector_node "
                f"(e.g. 'laptop', 'sim_front').")
        return vstate

    def _build_acquire_drive(self, drive_verb, *, gain, yaw_rate_pct):
        """Return an ``f(elapsed)`` that writes the requested motion, or None.

        Designed to keep ``motion_vision.vision_acquire`` agnostic of
        which axis is moving -- it just calls the closure each tick.
        """
        if not drive_verb:
            return None
        verb = drive_verb.strip().lower()
        if verb == 'yaw_left':
            yaw_pct = -abs(yaw_rate_pct)
            return lambda _t: self.pixhawk.send_rc_override(
                yaw=Pixhawk.percent_to_pwm(yaw_pct))
        if verb == 'yaw_right':
            yaw_pct = +abs(yaw_rate_pct)
            return lambda _t: self.pixhawk.send_rc_override(
                yaw=Pixhawk.percent_to_pwm(yaw_pct))
        if verb == 'move_forward':
            fwd_pct = abs(gain)
            return lambda _t: self.pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(fwd_pct))
        if verb == 'arc':
            fwd_pct = abs(gain)
            yaw_pct = float(yaw_rate_pct)
            return lambda _t: self.pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(fwd_pct),
                yaw=Pixhawk.percent_to_pwm(yaw_pct))
        raise ValueError(
            f"vision_acquire: unknown drive verb {drive_verb!r}; "
            f"expected one of '', 'yaw_left', 'yaw_right', 'move_forward', 'arc'")

    # ------------------------------------------------------------------ #
    #  vision_lock_fire                                                    #
    # ------------------------------------------------------------------ #

    def vision_lock_fire(self, camera, target_class, axes, duration,
                         deadband, kp_yaw, kp_lat, kp_depth, kp_forward,
                         target_bbox_h_frac, on_lost, stale_after,
                         depth_anchor_frac=0.0, distance_metric='',
                         stable_lock_s=3.0,
                         max_attempts=3, attempt_timeout=15.0,
                         fire_aux_channel=0, fire_pwm=1900,
                         offset_x=0.0, offset_y=0.0,
                         fire_channel=0, lost_patience_s=0.0):
        """Lock 3D position on target, verify stable hold, fire.

        Aligns on all requested axes; once all axes stay within deadband for
        stable_lock_s seconds continuously the on_stable callback fires:
          - fire_channel 1/2 = torpedo, 3/4 = dropper (ESP32 serial via payload driver)
          - fire_aux_channel > 0 = AUX PWM fallback
          - both 0 = log-only stub

        Retries up to max_attempts. On total failure fires at last captured
        aim-hold pose as a best-effort fallback.
        """
        axis_set = _parse_axes(axes)
        gains = VisionGains(kp_yaw=float(kp_yaw), kp_lat=float(kp_lat),
                            kp_depth=float(kp_depth),
                            kp_forward=float(kp_forward))
        fired = False
        last_att = self.pixhawk.get_attitude()
        _fire_ch  = int(fire_channel)
        _aux_ch   = int(fire_aux_channel)
        _aux_pwm  = int(fire_pwm)

        def _on_stable():
            nonlocal fired, last_att
            last_att = self.pixhawk.get_attitude()
            fired = True
            self._do_fire(_fire_ch, _aux_ch, _aux_pwm)

        for attempt in range(int(max_attempts)):
            if self._abort_fn and self._abort_fn():
                break
            self.log.info(
                f'[LOCK_FIRE] attempt {attempt + 1}/{int(max_attempts)} '
                f'stable_lock_s={stable_lock_s:.1f}s')
            self._run_vision_track(
                verb='vision_lock_fire', label='lock_fire',
                camera=camera, target_class=target_class,
                axes=axis_set, duration=float(attempt_timeout),
                gains=gains, deadband=float(deadband),
                target_h_frac=float(target_bbox_h_frac),
                visual_pid=False,
                on_lost=str(on_lost), stale_after=float(stale_after),
                depth_anchor_frac=float(depth_anchor_frac),
                distance_metric=str(distance_metric),
                stable_lock_s=float(stable_lock_s),
                on_stable=_on_stable,
                offset_x=float(offset_x),
                offset_y=float(offset_y),
                lost_patience_s=float(lost_patience_s))
            if fired:
                break

        if not fired:
            yaw   = last_att.get('yaw', 0.0)
            depth = last_att.get('depth', 0.0)
            self.log.warning(
                f'[LOCK_FIRE] all {int(max_attempts)} attempts failed; '
                f'fallback fire at last pose yaw={yaw:.1f} depth={depth:.2f}m')
            self._do_fire(_fire_ch, _aux_ch, _aux_pwm)

        return self._make_result(
            success=fired,
            message=f'vision_lock_fire: {"fired" if fired else "fallback fired"}',
            final_value=float(fired),
            error_value=0.0)

    def _do_fire(self, fire_channel: int, aux_channel: int, pwm: int) -> None:
        """Route fire to ESP32 payload driver, AUX PWM, or log-stub.

        Called from inside a vision tracking scope -- uses _fire_payload()
        (raw driver call) to avoid a nested command-scope deadlock.
        """
        if fire_channel > 0:
            ok = self._fire_payload(fire_channel)
            if not ok:
                self.log.warning(
                    f'[FIRE ] ch={fire_channel} failed (payload not ready) '
                    f'-- falling back to stub')
        elif aux_channel > 0:
            self.pixhawk.set_servo_pwm(aux_channel, pwm)
            self.log.info(f'[FIRE ] AUX ch={aux_channel} pwm={pwm}')
        else:
            self.log.info('[FIRE ] no channel configured -- log-only stub')
