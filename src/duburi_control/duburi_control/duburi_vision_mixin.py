#!/usr/bin/env python3
"""Vision verbs for the Duburi facade.

Split out of ``duburi.py`` so the motion-axis side of the facade
(forward/lateral/yaw/depth/lock/pause/stop) lives in one file and the
camera-driven side lives in another. Mixed into ``Duburi`` via
multiple inheritance:

    class Duburi(DuburiVisionMixin):
        ...

The mixin only references attributes the base ``Duburi`` provides
(``self.lock``, ``self.pixhawk``, ``self.log``, ``self._writers()``,
``self._send_neutral_and_settle()``, ``self._suspend_depth_lock()``,
``self._ensure_alt_hold()``, ``self._retarget_depth_lock()``,
``self._current_depth()``, ``self._make_result()``,
``self.vision_state_provider``) -- nothing rclpy-aware, identical
serialisation contract as the rest of the facade.

All six verbs share the same pipeline:
  1. Resolve VisionState for ``camera`` (lazy preflight in the manager).
  2. Build a VisionGains from the operator-supplied ``kp_*``.
  3. Hand off to motion_vision.vision_track_axes / vision_acquire.
  4. Wrap the VisionTrackResult in a Move.Result.

The single-axis convenience verbs (``vision_align_yaw``, ``_lat``,
``_depth``) are just ``vision_align_3d`` with ``axes`` pinned -- one
canonical loop, no copy-paste.
"""

from contextlib import contextmanager

from .motion_vision import (
    VisionGains, vision_acquire as run_vision_acquire,
    vision_track_axes,
)
from .pixhawk import Pixhawk


@contextmanager
def _nullctx():
    """Drop-in no-op context manager (Python 3.7+ alternative to
    contextlib.nullcontext, kept inline so the conditional ``with``
    blocks below stay readable)."""
    yield


def _parse_axes(csv: str):
    """``'yaw,forward'`` -> ``{'yaw','forward'}``. Whitespace + case tolerant."""
    out = set()
    for token in (csv or '').split(','):
        name = token.strip().lower()
        if name:
            out.add(name)
    return out


class DuburiVisionMixin:
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
                        stale_after):
        """Centre + maintain distance on the largest ``target_class`` bbox.

        ``axes`` is a CSV: any subset of ``'yaw,lat,depth,forward'``.
        This verb is the everything-on tool; the per-axis verbs below
        are pinned wrappers for missions that want to be explicit
        about intent.
        """
        axis_set = _parse_axes(axes)
        gains = VisionGains(kp_yaw=float(kp_yaw), kp_lat=float(kp_lat),
                            kp_depth=float(kp_depth),
                            kp_forward=float(kp_forward))
        return self._run_vision_track(
            label='align_3d', camera=camera, target_class=target_class,
            axes=axis_set, duration=float(duration),
            gains=gains, deadband=float(deadband),
            target_h_frac=float(target_bbox_h_frac),
            visual_pid=bool(visual_pid),
            on_lost=str(on_lost), stale_after=float(stale_after))

    def vision_align_yaw(self, camera, target_class, duration, deadband,
                         kp_yaw, on_lost, stale_after):
        gains = VisionGains(kp_yaw=float(kp_yaw))
        return self._run_vision_track(
            label='align_yaw', camera=camera, target_class=target_class,
            axes={'yaw'}, duration=float(duration), gains=gains,
            deadband=float(deadband), target_h_frac=0.0,
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after))

    def vision_align_lat(self, camera, target_class, duration, deadband,
                         kp_lat, on_lost, stale_after):
        gains = VisionGains(kp_lat=float(kp_lat))
        return self._run_vision_track(
            label='align_lat', camera=camera, target_class=target_class,
            axes={'lat'}, duration=float(duration), gains=gains,
            deadband=float(deadband), target_h_frac=0.0,
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after))

    def vision_align_depth(self, camera, target_class, duration, deadband,
                           kp_depth, on_lost, stale_after):
        gains = VisionGains(kp_depth=float(kp_depth))
        return self._run_vision_track(
            label='align_depth', camera=camera, target_class=target_class,
            axes={'depth'}, duration=float(duration), gains=gains,
            deadband=float(deadband), target_h_frac=0.0,
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after))

    def vision_hold_distance(self, camera, target_class, duration, deadband,
                             kp_forward, target_bbox_h_frac, on_lost,
                             stale_after):
        gains = VisionGains(kp_forward=float(kp_forward))
        return self._run_vision_track(
            label='hold_distance', camera=camera, target_class=target_class,
            axes={'forward'}, duration=float(duration), gains=gains,
            deadband=float(deadband),
            target_h_frac=float(target_bbox_h_frac),
            visual_pid=False, on_lost=str(on_lost),
            stale_after=float(stale_after))

    def vision_acquire(self, camera, target_class, target_name, timeout,
                       gain, yaw_rate_pct, stale_after):
        """Block until ``target_class`` appears.

        ``target_name`` picks an OPTIONAL drive verb to use while
        waiting (``''`` = wait in place). ``'arc'`` uses both ``gain``
        (forward thrust) and ``yaw_rate_pct`` so you can sweep an area.
        """
        with self.lock:
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
                stale_after=float(stale_after), log=self.log)
            self._send_neutral_and_settle()
            return self._make_result(
                outcome.success, f'vision_acquire: {outcome.reason}',
                final_value=outcome.elapsed_s,
                error_value=(0.0 if outcome.success else float(timeout)))

    # ---- vision helpers (private) ----------------------------------- #

    def _run_vision_track(self, *, label, camera, target_class, axes,
                          duration, gains, deadband, target_h_frac,
                          visual_pid, on_lost, stale_after):
        """Common path for every vision_align_* / vision_hold_distance verb.

        When ``'depth'`` is in the axis set we (a) ensure ALT_HOLD is
        engaged so ArduSub honours the streamed depth setpoints, and
        (b) suspend any active ``lock_depth`` so the two authors of
        depth setpoints don't fight; on exit we retarget the lock to
        the post-verb depth so subsequent commands inherit the new hold.
        """
        with self.lock:
            self._send_neutral_and_settle()
            vstate = self._resolve_vision_state(camera)
            depth_sign = -1 if camera in ('downward',) else +1
            touches_depth = 'depth' in axes
            if touches_depth:
                self._ensure_alt_hold(f'vision_{label}')
            self.log.info(
                f'[CMD  ] vision_{label}  camera={camera!r}  '
                f'class={target_class!r}  axes={sorted(axes)}  '
                f'duration={duration:.1f}s  on_lost={on_lost}')
            depth_ctx = (self._suspend_depth_lock() if touches_depth
                         else _nullctx())
            with depth_ctx:
                outcome = vision_track_axes(
                    pixhawk=self.pixhawk, vision_state=vstate,
                    target_class=target_class, axes=axes,
                    duration=duration, gains=gains,
                    target_h_frac=target_h_frac,
                    deadband=deadband, stale_after=stale_after,
                    on_lost=on_lost, depth_sign=depth_sign,
                    log=self.log, writers=self._writers(),
                    visual_pid=visual_pid)
                self._send_neutral_and_settle()
            if touches_depth:
                self._retarget_depth_lock(self._current_depth())
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
