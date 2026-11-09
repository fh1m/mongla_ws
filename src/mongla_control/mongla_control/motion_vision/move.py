"""`move_loop`: drive toward a target until it fills the frame, or through it.

One tick is `_Move.step`: sense, then commit, give up, or approach. The shared
parts are in `control.py`.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Optional, Tuple

from ..motion_rates import VISION_LOOP_HZ as LOOP_HZ
from .control import (
    ABORTED,
    ALIGNED,
    COMMIT_UNVERIFIED,
    ESTIMATED_CROSSING,
    KP_FORWARD_DEFAULT,
    KP_LAT_DEFAULT,
    NO_CAMERA,
    Outcome,
    TIMEOUT,
    VALID_MODES,
    VISION_BRAKE_GAIN,
    _PASSTHROUGH_COMMIT_S,
    _VisionLoop,
    _brake_axis,
    _camera_ready,
    _clamp,
    _crossing_evidence,
    _ema_step,
    _fill,
    _present,
    _range_gain,
    _tick,
)
from .freshness import (
    _authority,
    _is_evidence,
    _warn_low_fps,
)
from .srot_drive import (
    _srot_drive,
)

# ---------------------------------------------------------------------- #
#  move_loop -- drive forward to a bbox fill ratio                       #
# ---------------------------------------------------------------------- #
def move_loop(*,
              fc,
              vision_state,
              target_class: str,
              fwd_fill: float,
              mode: str = 'area',
              passthrough: bool = False,
              maintain_px: float = 0.0,
              maintain_on: bool = False,
              hold_s: float = 0.0,
              err_px: float = 40.0,
              duration: float = 20.0,
              gain: float = 30.0,
              gain_lat: Optional[float] = None,
              brake: bool = True,
              brake_gain: float = VISION_BRAKE_GAIN,
              kp_forward: float = KP_FORWARD_DEFAULT,
              kp_lat: float = KP_LAT_DEFAULT,
              lost_grace_s: float = 1.0,
              hold_through_loss: bool = False,
              range_gain_floor: float = 1.0,
              coast_s: float = 0.0,
              lock_s: float = 0.0,
              mixer_aware: bool = True,
              report_fn=None,
              writers=None,
              log=None,
              abort_fn=None) -> Outcome:
    """Drive forward toward ``target_class`` -- stop at a fill ratio or pass through.

    Two modes:

    * **fill-stop** (default): drive forward, slowing as the bbox fills,
      until it reaches ``fwd_fill`` (measured by ``mode``); ``hold_s``
      station-keeps there before exiting.
    * **pass-through** (``passthrough=True``, selected by the DSL when
      ``move(fwd=None)``): drive forward at ``gain`` while the target is
      visible; once the target has been seen and then leaves the frame,
      keep driving for a commit window (``hold_s`` if given, else
      ``_PASSTHROUGH_COMMIT_S``, and never past ``duration``) to carry the
      hull *through* the gate. It reports what it can stand behind (#79):
      ESTIMATED_CROSSING when the last live box surrounded the camera axis
      and ran off the frame, COMMIT_UNVERIFIED otherwise -- never ALIGNED,
      because losing a gate is not seeing the hull cross it. If the target
      is never seen, falls back to the normal loss path (LOST -> mission
      fallback search).

    ``maintain_on`` holds a lateral pixel offset (``maintain_px``) while
    driving; depth and yaw are never commanded (the board holds both). Returns an Outcome -- never
    raises on a miss.

    ``report_fn`` (if given) is called every PRESENT tick with the signed
    from-centre pixel offset ``(x_off, y_off)`` -- the live-telemetry sink for
    action feedback (same contract as align_loop).

    How one tick runs: `_Move.run`, one method per stage.
    """
    c = SimpleNamespace(**locals())   # every argument, by name, for _Move
    if mode not in VALID_MODES:
        raise ValueError(f"move_loop: mode must be one of {sorted(VALID_MODES)}")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    return _Move(c).run()


class _Move(_VisionLoop):
    """`move_loop`'s state, and one method per stage of a tick."""

    verb = 'move'

    def __init__(self, c) -> None:
        super().__init__(c)
        # Lateral 'maintain' strafe gets its own cap; forward stays capped by `gain`.
        self.g_lat = c.gain if c.gain_lat is None else c.gain_lat
        self.reached_at: Optional[float] = None
        self.commit_until: Optional[float] = None
        # (ex, half-width) of the last LIVE box: what the crossing verdict reads.
        self.last_live_box: Optional[Tuple[float, float]] = None
        self.fwd_ema = 0.0   # trailing EMA of forward command -> brake proxy (fill-stop)
        self.lat_ema = 0.0   # trailing EMA of the maintain strafe command -> brake proxy
        # Pass-through commit window: hold_s overrides the module default.
        self.commit_s = c.hold_s if c.hold_s > 0.0 else _PASSTHROUGH_COMMIT_S
        self.log.debug(
            f"[VIS  ] move class={c.target_class!r} "
            f"{'PASS-THROUGH commit=%.1fs' % self.commit_s if c.passthrough else 'fwd_fill=%.0f%%' % (c.fwd_fill * 100)} "
            f"mode={c.mode} maintain={'%+.0fpx' % c.maintain_px if c.maintain_on else 'off'} "
            f"gain={c.gain:.0f}% dur={c.duration:.0f}s")
        self.started = time.monotonic()
        self.deadline = self.started + max(c.duration, 0.0)
        self._last_pass = self.started
        self.dt = 1.0 / LOOP_HZ

    # -- the loop ------------------------------------------------------- #

    def run(self) -> Outcome:
        try:
            while True:
                out = self.step(time.monotonic())
                if out is not None:
                    return out
                _tick(self.c.vision_state)
        finally:
            self.neutral_on_exit()

    def step(self, now: float) -> Optional[Outcome]:
        """One tick: check the clock, sense, then commit, give up, or drive."""
        c = self.c
        # Seconds since the last pass, for the brake EMA. Clamped like
        # align_loop's: a stall must not dump a huge step into it.
        self.dt = min(max(now - self._last_pass, 1e-3), 4.0 / LOOP_HZ)
        self._last_pass = now
        if c.abort_fn and c.abort_fn():
            return self.outcome(ABORTED, "aborted", now)
        if now >= self.deadline:
            if self.commit_until is not None:
                # The commit was under way: duration bounds it, and what the
                # hull was doing is still a crossing verdict, not a timeout.
                return self.crossing(now, cut_short=True)
            if not self.seen:
                reason = self.never_seen()
            else:
                reason = ("passed-through window not closed (duration elapsed)"
                          if c.passthrough else "fill not reached (duration elapsed)")
            return self.outcome(TIMEOUT, reason, now)

        sample = self.sense()
        present = _present(sample)
        if present and not sample.coasted and sample.track_id >= 0:
            self.locked_id = sample.track_id   # follow this id when a gap coasts

        if c.passthrough and self.seen and not present:
            return self.commit(now)
        if not present:
            self.reached_at = None
            self.drive(0.0, 0.0)
            return self.lost(now)

        fill, fresh = self.acquire(sample, now)
        lat_pct = self.maintain(sample, fresh)
        x_off = sample.ex * self.half_w   # signed horizontal offset (operator px)
        if c.passthrough:
            # Drive forward at the speed cap until the target leaves frame.
            # Pass-through is defined to COAST through the gate -- never braked,
            # and forward is NOT freshness-decayed (it must clear the gate).
            self.drive(c.gain, lat_pct)
            if self.log_due(now):
                self.log.info(f"[ move PASS-THROUGH fill={fill * 100:.0f}% "
                              f"lat={x_off:+.0f}px ] ['{c.target_class}'] -> clear gate")
            return None

        out = self.approach(now, fill, fresh, lat_pct,
                            evidence=_is_evidence(sample, fresh))
        if out is not None:
            return out
        if self.log_due(now):
            hold_tag = ("" if self.reached_at is None
                        else f" hold={now - self.reached_at:.1f}/{c.hold_s:.0f}s")
            self.log.info(
                f"[ move fill={fill * 100:.0f}% -> {c.fwd_fill * 100:.0f}% "
                f"lat={x_off:+.0f}px ] ['{c.target_class}']{hold_tag}")
        return None

    # -- the stages ----------------------------------------------------- #

    def drive(self, fwd_pct: float, lat_pct: float) -> None:
        # move never commands yaw: the board holds heading.
        _srot_drive(self.c.fc, fwd_pct=fwd_pct, lat_pct=lat_pct, yaw_pct=0.0,
                    prioritise=self.c.mixer_aware)

    def sense(self):
        """The target box this tick.

        Pass-through (fwd=None) must DETECT "target gone" to fire its commit
        window, so it deliberately does NOT coast -- a coasted box would
        keep present=True and delay the commit by up to coast_s. Fill-stop
        moves coast normally.
        """
        c = self.c
        # The lock ladder, too: a follower or anchor box keeps the gate
        # "present" for up to its 2.5 s horizon after the detector lost it,
        # and the commit would start that much late -- inside the gate.
        eff_coast = 0.0 if c.passthrough else c.coast_s
        eff_lock = 0.0 if c.passthrough else c.lock_s
        return c.vision_state.bbox_error(
            c.target_class, locked_id=self.locked_id, coast_s=eff_coast,
            lock_s=eff_lock)

    def commit(self, now: float) -> Optional[Outcome]:
        """Pass-through trigger: the target was seen and has now left the frame.

        Keep driving straight for the commit window so the hull clears the
        gate, then report success.
        """
        c = self.c
        self.reached_at = None
        if self.commit_until is None:
            self.commit_until = now + self.commit_s
            self.log.debug(f"[VIS  ] move: {c.target_class!r} cleared frame -- "
                           f"committing {self.commit_s:.1f}s to pass through")
        if now >= self.commit_until:
            return self.crossing(now)
        self.drive(c.gain, 0.0)   # no detection -> no lateral, just drive on
        return None

    def crossing(self, now: float, cut_short: bool = False) -> Outcome:
        """End a pass-through commit with the verdict the last live box supports."""
        self.c.writers.neutral()
        why = (f"{self.commit_s:.1f}s commit" if not cut_short
               else "commit cut short by duration")
        box = self.last_live_box
        if box is not None and _crossing_evidence(*box):
            return self.outcome(
                ESTIMATED_CROSSING,
                f"gate surrounded the camera axis and ran off the frame "
                f"(ex {box[0]:+.2f}, width {box[1] * 2:.2f} of frame); {why} -- "
                f"an estimate, not an observed crossing", now)
        seen = ("never seen live" if box is None else
                f"last live box ex {box[0]:+.2f}, width {box[1] * 2:.2f} of frame, "
                f"did not run off the frame around the axis")
        return self.outcome(
            COMMIT_UNVERIFIED,
            f"gate lost before any sign of crossing ({seen}); {why} ran "
            f"open-loop", now)

    def acquire(self, sample, now: float) -> Tuple[float, float]:
        """The target is present: ``(fill, freshness)`` for this tick."""
        c = self.c
        self.seen = True
        self.lost_since = None
        self.commit_until = None
        if not getattr(sample, 'coasted', False):
            self.last_live_box = (sample.ex, sample.w_frac)
        fill = _fill(sample, c.mode)
        self.fill = fill
        self.note_frame(sample, now)
        fresh = _authority(sample, c.coast_s, self.det_interval, self.pipe_age)  # FPS staleness / coast decay
        _warn_low_fps(self.log, fresh, sample)     # F3: surface FPS-starvation, don't stall silently
        # End-position (returned) + live feedback sink.
        self.report(sample)
        return fill, fresh

    def maintain(self, sample, fresh: float) -> float:
        """The lateral strafe that holds ``maintain_px``; 0 when maintain is off."""
        c = self.c
        lat_pct = 0.0
        if c.maintain_on:
            ctrl = sample.ex - c.maintain_px / self.half_w
            self.residual_px = abs(ctrl) * self.half_w
            rgain = _range_gain(_fill(sample, c.mode), c.range_gain_floor)
            lat_pct = _clamp(ctrl * c.kp_lat * rgain, -self.g_lat, self.g_lat) * fresh
        self.lat_ema = _ema_step(self.lat_ema, lat_pct, self.dt)
        return lat_pct

    def approach(self, now: float, fill: float, fresh: float,
                 lat_pct: float, evidence: bool = True) -> Optional[Outcome]:
        """Fill-stop: drive toward ``fwd_fill``, then hold and brake on arrival.

        Arrival needs evidence (#78). A predicted or zero-authority box at the
        fill stops forward drive -- never closer on a guess -- but neither
        starts nor completes the arrival hold; the next real sighting does.
        """
        c = self.c
        if fill < c.fwd_fill:
            self.reached_at = None
            # Freshness-decay forward so a stale frame doesn't blind-drive the
            # approach past the fill target (yaw/depth untouched).
            fwd_pct = _clamp((c.fwd_fill - fill) * c.kp_forward, 0.0, c.gain) * fresh
            self.drive(fwd_pct, lat_pct)
            self.fwd_ema = _ema_step(self.fwd_ema, fwd_pct, self.dt)
            return None
        if not evidence:
            self.drive(0.0, lat_pct)
            self.fwd_ema = _ema_step(self.fwd_ema, 0.0, self.dt)
            return None
        if self.reached_at is None:
            self.reached_at = now
        # Station-keep: zero forward, keep correcting lateral if asked.
        self.drive(0.0, lat_pct)
        self.fwd_ema = _ema_step(self.fwd_ema, 0.0, self.dt)   # forward decays while holding
        if c.hold_s > 0.0 and (now - self.reached_at) < c.hold_s:
            return None
        # Arrival: bleed the forward (and maintain-strafe) inertia so the
        # hull halts in front of the target instead of creeping into it.
        if c.brake:
            _brake_axis(c.writers.forward, self.fwd_ema, c.brake_gain,
                        abort_fn=c.abort_fn, log=self.log, label='VBRK')
            if c.maintain_on:
                _brake_axis(c.writers.lateral, self.lat_ema, c.brake_gain,
                            abort_fn=c.abort_fn, log=self.log, label='VBRK')
        c.writers.neutral()
        return self.outcome(ALIGNED, f"reached fill={fill * 100:.0f}%", now)
