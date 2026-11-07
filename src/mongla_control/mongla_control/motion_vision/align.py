"""`align_loop`: centre a target on the chosen axes and hold it there.

One tick is `_Align.step`: sense, acquire, command, drive, count, hold. The
shared parts are in `control.py`.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Dict, NamedTuple, Optional, Set

from ..motion_rates import VISION_LOOP_HZ as LOOP_HZ
from .control import (
    ABORTED,
    ALIGNED,
    KP_FORWARD_DEFAULT,
    KP_LAT_DEFAULT,
    KP_YAW_DEFAULT,
    MIN_ALIGN_ERR_PX,
    NO_CAMERA,
    Outcome,
    SETTLE_PX_DEFAULT,
    TIMEOUT,
    VALID_AXES,
    VISION_BRAKE_GAIN,
    VISION_I_LAT_MAX,
    _ALIGN_STABLE_MIN_S,
    _VisionLoop,
    _axis_ctrl,
    _brake_axis,
    _camera_ready,
    _clamp,
    _ema_step,
    _fill,
    _forward_range_hold,
    _lock_gate,
    _present,
    _range_gain,
    _tick,
    _tool_offset,
    _verify_after_stop,
    _yaw_command,
)
from .freshness import (
    _authority,
    _derotated_ex,
    _warn_low_fps,
)
from .srot_drive import (
    _mixer_saturated,
    _srot_drive,
)

def align_loop(*,
               fc,
               vision_state,
               target_class: str,
               axes: Set[str],
               offsets: Dict[str, float],
               err_px: float,
               duration: float,
               gain: float,
               gain_lat: Optional[float] = None,
               gain_yaw: Optional[float] = None,
               brake: bool = True,
               brake_gain: float = VISION_BRAKE_GAIN,
               hold_s: float = 0.0,
               kp_lat: float = KP_LAT_DEFAULT,
               kp_yaw: float = KP_YAW_DEFAULT,
               lost_grace_s: float = 1.0,
               hold_through_loss: bool = False,
               align_stable_frames: int = 3,
               lock_on: bool = False,
               ctrl_conf: float = 0.0,
               range_gain_floor: float = 1.0,
               ki_lat: float = 0.0,
               i_lat_max: float = VISION_I_LAT_MAX,
               coast_s: float = 0.0,
               lock_s: float = 0.0,
               mixer_aware: bool = True,
               fwd_fill: float = 0.0,
               fwd_mode: str = 'area',
               kp_forward: float = KP_FORWARD_DEFAULT,
               settle_px: float = SETTLE_PX_DEFAULT,
               verify: bool = True,
               downward: bool = False,
               surge_sign: int = +1,
               on_locked=None,
               fire_t: float = 0.0,
               fire_pass: bool = False,
               fire_max_tilt_deg: float = 0.0,
               tilt_gate_fn=None,
               standoff_max_tilt_deg: float = 0.0,
               tool_offset_fn=None,
               derotate_fn=None,
               obliquity_fn=None,
               report_fn=None,
               writers=None,
               log=None,
               abort_fn=None) -> Outcome:
    """Hold ``target_class`` at the requested pixel offset on each active axis.

    ``axes`` is a subset of {'lat','yaw','depth'}; ``offsets`` carries the
    signed pixel offset for each active axis (0 = centre). ``depth`` exists
    only on the downward camera, where it is image-Y driven as surge.
    Returns an Outcome -- never raises on a miss.

    ``hold_s`` > 0 turns the verb into an ACTIVE station-keep: once centred,
    the loop keeps running its corrections for hold_s seconds
    (fighting water inertia) before exiting ALIGNED, instead of exiting on the
    first stable tick. hold_s counts against ``duration`` -- budget
    duration >= approach + hold_s or the verb TIMEOUTs mid-hold.

    ``on_locked`` (if given) is called AT MOST ONCE, on the first STABLY-ALIGNED
    tick at or after ``fire_t`` seconds into the hold window (measured from the
    first stable tick), BEFORE the hold-exit check -- so a payload fire fires
    mid-hold while the loop is still correcting, not on the drifting exit tick.
    It is gated on the target being in-band: if alignment is never held during
    the window, ``on_locked`` is NOT called (a torpedo never fires off-target).
    The caller is expected to make ``on_locked`` non-blocking (it spawns the fire
    on a background thread); the 20 Hz loop must not stall. ``fire_t`` should be
    < hold_s (the verb clamps it upstream). The fire is additionally gated on a
    FRESH detection (``is_new_frame and not sample.coasted``) -- it leaves only on
    the tick a genuinely new live box lands, so it is FPS-robust (fires promptly at
    low detector rate) yet never fires on a frozen detector's stale frame or a
    Kalman-coasted box.

    ``fire_pass`` (opt-in, default off): if ``on_locked`` never fired a strict
    in-band shot, fire it anyway on a NATURAL exit (TIMEOUT / hold-complete) as long
    as the target was seen LIVE within ``lost_grace_s`` -- a guaranteed partial-
    points shot when full alignment was not reached. Never fires when the target was
    never seen or only coasted.

    ``downward`` -- downward-camera frame remap (bottom-mounted cam, looks at the
    pool floor). The bin task hovers ABOVE the target, so the body axes rotate:
      * ``lat`` (image-X) -> lateral strafe  -- SAME as forward.
      * ``depth`` (image-Y) -> SURGE fore/aft -- REMAPPED. Two-sided P-on-pixel
        modelled on the lat axis (rgain + freshness-decay + arrival reverse-kick
        brake), NOT the depth setpoint. Surge is open-loop timed thrust so it MUST be
        braked or the 20 kg hull coasts past the bin. ``surge_sign`` flips its
        polarity for the physical mount (verify DISARMED with vision_thrust_check --
        a wrong sign is positive feedback that drives the hull AWAY from the bin).
      * fire -> dropper via ``on_locked`` -- same mechanism as torpedo.
    The board holds depth (``set_depth``) and heading throughout; yaw is not
    driven on the downward camera.
    ``report_fn`` (if given) is called every PRESENT tick with the signed
    from-centre pixel offset ``(x_off, y_off)`` of the target -- a live-telemetry
    sink for action feedback. It stays rclpy-free (just a callable); the caller
    wires it to a shared slot the manager's feedback pump reads.

    Precision knobs (all default to "no change"):
      * ``lock_on`` -- after the target is acquired, steer to the detection
        NEAREST the last-accepted centre within ``VISION_LOCK_GATE_NORM`` (not the
        largest box), so a second hole / spurious box can't steal the aim. Resets
        to largest-area acquisition after a real loss (LOST exit).
      * ``ctrl_conf`` -- control-side minimum detection score to accept a box.
      * ``range_gain_floor`` -- scales lat/surge kp DOWN as the bbox fills the
        frame (close) to stop the close-in overshoot (1.0 = off).
      * ``ki_lat`` -- lateral integral gain; cancels the steady-current offset of
        the open-loop lateral axis, accumulated only during the hold (0 = off).
      * ``settle_px`` -- settle gate (0 = off). When > 0, a tick only counts toward
        the stable-frame exit if the worst error is in-band AND barely moving
        (|Δworst| <= settle_px) -- so align exits SETTLED on target (like move's
        continuously-held lateral) instead of mid-pass through the band. Keyed on
        error velocity, so a steady current does not block it (that is ``ki_lat``'s
        job). Use it when an align must END accurate (terminal locks).

    Forward range-hold axis (``fwd_fill`` > 0; the unified standoff shot):
      align ALSO drives forward toward ``fwd_fill`` (fraction of frame, measured by
      ``fwd_mode``), ONE-SIDED -- drive while the bbox is smaller than the standoff,
      neutral at/past it. Forward joins the in-band stable/fire gate, so a mid-hold
      fire only leaves once lat/yaw AND the standoff range are all satisfied. With
      ``fwd_fill`` = 0 (default) there is no forward axis and behaviour is unchanged.

    How one tick runs: `_Align.run`, one method per stage.
    """
    c = SimpleNamespace(**locals())   # every argument, by name, for _Align
    bad = axes - VALID_AXES
    if bad:
        raise ValueError(f"align_loop: unknown axes {sorted(bad)}")
    if not axes:
        raise ValueError("align_loop: at least one axis required")
    # The board takes no depth setpoint over this wire, so nothing here may
    # pretend to move one. vision_verbs refuses both with the reason first.
    if 'depth' in axes and not downward:
        raise ValueError("align_loop: 'depth' is a downward-camera (surge) axis")
    if downward and float(fwd_fill) > 0.0:
        raise ValueError("align_loop: fwd_fill on the downward camera is a "
                         "depth descent, which the board does not take")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    return _Align(c).run()


class _Command(NamedTuple):
    """What one align tick asks of the thrusters, before freshness decay."""
    lat_pct: float
    yaw_pct: float
    fwd_pct: float
    p_lat: float          # the proportional part of lat_pct -> the brake proxy
    in_band: list         # one bool per active axis
    worst: float          # worst per-axis error, px


class _Align(_VisionLoop):
    """`align_loop`'s state, and one method per stage of a tick."""

    verb = 'align'

    def __init__(self, c) -> None:
        super().__init__(c)
        # Per-axis speed caps: each falls back to the global `gain` when unset, so
        # `align(gain=30)` caps every axis at 30% while `align(gain=40, yaw_gain=15)`
        # slows only yaw. (Mirrors the kp_* `or DEFAULT` idiom at the call site.)
        self.g_lat = c.gain if c.gain_lat is None else c.gain_lat
        self.g_yaw = c.gain if c.gain_yaw is None else c.gain_yaw
        self.g_fwd = c.gain                      # forward range-hold capped by the global gain
        self.use_fwd = float(c.fwd_fill) > 0.0   # optional forward standoff axis
        self.use_surge = 'depth' in c.axes       # downward image-Y -> surge fore/aft

        self.residual_px = float('inf')
        self.stable = 0
        self.aligned_at: Optional[float] = None   # monotonic of FIRST stable -> hold-window start
        self.fired = False          # on_locked fired once at fire_t into the hold (payload mid-hold)
        self.sq_warned = False      # one line per command, not one per tick
        self.standoff_held = False
        self.prev_worst: Optional[float] = None   # settle gate: worst error last NEW frame
        self.stable_since = float('inf')          # arrival of the FIRST frame of the current in-band run
        self.last_live_at = float('-inf')         # last LIVE (non-coasted) sighting -> fire_pass gate
        self.lat_ema = 0.0     # trailing EMA of the signed lateral command -> brake proxy
        self.surge_ema = 0.0   # downward: trailing EMA of the signed surge command -> brake proxy
        self.lat_i = 0.0       # lateral integral accumulator (Layer 2; 0 unless ki_lat>0)
        # The mixer's own limit, which the existing anti-windup cannot see.
        # `abs(p_lat + lat_i) < g_lat` watches the SOFTWARE gain cap. The board
        # scales a whole thruster group down when any motor in it passes full
        # (`mixer.cpp`), and it never reports that -- so with yaw already spending
        # horizontal authority the hull can be clipping at an effective lateral
        # well below `g_lat`, while the integrator happily winds against a wall it
        # has no way to detect. Carried from the previous tick because yaw is
        # computed after lateral in this loop; one tick of lag on a limit that
        # persists for many is immaterial.
        self.was_saturated = False
        # Nominal tick. Used to SEED the measured dt below and as its ceiling;
        # the loop does not sleep a fixed period (see `_tick`), so treating this
        # as the real interval would misstate both the lock gate and the integral
        # term the moment the loop starts running at detection rate.
        self.dt_nominal = 1.0 / LOOP_HZ
        self.dt = self.dt_nominal
        self._last_pass = time.monotonic()
        # Seeded at the control period -- the shortest gap possible, so the gate
        # starts at its tightest and widens to the real detection interval once one
        # has been observed. Erring tight before acquisition is safe: there is no
        # lock to drop yet.
        self.gate_norm = _lock_gate(self.dt) if c.lock_on else 0.0
        self.last_accept_t: Optional[float] = None
        self.locked_ex: Optional[float] = None   # last-accepted centre -> continuity lock
        self.locked_ey: Optional[float] = None

        # Effective deadband: clamp to a physical floor so an over-tight err can't
        # perpetually TIMEOUT on bbox jitter. err_px here is already post-coercion
        # (commands.fields_for turned a rosidl-0 "unset" into the default/param), so
        # this only ever floors a genuinely tiny positive request -- logged, never
        # silent, so 'aligned (Npx)' is always read against the real deadband.
        self.eff_err = max(float(c.err_px), MIN_ALIGN_ERR_PX)
        floored = self.eff_err > float(c.err_px)
        fwd_note = (f" fwd>={c.fwd_fill * 100:.0f}%({c.fwd_mode})"
                    if self.use_fwd else "")
        self.log.info(
            f"[VIS  ] align class={c.target_class!r} axes={sorted(c.axes)}{fwd_note} "
            f"err={self.eff_err:.0f}px"
            f"{' (floored from %.0f)' % c.err_px if floored else ''} "
            f"gain={c.gain:.0f}% dur={c.duration:.0f}s hold={c.hold_s:.0f}s")
        if c.downward:
            # Loudly announce the rotated frame so the operator never mistakes a
            # downward align's axis meanings for the forward ones (see docstring).
            self.log.info(f"[VIS  ] downward frame: image-X->lateral, image-Y->surge "
                          f"(sign {c.surge_sign:+d}); depth held by the board")
        # The hold, the deadline and the log throttle count from here.
        self.started = time.monotonic()
        self.deadline = self.started + max(c.duration, 0.0)

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
        """One tick: check the clock, sense, command, count, and maybe finish."""
        c = self.c
        # MEASURED, not assumed. The loop wakes on a new detection now, so
        # the interval is the detection interval when targets are visible
        # and the tick timeout when they are not. `dt` feeds the
        # continuity-lock gate and the lateral integral; holding it at the
        # nominal period would overstate both by up to 1.5x once the loop
        # starts running at 77 Hz instead of 50.
        #
        # Clamped: a debugger pause or a scheduling stall must not inject a
        # huge dt into an integrator. The floor keeps a burst of two
        # detections in the same millisecond from collapsing it to zero.
        self.dt = min(max(now - self._last_pass, 1e-3), self.dt_nominal * 4.0)
        self._last_pass = now
        if c.abort_fn and c.abort_fn():
            return self.outcome(ABORTED, "aborted", now)
        if now >= self.deadline:
            reason = ("not aligned (duration elapsed)" if self.seen
                      else self.never_seen())
            self.pass_fire()   # opt-in guaranteed shot if the target was seen live
            return self.outcome(TIMEOUT, reason, now)

        sample = self.sense()
        if not _present(sample):
            self.stable = 0
            self.lat_i = 0.0   # bleed integral windup while blind
            self.drive(0.0, 0.0)
            return self.lost(now)

        self.acquire(sample, now)
        cmd = self.command(sample)
        self.residual_px = cmd.worst
        # Freshness-decay: pace LATERAL authority to measurement freshness so
        # the loop doesn't blind-drive on a stale bbox between slow frames
        # (yaw excluded; the board holds depth). At healthy
        # FPS fresh==1.0 so this is a no-op. A COASTED sample decays on the
        # coast curve instead (gap decay), not freshness -- see _authority.
        fresh = _authority(sample, c.coast_s, self.det_interval, self.pipe_age)
        _warn_low_fps(self.log, fresh, sample)   # F3: surface FPS-starvation, don't stall silently
        lat_pct = cmd.lat_pct * fresh
        fwd_pct = cmd.fwd_pct * fresh   # forward shares the freshness/coast decay (never braked)
        # What the mixer will do to what we just asked for. Computed from
        # OUR demand and the frame's published matrix, so it needs nothing
        # from the board -- which is the only reason it is available at all.
        self.was_saturated = (_mixer_saturated(lat_pct, cmd.yaw_pct, fwd_pct)
                              if c.mixer_aware else False)
        self.drive(lat_pct, cmd.yaw_pct, fwd_pct)
        # Brake EMA tracks the PROPORTIONAL command only (a travel-momentum
        # proxy), NOT the full lat_pct: a hull holding STILL against a steady
        # current carries a nonzero integral (lat_i) but ~0 motion, so
        # including it would make the arrival brake reverse-kick a stationary
        # hull off the spot it was holding. Exclude lat_i here.
        self.lat_ema = _ema_step(self.lat_ema, cmd.p_lat * fresh, self.dt)
        if self.use_surge:
            # Downward surge is open-loop timed thrust (like lateral) -- track
            # its EMA so the arrival brake bleeds the fore/aft coast, or the hull
            # sails past the bin. fwd_pct already carries the signed, fresh-decayed
            # surge command, so the EMA is the true momentum proxy and the brake
            # reverse-kicks opposite it regardless of surge_sign.
            self.surge_ema = _ema_step(self.surge_ema, fwd_pct, self.dt)

        is_new_frame = self.count_frame(sample, now, cmd)
        if self.held_long_enough():
            out = self.hold(sample, now, is_new_frame, cmd.worst)
            if out is not None:
                return out

        if self.log_due(now):
            # The detector node owns the always-on operator bearing line, so
            # this per-verb copy is demoted to debug to avoid a duplicate in
            # the mission terminal. Worded as live offset, not a verdict.
            x_off = sample.ex * self.half_w
            y_off = sample.ey * self.half_h
            self.log.debug(
                f"[ offset lat={x_off:+.0f} depth={y_off:+.0f}px ] "
                f"'{c.target_class}' -> err {cmd.worst:.0f}/{self.eff_err:.0f}px")
        return None

    # -- the stages ----------------------------------------------------- #

    def drive(self, lat_pct: float, yaw_pct: float, fwd_pct: float = 0.0) -> None:
        """One MANUAL_CONTROL frame. ``fwd_pct`` is the forward range-hold
        (or the downward surge); 0 unless that axis is active."""
        _srot_drive(self.c.fc, fwd_pct=fwd_pct, lat_pct=lat_pct, yaw_pct=yaw_pct,
                    prioritise=self.c.mixer_aware)

    def sense(self):
        """The target box this tick.

        Continuity lock: once acquired, prefer the box NEAREST the last
        centre (within gate_norm) over the largest, so a 2nd hole / spurious
        box can't steal the aim. near=None (pre-acquire) or gate_norm=0
        (lock_on off) -> largest-area, unchanged. ctrl_conf gates low-score
        boxes out of the control target.
        """
        c = self.c
        near = (self.locked_ex, self.locked_ey) if self.locked_ex is not None else None
        return c.vision_state.bbox_error(
            c.target_class, near=near, gate_norm=self.gate_norm,
            min_score=c.ctrl_conf, locked_id=self.locked_id, coast_s=c.coast_s,
            lock_s=c.lock_s)

    def acquire(self, sample, now: float) -> None:
        """The target is present: remember which one, and where it is."""
        self.seen = True
        self.lost_since = None
        if not sample.coasted:
            self.last_live_at = now   # last LIVE sighting -> fire_pass recency gate
        # Re-arm the continuity lock on the accepted box (used as `near` next
        # tick) ONLY when lock_on -- otherwise leave near=None so selection
        # stays largest-area. After a real loss the loop exits LOST, so a
        # fresh verb call re-acquires largest -- no stale lock survives.
        if self.c.lock_on:
            self.locked_ex, self.locked_ey = sample.ex, sample.ey
        # Capture the tracker id of a LIVE box so a later gap coasts the right
        # target (VisionState only fills track_id when coast_s>0; a coasted
        # sample keeps the existing id). No-op when coasting is off.
        if not sample.coasted and sample.track_id >= 0:
            self.locked_id = sample.track_id
        # Signed from-centre offset of the target THIS tick: the end-position
        # the verb returns, and the live value report_fn streams to feedback.
        self.report(sample)

    def command(self, sample) -> _Command:
        """The per-axis P(+I) law: what each active axis asks for, and is it in band."""
        c = self.c
        half_w, half_h = self.half_w, self.half_h
        # Range-adaptive gain: soften lat/surge kp as the bbox fills the frame
        # (close) so the 20 kg hull doesn't overshoot a small target.
        rgain = _range_gain(_fill(sample, 'area'), c.range_gain_floor)
        yaw_pct = lat_pct = p_lat = fwd_pct = 0.0
        in_band = []
        worst = 0.0

        # Where the TOOL points, not the camera (see _tool_offset).
        tool_du, tool_dv = _tool_offset(c.tool_offset_fn, self.log)

        # Horizontal bearing NOW, not at capture. Forward camera only: a
        # yaw spins the downward image about its centre rather than
        # shifting it. Never on a coasted box -- the tracker already
        # extrapolated it. Absent rate or focal = the historical behaviour.
        ex_now = sample.ex
        if c.derotate_fn is not None and not c.downward and not sample.coasted:
            try:
                _dr = c.derotate_fn()
            except Exception:                             # noqa: BLE001
                _dr = None
            if _dr is not None:
                ex_now = _derotated_ex(sample.ex, sample.age_s,
                                       float(_dr[0]), float(_dr[1]), half_w)

        if 'lat' in c.axes:
            ctrl = _axis_ctrl(ex_now, c.offsets.get('lat', 0.0), tool_du, half_w)
            epx = abs(ctrl) * half_w
            worst = max(worst, epx)
            p_lat = ctrl * c.kp_lat * rgain
            # Lateral integral (Layer 2): Lateral is open-loop, so only the
            # integral nulls a steady-current offset. Accumulate ONLY during
            # the hold (aligned_at set), conditional on the output not being
            # saturated (anti-windup), clamped. ki_lat=0 -> exactly P.
            if c.ki_lat > 0.0 and self.aligned_at is not None \
                    and abs(p_lat + self.lat_i) < self.g_lat \
                    and not self.was_saturated:
                self.lat_i = _clamp(self.lat_i + c.ki_lat * ctrl * self.dt,
                                    -c.i_lat_max, c.i_lat_max)
            lat_pct = _clamp(p_lat + self.lat_i, -self.g_lat, self.g_lat)
            in_band.append(epx <= self.eff_err)

        if 'yaw' in c.axes:
            ctrl = _axis_ctrl(ex_now, c.offsets.get('yaw', 0.0), tool_du, half_w)
            epx = abs(ctrl) * half_w
            worst = max(worst, epx)
            yaw_pct = _yaw_command(ctrl, epx, self.eff_err, c.kp_yaw, self.g_yaw,
                                   _fill(sample, 'area'))
            in_band.append(epx <= self.eff_err)

        if self.use_surge:
            # DOWNWARD: the 'depth' axis (image-Y) drives SURGE fore/aft.
            # Two-sided P modelled on the lat axis (same kp/cap/rgain), so the
            # hull drives forward when the bin is ahead and BACK when behind --
            # the one-sided fwd/fill law can't back up and would never centre.
            # surge_sign flips polarity for the physical mount (verify disarmed).
            ctrl = _axis_ctrl(sample.ey, c.offsets.get('depth', 0.0), tool_dv, half_h)
            epx = abs(ctrl) * half_h
            worst = max(worst, epx)
            p_surge = ctrl * c.kp_lat * rgain
            fwd_pct = _clamp(p_surge, -self.g_fwd, self.g_fwd) * c.surge_sign
            in_band.append(epx <= self.eff_err)
        if self.use_fwd:
            fwd_pct, fwd_ok, self.fill, self.standoff_held = _forward_range_hold(
                sample, fwd_fill=c.fwd_fill, fwd_mode=c.fwd_mode,
                kp_forward=c.kp_forward, g_fwd=self.g_fwd,
                standoff_max_tilt_deg=c.standoff_max_tilt_deg,
                obliquity_fn=c.obliquity_fn, standoff_held=self.standoff_held,
                log=self.log)
            in_band.append(fwd_ok)
        return _Command(lat_pct, yaw_pct, fwd_pct, p_lat, in_band, worst)

    def count_frame(self, sample, now: float, cmd: _Command) -> bool:
        """Advance the stable-frame counter on a NEW frame; True if this was one.

        Distinct-detection gate: advance the stable-frame counter on new
        DETECTIONS, not loop ticks (see _FRAME_EPS_S). A re-read HOLDS the
        counter (neither advances nor resets) and only a genuinely new frame
        moves it. This makes align_stable_frames mean "N distinct in-band
        frames", FPS-independent, so a lucky single frame can't declare
        ALIGNED (or arm the fire) at low detector FPS.

        Settle gate (opt-in): require the hull to also be barely MOVING, not
        just in-band, before a frame counts toward the stable-frame exit -- so
        align ends settled on target (like move's held lateral) instead of
        mid-pass through the band. Keyed on |Δworst| (error velocity), so a
        steady current that holds a non-zero command at a perfect lock does
        NOT block it. settle_px=0 -> settled always True -> legacy behaviour.
        """
        c = self.c
        sampled_at, is_new_frame = self.note_frame(sample, now)
        if not is_new_frame:
            return False
        if c.lock_on:
            # Widen (or tighten) the gate to the interval actually
            # observed between accepted frames, floored at the control
            # period so a duplicate timestamp cannot collapse it to 0.
            if self.last_accept_t is not None:
                self.gate_norm = _lock_gate(
                    max(self.dt, sampled_at - self.last_accept_t))
            self.last_accept_t = sampled_at
        if self.stable == 0:
            # Start of a fresh in-band run. Stamped on the FRAME's own
            # arrival, not on `now`: `now` is a control tick and at
            # 50 Hz control over a 15 Hz camera the two differ by most
            # of a frame period, which would credit the dwell with time
            # the target was not actually in band.
            self.stable_since = sampled_at
        # Compared against the previous NEW frame's error (re-reads are
        # numerically identical and would trivially pass), so |Δworst| is a
        # real cross-frame error velocity.
        settled = (c.settle_px <= 0.0 or self.prev_worst is None
                   or abs(cmd.worst - self.prev_worst) <= c.settle_px)
        self.prev_worst = cmd.worst
        self.stable = self.stable + 1 if (all(cmd.in_band) and settled) else 0
        return True

    def held_long_enough(self) -> bool:
        """Both gates, for the reasons at _ALIGN_STABLE_MIN_S: enough distinct
        frames AND enough elapsed time across them."""
        return (self.stable >= self.c.align_stable_frames
                and self.last_frame_at - self.stable_since >= _ALIGN_STABLE_MIN_S)

    def hold(self, sample, now: float, is_new_frame: bool,
             worst: float) -> Optional[Outcome]:
        """Centred: open the hold window, fire if due, and finish when it closes.

        First confirmed-centred tick opens the hold window. With hold_s>0 we
        keep the loop ALIVE and correcting for hold_s -- the ACTIVE
        station-keep the operator needs to hold steady against water inertia
        while a payload fires (passive neutral would just drift). hold_s<=0
        keeps the original exit-on-stable behaviour bit-for-bit.

        ponytail: aligned_at is set ONCE and never reset (unlike move_loop's
        reached_at). Align drift is transient jitter the loop corrects;
        resetting on a drift-out tick would risk never completing the hold
        under a steady current. The in-band gate on this stage still
        guarantees we exit centred. Do NOT "fix" this to match move.
        """
        c = self.c
        if self.aligned_at is None:
            self.aligned_at = now
            if c.hold_s > 0.0:
                self.log.info(f"[VIS  ] align HELD -- station-keeping "
                              f"{c.hold_s:.1f}s ({worst:.0f}px)")
        self.fire_if_due(sample, now, is_new_frame)
        if c.hold_s <= 0.0 or (now - self.aligned_at) >= c.hold_s:
            return self.finish(now, worst)
        # Inside the hold window: keep correcting (this tick's drive already ran).
        return None

    def fire_if_due(self, sample, now: float, is_new_frame: bool) -> None:
        """The mid-hold payload shot, BEFORE the exit check.

        Fires while the loop is still correcting (not on the drifting exit
        tick). on_locked is non-blocking (spawns a thread) so a slow payload
        reconnect can't stall the 20 Hz station-keep. fire_t is clamped
        < hold_s upstream, so this trips while still holding.

        Freshness guard: fire ONLY on the tick a genuinely NEW, non-coasted
        detection lands (is_new_frame -- the same distinct-frame signal that
        gates `stable`). This guarantees freshness at ANY detector FPS (a new
        frame is by definition current), fixing the low-FPS miss where the old
        `age_s <= 0.10s` window rarely coincided with the aligned tick at
        3-4 Hz. It is ALSO strictly safer: a FROZEN detector never produces a
        new frame, so a stale cached box (or a Kalman-coasted one) can never
        fire even while `stable` stands held at threshold through a mid-hold
        freeze. A torpedo leaves on a real live sighting.
        """
        c = self.c
        fire_fresh = is_new_frame and not sample.coasted
        # AIM GATE (opt-in; 0.0 = off = byte-identical to before).
        # `fire_max_tilt_deg` is how far the TARGET'S FACE may be
        # tilted from square to our shot axis and still allow a shot --
        # not a vehicle attitude, and not a pixel error.
        # A round leaves along the hull's axis, so a centred box is not
        # enough -- fired 30 deg off-normal it misses an opening it was
        # perfectly centred on. `tilt_gate_fn` must account for the planar
        # FLIP AMBIGUITY (both branches inside tolerance), and it
        # answers False when there is no pose at all: for a firing gate
        # the fail-safe direction is DO NOT FIRE.
        #
        # Deliberately does NOT set `fired`: a refusal here is "not
        # yet", so a hull that squares up later in the hold still gets
        # its shot.
        square_ok = True
        if c.fire_max_tilt_deg > 0.0 and fire_fresh and not self.fired:
            try:
                square_ok = bool(c.tilt_gate_fn and c.tilt_gate_fn(c.fire_max_tilt_deg))
            except Exception as exc:   # noqa: BLE001
                self.log.error(f"[VIS  ] tilt_gate_fn raised {exc!r} -- refusing")
                square_ok = False
            if not square_ok and not self.sq_warned:
                self.sq_warned = True
                self.log.info(f"[VIS  ] fire HELD: target not square within "
                              f"{c.fire_max_tilt_deg:.0f} deg (or no pose)")
        if c.on_locked is not None and not self.fired and fire_fresh and \
                square_ok and \
                (now - self.aligned_at) >= c.fire_t:
            self.fired = True
            try:
                c.on_locked()
            except Exception as exc:   # noqa: BLE001 -- fire must not kill the loop
                self.log.error(f"[VIS  ] on_locked (fire) raised {exc!r}")

    def pass_fire(self) -> None:
        """fire_pass fallback: on a NATURAL exit (TIMEOUT / hold-complete) without a
        strict mid-hold fire, actuate the payload anyway so a detected-but-not-fully-
        aligned run still scores. Gated on a LIVE (non-coasted) sighting within
        lost_grace_s -- never fires into empty water or on a Kalman ghost. One-shot
        (guarded by ``fired``). Opt-in via fire_pass; no-op when off."""
        c = self.c
        if (not c.fire_pass or c.on_locked is None or self.fired
                or (time.monotonic() - self.last_live_at) > c.lost_grace_s):
            return
        self.fired = True
        self.log.info("[VIS  ] fire_pass -- target seen; firing at command end "
                      "(alignment not required)")
        try:
            c.on_locked()
        except Exception as exc:   # noqa: BLE001 -- fire must not kill the loop
            self.log.error(f"[VIS  ] on_locked (fire_pass) raised {exc!r}")

    def finish(self, now: float, worst: float) -> Outcome:
        """Arrival / hold complete: brake, stop, and grade the result.

        Bleed lateral inertia so the hull stops square and the next mission
        step starts from the planned position. Gated on the EMA, so a
        gently-converged lock (hole-lock) exits with ~0 EMA and is NOT kicked.
        Yaw never brakes.
        """
        c = self.c
        if c.brake and 'lat' in c.axes:
            _brake_axis(c.writers.lateral, self.lat_ema, c.brake_gain,
                        abort_fn=c.abort_fn, log=self.log, label='VBRK')
        # Downward: also bleed the SURGE coast (fore/aft) so the hull
        # stops square over the bin instead of sailing past. Surge is open-loop
        # timed thrust coasts exactly like lateral -- same self-gating brake.
        if c.brake and self.use_surge:
            _brake_axis(c.writers.forward, self.surge_ema, c.brake_gain,
                        abort_fn=c.abort_fn, log=self.log, label='VBRK')
        c.writers.neutral()
        # If a mid-hold strict fire never landed (e.g. hold completed but
        # freshness never coincided), fire_pass still actuates here on a
        # live+recent sighting. No-op when off or already fired.
        self.pass_fire()
        # State the deadband next to the residual so 'aligned (Npx)' is never
        # misread as "should have been 0" -- N is within the eff_err deadband
        # by construction.
        reason = (f"held {c.hold_s:.1f}s ({worst:.0f}/{self.eff_err:.0f}px)"
                  if c.hold_s > 0.0
                  else f"aligned ({worst:.0f}/{self.eff_err:.0f}px)")
        if not c.verify:
            return self.outcome(ALIGNED, reason, now)
        return _verify_after_stop(
            c.vision_state, c.target_class,
            near=((self.locked_ex, self.locked_ey)
                  if self.locked_ex is not None else None),
            gate_norm=self.gate_norm, ctrl_conf=c.ctrl_conf,
            locked_id=self.locked_id, tool_offset_fn=c.tool_offset_fn,
            axes=c.axes, offsets=c.offsets, half_w=self.half_w,
            half_h=self.half_h, depth_axis=self.use_surge,
            frames_needed=c.align_stable_frames, eff_err=self.eff_err,
            worst=worst, fill=self.fill, started=self.started,
            end_px=(self.end_x_px, self.end_y_px), reason=reason,
            abort_fn=c.abort_fn, log=self.log)


