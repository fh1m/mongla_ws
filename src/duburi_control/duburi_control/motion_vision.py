#!/usr/bin/env python3
"""Vision-driven closed-loop motion -- the cousin of motion_forward.

One function does all four axes (yaw, lat, depth, forward). The caller
picks WHICH axes are live via the `axes` set and supplies gains; the
loop reads `vision_state.bbox_error(target_class)` each tick and writes
ArduSub setpoints accordingly.

Why one function, not four:
  Per-axis loops would copy 80% of the same code (timing, stale check,
  neutral fallback, exit conditions). Splitting wouldn't make any axis
  easier to read; it would just spread the synchronization across four
  files. The convenience verbs in `Duburi` (`vision_align_yaw` etc) are
  one-line wrappers that pin `axes` -- the lean shape lives there.

Design
------
Per tick (20 Hz default):
  1. ask `vision_state.bbox_error(target_class)` for a Sample
        Sample carries: ex (-1..+1), ey (-1..+1), h_frac (0..1), age_s
  2. if Sample is None or age_s > stale_after :
        write neutral RC, freeze depth setpoint, count 'lost' ticks
        on_lost='fail' -> bail out once lost_count > LOST_TICK_BUDGET
        on_lost='hold' -> keep going until duration runs out
  3. else (good detection):
        per active axis: compute one P-step, clamp to safety limits
        write Ch5 (forward) + Ch6 (lateral) + Ch4 (yaw) in ONE packet
        depth runs at 5 Hz on its own sub-tick (SET_POSITION_TARGET)
        track 'in deadband' streak; success when streak reaches
        SETTLED_TICK_BUDGET on every active axis at once
  4. cleanup (always): write neutral, log composite error.

Math reference
--------------
Image-frame error (forward camera, BGR pixel coords; cy grows downward):
  ex = (cx - W/2) / (W/2)   in [-1, +1]   positive = target is RIGHT
  ey = (cy - H/2) / (H/2)   in [-1, +1]   positive = target is BELOW
  h_frac = bbox_h / H       in [0,  1]    bigger = closer (assumes fixed
                                          target size)

Per-axis output:
  yaw_pct  = clamp(-ex * Kp_yaw,  +/- YAW_PCT_MAX)             -> Ch4
                                          (negated: Ch4 > 1500 = yaw LEFT,
                                           so target-right needs Ch4 < 1500)
  lat_pct  = clamp(ex * Kp_lat,   +/- LAT_PCT_MAX)             -> Ch6
  fwd_pct  = clamp((target_h_frac - h_frac) * Kp_forward,
                   +/- FWD_PCT_MAX)                             -> Ch5
  depth nudge  = clamp(ey * Kp_depth, +/- MAX_DEPTH_NUDGE)
  target_depth = current_depth_setpoint - depth_nudge          (forward cam)

Depth integrates incrementally (no step jumps that could trip ALT_HOLD).
For a downward camera, the same maths apply but the AXIS MEANINGS change
(ex -> lat, ey -> forward, h_frac -> depth). The caller picks the axis
mapping by composing `axes` accordingly; the loop itself stays identical.

The PI(D) hook (`visual_pid=True`) is structural in v1 -- we read the
flag and reserve gains, but the body is P-only. Adding I + D in v2 means
extending `_PerAxisPID` below; the surface (Move.action, Duburi facade,
DuburiClient) does not need to change.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Set

from .pixhawk import Pixhawk


# ---- Default knobs (override per-call from Duburi facade) ------------- #
DEFAULT_DEADBAND       = 0.18      # matches vision.deadband in vision_tunables.yaml
DEFAULT_STALE_AFTER      = 2.5     # seconds; older detection is "lost"
# 3.0 s patience at LOOP_HZ=20 = 60 ticks. Pool conditions routinely produce
# 2-3 s blackouts (turbidity, occlusion, frame drop). Stale 2.5 s + patience
# 3.0 s = ~5.5 s total gap tolerance before a mission call fails.
# Override per-call via lost_patience_s for stricter/looser bounds.
DEFAULT_LOST_PATIENCE_S  = 3.0
# 2 ticks at LOOP_HZ=20 = 0.1 s. Deadband is the primary noise filter;
# the tick budget just prevents a single on-target frame from claiming
# success. With the widened deadband below this is still safe.
SETTLED_TICK_BUDGET    = 2

KP_YAW_DEFAULT         = 60.0;   YAW_PCT_MAX     = 35.0
KP_LAT_DEFAULT         = 60.0;   LAT_PCT_MAX     = 35.0
KP_FORWARD_DEFAULT     = 200.0;  FWD_PCT_MAX     = 50.0
KP_DEPTH_DEFAULT       = 0.05;   MAX_DEPTH_NUDGE = 0.02   # m / 5 Hz tick

from .motion_rates import VISION_LOOP_HZ as LOOP_HZ
from .motion_rates import DEPTH_SETPOINT_HZ as DEPTH_HZ
from .motion_rates import LOG_THROTTLE_S

VALID_AXES = {'yaw', 'lat', 'depth', 'forward'}


# ---------------------------------------------------------------------- #
#  Public dataclasses                                                    #
# ---------------------------------------------------------------------- #
@dataclass
class VisionGains:
    """Per-axis gains. Keep zeros to opt the axis out of the loop."""
    kp_yaw:     float = KP_YAW_DEFAULT
    kp_lat:     float = KP_LAT_DEFAULT
    kp_depth:   float = KP_DEPTH_DEFAULT
    kp_forward: float = KP_FORWARD_DEFAULT


@dataclass
class VisionTrackResult:
    """Outcome the Duburi facade needs to build a Move.Result.

    `composite_error` is a unitless [0, +inf) score:
      sqrt(mean(per-axis-normalized-error^2)) over enabled axes.
    """
    success:         bool
    reason:          str
    composite_error: float
    last_age_s:      float
    last_h_frac:     float
    elapsed_s:       float
    settled_axes:    int = 0
    lost_ticks:      int = 0


# ---------------------------------------------------------------------- #
#  The loop                                                              #
# ---------------------------------------------------------------------- #
def vision_track_axes(*,
                      pixhawk: Pixhawk,
                      vision_state,
                      target_class: str,
                      axes: Set[str],
                      duration: float,
                      gains: VisionGains,
                      target_h_frac: float = 0.0,
                      deadband: float = DEFAULT_DEADBAND,
                      stale_after: float = DEFAULT_STALE_AFTER,
                      on_lost: str = 'fail',
                      depth_sign: int = +1,
                      depth_anchor_frac: float = 0.5,
                      lock_mode: str = 'settle',
                      distance_metric: str = 'height',
                      gate_guard: bool = False,
                      gate_guard_min_w_frac: float = 0.35,
                      pass_at: float = 0.0,
                      pass_at_gain: float = 50.0,
                      offset_x: float = 0.0,
                      offset_y: float = 0.0,
                      forward_uses_ey: bool = False,
                      stable_lock_s: float = 0.0,
                      on_stable=None,
                      lost_patience_s: float = DEFAULT_LOST_PATIENCE_S,
                      log=None,
                      writers=None,
                      visual_pid: bool = False,
                      abort_fn=None,
                      speed: float = 1.0,
                      h_frac_close: float = 0.0,
                      proximity_min_scale: float = 0.2,
                      slew_limit_pct: float = 8.0,
                      coast_ticks: int = 10,
                      search_yaw_rate_pct: float = 20.0,
                      search_lat_pct: float = 0.0,
                      search_speed: float = 0.3,
                      search_timeout_s: float = 20.0,
                      search_dwell_s: float = 1.5) -> VisionTrackResult:
    """Run the vision-driven loop until success / lost / duration.

    Parameters
    ----------
    vision_state
        Duck-typed; needs `.bbox_error(class_name) -> Sample|None` and
        `.image_size() -> (W,H)`. See duburi_manager.vision_state.VisionState.
    axes
        Subset of {'yaw','lat','depth','forward'}. Empty -> raises.
    depth_anchor_frac
        Which vertical point on the bounding box to align to the image
        centre. 0.0 = top edge, 0.5 = centre (default / current behaviour),
        1.0 = bottom edge. Values around 0.2 work well for tall objects
        (people standing upright, poles) where the bbox centre is already
        near the frame centre, making the depth error appear near zero.
        Formula: ey_used = ey + (2*anchor - 1) * h_frac
    lock_mode
        Controls when the loop exits (in addition to duration / on_lost):
        'settle' (default) -- exit as soon as all axes are centred and
                              steady. Normal "align and done" behaviour.
        'follow'           -- never exit on settle. Keep tracking until
                              duration runs out. Good for following a
                              moving target (swimmer, diver) for N seconds.
        'pursue'           -- keep driving forward (never back off) until
                              the target fills target_h_frac of the frame.
                              Exit = target reached. Good for competition
                              approach / torpedo firing runs.
    distance_metric
        Which part of the bounding box to use as the distance proxy for
        the 'forward' axis:
        'height'    (default) -- bbox height fraction. Tall objects: buoy, pole, flare.
        'width'               -- bbox width fraction. Wide horizontal objects: bar.
        'area'                -- geometric mean of width and height fractions
                                 (sqrt(w*h)). More robust for wide targets
                                 like gates and torpedo holes.
        'diagonal'            -- normalised bounding box diagonal. Best
                                 all-rounder for targets of unknown aspect ratio.
        'vis_range'           -- monocular depth estimate from depth_estimation_node
                                 (0=far, 1=close). Requires that node running.
    gate_guard
        When True, suppress the forward axis whenever the gate bbox appears
        angled (w_frac / h_frac < gate_guard_min_w_frac). The lateral and
        yaw corrections keep running so the AUV self-corrects to a
        perpendicular approach before resuming forward drive. Experimental.
    gate_guard_min_w_frac
        Aspect ratio threshold for gate_guard (default 0.35). If the gate's
        bbox width fraction divided by its height fraction falls below this
        value, the sub is approaching at too steep an angle.
    pass_at
        Position-lock trigger (default 0.0 = disabled). When the selected
        distance metric reaches this fraction, the loop freezes lateral and
        depth corrections and drives straight forward at pass_at_gain%.
        Use for a committed gate pass: close alignment at range, then commit
        once the gate fills pass_at of the frame.
    pass_at_gain
        Forward thrust % to use during the pass-through phase (default 50%).
    target_h_frac
        Required when 'forward' is in `axes`; ignored otherwise.
    depth_sign
        +1 for forward camera (positive ey -> target lower in frame ->
        sub should descend = depth more negative). -1 for downward
        camera (ey 'below' means 'in front of' the sub).
    on_lost
        'fail' -> exit with success=False once we lose the target for
                  more than lost_patience_s seconds (default 3.0 s).
        'hold' -> stay parked (neutral RC, frozen depth setpoint) until
                  duration runs out, regardless of how long we've been
                  staring at nothing.
    visual_pid
        Reserved for v2. Currently P-only in either case; logged so the
        operator knows whether the gains they set match the math being
        run.
    """
    bad_axes = axes - VALID_AXES
    if bad_axes:
        raise ValueError(f"vision_track_axes: unknown axes {sorted(bad_axes)}; "
                         f"valid are {sorted(VALID_AXES)}")
    if not axes:
        raise ValueError("vision_track_axes: 'axes' must not be empty")
    if 'forward' in axes and target_h_frac <= 0.0:
        raise ValueError(
            "vision_track_axes: target_h_frac>0 required when 'forward' in axes "
            f"(got {target_h_frac})")
    if on_lost not in ('fail', 'hold', 'search'):
        raise ValueError(f"on_lost must be 'fail', 'hold', or 'search' (got {on_lost!r})")
    if lock_mode not in ('settle', 'follow', 'pursue', ''):
        raise ValueError(f"lock_mode must be 'settle', 'follow', or 'pursue' (got {lock_mode!r})")
    if distance_metric not in ('height', 'width', 'area', 'diagonal', 'vis_range', ''):
        raise ValueError(
            f"distance_metric must be 'height', 'width', 'area', 'diagonal', or 'vis_range' "
            f"(got {distance_metric!r})")

    # Normalise empty string defaults.
    _lock_mode       = lock_mode or 'settle'
    _distance_metric = distance_metric or 'height'

    controller_label = 'PI ' if visual_pid else 'P  '   # surfaced for the operator
    log.info(
        f"[VIS  ] track class={target_class!r} axes={sorted(axes)} "
        f"deadband={deadband:.2f} target_h={target_h_frac:.2f} "
        f"anchor={depth_anchor_frac:.2f} lock={_lock_mode} dist={_distance_metric} "
        f"stale={stale_after:.2f}s on_lost={on_lost} mode={controller_label}"
        f"gate_guard={gate_guard} pass_at={pass_at:.2f}")

    # Preflight: VisionState owns CameraInfo so a (0,0) here means we
    # never saw a single info frame. Bail loudly rather than chasing a
    # divide-by-zero in the error calc.
    image_width, image_height = vision_state.image_size()
    if image_width <= 0 or image_height <= 0:
        return _build_bail_result(
            "camera_info not seen yet -- preflight should have caught this",
            elapsed=0.0)

    # Pre-normalise pixel offsets once per call. Dividing by half-width/height
    # converts px to the same [-1,+1] space as ex/ey. Clamped to [-1.5, 1.5]
    # so a huge offset doesn't produce an unbounded setpoint.
    norm_offset_x = max(-1.5, min(1.5, offset_x / (image_width  * 0.5))) if offset_x else 0.0
    norm_offset_y = max(-1.5, min(1.5, offset_y / (image_height * 0.5))) if offset_y else 0.0

    # The depth setpoint is integrated incrementally so it never jumps.
    # Seed it with whatever depth ArduSub currently reports.
    current_depth  = _read_current_depth(pixhawk) or 0.0
    depth_setpoint = current_depth

    settled_tick_streak = 0
    lost_tick_streak    = 0
    last_log_time       = 0.0
    last_depth_send     = 0.0
    last_good_sample    = None
    pass_through_active = False   # set once when pass_at triggers; never cleared
    _lost_budget        = max(1, int(lost_patience_s * LOOP_HZ))
    prev_yaw = prev_lat = prev_fwd = 0.0  # slew limiter state

    # When depth axis is active, Ch3 must stay released (65535 = NO_OVERRIDE)
    # so ArduSub's ALT_HOLD depth PID has authority over the vertical thrusters.
    # Sending Ch3=1500 (neutral stick) every 20 Hz would tell ArduSub "hold
    # current depth right now" and override our 5 Hz set_target_depth setpoint.
    throttle_ch = 65535 if 'depth' in axes else 1500

    started  = time.monotonic()
    deadline = started + max(duration, 0.0)

    try:
        while True:
            now     = time.monotonic()
            elapsed = now - started
            if abort_fn and abort_fn():
                return _build_bail_result("aborted", elapsed=elapsed)
            if now >= deadline:
                # Time's up -- treat as soft success only if we'd also
                # been continuously settled. Otherwise it's a timeout.
                if settled_tick_streak >= SETTLED_TICK_BUDGET:

                    return _build_ok_result(
                        "duration elapsed while settled",
                        elapsed, last_good_sample, settled_tick_streak,
                        lost_tick_streak, axes, deadband, target_h_frac)
                return _build_fail_result(
                    "duration elapsed without settling",
                    elapsed, last_good_sample, settled_tick_streak,
                    lost_tick_streak, axes, deadband, target_h_frac)

            sample = vision_state.bbox_error(target_class)
            sample_is_stale = (sample is None) or (sample.age_s > stale_after)

            if sample_is_stale:
                lost_tick_streak    += 1
                settled_tick_streak  = 0

                # Coast: for the first coast_ticks after detection loss, output
                # decaying control toward neutral instead of snapping immediately.
                # This avoids sharp RC transitions that can worsen the loss.
                if last_good_sample is not None and 0 < lost_tick_streak <= coast_ticks:
                    decay = (1.0 - lost_tick_streak / coast_ticks) * 0.4
                    coast_yaw = _clamp(-last_good_sample.ex * gains.kp_yaw * decay,
                                       -YAW_PCT_MAX * decay, YAW_PCT_MAX * decay) if 'yaw' in axes else 0.0
                    coast_lat = _clamp(last_good_sample.ex * gains.kp_lat * decay,
                                       -LAT_PCT_MAX * decay, LAT_PCT_MAX * decay) if 'lat' in axes else 0.0
                    pixhawk.send_rc_override(
                        forward=Pixhawk.percent_to_pwm(0.0),
                        lateral=Pixhawk.percent_to_pwm(coast_lat),
                        yaw=Pixhawk.percent_to_pwm(coast_yaw),
                        throttle=throttle_ch,
                    )
                    if (now - last_log_time) >= LOG_THROTTLE_S:
                        log.info(f"[VIS  ] COAST  decay={decay:.2f}  tick={lost_tick_streak}/{coast_ticks}")
                        last_log_time = now
                else:
                    writers.neutral()
                    if (now - last_log_time) >= LOG_THROTTLE_S:
                        age = sample.age_s if sample is not None else float('inf')
                        log.info(
                            f"[VIS  ] LOST  age={age:5.2f}s  lost_ticks={lost_tick_streak}",
                            throttle_duration_sec=LOG_THROTTLE_S)
                        last_log_time = now

                # Keep depth setpoint frozen so ALT_HOLD doesn't drift.
                if 'depth' in axes:
                    pixhawk.set_target_depth(depth_setpoint)
                    last_depth_send = now

                if on_lost == 'fail' and lost_tick_streak > _lost_budget:
                    return _build_fail_result(
                        f"target_class={target_class!r} lost "
                        f"(stale > {stale_after:.2f}s for {lost_tick_streak} ticks "
                        f"/ patience {lost_patience_s:.1f}s)",
                        elapsed, last_good_sample, settled_tick_streak,
                        lost_tick_streak, axes, deadband, target_h_frac)

                if on_lost == 'search' and lost_tick_streak > _lost_budget:
                    log.info(f"[VIS  ] SEARCH  target_class={target_class!r} lost — sweeping")
                    found = _do_search_sweep(
                        pixhawk=pixhawk, writers=writers,
                        vision_state=vision_state, target_class=target_class,
                        yaw_rate_pct=search_yaw_rate_pct,
                        lat_pct=search_lat_pct,
                        timeout_s=search_timeout_s,
                        dwell_s=search_dwell_s,
                        stale_after=stale_after,
                        throttle_ch=throttle_ch,
                        abort_fn=abort_fn, log=log)
                    if found:
                        lost_tick_streak = 0
                        prev_yaw = prev_lat = prev_fwd = 0.0
                    else:
                        return _build_fail_result(
                            f"target_class={target_class!r} not found after search sweep",
                            elapsed, last_good_sample, settled_tick_streak,
                            lost_tick_streak, axes, deadband, target_h_frac)

                time.sleep(1.0 / LOOP_HZ)
                continue

            # Fresh sample -- reset lost streak, run the controller.
            if lost_tick_streak > 0:
                prev_yaw = prev_lat = prev_fwd = 0.0  # clear coast residual on re-acquire
            lost_tick_streak = 0
            last_good_sample = sample

            # Apply pixel offsets in the normalised error space.
            # ex_ctrl/ey_ctrl are what the PID sees; sample.ex/ey stay for logging.
            ex_ctrl = max(-1.5, min(1.5, sample.ex - norm_offset_x))
            ey_ctrl = max(-1.5, min(1.5, sample.ey - norm_offset_y))

            yaw_pct = lat_pct = forward_pct = 0.0
            axes_in_deadband = []   # one bool per active axis this tick

            if 'yaw' in axes:
                yaw_pct = _yaw_pct(ex_ctrl, gains.kp_yaw)
                axes_in_deadband.append(abs(ex_ctrl) <= deadband)

            if 'lat' in axes:
                lat_pct = _lat_pct(ex_ctrl, gains.kp_lat)
                axes_in_deadband.append(abs(ex_ctrl) <= deadband)

            if 'forward' in axes:
                if forward_uses_ey:
                    # Downward camera: ey drives forward/backward (not h_frac).
                    # kp_forward sign controls direction — negate if AUV reverses.
                    forward_pct = _clamp(ey_ctrl * gains.kp_forward,
                                         -FWD_PCT_MAX, FWD_PCT_MAX)
                    axes_in_deadband.append(abs(ey_ctrl) <= deadband)
                else:
                    # 'area' handles wide targets (gates); 'diagonal' is the best
                    # all-rounder for unknown shapes; 'vis_range' uses monocular depth.
                    forward_pct, distance_error = _forward_decision(
                        sample, _distance_metric, target_h_frac,
                        gains.kp_forward, _lock_mode)
                    if distance_error is None:
                        # Suppressed: vis_range metric with no depth signal.
                        axes_in_deadband.append(False)
                        log.warning(
                            '[VIS  ] vis_range=0 (depth node offline?) -- '
                            'forward thrust suppressed',
                            throttle_duration_sec=2.0)
                    else:
                        axes_in_deadband.append(abs(distance_error) <= deadband)

            if 'depth' in axes and not pass_through_active:
                # depth_anchor_frac shifts which point on the bbox we align
                # to the image centre. At 0.5 (default) this equals sample.ey
                # exactly -- no change from original behaviour.
                # At 0.2 we align a point near the top of the box, which
                # gives a real error signal even when the bbox centre is
                # already at the frame centre (tall objects like people).
                ey_depth = ey_ctrl + (2.0 * depth_anchor_frac - 1.0) * sample.h_frac
                depth_step = _clamp(ey_depth * gains.kp_depth,
                                    -MAX_DEPTH_NUDGE, MAX_DEPTH_NUDGE) * depth_sign
                depth_setpoint -= depth_step
                axes_in_deadband.append(abs(ey_depth) <= deadband)
            elif 'depth' in axes:
                # pass_through_active: depth setpoint is frozen; still counts as "in deadband"
                ey_depth = ey_ctrl + (2.0 * depth_anchor_frac - 1.0) * sample.h_frac
                axes_in_deadband.append(abs(ey_depth) <= deadband)

            # gate_guard: suppress forward if gate bbox appears angled.
            # A gate viewed straight-on has a predictable w/h aspect ratio;
            # when the approach angle is too steep, w_frac/h_frac drops.
            # Suppress forward and let lat/yaw corrections realign the sub.
            if gate_guard and 'forward' in axes and not pass_through_active:
                aspect = sample.w_frac / max(sample.h_frac, 0.01)
                if aspect < gate_guard_min_w_frac:
                    forward_pct = 0.0

            # pass_at: once the distance metric reaches the trigger,
            # commit to a straight-through pass — freeze lat+depth corrections
            # and drive forward at pass_at_gain%.
            if pass_at > 0.0 and not pass_through_active:
                size_for_pass = _distance_size(sample, _distance_metric)
                if size_for_pass >= pass_at:
                    pass_through_active = True
                    log.info(
                        f"[VIS  ] PASS-THROUGH: size={size_for_pass:.2f} >= "
                        f"{pass_at:.2f} — freezing lat+depth, fwd={pass_at_gain:.0f}%")

            if pass_through_active:
                forward_pct = _clamp(pass_at_gain, 0.0, FWD_PCT_MAX)
                lat_pct     = 0.0
                settled_tick_streak = 0   # don't settle-exit during the pass

            # Proximity-aware gain scaling: shrink output cap as bbox grows.
            # h_frac_close=0 disables; speed=1.0 is identity.
            if not pass_through_active:
                if h_frac_close > 0.0:
                    prox_scale = max(proximity_min_scale,
                                     1.0 - (sample.h_frac / h_frac_close)
                                     * (1.0 - proximity_min_scale))
                    effective_scale = speed * prox_scale
                else:
                    effective_scale = speed
                if effective_scale < 1.0:
                    yaw_pct     = _clamp(yaw_pct,     -YAW_PCT_MAX * effective_scale, YAW_PCT_MAX * effective_scale)
                    lat_pct     = _clamp(lat_pct,     -LAT_PCT_MAX * effective_scale, LAT_PCT_MAX * effective_scale)
                    forward_pct = _clamp(forward_pct, -FWD_PCT_MAX * effective_scale, FWD_PCT_MAX * effective_scale)

            # Slew limiter: cap per-tick RC delta to damp oscillation.
            if slew_limit_pct > 0.0:
                yaw_pct     = prev_yaw + _clamp(yaw_pct - prev_yaw,     -slew_limit_pct, slew_limit_pct)
                lat_pct     = prev_lat + _clamp(lat_pct - prev_lat,     -slew_limit_pct, slew_limit_pct)
                forward_pct = prev_fwd + _clamp(forward_pct - prev_fwd, -slew_limit_pct, slew_limit_pct)
            prev_yaw, prev_lat, prev_fwd = yaw_pct, lat_pct, forward_pct

            # ONE RC packet carries Ch3 + Ch4 + Ch5 + Ch6.
            # throttle_ch is 65535 (released) when depth is active so
            # ArduSub's ALT_HOLD PID honours our set_target_depth setpoint.
            pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(forward_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=Pixhawk.percent_to_pwm(yaw_pct),
                throttle=throttle_ch,
            )

            if 'depth' in axes and (now - last_depth_send) >= 1.0 / DEPTH_HZ:
                pixhawk.set_target_depth(depth_setpoint)
                last_depth_send = now

            settled_tick_streak = (settled_tick_streak + 1
                                   if all(axes_in_deadband) else 0)

            if (now - last_log_time) >= LOG_THROTTLE_S:
                size_for_log = _distance_size(sample, _distance_metric) if 'forward' in axes else sample.h_frac
                offset_tag = (f" offset=({norm_offset_x:+.2f},{norm_offset_y:+.2f})"
                              if (norm_offset_x or norm_offset_y) else "")
                log.info(
                    f"[VIS  ] ex={sample.ex:+.2f}({ex_ctrl:+.2f}) "
                    f"ey={sample.ey:+.2f}({ey_ctrl:+.2f}){offset_tag} "
                    f"size={size_for_log:.2f} (tgt {target_h_frac:.2f})  "
                    f"yaw={yaw_pct:+5.1f}% lat={lat_pct:+5.1f}% "
                    f"fwd={forward_pct:+5.1f}% dep={depth_setpoint:+.2f}m  "
                    f"mode={_lock_mode} settled={settled_tick_streak}/{SETTLED_TICK_BUDGET}",
                    throttle_duration_sec=LOG_THROTTLE_S)
                last_log_time = now

            # pursue: exit the moment the target is close enough.
            if _lock_mode == 'pursue' and 'forward' in axes:
                size = _distance_size(sample, _distance_metric)
                if size >= target_h_frac:
                    return _build_ok_result(
                        f"pursued to target: size={size:.2f} >= {target_h_frac:.2f}",
                        elapsed, last_good_sample, settled_tick_streak,
                        lost_tick_streak, axes, deadband, target_h_frac)

            # settle / follow: exit on settle only in settle mode.
            if _lock_mode != 'follow' and _lock_mode != 'pursue':
                if stable_lock_s > 0.0:
                    stable_s = settled_tick_streak / LOOP_HZ
                    if stable_s >= stable_lock_s:
                        if on_stable is not None:
                            on_stable()
                        return _build_ok_result(
                            f"stable lock: {stable_s:.1f}s >= {stable_lock_s:.1f}s",
                            elapsed, last_good_sample, settled_tick_streak,
                            lost_tick_streak, axes, deadband, target_h_frac)
                elif settled_tick_streak >= SETTLED_TICK_BUDGET:
                    return _build_ok_result(
                        f"all axes within {deadband:.2f}",
                        elapsed, last_good_sample, settled_tick_streak,
                        lost_tick_streak, axes, deadband, target_h_frac)

            time.sleep(1.0 / LOOP_HZ)

    finally:
        # Whatever happened, leave the autopilot in a known state.
        try:
            writers.neutral()
        except Exception as exc:
            log.warning(f"[VIS  ] cleanup neutral raised: {exc!r}")


# ---------------------------------------------------------------------- #
#  vision_acquire -- "drive until target seen"                            #
# ---------------------------------------------------------------------- #
def vision_acquire(*,
                   pixhawk: Pixhawk,
                   vision_state,
                   target_class: str,
                   timeout: float,
                   drive_writer: Optional[Callable[[float], None]] = None,
                   stale_after: float = DEFAULT_STALE_AFTER,
                   writers=None,
                   log,
                   abort_fn=None) -> VisionTrackResult:
    """Block until at least one fresh detection of `target_class` arrives.

    `drive_writer(elapsed_s) -> None` is OPTIONAL. When supplied it's
    called every tick at LOOP_HZ to keep the sub moving while we wait
    (e.g. slow yaw, slow forward). Pass `None` to wait in place.

    Composing this with motion verbs is the Duburi facade's job; this
    function just owns the watch-loop so the test logic stays in one
    place.
    """
    log.info(
        f"[VIS  ] acquire class={target_class!r} timeout={timeout:.1f}s "
        f"drive={'yes' if drive_writer else 'no'}")

    started       = time.monotonic()
    deadline      = started + max(timeout, 0.0)
    last_log_time = 0.0

    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            if abort_fn and abort_fn():
                return _build_bail_result("aborted", elapsed=elapsed)
            if now >= deadline:
                return _build_bail_result(
                    f"target_class={target_class!r} not seen within "
                    f"{timeout:.1f}s",
                    elapsed=elapsed)

            sample = vision_state.bbox_error(target_class)
            if sample is not None and sample.age_s <= stale_after:
                (writers.neutral if writers is not None else pixhawk.send_neutral)()
                return VisionTrackResult(
                    success=True,
                    reason=(f"acquired class={target_class!r} after "
                            f"{elapsed:.2f}s"),
                    composite_error=0.0,
                    last_age_s=sample.age_s,
                    last_h_frac=sample.h_frac,
                    elapsed_s=elapsed)

            if drive_writer is not None:
                try:
                    drive_writer(elapsed)
                except Exception as exc:
                    log.warning(f"[VIS  ] drive_writer raised: {exc!r}")

            if (now - last_log_time) >= LOG_THROTTLE_S:
                age = sample.age_s if sample is not None else float('inf')
                log.info(
                    f"[VIS  ] acquire... age={age:5.2f}s elapsed={elapsed:.1f}s",
                    throttle_duration_sec=LOG_THROTTLE_S)
                last_log_time = now

            time.sleep(1.0 / LOOP_HZ)
    finally:
        try:
            (writers.neutral if writers is not None else pixhawk.send_neutral)()
        except Exception:
            pass


# ---------------------------------------------------------------------- #
#  Search sweep (on_lost='search')                                        #
# ---------------------------------------------------------------------- #
def _do_search_sweep(*,
                     pixhawk, writers, vision_state, target_class: str,
                     yaw_rate_pct: float, lat_pct: float,
                     timeout_s: float, dwell_s: float,
                     stale_after: float, throttle_ch: int,
                     abort_fn, log) -> bool:
    """Yaw-sweep while checking for detection. Returns True if found."""
    started = time.monotonic()
    deadline = started + timeout_s
    tick_s = 1.0 / LOOP_HZ
    dwell_ticks = max(1, int(dwell_s * LOOP_HZ))

    while time.monotonic() < deadline:
        if abort_fn and abort_fn():
            return False

        # Drive yaw (and optional lat) for dwell_ticks, checking each tick.
        for _ in range(dwell_ticks):
            if abort_fn and abort_fn():
                return False
            if time.monotonic() >= deadline:
                break
            pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(0.0),
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=Pixhawk.percent_to_pwm(yaw_rate_pct),
                throttle=throttle_ch,
            )
            sample = vision_state.bbox_error(target_class)
            if sample is not None and sample.age_s <= stale_after:
                writers.neutral()
                log.info(f"[VIS  ] SEARCH  found {target_class!r} after "
                         f"{time.monotonic() - started:.1f}s")
                return True
            time.sleep(tick_s)

    writers.neutral()
    log.info(f"[VIS  ] SEARCH  {target_class!r} not found within {timeout_s:.1f}s")
    return False


# ---------------------------------------------------------------------- #
#  Tiny pure helpers                                                      #
# ---------------------------------------------------------------------- #
def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _yaw_pct(ex: float, kp_yaw: float) -> float:
    """Horizontal error → Ch4 yaw percent.

    NEGATED: Ch4 > 1500 = yaw LEFT, so a target to the RIGHT (ex > 0) must
    produce a NEGATIVE pct (Ch4 < 1500) to yaw right toward it. Sign errors
    here are exactly what bite in the pool — keep this in one tested place.
    """
    return _clamp(-ex * kp_yaw, -YAW_PCT_MAX, YAW_PCT_MAX)


def _lat_pct(ex: float, kp_lat: float) -> float:
    """Horizontal error → Ch6 lateral percent.

    NOT negated (opposite of _yaw_pct): Ch6 > 1500 = strafe RIGHT, so a target
    to the RIGHT (ex > 0) needs a POSITIVE pct to strafe toward it. The
    yaw/lateral sign asymmetry is the `1801fe2` lateral-sign bug class — keep
    both in one tested place so the polarities can't silently drift together.
    """
    return _clamp(ex * kp_lat, -LAT_PCT_MAX, LAT_PCT_MAX)


def _forward_decision(sample, metric, target_h_frac, kp_forward, lock_mode):
    """Forward axis: returns (forward_pct, distance_error_or_None).

    distance_error is None when the axis is SUPPRESSED — i.e. metric is
    'vis_range' but there is no depth signal (`sample.vis_range <= 0.0`,
    the default when depth_estimation_node is offline). A 0.0 vis_range is
    "no signal", NOT "target far"; driving forward on it would thrust until
    timeout. The caller treats a None as not-settled + warns.
    """
    size = _distance_size(sample, metric)
    if metric == 'vis_range' and size <= 0.0:
        return 0.0, None
    distance_error = target_h_frac - size
    # 'pursue' only drives forward (lower-clamp 0) so over-close never reverses.
    lo = 0.0 if lock_mode == 'pursue' else -FWD_PCT_MAX
    return _clamp(distance_error * kp_forward, lo, FWD_PCT_MAX), distance_error


def _distance_size(sample, metric: str) -> float:
    """Return the distance proxy for the 'forward' axis.

    'height'    -- bbox height fraction (default; tall objects: buoy, pole, flare).
    'width'     -- bbox width fraction (wide horizontal objects: bars, torpedo panels).
    'area'      -- geometric mean of width and height; robust for wide targets
                   (gates, torpedo holes, anything wider than it is tall).
    'diagonal'  -- normalised diagonal; best all-rounder for unknown shapes.
    'vis_range' -- monocular depth estimate from depth_estimation_node (0=far, 1=close).
                   Requires depth_estimation_node running on the same camera.
    """
    if metric == 'vis_range':
        return sample.vis_range
    if metric == 'area':
        return (sample.h_frac * sample.w_frac) ** 0.5
    if metric == 'diagonal':
        return ((sample.h_frac ** 2 + sample.w_frac ** 2) ** 0.5) / (2 ** 0.5)
    if metric == 'width':
        return sample.w_frac
    return sample.h_frac  # 'height' (default)


def _read_current_depth(pixhawk) -> Optional[float]:
    att = pixhawk.get_attitude()
    return float(att['depth']) if att else None


def _composite_error(sample, axes, deadband, target_h_frac) -> float:
    """RMS of per-axis normalized errors over enabled axes."""
    if sample is None:
        return float('inf')
    per_axis_errors = []
    if 'yaw' in axes or 'lat' in axes:
        per_axis_errors.append(abs(sample.ex))
    if 'depth' in axes:
        per_axis_errors.append(abs(sample.ey))
    if 'forward' in axes:
        per_axis_errors.append(abs(target_h_frac - sample.h_frac))
    if not per_axis_errors:
        return 0.0
    mean_square = sum(err * err for err in per_axis_errors) / len(per_axis_errors)
    return mean_square ** 0.5


def _build_ok_result(reason, elapsed, sample, settled, lost,
                     axes, deadband, target_h_frac):
    return VisionTrackResult(
        success=True, reason=reason,
        composite_error=_composite_error(sample, axes, deadband, target_h_frac),
        last_age_s=(sample.age_s if sample else 0.0),
        last_h_frac=(sample.h_frac if sample else 0.0),
        elapsed_s=elapsed,
        settled_axes=settled, lost_ticks=lost)


def _build_fail_result(reason, elapsed, sample, settled, lost,
                       axes, deadband, target_h_frac):
    return VisionTrackResult(
        success=False, reason=reason,
        composite_error=_composite_error(sample, axes, deadband, target_h_frac),
        last_age_s=(sample.age_s if sample else float('inf')),
        last_h_frac=(sample.h_frac if sample else 0.0),
        elapsed_s=elapsed,
        settled_axes=settled, lost_ticks=lost)


def _build_bail_result(reason, elapsed):
    return VisionTrackResult(
        success=False, reason=reason,
        composite_error=float('inf'),
        last_age_s=float('inf'), last_h_frac=0.0,
        elapsed_s=elapsed)
