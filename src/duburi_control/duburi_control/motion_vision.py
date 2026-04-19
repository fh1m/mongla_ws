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
  yaw_pct  = clamp(ex * Kp_yaw,   +/- YAW_PCT_MAX)             -> Ch4
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
DEFAULT_STALE_AFTER    = 1.5       # seconds; older detection is "lost"
# 40 ticks at LOOP_HZ=20 = 2.0 s. Webcams drop frames in bursts; we'd
# rather ride out a 1-2 s blackout than abort a mostly-finished command.
# Override per-call from missions if a real AUV needs stricter bounds.
LOST_TICK_BUDGET       = 40
# 2 ticks at LOOP_HZ=20 = 0.1 s. Deadband is the primary noise filter;
# the tick budget just prevents a single on-target frame from claiming
# success. With the widened deadband below this is still safe.
SETTLED_TICK_BUDGET    = 2

KP_YAW_DEFAULT         = 60.0;   YAW_PCT_MAX     = 35.0
KP_LAT_DEFAULT         = 60.0;   LAT_PCT_MAX     = 35.0
KP_FORWARD_DEFAULT     = 200.0;  FWD_PCT_MAX     = 50.0
KP_DEPTH_DEFAULT       = 0.05;   MAX_DEPTH_NUDGE = 0.02   # m / 5 Hz tick

LOOP_HZ                = 20.0      # RC override write rate
DEPTH_HZ               = 5.0       # depth setpoint write rate
LOG_THROTTLE_S         = 0.5       # seconds between [VIS  ] log lines

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
                      log,
                      writers,
                      visual_pid: bool = False) -> VisionTrackResult:
    """Run the vision-driven loop until success / lost / duration.

    Parameters
    ----------
    vision_state
        Duck-typed; needs `.bbox_error(class_name) -> Sample|None` and
        `.image_size() -> (W,H)`. See duburi_manager.vision_state.VisionState.
    axes
        Subset of {'yaw','lat','depth','forward'}. Empty -> raises.
    target_h_frac
        Required when 'forward' is in `axes`; ignored otherwise.
    depth_sign
        +1 for forward camera (positive ey -> target lower in frame ->
        sub should descend = depth more negative). -1 for downward
        camera (ey 'below' means 'in front of' the sub).
    on_lost
        'fail' -> exit with success=False once we lose the target for
                  more than LOST_TICK_BUDGET ticks.
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
    if on_lost not in ('fail', 'hold'):
        raise ValueError(f"on_lost must be 'fail' or 'hold' (got {on_lost!r})")

    controller_label = 'PI ' if visual_pid else 'P  '   # surfaced for the operator
    log.info(
        f"[VIS  ] track class={target_class!r} axes={sorted(axes)} "
        f"deadband={deadband:.2f} target_h={target_h_frac:.2f} "
        f"stale={stale_after:.2f}s on_lost={on_lost} mode={controller_label}")

    # Preflight: VisionState owns CameraInfo so a (0,0) here means we
    # never saw a single info frame. Bail loudly rather than chasing a
    # divide-by-zero in the error calc.
    image_width, image_height = vision_state.image_size()
    if image_width <= 0 or image_height <= 0:
        return _build_bail_result(
            "camera_info not seen yet -- preflight should have caught this",
            elapsed=0.0)

    # The depth setpoint is integrated incrementally so it never jumps.
    # Seed it with whatever depth ArduSub currently reports.
    current_depth  = _read_current_depth(pixhawk) or 0.0
    depth_setpoint = current_depth

    settled_tick_streak = 0
    lost_tick_streak    = 0
    last_log_time       = 0.0
    last_depth_send     = 0.0
    last_good_sample    = None

    started  = time.monotonic()
    deadline = started + max(duration, 0.0)

    try:
        while True:
            now     = time.monotonic()
            elapsed = now - started
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
                writers.neutral()
                # Keep depth setpoint frozen so ALT_HOLD doesn't drift.
                if 'depth' in axes:
                    pixhawk.set_target_depth(depth_setpoint)
                    last_depth_send = now

                if (now - last_log_time) >= LOG_THROTTLE_S:
                    age = sample.age_s if sample is not None else float('inf')
                    log.info(
                        f"[VIS  ] LOST  age={age:5.2f}s  lost_ticks={lost_tick_streak}",
                        throttle_duration_sec=LOG_THROTTLE_S)
                    last_log_time = now

                if on_lost == 'fail' and lost_tick_streak > LOST_TICK_BUDGET:
                    return _build_fail_result(
                        f"target_class={target_class!r} lost "
                        f"(stale > {stale_after:.2f}s for {lost_tick_streak} ticks)",
                        elapsed, last_good_sample, settled_tick_streak,
                        lost_tick_streak, axes, deadband, target_h_frac)

                time.sleep(1.0 / LOOP_HZ)
                continue

            # Fresh sample -- reset lost streak, run the controller.
            lost_tick_streak = 0
            last_good_sample = sample

            yaw_pct = lat_pct = forward_pct = 0.0
            axes_in_deadband = []   # one bool per active axis this tick

            if 'yaw' in axes:
                yaw_pct = _clamp(sample.ex * gains.kp_yaw,
                                 -YAW_PCT_MAX, YAW_PCT_MAX)
                axes_in_deadband.append(abs(sample.ex) <= deadband)

            if 'lat' in axes:
                lat_pct = _clamp(sample.ex * gains.kp_lat,
                                 -LAT_PCT_MAX, LAT_PCT_MAX)
                axes_in_deadband.append(abs(sample.ex) <= deadband)

            if 'forward' in axes:
                distance_error = target_h_frac - sample.h_frac
                forward_pct = _clamp(distance_error * gains.kp_forward,
                                     -FWD_PCT_MAX, FWD_PCT_MAX)
                axes_in_deadband.append(abs(distance_error) <= deadband)

            if 'depth' in axes:
                depth_step = _clamp(sample.ey * gains.kp_depth,
                                    -MAX_DEPTH_NUDGE, MAX_DEPTH_NUDGE) * depth_sign
                # ey > 0 (target below centre) for a forward camera means
                # we want to go DEEPER (more negative depth). The sign is
                # the natural one once depth_sign is folded in.
                depth_setpoint -= depth_step
                axes_in_deadband.append(abs(sample.ey) <= deadband)

            # ONE RC packet carries Ch4 + Ch5 + Ch6 -- full 3D in one shot.
            pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(forward_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=Pixhawk.percent_to_pwm(yaw_pct),
            )

            if 'depth' in axes and (now - last_depth_send) >= 1.0 / DEPTH_HZ:
                pixhawk.set_target_depth(depth_setpoint)
                last_depth_send = now

            settled_tick_streak = (settled_tick_streak + 1
                                   if all(axes_in_deadband) else 0)

            if (now - last_log_time) >= LOG_THROTTLE_S:
                log.info(
                    f"[VIS  ] ex={sample.ex:+.2f} ey={sample.ey:+.2f} "
                    f"h={sample.h_frac:.2f} (tgt {target_h_frac:.2f})  "
                    f"yaw={yaw_pct:+5.1f}% lat={lat_pct:+5.1f}% "
                    f"fwd={forward_pct:+5.1f}% dep={depth_setpoint:+.2f}m  "
                    f"settled={settled_tick_streak}/{SETTLED_TICK_BUDGET}",
                    throttle_duration_sec=LOG_THROTTLE_S)
                last_log_time = now

            if settled_tick_streak >= SETTLED_TICK_BUDGET:
                return _build_ok_result(
                    f"all axes within {deadband:.2f} for "
                    f"{settled_tick_streak} ticks",
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
                   log) -> VisionTrackResult:
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
            if now >= deadline:
                return _build_bail_result(
                    f"target_class={target_class!r} not seen within "
                    f"{timeout:.1f}s",
                    elapsed=elapsed)

            sample = vision_state.bbox_error(target_class)
            if sample is not None and sample.age_s <= stale_after:
                pixhawk.send_neutral()
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
            pixhawk.send_neutral()
        except Exception:
            pass


# ---------------------------------------------------------------------- #
#  Tiny pure helpers                                                      #
# ---------------------------------------------------------------------- #
def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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
