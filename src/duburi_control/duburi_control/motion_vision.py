#!/usr/bin/env python3
"""Vision-driven closed-loop motion -- two loops, pixel-native.

This module is the control engine behind the two mission verbs
``vision_align`` and ``vision_move``. Everything here is P-control on
**pixel error**, with ``gain`` acting as a hard max-speed cap. There are
no frame-age knobs, no on_lost policies, no lock modes -- the mission DSL
owns search/recovery via a Python ``fallback`` function.

Two loops
---------
``align_loop`` -- centre the target on the selected axes (lat / yaw /
    depth), each with an optional pixel offset. Exits when every active
    axis is within ``err_px`` for ``align_stable_frames`` ticks.

``move_loop`` -- drive forward until the target's bounding box fills the
    frame to ``fwd_fill`` (measured by ``mode`` = area/width/height),
    optionally holding a lateral pixel offset (``maintain``). Never
    re-centres yaw/depth; depth is left to ArduSub's depth-hold.

Both return an :class:`Outcome` whose ``code`` the facade copies into
``Move.Result.final_value`` so the DSL can branch without the action
client ever raising on a miss.

Sign rules (forward camera, image y grows downward)
---------------------------------------------------
  ex > 0  -> target RIGHT of centre
  ey > 0  -> target BELOW centre
  yaw : target-right -> yaw RIGHT toward it -> NO negate (same polarity as
        lateral; pool-verified -- the old `-ex` negation drove away from the
        target)
  lat : Ch6 > 1500 = strafe RIGHT, so target-right needs Ch6 > 1500 -> NO negate
  depth: target below -> descend -> depth setpoint more negative

Offsets (pixels, signed): a positive ``lat``/``yaw`` offset keeps the
target that many px to the RIGHT of centre; a positive ``depth`` offset
keeps it that many px BELOW centre. ``0`` = dead centre.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional, Set

from .pixhawk import Pixhawk
from .motion_rates import VISION_LOOP_HZ as LOOP_HZ
from .motion_rates import DEPTH_SETPOINT_HZ as DEPTH_HZ
from .motion_rates import LOG_THROTTLE_S


# ---- Outcome codes (copied to Move.Result.final_value) --------------- #
ALIGNED   = 0   # align: centred; move: bbox reached fwd_fill
LOST      = 1   # target gone past lost_grace_s (DSL runs fallback)
TIMEOUT   = 2   # duration elapsed without success
NO_CAMERA = 3   # camera_info never seen -- pipeline not up
ABORTED   = 4   # cooperative abort (goal cancelled)

_CODE_NAME = {ALIGNED: 'OK', LOST: 'LOST', TIMEOUT: 'TIMEOUT',
              NO_CAMERA: 'NO_CAMERA', ABORTED: 'ABORTED'}


# ---- Tunable defaults (P-gains live as ROS params; these are the floor) #
KP_LAT_DEFAULT     = 60.0
KP_YAW_DEFAULT     = 60.0
KP_DEPTH_DEFAULT   = 0.05
KP_FORWARD_DEFAULT = 200.0

# A detection older than this (seconds) counts as "no target this tick".
# bbox_error() returns None when the class is absent; this only catches a
# detector that has died while the last box is still cached.
_STALE_LIMIT_S = 1.0

# Depth setpoint nudge ceiling per tick at gain=100 (scaled by gain/100).
_MAX_DEPTH_NUDGE = 0.02   # metres / tick

# Pass-through commit: how long to keep driving forward AFTER the target
# leaves the frame, so the hull fully clears the gate. Overridable per
# call via move(..., hold=<s>).
_PASSTHROUGH_COMMIT_S = 2.0

# Surfacing safety floor: never command shallower than 0.2 m.
_MIN_DEPTH_M = -0.2

VALID_AXES = {'lat', 'yaw', 'depth'}
VALID_MODES = {'area', 'width', 'height'}


@dataclass
class Outcome:
    """What the facade needs to build a Move.Result.

    ``code`` is one of the module constants above. ``last_err_px`` is the
    worst per-axis pixel error at exit (align) or the lateral error
    (move). ``fill`` is the bbox fill fraction at exit (move; 0 for
    align).
    """
    code:        int
    reason:      str
    last_err_px: float = 0.0
    fill:        float = 0.0
    elapsed_s:   float = 0.0

    @property
    def ok(self) -> bool:
        return self.code == ALIGNED


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _fill(sample, mode: str) -> float:
    """Bounding-box fill fraction of the frame.

    'area'   -> geometric mean sqrt(w*h)  (1.0 when bbox fills frame)
    'width'  -> w_frac                     (wide targets: gate bar)
    'height' -> h_frac                     (tall targets: slalom pipe)
    """
    if mode == 'width':
        return sample.w_frac
    if mode == 'height':
        return sample.h_frac
    return (sample.w_frac * sample.h_frac) ** 0.5   # 'area'


def _present(sample) -> bool:
    """True when this tick has a usable detection."""
    return sample is not None and sample.age_s <= _STALE_LIMIT_S


def _read_depth(pixhawk) -> float:
    att = pixhawk.get_attitude()
    return float(att['depth']) if att else 0.0


def _camera_ready(vision_state) -> bool:
    """True once the camera pipeline has published a CameraInfo.

    Uses ``info_seen()`` when available (the real VisionState); falls back
    to a positive image size so lightweight test doubles still work. Until
    this is True the controller has no trustworthy pixel scale, so the
    verbs return NO_CAMERA rather than steer on mis-scaled pixel error.
    """
    seen = getattr(vision_state, 'info_seen', None)
    if callable(seen):
        return bool(seen())
    width, height = vision_state.image_size()
    return width > 0 and height > 0


def _live_classes(vision_state):
    """Class ids the detector is currently publishing (for the no-match hint)."""
    fn = getattr(vision_state, 'list_classes', None)
    return fn() if callable(fn) else []


# ---------------------------------------------------------------------- #
#  align_loop -- centre on lat / yaw / depth                             #
# ---------------------------------------------------------------------- #
def align_loop(*,
               pixhawk: Pixhawk,
               vision_state,
               target_class: str,
               axes: Set[str],
               offsets: Dict[str, float],
               err_px: float,
               duration: float,
               gain: float,
               kp_lat: float = KP_LAT_DEFAULT,
               kp_yaw: float = KP_YAW_DEFAULT,
               kp_depth: float = KP_DEPTH_DEFAULT,
               lost_grace_s: float = 1.0,
               hold_through_loss: bool = False,
               align_stable_frames: int = 3,
               depth_sign: int = +1,
               release_yaw: bool = False,
               writers=None,
               log=None,
               abort_fn=None) -> Outcome:
    """Hold ``target_class`` at the requested pixel offset on each active axis.

    ``axes`` is a subset of {'lat','yaw','depth'}; ``offsets`` carries the
    signed pixel offset for each active axis (0 = centre). Returns an
    Outcome -- never raises on a miss.
    """
    bad = axes - VALID_AXES
    if bad:
        raise ValueError(f"align_loop: unknown axes {sorted(bad)}")
    if not axes:
        raise ValueError("align_loop: at least one axis required")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    width, height = vision_state.image_size()
    half_w, half_h = width * 0.5, height * 0.5

    use_depth   = 'depth' in axes
    throttle_ch = 65535 if use_depth else 1500   # release Ch3 for ALT_HOLD depth PID
    max_nudge   = _MAX_DEPTH_NUDGE * max(gain, 0.0) / 100.0

    depth_setpoint = _read_depth(pixhawk)

    def _drive(lat_pct: float, yaw_pct: float) -> None:
        """Write the translation/yaw RC frame, honouring an active lock.

        When ``release_yaw`` is set the background heading lock owns Ch4,
        so touch only throttle/forward/lateral and leave yaw released --
        writing yaw=1500 here would race the lock's 20 Hz Ch4 stream
        (the same fight ``move_loop`` avoids via ``send_rc_translation``).
        """
        if release_yaw:
            pixhawk.send_rc_translation(
                throttle=throttle_ch, forward=1500,
                lateral=Pixhawk.percent_to_pwm(lat_pct))
        else:
            pixhawk.send_rc_override(
                forward=1500,
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=Pixhawk.percent_to_pwm(yaw_pct),
                throttle=throttle_ch)

    stable      = 0
    lost_since: Optional[float] = None
    last_log    = 0.0
    last_depth  = 0.0
    last_err_px = float('inf')

    log.info(
        f"[VIS  ] align class={target_class!r} axes={sorted(axes)} "
        f"offsets={ {k: round(v) for k, v in offsets.items()} } "
        f"err={err_px:.0f}px gain={gain:.0f}% dur={duration:.0f}s "
        f"hold_thru_loss={hold_through_loss}")

    started  = time.monotonic()
    deadline = started + max(duration, 0.0)
    try:
        while True:
            now     = time.monotonic()
            elapsed = now - started
            if abort_fn and abort_fn():
                return Outcome(ABORTED, "aborted", last_err_px, 0.0, elapsed)
            if now >= deadline:
                return Outcome(TIMEOUT, "not aligned (duration elapsed)",
                               last_err_px, 0.0, elapsed)

            sample = vision_state.bbox_error(target_class)
            if not _present(sample):
                stable = 0
                _drive(0.0, 0.0)
                if use_depth:
                    pixhawk.set_target_depth(depth_setpoint)
                if lost_since is None:
                    lost_since = now
                if (not hold_through_loss) and (now - lost_since) >= lost_grace_s:
                    return Outcome(LOST, f"target {target_class!r} lost",
                                   last_err_px, 0.0, elapsed)
                if (now - last_log) >= LOG_THROTTLE_S:
                    live = _live_classes(vision_state)
                    if live:
                        log.warning(
                            f"[VIS  ] align: {target_class!r} not among live "
                            f"detections {live} -- check classes filter / model")
                    else:
                        log.info(f"[VIS  ] align LOST {now - lost_since:.1f}s "
                                 f"(grace {lost_grace_s:.1f}s)")
                    last_log = now
                time.sleep(1.0 / LOOP_HZ)
                continue

            lost_since = None
            yaw_pct = lat_pct = 0.0
            in_band = []
            worst   = 0.0

            if 'lat' in axes:
                ctrl = sample.ex - offsets.get('lat', 0.0) / half_w
                epx  = abs(ctrl) * half_w
                worst = max(worst, epx)
                lat_pct = _clamp(ctrl * kp_lat, -gain, gain)
                in_band.append(epx <= err_px)

            if 'yaw' in axes:
                ctrl = sample.ex - offsets.get('yaw', 0.0) / half_w
                epx  = abs(ctrl) * half_w
                worst = max(worst, epx)
                # Same polarity as the lateral axis above (no negation): a
                # target to the RIGHT (ex > 0) yaws RIGHT toward it. The old
                # `-ctrl` negation drove the AUV away from the target -- pool-
                # verified inversion; the working lateral + heading_lock paths
                # both confirm this sign.
                yaw_pct = _clamp(ctrl * kp_yaw, -gain, gain)
                in_band.append(epx <= err_px)

            if use_depth:
                ctrl = sample.ey - offsets.get('depth', 0.0) / half_h
                epx  = abs(ctrl) * half_h
                worst = max(worst, epx)
                step = _clamp(ctrl * kp_depth, -max_nudge, max_nudge) * depth_sign
                depth_setpoint = min(depth_setpoint - step, _MIN_DEPTH_M)
                in_band.append(epx <= err_px)

            last_err_px = worst
            _drive(lat_pct, yaw_pct)
            if use_depth and (now - last_depth) >= 1.0 / DEPTH_HZ:
                pixhawk.set_target_depth(depth_setpoint)
                last_depth = now

            stable = stable + 1 if all(in_band) else 0
            if stable >= align_stable_frames:
                writers.neutral()
                return Outcome(ALIGNED, f"aligned ({worst:.0f}px)",
                               worst, 0.0, elapsed)

            if (now - last_log) >= LOG_THROTTLE_S:
                log.info(
                    f"[VIS  ] align err={worst:5.0f}px (tgt {err_px:.0f}) "
                    f"yaw={yaw_pct:+5.1f}% lat={lat_pct:+5.1f}% "
                    f"dep={depth_setpoint:+.2f}m stable={stable}/{align_stable_frames}")
                last_log = now
            time.sleep(1.0 / LOOP_HZ)
    finally:
        try:
            writers.neutral()
        except Exception as exc:   # noqa: BLE001
            log.warning(f"[VIS  ] align cleanup neutral raised: {exc!r}")


# ---------------------------------------------------------------------- #
#  move_loop -- drive forward to a bbox fill ratio                       #
# ---------------------------------------------------------------------- #
def move_loop(*,
              pixhawk: Pixhawk,
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
              kp_forward: float = KP_FORWARD_DEFAULT,
              kp_lat: float = KP_LAT_DEFAULT,
              lost_grace_s: float = 1.0,
              hold_through_loss: bool = False,
              release_yaw: bool = False,
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
      ``_PASSTHROUGH_COMMIT_S``) and report ALIGNED -- this carries the
      hull *through* the gate. If the target is never seen, falls back to
      the normal loss path (LOST -> mission fallback search).

    ``maintain_on`` holds a lateral pixel offset (``maintain_px``) while
    driving; depth and yaw are never commanded (ArduSub holds depth;
    heading lock or the autopilot holds yaw). Returns an Outcome -- never
    raises on a miss.
    """
    if mode not in VALID_MODES:
        raise ValueError(f"move_loop: mode must be one of {sorted(VALID_MODES)}")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    width, height = vision_state.image_size()
    half_w = width * 0.5

    def _drive(fwd_pct: float, lat_pct: float) -> None:
        if release_yaw:
            pixhawk.send_rc_translation(
                forward=Pixhawk.percent_to_pwm(fwd_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct))
        else:
            pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(fwd_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=1500, throttle=1500)

    seen_once    = False
    lost_since:   Optional[float] = None
    reached_at:   Optional[float] = None
    commit_until: Optional[float] = None
    last_log     = 0.0
    last_fill    = 0.0
    last_lat_err = 0.0

    # Pass-through commit window: hold_s overrides the module default.
    commit_s = hold_s if hold_s > 0.0 else _PASSTHROUGH_COMMIT_S

    log.info(
        f"[VIS  ] move class={target_class!r} "
        f"{'PASS-THROUGH commit=%.1fs' % commit_s if passthrough else 'fwd_fill=%.0f%%' % (fwd_fill * 100)} "
        f"mode={mode} maintain={'%+.0fpx' % maintain_px if maintain_on else 'off'} "
        f"gain={gain:.0f}% dur={duration:.0f}s")

    started  = time.monotonic()
    deadline = started + max(duration, 0.0)
    try:
        while True:
            now     = time.monotonic()
            elapsed = now - started
            if abort_fn and abort_fn():
                return Outcome(ABORTED, "aborted", last_lat_err, last_fill, elapsed)
            if now >= deadline:
                reason = ("passed-through window not closed (duration elapsed)"
                          if passthrough else "fill not reached (duration elapsed)")
                return Outcome(TIMEOUT, reason, last_lat_err, last_fill, elapsed)

            sample  = vision_state.bbox_error(target_class)
            present = _present(sample)

            # Pass-through trigger: the target was seen and has now left the
            # frame -- keep driving straight for the commit window so the hull
            # clears the gate, then report success.
            if passthrough and seen_once and not present:
                reached_at = None
                if commit_until is None:
                    commit_until = now + commit_s
                    log.info(f"[VIS  ] move: {target_class!r} cleared frame -- "
                             f"committing {commit_s:.1f}s to pass through")
                if now >= commit_until:
                    writers.neutral()
                    return Outcome(ALIGNED, "passed through", last_lat_err,
                                   last_fill, elapsed)
                _drive(gain, 0.0)   # no detection -> no lateral, just drive on
                time.sleep(1.0 / LOOP_HZ)
                continue

            if not present:
                reached_at = None
                _drive(0.0, 0.0)
                if lost_since is None:
                    lost_since = now
                if (not hold_through_loss) and (now - lost_since) >= lost_grace_s:
                    return Outcome(LOST, f"target {target_class!r} lost",
                                   last_lat_err, last_fill, elapsed)
                if (now - last_log) >= LOG_THROTTLE_S:
                    live = _live_classes(vision_state)
                    if live:
                        log.warning(
                            f"[VIS  ] move: {target_class!r} not among live "
                            f"detections {live} -- check classes filter / model")
                    else:
                        log.info(f"[VIS  ] move LOST {now - lost_since:.1f}s "
                                 f"(grace {lost_grace_s:.1f}s)")
                    last_log = now
                time.sleep(1.0 / LOOP_HZ)
                continue

            seen_once    = True
            lost_since   = None
            commit_until = None
            fill = _fill(sample, mode)
            last_fill = fill

            lat_pct = 0.0
            if maintain_on:
                ctrl = sample.ex - maintain_px / half_w
                last_lat_err = abs(ctrl) * half_w
                lat_pct = _clamp(ctrl * kp_lat, -gain, gain)

            if passthrough:
                # Drive forward at the speed cap until the target leaves frame.
                _drive(gain, lat_pct)
                if (now - last_log) >= LOG_THROTTLE_S:
                    log.info(f"[VIS  ] move PASS-THROUGH fill={fill * 100:5.1f}% "
                             f"fwd={gain:.0f}% lat={lat_pct:+5.1f}%")
                    last_log = now
                time.sleep(1.0 / LOOP_HZ)
                continue

            if fill >= fwd_fill:
                if reached_at is None:
                    reached_at = now
                # Station-keep: zero forward, keep correcting lateral if asked.
                _drive(0.0, lat_pct)
                if hold_s <= 0.0 or (now - reached_at) >= hold_s:
                    writers.neutral()
                    return Outcome(ALIGNED,
                                   f"reached fill={fill * 100:.0f}%",
                                   last_lat_err, fill, elapsed)
            else:
                reached_at = None
                fwd_pct = _clamp((fwd_fill - fill) * kp_forward, 0.0, gain)
                _drive(fwd_pct, lat_pct)

            if (now - last_log) >= LOG_THROTTLE_S:
                hold_tag = ("" if reached_at is None
                            else f" hold={now - reached_at:.1f}/{hold_s:.0f}s")
                log.info(
                    f"[VIS  ] move fill={fill * 100:5.1f}% (tgt {fwd_fill * 100:.0f}%) "
                    f"lat={lat_pct:+5.1f}%{hold_tag}")
                last_log = now
            time.sleep(1.0 / LOOP_HZ)
    finally:
        try:
            writers.neutral()
        except Exception as exc:   # noqa: BLE001
            log.warning(f"[VIS  ] move cleanup neutral raised: {exc!r}")
