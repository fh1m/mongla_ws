#!/usr/bin/env python3
"""Depth-drive loop -- canonical Blue Robotics pattern.

We send an absolute depth setpoint via ``SET_POSITION_TARGET_GLOBAL_INT``
and let ArduSub's onboard 400 Hz depth controller drive Ch3. The yaw
axis is the inverse pattern: there we write Ch4 rate-overrides and
do the loop in Python because we don't trust ArduSub's compass; here
we trust ArduSub's pressure sensor + onboard depth PID, so the Python
side is just a setpoint streamer (5 Hz) plus a "did we get there"
watchdog. No Python PID stacked on top of ArduSub's loop, no Ch3 RC
override fights.

Caller is expected to have already engaged ALT_HOLD (the only mode
that honours the absolute-Z setpoint and also holds the current depth
between frames). ``Duburi.set_depth`` invokes ``_ensure_alt_hold``
first so direct callers of this function do not have to.

Two phases:

  Phase 1 -- prime_alt_hold (0.5 s)
      Stream the *current* depth as the target while sending a neutral
      RC override. This drains any stale ALT_HOLD integrator state from
      a previous mode entry (well-known ArduSub quirk where the
      depth-hold I-term is not reset on mode switch -- see the Blue
      Robotics forum thread "Depth Hold Problems?"). Without this the
      first frame can fire the vertical thrusters at full power.

  Phase 2 -- wait_for_depth (until tolerance or timeout)
      Stream the real target at 5 Hz. Poll AHRS depth; return None when
      the error falls below TOL_M; raise MovementTimeout otherwise
      (caught by the action server and surfaced as
      Move.Result.success = False).
"""

import time

from .errors import MovementTimeout


from .motion_rates import DEPTH_SETPOINT_HZ   as SETPOINT_HZ
from .motion_rates import DEPTH_RAMP_S        as RAMP_S
from .motion_rates import DEPTH_BRAKE_ZONE_M  as BRAKE_ZONE_M
from .motion_rates import DEPTH_RAMP_ADVANCE_M as RAMP_ADVANCE_M
from .motion_rates import LOG_THROTTLE_S      as LOG_THROTTLE

TOL_M         = 0.10    # exit tolerance (m). 10 cm is realistic; tighter values cause
                        # timeout when ArduSub's depth PID settles with a small residual.
PRIME_SECONDS = 0.5     # drain stale ALT_HOLD I-term before driving anywhere
# RAMP_S and BRAKE_ZONE_M are imported from motion_rates -- tune there.


def hold_depth(pixhawk, target_m, timeout, log, neutral_writer=None):
    """Drive the sub to `target_m` (negative = below surface) and hold.

    Caller MUST already be in ALT_HOLD; this function does not switch
    modes itself (that's `Duburi.set_depth`'s job).

    `neutral_writer` is an optional callable that writes "neutral" --
    defaults to `pixhawk.send_neutral` which sends 1500 on all six
    channels. Heading-lock-aware callers pass `writers.neutral` so Ch4
    stays released and the HeadingLock thread's Ch4 rate-override wins.
    """
    if neutral_writer is None:
        neutral_writer = pixhawk.send_neutral

    starting = pixhawk.get_attitude()
    prime_d  = starting['depth'] if starting is not None else target_m

    prime_alt_hold(pixhawk, hold_at=prime_d, neutral_writer=neutral_writer)

    # Re-read depth AFTER prime so the ramp starts from the actual position.
    # Reading before prime causes `going_down` to be wrong when the sub
    # drifts during the 0.5 s prime phase (e.g. positively buoyant sub
    # rising while we hold-prime a deeper depth), which makes the first
    # ramp frame command the stale pre-prime depth and briefly drive the
    # wrong direction before correcting.
    after_prime = pixhawk.get_attitude()
    start_d = after_prime['depth'] if after_prime is not None else prime_d

    # Advance the ramp start 0.5 m toward the target so ArduSub sees a real
    # depth error from tick 1. Without this, the ramp starts at start_d and
    # ArduSub's error is 0 on the first tick; the downward I-term accumulated
    # at depth then briefly pushes the sub the wrong way before the P-term
    # grows large enough to overcome it (visible as "goes deeper first" when
    # commanded to a shallower depth from a deep hold).
    if start_d is not None and abs(target_m - start_d) > RAMP_ADVANCE_M:
        if target_m > start_d:
            start_d = min(start_d + RAMP_ADVANCE_M, target_m)
        else:
            start_d = max(start_d - RAMP_ADVANCE_M, target_m)

    wait_for_depth(pixhawk, target_m, timeout, log, start_d=start_d)


def prime_alt_hold(pixhawk, hold_at, neutral_writer):
    """Phase 1: drain ArduSub's stale ALT_HOLD integrator.

    Streams `hold_at` as the depth target with a neutral RC override
    for `PRIME_SECONDS`. The autopilot's depth controller is now happy
    (already at target, zero error) so its I-term can decay before we
    ask it to drive somewhere.
    """
    deadline = time.time() + PRIME_SECONDS
    while time.time() < deadline:
        pixhawk.set_target_depth(hold_at)
        neutral_writer()
        time.sleep(1.0 / SETPOINT_HZ)


def wait_for_depth(pixhawk, target_m, timeout, log, start_d=None):
    """Phase 2: stream the real target until reached or timeout.

    Ramps the setpoint from `start_d` toward `target_m` over RAMP_S
    seconds.  Crucially, the setpoint *tracks* the sub's current depth
    when it moves faster than the ramp — this prevents ArduSub's depth
    PID from commanding the sub backward to catch up with a lagging
    setpoint (the classic "goes down, bounces back up, oscillates" symptom).

    `start_d` is offset 0.5 m toward the target by `hold_depth` before
    this function is called, so ArduSub sees a non-zero depth error from
    tick 1 and its accumulated I-term cannot briefly push the sub the
    wrong way.

    Ramp phase logic:
      - Compute the linear ramp position at elapsed time.
      - If the sub is within BRAKE_ZONE_M of the target, hold the setpoint
        at target_m (brake zone). ArduSub now has a real depth error and
        applies corrective thrust for a controlled arrival.
      - Outside the brake zone: if the sub is ahead of the ramp, advance
        the setpoint to track it so ArduSub never reverses direction to
        chase a lagging setpoint.
      - Clamp to target so the setpoint never overshoots it.

    After RAMP_S seconds, hold the setpoint at target_m until the
    sub arrives within TOL_M or the timeout expires.

    Note: vision depth alignment bypasses this ramp by design -- it
    sends incremental nudges (≤ 0.02 m per tick at 5 Hz) which are
    gentler for small corrections. This ramp is only for large
    commanded depth changes via `set_depth`.
    """
    deadline   = time.time() + timeout
    t_start    = time.time()
    closest    = None
    going_down = (start_d is not None) and (target_m < start_d)

    while time.time() < deadline:
        # Read depth BEFORE computing the setpoint so we can track the sub.
        attitude = pixhawk.get_attitude()
        current  = attitude['depth'] if attitude is not None else None

        elapsed = time.time() - t_start
        if start_d is not None and elapsed < RAMP_S:
            frac   = elapsed / RAMP_S
            ramped = start_d + (target_m - start_d) * frac
            if current is not None:
                near_target = abs(target_m - current) < BRAKE_ZONE_M
                if near_target:
                    # Brake zone: hold setpoint at target so ArduSub has a
                    # real error to work against and brakes the final approach.
                    # Without this, tracking zeros ArduSub's depth error and
                    # the sub drifts in under momentum and overshoots.
                    setpoint = target_m
                elif going_down:
                    setpoint = max(min(ramped, current), target_m)
                else:
                    setpoint = min(max(ramped, current), target_m)
            else:
                setpoint = ramped
        else:
            setpoint = target_m

        pixhawk.set_target_depth(setpoint)

        if current is not None:
            error = abs(target_m - current)
            if closest is None or error < abs(target_m - closest):
                closest = current

            log.info(
                f'[DEPTH] -> {target_m:+.2f}m  '
                f'setpt:{setpoint:+.2f}m  now:{current:+.2f}m  err:{error:.2f}m',
                throttle_duration_sec=LOG_THROTTLE)

            if error < TOL_M:
                log.info(f'[DEPTH] OK {current:+.2f}m')
                return

        time.sleep(1.0 / SETPOINT_HZ)

    if closest is not None:
        error = abs(target_m - closest)
        log.info(
            f'[DEPTH] !! timeout  closest:{closest:+.2f}m  err:{error:.2f}m')
        raise MovementTimeout(
            f'set_depth timeout after {timeout:.1f}s -- '
            f'tgt={target_m:+.2f}m closest={closest:+.2f}m err={error:.2f}m')
    log.info('[DEPTH] !! timeout -- no telemetry')
    raise MovementTimeout(
        f'set_depth timeout after {timeout:.1f}s -- no AHRS telemetry received')
