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


from .motion_rates import DEPTH_SETPOINT_HZ as SETPOINT_HZ
from .motion_rates import LOG_THROTTLE_S as LOG_THROTTLE

TOL_M         = 0.07    # exit tolerance (m). Achievable now we're not fighting ArduSub.
PRIME_SECONDS = 0.5     # drain stale ALT_HOLD I-term before driving anywhere


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
    start_d  = starting['depth'] if starting is not None else target_m

    prime_alt_hold(pixhawk, hold_at=start_d, neutral_writer=neutral_writer)
    wait_for_depth(pixhawk, target_m, timeout, log)


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


def wait_for_depth(pixhawk, target_m, timeout, log):
    """Phase 2: stream the real target until reached or timeout.

    Returns None on success. Raises MovementTimeout otherwise -- the
    message includes the closest depth we ever reached so the operator
    can tell "stuck on the way" from "no telemetry at all".
    """
    deadline    = time.time() + timeout
    closest     = None

    while time.time() < deadline:
        pixhawk.set_target_depth(target_m)

        attitude = pixhawk.get_attitude()
        if attitude is not None:
            current = attitude['depth']
            error   = abs(target_m - current)
            if closest is None or error < abs(target_m - closest):
                closest = current

            log.info(
                f'[DEPTH] -> {target_m:+.2f}m  '
                f'now:{current:+.2f}m  err:{error:.2f}m',
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
