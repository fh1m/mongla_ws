#!/usr/bin/env python3
"""Single source of truth for every motion-loop rate / throttle.

Why this file exists
--------------------
Before this module existed each per-axis file declared its own
``THRUST_RATE_HZ`` / ``YAW_RATE_HZ`` / ``STREAM_HZ`` / ``SETPOINT_HZ``
/ ``LOG_THROTTLE`` constant at the top of the file. Tuning the
ArduSub-RC publish rate or the depth setpoint cadence meant grepping
six files and hoping you matched every spelling. Worse, the same
quantity was named differently in different modules
(``THRUST_RATE_HZ`` vs ``STREAM_HZ`` vs ``LOOP_HZ``) which made it
impossible to tell at a glance whether two loops were intentionally
running at the same cadence or not.

The constants below are now the *only* place those numbers are
written down. Each producer module re-imports the value it needs and
re-exports it under its historical name for backward compatibility,
so existing call-sites keep working without touching them all.

Rate sizing rationale
---------------------
``THRUST_HZ = 20``        ArduSub treats an RC override as stale after
                          ~1 s of silence. 20 Hz keeps the channel
                          fresh with margin and matches the ESC
                          control rate so the bench feels responsive.

``YAW_RATE_HZ = 10``      Ch4 rate writes are larger swings than the
                          translation channels (we lean against the
                          AHRS error). 10 Hz is the smallest rate
                          where the proportional loop still feels
                          crisp without saturating the SITL UART.

``LOCK_STREAM_HZ = 20``   HeadingLock has to refresh Ch4 just as
                          often as a translation command would
                          otherwise overwrite it -- if the lock fell
                          behind even one cycle the next motion
                          command's neutral 1500 us would steal Ch4.

``DEPTH_SETPOINT_HZ = 5`` ArduSub closes the depth loop at ~400 Hz
                          internally; we only refresh the *setpoint*
                          inside ``motion_depth.hold_depth`` while it
                          drives to a new target. Once the target is
                          reached we hand depth back to ArduSub's
                          onboard ALT_HOLD (no continuous streaming).

``HEARTBEAT_HZ = 5``      Cadence of ``Heartbeat`` -- the all-neutral
                          RC override stream that prevents
                          ``FS_PILOT_INPUT`` (ArduSub disarms after
                          ~3 s of no override). 5 Hz = 200 ms gap,
                          comfortably inside the 0.1-3.0 s window.

``VISION_LOOP_HZ = 20``   Matches THRUST_HZ so the vision controller
                          and the per-axis writer share a tick.

``LOG_THROTTLE_S = 0.5``  Half-second cadence keeps the [FWD ] /
                          [YAW ] / [VIS ] / [DEPTH] heartbeat lines
                          dense enough to spot stalls but rare
                          enough not to drown grep.

``DEPTH_RAMP_S = 2.5``   Seconds over which ``motion_depth.wait_for_depth``
                          linearly ramps the setpoint from the current
                          depth to the target. Prevents ArduSub's depth
                          PID from receiving a step change and overshooting.
                          Tune at pool: reduce if dives feel sluggish,
                          increase if overshoot returns.

``DEPTH_BRAKE_ZONE_M = 0.30``
                          When the sub is within this distance of the
                          target, the ramp tracking stops and the
                          setpoint is held at target_m. ArduSub then
                          has a real error to act on and brakes the sub
                          into the target instead of letting it drift
                          in under momentum. Without this, tracking
                          zeros ArduSub's depth error during the
                          approach so the sub arrives at full speed.
                          Tune: increase if still overshooting; decrease
                          (min ~0.10 m) if approach feels jerky.
"""

THRUST_HZ          = 20.0   # RC override publish rate (forward, lateral, arc)
YAW_RATE_HZ        = 10.0   # Ch4 rate-override publish rate (yaw_snap / yaw_glide)
LOCK_STREAM_HZ     = 20.0   # HeadingLock background refresh rate
DEPTH_SETPOINT_HZ  = 5.0    # set_target_depth publish rate inside motion_depth.hold_depth
DEPTH_RAMP_S       = 2.5    # setpoint ramp duration for set_depth (seconds)
DEPTH_BRAKE_ZONE_M = 0.30   # within this distance, stop tracking so ArduSub brakes the approach
HEARTBEAT_HZ       = 5.0    # Heartbeat all-neutral RC override (FS_PILOT_INPUT guard)
VISION_LOOP_HZ     = 20.0   # motion_vision tick rate
LOG_THROTTLE_S     = 0.5    # seconds between motion-loop log heartbeats

__all__ = [
    'THRUST_HZ',
    'YAW_RATE_HZ',
    'LOCK_STREAM_HZ',
    'DEPTH_SETPOINT_HZ',
    'DEPTH_RAMP_S',
    'DEPTH_BRAKE_ZONE_M',
    'HEARTBEAT_HZ',
    'VISION_LOOP_HZ',
    'LOG_THROTTLE_S',
]
