#!/usr/bin/env python3
"""Single source of truth for every motion-loop rate/throttle constant."""

THRUST_HZ          = 20.0   # RC override publish rate (forward, lateral, arc)
YAW_RATE_HZ        = 10.0   # Ch4 rate-override publish rate (yaw_snap / yaw_glide)
LOCK_STREAM_HZ     = 50.0   # HeadingLock background refresh rate — match BNO085 firmware output
DEPTH_SETPOINT_HZ  = 5.0    # set_target_depth publish rate inside motion_depth.hold_depth
DEPTH_RAMP_S          = 2.5   # setpoint ramp duration for set_depth (seconds)
DEPTH_BRAKE_ZONE_M    = 0.30  # within this distance, stop tracking so ArduSub brakes the approach
DEPTH_RAMP_ADVANCE_M  = 0.50  # advance the initial ramp setpoint this far in the target direction
                               # so ArduSub sees a real error from tick 1 and its I-term does not
                               # briefly push the sub the wrong way (especially visible going up
                               # from deep depth where the accumulated downward I-term is large)
HEARTBEAT_HZ       = 5.0    # Heartbeat all-neutral RC override (FS_PILOT_INPUT guard)
VISION_LOOP_HZ     = 20.0   # motion_vision tick rate
LOG_THROTTLE_S     = 0.5    # seconds between motion-loop log heartbeats
