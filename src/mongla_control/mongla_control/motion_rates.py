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
VISION_LOOP_HZ     = 20.0   # motion_vision tick rate -- ARDUSUB path
# SROT path. Deliberately a SEPARATE constant rather than a raised shared one:
# 20 Hz is not a perception limit, it is what the ArduSub link can carry
# alongside everything else on that backend -- RC override at 20 Hz, HeadingLock
# streaming Ch4 at 50 Hz, the depth setpoint at 5 Hz and a 5 Hz neutral-RC
# heartbeat, all through one link and one _tx_lock (~80 Hz of writes at peak).
#
# srot removes almost all of that. There is no host heading lock (the board
# holds heading at 500 Hz), no depth setpoint stream and no neutral-RC
# heartbeat, so the vision loop is nearly the only writer. Measured link load:
# telemetry 19 %, +13 % for ATTITUDE at 50 Hz; MANUAL_CONTROL at 50 Hz adds
# ~1500 B/s = 13 % of the 11520 B/s link.
#
# 50 Hz, not the ~54 Hz the detector achieves: the loop should not be faster
# than its own feedback, and running exactly AT the perception rate means every
# tick either has a new frame or does not, with no margin -- the freshness decay
# then chatters on frame jitter rather than on real loss.
VISION_LOOP_HZ_SROT = 50.0
LOG_THROTTLE_S     = 0.5    # seconds between motion-loop log heartbeats
