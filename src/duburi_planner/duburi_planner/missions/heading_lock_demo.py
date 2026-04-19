#!/usr/bin/env python3
"""heading_lock_demo -- continuous heading-hold during a strafing square.

Locks the current heading, runs a strafing square so the lock thread
is the only thing keeping the bow pointed forward, retargets the lock
mid-mission with a yaw_left, then unlocks and shuts down.

Run: `ros2 run duburi_planner mission heading_lock_demo`
"""

LEG_DURATION_S = 3.0
LEG_GAIN_PCT   = 50.0
LOCK_TIMEOUT_S = 120.0
PIVOT_DEG      = 45.0


def run(duburi, log):
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(0.0, timeout=40.0)

    duburi.lock_heading(0.0, timeout=LOCK_TIMEOUT_S)

    for leg in range(4):
        duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)
        if leg % 2 == 0:
            duburi.move_right(LEG_DURATION_S, gain=LEG_GAIN_PCT)
        else:
            duburi.move_left(LEG_DURATION_S, gain=LEG_GAIN_PCT)

    # Retarget the lock by yawing -- yaw_* writes a new heading setpoint.
    duburi.yaw_left(PIVOT_DEG)
    duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)

    duburi.release_heading()
    duburi.disarm()
