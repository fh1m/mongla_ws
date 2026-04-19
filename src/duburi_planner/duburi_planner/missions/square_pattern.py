#!/usr/bin/env python3
"""square_pattern -- the classic Duburi shakedown.

Four 5-second forward legs separated by 90deg right turns, with a
1-second pause between each turn and the next leg to drain residual
yaw inertia from the ArduSub heading-hold integrator
(see .claude/context/axis-isolation.md).

Run: `ros2 run duburi_planner mission square_pattern`
"""

LEG_DURATION_S = 5.0
LEG_GAIN_PCT   = 60.0
TURN_DEG       = 90.0
PAUSE_S        = 1.0


def run(duburi, log):
    duburi.arm()
    duburi.set_mode('MANUAL')
    duburi.set_depth(0.0, timeout=40.0)

    for leg in range(4):
        duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)
        if leg < 3:
            duburi.pause(PAUSE_S)
            duburi.yaw_right(TURN_DEG)
            duburi.pause(PAUSE_S)

    duburi.disarm()
