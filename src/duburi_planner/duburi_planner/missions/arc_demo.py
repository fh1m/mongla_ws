#!/usr/bin/env python3
"""arc_demo -- side-by-side comparison of sharp vs curved turns.

Phase A (sharp): forward -> pause -> yaw -> pause -> forward
                 verifies axis isolation -- each verb settles cleanly
                 before the next runs.
Phase B (curved): arc(+yaw) -> arc(-yaw)
                  verifies the Ch5 + Ch4 single-packet path.

Run: `ros2 run duburi_planner mission arc_demo`
"""

LEG_DURATION_S = 4.0
LEG_GAIN_PCT   = 60.0
TURN_DEG       = 90.0
ARC_DURATION_S = 4.0
ARC_GAIN_PCT   = 50.0
ARC_YAW_RATE   = 30.0


def run(duburi, log):
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(0.0, timeout=40.0)

    log.info('--- Phase A: sharp turns (yaw_*, settle between)')
    for i in range(2):
        duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)
        duburi.pause(1.5)
        if i == 0:
            duburi.yaw_right(TURN_DEG)
        else:
            duburi.yaw_left(TURN_DEG)
        duburi.pause(1.0)
    duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT, settle=2.0)

    log.info('--- Phase B: curved turns (arc, no settle between)')
    for i in range(2):
        rate = ARC_YAW_RATE if i % 2 == 0 else -ARC_YAW_RATE
        duburi.arc(ARC_DURATION_S, gain=ARC_GAIN_PCT, yaw_rate_pct=rate)

    duburi.disarm()
