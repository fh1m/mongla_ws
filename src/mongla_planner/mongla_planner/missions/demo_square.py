#!/usr/bin/env python3
"""demo_square -- the classic shakedown mission.

Four 5-second forward legs separated by 90-degree right turns, with a
1-second hold before and after each turn so the hull coasts to a stop and
the next leg starts clean.

Each verb is ONE move the board runs and brakes itself; between them the
board holds heading and depth.

Use this to verify:
  - Thrusters all fire in the right direction.
  - 90-degree yaw_right turns the bow right, not left.
  - move_forward does not crab (if it does, check thruster spin directions).

Every verb here is a SROT_MOVE, which closes the depth loop -- see
CLAUDE.md §2 for the two armed bench checks that gate it.

Run: ros2 run mongla_planner mission demo_square
"""

LEG_DURATION_S = 5.0
LEG_GAIN_PCT   = 60.0   # 60% forward thrust; raise for more speed
TURN_DEG       = 90.0
PAUSE_S        = 1.0    # hold time between legs and turns
DEPTH_M        = -0.5   # negative = below the surface


def run(mongla, log):
    mongla.mission_reset()
    mongla.arm()
    mongla.set_depth(DEPTH_M, timeout=40.0)

    for leg in range(4):
        # Timed open-loop thrust: no position feedback, the board brakes at
        # the end of the leg.
        mongla.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)

        if leg < 3:
            mongla.pause(PAUSE_S)            # station-keep while the hull coasts down
            mongla.yaw_right(TURN_DEG)       # relative turn, closed on the board's heading
            mongla.pause(PAUSE_S)

    mongla.surface()
    mongla.disarm()
