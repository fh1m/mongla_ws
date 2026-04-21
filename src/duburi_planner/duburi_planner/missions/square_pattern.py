#!/usr/bin/env python3
"""square_pattern -- the classic shakedown mission.

Four 5-second forward legs separated by 90-degree right turns.
A 1-second pause before and after each turn drains the residual spin
left over in ArduSub's heading-hold integrator -- without it the vehicle
drifts slightly sideways into the next leg.

Use this to verify:
  - Thrusters all fire in the right direction.
  - 90-degree yaw_right turns the bow right, not left.
  - move_forward does not crab (if it does, check thruster spin directions).

Run: ros2 run duburi_planner mission square_pattern
"""

LEG_DURATION_S = 5.0
LEG_GAIN_PCT   = 60.0   # 60% forward thrust; raise for more speed
TURN_DEG       = 90.0
PAUSE_S        = 1.0    # coast-down time between turns


def run(duburi, log):
    # arm: power the thrusters, wait for ArduSub to confirm.
    duburi.arm()

    # set_mode MANUAL: we start in MANUAL because this mission does not
    # need depth-hold. MANUAL gives the sharpest, most direct response
    # to RC commands -- ideal for thrust verification.
    duburi.set_mode('MANUAL')

    # set_depth: even in MANUAL this engages the depth setpoint so the
    # vehicle stays roughly at surface level rather than sinking.
    duburi.set_depth(0.0, timeout=40.0)

    for leg in range(4):
        # move_forward: open-loop thrust on the forward channel for
        # LEG_DURATION_S seconds. No position feedback -- purely timed.
        # gain= is 0-100% thrust. The vehicle won't stop early.
        duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)

        if leg < 3:
            # pause: send neutral signals for 1 s. Lets the vehicle coast
            # to a stop and clears leftover heading spin before turning.
            duburi.pause(PAUSE_S)

            # yaw_right: rotate 90 degrees right. Streams a heading
            # setpoint until the compass confirms the turn is done.
            # Tune: if the turn overshoots, add settle= after yaw_right.
            duburi.yaw_right(TURN_DEG)

            # pause again: let the heading PID finish settling after the
            # turn so the next forward leg starts clean.
            duburi.pause(PAUSE_S)

    # disarm: cut thruster power. No surface command needed here since
    # we started at the surface.
    duburi.disarm()
