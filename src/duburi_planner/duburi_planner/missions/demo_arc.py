#!/usr/bin/env python3
"""arc_demo -- side-by-side comparison of sharp vs curved turns.

Phase A (sharp): forward → pause → yaw → pause → forward
  Each verb settles cleanly before the next one runs. Good for
  precision approaches where you need exact headings.

Phase B (curved): arc(heading A) → arc(heading B)
  forward + a heading-PID run in the same command packet, so the vehicle
  curves smoothly ONTO each absolute heading -- like a car sweeping a corner.

Use this to verify:
  - yaw_right / yaw_left turn the correct direction.
  - arc(target_yaw) sweeps onto an absolute heading (auto-computed direction).
  - pause() actually stops forward momentum before a turn.

Run: ros2 run duburi_planner mission arc_demo
"""

LEG_DURATION_S = 4.0
LEG_GAIN_PCT   = 60.0
TURN_DEG       = 90.0
ARC_DURATION_S = 4.0
ARC_GAIN_PCT   = 50.0
ARC_TARGETS    = (90.0, 0.0)   # absolute headings to sweep onto in Phase B


def run(duburi, log):
    duburi.mission_reset()
    # arm + depth: standard startup sequence.
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(0.0, timeout=40.0)

    # ------------------------------------------------------------------ #
    #  Phase A: sharp turns -- forward, pause, yaw, pause, forward        #
    # ------------------------------------------------------------------ #
    log('--- Phase A: sharp turns (yaw_*, settle between)')
    for i in range(2):
        # move_forward: open-loop thrust. gain=60% for 4 seconds.
        # No vision or position feedback.
        duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)

        # pause 1.5 s: coast down and clear heading-hold integrator so
        # the yaw command starts from a steady state.
        duburi.pause(1.5)

        # yaw_right / yaw_left: spin to the target heading via ArduSub's
        # heading PID. Returns when the compass reports the turn is done.
        # Increase timeout= if the turn times out on a sluggish vehicle.
        if i == 0:
            duburi.yaw_right(TURN_DEG)
        else:
            duburi.yaw_left(TURN_DEG)

        # pause 1.0 s: let heading overshoot die down before the next leg.
        duburi.pause(1.0)

    # settle=2.0: adds a 2-second neutral hold after the final forward leg
    # before Phase B starts, so the vehicle is completely still.
    duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT, settle=2.0)

    # ------------------------------------------------------------------ #
    #  Phase B: curved turns -- forward + yaw in one command              #
    # ------------------------------------------------------------------ #
    log('--- Phase B: curved turns (arc to an absolute heading)')
    for target in ARC_TARGETS:
        # arc: forward thrust (gain=50%) + a heading PID in the same command
        # packet. The hull curves ONTO the absolute heading `target` (deg) and
        # holds it for the rest of the run -- a sweeping turn-and-go, not a
        # stop-and-pivot. Turn direction is auto-computed (shortest path).
        # Tune: raise gain for a faster/tighter curve; longer seconds travels
        #       further after the heading is reached.
        duburi.arc(target, ARC_DURATION_S, gain=ARC_GAIN_PCT)

    # disarm: cut thruster power.
    duburi.disarm()
