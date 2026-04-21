#!/usr/bin/env python3
"""arc_demo -- side-by-side comparison of sharp vs curved turns.

Phase A (sharp): forward → pause → yaw → pause → forward
  Each verb settles cleanly before the next one runs. Good for
  precision approaches where you need exact headings.

Phase B (curved): arc(+yaw) → arc(-yaw)
  forward and yaw run in the same command packet so the vehicle
  traces a smooth curve -- like a car turning a corner.

Use this to verify:
  - yaw_right / yaw_left turn the correct direction.
  - arc() traces a real curve (if it goes straight, check yaw_rate_pct).
  - pause() actually stops forward momentum before a turn.

Run: ros2 run duburi_planner mission arc_demo
"""

LEG_DURATION_S = 4.0
LEG_GAIN_PCT   = 60.0
TURN_DEG       = 90.0
ARC_DURATION_S = 4.0
ARC_GAIN_PCT   = 50.0
ARC_YAW_RATE   = 30.0   # 30% yaw stick during the arc (positive = right turn)


def run(duburi, log):
    # arm + depth: standard startup sequence.
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(0.0, timeout=40.0)

    # ------------------------------------------------------------------ #
    #  Phase A: sharp turns -- forward, pause, yaw, pause, forward        #
    # ------------------------------------------------------------------ #
    log.info('--- Phase A: sharp turns (yaw_*, settle between)')
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
    log.info('--- Phase B: curved turns (arc, no settle between)')
    for i in range(2):
        # arc: combines forward thrust (gain=50%) with a continuous yaw
        # command (yaw_rate_pct=30%) in the same command packet.
        # Positive yaw_rate_pct = turn right; negative = turn left.
        # Result: the vehicle sweeps a smooth curve rather than
        # stopping and pivoting.
        # Tune: raise yaw_rate_pct for a tighter turn radius.
        #       raise gain for a faster curve.
        rate = ARC_YAW_RATE if i % 2 == 0 else -ARC_YAW_RATE
        duburi.arc(ARC_DURATION_S, gain=ARC_GAIN_PCT, yaw_rate_pct=rate)

    # disarm: cut thruster power.
    duburi.disarm()
