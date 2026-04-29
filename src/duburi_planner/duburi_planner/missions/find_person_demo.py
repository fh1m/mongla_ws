#!/usr/bin/env python3
"""find_person_demo -- canonical vision-driven mission, also a verb reference.

Exercises every duburi.vision.* verb in sequence. Use as a template for
new competition tasks -- each verb block below maps to one future YASMIN state.

Verb quick-reference
--------------------
duburi.vision.find(move, timeout)
    Watch until target appears. move='still'/'forward'/'yaw_right'/'yaw_left'/'arc'.
    Exits on first detection. Aborts mission after timeout seconds.

duburi.vision.turn(duration)
    Yaw left/right to bring target's horizontal centre to frame centre (Ch4).
    Exits when centred within deadband, or duration expires.

duburi.vision.approach(dist, metric, duration)
    Drive forward/back so target fills dist fraction of frame.
    metric: 'height'/'width'/'area'/'diagonal'. Exits when at standoff, or timeout.

duburi.vision.home(yaw, lat, depth, forward, dist, metric, duration)
    Multi-axis lock. All active axes run in one 20 Hz loop.
    Exits when all axes settle within deadband, or duration expires.

duburi.vision.track(yaw, forward, depth, lat, dist, duration)
    Continuous tracking (never exits on settle). Good for moving targets.

Live tuning (no rebuild):
  ros2 param set /duburi_manager vision.kp_yaw 80.0
  ros2 param set /duburi_manager vision.deadband 0.06

WARNING: this mission arms the vehicle.
"""

CAMERA       = 'laptop'
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5

HOLD_DISTANCE = 0.55   # stop when bbox fills 55% of frame height

ACQUIRE_TIMEOUT_S = 25.0
ALIGN_DURATION_S  = 8.0
HOLD_DURATION_S   = 12.0
LOCK_DURATION_S   = 15.0


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    # find: rotate right until target appears. Aborts after 25 s.
    duburi.vision.find(move='yaw_right', timeout=ACQUIRE_TIMEOUT_S)

    # turn: yaw only — centre target horizontally before approaching.
    duburi.vision.turn(duration=ALIGN_DURATION_S)

    # approach: drive to standoff distance by bbox height fraction.
    # on_lost='hold' rides out brief detection gaps (turbid water).
    duburi.vision.approach(
        dist=HOLD_DISTANCE,
        duration=HOLD_DURATION_S,
        on_lost='hold')

    # home: yaw + forward simultaneously in one control loop.
    duburi.vision.home(
        yaw=True, forward=True,
        dist=HOLD_DISTANCE,
        duration=LOCK_DURATION_S)

    # Deliberate de-target then re-acquire: tests full search → lock cycle.
    duburi.yaw_left(90.0)
    duburi.vision.find(move='yaw_right', timeout=ACQUIRE_TIMEOUT_S)

    # home: 3-axis lock. Depth axis nudges the ALT_HOLD setpoint to keep
    # the target vertically centred. All three axes run in the same loop.
    duburi.vision.home(
        yaw=True, forward=True, depth=True,
        dist=HOLD_DISTANCE,
        duration=LOCK_DURATION_S,
        on_lost='hold')

    duburi.stop()
    duburi.set_depth(0.0)
    duburi.disarm()
