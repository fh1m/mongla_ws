#!/usr/bin/env python3
"""pursue_demo -- torpedo-style approach using home(lock_mode='pursue').

Demonstrates lock_mode='pursue':
  1. find    -- rotate right until target appears.
  2. turn    -- centre horizontally before approaching (settle mode).
  3. home    -- yaw + forward; lock_mode='pursue' drives forward-only until
                the target fills APPROACH_FILL fraction of the frame.

Core pattern for a competition torpedo task:
  Step 1: centre the target  (turn)
  Step 2: close range and fire  (home with pursue)

Bench/sim test:
  ros2 run duburi_planner mission pursue_demo
  Watch [VIS] log: bbox size should grow toward 0.80.

Tune: lower APPROACH_FILL (e.g. 0.55) for a safer standoff.
      lower kp_forward for a slower approach.

WARNING: pursues until bbox fills 80% OR 20 s timeout.
         On real hardware the vehicle will keep moving forward.
         Have a kill switch ready.
"""

CAMERA       = 'laptop'
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5

ACQUIRE_TIMEOUT_S = 20.0
APPROACH_FILL     = 0.80   # target fill fraction to stop at (80% = contact range)
APPROACH_TIMEOUT  = 20.0


def run(duburi, log):
    duburi.mission_reset()
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    duburi.vision.find(move='yaw_right', timeout=ACQUIRE_TIMEOUT_S)

    # Settle yaw before approaching so we're aimed straight at the target.
    duburi.vision.turn(duration=8.0)

    # pursue: forward-only (never backs off). Exits when APPROACH_FILL reached.
    # on_lost='hold' keeps the last thrust if detection drops briefly.
    duburi.vision.home(
        yaw=True, forward=True,
        dist=APPROACH_FILL,
        duration=APPROACH_TIMEOUT,
        lock_mode='pursue',
        on_lost='hold')

    duburi.move_back(2.0, gain=50.0)
    duburi.stop()
    duburi.set_depth(0.0)
    duburi.disarm()
