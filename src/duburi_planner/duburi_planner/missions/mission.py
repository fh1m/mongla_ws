#!/usr/bin/env python3
"""pursue_demo -- torpedo-style approach using lock_mode='pursue'.

Demonstrates the pursue lock mode:
  1. Find the target.
  2. Turn (yaw) until centred (settle mode -- exits when done).
  3. vision.home with lock_mode='pursue': drive forward while keeping
     the target centred, stop the moment the target fills APPROACH_FILL
     of the frame (simulates reaching contact range / firing distance).

This is the core pattern for a competition torpedo task:
  - Phase 1: centre the target horizontally (turn).
  - Phase 2: pursue forward until bbox fills the target fraction.

Safe bench/sim test sequence:
  ros2 run duburi_planner mission pursue_demo
  (No pool needed -- watch the [VIS] log lines: size should grow toward 0.80.)

WARNING: pursues until bbox fills 80% of frame OR 20 s timeout.
         On real hardware the vehicle will keep moving forward.
         Have a kill switch ready.

Run: ros2 run duburi_planner mission pursue_demo
"""

CAMERA           = 'laptop'
TARGET_CLASS     = 'person'
DIVE_DEPTH_M     = -0.5

ACQUIRE_TIMEOUT_S = 20.0

# Phase 2: approach until the target fills this fraction of the frame.
# 0.80 = target nearly filling the frame = very close / contact range.
# Lower this (e.g. 0.55) to stop at a comfortable standoff distance.
APPROACH_FILL    = 0.80
APPROACH_TIMEOUT = 20.0  # safety: abort if target not reached in 20 s


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    # arm + depth-hold: standard startup
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)
    duburi.lock_heading(degrees=0.0, timeout=300.0)

    # DSL verb examples (uncomment the section you want to test)

    # Open-loop motion
    # duburi.move_forward(5, gain=80.0)
    # duburi.move_back(5, gain=80.0)
    # duburi.move_left(5, gain=80.0)
    # duburi.move_right(5, gain=80.0)
    # duburi.yaw_left(90, timeout=30.0)
    # duburi.yaw_right(90, timeout=30.0)

    # Vision: find → turn → pursue
    # duburi.vision.find(target=TARGET_CLASS, move='yaw_right', timeout=ACQUIRE_TIMEOUT_S)
    # duburi.vision.turn(target=TARGET_CLASS, duration=8.0)
    # duburi.vision.slide(target=TARGET_CLASS, duration=300.0, lock_mode='follow')
    # duburi.vision.hover(target=TARGET_CLASS, duration=8.0)
    # duburi.vision.approach(target=TARGET_CLASS, dist=0.55, metric='height', duration=12.0)
    # duburi.vision.home(target=TARGET_CLASS, yaw=True, forward=True, depth=True,
    #                    dist=0.50, metric='height', duration=15.0, lock_mode='follow')

    # Pursue: drive forward while keeping centred; exits when size >= APPROACH_FILL
    # duburi.vision.home(
    #     target=TARGET_CLASS,
    #     yaw=True, forward=True,
    #     dist=APPROACH_FILL,
    #     metric='height',
    #     duration=APPROACH_TIMEOUT,
    #     lock_mode='pursue',
    #     on_lost='hold',
    # )

    # Back off after contact range
    # duburi.move_back(2.0, gain=50.0)

    duburi.unlock_heading()
    duburi.stop()
    duburi.disarm()
