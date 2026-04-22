#!/usr/bin/env python3
"""pursue_demo -- torpedo-style approach using lock_mode='pursue'.

Demonstrates the pursue lock mode:
  1. Find the target.
  2. Align yaw until centred (settle mode -- exits when done).
  3. vision.lock with lock_mode='pursue': drive forward while keeping
     the target centred, stop the moment the target fills 80% of
     the frame (simulates reaching contact range / firing distance).

This is the core pattern for a competition torpedo task:
  - Phase 1: centre the target horizontally (yaw).
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

    # arm + depth-hold: standard startup + standard control system commands
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)
    duburi.lock_heading(degrees=0.0, timeout=300.0)
    duburi.vision.lateral(target='person', duration=300.0, lock_mode="follow")
    duburi.move_forward(5, gain=80.0)
    duburi.move_back(5, gain=80.0)
    duburi.move_left(5, gain=80.0)
    duburi.move_right(5, gain=80.0)
    # duburi.yaw_left(90, timeout=30.0)
    # duburi.yaw_right(90, timeout=30.0)

    # Heading Lock
    # duburi.lock_heading(degrees=0.0, timeout=300.0)
        # Perform manuvers
    duburi.unlock_heading()

    # Vision commands
    # duburi.vision.find(target='person', sweep='right', timeout=25.0)
    # duburi.vision.yaw(target='person', duration=8.0)
    # duburi.vision.lateral(target='person', duration=300.0)
    # duburi.vision.depth(target='person', duration=8.0)
    # duburi.vision.forward(target='person', distance=0.55, duration=12.0)
    # duburi.vision.lock(target='gate', axes='yaw,forward,depth', distance=0.50, duration=15.0, lock_mode='follow')


    # vision.find: rotate right until the target appears in frame.
    # Returns as soon as one detection arrives.
    # duburi.vision.find(sweep='right', timeout=ACQUIRE_TIMEOUT_S)

    # vision.yaw: settle mode -- steer until target is horizontally centred,
    # then return. This ensures we're aimed straight at the target before
    # starting the approach.
    # duburi.vision.yaw(duration=8.0)

    # vision.lock with lock_mode='pursue':
    #   axes='yaw,forward' -- keep centred horizontally while approaching.
    #   distance=APPROACH_FILL -- the target fill fraction to reach.
    #   lock_mode='pursue' -- forward thrust is one-way (never backs off).
    #                         Exits the moment the target fills APPROACH_FILL
    #                         fraction of the frame, OR when timeout hits.
    #   on_lost='hold' -- if detection drops briefly, hold the last thrust
    #                     instead of aborting (useful in turbid water).
    #
    # Tune: lower APPROACH_FILL to stop at a safer standoff distance.
    #       lower kp_forward to approach more slowly.
    #       raise deadband so minor yaw wobble doesn't slow the approach.
    # duburi.vision.lock(
    #     axes='yaw,forward',
    #     distance=APPROACH_FILL,
    #     duration=APPROACH_TIMEOUT,
    #     lock_mode='pursue',
    #     on_lost='hold',
    # )

    # Back off: open-loop reverse so we don't stay at contact range.
    # duburi.move_back(2.0, gain=50.0)

    # Surface and shut down.
    duburi.stop()
    # duburi.set_depth(0.0)
    duburi.disarm()
