#!/usr/bin/env python3
"""move_and_see -- minimal open-loop + vision + open-loop demo.

The shortest mission that proves vision and control compose cleanly:

    arm
    set_depth (-0.5 m)
    move_forward 3 s        <- open-loop scout leg
    vision.find             <- discover the target
    vision.lock 12 s        <- investigate (yaw + distance held together)
    move_back 3 s           <- open-loop withdraw
    surface
    disarm

Run:
  ros2 run duburi_planner mission move_and_see

Live tuning during the run:
  ros2 param set /duburi_manager vision.kp_yaw 80.0
  ros2 param set /duburi_manager vision.target_bbox_h_frac 0.65
"""

CAMERA       = 'laptop'
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5

SCOUT_DURATION_S    = 3.0
SCOUT_GAIN_PCT      = 60.0
WITHDRAW_DURATION_S = 3.0
WITHDRAW_GAIN_PCT   = 60.0

INVESTIGATE_DURATION_S = 120.0
INVESTIGATE_DISTANCE   = 0.55


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    duburi.arm()
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    duburi.move_forward(SCOUT_DURATION_S, gain=SCOUT_GAIN_PCT)
    duburi.vision.find(sweep='right', timeout=20.0)
    duburi.vision.lock(axes='yaw,forward',
                       distance=INVESTIGATE_DISTANCE,
                       duration=INVESTIGATE_DURATION_S,
                       on_lost='hold')
    duburi.move_back(WITHDRAW_DURATION_S, gain=WITHDRAW_GAIN_PCT)

    duburi.set_depth(0.0)
    duburi.disarm()
