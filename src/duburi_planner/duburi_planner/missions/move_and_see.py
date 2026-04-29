#!/usr/bin/env python3
"""move_and_see -- minimal open-loop + vision + open-loop demo.

  arm
  set_depth (-0.5 m)     depth-hold engaged; autopilot owns vertical from here
  move_forward 3 s       open-loop scout leg (timed thrust, no vision)
  vision.find            discover target by sweeping right
  vision.home 120 s      hold yaw + distance (on_lost='hold' rides out gaps)
  move_back 3 s          open-loop withdraw
  set_depth 0            surface
  disarm

Run:
  ros2 run duburi_planner mission move_and_see

Live tuning (no rebuild):
  ros2 param set /duburi_manager vision.kp_yaw 80.0
  ros2 param set /duburi_manager vision.target_bbox_h_frac 0.65
"""

CAMERA       = 'laptop'
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5

SCOUT_GAIN       = 60.0
WITHDRAW_GAIN    = 60.0
INVESTIGATE_DIST = 0.55   # hold when target fills 55% of frame height


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    duburi.arm()
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    # Open-loop scout: move forward 3 s to close distance before scanning.
    duburi.move_forward(3.0, gain=SCOUT_GAIN)

    # find: sweep right while watching the camera. Returns on first detection.
    duburi.vision.find(move='yaw_right', timeout=20.0)

    # home: yaw + forward distance held for 120 s.
    # on_lost='hold' keeps last thrust if detection drops briefly.
    duburi.vision.home(
        yaw=True, forward=True,
        dist=INVESTIGATE_DIST,
        duration=120.0,
        on_lost='hold')

    # Open-loop withdraw: reverse 3 s.
    duburi.move_back(3.0, gain=WITHDRAW_GAIN)

    duburi.set_depth(0.0)
    duburi.disarm()
