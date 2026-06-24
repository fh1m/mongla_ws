#!/usr/bin/env python3
"""move_and_see -- minimal open-loop + vision-tracking demo.

  arm
  set_depth (-0.5 m)          depth-hold engaged; autopilot owns vertical
  move_forward 3 s            open-loop scout leg
  detected() scan             yaw left in 90 deg steps until target found
  vision.track                lateral-only tracking (yaw/forward disabled)
  move_back 3 s               open-loop withdraw
  set_depth 0                 surface
  disarm

Sim / webcam testing (yolov11n pretrained):
  ros2 run duburi_vision vision_display --ros-args \\
      -p launch_pipeline:=true -p model:=yolov11n -p classes:=person
  ros2 run duburi_planner mission move_and_see

Live tuning (no rebuild):
  ros2 param set /duburi_manager vision.kp_lat 80.0
  ros2 param set /duburi_manager vision.target_bbox_h_frac 0.65
"""

CAMERA              = 'video'
TARGET_CLASS        = 'gate'
DIVE_DEPTH_M        = -0.5

SCOUT_GAIN          = 60.0
WITHDRAW_GAIN       = 60.0
TRACK_DURATION_S    = 200.0

_MAX_SEARCH_STEPS   = 20     # 20 × 90° = up to 5 full sweeps before giving up


def run(duburi, log):
    duburi.mission_reset()
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    duburi.arm()
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    # Open-loop scout: close distance before scanning.
    duburi.move_forward(3.0, gain=SCOUT_GAIN)

    # Bounded scan: yaw left in 90° steps until target appears.
    for _ in range(_MAX_SEARCH_STEPS):
        if duburi.detected(TARGET_CLASS):
            break
        duburi.move_forward(3, gain=50)
        duburi.pause(3)
        duburi.yaw_left(90)
    else:
        log(f'target {TARGET_CLASS!r} not found after {_MAX_SEARCH_STEPS} steps — aborting')
        duburi.set_depth(0.0)
        duburi.disarm()
        return

    # Vision-track laterally to keep target centred (no yaw, no approach).
    duburi.vision.track(
        target=TARGET_CLASS,
        yaw=False, lat=True, forward=True,
        duration=TRACK_DURATION_S,
        on_lost='hold',
    )

    # Withdraw and surface.
    duburi.move_back(3.0, gain=WITHDRAW_GAIN)
    duburi.set_depth(0.0)
    duburi.disarm()
