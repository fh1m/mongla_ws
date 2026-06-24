#!/usr/bin/env python3
"""move_and_see -- minimal open-loop + vision demo (two-verb).

  arm
  set_depth (-0.5 m)          depth-hold engaged; autopilot owns vertical
  move_forward 3 s            open-loop scout leg
  vision.align (lat)          slide to keep target centred (creep fallback)
  vision.move                 drive in until target fills the frame
  move_back 3 s               open-loop withdraw
  set_depth 0                 surface
  disarm

Sim / webcam testing (yolov11n pretrained):
  ros2 run duburi_vision vision_display --ros-args \\
      -p launch_pipeline:=true -p model:=yolov11n -p classes:=person
  ros2 run duburi_planner mission move_and_see

Live tuning (no rebuild):
  ros2 param set /duburi_manager vision.kp_lat 80.0
"""

CAMERA       = 'video'
TARGET_CLASS = 'gate'
DIVE_DEPTH_M = -0.5

SCOUT_GAIN    = 60.0
WITHDRAW_GAIN = 60.0

ALIGN_ERR_PX  = 40
ALIGN_GAIN    = 30
APPROACH_GAIN = 35
FWD_FILL      = 70
ALIGN_S       = 20.0
MOVE_S        = 30.0


def run(duburi, log):
    duburi.mission_reset()
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    duburi.arm()
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    # Open-loop scout: close distance before aligning.
    duburi.move_forward(3.0, gain=SCOUT_GAIN)

    # Slide to centre the target (creep+sweep fallback finds it if lost).
    duburi.vision.align(
        TARGET_CLASS, camera=CAMERA, lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=ALIGN_S,
        fallback=scout)

    # Drive in until the target fills the frame.
    duburi.vision.move(
        TARGET_CLASS, camera=CAMERA, fwd=FWD_FILL, mode='area',
        gain=APPROACH_GAIN, duration=MOVE_S, fallback=scout)

    # Withdraw and surface.
    duburi.move_back(3.0, gain=WITHDRAW_GAIN)
    duburi.set_depth(0.0)
    duburi.disarm()


# ── Mission-authored fallback search pattern (pure control) ─────────────────────
def scout(duburi, should_stop):
    """Creep forward then yaw-step, looking for the target."""
    duburi.move_forward(3.0, gain=50)
    if should_stop():
        return
    duburi.yaw_left(90.0)
