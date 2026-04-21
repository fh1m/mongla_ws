#!/usr/bin/env python3
"""move_and_see -- minimal open-loop + vision + open-loop demo.

The shortest mission that proves vision and control compose cleanly:

  arm
  set_depth (-0.5 m)     depth-hold engaged; autopilot owns vertical from here
  move_forward 3 s       open-loop scout leg (no vision, pure timed thrust)
  vision.find            discover the target by sweeping
  vision.lock 120 s      hold yaw + distance (on_lost='hold' rides out gaps)
  move_back 3 s          open-loop withdraw (depth still held by autopilot)
  set_depth 0            surface
  disarm

Run:
  ros2 run duburi_planner mission move_and_see

Live tuning during the run (no rebuild needed):
  ros2 param set /duburi_manager vision.kp_yaw 80.0
  ros2 param set /duburi_manager vision.target_bbox_h_frac 0.65
"""

CAMERA       = 'laptop'
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5

SCOUT_DURATION_S    = 3.0
SCOUT_GAIN_PCT      = 60.0      # 60% forward thrust = moderate speed
WITHDRAW_DURATION_S = 3.0
WITHDRAW_GAIN_PCT   = 60.0

INVESTIGATE_DURATION_S = 120.0
INVESTIGATE_DISTANCE   = 0.55   # stop when target fills 55% of frame height


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    # arm: power the thrusters, wait for ArduSub confirmation.
    duburi.arm()

    # set_depth: engage depth-hold and descend to 0.5 m. settle=1.0 waits
    # 1 extra second after arrival so the vehicle stops bobbing before we
    # start moving forward.
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    # move_forward: open-loop thrust on the forward channel for 3 seconds.
    # gain=60 means 60% forward thrust. No feedback -- purely time-based.
    # Increase gain for more speed; decrease if the vehicle spins or drifts.
    duburi.move_forward(SCOUT_DURATION_S, gain=SCOUT_GAIN_PCT)

    # vision.find: sweep right while watching the camera. Returns as soon
    # as the target class appears in frame. Fails after 20 s if nothing seen.
    duburi.vision.find(sweep='right', timeout=20.0)

    # vision.lock: hold yaw + forward distance together for up to 120 s.
    #   axes='yaw,forward' -- horizontal centering + distance control run
    #   in the same loop. No oscillating back and forth between them.
    #   distance=0.55 -- stay at the range where the bbox fills ~55% of height.
    #   on_lost='hold' -- if detection drops for up to 2 s, hold last thrust
    #   rather than aborting. Good for turbid water or brief occlusions.
    #   Exits early if both axes settle within deadband.
    #   Tune: kp_yaw (yaw speed), kp_forward (approach speed), deadband.
    duburi.vision.lock(axes='yaw,forward',
                       distance=INVESTIGATE_DISTANCE,
                       duration=INVESTIGATE_DURATION_S,
                       on_lost='hold')

    # move_back: open-loop reverse for 3 seconds. Symmetrical to the scout.
    duburi.move_back(WITHDRAW_DURATION_S, gain=WITHDRAW_GAIN_PCT)

    # Surface: drive back to 0 m. Autopilot holds depth until disarm.
    duburi.set_depth(0.0)

    # disarm: cut thruster power.
    duburi.disarm()
