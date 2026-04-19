#!/usr/bin/env python3
"""find_person_demo -- the canonical vision-driven mission.

Walks every `duburi.vision.*` verb in the order a real mission would
exercise them, so this file doubles as a template for new vision
missions.

Designing your own mission in 30 seconds
----------------------------------------
1. Drop a new file in `missions/`, e.g. `follow_gate.py`.
2. Define `def run(duburi, log): ...`.
3. Inside, call `duburi.<verb>(...)` lines top-to-bottom. The DSL
   prints one outcome line per call -- no logging boilerplate needed.
4. `colcon build --packages-select duburi_planner`, then
   `ros2 run duburi_planner mission follow_gate`.

Live tuning (no code edit, no rebuild):

    ros2 param set /duburi_manager vision.kp_yaw 80.0
    ros2 param set /duburi_manager vision.deadband 0.06

WARNING: this mission arms the vehicle. Run on bench / Gazebo first.
"""

CAMERA       = 'laptop'        # 'laptop' (webcam) or 'sim_front' (Gazebo)
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5            # negative = below surface
HOLD_DISTANCE      = 0.55      # bbox-height fraction = "stop here" proxy
ACQUIRE_TIMEOUT_S  = 25.0
ALIGN_DURATION_S   = 8.0
HOLD_DURATION_S    = 12.0
LOCK_DURATION_S    = 15.0


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    # Power on, hover at a known shallow depth, then engage the
    # background depth-streamer so every subsequent verb runs with
    # depth held automatically (the cousin of `lock_heading`).
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)
    duburi.lock_depth(DIVE_DEPTH_M)

    # Sweep until the target is in frame.
    duburi.vision.find(sweep='right', timeout=ACQUIRE_TIMEOUT_S)

    # Single-axis demos: yaw only, then forward (distance) only.
    duburi.vision.yaw(duration=ALIGN_DURATION_S)
    duburi.vision.forward(distance=HOLD_DISTANCE,
                          duration=HOLD_DURATION_S,
                          on_lost='hold')

    # Multi-axis: yaw + forward at the same time.
    duburi.vision.lock(axes='yaw,forward',
                       distance=HOLD_DISTANCE,
                       duration=LOCK_DURATION_S)

    # Lose + reacquire: rotate away, then sweep back to it.
    duburi.yaw_left(90.0)
    duburi.vision.find(sweep='right', timeout=ACQUIRE_TIMEOUT_S)

    # Full 3D: yaw + depth + forward held simultaneously.
    duburi.vision.lock(axes='yaw,forward,depth',
                       distance=HOLD_DISTANCE,
                       duration=LOCK_DURATION_S,
                       on_lost='hold')

    # Surface and shut down. Release the depth lock first so the
    # final set_depth(0.0) is the sole author of the depth setpoint.
    duburi.stop()
    duburi.release_depth()
    duburi.set_depth(0.0)
    duburi.disarm()
