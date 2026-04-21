#!/usr/bin/env python3
"""find_person_demo -- the canonical vision-driven mission.

Walks every duburi.vision.* verb in order, so this file doubles as a
template for new vision missions.

How to write a new mission in 30 seconds
-----------------------------------------
1. Drop a new file in missions/, e.g. follow_gate.py.
2. Define  def run(duburi, log): ...
3. Inside, call duburi.<verb>(...) lines top-to-bottom. One result line
   is printed per call automatically -- no logging boilerplate needed.
4. colcon build --packages-select duburi_planner, then
   ros2 run duburi_planner mission follow_gate

Live tuning (no code edit, no rebuild)
---------------------------------------
  ros2 param set /duburi_manager vision.kp_yaw 80.0
  ros2 param set /duburi_manager vision.deadband 0.06

Quick command reference
-----------------------
duburi.arm()
    Power the thrusters. Waits up to 15 s for ArduSub to confirm.
    Check: ros2 topic echo /duburi/state (armed=true).

duburi.set_mode('ALT_HOLD')
    Tells ArduSub to switch into depth-hold mode. After this, the
    onboard 400 Hz autopilot manages the vertical thrusters and keeps
    depth steady without any further commands from us.

duburi.set_depth(meters)
    Drives to the requested depth (negative = below surface) and returns
    once ArduSub reports it is within 7 cm of the target. The autopilot
    keeps holding that depth for the rest of the mission.
    Tune: timeout= (default 30 s) if pool is turbulent.

duburi.vision.find(sweep, timeout)
    Rotates slowly while watching the camera. Returns as soon as the
    target class appears in frame. If it does not appear within timeout
    seconds, the mission is aborted.
    Tune: sweep= direction ('right'/'left'/'forward'/'still').
          yaw_rate_pct= yaw speed during the sweep (default 22%).
          timeout= how many seconds to search before giving up.

duburi.vision.yaw(duration)
    Steers left or right to bring the target's horizontal center into
    the middle of the frame. Uses ArduSub's heading PID -- the correction
    gets smoother the closer it gets.
    Tune: kp_yaw= (default 60). Raise → faster response; lower → less
          overshoot. deadband= (default 0.18) is the "close enough" zone
          as a fraction of the frame width.
    Exits: target is centred and steady, OR duration seconds elapsed.

duburi.vision.forward(distance, duration)
    Drives toward or away from the target to reach the requested
    "distance" -- expressed as what fraction of the frame height the
    target's bounding box should fill. 0.55 ≈ 55% of frame height =
    a medium close-up. Larger value = stop closer.
    Tune: distance= (bbox height fraction). kp_forward= (default 200).
          on_lost= 'hold' keeps the last thrust if detection drops briefly.
    Exits: bbox height matches distance target, OR duration elapsed.

duburi.vision.lock(axes, distance, duration)
    Runs multiple axes at once in the same control loop -- one ArduSub
    command packet every 20 ms. All requested axes run together; the
    vehicle settles when ALL axes are within their deadband at the same
    time.
    Axes:  'yaw'     = steer toward horizontal centre (heading PID)
           'lat'     = strafe left/right toward centre (lateral thrust)
           'depth'   = nudge depth setpoint up/down for vertical centre
           'forward' = close / open distance by bbox height fraction
    Tune: Same kp_* and deadband as individual verbs. 'on_lost'='hold'
          keeps the last setpoints if detection drops briefly.
    Exits: all axes centred and steady, OR duration elapsed.

duburi.yaw_left(degrees) / duburi.yaw_right(degrees)
    Commands ArduSub to rotate by exactly N degrees. Streams a heading
    setpoint until the compass reports the turn is complete.
    Tune: timeout= (default 30 s). settle= pause after the turn.

duburi.stop()
    Sends neutral signals on all thruster channels (equivalent to
    releasing the joystick). ArduSub holds the last heading and depth.

duburi.set_depth(0.0)
    Surface command. ArduSub drives back to 0 m (surface) and holds.

duburi.disarm()
    Cuts thruster power. Always call this at mission end.

WARNING: this mission arms the vehicle. Run on bench / Gazebo first.
"""

CAMERA       = 'laptop'        # 'laptop' (webcam) or 'sim_front' (Gazebo)
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5            # 0.5 m below the surface

HOLD_DISTANCE      = 0.55      # stop when bbox fills 55% of frame height
ACQUIRE_TIMEOUT_S  = 25.0      # give up searching after 25 s
ALIGN_DURATION_S   = 8.0       # max time for single-axis alignment steps
HOLD_DURATION_S    = 12.0      # max time for distance-hold step
LOCK_DURATION_S    = 15.0      # max time for multi-axis lock steps


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    # arm: powers the thrusters. Waits for ArduSub confirmation.
    duburi.arm()

    # set_mode: engage depth-hold. From this point the autopilot owns the
    # vertical thrusters; we only need to give it depth targets.
    duburi.set_mode('ALT_HOLD')

    # set_depth: descend to 0.5 m and hold. settle=1.0 adds a 1-second
    # neutral pause after arrival so depth oscillations die down before
    # vision commands start.
    duburi.set_depth(DIVE_DEPTH_M, settle=1.0)

    # vision.find: rotate right while watching the camera. Returns the
    # moment the target appears. Aborts if nothing is seen in 25 s.
    duburi.vision.find(sweep='right', timeout=ACQUIRE_TIMEOUT_S)

    # vision.yaw: steer left/right until the target is horizontally
    # centred in the frame. Only the heading channel is active here.
    # Raise kp_yaw if the turn feels sluggish; lower it if oscillating.
    duburi.vision.yaw(duration=ALIGN_DURATION_S)

    # vision.forward: approach until the target fills 55% of frame height.
    # on_lost='hold': if detection drops for a moment, keep the last thrust
    # instead of immediately aborting -- useful in turbid water.
    duburi.vision.forward(distance=HOLD_DISTANCE,
                          duration=HOLD_DURATION_S,
                          on_lost='hold')

    # vision.lock (yaw + forward): hold both horizontal centre AND distance
    # at the same time. Both axes run in the same loop so there is no
    # sequential back-and-forth.
    duburi.vision.lock(axes='yaw,forward',
                       distance=HOLD_DISTANCE,
                       duration=LOCK_DURATION_S)

    # Intentionally lose the target, then reacquire -- tests the full
    # search-and-lock cycle back-to-back.
    duburi.yaw_left(90.0)
    duburi.vision.find(sweep='right', timeout=ACQUIRE_TIMEOUT_S)

    # vision.lock (yaw + forward + depth): full 3-axis lock. The depth axis
    # nudges the depth setpoint up or down to keep the target vertically
    # centred. All three corrections run in the same loop.
    # on_lost='hold': ride out brief detection gaps rather than aborting.
    duburi.vision.lock(axes='yaw,forward,depth',
                       distance=HOLD_DISTANCE,
                       duration=LOCK_DURATION_S,
                       on_lost='hold')

    # stop: release all thruster channels. Clean slate before surfacing.
    duburi.stop()

    # Surface: drive back to 0 m and hold until disarm.
    duburi.set_depth(0.0)

    # disarm: cut thruster power. Always the last call.
    duburi.disarm()
