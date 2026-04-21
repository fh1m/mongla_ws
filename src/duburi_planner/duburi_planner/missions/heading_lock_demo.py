#!/usr/bin/env python3
"""heading_lock_demo -- continuous heading-hold during a strafing square.

Demonstrates that heading_lock runs as a background correction while
other commands (move_forward, move_left, move_right) execute in the
foreground. Without the lock the vehicle would slowly spin as lateral
thrusters create a torque reaction. With the lock the bow stays pointed
at the initial heading automatically.

Phase 1: lock current heading, run a strafing square (forward + left/right).
Phase 2: retarget the lock by yawing 45 degrees mid-mission -- proves
         the lock updates its reference without stopping the mission.
Phase 3: release the lock, disarm.

Use this to verify:
  - heading_lock actually holds the bow during sideways movement.
  - move_left / move_right strafe the right way (left = leftward, right = rightward).
  - yaw_left during an active lock re-aims the lock at the new heading.

Run: ros2 run duburi_planner mission heading_lock_demo
"""

LEG_DURATION_S = 3.0
LEG_GAIN_PCT   = 50.0    # 50% thrust -- gentler for a strafing demo
LOCK_TIMEOUT_S = 120.0   # how long the heading lock stays active
PIVOT_DEG      = 45.0    # mid-mission yaw to prove lock retargeting


def run(duburi, log):
    # arm + depth-hold: standard startup.
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(0.0, timeout=40.0)

    # lock_heading(0.0): start the background heading-hold thread.
    # Passing 0.0 means "lock to the CURRENT compass heading" (0.0 is
    # the special value meaning "use wherever the vehicle is pointing now").
    # The lock streams heading setpoints at 20 times per second in the
    # background until release_heading() is called.
    # Tune: timeout= is the max time before the lock thread auto-releases
    #       (safety; set long for a real mission).
    duburi.lock_heading(0.0, timeout=LOCK_TIMEOUT_S)

    for leg in range(4):
        # move_forward: straight thrust, 3 s, 50% gain.
        # The heading lock keeps the bow pointed forward automatically --
        # no yaw commands needed between legs.
        duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)

        # move_right / move_left: lateral strafe using the side thrusters.
        # The heading lock fights any spin caused by the lateral thrust.
        # If the bow drifts during strafe, raise the lock's background
        # correction rate via ros2 param set /duburi_manager vision.kp_yaw.
        if leg % 2 == 0:
            duburi.move_right(LEG_DURATION_S, gain=LEG_GAIN_PCT)
        else:
            duburi.move_left(LEG_DURATION_S, gain=LEG_GAIN_PCT)

    # yaw_left: rotate 45 degrees. Because heading_lock is active, this
    # RETARGETS the lock to the new heading. After the turn completes,
    # the lock holds the new heading automatically.
    duburi.yaw_left(PIVOT_DEG)

    # move_forward after retarget: verify the vehicle still goes straight
    # on the new heading with the lock active.
    duburi.move_forward(LEG_DURATION_S, gain=LEG_GAIN_PCT)

    # release_heading: stop the background heading-hold thread. After this
    # the vehicle will slowly drift in heading under open-loop commands.
    duburi.release_heading()

    # disarm: cut thruster power.
    duburi.disarm()
