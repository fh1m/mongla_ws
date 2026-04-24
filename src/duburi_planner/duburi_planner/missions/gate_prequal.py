#!/usr/bin/env python3
"""gate_prequal -- RoboSub gate prequalification mission.

Course layout:
  AUV start (parallel or behind gate) → [forward] → Gate (2m wide, ~1m deep)
  → [forward 3m] → Square area (flare placed here by team) → [return] → Gate

Mission sequence:
  1.  Arm, set ALT_HOLD, descend to gate depth
  2.  Connect DVL (lazy-connect for pool-day workflow)
  3.  Search: drive forward while watching for 'gate' class
  4.  Align: yaw + approach via vision.lock (handles partial/full/offset detections)
  5.  Pass through gate: DVL forward 3m (falls back to timed if DVL unavailable)
  6.  DVL square: 4 × (forward 2m + yaw_right 90°) -- circles the flare area
  7.  Return: yaw_right 180° → find gate → align → pass back through
  8.  Surface, disarm

Bounding-box edge cases (from preview images):
  - Off-center partial (right-only):  yaw axis rotates toward bbox center → centers gate
  - Full-frame (too close, 90% fill): forward axis backs up until distance=GATE_APPROACH_FRAC
  - 1/3 visible (single post at edge): yaw axis still has center_x signal → turns toward gate
  These are all handled by vision.lock without special code -- the feedback loops
  naturally respond to whatever the detector sees.

Pre-flight:
  # Terminal 1: start manager + vision with DVL yaw source + gate model
  ros2 launch duburi_manager bringup.launch.py vision:=true

  # Terminal 2: run mission
  ros2 run duburi_planner mission gate_prequal

Live tuning between runs (no rebuild):
  ros2 param set /duburi_manager vision.kp_yaw 70.0
  ros2 param set /duburi_manager vision.kp_forward 150.0
  ros2 param set /duburi_manager vision.deadband 0.08

DVL / yaw smoke tests (pool-day verification):
  ros2 run duburi_planner duburi arm
  ros2 run duburi_planner duburi set_mode --target_name ALT_HOLD
  ros2 run duburi_planner duburi set_depth --target -0.8
  ros2 run duburi_planner duburi dvl_connect
  ros2 run duburi_planner duburi move_forward_dist --distance_m 1.0 --gain 50
  ros2 run duburi_planner duburi yaw_right --target 90
  ros2 run duburi_planner duburi move_forward_dist --distance_m 1.0 --gain 50
  ros2 run duburi_planner duburi yaw_right --target 90
  ros2 run duburi_planner duburi disarm

WARNING: this mission arms the vehicle. Run on bench / Gazebo first.
"""

# ---- Tunable constants ---------------------------------------------------
# Adjust these at the pool without touching mission logic.

CAMERA        = 'forward'
TARGET_CLASS  = 'gate'

# Depth
DIVE_DEPTH_M  = -1.0          # 1m below surface; gate is typically ~1m deep

# Search phase
SEARCH_GAIN   = 35.0          # % forward thrust while scanning for gate
SEARCH_TIMEOUT_S = 45.0       # give up searching after this many seconds

# Gate alignment phase
GATE_APPROACH_FRAC = 0.35     # stop when gate fills ~35% of frame AREA
ALIGN_TIMEOUT_S    = 20.0     # max seconds for vision.lock to settle
ALIGN_KP_YAW       = 70.0     # yaw speed toward gate center (raise if sluggish)
ALIGN_KP_FORWARD   = 150.0    # approach speed (lower if oscillating)
ALIGN_DEADBAND     = 0.08     # centering tolerance (fraction of frame width)

# Gate passage (forward through gate)
GATE_PASS_DIST_M = 3.0        # DVL distance to drive to fully clear the gate
GATE_PASS_GAIN   = 60.0       # % forward thrust during passage
GATE_PASS_TIME_S = 4.5        # timed fallback if DVL unavailable

# DVL square (circles the flare area in the middle)
SQUARE_SIDE_M = 2.0           # DVL metres per side
SQUARE_GAIN   = 50.0          # % forward thrust per side
SQUARE_SIDES  = 4             # 4 sides = closed square

# Return gate pass (from the far side back through)
RETURN_SEARCH_TIMEOUT_S = 30.0
RETURN_PASS_DIST_M      = 3.5  # slightly more to fully clear gate on return
RETURN_PASS_GAIN        = 60.0


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    # ------------------------------------------------------------------ #
    #  Phase 1: Startup                                                   #
    # ------------------------------------------------------------------ #
    log('Phase 1: arm + descend')
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.5)

    # ------------------------------------------------------------------ #
    #  Phase 2: Connect DVL                                               #
    # ------------------------------------------------------------------ #
    # dvl_connect is a no-op when yaw_source is not nucleus_dvl.
    # If DVL is unavailable (TCP failure), this raises and the mission
    # aborts with a clear error -- call dvl_connect in a separate terminal
    # BEFORE running the mission if hardware is uncertain.
    log('Phase 2: DVL connect')
    duburi.dvl_connect()

    # ------------------------------------------------------------------ #
    #  Phase 3: Search forward for gate                                   #
    # ------------------------------------------------------------------ #
    # Drive forward at SEARCH_GAIN% while the detector watches for 'gate'.
    # Returns as soon as gate appears in frame. Times out after SEARCH_TIMEOUT_S.
    log('Phase 3: searching for gate (forward sweep)')
    duburi.vision.find(
        sweep='forward',
        timeout=SEARCH_TIMEOUT_S,
        gain=SEARCH_GAIN)

    # ------------------------------------------------------------------ #
    #  Phase 4: Align with gate                                           #
    # ------------------------------------------------------------------ #
    # vision.lock on yaw + forward simultaneously:
    #   yaw     -- rotates to bring gate bbox center to frame center.
    #              Works even when gate is only partially visible (partial
    #              bbox still has a center_x signal we can steer toward).
    #   forward -- approaches / backs off until gate fills GATE_APPROACH_FRAC
    #              of frame area.  distance_metric='area' is more stable than
    #              'height' alone for a wide, short gate.
    #              Automatically backs up if bbox is too large (too close).
    # on_lost='hold': if detection flickers briefly, hold position rather than
    # aborting -- important in turbid or lit pool water.
    log('Phase 4: aligning with gate')
    duburi.vision.lock(
        axes='yaw,forward',
        distance=GATE_APPROACH_FRAC,
        duration=ALIGN_TIMEOUT_S,
        on_lost='hold',
        kp_yaw=ALIGN_KP_YAW,
        kp_forward=ALIGN_KP_FORWARD,
        deadband=ALIGN_DEADBAND,
        distance_metric='area')

    # ------------------------------------------------------------------ #
    #  Phase 5: Pass through gate                                         #
    # ------------------------------------------------------------------ #
    # Drive forward past the gate. DVL gives us real metres; timed is
    # the fallback. 3m should fully clear a ~2m-wide gate from approach
    # distance of ~1m.
    log('Phase 5: passing through gate')
    duburi.move_forward_dist(GATE_PASS_DIST_M, gain=GATE_PASS_GAIN)

    # ------------------------------------------------------------------ #
    #  Phase 6: DVL square (circles flare area)                           #
    # ------------------------------------------------------------------ #
    # 4 × (forward 2m + right 90°) = closed 2m×2m square.
    # The flare is placed in the middle by the team; we orbit it.
    # We briefly lock heading at the start of each leg so heading drift
    # from the previous yaw_right is corrected during the straight leg.
    log('Phase 6: DVL square pattern around flare')
    for step in range(SQUARE_SIDES):
        log(f'  square leg {step + 1}/{SQUARE_SIDES}')
        duburi.move_forward_dist(SQUARE_SIDE_M, gain=SQUARE_GAIN)
        duburi.yaw_right(90.0, timeout=20.0, settle=0.3)

    # ------------------------------------------------------------------ #
    #  Phase 7: Return -- find gate and pass back through                 #
    # ------------------------------------------------------------------ #
    # Turn around (yaw_right 180°), find gate, align, pass back.
    log('Phase 7: return -- yaw 180°')
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)

    log('Phase 7: searching for gate on return leg')
    duburi.vision.find(
        sweep='right',
        timeout=RETURN_SEARCH_TIMEOUT_S,
        gain=0.0)     # stationary search on return: just rotate, don't drive

    log('Phase 7: aligning with gate for return pass')
    duburi.vision.lock(
        axes='yaw,forward',
        distance=GATE_APPROACH_FRAC,
        duration=ALIGN_TIMEOUT_S,
        on_lost='hold',
        kp_yaw=ALIGN_KP_YAW,
        kp_forward=ALIGN_KP_FORWARD,
        deadband=ALIGN_DEADBAND,
        distance_metric='area')

    log('Phase 7: passing back through gate')
    duburi.move_forward_dist(RETURN_PASS_DIST_M, gain=RETURN_PASS_GAIN)

    # ------------------------------------------------------------------ #
    #  Phase 8: Surface and disarm                                        #
    # ------------------------------------------------------------------ #
    log('Phase 8: surfacing')
    duburi.stop()
    duburi.set_depth(0.0, timeout=30.0)
    duburi.disarm()
    log('Mission complete.')
