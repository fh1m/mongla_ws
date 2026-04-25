#!/usr/bin/env python3
"""gate_flare_prequal -- fully autonomous RoboSub gate+flare pre-qualification.

Course layout (from 2026 briefing image):
  Start → [~3 m] → Gate (2 m wide, ~1 m tall, ~1 m below surface)
         → [~10 m] → Flare/marker (vertical pipe, centre of field)
         → [return ~13 m] → Gate → finish at start

Sequence:
  0. Countdown (operator removes tether in this window)
  1. Arm, engage ALT_HOLD, descend to gate depth
  2. Connect DVL
  3. Load gate+flare combined model, start with 'gate' class filter
  4. Search: drive forward slowly while watching for gate
  5. Align: centre on gate (yaw + forward distance) with on_lost='hold'
  6. Pass through gate: DVL forward 3.5 m (timed fallback)
  7. Switch class filter to 'flare'
  8. Search: sweep right looking for flare
  9. Approach flare: yaw + forward + depth lock at standoff
  10. Orbit flare: 12 x 30-degree yaw steps x re-lock (360-degree polygon)
  11. Turn 180 degrees to face return heading
  12. Switch class filter back to 'gate'
  13. Reacquire gate: short sweep + align
  14. Pass back through gate: DVL forward 3.5 m
  15. Surface and disarm

Bounding-box robustness notes (tuned from underwater preview footage):
  - Gate partial/off-edge: yaw axis still has center_x signal; vehicle turns
    toward visible post. Works with as little as 20% of gate in frame.
  - Gate too close (fills 90%+): forward axis backs off until GATE_STANDOFF.
    distance_metric='area' is more stable for wide objects than 'height'.
  - Flare narrow at distance: force 'height' metric; flare is taller than wide.
  - High-confidence models (0.90-0.97 in murky water): conf=0.45 eliminates
    background noise without losing any real detections.

Pre-flight:
  ros2 launch duburi_manager bringup.launch.py vision:=true
  # bringup.launch.py loads detector.yaml which now defaults to
  # gate_flare_medium_100ep with conf=0.45 and classes=gate

  ros2 run duburi_vision vision_check --camera forward --require-class gate
  ros2 run duburi_vision vision_thrust_check --camera forward --duration 4

  ros2 run duburi_planner mission gate_flare_prequal

Live tuning (between runs, no rebuild):
  ros2 param set /duburi_manager vision.kp_yaw      70.0
  ros2 param set /duburi_manager vision.kp_forward  180.0
  ros2 param set /duburi_manager vision.deadband     0.08

WARNING: this mission arms the vehicle and removes the tether.
         Run in Gazebo sim first. Have a safety diver / kill switch ready.
"""

# ── Camera / model -----------------------------------------------------------
CAMERA      = 'forward'    # forward camera profile name
GATE_CLASS  = 'gate'
FLARE_CLASS = 'flare'
# Model is set at launch via detector.yaml (gate_flare_medium_100ep).
# class filter is switched live by duburi.set_classes() during the mission.

# ── Depth -------------------------------------------------------------------
DIVE_DEPTH_M     = -1.0    # 1 m below surface; gate sits here
DEPTH_SETTLE_S   = 2.0     # let depth hold settle before moving

# ── Tether countdown --------------------------------------------------------
COUNTDOWN_S = 10           # seconds to disconnect tether

# ── Gate search -------------------------------------------------------------
GATE_SEARCH_GAIN  = 40.0   # gentle forward thrust while scanning
GATE_SEARCH_YAW   = 20.0   # slow yaw sweep rate
GATE_SEARCH_T     = 45.0   # give up after this many seconds

# ── Gate alignment ----------------------------------------------------------
GATE_STANDOFF     = 0.42   # gate fills ~42% of frame AREA at standoff
GATE_ALIGN_T      = 20.0   # max seconds for lock to settle
GATE_KP_YAW       = 70.0
GATE_KP_FWD       = 160.0
GATE_DEADBAND     = 0.08

# ── Gate passage ------------------------------------------------------------
GATE_PASS_M       = 3.5    # DVL metres to fully clear gate
GATE_PASS_GAIN    = 60.0
GATE_PASS_T       = 5.0    # timed fallback (DVL unavailable)

# ── Flare search ------------------------------------------------------------
FLARE_SEARCH_YAW  = 25.0
FLARE_SEARCH_T    = 40.0

# ── Flare approach ----------------------------------------------------------
FLARE_STANDOFF    = 0.38   # flare fills ~38% of frame height (narrow pipe)
FLARE_ALIGN_T     = 20.0
FLARE_KP_YAW      = 65.0
FLARE_KP_FWD      = 180.0

# ── Flare orbit (12 x 30 deg = 360 deg polygon) ----------------------------
ORBIT_STEP_DEG    = 30.0
ORBIT_STEPS       = 12
ORBIT_STEP_T      = 10.0   # per-step yaw timeout
ORBIT_SETTLE_S    = 0.3
ORBIT_LOCK_T      = 3.0    # re-lock duration after each step

# ── Return ------------------------------------------------------------------
RETURN_SEARCH_T   = 30.0
RETURN_PASS_M     = 3.5
RETURN_PASS_GAIN  = 60.0
RETURN_PASS_T     = 5.0


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = GATE_CLASS

    # ── Phase 0: Tether removal window -------------------------------------
    log('Phase 0: tether removal countdown')
    duburi.countdown(
        COUNTDOWN_S,
        message='Wire removed  --  Duburi is now autonomous. Good luck.')

    # ── Phase 1: Startup ---------------------------------------------------
    log('Phase 1: arm + ALT_HOLD + descend')
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=DEPTH_SETTLE_S)

    # ── Phase 2: DVL connect -----------------------------------------------
    # No-op when yaw_source is not nucleus_dvl. Aborts with a clear error
    # if DVL TCP fails -- call dvl_connect manually before the mission if
    # the hardware is uncertain.
    log('Phase 2: DVL connect')
    duburi.dvl_connect()

    # ── Phase 3: Class filter: gate ----------------------------------------
    # gate_flare_medium_100ep has both classes; only publish gate detections
    # during the gate approach phase to prevent flare false-positives.
    log('Phase 3: set class filter -> gate')
    duburi.set_classes(GATE_CLASS)

    # ── Phase 4: Search for gate -------------------------------------------
    # Drive forward at GATE_SEARCH_GAIN% while watching for gate class.
    # yaw_rate_pct=20 adds a gentle search sweep if AUV is slightly off-axis.
    log('Phase 4: searching for gate (forward sweep)')
    duburi.vision.find(
        target=GATE_CLASS,
        sweep='forward',
        timeout=GATE_SEARCH_T,
        gain=GATE_SEARCH_GAIN,
        yaw_rate_pct=GATE_SEARCH_YAW)

    # ── Phase 5: Align with gate -------------------------------------------
    # yaw + forward simultaneously. distance_metric='area' is stable for
    # wide rectangular gates. on_lost='hold' rides out detection flickers
    # common in turbid pool water.
    log('Phase 5: aligning with gate')
    duburi.vision.lock(
        target=GATE_CLASS,
        axes='yaw,forward',
        distance=GATE_STANDOFF,
        duration=GATE_ALIGN_T,
        on_lost='hold',
        kp_yaw=GATE_KP_YAW,
        kp_forward=GATE_KP_FWD,
        deadband=GATE_DEADBAND,
        distance_metric='area')

    # ── Phase 6: Pass through gate -----------------------------------------
    log('Phase 6: passing through gate (DVL forward)')
    duburi.move_forward_dist(GATE_PASS_M, gain=GATE_PASS_GAIN)

    # ── Phase 7: Switch to flare detection ---------------------------------
    log('Phase 7: set class filter -> flare')
    duburi.set_classes(FLARE_CLASS)
    duburi.target = FLARE_CLASS

    # ── Phase 8: Search for flare ------------------------------------------
    # Sweep right from the heading we came through the gate.
    log('Phase 8: searching for flare (right sweep)')
    duburi.vision.find(
        target=FLARE_CLASS,
        sweep='right',
        timeout=FLARE_SEARCH_T,
        gain=0.0,             # stationary sweep; flare is only ~10 m away
        yaw_rate_pct=FLARE_SEARCH_YAW)

    # ── Phase 9: Approach flare --------------------------------------------
    # 3-axis lock: yaw + forward + depth. 'height' metric for the tall narrow
    # pipe. Depth axis keeps the flare vertically centred across approaches.
    log('Phase 9: approaching flare (3-axis lock)')
    duburi.vision.lock(
        target=FLARE_CLASS,
        axes='yaw,forward,depth',
        distance=FLARE_STANDOFF,
        duration=FLARE_ALIGN_T,
        on_lost='hold',
        kp_yaw=FLARE_KP_YAW,
        kp_forward=FLARE_KP_FWD,
        lock_mode='settle')

    # ── Phase 10: Orbit flare (360-degree polygon) --------------------------
    # 12 steps x 30 degrees = 360 degrees.
    # Each step: yaw_left 30 deg -> flare goes off-centre -> re-lock centres it.
    log('Phase 10: orbiting flare (12 x 30 deg)')
    for step in range(ORBIT_STEPS):
        log(f'  orbit step {step + 1}/{ORBIT_STEPS} '
            f'({(step + 1) * ORBIT_STEP_DEG:.0f} deg total)')
        duburi.yaw_left(
            ORBIT_STEP_DEG,
            timeout=ORBIT_STEP_T,
            settle=ORBIT_SETTLE_S)
        duburi.vision.lock(
            target=FLARE_CLASS,
            axes='yaw,forward,depth',
            distance=FLARE_STANDOFF,
            duration=ORBIT_LOCK_T,
            on_lost='hold',
            lock_mode='follow')

    # ── Phase 11: Turn 180 to return heading --------------------------------
    log('Phase 11: yaw 180 deg to return heading')
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)

    # ── Phase 12: Switch back to gate detection ----------------------------
    log('Phase 12: set class filter -> gate')
    duburi.set_classes(GATE_CLASS)
    duburi.target = GATE_CLASS

    # ── Phase 13: Reacquire gate on return ----------------------------------
    log('Phase 13: searching for gate (return leg, stationary sweep)')
    duburi.vision.find(
        target=GATE_CLASS,
        sweep='right',
        timeout=RETURN_SEARCH_T,
        gain=0.0)

    log('Phase 13: aligning with gate for return pass')
    duburi.vision.lock(
        target=GATE_CLASS,
        axes='yaw,forward',
        distance=GATE_STANDOFF,
        duration=GATE_ALIGN_T,
        on_lost='hold',
        kp_yaw=GATE_KP_YAW,
        kp_forward=GATE_KP_FWD,
        deadband=GATE_DEADBAND,
        distance_metric='area')

    # ── Phase 14: Return through gate ----------------------------------------
    log('Phase 14: passing back through gate (DVL forward)')
    duburi.move_forward_dist(RETURN_PASS_M, gain=RETURN_PASS_GAIN)

    # ── Phase 15: Surface and disarm ----------------------------------------
    log('Phase 15: surfacing and disarming')
    duburi.stop()
    duburi.set_depth(0.0, timeout=30.0)
    duburi.disarm()
    log('Mission complete.')
