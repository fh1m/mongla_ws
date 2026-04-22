#!/usr/bin/env python3
"""robosub_prequal -- RoboNation pre-qualification sequence.

Course layout (from competitor briefing):
  Start → [3 m] → Gate (2 m wide, 1 m tall, ~1 m below surface)
                → [10 m] → Marker/Flare (vertical yellow pipe)

Sequence:
  1. Arm, engage depth-hold, dive to gate depth
  2. Search forward until the gate is detected
  3. Centre on the gate and approach to standoff distance
  4. Strafe through the left side of the gate
  5. Drive forward to fully clear the gate frame
  6. Search right-sweeping for the flare marker
  7. Approach the flare and hold 3-axis lock (yaw + forward + depth)
  8. Orbit the flare: 12 × (rotate 30° + re-lock) ≈ 360° polygon orbit
  9. Rotate 180° to face the return heading
  10. Reacquire the gate, align, and pass through (return leg)
  11. Surface and disarm

Orbit technique note:
  Each orbit step rotates the vehicle 30° in place with yaw_left, moving
  the flare off-centre, then vision.lock re-centres and holds standoff
  distance. Twelve steps × 30° = 360°. The path is a 12-sided polygon
  (not a smooth circle) but this is sufficient for pre-qualification.
  Alternative: replace the loop with arc() for a curved path.

Tune these at pool before the run:
  ros2 param set /duburi_manager vision.kp_yaw      60.0
  ros2 param set /duburi_manager vision.kp_forward  200.0
  ros2 param set /duburi_manager vision.deadband     0.12

WARNING: this mission arms the vehicle. Verify camera angle and
model detects gate + flare before pool day.
"""

# ── Target classes (must match your YOLO model's class names) ─────────────── #
GATE_CLASS  = 'gate'
FLARE_CLASS = 'flare'
CAMERA      = 'forward'   # forward-facing camera profile name

# ── Depth (negative = below surface, ArduSub convention) ─────────────────── #
MISSION_DEPTH_METERS        = -1.0   # gate sits ~1 m below surface
DEPTH_SETTLE_SECONDS        = 2.0   # wait after reaching depth before moving

# ── Gate approach ─────────────────────────────────────────────────────────── #
GATE_SEARCH_GAIN_PCT        = 40.0  # gentle forward thrust while searching
GATE_SEARCH_TIMEOUT_SECONDS = 45.0  # abort search after this long
GATE_SEARCH_YAW_RATE_PCT    = 20.0  # slow yaw during search sweep
GATE_STANDOFF_FRACTION      = 0.45  # gate fills 45% of frame height at target
GATE_ALIGN_DURATION_SECONDS = 20.0  # max time to hold gate alignment

# ── Gate pass (strafe + drive) ─────────────────────────────────────────────── #
GATE_STRAFE_DURATION_SECONDS = 4.0  # strafe duration to clear one gate post
GATE_STRAFE_GAIN_PCT         = 55.0
GATE_DRIVE_DURATION_SECONDS  = 3.0  # forward drive to fully clear the gate frame
GATE_DRIVE_GAIN_PCT          = 60.0

# ── Flare approach ─────────────────────────────────────────────────────────── #
FLARE_SEARCH_TIMEOUT_SECONDS = 35.0
FLARE_SEARCH_YAW_RATE_PCT    = 25.0
FLARE_STANDOFF_FRACTION      = 0.40  # flare fills 40% of frame height at target
FLARE_ALIGN_DURATION_SECONDS = 20.0

# ── Flare orbit ───────────────────────────────────────────────────────────── #
ORBIT_STEP_DEGREES           = 30.0  # degrees per yaw step
ORBIT_STEPS_TOTAL            = 12    # 12 × 30° = 360°
ORBIT_STEP_TIMEOUT_SECONDS   = 10.0  # abort if yaw step takes longer than this
ORBIT_SETTLE_SECONDS         = 0.3   # brief pause after each yaw step
ORBIT_LOCK_DURATION_SECONDS  = 3.0   # re-lock duration after each step

# ── Return through gate ───────────────────────────────────────────────────── #
RETURN_TURN_DEGREES          = 180.0
RETURN_SEARCH_TIMEOUT_SECONDS = 30.0
RETURN_ALIGN_DURATION_SECONDS = 20.0
RETURN_DRIVE_DURATION_SECONDS = 5.0  # longer to fully clear gate on return leg
RETURN_DRIVE_GAIN_PCT         = 60.0


def run(duburi, log):
    duburi.camera = CAMERA

    # ── Phase 1: Startup ──────────────────────────────────────────────────── #
    log('Phase 1: arming and diving...')
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    # Descend to gate depth and wait for oscillations to settle before moving.
    duburi.set_depth(MISSION_DEPTH_METERS, settle=DEPTH_SETTLE_SECONDS)

    # ── Phase 2: Find the gate ─────────────────────────────────────────────── #
    log('Phase 2: searching for gate (forward sweep)...')
    # Drive slowly forward while scanning for the gate. A gentle yaw sweep
    # widens the search cone if the gate is slightly off-axis.
    duburi.vision.find(
        target=GATE_CLASS,
        sweep='forward',
        timeout=GATE_SEARCH_TIMEOUT_SECONDS,
        gain=GATE_SEARCH_GAIN_PCT,
        yaw_rate_pct=GATE_SEARCH_YAW_RATE_PCT,
    )

    # ── Phase 3: Align with the gate ─────────────────────────────────────────  #
    log('Phase 3: centering on gate and approaching to standoff...')
    # Use 'area' distance metric for the gate: the gate is wide so bbox area
    # tracks distance better than raw height fraction alone.
    duburi.vision.lock(
        target=GATE_CLASS,
        axes='yaw,forward',
        distance=GATE_STANDOFF_FRACTION,
        distance_metric='area',
        duration=GATE_ALIGN_DURATION_SECONDS,
        on_lost='hold',
        lock_mode='settle',
    )

    # ── Phase 4: Pass through the gate ─────────────────────────────────────── #
    log('Phase 4: passing through gate (left side)...')
    # Strafe past the left gate post, then drive forward through the opening.
    # TODO pool calibration: tune duration vs measured gate width at approach.
    duburi.move_left(GATE_STRAFE_DURATION_SECONDS, gain=GATE_STRAFE_GAIN_PCT)
    duburi.move_forward(GATE_DRIVE_DURATION_SECONDS, gain=GATE_DRIVE_GAIN_PCT)

    # ── Phase 5: Find the flare ───────────────────────────────────────────── #
    log('Phase 5: searching for flare (right sweep)...')
    duburi.vision.find(
        target=FLARE_CLASS,
        sweep='right',
        timeout=FLARE_SEARCH_TIMEOUT_SECONDS,
        gain=GATE_SEARCH_GAIN_PCT,
        yaw_rate_pct=FLARE_SEARCH_YAW_RATE_PCT,
    )

    # ── Phase 6: Approach the flare ──────────────────────────────────────── #
    log('Phase 6: approaching flare (3-axis lock)...')
    # Hold yaw, forward distance, AND depth simultaneously. The depth axis
    # nudges the depth setpoint so the flare stays vertically centred in frame.
    duburi.vision.lock(
        target=FLARE_CLASS,
        axes='yaw,forward,depth',
        distance=FLARE_STANDOFF_FRACTION,
        duration=FLARE_ALIGN_DURATION_SECONDS,
        on_lost='hold',
        lock_mode='settle',
    )

    # ── Phase 7: Orbit the flare (12 × 30° = 360°) ────────────────────────── #
    log('Phase 7: orbiting flare...')
    for step_number in range(ORBIT_STEPS_TOTAL):
        log(f'  orbit step {step_number + 1}/{ORBIT_STEPS_TOTAL} '
            f'({(step_number + 1) * ORBIT_STEP_DEGREES:.0f}° total)')

        # Rotate 30° counter-clockwise. The flare drifts off-centre after
        # the turn — the vision.lock call below re-centres it.
        duburi.yaw_left(
            ORBIT_STEP_DEGREES,
            timeout=ORBIT_STEP_TIMEOUT_SECONDS,
            settle=ORBIT_SETTLE_SECONDS,
        )

        # Re-lock onto the flare at this new angle. lock_mode='follow' keeps
        # adjusting without exiting on settle, so the vehicle actively holds
        # the flare in frame for the full ORBIT_LOCK_DURATION_SECONDS window.
        duburi.vision.lock(
            target=FLARE_CLASS,
            axes='yaw,forward,depth',
            distance=FLARE_STANDOFF_FRACTION,
            duration=ORBIT_LOCK_DURATION_SECONDS,
            on_lost='hold',
            lock_mode='follow',
        )

    # ── Phase 8: Face the return heading ─────────────────────────────────── #
    log('Phase 8: rotating 180° to return heading...')
    duburi.yaw_right(RETURN_TURN_DEGREES, timeout=20.0)

    # ── Phase 9: Reacquire the gate on return ────────────────────────────── #
    log('Phase 9: searching for gate (return leg)...')
    duburi.vision.find(
        target=GATE_CLASS,
        sweep='right',
        timeout=RETURN_SEARCH_TIMEOUT_SECONDS,
        gain=GATE_SEARCH_GAIN_PCT,
        yaw_rate_pct=GATE_SEARCH_YAW_RATE_PCT,
    )
    duburi.vision.lock(
        target=GATE_CLASS,
        axes='yaw,forward',
        distance=GATE_STANDOFF_FRACTION,
        distance_metric='area',
        duration=RETURN_ALIGN_DURATION_SECONDS,
        on_lost='hold',
        lock_mode='settle',
    )

    # ── Phase 10: Pass through gate (return) ─────────────────────────────── #
    log('Phase 10: passing through gate (return leg)...')
    duburi.move_left(GATE_STRAFE_DURATION_SECONDS, gain=GATE_STRAFE_GAIN_PCT)
    duburi.move_forward(RETURN_DRIVE_DURATION_SECONDS, gain=RETURN_DRIVE_GAIN_PCT)

    # ── Shutdown ──────────────────────────────────────────────────────────── #
    log('Shutdown: surfacing and disarming...')
    duburi.stop()
    duburi.set_depth(0.0)
    duburi.disarm()
