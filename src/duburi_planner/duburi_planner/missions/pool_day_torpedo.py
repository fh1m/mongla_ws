"""Pool day torpedo practice: approach board → coarse align → fine-lock hole → fire.

Tests the full proximity-scaling + stable-lock-fire chain:
  - speed/h_frac_close: AUV slows as it gets close to hole (bbox grows)
  - stable_lock_s:      must hold still for N seconds before firing
  - vision_lock_fire:   aligns + triggers ESP32 serial fire in one verb

All tunables are in the OPERATOR TUNABLES block below.
Edit values here; no colcon build needed between runs.

Launch:
    ros2 run duburi_planner mission pool_day_torpedo

Ctrl-C at any time → AUV stops and disarms cleanly.
"""

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                         OPERATOR TUNABLES                                   │
# └─────────────────────────────────────────────────────────────────────────────┘

# ── STARTUP ───────────────────────────────────────────────────────────────────
TETHER_REMOVE_PAUSE_S    = 5.0     # remove tether before thrusters arm

# ── DEPTH ─────────────────────────────────────────────────────────────────────
TORPEDO_DEPTH_M          = None    # ★ FILL AT POOL — align with hole height (e.g. -0.8)

# ── HEADING (degrees, or None = skip) ─────────────────────────────────────────
TORPEDO_HEADING_DEG      = None    # face board before searching

# ── SEARCH ────────────────────────────────────────────────────────────────────
SEARCH_FORWARD_GAIN      = 40      # % thrust while sweeping forward for board
SEARCH_MAX_STEPS         = 8       # forward steps before giving up
SEARCH_STEP_S            = 1.0     # seconds per forward step

# ── COARSE BOARD ALIGN (yaw + lat on 'torpedo' bbox) ─────────────────────────
BOARD_ALIGN_SPEED        = 0.55    # gain scalar (0-1), fast since far from board
BOARD_ALIGN_H_FRAC_CLOSE = 0.40   # bbox fraction at which proximity kicks in
BOARD_ALIGN_DURATION_S   = 20      # max seconds to centre on board
BOARD_DEADBAND           = 0.12    # fraction of frame — coarse, board is big

# ── FINE HOLE LOCK (yaw + lat + depth on 'hole' bbox) ────────────────────────
HOLE_ALIGN_SPEED         = 0.25   # slow — close to hole, must be precise
HOLE_ALIGN_H_FRAC_CLOSE  = 0.30   # hole fills 30% frame height = fully close
HOLE_ALIGN_DURATION_S    = 30     # max seconds to dial in

# ── FIRE ─────────────────────────────────────────────────────────────────────
FIRE_CHANNEL             = 1       # 1=torpedo_1, 2=torpedo_2
FIRE_STABLE_LOCK_S       = 2.5    # ★ TUNE UP at pool: seconds of stillness before fire
FIRE_DEADBAND            = 0.05   # tight deadband — must be very centred to count
FIRE_SPEED               = 0.15   # very slow — barely correcting when near-locked
FIRE_H_FRAC_CLOSE        = 0.25   # proximity scaling at this bbox fraction
FIRE_MAX_ATTEMPTS        = 3      # retry attempts before fallback-fire

_FWD = '/duburi_detector_fwd'


def run(duburi, log=None):
    duburi.mission_reset()   # clear heading lock + abort from any previous run

    def info(msg):
        if log:
            log(msg)

    assert TORPEDO_DEPTH_M is not None, (
        'TORPEDO_DEPTH_M not set — measure hole height at pool and edit this file')

    # ── 0. Countdown ──────────────────────────────────────────────────────────
    import time
    info(f'Pool-day torpedo practice — removing tether in {TETHER_REMOVE_PAUSE_S:.0f}s ...')
    time.sleep(TETHER_REMOVE_PAUSE_S)

    # ── 1. Arm + depth ────────────────────────────────────────────────────────
    duburi.arm()
    duburi.set_depth(TORPEDO_DEPTH_M)
    info(f'Armed. Descending to {TORPEDO_DEPTH_M}m ...')
    time.sleep(4.0)   # let depth settle before vision starts

    # ── 2. Optional heading turn to face board ────────────────────────────────
    if TORPEDO_HEADING_DEG is not None:
        info(f'Turning to torpedo heading {TORPEDO_HEADING_DEG}°')
        duburi.turn(TORPEDO_HEADING_DEG)

    # ── 3. Set up detector ────────────────────────────────────────────────────
    duburi.resume_detector('forward')
    duburi.set_model('torpedo_hole', node=_FWD)        # ★ verify model name matches yaml
    duburi.set_classes('torpedo,hole', node=_FWD)

    # ── 4. Search for board ───────────────────────────────────────────────────
    info('Searching for torpedo board ...')
    for _ in range(SEARCH_MAX_STEPS):
        if duburi.detected('torpedo', stale_after=1.0):
            break
        duburi.move_forward(SEARCH_STEP_S, gain=SEARCH_FORWARD_GAIN)
    else:
        # on_lost='search' used during align so board-lost will re-sweep
        info('Warning: torpedo board not detected during initial sweep — proceeding with align')

    # ── 5. Coarse align: centre on whole torpedo board (yaw + lat) ────────────
    #   Speed is moderate here — far from board, bbox is small, moves are safe.
    info('Coarse align: centring on torpedo board ...')
    duburi.vision.home(
        'torpedo',
        camera='forward',
        yaw=True, lat=True, depth=False, forward=False,
        speed=BOARD_ALIGN_SPEED,
        h_frac_close=BOARD_ALIGN_H_FRAC_CLOSE,
        deadband=BOARD_DEADBAND,
        timeout=BOARD_ALIGN_DURATION_S,
        on_lost='hold',   # hold last correction while board briefly disappears
        lock_mode='settle',
    )

    # ── 6. Switch to hole class ───────────────────────────────────────────────
    info('Switching detection to hole class for fine lock ...')
    duburi.set_classes('hole', node=_FWD)
    time.sleep(0.3)   # let one detection tick arrive before starting loop

    # ── 7. Fine lock + depth: very slow, proximity-scaled ────────────────────
    #   At HOLE_ALIGN_H_FRAC_CLOSE (bbox = 30% frame height), speed shrinks to
    #   HOLE_ALIGN_SPEED * proximity_min_scale (default 0.2) = 5% of full cap.
    #   AUV barely drifts when locked close — correct for fire.
    info('Fine lock on hole: yaw + lat + depth at slow speed ...')
    duburi.vision.home(
        'hole',
        camera='forward',
        yaw=True, lat=True, depth=True, forward=False,
        speed=HOLE_ALIGN_SPEED,
        h_frac_close=HOLE_ALIGN_H_FRAC_CLOSE,
        deadband=FIRE_DEADBAND,
        timeout=HOLE_ALIGN_DURATION_S,
        on_lost='hold',
        lock_mode='settle',
    )

    # ── 8. Stable-lock + fire ─────────────────────────────────────────────────
    #   vision_lock_fire holds fine lock until all axes stable for FIRE_STABLE_LOCK_S,
    #   then calls duburi.fire(FIRE_CHANNEL) via ESP32 serial.
    #   If unstable, retries up to FIRE_MAX_ATTEMPTS before fallback-firing.
    info(f'Lock-fire: need {FIRE_STABLE_LOCK_S:.1f}s stable → channel {FIRE_CHANNEL} ...')
    duburi.vision_lock_fire(
        camera='forward',
        target_class='hole',
        axes='yaw,lat,depth',
        duration=60.0,
        deadband=FIRE_DEADBAND,
        kp_yaw=60.0,   # ★ tune these gains at pool based on oscillation
        kp_lat=60.0,
        kp_depth=0.05,
        kp_forward=0.0,
        target_bbox_h_frac=0.0,
        on_lost='hold',
        stale_after=1.5,
        stable_lock_s=FIRE_STABLE_LOCK_S,
        max_attempts=FIRE_MAX_ATTEMPTS,
        attempt_timeout=20.0,
        fire_channel=float(FIRE_CHANNEL),
        speed=FIRE_SPEED,
        h_frac_close=FIRE_H_FRAC_CLOSE,
    )

    # ── 9. Disarm ─────────────────────────────────────────────────────────────
    info('Torpedo practice complete. Disarming.')
    duburi.stop()
    duburi.disarm()
