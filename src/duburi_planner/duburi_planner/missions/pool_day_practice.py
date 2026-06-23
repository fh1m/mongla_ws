"""Pool day practice run: Gate → Slalom → Torpedo → Bin.

All tunable values are in the OPERATOR TUNABLES block below.
Edit this file, then re-run — no colcon build needed.

Launch:
    ros2 run duburi_planner mission pool_day_practice

Skip to a specific task: comment out earlier phases in run().
Ctrl-C at any time → AUV stops and disarms cleanly.
"""

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                         OPERATOR TUNABLES                                   │
# │   Edit these before each run. None = manual manoeuvre / skip that turn.     │
# └─────────────────────────────────────────────────────────────────────────────┘

# ── STARTUP ───────────────────────────────────────────────────────────────────
TETHER_REMOVE_PAUSE_S    = 5.0   # seconds to remove tether before thrusters arm

# ── DEPTHS (metres, negative = below surface) ─────────────────────────────────
GATE_SEARCH_DEPTH_M      = -0.4  # initial descent for whole run
GATE_PASS_DEPTH_M        = -0.6  # descend here before driving through gate
SLALOM_DEPTH_M           = -0.6  # depth to hold while weaving slalom
TORPEDO_DEPTH_M          = -0.7  # ★ FILL AT POOL — align with hole height
BIN_DEPTH_M              = -1.0  # deep enough for downward cam to see bin

# ── HEADINGS (degrees) — fill after compass survey, or leave None ──────────────
GATE_HEADING_DEG         = None  # heading to face gate at start (None = skip)
SLALOM_HEADING_DEG       = None  # heading from gate to slalom
TORPEDO_HEADING_DEG      = None  # heading from slalom to torpedo board
BIN_HEADING_DEG          = None  # heading from torpedo to bin

# ── GATE ──────────────────────────────────────────────────────────────────────
GATE_PASS_BBOX_FRAC      = 0.80  # gate bbox height fraction = "through gate"
GATE_ALIGN_DURATION_S    = 15    # max seconds to centre on gate
GATE_RESCUE_ALIGN_S      = 10    # max seconds to slide onto rescue/repair marker
GATE_APPROACH_DURATION_S = 25    # max seconds to drive through (increase if slow)

# ── SLALOM ────────────────────────────────────────────────────────────────────
SLALOM_PIPE_COUNT        = 3     # number of pipes to pass
SLALOM_PIPE_OFFSET_PX    = 80    # lateral pixel offset from pipe centre (positive = right)
SLALOM_PER_PIPE_DURATION_S = 25  # max seconds per pipe

# ── TORPEDO ───────────────────────────────────────────────────────────────────
TORPEDO_BOARD_ALIGN_S    = 15    # coarse align on full board
TORPEDO_BLOOD_ALIGN_S    = 12    # fine align on blood marker
TORPEDO_FIRE_CHANNEL     = 1     # 1=torpedo_1, 2=torpedo_2
TORPEDO_STABLE_LOCK_S    = 3.0   # seconds of stable lock before firing
TORPEDO_DEADBAND         = 0.04  # fraction of frame — jitter < this = stable
TORPEDO_KP_YAW           = 80.0  # decrease if yaw oscillates
TORPEDO_KP_LAT           = 80.0  # decrease if lateral oscillates
TORPEDO_KP_DEPTH         = 0.08  # decrease if depth oscillates
TORPEDO_MAX_ATTEMPTS     = 3     # fire retries before giving up
TORPEDO_LOCK_DURATION_S  = 60    # total time budget for lock+fire phase

# ── BIN ───────────────────────────────────────────────────────────────────────
BIN_ALIGN_DURATION_S     = 20    # max seconds to position above fire marker
BIN_KP_LAT               = 60.0  # lateral gain (downward cam)
BIN_KP_FWD               = -60.0 # NEGATIVE — downward cam: ey>0 means target is AFT
BIN_DEADBAND             = 0.06  # fraction of frame — jitter < this = stable
BIN_STABILITY_PAUSE_S    = 3.0   # hold steady this long before dropping
BIN_DROP_CHANNEL         = 3     # 3=dropper_1, 4=dropper_2

# ── SEARCH (shared across all tasks) ─────────────────────────────────────────
SEARCH_FORWARD_GAIN      = 40    # % gain for slow creep during search
SEARCH_MAX_STEPS         = 60    # creep steps before falling back to yaw scan
SEARCH_SCAN_STEP         = 15    # degrees per scan step
SEARCH_SCAN_DWELL_S      = 1.5   # seconds at each scan step

# ─────────────────────────────────────────────────────────────────────────────

_FWD = '/duburi_detector_fwd'
_DWN = '/duburi_detector_dwn'


def _find(duburi, target, camera='forward', scan_speed=40, scan_duration=60):
    """Creep forward until target detected; fall back to yaw scan. Returns True if found."""
    for _ in range(SEARCH_MAX_STEPS):
        if duburi.detected(target, camera=camera, stale_after=1.0):
            return True
        duburi.move_forward(0.5, gain=SEARCH_FORWARD_GAIN)
    result = duburi.vision.scan(
        target=target, camera=camera,
        step=SEARCH_SCAN_STEP, dwell=SEARCH_SCAN_DWELL_S,
        speed=scan_speed, duration=scan_duration)
    return result.success


def run(duburi, log=None):
    duburi.mission_reset()   # clear heading lock + abort from any previous run

    def info(msg):
        if log:
            log(msg)

    try:
        # ── ARM + INITIAL SET ─────────────────────────────────────────────────
        info('[practice] pausing to remove tether...')
        duburi.pause(TETHER_REMOVE_PAUSE_S)
        duburi.arm()
        duburi.set_depth(GATE_SEARCH_DEPTH_M, timeout=30)
        if GATE_HEADING_DEG is not None:
            duburi.turn(GATE_HEADING_DEG)
        duburi.lock_heading(0.0, timeout=600)

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 1: GATE
        # Find gate → align to gate centre → slide to rescue marker →
        # descend to pass depth → drive through gate.
        # ══════════════════════════════════════════════════════════════════════
        info('[gate] searching...')
        duburi.resume_detector('forward')
        duburi.set_model('gate_rescue_repair', node=_FWD)
        duburi.set_classes('gate,rescue,repair', node=_FWD)

        if _find(duburi, 'gate'):
            duburi.vision.home(
                target='gate', camera='forward',
                yaw=True, lat=True, on_lost='hold',
                duration=GATE_ALIGN_DURATION_S)

            duburi.set_classes('rescue,repair', node=_FWD)
            duburi.vision.home(
                target='rescue', camera='forward',
                yaw=False, lat=True, on_lost='hold',
                duration=GATE_RESCUE_ALIGN_S)

            duburi.set_depth(GATE_PASS_DEPTH_M, timeout=20)
            duburi.set_classes('gate', node=_FWD)
            duburi.vision.approach(
                target='gate', camera='forward',
                dist=GATE_PASS_BBOX_FRAC, metric='height',
                on_lost='hold', duration=GATE_APPROACH_DURATION_S,
                lock_mode='pursue')
            duburi.pause(2.0)
            info('[gate] passed through gate')
        else:
            info('[gate] NOT FOUND — skipping to slalom')

        duburi.pause_detector('forward')

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 2: SLALOM
        # Manual heading turn → find first red pipe → offset right by
        # SLALOM_PIPE_OFFSET_PX and drive through SLALOM_PIPE_COUNT pipes.
        # ══════════════════════════════════════════════════════════════════════
        info('[slalom] searching...')
        if SLALOM_HEADING_DEG is not None:
            duburi.turn(SLALOM_HEADING_DEG)
        duburi.set_depth(SLALOM_DEPTH_M, timeout=20)

        duburi.resume_detector('forward')
        duburi.set_model('slalom_red_pipe', node=_FWD)
        duburi.set_classes('red_pipe', node=_FWD)

        if _find(duburi, 'red_pipe'):
            for pipe_num in range(1, SLALOM_PIPE_COUNT + 1):
                info(f'[slalom] pipe {pipe_num}/{SLALOM_PIPE_COUNT}')
                duburi.vision.home(
                    target='red_pipe', camera='forward',
                    yaw=True, lat=True,
                    offset_x=SLALOM_PIPE_OFFSET_PX,   # always right
                    forward=True, on_lost='hold',
                    duration=SLALOM_PER_PIPE_DURATION_S)
                duburi.pause(0.5)
            info('[slalom] all pipes done')
        else:
            info('[slalom] NOT FOUND — skipping to torpedo')

        duburi.pause_detector('forward')

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 3: TORPEDO
        # Manual turn → find board → coarse align (board) → fine align (blood) →
        # stable lock on hole → fire torpedo.
        # ══════════════════════════════════════════════════════════════════════
        info('[torpedo] searching...')
        if TORPEDO_HEADING_DEG is not None:
            duburi.turn(TORPEDO_HEADING_DEG)
        duburi.set_depth(TORPEDO_DEPTH_M, timeout=30)

        duburi.resume_detector('forward')
        duburi.set_model('torpedo_blood_hole', node=_FWD)
        duburi.set_classes('torpedo', node=_FWD)

        if _find(duburi, 'torpedo'):
            duburi.vision.home(
                target='torpedo', camera='forward',
                yaw=True, lat=True, depth=True, on_lost='hold',
                duration=TORPEDO_BOARD_ALIGN_S)

            duburi.set_classes('blood,hole', node=_FWD)
            duburi.vision.home(
                target='blood', camera='forward',
                yaw=True, lat=True, depth=True, on_lost='hold',
                duration=TORPEDO_BLOOD_ALIGN_S)

            duburi.vision.vision_lock_fire(
                target='hole', camera='forward',
                fire_channel=TORPEDO_FIRE_CHANNEL,
                yaw=True, lat=True, depth=True, forward=False,
                stable_lock_s=TORPEDO_STABLE_LOCK_S,
                max_attempts=TORPEDO_MAX_ATTEMPTS,
                deadband=TORPEDO_DEADBAND,
                kp_yaw=TORPEDO_KP_YAW,
                kp_lat=TORPEDO_KP_LAT,
                kp_depth=TORPEDO_KP_DEPTH,
                duration=TORPEDO_LOCK_DURATION_S)

            duburi.pause(2.0)
            info('[torpedo] fired')
        else:
            info('[torpedo] board NOT FOUND — skipping to bin')

        duburi.pause_detector('forward')

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 4: BIN
        # Manual turn → descend deep → switch to downward camera → find fire
        # marker → centre above it → drop.
        # NOTE: BIN_KP_FWD must be NEGATIVE for downward cam.
        #   ey > 0 means target is AFT of AUV, so positive error → drive backward.
        # ══════════════════════════════════════════════════════════════════════
        info('[bin] searching...')
        if BIN_HEADING_DEG is not None:
            duburi.turn(BIN_HEADING_DEG)
        duburi.set_depth(BIN_DEPTH_M, timeout=30)

        duburi.use_camera('downward')
        duburi.resume_detector('downward')
        duburi.set_model('bin_fire_blood', node=_DWN)
        duburi.set_classes('fire,blood', node=_DWN)

        if _find(duburi, 'fire', camera='downward', scan_speed=30):
            duburi.vision.home(
                target='fire', downward_cam=True,
                lat=True, forward=True, yaw=False, depth=False,
                kp_lat=BIN_KP_LAT,
                kp_forward=BIN_KP_FWD,
                deadband=BIN_DEADBAND,
                on_lost='hold', duration=BIN_ALIGN_DURATION_S)

            info('[bin] aligned — holding for stability...')
            duburi.pause(BIN_STABILITY_PAUSE_S)
            duburi.fire(BIN_DROP_CHANNEL)
            duburi.pause(2.0)
            info('[bin] drop complete')
        else:
            info('[bin] fire NOT FOUND — skipping drop')

        duburi.pause_detector('downward')
        duburi.use_camera('forward')

    except Exception as exc:
        if log:
            log(f'[practice] ABORT: {exc}')
        raise
    finally:
        duburi.release_heading()
        duburi.stop()
        duburi.disarm()
        info('[practice] disarmed — run complete')
