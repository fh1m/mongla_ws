"""Pool day practice run: Gate → Slalom → Torpedo → Bin.

Two-verb vision (align + move). All tunable values are in the OPERATOR
TUNABLES block below. Edit this file, then re-run — no colcon build needed.

Launch:
    ros2 run mongla_planner mission pool_day_practice

Skip to a specific task: comment out earlier phases in run().
Ctrl-C at any time → AUV stops and disarms cleanly.

A missed vision step never aborts the run — align()/move() log the miss
and the mission moves on. Search is the mission-authored fallbacks at the
bottom of this file (pure control), run automatically on target loss.
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

# ── ALIGNMENT (pixels) + SPEED CAPS (% thrust) ────────────────────────────────
ALIGN_ERR_PX             = 40    # "centred" pixel tolerance
FINE_ERR_PX              = 14    # tight tolerance for the torpedo hole lock
BIN_CENTRE_ERR_PX        = 30    # centring tolerance over the bin
BIN_SURGE_SIGN           = +1    # downward Ch5 fore/aft polarity -- verify DISARMED
                                 # (vision_thrust_check --camera downward); -1 if reversed
ALIGN_GAIN               = 30    # max speed while centring
APPROACH_GAIN            = 35    # max speed while driving forward
FINE_GAIN                = 12    # slow + precise for the fire lock

# ── FILL TARGETS (% of frame the bbox fills to count as reached) ──────────────
GATE_PASS_FWD_FILL       = 80    # gate height % = "through gate"
SLALOM_FWD_FILL          = 60    # pipe height % at closest pass
TORPEDO_BLOOD_FWD_FILL   = 30    # blood height % at end of approach

# ── PER-PHASE TIME BUDGETS (seconds) ──────────────────────────────────────────
GATE_ALIGN_DURATION_S    = 30
GATE_RESCUE_ALIGN_S      = 10
GATE_APPROACH_DURATION_S = 25
SLALOM_PIPE_COUNT        = 3
SLALOM_PIPE_OFFSET_PX    = 80    # signed lateral pixel offset (positive = right)
SLALOM_PER_PIPE_DURATION_S = 25
TORPEDO_BOARD_ALIGN_S    = 15
TORPEDO_APPROACH_S       = 30
TORPEDO_LOCK_S           = 25
TORPEDO_FIRE_CHANNEL     = 1     # 1=torpedo_1, 2=torpedo_2
BIN_ALIGN_DURATION_S     = 25
BIN_DROP_CHANNEL         = 3     # 3=dropper_1, 4=dropper_2
BIN_STABILITY_PAUSE_S    = 3.0

# ── SEARCH (mission-authored fallbacks) ───────────────────────────────────────
SEARCH_FORWARD_GAIN      = 40    # % gain for slow creep during search
SEARCH_CREEP_S           = 0.6   # seconds of forward creep per fallback cycle
SEARCH_YAW_STEP_DEG      = 15    # degrees per sweep look

# ─────────────────────────────────────────────────────────────────────────────

_FWD = '/mongla_detector_forward'
_DWN = '/mongla_detector_downward'


def run(mongla, log=None):
    mongla.mission_reset()   # clear heading lock + abort from any previous run

    def info(msg):
        if log:
            log(msg)

    try:
        # ── ARM + INITIAL SET ─────────────────────────────────────────────────
        info('[practice] pausing to remove tether...')
        mongla.pause(TETHER_REMOVE_PAUSE_S)
        mongla.arm()
        mongla.set_depth(GATE_SEARCH_DEPTH_M, timeout=30)
        if GATE_HEADING_DEG is not None:
            mongla.turn(GATE_HEADING_DEG)
        mongla.lock_heading(0.0, timeout=600)

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 1: GATE
        # ══════════════════════════════════════════════════════════════════════
        info('[gate] aligning...')
        mongla.resume_detector('forward')
        mongla.set_model('gate_rescue_repair', node=_FWD)
        mongla.set_classes('gate,rescue,repair', node=_FWD)

        mongla.vision.align(
            'gate', camera='forward', yaw=0, lat=0,
            err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
            duration=GATE_ALIGN_DURATION_S, fallback=creep_forward)

        mongla.set_classes('rescue,repair', node=_FWD)
        mongla.vision.align(
            'rescue', camera='forward', lat=0,
            err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=GATE_RESCUE_ALIGN_S,
            fallback=creep_forward)

        mongla.set_depth(GATE_PASS_DEPTH_M, timeout=20)
        mongla.set_classes('gate', node=_FWD)
        mongla.vision.move(
            'gate', camera='forward', fwd=GATE_PASS_FWD_FILL, mode='height',
            gain=APPROACH_GAIN, duration=GATE_APPROACH_DURATION_S,
            fallback=creep_forward)
        info('[gate] phase done')
        mongla.pause_detector('forward')

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 2: SLALOM
        # ══════════════════════════════════════════════════════════════════════
        info('[slalom] weaving...')
        if SLALOM_HEADING_DEG is not None:
            mongla.turn(SLALOM_HEADING_DEG)
        mongla.set_depth(SLALOM_DEPTH_M, timeout=20)

        mongla.resume_detector('forward')
        mongla.set_model('slalom_red_pipe', node=_FWD)
        mongla.set_classes('red_pipe', node=_FWD)

        for pipe_num in range(1, SLALOM_PIPE_COUNT + 1):
            sign = +1 if pipe_num % 2 else -1
            info(f'[slalom] pipe {pipe_num}/{SLALOM_PIPE_COUNT}')
            mongla.vision.align(
                'red_pipe', camera='forward',
                yaw=0, lat=sign * SLALOM_PIPE_OFFSET_PX,
                err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                duration=SLALOM_PER_PIPE_DURATION_S, fallback=sweep_forward)
            mongla.vision.move(
                'red_pipe', camera='forward', fwd=SLALOM_FWD_FILL, mode='height',
                maintain=sign * SLALOM_PIPE_OFFSET_PX,
                gain=APPROACH_GAIN, duration=15, fallback=creep_forward)
            mongla.pause(0.5)
        mongla.pause_detector('forward')

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 3: TORPEDO
        # ══════════════════════════════════════════════════════════════════════
        info('[torpedo] aligning...')
        if TORPEDO_HEADING_DEG is not None:
            mongla.turn(TORPEDO_HEADING_DEG)
        mongla.set_depth(TORPEDO_DEPTH_M, timeout=30)

        mongla.resume_detector('forward')
        mongla.set_model('torpedo_blood_hole', node=_FWD)
        mongla.set_classes('torpedo,blood,hole', node=_FWD)

        mongla.vision.align(
            'torpedo', camera='forward', yaw=0, lat=0, depth=0,
            err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
            duration=TORPEDO_BOARD_ALIGN_S, fallback=creep_forward)

        mongla.set_classes('blood,hole', node=_FWD)
        mongla.vision.move(
            'blood', camera='forward', fwd=TORPEDO_BLOOD_FWD_FILL, mode='height',
            gain=APPROACH_GAIN, duration=TORPEDO_APPROACH_S, fallback=creep_forward)

        mongla.set_classes('hole', node=_FWD)
        if mongla.vision.align(
                'hole', camera='forward', yaw=0, lat=0, depth=0,
                err=FINE_ERR_PX, gain=FINE_GAIN,
                duration=TORPEDO_LOCK_S, fallback=creep_forward):
            mongla.fire(TORPEDO_FIRE_CHANNEL)
            info('[torpedo] fired')
        else:
            info('[torpedo] hole never locked — holding fire')
        mongla.pause(2.0)
        mongla.pause_detector('forward')

        # ══════════════════════════════════════════════════════════════════════
        # PHASE 4: BIN  (downward camera)
        # ══════════════════════════════════════════════════════════════════════
        info('[bin] centring above bin...')
        if BIN_HEADING_DEG is not None:
            mongla.turn(BIN_HEADING_DEG)
        mongla.set_depth(BIN_DEPTH_M, timeout=30)

        # use_camera auto-switches the live detector to downward (pauses forward)
        # + flips the HUD. Downward frame: lat=left/right (Ch6), depth axis=fore/aft
        # SURGE (Ch5, two-sided + braked); ArduSub holds BIN_DEPTH_M on Ch3.
        mongla.use_camera('downward')
        mongla.set_model('bin_fire_blood', node=_DWN)
        mongla.set_classes('fire,blood', node=_DWN)

        if mongla.vision.align(
                'fire', camera='downward', lat=0, fwd=0,   # downward: lat=Ch6, fwd=Ch5 surge
                err=BIN_CENTRE_ERR_PX, gain=ALIGN_GAIN, surge_sign=BIN_SURGE_SIGN,
                duration=BIN_ALIGN_DURATION_S, fallback=creep_forward):
            info('[bin] aligned — holding for stability...')
            mongla.pause(BIN_STABILITY_PAUSE_S)
            mongla.fire(BIN_DROP_CHANNEL)
            info('[bin] drop complete')
        else:
            info('[bin] never centred — skipping drop')

        mongla.use_camera('forward')   # auto-pauses the downward detector

    except Exception as exc:
        if log:
            log(f'[practice] ABORT: {exc}')
        raise
    finally:
        mongla.release_heading()
        mongla.stop()
        mongla.disarm()
        info('[practice] disarmed — run complete')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    """One short forward creep, then return so the vision loop retries."""
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)


def sweep_forward(mongla, should_stop):
    """Creep forward while fanning yaw left/right; bail when target reappears."""
    heading = mongla.head()
    for step in (SEARCH_YAW_STEP_DEG, -2 * SEARCH_YAW_STEP_DEG,
                 2 * SEARCH_YAW_STEP_DEG):
        mongla.turn(heading + step)
        if should_stop():
            mongla.turn(heading)
            return
    mongla.turn(heading)
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
