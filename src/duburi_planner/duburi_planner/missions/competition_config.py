"""Competition pool-day constants for RoboSub 2026.

Edit this file at the pool before a run — no colcon build needed (hot-reload applies).
None values = fill at pool after compass survey and depth sighting.

All vision tuning here is in the two-verb pixel-native vocabulary:
  * ``*_ERR_PX``   -- pixel tolerance for vision.align (aligned when |err| <= this)
  * ``*_FWD_FILL`` -- % of frame the bbox must fill for vision.move to stop
  * ``*_GAIN``     -- hard max-speed cap (% thrust) for that phase
  * ``*_OFFSET_PX``-- signed lateral pixel offset to hold
"""

# ── Depths ────────────────────────────────────────────────────────────────────
GATE_SEARCH_DEPTH_M   = -0.4    # initial descent depth for whole run
GATE_PASS_DEPTH_M     = -0.6    # depth to pass through gate opening
BIN_DEPTH_M           = -1.0    # downward camera must see bin clearly
TORPEDO_DEPTH_M       = None    # align with hole center; fill at pool

# ── Headings (degrees) — fill after compass survey ─────────────────────────────
SLALOM_HEADING_DEG    = None    # compass heading to slalom course
BIN_HEADING_DEG       = None    # compass heading to bin
TORPEDO_HEADING_DEG   = None    # compass heading to torpedo board
RETURN_HEADING_DEG    = None    # compass heading back to gate

# ── Alignment tolerances (pixels) ──────────────────────────────────────────────
ALIGN_ERR_PX          = 40      # default "centred" tolerance
FINE_ERR_PX           = 14      # tight tolerance for the torpedo hole / fire lock
ALIGN_GAIN            = 30      # max speed during centring
APPROACH_GAIN         = 35      # max speed while driving forward

# ── Gate ───────────────────────────────────────────────────────────────────────
GATE_PASS_FWD_FILL    = 80      # gate bbox height % of frame that means "through gate"

# ── Slalom ────────────────────────────────────────────────────────────────────
SLALOM_PIPE_OFFSET_PX = 80      # lateral pixel offset from pipe centre (positive=right)
SLALOM_FWD_FILL       = 60      # pipe bbox height % of frame at closest pass

# ── Bin (DOWNWARD camera: image-X→lat Ch6, image-Y→surge Ch5, fill→depth) ────────
BIN_CENTRE_ERR_PX     = 30      # how tightly to centre over the bin before dropping
BIN_SURGE_SIGN        = +1      # NOTE: no longer read by task_bin -- the downward Ch5
                                # fore/aft polarity is now the PERMANENT `vision.surge_sign`
                                # deck default (-1) in duburi_manager/vision_tunables.py, so
                                # missions omit surge_sign entirely. Kept here for reference;
                                # to change the mount polarity edit vision.surge_sign (or set
                                # it live: `ros2 param set /duburi_manager vision.surge_sign -1`).
                                # VERIFY DISARMED: vision_thrust_check --camera downward.
BIN_DESCEND_FILL      = 0       # optional: descend until the bin fills this % (height) for a
                                # closer drop. 0 = OFF (hold BIN_DEPTH_M; core path). When >0,
                                # bounded by BIN_MAX_DEPTH_M so it can't drive into the floor.
BIN_MAX_DEPTH_M       = -1.6    # deepest allowed setpoint for BIN_DESCEND_FILL (negative m)
BIN_DEPTH_CEILING_M   = -0.4    # SHALLOWEST setpoint on the downward align -- surface guard so
                                # ratio/alignment can't lift the hull out of the water (negative m).
BIN_DROPPER_CHANNEL   = 3       # 3 = dropper_1, 4 = dropper_2 (marker drop)

# ── Payload fire ─────────────────────────────────────────────────────────────────
FIRE_GAP_S            = 1.0     # seconds BETWEEN channels when firing a LIST (fire=[1,4]/[2,3]).
                                # The solenoid launcher misfires if two go together -- space them.

# ── Torpedo ──────────────────────────────────────────────────────────────────────
TORPEDO_BLOOD_FWD_FILL = 30     # blood bbox height % at end of COARSE approach (gets in range)
# Firing STANDOFF: the terminal align('hole', fwd=…) drives forward to this hole-height
# fill and HOLDS it while firing. RoboSub awards bonus points for firing FURTHER from the
# board (far 0.3m / farther 0.46m) AND a large+stable bbox at standoff is far easier to
# hold steady than point-blank -- so tune this so the hull parks ~0.3-0.46m off the board
# (smaller fill = further back). Read the live `[ align … fwd>=…% ]` line to calibrate.
TORPEDO_STANDOFF_FILL  = 35     # hole bbox height % of frame at the firing standoff
TORPEDO_STANDOFF_HOLD_S = 4.0   # seconds to station-keep (forward+lat+depth) while firing
TORPEDO_FIRE_T         = 1.0    # seconds into the hold to fire (must be < STANDOFF_HOLD_S;
                                # leaves ~3 s for a CH340-reconnect-delayed shot to still land)

# ── Search behaviour (mission-authored fallback functions) ──────────────────────
SEARCH_FORWARD_GAIN   = 40      # % gain for forward creep between detections
SEARCH_CREEP_S        = 0.6     # seconds of forward creep per fallback cycle
SEARCH_YAW_STEP_DEG   = 20      # yaw step per sweep look

# ── Style roll ────────────────────────────────────────────────────────────────
STYLE_ROLL_HEADROOM_M = 0.4     # depth below surface for ACRO roll clearance
STYLE_ROLL_GAIN       = 60      # roll speed gain
