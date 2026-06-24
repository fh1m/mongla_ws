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

# ── Bin ─────────────────────────────────────────────────────────────────────────
BIN_CENTRE_ERR_PX     = 30      # how tightly to centre over the bin before dropping

# ── Torpedo ──────────────────────────────────────────────────────────────────────
TORPEDO_BLOOD_FWD_FILL = 30     # blood bbox height % of frame at end of approach

# ── Search behaviour (mission-authored fallback functions) ──────────────────────
SEARCH_FORWARD_GAIN   = 40      # % gain for forward creep between detections
SEARCH_CREEP_S        = 0.6     # seconds of forward creep per fallback cycle
SEARCH_YAW_STEP_DEG   = 20      # yaw step per sweep look

# ── Style roll ────────────────────────────────────────────────────────────────
STYLE_ROLL_HEADROOM_M = 0.4     # depth below surface for ACRO roll clearance
STYLE_ROLL_GAIN       = 60      # roll speed gain
