"""Competition pool-day constants for RoboSub 2026.

Edit this file at the pool before a run — no colcon build needed (hot-reload applies).
None values = fill at pool after compass survey and depth sighting.
"""

# ── Depths ────────────────────────────────────────────────────────────────────
GATE_SEARCH_DEPTH_M   = -0.4    # initial descent depth for whole run
GATE_PASS_DEPTH_M     = -0.6    # depth to pass through gate opening
BIN_DEPTH_M           = -1.0    # downward camera must see bin clearly
TORPEDO_DEPTH_M       = None    # align with hole center; fill at pool

# ── Gate pass bbox threshold ───────────────────────────────────────────────────
GATE_PASS_BBOX_FRAC   = 0.80    # gate bbox height fraction that means "through gate"

# ── Headings (degrees) — fill after compass survey ─────────────────────────────
SLALOM_HEADING_DEG    = None    # compass heading to slalom course
BIN_HEADING_DEG       = None    # compass heading to bin
TORPEDO_HEADING_DEG   = None    # compass heading to torpedo board
RETURN_HEADING_DEG    = None    # compass heading back to gate

# ── Slalom ────────────────────────────────────────────────────────────────────
SLALOM_PIPE_OFFSET_PX = 80      # lateral pixel offset from pipe centre (positive=right)

# ── Search behaviour ──────────────────────────────────────────────────────────
SEARCH_FORWARD_GAIN   = 40      # % gain for forward creep between detections
SEARCH_MAX_STEPS      = 60      # max forward steps before fallback to look_around

# ── Style roll ────────────────────────────────────────────────────────────────
STYLE_ROLL_HEADROOM_M = 0.4     # depth below surface for ACRO roll clearance
STYLE_ROLL_GAIN       = 60      # roll speed gain
