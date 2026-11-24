# Periodic state snapshot published by auv_manager_node on /mongla/state.
#
# Replaces the old "JSON encoded inside std_msgs/String" pattern. Numeric
# fields use NaN (or empty string for `mode`) to signal "no data" instead
# of an out-of-band sentinel, so subscribers can do `math.isnan(s.depth_m)`
# and never have to guess.
std_msgs/Header header

bool   armed
string mode              # flight mode name -- STABILIZE/ACRO/DEPTH_HOLD/SURFACE/MANUAL/AUTO (+ tuning modes), or '' if unknown
float32 yaw_deg          # NaN when no AHRS sample
float32 depth_m          # negative below surface; NaN when no AHRS sample
float32 battery_voltage  # NaN when no BATTERY_STATUS sample

# The three safety states the board decodes, each tri-state. UNKNOWN is 0 so
# that a message built without them reads unknown, never dry or clear.
int8 SAFETY_UNKNOWN = 0
int8 SAFETY_CLEAR = 1
int8 SAFETY_TRIPPED = 2
int8 leak                # TRIPPED = water inside; UNKNOWN until the board says
int8 kill_switch         # TRIPPED = killed; UNKNOWN with no second-board link (the firmware then reports 0)
int8 baro                # CLEAR = fault code 0; TRIPPED = any other code; UNKNOWN = not reported
string baro_text         # the board's fault code as words (srot_protocol.baro_health_text), '' when unknown
