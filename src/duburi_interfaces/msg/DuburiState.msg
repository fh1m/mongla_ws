# Periodic state snapshot published by auv_manager_node on /duburi/state.
#
# Replaces the old "JSON encoded inside std_msgs/String" pattern. Numeric
# fields use NaN (or empty string for `mode`) to signal "no data" instead
# of an out-of-band sentinel, so subscribers can do `math.isnan(s.depth_m)`
# and never have to guess.
std_msgs/Header header

bool   armed
string mode              # ArduSub mode name (MANUAL, ALT_HOLD, ...) or '' if unknown
float32 yaw_deg          # NaN when no AHRS sample
float32 depth_m          # negative below surface; NaN when no AHRS sample
float32 battery_voltage  # NaN when no BATTERY_STATUS sample
