# Is localization working, and where does it put the hull? One message,
# published by mongla_localization at 10 Hz whether or not anything feeds it,
# so a running node with no inputs reads as exactly that.
#
# Read by the manager (the `locate` and `go_to` verbs), the DSL
# (`mongla.localization()`), `bringup_check` and Batighor. `usable` is the
# single answer; `reason` says why not. Absence is NaN, never 0.0.

std_msgs/Header header   # stamp: the newest IMU sample (board clock); 0 before any
string  frame            # 'pool' once anchored, 'odom_ned' before, '' before any input
bool    anchored         # the heading has been made absolute from a prop
float32 x_m              # position in `frame`, NED; NaN before any input
float32 y_m
float32 depth_m          # negative below the surface, as everywhere else in Mongla
float32 yaw_deg          # in `frame`
float32 sigma_xy_m       # 1-sigma of the worse horizontal axis, attitude terms included
float32 sigma_yaw_deg
string  aiding           # 'aided' | 'aided (flow uncalibrated)' | 'unaided' | ''
string  motion           # 'ok' | 'blocked' | 'unknown' | '' (none yet)
float32 imu_age_s        # host seconds since the last IMU sample fused; NaN = never
float32 depth_age_s      # ... depth sample fused
float32 flow_age_s       # ... flow velocity fused
uint32  flow_fused       # running counts, so a rate of zero is visible
uint32  flow_refused
uint32  fixes            # prop position fixes fused
bool    usable           # IMU live, heading anchored, position aided
string  reason           # why not usable; '' when usable
