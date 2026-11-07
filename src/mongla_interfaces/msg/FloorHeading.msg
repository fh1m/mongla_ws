# A heading read off the pool floor: the tile grid (modulo 90 deg) or a lane
# line (modulo 180 deg). The topic says which.
#
# The stamp is the camera frame it was read from, not the time it was sent.
# Localization replays the yaw bound at that instant (BUGS B-91); a bare angle
# would be compared with the heading at arrival, which during a turn is
# somewhere else.
std_msgs/Header header
float32 heading_deg
