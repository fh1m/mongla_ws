# The point pairs one ripe flow interval measured: each anchor point and
# where it was found in the frame the interval ends on, both in that frame's
# own pixels. Only the pairs that survived tracking and forward-backward
# rejection are here. A velocity, when the interval is not refused (no height,
# no gyro, too little flow), is computed from these.
#
# The stamp is the camera frame the interval ends on, so a viewer draws the
# pairs over the picture they were measured in.
std_msgs/Header header
float32[] anchor_xy   # x0, y0, x1, y1, ...
float32[] now_xy      # the same points, found in this frame
