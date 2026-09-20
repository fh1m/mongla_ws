# Metric 6-DoF pose of a known-size planar target, from the anchor rung.
#
# NOT geometry_msgs/PoseWithCovarianceStamped, and the reason is load-bearing:
# a covariance asserts a UNIMODAL distribution -- "probably near the mean".
# Planar pose carries a FLIP AMBIGUITY, which is bimodal -- "one of two
# places", mirrored about zero. Encoding that as a covariance would
# misrepresent the single property that makes this safe to act on, so the two
# branches' disagreement rides explicitly in *_spread_deg instead.
#
# A consumer deciding whether to FIRE must read the spreads, never the point
# estimate alone. `square_within(tol)` in mongla_vision/anchor/pose.py is the
# reference implementation of that test.

std_msgs/Header header      # stamp = the CAPTURE instant of the frame matched

bool ok                     # false => every field below is meaningless
string reason               # why not, when ok is false

float32 yaw_deg             # + = board's normal points RIGHT of the optical axis
float32 pitch_deg           # + = normal points DOWN
float32 roll_deg            # in-plane
float32 range_m             # metric, from the target's known width

float32 yaw_spread_deg      # how far apart the two flip branches are
float32 pitch_spread_deg    #   -- the WIDTH of the answer, not its error bar

float32 off_axis_deg        # total angle off the board's normal (point estimate)
float32 reproj_px           # how well the pose explains the pixels it was fit to
float32 ambiguity           # best/second reprojection error; ->1.0 = indistinguishable
uint16 n_points             # RANSAC inliers the pose was fitted from
