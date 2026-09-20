# 2D-3D correspondences: the EVIDENCE a pose is solved from, before it is
# solved. One message, any evidence source, one solver.
#
# WHY THIS EXISTS AS A MESSAGE. A 6-DoF pose is a lossy summary of the points
# it was fitted to. Publish only the pose and a shot that missed can never be
# explained: you cannot tell a bad solve from bad evidence, and you cannot try
# a different gate. Publish the correspondences and a recorded run is
# RE-SOLVABLE off a bag, months later, with different thresholds -- which is
# the only way several of this repo's measured numbers were ever settled.
#
# It also decouples evidence from geometry. The anchor's RANSAC inliers, a
# board's published opening centres, a segmentation contour and a set of
# hand-clicked corners are four different ways to answer the same question --
# "where on the target is this pixel?" -- and every one of them lands here and
# gets the identical IPPE-interval / SQPnP-point-estimate treatment in
# mongla_vision/anchor/pose.py::solve_pnp.

std_msgs/Header header      # stamp = the CAPTURE instant of the frame matched

string camera               # whose camera_info gives K, e.g. 'forward'
string target_label         # the class/target these points lie on
string source               # what produced them, e.g. 'anchor' -- for a bag

# ⛔ WHICH OPTICS THE PIXELS ARE IN. Underwater a flat port is not a pinhole,
# so `image_points` and the K they are solved against must BOTH be rectified
# or NEITHER. Rectifying one side compares water geometry against air geometry
# and puts the whole error into the pose, silently -- so the publisher states
# what it sent and the solver never has to guess. There is no default: an
# unset value is a refusal, not an assumption.
uint8 OPTICS_UNSET=0
uint8 OPTICS_RAW=1          # straight off the camera; the solver rectifies
uint8 OPTICS_RECTIFIED=2    # publisher already applied RefractiveRectifier
uint8 optics

# ⛔ WHICH PIXEL FRAME `image_points` ARE IN. A producer may work at a reduced
# resolution -- the anchor matches in its backend's 320x240, not the camera's
# full frame -- and CameraInfo describes the FULL frame. Solving reduced pixels
# against a full-resolution K scales every recovered angle and range, silently,
# with a plausible number and no error. So the producer states its own frame
# and the solver scales K to match. Zero means "same as CameraInfo".
uint32 image_width
uint32 image_height

# Points on the TARGET's own frame, in METRES, flat: [x0,y0,z0, x1,y1,z1, ...].
# Metres and target-frame together are what make the answer metric and what let
# a caller use a published layout instead of a snapped patch. A planar target
# has z = 0 throughout and is the case that carries a flip ambiguity.
float64[] object_points

# The matching pixels, flat: [u0,v0, u1,v1, ...], in `camera`'s frame.
# len(image_points)/2 MUST equal len(object_points)/3, and both at least 4.
float64[] image_points
