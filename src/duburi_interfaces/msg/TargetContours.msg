# The OUTLINE of each detection, in image pixels. One message for a box and a
# mask alike.
#
# ⛔ WHY A CONTOUR AND NOT A BITMAP, AND WHY IT IS THE SAME MESSAGE FOR BOTH.
# This is the format the 2025/2026 RoboSub champions ship: their
# `DetectedObject2D` carries `uint16[] contour` with the note "2D bounding box
# OR contour surrounding the object", plus area, centre and bbox size marked
# "can be derived from contour, but added for convenience", plus an OBB angle.
# They have no mask message at all.
#
# The structural consequence is the reason to copy it: a BOX IS A FOUR-POINT
# CONTOUR and a mask is an N-point one, so a detection model and a segmentation
# model publish the SAME message and every consumer downstream has exactly one
# code path. A mission that switches from a detector to a seg model -- or runs
# both -- changes what arrives here in resolution, not in kind.
#
# A bitmap would have been the obvious choice and is the wrong one. The
# consumer that wants a mask wants it to (a) fit a pose to an outline and
# (b) test whether a symbol mask intersects a structure's rim. Both are
# polygon operations. A 640x480 bitmap per detection is ~300 kB of mostly
# zeros on a link that also carries two camera streams, and every consumer
# would begin by extracting the contour anyway.
#
# Pixels, not metres, and no pose: this is EVIDENCE, the same layer as
# TargetCorrespondences. The solver turns points into geometry; this message
# only says where they are.

std_msgs/Header header      # stamp = the CAPTURE instant of the frame

string camera               # e.g. 'forward'

# ⛔ THE PIXEL FRAME THESE POINTS ARE IN, for the same reason
# TargetCorrespondences carries it: a producer may work at a reduced
# resolution, and scaling a contour by the wrong factor yields a plausible
# polygon in the wrong place. Zero means "same as the matching CameraInfo".
uint32 image_width
uint32 image_height

# One entry per contour. `class_name` is the identity that survives a model
# switch -- an integer class index is a property of whichever model minted it,
# so it is deliberately absent here, exactly as in the detection path.
string[] class_name
float32[] score

# Orientation of the minimum-area rectangle, degrees, 0..180. -1 means the
# producer did not compute one. A box-only producer leaves it -1 rather than
# publishing 0, because 0 is a real angle.
int16[] angle_deg

# Filled area in pixels. Derivable from `points`, carried because every
# consumer wants it and recomputing a polygon area per detection per frame is
# the kind of cost that is invisible until it is not.
uint32[] area_px

# ⛔ RAGGED ARRAYS, FLATTENED. Contour i occupies
# points[2*offset[i] : 2*offset[i+1]], as x,y pairs. `offset` therefore has
# ONE MORE entry than `class_name`: the final element is the total point
# count, so the last contour needs no special case. A consumer that assumes
# len(offset) == len(class_name) reads the last contour as empty.
uint32[] offset
uint16[] points
