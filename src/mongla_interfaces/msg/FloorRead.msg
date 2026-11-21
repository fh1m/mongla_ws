# What the downward camera read off the floor in one frame, as image geometry
# in that frame's own full-size pixels, for an operator to see it drawn over
# the picture (mongla_vision/flow/floor_read.py). The heading and height these
# readings give are published elsewhere (floor_grid_deg, lane_heading_deg,
# floor_height); this is where on the picture they came from. Published only
# while someone subscribes, at the floor read's rate.
#
# The stamp is the camera frame the reading was taken on.
std_msgs/Header header
uint32 width
uint32 height

bool tiles                     # the tile grating answered on this frame
float32 period_px              # the grout spacing
float32[] grout_normal_deg     # one per family of grout lines (two)
float32[] grout_offset_px      # n . p of one line of that family, p from the top-left pixel
float32 strength               # the grating's spectral peak share
float32 tile_height_m          # f * tile_m / period_px; NaN when no tile size is set

bool lane                      # a lane line was found on this frame
float32 lane_direction_deg     # its direction in the image, [0, 180)
float32 lane_offset_px         # its centreline's signed distance from the frame centre
float32 lane_elongation        # length over width of the dark band
