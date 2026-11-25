# The three safety states the board decodes, each tri-state, on /mongla/safety.
#
# Their own message, not fields on MonglaState: ROS 2 replays a bag against
# today's definition, so a field added there leaves every bag recorded before
# it undecodable (test_recorded_messages_keep_their_shape.py). Published with
# each /mongla/state, under the same header.
#
# UNKNOWN is 0 so that a message built without them reads unknown, never dry
# or clear.
std_msgs/Header header

int8 SAFETY_UNKNOWN = 0
int8 SAFETY_CLEAR = 1
int8 SAFETY_TRIPPED = 2

int8 leak                # TRIPPED = water inside; UNKNOWN until the board says
int8 kill_switch         # TRIPPED = killed; UNKNOWN with no second-board link (the firmware then reports 0)
int8 baro                # CLEAR = fault code 0; TRIPPED = any other code; UNKNOWN = not reported
string baro_text         # the board's fault code as words (srot_protocol.baro_health_text), '' when unknown
