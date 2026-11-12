# What the SROT link carries, measured by the manager's reader (the one thread
# that reads the board) over the last few seconds (link_meter.WINDOW_S).
# Absent values are NaN, never 0.
std_msgs/Header header
float32 bytes_per_s            # every frame's bytes, as received
float32 capacity_bytes_per_s   # what the line can carry: 115200 baud, 8N1
float32 board_drop_pct         # the board's own view, SYS_STATUS.drop_rate_comm
float32 board_errors           # SYS_STATUS.errors_comm
MessageRate[] rates            # frames per second, per MAVLink message type
NamedValue[] named             # every NAMED_VALUE_FLOAT channel the board sends
