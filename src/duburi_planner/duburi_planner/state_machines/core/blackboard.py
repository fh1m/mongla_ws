"""Typed blackboard key constants.

Use BK.* everywhere — prevents key typos and makes it grep-able.
"""


class BK:
    START_HEADING   = 'start_heading'    # float deg — set by ArmState
    GATE_HEADING    = 'gate_heading'     # float deg — passed via params
    LAST_ERROR      = 'last_error'       # str — set on unhandled exception
    MISSION_START_T = 'mission_start_t'  # float monotonic — set by CountdownState
    DVL_CONNECTED   = 'dvl_connected'    # bool — set by ArmState after dvl_connect
