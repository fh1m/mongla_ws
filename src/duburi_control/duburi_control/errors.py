"""Movement-layer exceptions.

Raised by the per-axis modules (motion_yaw, motion_forward, motion_lateral, motion_depth)
and by `Duburi` itself to fail loudly to the action server instead of
silently returning. The ActionServer catches these in its execute
callback, sets `Move.Result.success = False`, and aborts the goal so
callers (CLI, mission scripts) see the failure.

Three flavours, narrow on purpose:

  MovementError    -- Base class. Catch this if you want all
                      movement-layer failures (timeouts, mode
                      rejections, bad inputs).
  MovementTimeout  -- Yaw or depth loop reached its deadline without
                      locking inside tolerance. Includes target /
                      current values in the string for log triage.
  ModeChangeError  -- Autopilot refused to enter a mode we needed
                      (e.g. ALT_HOLD for yaw or set_depth). NOT
                      swallowed -- the move cannot succeed, so the
                      action result has to reflect it.
"""


class MovementError(Exception):
    """Base class for any movement-layer failure surfaced to the operator."""


class MovementTimeout(MovementError):
    """Yaw or depth loop reached its deadline without locking on target."""


class ModeChangeError(MovementError):
    """Autopilot refused the requested mode within the timeout."""


class NotArmedError(MovementError):
    """Command rejected because the AUV is not armed.

    ArduSub silently drops every RC_CHANNELS_OVERRIDE and
    SET_POSITION_TARGET_GLOBAL_INT frame while disarmed, so motion
    commands would appear to succeed but produce no thrust.
    Call arm() first.
    """
