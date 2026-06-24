#!/usr/bin/env python3
"""vision_tunables -- ROS-param defaults for every vision_* command.

Layered defaults (resolved per goal in `commands.fields_for`):
    1. value the goal supplied                  (per-call wins)
    2. live ROS-param value                     (this module)
    3. hardcoded spec default in COMMANDS       (last resort)

The point of layer 2 is pool-day tuning: an operator on the deck can
type `ros2 param set /duburi_manager vision.kp_yaw 80.0` and the next
vision_* goal picks up the new value -- without anyone touching Python
or restarting the manager.

Param names use `vision.` prefix so they namespace cleanly under the
manager node and read well in `ros2 param list`. The translation from
ROS-param key (`vision.kp_yaw`) to COMMANDS field name (`kp_yaw`) is
spelled out in `_FIELD_FROM_PARAM` below.

Live mid-loop tuning is intentionally NOT supported in v1: param
changes take effect on the NEXT vision goal so we never hand a moving
control loop a discontinuous gain. Add a per-tick re-snapshot only when
pool data demands it.
"""

from __future__ import annotations

from typing import Any, Dict


# ---------------------------------------------------------------------- #
#  Single source of truth: ROS-param name -> default value               #
# ---------------------------------------------------------------------- #
# Mirrors the spec defaults in duburi_control/commands.py exactly so
# nothing changes if the operator never sets a param.
VISION_PARAM_DEFAULTS: Dict[str, Any] = {
    # P-gains: pct thrust per unit normalized pixel error (kp_depth is m/tick).
    # Mirror the engine defaults in duburi_control/motion_vision.py.
    'vision.kp_lat':              60.0,
    'vision.kp_yaw':              60.0,
    'vision.kp_depth':             0.05,
    'vision.kp_forward':         200.0,
    # lost_grace_s: seconds the server coasts on target loss before reporting
    # LOST (so the DSL can run a fallback search). 1.0 s rides typical pool
    # turbidity blackouts without false-triggering.
    'vision.lost_grace_s':         1.0,
    # frame_fill_default: % of frame the bbox must fill for vision_move to count
    # as "reached" when the mission leaves fwd_fill at 0.
    'vision.frame_fill_default':  95.0,
    # align_stable_frames: ticks (at 20 Hz) every active axis must stay within
    # err_px before vision_align reports ALIGNED. 3 ticks = 0.15 s.
    'vision.align_stable_frames':  3.0,
}


# Per-command map: COMMANDS-field name -> ROS-param name.
#
# These let a deck operator tune gains / grace live with `ros2 param set
# /duburi_manager vision.kp_yaw 80.0` and have the NEXT vision goal pick
# up the value -- without editing mission code. The fields here are NOT
# mission-facing (the DSL leaves them at the rosidl zero), so the
# substitution in `commands.fields_for` is what fills them.
_FIELDS_PER_COMMAND: Dict[str, Dict[str, str]] = {
    'vision_align': {
        'kp_lat':              'vision.kp_lat',
        'kp_yaw':              'vision.kp_yaw',
        'kp_depth':            'vision.kp_depth',
        'lost_grace_s':        'vision.lost_grace_s',
        'align_stable_frames': 'vision.align_stable_frames',
    },
    'vision_move': {
        'kp_forward':   'vision.kp_forward',
        'kp_lat':       'vision.kp_lat',
        'lost_grace_s': 'vision.lost_grace_s',
        'fwd_fill':     'vision.frame_fill_default',
    },
}


# ---------------------------------------------------------------------- #
#  Public API used by auv_manager_node                                   #
# ---------------------------------------------------------------------- #

def declare_vision_params(node) -> None:
    """Declare every `vision.*` parameter on `node` with its default.

    Idempotent: re-declaration is a no-op (rclpy raises only the second
    time, and we let it because that means someone wired this twice).
    """
    for name, default in VISION_PARAM_DEFAULTS.items():
        node.declare_parameter(name, default)


def snapshot_from_node(node) -> Dict[str, Any]:
    """Read every `vision.*` ROS-param value into a plain dict.

    Returns the snapshot the manager hands to `runtime_defaults_for_command`.
    Cheap enough to call every time a new goal arrives, which is exactly
    what we do so freshly-set params land on the next goal.
    """
    snapshot: Dict[str, Any] = {}
    for name in VISION_PARAM_DEFAULTS:
        snapshot[name] = node.get_parameter(name).value
    return snapshot


def runtime_defaults_for_command(cmd: str,
                                 snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Build the {COMMANDS-field-name: live-value} map for `cmd`.

    Returns an empty dict for non-vision commands so the caller can hand
    the result straight to `fields_for(cmd, request, runtime_defaults=...)`
    unconditionally.
    """
    field_to_param = _FIELDS_PER_COMMAND.get(cmd)
    if not field_to_param:
        return {}
    out: Dict[str, Any] = {}
    for field, param_name in field_to_param.items():
        if param_name in snapshot:
            out[field] = snapshot[param_name]
    return out
