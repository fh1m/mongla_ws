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
    'vision.kp_yaw':              60.0,
    'vision.kp_lat':              60.0,
    'vision.kp_depth':             0.05,
    'vision.kp_forward':         200.0,
    # Loose defaults tuned for sim + webcam smoke tests: a seated person
    # inside a reasonable "centred" zone satisfies `deadband`, fills
    # ~30% of frame height from a normal chair distance, and webcam
    # detection stutters up to 1.5 s without tripping the lost-target
    # budget. Pool missions override per-call via the CLI / mission DSL
    # or by editing vision_tunables.yaml for the run.
    'vision.deadband':             0.18,
    'vision.target_bbox_h_frac':   0.30,
    'vision.stale_after':          1.5,
    'vision.on_lost':             'fail',
    'vision.acquire_yaw_rate_pct': 22.0,
    'vision.acquire_gain':         25.0,
    # depth_anchor_frac: which vertical point on the bbox to align to centre.
    # 0.5 = centre (default, same as before). 0.2 = near-top (use for tall
    # objects like standing people -- prevents the controller stalling when
    # the bbox centre is already at the frame centre).
    'vision.depth_anchor_frac':    0.5,
    # lock_mode: 'settle' (exit when centred), 'follow' (track until duration),
    # 'pursue' (approach-only, exit when target fills target_bbox_h_frac).
    'vision.lock_mode':           'settle',
    # distance_metric: how to measure target distance from its bounding box.
    # 'height' (default), 'area' (better for wide targets), 'diagonal'.
    'vision.distance_metric':     'height',
}


# Per-command map: COMMANDS-field name -> ROS-param name.
#
# Some commands reuse the same conceptual gain (e.g. vision_acquire's
# `gain` is the search-thrust percent, sourced from
# `vision.acquire_gain`; vision_align_yaw's `kp_yaw` is sourced from
# `vision.kp_yaw`). Keep this table explicit -- never guess.
_FIELDS_PER_COMMAND: Dict[str, Dict[str, str]] = {
    'vision_align_3d': {
        'kp_yaw':              'vision.kp_yaw',
        'kp_lat':              'vision.kp_lat',
        'kp_depth':            'vision.kp_depth',
        'kp_forward':          'vision.kp_forward',
        'deadband':            'vision.deadband',
        'target_bbox_h_frac':  'vision.target_bbox_h_frac',
        'stale_after':         'vision.stale_after',
        'on_lost':             'vision.on_lost',
        'depth_anchor_frac':   'vision.depth_anchor_frac',
        'lock_mode':           'vision.lock_mode',
        'distance_metric':     'vision.distance_metric',
    },
    'vision_align_yaw': {
        'kp_yaw':      'vision.kp_yaw',
        'deadband':    'vision.deadband',
        'stale_after': 'vision.stale_after',
        'on_lost':     'vision.on_lost',
        'lock_mode':   'vision.lock_mode',
    },
    'vision_align_lat': {
        'kp_lat':      'vision.kp_lat',
        'deadband':    'vision.deadband',
        'stale_after': 'vision.stale_after',
        'on_lost':     'vision.on_lost',
        'lock_mode':   'vision.lock_mode',
    },
    'vision_align_depth': {
        'kp_depth':           'vision.kp_depth',
        'deadband':           'vision.deadband',
        'stale_after':        'vision.stale_after',
        'on_lost':            'vision.on_lost',
        'depth_anchor_frac':  'vision.depth_anchor_frac',
        'lock_mode':          'vision.lock_mode',
    },
    'vision_hold_distance': {
        'kp_forward':         'vision.kp_forward',
        'target_bbox_h_frac': 'vision.target_bbox_h_frac',
        'deadband':           'vision.deadband',
        'stale_after':        'vision.stale_after',
        'on_lost':            'vision.on_lost',
        'lock_mode':          'vision.lock_mode',
        'distance_metric':    'vision.distance_metric',
    },
    'vision_acquire': {
        'gain':         'vision.acquire_gain',
        'yaw_rate_pct': 'vision.acquire_yaw_rate_pct',
        'stale_after':  'vision.stale_after',
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
