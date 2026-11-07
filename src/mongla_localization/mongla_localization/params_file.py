"""The node half of `~/.mongla/bringup.yaml`: a typo in `nodes:` stops the node.

The deck has ONE settings file. Its top level holds launch arguments; its
`nodes:` key holds node parameters, by node name:

    vision: true                    # a launch argument
    nodes:
      mongla_manager:
        vision.kp_yaw: 80.0         # a parameter of the manager node

Bringup (`mongla_localization/launch_defaults.py`) writes `nodes:` out as an
ordinary ROS parameter file and hands it to every node it starts. ROS then
does two silent things this module exists to make loud:

1. **A key nobody declared is ignored.** `kp_yaww: 80.0` would leave the gain
   at its default and the log would look exactly like a tuned run.
2. **A key the launch file also sets loses.** Launch passes its own values
   after the file, so `vision.mixer_aware: false` under `nodes:` would lose to
   the `mixer_aware` argument without a word. Those keys belong at the top
   level, under the argument's name.

So every node calls `refuse_unknown(node)` once it is built, and either case
stops it at launch with what to do instead. Only the file bringup applied is
checked (its path arrives in `APPLIED_ENV`); a node started any other way is
left alone.

It lives in `mongla_localization` because that is the one package every node
package already depends on: `mongla_vision` and `mongla_manager` both do.
"""
from __future__ import annotations

import difflib
import math
import os

APPLIED_ENV = 'MONGLA_NODES_APPLIED'


def flatten(params: dict, prefix: str = '') -> dict:
    """{'vision': {'kp_yaw': 80}} -> {'vision.kp_yaw': 80}, as ROS names them."""
    out = {}
    for k, v in params.items():
        name = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, name + '.'))
        else:
            out[name] = v
    return out


def section_for(body: dict, node_name: str) -> dict:
    """This node's `ros__parameters`, flattened. Matches `name` or `/name`."""
    for key, sec in (body or {}).items():
        if str(key).strip('/') == node_name.strip('/'):
            return flatten((sec or {}).get('ros__parameters') or {})
    return {}


def unknown(keys, declared) -> list[str]:
    """One line per key in `keys` that is not in `declared`, with the closest name."""
    out = []
    for k in sorted(set(keys) - set(declared)):
        close = difflib.get_close_matches(k, list(declared), n=1)
        out.append(f'{k!r}' + (f' (did you mean {close[0]!r}?)' if close else ''))
    return out


def _same(a, b) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        return math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-12) or \
            (math.isnan(float(a)) and math.isnan(float(b)))
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a == b


def overridden(wanted: dict, live: dict) -> list[str]:
    """Keys whose live value is not the file's: the launch file set them later."""
    return [f'{k!r} (file {wanted[k]!r}, running {live[k]!r})'
            for k in sorted(wanted) if k in live and not _same(wanted[k], live[k])]


def refuse_unknown(node) -> None:
    """Raise if `nodes:` names a parameter `node` lacks, or one launch overrode."""
    path = os.environ.get(APPLIED_ENV, '')
    if not path:
        return
    import yaml
    with open(path) as f:
        body = yaml.safe_load(f) or {}
    wanted = section_for(body, node.get_fully_qualified_name())
    if not wanted:
        return
    # `has_parameter` decides; the names are only for the "did you mean" hint
    # (Humble's rclpy has no public way to list them).
    missing = [k for k in wanted if not node.has_parameter(k)]
    bad = unknown(missing, getattr(node, '_parameters', {}).keys())
    if bad:
        raise RuntimeError(f'bringup.yaml nodes: {node.get_name()}: no parameter '
                           f'{", ".join(bad)} -- ROS would have ignored it and '
                           f'run on the default')
    live = {k: node.get_parameter(k).value for k in wanted}
    lost = overridden(wanted, live)
    if lost:
        raise RuntimeError(f'bringup.yaml nodes: {node.get_name()}: {", ".join(lost)} '
                           f'-- a launch argument sets this; set the argument at '
                           f'the top level of bringup.yaml instead')
