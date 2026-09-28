#!/usr/bin/env python3
"""Node parameters that NO launch file passes.

⛔ THE THIRD FACE OF THE SAME DEFECT. `orphan_sweep.py` finds modules nothing
imports. `topic_wiring_sweep.py` finds topics nothing reads. This finds the
one they both miss: a parameter a node declares, with a sensible default and
working code behind it, that **no launch argument reaches** -- so the only way
to change it is `ros2 param set` after the node is already running.

⭐ FOUND ON ITS FIRST RUN, in work from the same session that wrote it:
`lock_node` declares `act_conf` -- the B-59 acting bar, the thing that stops
the ladder locking onto open water -- and `vision_pi.launch.py` passed six
parameters, none of them that one. The bar a pool day would want to raise was
the bar a pool day could not reach. `anchor_xfeat_hef` and
`anchor_semi_dense` were in the same state.

⚠ WHAT IS *NOT* A DEFECT, so the output is not over-read:

  * a parameter that is deliberately runtime-only. Some exist to be tuned live
    (`ros2 param set /mongla_manager vision.<name>`) and were never meant to
    have a launch argument -- ROADMAP section 9 lists those on purpose.
  * a parameter whose default is the only sane value and which exists to be
    read, not set.
  * `camera`, `use_sim_time` and similar, set by other means.

So: a question list again. For each row, say out loud whether an operator
should be able to set it BEFORE the node starts. If yes, it needs a launch
argument; if no, say so where the parameter is declared.

    python3 tools/launch_param_sweep.py
    python3 tools/launch_param_sweep.py --node lock_node

⛔ TRIAGED 2026-09-28, AND DELIBERATELY NOT MADE A TEST. The other three
reachability sweeps now fail the build -- modules, functions and topics each
have a guard with a named-exemption list. This one does not, and that is a
decision rather than an omission.

102 of the declared parameters have no launch argument, and the great majority
are legitimately runtime-only: `gyro_gain_x`, `ransac_px`, `fb_reject_px`,
`max_dispersion_ratio` and their kind are tuned live with `ros2 param set`
against a running graph, which is the whole point of them. Turning that into a
test means writing a 102-row exemption registry that nobody will read and that
will rot -- the failure mode this repo keeps naming. A question list that a
human runs occasionally is the right instrument here.

⚠ ONE ASYMMETRY FOUND AND LEFT ALONE, so it is not re-discovered as a defect:
`allow_saturated_depth_arm` has no launch argument while its sibling
`allow_fw_behaviour_mismatch` does. Both are arming overrides. It is NOT
broken -- the value is read once at construction and
`--ros-args -p allow_saturated_depth_arm:=true` reaches it fine -- and for an
override that accepts "full uncommanded heave", friction is arguably the
correct default. Recorded because the asymmetry looks like an oversight and is
not.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys
from collections import defaultdict

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'

_DECL = re.compile(r"declare_parameter\(\s*['\"]([a-zA-Z0-9_.]+)['\"]")
# `'name': LaunchConfiguration(...)` or `'name': True` inside a Node(...)
_PASS = re.compile(r"['\"]([a-zA-Z0-9_.]+)['\"]\s*:\s*(?:LaunchConfiguration|"
                   r"ParameterValue|True|False|[0-9'\"])")

# Set by ROS itself or by the node's own plumbing, never by a launch argument.
_EXEMPT = {'use_sim_time', 'camera', 'qos_overrides'}


def declared():
    out = defaultdict(set)
    for p in SRC.rglob('*.py'):
        if '/test' in str(p) or '/launch/' in str(p):
            continue
        try:
            txt = p.read_text()
        except (OSError, UnicodeDecodeError):
            continue
        names = set(_DECL.findall(txt))
        if names:
            out[p.stem] |= names
    return out


def passed():
    out = set()
    for p in SRC.rglob('launch/*.py'):
        try:
            out |= set(_PASS.findall(p.read_text()))
        except (OSError, UnicodeDecodeError):
            continue
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--node', default='')
    a = ap.parse_args()

    decl, sent = declared(), passed()
    total = 0
    for node in sorted(decl):
        if a.node and a.node not in node:
            continue
        missing = sorted(n for n in decl[node]
                         if n not in sent and n not in _EXEMPT)
        if not missing:
            continue
        total += len(missing)
        print(f'\n{node}: {len(missing)} declared, never passed by a launch '
              f'file')
        for n in missing:
            print(f'   {n}')

    if not total:
        print('every declared parameter is reachable from a launch file')
        return 0
    print(f'\n{total} parameter(s) with no launch argument.\n'
          '⚠ Not a defect list. Some are deliberately runtime-only (ROADMAP '
          'section 9) and some exist to be read rather than set. For each, say '
          'out loud whether an operator should be able to set it BEFORE the '
          'node starts -- if yes it needs a launch argument, if no say so '
          'where it is declared. `act_conf` was found exactly that way.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
