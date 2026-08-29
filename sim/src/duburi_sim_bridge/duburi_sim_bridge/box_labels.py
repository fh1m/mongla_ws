#!/usr/bin/env python3

"""YOLO labels from Gazebo's bounding-box camera.

This replaced `gt_labels.py`, which projected prop AABBs itself and admitted in
its own docstring that it did "not a full Gazebo visibility check". The
consequences were real: a crate hidden behind another crate still got a full
box, a prop behind the vehicle could be labelled, and every prop's half-extents
had to be hand-maintained in a table that could -- and once did -- fall out of
step with the class list.

Gazebo already renders the scene. Asking it for the boxes gives occlusion and
truncation for free, on the GPU, and deletes three hand-maintained tables.

Two properties worth stating because they are what changed:

  * RUNTIME SPAWNS ARE LABELLED. The projector read the course YAML, so a prop
    added mid-run with `props add` was invisible to it. The sensor sees the
    scene, so anything with a label is annotated however it got there.
  * THE CLASS LIST LIVES WITH THE PROPS. `prop_library.DETECTION_CLASSES` is
    both the YOLO class index and the Gazebo semantic label, so there is no
    second mapping to disagree with the first.
"""

from __future__ import annotations

import importlib.util
import os
import sys

from ament_index_python.packages import get_package_share_directory


def _prop_library():
    """Import prop_library from duburi_sim_worlds' share directory.

    That package is ament_cmake, so its scripts install to share/ rather than
    onto the Python path. Same loader duburi_sim_scenarios.prop_catalog uses.
    """
    scripts = os.path.join(
        get_package_share_directory('duburi_sim_worlds'), 'scripts')
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location(
        'prop_library', os.path.join(scripts, 'prop_library.py'))
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault('prop_library', mod)
    spec.loader.exec_module(mod)
    return mod


CLASSES = _prop_library().DETECTION_CLASSES

# Below this many pixels a box is noise -- a prop at the far wall, or one
# clipped to a sliver at the frame edge. Training on those teaches a detector
# to fire on nothing. Matches the threshold the projector used.
MIN_BOX_PX = 2
MIN_BOX_FRAC = 0.0005


def boxes_to_yolo(msg, width: int, height: int) -> list:
    """`vision_msgs/Detection2DArray` -> YOLO rows, normalised to the frame.

    Gazebo gives centre and size in PIXELS; YOLO wants both normalised. The
    class id arrives as a STRING in `hypothesis.class_id` because that field is
    a string in the ROS message -- it is our integer label, stringified by the
    bridge, and it must be parsed back rather than hashed.
    """
    rows = []
    for det in getattr(msg, 'detections', []):
        if not det.results:
            continue
        try:
            cid = int(det.results[0].hypothesis.class_id)
        except (TypeError, ValueError):
            continue
        if cid <= 0 or cid >= len(CLASSES):
            continue        # 0 is reserved background; out of range is stale
        bw, bh = float(det.bbox.size_x), float(det.bbox.size_y)
        if bw < MIN_BOX_PX or bh < MIN_BOX_PX:
            continue
        xc = float(det.bbox.center.position.x) / width
        yc = float(det.bbox.center.position.y) / height
        nw, nh = bw / width, bh / height
        if nw * nh < MIN_BOX_FRAC:
            continue
        # Gazebo can report a centre slightly outside the frame for a box that
        # is mostly off-screen; clamp rather than drop, since the visible part
        # is still a true positive.
        xc, yc = min(max(xc, 0.0), 1.0), min(max(yc, 0.0), 1.0)
        rows.append((cid, xc, yc, min(nw, 1.0), min(nh, 1.0)))
    return rows


def write_classes_file(path) -> None:
    """Write classes.txt so a recorded run is self-describing."""
    path.write_text('\n'.join(CLASSES) + '\n')


def format_yolo_line(row) -> str:
    cid, xc, yc, w, h = row
    return f'{cid} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}'
