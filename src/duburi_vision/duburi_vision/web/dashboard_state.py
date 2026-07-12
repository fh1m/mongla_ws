"""Pure (rclpy-free) helpers for the mission-web console.

Kept separate from the node so the snapshot assembly, the pixel-offset
math, the parameter-type mapping and the camera-exclusivity list are all
unit-testable without spinning ROS. The node builds a plain ``store`` dict
(under its lock) and hands it here; nothing in this module imports rclpy.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

# Detector params and the ROS parameter type each must be sent as. Mapping by
# NAME (not python type) matters: a browser JSON `conf: 1` arrives as int but the
# detector's `conf` is a DOUBLE param -- sending it as INTEGER is rejected. Same
# for the reverse. Anything not listed falls back to python-type inference.
_PARAM_TYPES: Dict[str, str] = {
    'conf':         'double',
    'model_conf':   'string',
    'active_model': 'string',
    'classes':      'string',
    'max_det':      'integer',
    'paused':       'bool',
}


def param_value_for(name: str, value: Any) -> Tuple[str, Any]:
    """Resolve (ros_type, coerced_value) for a detector SetParameters write.

    ``ros_type`` is one of 'double' | 'integer' | 'bool' | 'string'. The node
    maps it to an rcl_interfaces ParameterValue. Coercion is explicit so a JSON
    number/str lands on the right ArduSub-side type.

    Raises ``ValueError`` on a value that cannot be coerced to the target type
    (e.g. ``conf=null`` or ``conf={...}``) so the caller returns a clean
    ok:false instead of the handler thread dying -- pool day cannot afford a
    silent-drop or a wedged request.
    """
    ros_type = _PARAM_TYPES.get(name)
    if ros_type is None:
        # Unknown param: infer from the python type (bool BEFORE int -- bool is
        # a subclass of int, so `isinstance(True, int)` is True).
        if isinstance(value, bool):
            ros_type = 'bool'
        elif isinstance(value, int):
            ros_type = 'integer'
        elif isinstance(value, float):
            ros_type = 'double'
        else:
            ros_type = 'string'

    # Reject containers/None outright for numeric/bool targets -- float(None),
    # int({...}) etc. would raise deep in the call; do it here with a clear msg.
    if ros_type in ('double', 'integer', 'bool') and (
            value is None or isinstance(value, (list, dict))):
        raise ValueError(f'{name}: cannot set {ros_type} from {value!r}')

    try:
        if ros_type == 'double':
            return ros_type, float(value)
        if ros_type == 'integer':
            return ros_type, int(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name}: {value!r} is not a valid {ros_type}')
    if ros_type == 'bool':
        # Accept JSON true/false and the strings "true"/"false"/"1"/"0".
        if isinstance(value, str):
            return ros_type, value.strip().lower() in ('true', '1', 'yes', 'on')
        return ros_type, bool(value)
    return ros_type, str(value)


def active_camera_targets(target: str, known: Tuple[str, ...]) -> List[str]:
    """Cameras to PAUSE when making ``target`` the single live detector.

    Mirrors the DSL ``_activate_camera`` exclusivity: pause every known camera
    except the target (the node skips absent ones via the graph before writing).
    """
    return [c for c in known if c != target]


def bbox_metrics(cx: float, cy: float, w: float, h: float,
                 frame: Optional[Tuple[int, int]]) -> Tuple[Optional[float],
                                                            Optional[float],
                                                            Optional[float]]:
    """(dx, dy, fill_pct) for a bbox given the frame size.

    dx/dy are the box centre offset from the FRAME centre in pixels (the exact
    quantity a ``vision.align(lat=<dx>, ...)`` author needs; +x = right, +y =
    below). fill_pct is bbox area as a %% of frame area (the ``vision.move`` knob).
    Returns None for a metric when the frame size is unknown/degenerate.
    """
    if not frame:
        return None, None, None
    fw, fh = frame
    if fw <= 0 or fh <= 0:
        return None, None, None
    dx = cx - fw / 2.0
    dy = cy - fh / 2.0
    fill = 100.0 * (w * h) / (fw * fh)
    return dx, dy, fill


def _round(v: Optional[float], nd: int = 1) -> Optional[float]:
    return None if v is None else round(float(v), nd)


def build_camera_view(cam: Dict[str, Any]) -> Dict[str, Any]:
    """Assemble one camera's JSON view (detections + per-class counts)."""
    frame = cam.get('frame')
    dets_out: List[Dict[str, Any]] = []
    counts: Dict[str, Dict[str, float]] = {}
    for d in cam.get('dets', []):
        dx, dy, fill = bbox_metrics(d['cx'], d['cy'], d['w'], d['h'], frame)
        conf = float(d['conf'])
        cls = str(d['cls'])
        dets_out.append({
            'cls':  cls,
            'conf': round(conf, 3),
            'dx':   _round(dx), 'dy': _round(dy),
            'cx':   _round(d['cx']), 'cy': _round(d['cy']),
            'fill': _round(fill, 2),
            'vis':  _round(d.get('vis'), 2),
            'id':   d.get('id', ''),
        })
        c = counts.setdefault(cls, {'n': 0, 'best': 0.0})
        c['n'] += 1
        c['best'] = max(c['best'], conf)
    # Strongest detections first -- the operator reads the top row to author a verb.
    dets_out.sort(key=lambda r: r['conf'], reverse=True)
    return {
        'present':      bool(cam.get('present', False)),
        'paused':       cam.get('paused'),
        'active_model': cam.get('active_model', ''),
        'models':       cam.get('models', []),
        'conf':         _round(cam.get('conf'), 3),
        'classes':      cam.get('classes', []),
        'fps':          _round(cam.get('fps', 0.0), 1),
        'frame':        list(frame) if frame else None,
        'dets':         dets_out,
        'counts':       counts,
    }


def build_snapshot(store: Dict[str, Any], video_port: int) -> Dict[str, Any]:
    """Full JSON snapshot pushed to the browser over SSE.

    Pure transform of the node's ``store`` -- easy to assert in a unit test.
    """
    cams = store.get('cameras', {})
    return {
        'cameras':       {name: build_camera_view(c) for name, c in cams.items()},
        'active_camera': store.get('active_camera'),
        'state':         store.get('state', {}),
        'video_port':    video_port,
        'ts':            store.get('ts', 0.0),
    }
