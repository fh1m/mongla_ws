"""How the detector's parameters are read.

Model specs (`active_model`, `models`), per-model confidence, and the "unset"
sentinels that let a launch profile fill in what the operator left alone.
Pure functions: no ROS, so each is tested without a node.
"""
import os
from typing import Dict


def _split_active(spec: str):
    """`active_model` -> (primary, [secondaries]).

    ONE param, not two, and comma-separated rather than a second
    `extra_models` key. Two params cannot be set atomically, so a mission
    switching from `a` to `b,c` would pass through a tick where the primary is
    already `b` while the secondary list still says the old model -- a frame
    detected with the wrong pair, and nothing would log it. A single CSV
    changes the whole selection in one `SetParameters` call.

    No comma is byte-for-byte the behaviour that shipped before this existed.
    """
    parts = [p.strip() for p in str(spec or '').split(',') if p.strip()]
    if not parts:
        return '', []
    return parts[0], parts[1:]


def _model_stem(path: str) -> str:
    """Canonical model identity: basename without extension.

    ``'gate_sharks'`` -> ``'gate_sharks'``;
    ``'/models/gate_sharks.pt'`` -> ``'gate_sharks'``.
    Missions name models by this stem (``mongla.models('stem')`` / ClassRef /
    ``set_model('stem')``), so the node matches ``active_model`` against it
    regardless of whether the model was launched single (``model:=stem``) or in a
    registry (``models:=key=stem`` or bare ``models:=stem``).
    """
    return os.path.splitext(os.path.basename(str(path).strip()))[0]


def _parse_model_conf(s: str) -> Dict[str, float]:
    """Parse 'torpedo_blood_hole=0.55,gate=0.35' -> {name: conf_float}.

    Skips malformed / non-numeric pairs silently (a live-tuned param must never
    crash the callback). Bare entries (no '=') are ignored -- a per-model conf
    needs a model name.
    """
    result: Dict[str, float] = {}
    for part in s.split(','):
        part = part.strip()
        if not part or '=' not in part:
            continue
        name, _, value = part.partition('=')
        name, value = name.strip(), value.strip()
        if not name:
            continue
        try:
            result[name] = float(value)
        except ValueError:
            continue
    return result


# "not set by the operator". A launch file passes every parameter, so the
# only way to tell a deliberate choice from a filled-in default is a value
# that means nothing on its own. Each is outside the setting's real range.
_UNSET = {
    # `conf` is DELIBERATELY ABSENT. It also feeds the tracker's confidence
    # clamp through the launch (`detector_conf`), and a sentinel there would
    # silently disable the clamp -- which measured as the tracker emitting
    # NOTHING on real water. A profile therefore cannot lower `conf` below an
    # explicit launch value; set both, or set neither.
    'preprocess': 'auto',     # 'off'/'clahe' are the real values
    'preprocess_clip': 0.0,   # a real clip limit is > 0
    'range_crop': -1,         # an int, because a bool cannot carry a third state
}


def _is_unset(name, value) -> bool:
    """True when `value` is the documented sentinel for `name`."""
    s = _UNSET.get(name)
    if s is None:
        return False
    if isinstance(s, str):
        return str(value).strip().lower() == s
    return value == s


def _parse_models_param(s: str) -> Dict[str, str]:
    """Parse 'gate=gate_sharks,sauvc=sauvc_sim' → {name: stem}.

    Bare entries without '=' use the stem as both name and path:
        'gate_sharks' → {'gate_sharks': 'gate_sharks'}
    """
    result: Dict[str, str] = {}
    for part in s.split(','):
        part = part.strip()
        if not part:
            continue
        if '=' in part:
            k, v = part.split('=', 1)
            result[k.strip()] = v.strip()
        else:
            result[part] = part
    return result
