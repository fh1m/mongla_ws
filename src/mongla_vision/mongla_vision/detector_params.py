"""How the detector's parameters are read.

Model specs (`active_model`, `models`) and per-model confidence.
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

    Raises ValueError naming every malformed pair: a bare name, a missing
    name or a conf that is not a number. Skipping them silently left the
    model on its old conf while the set reported success.
    """
    result: Dict[str, float] = {}
    bad = []
    for part in str(s or '').split(','):
        part = part.strip()
        if not part:
            continue
        name, eq, value = part.partition('=')
        name, value = name.strip(), value.strip()
        try:
            if not (eq and name):
                raise ValueError
            result[name] = float(value)
        except ValueError:
            bad.append(part)
    if bad:
        raise ValueError(f'model_conf pairs must be name=number; not {bad}')
    return result


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
