"""What a `.hef` and its sidecar say about themselves, read without a device.

The class names (from the `<stem>.yaml` sidecar, because a HEF carries none),
the NMS score floor compiled into it, and whether it decodes on the chip or
leaves raw heads for the host. `hailo.py` and `factory.py` read these; none of
them opens the chip.
"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path
from typing import Dict, Optional


def load_class_index(model_path: str) -> Optional[Dict[int, str]]:
    """{id: name} from the sidecar YAML beside the model, or None.

    Same suffix-swap convention as the YOLO backend, so a stem resolves the
    same sidecar whatever backend loads it -- that is what keeps missions,
    `ClassRef` and `mongla.use('<stem>')` backend-agnostic.
    """
    yaml_path = Path(model_path).with_suffix('.yaml')
    if not yaml_path.exists():
        return None
    try:
        import yaml
        data = yaml.safe_load(yaml_path.read_text())
        names = data.get('names', {}) if isinstance(data, dict) else {}
        return {int(k): str(v) for k, v in names.items()}
    except Exception:
        return None


def baked_score_threshold(hef_path: str) -> Optional[float]:
    """The NMS floor compiled INTO the HEF, or None if it cannot be read.

    There is no Python API for this -- `HEF` exposes stream infos and nothing
    about the post-process, and the configured network group has only
    `set_scheduler_threshold`, which is a different thing entirely. So this
    shells out to `hailortcli parse-hef`, which prints it and costs 15 ms once
    per detector.

    It matters because the floor is INVISIBLE otherwise: the HEF's NMS layer
    drops everything below it before the host sees a byte, so a mission that
    sets conf=0.03 gets 0.05 and is told nothing. Ours are 0.050 and the stock
    Model Zoo models are 0.200 -- which is why the 0.12-0.15 operating point
    this backend recommends is reachable on our models and NOT on a stock one.

    Best effort by design. `hailortcli` may not be on PATH, and a detector that
    refused to start over a diagnostic would be worse than one that says it
    could not read it.
    """
    try:
        out = subprocess.run(['hailortcli', 'parse-hef', hef_path],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:                                            # noqa: BLE001
        return None
    m = re.search(r'Score threshold:\s*([0-9.]+)', out)
    return float(m.group(1)) if m else None


def emits_raw_heads(hef_path: str) -> bool:
    """True when the HEF post-processes NOTHING and the host must decode.

    The test is the output count, not the file name. A HEF whose NMS ran
    on-chip has exactly ONE output -- the HAILO_NMS buffer. A raw head has one
    tensor per scale per branch (ten for YOLOv8-seg). Naming the seg models by
    convention instead would put the two decodes one typo apart, and picking
    the wrong one does not raise: an NMS decode of a raw head reads
    convolution activations as box coordinates.

    Cheap: reads the HEF's metadata, no device and no configure.
    """
    try:
        from hailo_platform import HEF
        return len(HEF(str(hef_path)).get_output_vstream_infos()) > 1
    except Exception:                                            # noqa: BLE001
        return False
