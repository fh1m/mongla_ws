"""references -- disk persistence for named anchor reference frames.

Named references are saved as ``duburi_vision/duburi_vision/references/<name>.png``
(inside the package source tree so they're versioned with the repo and survive
restarts -- snap a prop the day before, reload it on competition day). Only the
BGR frame is stored (re-described on load); keeping it a plain PNG makes a
reference inspectable and portable.

Path resolution mirrors ``yolo._find_src_models_dir``: prefer the workspace
source tree (works on any clone without a colcon build), fall back to the
package dir next to this file.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

# Reference names become filenames -- keep them filesystem-safe.
_SAFE_NAME = re.compile(r'[^A-Za-z0-9._-]')


def _safe_name(name: str) -> str:
    return _SAFE_NAME.sub('_', str(name).strip()) or 'ref'


def references_dir() -> Path:
    """Return (creating if needed) the writable references directory.

    Same /install/ split trick as yolo._find_src_models_dir so writes land in
    the SOURCE tree (versioned, persistent) on an installed deployment too.
    """
    this = Path(__file__).resolve()
    path_str = str(this)
    if '/install/' in path_str:
        ws_root = Path(path_str.split('/install/')[0])
        src_dir = ws_root / 'src' / 'duburi_vision' / 'duburi_vision' / 'references'
        if src_dir.parent.is_dir():
            src_dir.mkdir(parents=True, exist_ok=True)
            return src_dir
    # Pre-build / source tree: .../duburi_vision/duburi_vision/anchor/references.py
    cand = Path(__file__).parent.parent / 'references'
    cand.mkdir(parents=True, exist_ok=True)
    return cand


def reference_path(name: str) -> Path:
    return references_dir() / f'{_safe_name(name)}.png'


def save_reference(name: str, frame_bgr: np.ndarray, bbox=None) -> Optional[Path]:
    """Write ``frame_bgr`` to references/<name>.png (the FULL frame).

    When ``bbox=(x1,y1,x2,y2)`` is given (a crop reference), also write
    ``<name>.json`` carrying the bbox so ``load_reference`` can re-crop on
    reload and keep the crop geometry. Returns the image path or None.
    """
    if frame_bgr is None or getattr(frame_bgr, 'size', 0) == 0:
        return None
    path = reference_path(name)
    if not cv2.imwrite(str(path), frame_bgr):
        return None
    side = path.with_suffix('.json')
    if bbox is not None:
        side.write_text(json.dumps({'bbox': [int(round(v)) for v in bbox]}))
    elif side.exists():
        side.unlink()   # stale sidecar from a prior crop save of the same name
    return path


def load_reference(name: str) -> Tuple[Optional[np.ndarray], Optional[tuple]]:
    """Read references/<name>.png -> (frame_bgr, bbox|None).

    ``bbox`` is read from the sidecar JSON if present (crop reference), else
    None (whole-frame reference). Returns (None, None) when the PNG is missing.
    """
    path = reference_path(name)
    if not path.exists():
        return None, None
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None or img.size == 0:
        return None, None
    bbox = None
    side = path.with_suffix('.json')
    if side.exists():
        try:
            data = json.loads(side.read_text())
            b = data.get('bbox')
            if isinstance(b, list) and len(b) == 4:
                bbox = tuple(int(v) for v in b)
        except Exception:
            bbox = None
    return img, bbox
