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

import re
from pathlib import Path
from typing import Optional

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


def save_reference(name: str, frame_bgr: np.ndarray) -> Optional[Path]:
    """Write ``frame_bgr`` to references/<name>.png. Returns the path or None."""
    if frame_bgr is None or getattr(frame_bgr, 'size', 0) == 0:
        return None
    path = reference_path(name)
    return path if cv2.imwrite(str(path), frame_bgr) else None


def load_reference(name: str) -> Optional[np.ndarray]:
    """Read references/<name>.png as BGR, or None if missing/unreadable."""
    path = reference_path(name)
    if not path.exists():
        return None
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    return img if img is not None and img.size > 0 else None
