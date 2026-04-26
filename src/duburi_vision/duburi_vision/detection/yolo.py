"""YoloDetector — Ultralytics YOLO26 (or any compatible model file).

Robust to:
  * missing CUDA   -> select_device fails loudly with a friendly message
  * missing weights -> ultralytics auto-downloads on first call
  * empty results  -> infer() returns [] (never raises on a black frame)
  * cold start     -> one warmup forward pass on init so the first real
                      frame isn't 1-2 s of kernel compilation
  * class allowlist -> filtered AFTER inference so the model still uses its
                       full COCO context (better than retraining for two
                       classes); allowlist=None means "keep everything"
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, List, Optional

import numpy as np

from .detector    import Detector, Detection
from .gpu         import select_device
from .class_index import load_class_index


# Maps our descriptive model names to Ultralytics' official short stems.
# Used only when no custom .pt exists in models/ — lets Ultralytics auto-download
# the right pretrained weights without the caller knowing the short stem.
_PRETRAINED_ALIASES: dict[str, str] = {
    'yolo26_nano_pretrained':   'yolo26n',
    'yolo26_small_pretrained':  'yolo26s',
    'yolo26_medium_pretrained': 'yolo26m',
    'yolo26_large_pretrained':  'yolo26l',
    'yolo26_xlarge_pretrained': 'yolo26x',
}


def _resolve_model_path(name: str) -> str:
    """Resolve a bare model name to a full path.

    Priority order:
      1. Path as-is     — if *name* has a separator or .pt extension.
      2. share dir      — ``<package_share>/models/<name>.pt`` via ament_index.
      3. Source tree    — ``<this_file>/../../models/<name>.pt`` (dev / pre-build).
      4. Alias table    — maps descriptive names like ``yolo26_nano_pretrained``
                          to Ultralytics short stems like ``yolo26n`` so auto-
                          download works correctly.
      5. Bare stem      — ``name + '.pt'``; Ultralytics will try to download it.
    """
    if os.sep in name or name.endswith('.pt'):
        return name
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('duburi_vision')
        candidate = Path(share) / 'models' / f'{name}.pt'
        if candidate.exists():
            return str(candidate)
    except Exception:
        pass
    # Source-tree fallback: works before/during colcon build.
    # __file__ is duburi_vision/detection/yolo.py → ../../../models/
    src_candidate = Path(__file__).parent.parent.parent / 'models' / f'{name}.pt'
    if src_candidate.exists():
        return str(src_candidate)
    # Map descriptive name → Ultralytics short stem so auto-download works.
    if name in _PRETRAINED_ALIASES:
        return f'{_PRETRAINED_ALIASES[name]}.pt'
    return f'{name}.pt'


class YoloDetector(Detector):
    name = 'yolo'

    def __init__(self, *, model_path='yolo26_nano_pretrained', device='cuda:0',
                 conf=0.35, iou=0.5, imgsz=640, half=False,
                 class_allowlist: Optional[Iterable[str]] = ('person',),
                 warmup=True, logger=None):
        from ultralytics import YOLO

        self._log     = logger
        self._device  = select_device(device, logger=logger)
        self._conf    = float(conf)
        self._iou     = float(iou)
        self._imgsz   = int(imgsz)
        self._half    = bool(half) and self._device.startswith('cuda')

        resolved_path = _resolve_model_path(model_path)
        self._model = YOLO(resolved_path)
        try:
            self._model.to(self._device)
        except Exception as exc:
            raise RuntimeError(
                f"failed to move {resolved_path!r} to {self._device}: {exc!r}") from exc

        # Prefer class names from a sidecar YAML (e.g. yolo26_nano_pretrained.yaml)
        # so custom models can override the embedded names table.
        yaml_names = load_class_index(resolved_path)
        self._names = yaml_names if yaml_names else dict(getattr(self._model, 'names', {}) or {})

        if self._log and self._names:
            cols = 5
            rows = [list(self._names.items())[i:i+cols]
                    for i in range(0, len(self._names), cols)]
            table = '\n    '.join(
                '  '.join(f'{cid:3d}: {cname:<15s}' for cid, cname in row)
                for row in rows)
            self._log.info(
                f"[YOLO ] model: {Path(resolved_path).name} ({imgsz}×{imgsz})\n"
                f"[YOLO ] available classes ({len(self._names)}):\n    {table}")

        self._allow_ids = None
        if class_allowlist is not None:
            allow_lower = {str(c).strip().lower() for c in class_allowlist if c}
            self._allow_ids = {
                cid for cid, cname in self._names.items()
                if str(cname).strip().lower() in allow_lower
            }
            if not self._allow_ids and allow_lower:
                if self._log:
                    self._log.warning(
                        f"[YOLO ] class_allowlist={sorted(allow_lower)} matched 0 classes "
                        f"in model. Detector will return [] for every frame. Available: "
                        f"{sorted(self._names.values())[:10]}...")

        self._ready = False
        if warmup:
            self._do_warmup()
        self._ready = True

        if self._log:
            active = self._allowlist_repr()
            self._log.info(
                f"[YOLO ] {Path(resolved_path).name} ready on {self._device}  "
                f"conf={self._conf}  iou={self._iou}  imgsz={self._imgsz}  "
                f"half={self._half}  active_filter={active}")

    def _allowlist_repr(self):
        if self._allow_ids is None:
            return '*'
        return sorted(self._names[i] for i in self._allow_ids)

    def _do_warmup(self):
        dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8)
        try:
            self._model.predict(
                dummy, conf=self._conf, iou=self._iou, imgsz=self._imgsz,
                device=self._device, half=self._half, verbose=False)
        except Exception as exc:
            if self._log:
                self._log.warning(f"[YOLO ] warmup failed (non-fatal): {exc!r}")

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        results = self._model.predict(
            frame_bgr,
            conf=self._conf, iou=self._iou, imgsz=self._imgsz,
            device=self._device, half=self._half, verbose=False)
        if not results:
            return []
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []

        xyxy   = boxes.xyxy.detach().cpu().numpy()
        conf   = boxes.conf.detach().cpu().numpy()
        cls_id = boxes.cls.detach().cpu().numpy().astype(int)

        out: List[Detection] = []
        for (x1, y1, x2, y2), score, cid in zip(xyxy, conf, cls_id):
            if self._allow_ids is not None and cid not in self._allow_ids:
                continue
            out.append(Detection(
                class_id=int(cid),
                class_name=str(self._names.get(int(cid), str(int(cid)))),
                score=float(score),
                xyxy=(float(x1), float(y1), float(x2), float(y2)),
            ))
        return out

    def update_allowlist(self, class_allowlist):
        """Hot-swap the class filter without reloading the model.

        class_allowlist -- list of class name strings, or None to keep all.
        Called by detector_node when the 'classes' ROS param changes live.
        """
        if class_allowlist is None:
            self._allow_ids = None
        else:
            allow_lower = {str(c).strip().lower() for c in class_allowlist if c}
            self._allow_ids = {
                cid for cid, cname in self._names.items()
                if str(cname).strip().lower() in allow_lower
            }
            if not self._allow_ids and allow_lower and self._log:
                self._log.warning(
                    f"[YOLO ] update_allowlist: {sorted(allow_lower)} matched 0 classes. "
                    f"Available: {sorted(self._names.values())[:10]}")

    def class_names(self):
        return dict(self._names)

    def is_ready(self) -> bool:
        return self._ready

    def close(self) -> None:
        self._model = None
