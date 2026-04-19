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

from typing import Iterable, List, Optional

import numpy as np

from .detector import Detector, Detection
from .gpu      import select_device


class YoloDetector(Detector):
    name = 'yolo'

    def __init__(self, *, model_path='models/yolo26n.pt', device='cuda:0',
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

        self._model = YOLO(model_path)
        try:
            self._model.to(self._device)
        except Exception as exc:
            raise RuntimeError(
                f"failed to move {model_path!r} to {self._device}: {exc!r}") from exc

        self._names = dict(getattr(self._model, 'names', {}) or {})

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
                        f"[YOLO] class_allowlist={sorted(allow_lower)} matched 0 classes "
                        f"in model. Detector will return [] for every frame. Available: "
                        f"{sorted(self._names.values())[:10]}...")

        self._ready = False
        if warmup:
            self._do_warmup()
        self._ready = True

        if self._log:
            self._log.info(
                f"[YOLO] {model_path} ready on {self._device}  "
                f"conf={self._conf}  iou={self._iou}  imgsz={self._imgsz}  "
                f"half={self._half}  allow={self._allowlist_repr()}")

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
                self._log.warning(f"[YOLO] warmup failed (non-fatal): {exc!r}")

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

    def class_names(self):
        return dict(self._names)

    def is_ready(self) -> bool:
        return self._ready

    def close(self) -> None:
        self._model = None
