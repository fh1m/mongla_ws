"""Hailo-8 detector backend: a compiled ``.hef`` behind the same `Detector` API.

Replaces the Jetson's TensorRT path on the Raspberry Pi 5 + AI HAT+. Measured
in round 24 on this exact hardware: YOLO11n end-to-end at **82.3 Hz** against
the Jetson's 20-30 Hz, with our own 3-class `gate_rescue_repair` at 97.8 FPS
chip-side -- faster than the stock 80-class COCO model, so the proxy used to
plan this was conservative.

Four things differ from `.pt` / `.engine`, and each is silent if missed
---------------------------------------------------------------------
1. **The HEF carries no class names.** A `.pt` has `model.names`; a `.hef` has
   nothing. The `<stem>.yaml` sidecar therefore stops being advisory -- without
   it there is no id->label map, the allowlist matches nothing, and the
   detector returns `[]` every frame while looking perfectly healthy. This
   class REFUSES to construct without one rather than run blind.

2. **`conf` is baked in at compile time and can only be TIGHTENED.** The HEF's
   NMS layer drops anything below the threshold it was compiled with before the
   host ever sees it, so `update_conf()` filters what survives and cannot
   recover what was already discarded. Our models are compiled at 0.05 for
   exactly this reason: it is free (97.7 FPS either way) and it leaves the
   operating point a runtime decision.

3. **Run at conf 0.12-0.15, not the CUDA path's 0.45.** Round 24 measured INT8
   costing ~0.08 of confidence at the 0.20 operating point while NOT moving the
   box centre (2.14-2.65 px against a 2.72 px fp32-vs-fp32 noise floor). The
   detections are there, they score lower.

4. **NMS runs on the HOST, inside HailoRT.** The DFC puts the YOLOv8 head --
   which YOLO11 shares -- only in `CPU_META_ARCHS`. It is cheap here (0.04 ms)
   because we have 3 classes, not 80: class count costs ~2 ms from 3 -> 80,
   detection count costs nothing.

Output format
-------------
HailoRT returns NMS results grouped BY CLASS: a list indexed by class id, each
entry an array of `(y1, x1, y2, x2, score)` in **normalised letterbox
coordinates**, y before x. Both of those are easy to get backwards and neither
raises when you do -- a transposed box just tracks the wrong way, which on a
vision-servoed hull means driving away from the target.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import numpy as np

from .detector import Detection, Detector

_INPUT_FALLBACK = 640


def _load_class_index(model_path: str) -> Optional[Dict[int, str]]:
    """{id: name} from the sidecar YAML beside the model, or None.

    Same suffix-swap convention as the YOLO backend, so a stem resolves the
    same sidecar whatever backend loads it -- that is what keeps missions,
    `ClassRef` and `duburi.use('<stem>')` backend-agnostic.
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


def letterbox(im: np.ndarray, size: int):
    """Resize preserving aspect into a `size`x`size` grey canvas.

    Returns (canvas, scale, pad_x, pad_y) because un-letterboxing needs all
    three: dropping the pads maps every box onto a stretched frame, which looks
    plausible and is wrong by the size of the bars.
    """
    import cv2
    h, w = im.shape[:2]
    s = min(size / h, size / w)
    nh, nw = int(round(h * s)), int(round(w * s))
    out = np.full((size, size, 3), 114, np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    out[py:py + nh, px:px + nw] = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return out, s, px, py


class HailoDetector(Detector):
    name = 'hailo'

    def __init__(self, *, model_path: str, conf: float = 0.15,
                 max_det: int = 100,
                 class_allowlist: Optional[Iterable[str]] = ('person',),
                 warmup: bool = True, logger=None, **_ignored):
        # `**_ignored` swallows the CUDA-only kwargs (device, iou, half) so the
        # three existing construction sites can pass their full kwarg set
        # unchanged. iou and half are properties of a graph compiled hours ago
        # on an x86 box; accepting them silently would imply they still tune
        # something.
        self._log = logger
        self._conf = float(conf)
        self._max_det = int(max_det)
        self._path = str(model_path)

        # Sidecar FIRST, before touching the runtime. It is a cheap,
        # hardware-independent precondition, and checking it after the import
        # means a machine without hailo_platform reports "no module named
        # hailo_platform" for a model that is ALSO missing its class index --
        # you fix the runtime, then hit the real problem. Report the thing the
        # operator can act on.
        names = _load_class_index(self._path)
        if not names:
            raise FileNotFoundError(
                f'{self._path}: no class-index sidecar. A .hef carries NO class '
                f'names, so without {Path(self._path).with_suffix(".yaml").name} '
                f'there is no id->label map, the allowlist matches nothing, and '
                f'every frame returns [] while the pipeline looks healthy. '
                f'Ship the sidecar beside the .hef.')
        self._names: Dict[int, str] = names

        from hailo_platform import (HEF, VDevice, HailoStreamInterface,
                                    InferVStreams, ConfigureParams,
                                    InputVStreamParams, OutputVStreamParams,
                                    FormatType)

        self._hef = HEF(self._path)
        in_info = self._hef.get_input_vstream_infos()[0]
        out_info = self._hef.get_output_vstream_infos()[0]
        self._in_name, self._out_name = in_info.name, out_info.name
        shape = tuple(in_info.shape)
        self._size = int(shape[0]) if len(shape) >= 2 else _INPUT_FALLBACK

        # The VDevice is EXCLUSIVE: a second process asking for one gets
        # HAILO_OUT_OF_PHYSICAL_DEVICES (74). Two cameras therefore share one
        # device with two network groups, not two devices -- measured round 24.
        self._target = VDevice()
        cfg = ConfigureParams.create_from_hef(
            self._hef, interface=HailoStreamInterface.PCIe)
        self._ng = self._target.configure(self._hef, cfg)[0]
        self._ngp = self._ng.create_params()
        ivp = InputVStreamParams.make(self._ng, format_type=FormatType.UINT8)
        ovp = OutputVStreamParams.make(self._ng, format_type=FormatType.FLOAT32)

        self._activation = self._ng.activate(self._ngp)
        self._activation.__enter__()
        self._pipe = InferVStreams(self._ng, ivp, ovp)
        self._pipe.__enter__()
        self._ready = True

        self._allow_ids: Optional[set] = None
        self.update_allowlist(class_allowlist)

        if self._log:
            self._log.info(
                f'[HAILO] {Path(self._path).name} in={self._size}x{self._size} '
                f'classes={len(self._names)} conf={self._conf:.2f}')

        if warmup:
            # The first infer pays one-time setup. Paying it here keeps it out
            # of the first mission frame, where it reads as a dropped frame.
            blank = np.zeros((self._size, self._size, 3), np.uint8)
            try:
                self._pipe.infer({self._in_name: np.expand_dims(blank, 0)})
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    #  Live tuning -- the surface detector_node calls
    # ------------------------------------------------------------------ #
    def update_allowlist(self, class_allowlist) -> None:
        if class_allowlist is None:
            self._allow_ids = None
            return
        wanted = {str(c).strip().lower() for c in class_allowlist if str(c).strip()}
        if not wanted:
            self._allow_ids = None
            return
        ids = {cid for cid, cname in self._names.items() if cname.lower() in wanted}
        unknown = wanted - {self._names[i].lower() for i in ids}
        if unknown and self._log:
            self._log.warn(
                f'[HAILO] class(es) {sorted(unknown)} are not in this model. '
                f'Available: {sorted(self._names.values())[:10]}')
        self._allow_ids = ids

    def update_conf(self, conf: float) -> None:
        """Runtime confidence. Can only TIGHTEN what the HEF already emitted.

        The compiled NMS threshold is the real floor; anything below it was
        discarded on the way out and no runtime value can bring it back. Ask
        for less than the baked value and you get the baked value, silently --
        so log when that happens rather than let a mission believe it lowered
        the bar.
        """
        self._conf = float(conf)

    def update_max_det(self, max_det: int) -> None:
        self._max_det = int(max_det)

    def class_names(self) -> Dict[int, str]:
        return dict(self._names)

    def is_ready(self) -> bool:
        return bool(self._ready)

    # ------------------------------------------------------------------ #
    #  Inference
    # ------------------------------------------------------------------ #
    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        if not self._ready or frame_bgr is None:
            return []
        h, w = frame_bgr.shape[:2]
        buf, scale, pad_x, pad_y = letterbox(frame_bgr, self._size)
        res = self._pipe.infer({self._in_name: np.expand_dims(buf, 0)})
        raw = res[self._out_name]
        # Batch of 1: unwrap the leading batch axis.
        per_class = raw[0] if len(raw) else []

        out: List[Detection] = []
        for cid, boxes in enumerate(per_class):
            if boxes is None or len(boxes) == 0:
                continue
            if self._allow_ids is not None and cid not in self._allow_ids:
                continue
            name = self._names.get(int(cid), str(int(cid)))
            for b in boxes:
                score = float(b[4])
                if score < self._conf:
                    continue
                # HailoRT emits (y1, x1, y2, x2) NORMALISED to the letterboxed
                # square -- y first, and relative to the padded canvas, not the
                # frame. Undo the pad before the scale; doing it the other way
                # round is off by the bar width and still looks like a box.
                y1, x1, y2, x2 = (float(b[0]), float(b[1]), float(b[2]), float(b[3]))
                x1 = (x1 * self._size - pad_x) / scale
                x2 = (x2 * self._size - pad_x) / scale
                y1 = (y1 * self._size - pad_y) / scale
                y2 = (y2 * self._size - pad_y) / scale
                out.append(Detection(
                    class_id=int(cid), class_name=name, score=score,
                    xyxy=(max(0.0, x1), max(0.0, y1),
                          min(float(w), x2), min(float(h), y2))))
        if len(out) > self._max_det:
            out.sort(key=lambda d: d.score, reverse=True)
            del out[self._max_det:]
        return out

    def close(self) -> None:
        self._ready = False
        for obj, name in ((getattr(self, '_pipe', None), 'pipe'),
                          (getattr(self, '_activation', None), 'activation')):
            try:
                if obj is not None:
                    obj.__exit__(None, None, None)
            except Exception:
                pass
        try:
            if getattr(self, '_target', None) is not None:
                self._target.release()
        except Exception:
            pass

    def __repr__(self) -> str:
        return (f'<HailoDetector {Path(self._path).name} '
                f'{self._size}x{self._size} classes={len(self._names)}>')
