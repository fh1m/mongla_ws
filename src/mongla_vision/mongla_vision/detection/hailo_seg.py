"""`HailoSegDetector`: a raw segmentation HEF behind the same `Detector` API.

The chip work is `HailoDetector`'s; this subclass maps the ten raw output
tensors to their roles and un-letterboxes what `seg_decode` returns.
`factory.make_detector` picks it when `hailo.emits_raw_heads` says the HEF
carries no NMS.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, List

import numpy as np

from .detector import Detection
from .hailo import _ASYNC_READY_MS, _ASYNC_WAIT_MS, HailoDetector


class HailoSegDetector(HailoDetector):
    """A RAW segmentation HEF behind the same `Detector` API.

    Public Model Zoo `*_seg.hef` files carry no NMS and no post-process at
    all: ten uint8 tensors come off the chip and the host does the whole
    decode. `seg_decode` is that decode and holds the reasoning; this class is
    only the wiring -- which output is which, and the un-letterboxing.

    Measured on the vehicle, 2026-09-11 (`yolov8n_seg`, 640x640, Hailo-8):

        chip inference                        6.28 ms
        host decode, 80 classes, with masks   2.98 ms
        host decode, 80 classes, boxes only   0.93 ms
        host decode, 3 classes,  boxes only   0.62 ms

    i.e. ~107 Hz end to end WITH masks and WITHOUT needing a model trained on
    our classes. The earlier budget in `.claude/context/` measured the same
    chip against a conventional decode at 31.5 ms and concluded segmentation
    was host-bound and needed a 3-class model for CPU reasons. That conclusion
    is OVERTURNED: gating the class head now buys 0.3 ms, not 26. A model on
    our classes is still worth having -- for accuracy, which is a different
    argument -- but it is no longer a performance prerequisite.

    Class gating is kept anyway because it is free and because an 80-class
    model WILL emit boxes for classes no mission asked for.
    """
    name = 'hailo_seg'

    def __init__(self, *, iou: float = 0.45, masks: bool = True, **kwargs):
        # Set before super(): the base __init__ runs a warm-up inference, and
        # by then `infer` must be able to run.
        self._iou = float(iou)
        self._want_masks = bool(masks)
        self._layout = None
        self._heads: list = []
        self._proto_name = ''
        self._bufs: Dict[str, np.ndarray] = {}
        if os.environ.get('MONGLA_HAILO_FORCE_BLOCKING'):
            # The blocking branch of the base __init__ builds a single-output
            # InferVStreams pipe and never calls `_configure_outputs`, so a seg
            # model would come up with no layout and return [] every frame
            # while looking healthy. Refuse instead.
            raise RuntimeError(
                'MONGLA_HAILO_FORCE_BLOCKING is set and the blocking HailoRT '
                'API has no multi-output path here. Unset it to run a '
                'segmentation model.')
        super().__init__(iou=iou, **kwargs)

    def set_masks(self, want: bool) -> None:
        """Turn mask decoding on or off between frames.

        Masks are the dominant host cost once the class head is gated: measured
        on the vehicle at 640x640 with 80 classes, decode goes 0.93 ms boxes-only
        to 2.98 ms with masks, and end to end 113.3 Hz to 85.2 Hz. A transit leg
        that only needs a bearing should not pay for outlines; a torpedo board
        approach, where the contour IS the geometry, should.

        Read on the next frame -- there is no state to rebuild, which is why
        this can be a live parameter rather than a relaunch.
        """
        self._want_masks = bool(want)

    # ------------------------------------------------------------------ #
    #  Which tensor is which
    # ------------------------------------------------------------------ #
    def _configure_outputs(self) -> None:
        """Read the ten outputs into a `SegLayout`, and ask for UINT8.

        UINT8, not FLOAT32, and that is the whole optimisation. Asking
        HailoRT for FLOAT32 makes it dequantise 672,000 class bytes plus
        819,200 prototype bytes on the host before our code sees them -- work
        that `seg_decode` then proves is unnecessary, because both the class
        threshold and the mask threshold commute with the affine
        dequantisation. Taking the bytes raw is what makes the decode 0.93 ms
        instead of 31.5.

        The heads are identified by SHAPE, not by name: `conv44` means nothing
        outside one export and a name-keyed decode breaks silently on the next
        model. The one genuine ambiguity is a 32-class model, whose class head
        and mask-coefficient head are both 32 channels -- broken there by the
        class head's signature quantisation (zp 0, scale 1/255), and refused
        loudly if that does not separate them.
        """
        from hailo_platform import FormatType
        from .seg_decode import MASK_DIM, Quant, REG_MAX, ScaleLayout, SegLayout

        for name in self._model.output_names:
            self._model.output(name).set_format_type(FormatType.UINT8)

        infos = {i.name: i for i in self._hef.get_output_vstream_infos()}
        by_grid: Dict[int, list] = {}
        for name, info in infos.items():
            shape = tuple(info.shape)
            by_grid.setdefault(int(shape[0]), []).append((name, int(shape[2])))

        proto_grid = max(by_grid)
        proto = [n for n, c in by_grid.pop(proto_grid) if c == MASK_DIM]
        if len(proto) != 1:
            raise ValueError(
                f'{Path(self._path).name}: expected one {MASK_DIM}-channel '
                f'prototype tensor at the {proto_grid}x{proto_grid} grid, '
                f'found {len(proto)}. This does not look like a YOLO seg HEF.')
        self._proto_name = proto[0]

        def _q(name) -> 'Quant':
            qi = infos[name].quant_info
            return Quant(float(qi.qp_zp), float(qi.qp_scale))

        scales, heads, ncls = [], [], None
        # Fine grid first, matching stride 8/16/32 -- the order `seg_decode`
        # expects and the order the anchor arithmetic depends on.
        for grid in sorted(by_grid, reverse=True):
            entries = by_grid[grid]
            box = [n for n, c in entries if c == 4 * REG_MAX]
            rest = [(n, c) for n, c in entries if c != 4 * REG_MAX]
            coeff = [n for n, c in rest if c == MASK_DIM and not _q(n).is_unit_probability]
            cls = [n for n, c in rest if n not in coeff]
            if len(box) != 1 or len(coeff) != 1 or len(cls) != 1:
                raise ValueError(
                    f'{Path(self._path).name}: the {grid}x{grid} grid has '
                    f'{len(box)} box / {len(cls)} class / {len(coeff)} '
                    f'coefficient heads. Cannot decode a head this shape.')
            nc = int(infos[cls[0]].shape[2])
            if ncls is not None and nc != ncls:
                raise ValueError(f'{Path(self._path).name}: class heads '
                                 f'disagree on class count ({ncls} vs {nc}).')
            ncls = nc
            scales.append(ScaleLayout(stride=self._size // grid, box=_q(box[0]),
                                      cls=_q(cls[0]), coeff=_q(coeff[0])))
            heads.append((box[0], cls[0], coeff[0]))

        self._layout = SegLayout(size=self._size, num_classes=int(ncls),
                                 scales=tuple(scales), proto=_q(self._proto_name))
        self._heads = heads
        self._out_shape = ()
        if self._log:
            probs = [s.cls.is_unit_probability for s in scales]
            self._log.info(
                f'[HAILO] seg heads: strides {[s.stride for s in scales]} '
                f'classes={ncls} proto={proto_grid}x{proto_grid} '
                f'sigmoid-baked={all(probs)}')

    def _bind_outputs(self) -> None:
        self._bufs = {n: np.zeros(tuple(self._model.output(n).shape), np.uint8)
                      for n in self._model.output_names}
        for name, buf in self._bufs.items():
            self._bindings.output(name).set_buffer(buf)
        self._out_buf = None

    # ------------------------------------------------------------------ #
    #  Inference
    # ------------------------------------------------------------------ #
    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        if not self._ready or frame_bgr is None or self._layout is None:
            return []
        import cv2
        from .seg_decode import decode

        h, w = frame_bgr.shape[:2]
        with self._infer_lock:
            cim = self._configured()
            scale, pad_x, pad_y = self._letterbox_into_bound(frame_bgr)
            cim.wait_for_async_ready(timeout_ms=_ASYNC_READY_MS)
            cim.run_async([self._bindings]).wait(_ASYNC_WAIT_MS)
            # Decode INSIDE this detector's lock: the output buffers stay
            # bound and a second inference on this detector would overwrite
            # them mid-decode. Other models keep the chip meanwhile.
            heads = [(self._bufs[b], self._bufs[c], self._bufs[m])
                     for b, c, m in self._heads]
            xyxy, scores, cids, masks = decode(
                self._layout, heads, self._bufs[self._proto_name],
                conf=self._conf,
                allow_ids=(None if self._allow_ids is None else sorted(self._allow_ids)),
                iou=self._iou, max_det=self._max_det,
                want_masks=self._want_masks)

        out: List[Detection] = []
        for i in range(len(scores)):
            x1 = (float(xyxy[i, 0]) - pad_x) / scale
            x2 = (float(xyxy[i, 2]) - pad_x) / scale
            y1 = (float(xyxy[i, 1]) - pad_y) / scale
            y2 = (float(xyxy[i, 3]) - pad_y) / scale
            mask = None
            if i < len(masks):
                # The mask came back at the box's size in NETWORK pixels; the
                # box is about to be reported in FRAME pixels. Resize by the
                # same letterbox scale so the two stay the same object.
                fw = max(1, int(round(min(float(w), x2) - max(0.0, x1))))
                fh = max(1, int(round(min(float(h), y2) - max(0.0, y1))))
                m = masks[i]
                if m.shape != (fh, fw):
                    m = cv2.resize(m, (fw, fh), interpolation=cv2.INTER_NEAREST)
                mask = m
            cid = int(cids[i])
            out.append(Detection(
                class_id=cid, class_name=self._names.get(cid, str(cid)),
                score=float(scores[i]),
                xyxy=(max(0.0, x1), max(0.0, y1),
                      min(float(w), x2), min(float(h), y2)),
                mask=mask))
        return out

    def __repr__(self) -> str:
        nc = 0 if self._layout is None else self._layout.num_classes
        return (f'<HailoSegDetector {Path(self._path).name} '
                f'{self._size}x{self._size} classes={nc} '
                f'masks={self._want_masks}>')
