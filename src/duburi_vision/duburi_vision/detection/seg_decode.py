"""Host-side decode of a RAW YOLOv8/v11-seg head, in the QUANTISED domain.

Why this module exists
----------------------
A public Model Zoo `*_seg.hef` has NO on-chip NMS. It emits ten raw tensors --
per scale a 64-channel DFL box head, an 80-channel class head and a
32-channel mask-coefficient head, plus one 160x160x32 prototype tensor -- and
every one of them is UINT8. The chip runs `yolov8n_seg` in 6.28 ms; the host
decode is what decides whether segmentation is usable, and the naive decode
measured 31.5 ms on this Pi. So the decode is the whole problem.

⛔ THE CLASS HEAD IS ALREADY A PROBABILITY, AND THAT IS THE LEVER.
Measured on the vehicle 2026-09-11, `hailortcli`/`quant_info` for all three
class heads (conv74/conv61/conv45):

    zp = 0.0,  scale = 0.00392157  ( = 1/255 )

That is not an arbitrary quantisation. A dequantised value is exactly
`u8 / 255`, so the head's output already lies in [0, 1]: the sigmoid is baked
into the graph, and the uint8 byte IS the confidence on a 1/255 grid. The
consequence is the one nobody writes down, because every published decode
dequantises first and then thresholds:

    thresholding the RAW BYTES is bit-exact identical to thresholding the
    dequantised floats.

`u8 -> (u8 - zp) * scale` is affine with a positive scale, so it is strictly
monotone, so `dequant(u8) >= conf` and `u8 >= ceil(conf/scale + zp)` select
the SAME cells -- not approximately, exactly. The expensive half of the naive
decode (converting 80x80x80 + 40x40x80 + 20x20x80 = 672,000 uint8 into
float32 and comparing) collapses into one uint8 comparison, and the float
conversion is then paid only for the handful of cells that survived.

This composes with class gating rather than competing with it: gating shrinks
the tensor you scan, the quantised threshold makes the scan itself cheap, and
a 3-class model gets both. It is also what makes an 80-class COCO seg model
survivable at all, which matters because the model trained on OUR classes does
not exist yet.

The same trick is NOT available on the box head (zp 74-86, scale 0.08-0.14 --
a real affine quantisation of logits that must be dequantised before the DFL
softmax) and is not needed there, because the box head is only ever read for
cells that already survived the class gate.

Everything here is plain numpy so it can be tested with no chip present; the
Hailo backend supplies the arrays and the layout.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

# YOLOv8/v11 head constants. Both are properties of the ARCHITECTURE, not of a
# particular export, which is why they are constants and not parameters: a head
# with a different reg_max is a different decode, and pretending otherwise with
# a knob would produce plausible boxes from the wrong arithmetic.
REG_MAX = 16        # DFL bins per box side; box head is 4 * REG_MAX channels
MASK_DIM = 32       # mask coefficients per detection == prototype channels


@dataclass(frozen=True)
class Quant:
    """One output's affine quantisation: `value = (u8 - zp) * scale`."""
    zp: float
    scale: float

    def dequant(self, u8: np.ndarray) -> np.ndarray:
        return (u8.astype(np.float32) - self.zp) * self.scale

    @property
    def is_unit_probability(self) -> bool:
        """True when the dequantised range is exactly [0, 1].

        zp = 0 and scale = 1/255 means the graph already applied the sigmoid
        and quantised a probability. Checking rather than assuming matters:
        a differently-compiled HEF may emit logits, and treating logits as
        probabilities silently moves the operating point instead of raising.
        """
        return abs(self.zp) < 1e-9 and abs(self.scale * 255.0 - 1.0) < 1e-6

    def score_floor_u8(self, conf: float) -> int:
        """The smallest byte whose dequantised score reaches `conf`.

        Exact, not approximate -- see the module docstring. Returned as an int
        so the caller compares uint8 against uint8 and never allocates a float
        array the size of the class head.
        """
        if self.is_unit_probability:
            target = float(conf)
        else:
            # Logit head: invert the sigmoid first, then the quantisation.
            c = min(max(float(conf), 1e-6), 1.0 - 1e-6)
            target = math.log(c / (1.0 - c))
        raw = target / self.scale + self.zp
        return int(max(0, min(255, math.ceil(raw - 1e-9))))

    def dequant_score(self, u8: np.ndarray) -> np.ndarray:
        """Bytes -> confidence in [0, 1], applying the sigmoid only if needed."""
        v = self.dequant(u8)
        if self.is_unit_probability:
            return v
        return 1.0 / (1.0 + np.exp(-v))


@dataclass(frozen=True)
class ScaleLayout:
    """Static description of one detection scale, read once from the HEF."""
    stride: int
    box: Quant
    cls: Quant
    coeff: Quant


@dataclass(frozen=True)
class SegLayout:
    """Everything about a seg HEF that does not change frame to frame."""
    size: int                       # network input edge, e.g. 640
    num_classes: int
    scales: Tuple[ScaleLayout, ...]
    proto: Quant


def candidates(cls_u8: np.ndarray, floor_u8: int,
               allow_ids: Optional[np.ndarray]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Cells whose best allowed class reaches the floor.

    Returns (flat_cell_index, class_id, score_u8). The scan is uint8
    throughout -- see the module docstring for why that is exact.

    `allow_ids` is the gate that cost 11.7 ms ungated and 0.2 ms on three
    classes. It is applied by SLICING the channel axis, not by masking after
    the fact: a mask still touches every byte.
    """
    h, w, nc = cls_u8.shape
    view = cls_u8 if allow_ids is None else cls_u8[:, :, allow_ids]
    flat = view.reshape(h * w, view.shape[2])
    # `max` then compare beats `any` then argmax: one pass, and the winning
    # score is needed anyway.
    best = flat.max(axis=1)
    keep = np.nonzero(best >= floor_u8)[0]
    if keep.size == 0:
        empty_i = np.empty(0, np.int64)
        return empty_i, empty_i, np.empty(0, np.uint8)
    local = flat[keep].argmax(axis=1)
    cids = local.astype(np.int64) if allow_ids is None else allow_ids[local]
    return keep, cids, best[keep]


def dfl_boxes(box_u8: np.ndarray, cells: np.ndarray, *,
              grid_w: int, stride: int, quant: Quant) -> np.ndarray:
    """DFL box head -> xyxy in NETWORK (letterboxed) pixels, for `cells` only.

    The 64 channels are (side, bin) with bin contiguous -- the layout
    ultralytics' `DFL` module assumes when it views (b, 4, 16, a) and softmaxes
    axis 2. A transposed read produces boxes that are plausible and wrong, so
    this is verified end to end against ultralytics rather than reasoned about.
    """
    rows = box_u8.reshape(-1, 4 * REG_MAX)[cells]
    logits = quant.dequant(rows).reshape(-1, 4, REG_MAX)
    logits -= logits.max(axis=2, keepdims=True)     # stable softmax
    np.exp(logits, out=logits)
    logits /= logits.sum(axis=2, keepdims=True)
    dist = logits @ np.arange(REG_MAX, dtype=np.float32)   # (N, 4) in cells
    dist *= stride

    cy, cx = np.divmod(cells, grid_w)
    ax = (cx.astype(np.float32) + 0.5) * stride
    ay = (cy.astype(np.float32) + 0.5) * stride
    return np.stack([ax - dist[:, 0], ay - dist[:, 1],
                     ax + dist[:, 2], ay + dist[:, 3]], axis=1)


def nms(xyxy: np.ndarray, scores: np.ndarray, cids: np.ndarray,
        iou_thr: float, max_det: int) -> np.ndarray:
    """Class-wise NMS. Returns surviving indices, highest score first.

    Class-wise, not global: two different props that overlap in the image are
    two detections, and a global NMS would delete the one behind.
    """
    order = np.argsort(scores)[::-1]
    keep: List[int] = []
    areas = ((xyxy[:, 2] - xyxy[:, 0]).clip(0) *
             (xyxy[:, 3] - xyxy[:, 1]).clip(0))
    while order.size and len(keep) < max_det:
        i = int(order[0])
        keep.append(i)
        rest = order[1:]
        if rest.size == 0:
            break
        xx1 = np.maximum(xyxy[i, 0], xyxy[rest, 0])
        yy1 = np.maximum(xyxy[i, 1], xyxy[rest, 1])
        xx2 = np.minimum(xyxy[i, 2], xyxy[rest, 2])
        yy2 = np.minimum(xyxy[i, 3], xyxy[rest, 3])
        inter = (xx2 - xx1).clip(0) * (yy2 - yy1).clip(0)
        iou = inter / np.maximum(areas[i] + areas[rest] - inter, 1e-9)
        # Suppress only within the same class.
        order = rest[(iou <= iou_thr) | (cids[rest] != cids[i])]
    return np.asarray(keep, dtype=np.int64)


def masks_for(coeff: np.ndarray, proto_u8: np.ndarray, quant: Quant,
              boxes_net: np.ndarray, *, size: int,
              thresh: float = 0.5) -> List[np.ndarray]:
    """Per-detection binary masks, each cropped to its own box.

    The prototype tensor is 160x160x32 and shared by every detection, so the
    whole cost here is one (P, 32) x (32, N) matmul plus a crop. Returned
    per-box rather than as a full-frame layer because a full-frame mask is
    640x640 per detection of mostly zeros, and every consumer wants the box
    anyway.

    Each mask is uint8 0/1 at the resolution of its own integer box in NETWORK
    (letterboxed) pixels -- the caller lifts it to frame coordinates with the
    same letterbox transform it uses for the box, so the two cannot disagree.
    """
    import cv2
    if coeff.shape[0] == 0:
        return []
    ph, pw, _ = proto_u8.shape
    proto = quant.dequant(proto_u8).reshape(ph * pw, MASK_DIM)
    logits = proto @ coeff.T                       # (P, N)
    np.clip(logits, -30.0, 30.0, out=logits)
    prob = 1.0 / (1.0 + np.exp(-logits))
    prob = prob.reshape(ph, pw, -1)

    sx, sy = pw / float(size), ph / float(size)
    out: List[np.ndarray] = []
    for n in range(coeff.shape[0]):
        x1, y1, x2, y2 = boxes_net[n]
        # Crop in PROTOTYPE space first: resizing the whole 160x160 plane to
        # box size and then cropping does the interpolation on pixels that are
        # about to be thrown away.
        px1 = int(np.clip(math.floor(x1 * sx), 0, pw - 1))
        py1 = int(np.clip(math.floor(y1 * sy), 0, ph - 1))
        px2 = int(np.clip(math.ceil(x2 * sx), px1 + 1, pw))
        py2 = int(np.clip(math.ceil(y2 * sy), py1 + 1, ph))
        bw = max(1, int(round(x2 - x1)))
        bh = max(1, int(round(y2 - y1)))
        patch = prob[py1:py2, px1:px2, n]
        patch = cv2.resize(patch, (bw, bh), interpolation=cv2.INTER_LINEAR)
        out.append((patch >= thresh).astype(np.uint8))
    return out


def decode(layout: SegLayout, heads: Sequence[Tuple[np.ndarray, np.ndarray, np.ndarray]],
           proto_u8: Optional[np.ndarray], *, conf: float,
           allow_ids: Optional[Sequence[int]] = None,
           iou: float = 0.45, max_det: int = 100,
           want_masks: bool = True):
    """Raw heads -> (xyxy in network pixels, scores, class ids, masks).

    `heads` is one (box_u8, cls_u8, coeff_u8) triple per entry of
    `layout.scales`, in the same order. Masks are omitted entirely when
    `want_masks` is False -- they are the dominant cost once the class head is
    gated (9.1 ms for 8 detections against 0.5 ms for everything else), so a
    consumer that only wants boxes must be able to not pay for them.
    """
    allow = None if allow_ids is None else np.asarray(sorted(set(int(c) for c in allow_ids)),
                                                      dtype=np.int64)
    if allow is not None and allow.size == 0:
        allow = None

    all_box, all_score, all_cid, all_coeff = [], [], [], []
    for sl, (box_u8, cls_u8, coeff_u8) in zip(layout.scales, heads):
        floor = sl.cls.score_floor_u8(conf)
        cells, cids, score_u8 = candidates(cls_u8, floor, allow)
        if cells.size == 0:
            continue
        gw = cls_u8.shape[1]
        all_box.append(dfl_boxes(box_u8, cells, grid_w=gw, stride=sl.stride,
                                 quant=sl.box))
        all_score.append(sl.cls.dequant_score(score_u8))
        all_cid.append(cids)
        all_coeff.append(sl.coeff.dequant(coeff_u8.reshape(-1, MASK_DIM)[cells]))

    if not all_box:
        z = np.zeros((0, 4), np.float32)
        return z, np.zeros(0, np.float32), np.zeros(0, np.int64), []

    xyxy = np.concatenate(all_box).astype(np.float32)
    scores = np.concatenate(all_score).astype(np.float32)
    cids = np.concatenate(all_cid)
    keep = nms(xyxy, scores, cids, iou, max_det)
    xyxy, scores, cids = xyxy[keep], scores[keep], cids[keep]

    masks: List[np.ndarray] = []
    if want_masks and proto_u8 is not None:
        coeff = np.concatenate(all_coeff)[keep]
        masks = masks_for(coeff, proto_u8, layout.proto, xyxy, size=layout.size)
    return xyxy, scores, cids, masks
