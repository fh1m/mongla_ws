"""Hailo-8 detector backend: a compiled ``.hef`` behind the same `Detector` API.

Replaces the Jetson's TensorRT path on the Raspberry Pi 5 + AI HAT+. Round 29
measured this pipeline at **98.0 Hz** on `gate_rescue_repair` against a
`hailortcli --hw-only` benchmark of **97.9 FPS** on the same HEF -- so the host
code is at 100 % of the chip and there is no preprocessing overhead left to
recover. The full campaign, including why the ceiling is the model's CONTEXT
COUNT and why async inference buys exactly nothing here, is in
`.claude/context/hailo-vision.md`.

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

3. **Run at conf 0.10-0.15, not the CUDA path's 0.45.**

   MEASURED ON REAL COMPETITION FOOTAGE (2026-09-04), which the round-24
   figure below was not -- that one was a score DISTRIBUTION with the camera
   pointed at a room. 1830 frames of the RoboSub 2025 gate approach, through
   the model trained on that footage, tracker clamped to follow the floor:

       floor   presence   boxes/frame   multi-box frames   centre spread
       0.25      17.0 %       1.08            7.6 %           0.0351
       0.15      45.1 %       1.12           11.3 %           0.0493
       0.10      53.6 %       1.20           18.0 %           0.0483
       0.05      56.1 %       1.45           35.4 %           0.0522
       0.02        --         1.86           58.4 %           0.0563

   0.15 -> 0.10 buys 8.5 points of presence and the extra boxes are the SAME
   target: the centre spread does not grow (0.0493 -> 0.0483) and jitter
   actually falls (p95 0.0055 -> 0.0052). Below 0.10 it inverts -- at 0.05,
   35 % of frames carry more than one box and jitter jumps 53 %, which on a
   vision-servoed hull is steering at the wrong thing.

   **0.10 is the floor this evidence supports.** Not shipped as the default
   because it has not been through water with a live control loop; set it
   deliberately (`conf:=0.10`) and watch `lock_on` behaviour.

4. **The round-24 note, kept because it is still the INT8 story.** Round 24 measured INT8
   costing ~0.08 of confidence at the 0.20 operating point while NOT moving the
   box centre (2.14-2.65 px against a 2.72 px fp32-vs-fp32 noise floor). The
   detections are there, they score lower.

5. **NMS runs on the HOST, inside HailoRT.** The DFC puts the YOLOv8 head --
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
import re
import subprocess
import threading
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import numpy as np

from .detector import Detection, Detector

_INPUT_FALLBACK = 640

# Async submit/wait budgets. Generous: they are a DEADLOCK guard, not a
# latency knob -- an inference that takes 1 s has already broken the mission,
# and a tight bound here would turn a slow frame into an exception.
_ASYNC_READY_MS = 1000
_ASYNC_WAIT_MS = 1000

# How many activation swaps before saying so. One is the normal handover from
# a camera switch; a stream of them means two detectors are competing.
_SWAP_WARN_AT = 20

# The operating point this backend is compiled FOR. Round 24 measured INT8
# costing ~0.08 of confidence at the 0.20 point while NOT moving the box centre
# (2.5 px against a 2.72 px fp32-vs-fp32 noise floor), so the detections are
# there and score lower. Every launch path ships 0.35-0.45, which is the CUDA
# number.
_INT8_OPERATING_POINT_MIN = 0.12
_INT8_OPERATING_POINT_MAX = 0.15


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


# --------------------------------------------------------------------------- #
#  ONE DEVICE PER PROCESS, ONE ACTIVE GRAPH AT A TIME                         #
# --------------------------------------------------------------------------- #
# Measured on this hardware, because all three options look plausible on paper
# and two of them do not work:
#
#   two PROCESSES, a VDevice each   -> HAILO_OUT_OF_PHYSICAL_DEVICES (74) on the
#                                      second. This is what `vision_dual` does
#                                      today, so the dual-camera launch has
#                                      never been able to start.
#   multi_process_service + scheduler -> HailoRTInvalidOperationException. The
#                                      API accepts the flag; the `hailort_service`
#                                      daemon it needs is not installed.
#   ROUND_ROBIN scheduler, one process -> SIGSEGV. Not an exception, a core dump.
#   ONE process, one VDevice, both HEFs configured, activation handed back and
#   forth                            -> WORKS. 98.2 Hz on the live graph.
#
# So two detectors must live in ONE process and take turns. The turn-taking is
# cheap: swapping costs 4.15 ms median against a 10.18 ms frame, and swapping on
# EVERY frame still gives 73.8 Hz across both cameras (36.9 Hz each). With the
# mission model -- one camera live, the other paused -- there are no swaps at
# all. The tail is the thing to know about: one swap in 295 took 42.69 ms.
_DEVICE = None
_DEVICE_LOCK = threading.Lock()
_ACTIVE: Optional['HailoDetector'] = None


def _shared_device():
    """The process's single VDevice. Created on first use, never released.

    Not released on close() either: releasing it while a second detector still
    holds a network group configured on it is how you turn a clean shutdown
    into a segfault, and the process is about to exit anyway.
    """
    global _DEVICE
    if _DEVICE is None:
        from hailo_platform import VDevice
        _DEVICE = VDevice()
    return _DEVICE


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

        from hailo_platform import HEF, FormatType

        self._hef = HEF(self._path)
        in_info = self._hef.get_input_vstream_infos()[0]
        out_info = self._hef.get_output_vstream_infos()[0]
        self._in_name, self._out_name = in_info.name, out_info.name
        shape = tuple(in_info.shape)
        self._size = int(shape[0]) if len(shape) >= 2 else _INPUT_FALLBACK

        self._target = _shared_device()

        # THE ASYNC API, AND THE REASON IS THE GIL, NOT SPEED.
        #
        # The blocking `InferVStreams.infer()` HOLDS THE GIL for its entire
        # ~10 ms. Measured against two controls that separate cleanly
        # (`tools/gil_probe.py`), another Python thread in the process woke:
        #
        #     control: time.sleep (releases)     0.05 ms late,  0.0 % over 4 ms
        #     control: pure-Python spin (holds)  5.11 ms late, 99.9 %
        #     InferVStreams.infer()              9.21 ms late, 99.6 %
        #     InferModel run_async + wait        0.05 ms late,  0.0 %
        #
        # It is WORSE than the spin control, because the spin at least yields
        # on CPython's 5 ms switch interval and a C call that never releases
        # never yields at all. So during every inference the camera's capture
        # pump could not publish a newer frame and no rclpy executor thread
        # could run -- 70+ times a second.
        #
        # That was measured end to end before it was explained: a composed
        # camera+detector process showed a frame WAITING 13.82 ms in the
        # camera's slot, of which only 1.93 ms was our decode, against a
        # 12.16 ms loop period and 10.22 ms of inference.
        #
        # Round 29 measured this API and recorded "async inference buys
        # exactly nothing". That was true and it was a THROUGHPUT measurement
        # -- 98.2 Hz async against 98.0 blocking, no gain. Nobody asked
        # whether it releases the GIL, which is the question that mattered.
        # The conclusion is not overturned, it was answering a different one.
        # The blocking path is KEPT, behind an env flag, for two reasons: it
        # is what `tools/hailo_api_equivalence.py` compares against to prove
        # the switch moved no pixels, and it is the fallback if a HailoRT
        # version ever regresses the async path on the vehicle. It is not a
        # tuning knob and nothing sets it in normal operation.
        self._blocking = bool(os.environ.get('DUBURI_HAILO_FORCE_BLOCKING'))
        if self._blocking:
            from hailo_platform import (HailoStreamInterface, InferVStreams,
                                        ConfigureParams, InputVStreamParams,
                                        OutputVStreamParams)
            cfg = ConfigureParams.create_from_hef(
                self._hef, interface=HailoStreamInterface.PCIe)
            self._ng = self._target.configure(self._hef, cfg)[0]
            self._ngp = self._ng.create_params()
            self._ivp = InputVStreamParams.make(
                self._ng, format_type=FormatType.UINT8)
            self._ovp = OutputVStreamParams.make(
                self._ng, format_type=FormatType.FLOAT32)
            self._InferVStreams = InferVStreams
            self._model = None
            self._out_shape = ()
            if self._log:
                self._log.warn(
                    '[HAILO] DUBURI_HAILO_FORCE_BLOCKING is set: using the '
                    'blocking API, which HOLDS THE GIL for the whole ~10 ms '
                    'inference and stalls the camera pump and every rclpy '
                    'thread in this process. Diagnostics only.')
        else:
            self._model = self._target.create_infer_model(self._path)
            self._model.input().set_format_type(FormatType.UINT8)
            self._model.output().set_format_type(FormatType.FLOAT32)
            self._out_shape = tuple(self._model.output().shape)

        # ACTIVATION IS DEFERRED, and that is the whole point of this class
        # holding a shared device. Activating here would mean the SECOND
        # detector constructed in the process fails at construction -- exactly
        # the failure this replaces, moved one layer down. It is taken on the
        # first infer and handed over when the other detector needs it.
        self._cim = None            # ConfiguredInferModel, once activated
        self._bindings = None
        self._in_buf = None
        self._out_buf = None
        self._swaps = 0
        self._pipe = None
        self._activation = None
        self._nms_classes = len(self._names)
        self._nms_warned = False
        self._ready = True

        self._baked_conf = baked_score_threshold(self._path)

        self._allow_ids: Optional[set] = None
        self.update_allowlist(class_allowlist)

        if self._log:
            baked = ('?' if self._baked_conf is None
                     else f'{self._baked_conf:.3f}')
            self._log.info(
                f'[HAILO] {Path(self._path).name} in={self._size}x{self._size} '
                f'classes={len(self._names)} conf={self._conf:.2f} '
                f'baked={baked}')
        self._warn_conf(self._conf)

        if warmup:
            # The first infer pays one-time setup. Paying it here keeps it out
            # of the first mission frame, where it reads as a dropped frame.
            blank = np.zeros((self._size, self._size, 3), np.uint8)
            try:
                self.infer(blank)
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
        self._warn_conf(self._conf)

    def _warn_conf(self, conf: float) -> None:
        """Say when the runtime threshold cannot do what it was asked to.

        The class docstring has promised this log since the backend was
        written and it did not exist, so a mission could believe it had lowered
        the bar and be silently overruled by a graph compiled hours earlier on
        an x86 box.
        """
        if self._log is None or self._baked_conf is None:
            return
        if conf < self._baked_conf - 1e-6:
            self._log.warn(
                f'[HAILO] conf={conf:.3f} is BELOW this HEF\'s baked NMS floor '
                f'of {self._baked_conf:.3f} -- everything under the floor was '
                f'discarded on-chip and no runtime value brings it back. The '
                f'effective threshold is {self._baked_conf:.3f}. Recompile the '
                f'model if you need lower.')
        elif conf > _INT8_OPERATING_POINT_MAX and self._baked_conf <= 0.1:
            # Not an error -- a mission may want a tight gate -- but this is the
            # single most likely cause of "the Hailo model misses things": INT8
            # costs ~0.08 of score without moving the box, so a CUDA-path 0.45
            # is roughly three times the intended operating point here.
            self._log.warn(
                f'[HAILO] conf={conf:.3f} on an INT8 graph. INT8 costs ~0.08 of '
                f'score without moving the box centre, so the CUDA path\'s '
                f'0.35-0.45 is ~3x the intended operating point. This model is '
                f'baked at {self._baked_conf:.3f} precisely so '
                f'{_INT8_OPERATING_POINT_MIN:.2f}-{_INT8_OPERATING_POINT_MAX:.2f} '
                f'is available.')

    def update_max_det(self, max_det: int) -> None:
        self._max_det = int(max_det)

    def class_names(self) -> Dict[int, str]:
        return dict(self._names)

    def is_ready(self) -> bool:
        return bool(self._ready)

    # ------------------------------------------------------------------ #
    #  Inference
    # ------------------------------------------------------------------ #
    def _acquire_locked(self):
        """Take the chip's single activation, evicting whoever holds it.

        THE CALLER MUST HOLD `_DEVICE_LOCK` FOR THE INFER TOO, not just for
        this. Guarding only the handover was measured as a SIGSEGV on the Pi:
        detector A sat inside `pipe.infer()` while detector B evicted its
        activation on another thread -- a use-after-free on the stream, and the
        stack trace showed exactly that (one thread in `pyhailort.infer`, the
        other in `activate().__enter__`). Two detector nodes in one process are
        two rclpy callbacks and a MultiThreadedExecutor puts them on different
        threads, so this is the normal case, not a corner.

        Serialising the infer costs nothing real: the chip runs one graph at a
        time regardless, and the measured both-cameras figure (69-74 Hz total)
        was taken serialised.
        """
        global _ACTIVE
        if _ACTIVE is self and self._cim is not None:
            return self._cim
        if _ACTIVE is not None and _ACTIVE is not self:
            _ACTIVE._release_locked()
        # `configure()` builds the ConfiguredInferModel; it does NOT activate
        # it. Skipping `activate()` raises HAILO_STREAM_NOT_ACTIVATED (72) at
        # the first `run_async`, which reads like an API-mixing problem and is
        # really a missing call. (It is invalid with the HailoRT scheduler
        # enabled -- and enabling the scheduler with two graphs SIGSEGVs, so
        # taking turns by hand is the only arrangement that works here.)
        if self._blocking:
            self._activation = self._ng.activate(self._ngp)
            self._activation.__enter__()
            self._pipe = self._InferVStreams(self._ng, self._ivp, self._ovp)
            self._pipe.__enter__()
            _ACTIVE = self
            self._swaps += 1
            return self._pipe
        # CONFIGURE ONCE, ACTIVATE MANY. These are NOT the same cost and
        # conflating them fills the chip.
        #
        # `configure()` allocates the network group into the Hailo-8's
        # ON-CHIP SRAM. `activate()` merely makes an already-resident group
        # the running one. Calling configure on every swap -- which is what
        # this did first -- allocates a fresh group each time and the old ones
        # are not reclaimed fast enough, so after a few dozen camera switches
        # the firmware answers:
        #
        #     CONTEXT_SWITCH_STATUS_SRAM_MEMORY_FULL
        #     HAILO_OUT_OF_FW_MEMORY (71)
        #
        # ...and then EVERY inference fails, forever, in a tight loop: 98 %
        # CPU, no detections, and an image topic starved to 1.6 Hz. It was
        # invisible in the single-detector profiler because that never swaps.
        #
        # Two configured groups are resident at once here, which is exactly
        # what the blocking path did (one `VDevice.configure` per detector in
        # __init__) and is known to fit.
        if self._cim is None:
            self._cim = self._model.configure()
            self._cim.__enter__()
            # Buffers allocated ONCE and reused. `set_buffer` binds the array,
            # so a fresh allocation per frame would be 640x640x3 of churn
            # inside the hot loop, plus a rebind.
            self._in_buf = np.zeros((self._size, self._size, 3), np.uint8)
            self._out_buf = np.zeros(self._out_shape, np.float32)
            self._bindings = self._cim.create_bindings()
            self._bindings.input().set_buffer(self._in_buf)
            self._bindings.output().set_buffer(self._out_buf)
        self._cim.activate()
        _ACTIVE = self
        self._swaps += 1
        if self._swaps == _SWAP_WARN_AT and self._log:
            # Not an error -- the mission model is one camera live -- but
            # two UNPAUSED detectors thrash the activation, and 73.8 Hz
            # across both cameras reads as "the chip got slower" unless
            # something says why.
            self._log.warn(
                f'[HAILO] {Path(self._path).name} has taken the activation '
                f'{self._swaps} times -- another detector is competing for '
                f'the chip. Each swap costs ~4 ms; pause the camera you are '
                f'not steering on.')
        return self._cim

    def _release_locked(self) -> None:
        """Give up the activation. Caller holds `_DEVICE_LOCK`."""
        for obj in (getattr(self, '_pipe', None),
                    getattr(self, '_activation', None)):
            try:
                if obj is not None:
                    obj.__exit__(None, None, None)
            except Exception:
                pass
        self._pipe = self._activation = None
        # DEACTIVATE ONLY. The ConfiguredInferModel stays -- it is the SRAM
        # allocation, and tearing it down on every swap is what filled the
        # chip. It is released in `close()`, when the detector is done.
        if self._cim is not None:
            try:
                self._cim.deactivate()
            except Exception:
                pass

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        if not self._ready or frame_bgr is None:
            return []
        h, w = frame_bgr.shape[:2]
        buf, scale, pad_x, pad_y = letterbox(frame_bgr, self._size)
        # The lock spans acquire AND infer. See `_acquire_locked`.
        #
        # It still spans the wait, and that is deliberate: the chip runs one
        # graph at a time whatever we do, and releasing the lock across the
        # wait would let the other detector evict this activation mid-flight
        # -- the use-after-free that was a measured SIGSEGV here.
        #
        # THE GIL IS A DIFFERENT LOCK AND `job.wait()` RELEASES IT. That is
        # the entire reason this path exists: another Python thread -- the
        # camera's capture pump, an rclpy executor -- runs freely during the
        # ~10 ms this is waiting, where the blocking API froze all of them.
        if self._blocking:
            with _DEVICE_LOCK:
                res = self._acquire_locked().infer(
                    {self._in_name: np.expand_dims(buf, 0)})
            arr = res[self._out_name]
            per_class = arr[0] if len(arr) else []
            return self._boxes_to_detections(per_class, w, h, scale,
                                             pad_x, pad_y)
        with _DEVICE_LOCK:
            cim = self._acquire_locked()
            # Copy into the bound buffer rather than rebinding a new array:
            # the binding is set up once in `_acquire_locked`.
            self._in_buf[...] = buf
            cim.wait_for_async_ready(timeout_ms=_ASYNC_READY_MS)
            job = cim.run_async([self._bindings])
            job.wait(_ASYNC_WAIT_MS)
            raw = self._out_buf
        # THE ASYNC PATH HANDS BACK A FLAT BUFFER, AND THIS IS THE ONE PLACE
        # THE TWO APIs GENUINELY DIFFER.
        #
        # `InferVStreams` returned a dict keyed by vstream name holding a
        # ragged per-class object array. `InferModel` writes into the buffer we
        # bound, and for a HAILO_NMS output that buffer is FLAT float32:
        #
        #     [ count_0, (y1 x1 y2 x2 score) * MAX, count_1, ... ]
        #
        # with a FIXED per-class stride, so class 1 starts at a constant
        # offset whatever class 0 detected. Measured on this hardware:
        # shape (1503,) for a 3-class model = 3 * (1 + 100 * 5).
        #
        # Getting this wrong does not raise -- it reads scores as coordinates
        # and hands the control loop a box that tracks nothing. `_nms_stride`
        # therefore VALIDATES the arithmetic and says so rather than guessing.
        per_class = self._decode_nms(raw)
        return self._boxes_to_detections(per_class, w, h, scale, pad_x, pad_y)

    def _boxes_to_detections(self, per_class, w, h, scale, pad_x, pad_y):
        """Per-class boxes -> Detections. ONE copy, shared by both APIs.

        The blocking and async paths differ ONLY in how the buffer arrives.
        Two copies of this arithmetic is how they would come to disagree about
        where a target is, and `tools/hailo_api_equivalence.py` would then be
        comparing two different box maths rather than two transports.
        """
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

    def _decode_nms(self, raw) -> list:
        """Flat HAILO_NMS buffer -> a list of (N, 5) arrays, one per class.

        THE LAYOUT IS PACKED, NOT FIXED-STRIDE, AND THE DIFFERENCE IS SILENT.

            [ count_0, (y1 x1 y2 x2 score) * count_0,
              count_1, (y1 x1 y2 x2 score) * count_1, ... ]

        Each class's boxes follow its own count immediately; the next count
        sits right after them. The buffer is SIZED for the worst case --
        measured (1503,) for a 3-class model, i.e. 3 * (1 + 100 * 5) -- which
        makes a fixed per-class stride look plausible and arithmetically
        perfect. It is wrong: reading class 2 at a constant offset lands in
        the tail padding, finds a count of 0, and silently returns NOTHING for
        that class.

        Caught only because `tools/hailo_api_equivalence.py` ran both APIs
        over the same frames: 56 detections blocking, 22 async, with every
        surviving box IDENTICAL -- boxes being lost, not moved. A smoke test
        would have passed; so would any check that only looked at class 0.

        Returns the same per-class shape the box maths already expects, so
        only the transport changed.
        """
        ncls = self._nms_classes
        n_floats = int(raw.size)
        out = []
        i = 0
        for _ in range(ncls):
            if i >= n_floats:
                # Ran out of buffer: the remaining classes simply had no
                # detections and the tail is padding.
                out.append(())
                continue
            n = int(raw[i])
            i += 1
            # A count that cannot fit in what is left is corruption, not a
            # big detection list. Stop rather than reinterpret padding as
            # boxes -- a fabricated box steers the vehicle.
            if n < 0 or i + n * 5 > n_floats:
                if not self._nms_warned:
                    self._nms_warned = True
                    if self._log:
                        self._log.error(
                            f'[HAILO] NMS buffer claims {n} boxes with '
                            f'{n_floats - i} floats left -- refusing to '
                            f'decode past the end. Detections will be short '
                            f'rather than invented.')
                out.append(())
                break
            out.append(raw[i:i + n * 5].reshape(n, 5) if n else ())
            i += n * 5
        while len(out) < ncls:
            out.append(())
        return out

    def close(self) -> None:
        """Drop this detector's activation. The DEVICE stays.

        Releasing the shared VDevice here would pull it out from under a second
        detector that still has a network group configured on it -- a segfault
        during shutdown, which is the hardest kind to read. It is process-wide
        and the process is exiting.
        """
        global _ACTIVE
        self._ready = False
        with _DEVICE_LOCK:
            self._release_locked()
            # The one place the SRAM allocation is actually handed back.
            if self._cim is not None:
                try:
                    self._cim.__exit__(None, None, None)
                except Exception:
                    pass
                self._cim = self._bindings = None
                self._in_buf = self._out_buf = None
            if _ACTIVE is self:
                _ACTIVE = None

    def __repr__(self) -> str:
        return (f'<HailoDetector {Path(self._path).name} '
                f'{self._size}x{self._size} classes={len(self._names)}>')
