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


def emits_raw_heads(hef_path: str) -> bool:
    """True when the HEF post-processes NOTHING and the host must decode.

    THE TEST IS THE OUTPUT COUNT, NOT THE FILE NAME. A HEF whose NMS ran
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

    ⛔ THE CREATION IS LOCKED, AND THAT IS NOT DEFENSIVE. `detector_node` builds
    a registry in a `ThreadPoolExecutor`, so two detectors are constructed
    GENUINELY CONCURRENTLY. Unlocked, both threads saw `_DEVICE is None` and
    both called `VDevice()` -- and the note above this function already records
    what a second VDevice does. Measured on the vehicle: sequential
    construction of `gate_rescue_repair` and `bin_fire_blood` gives OK/OK,
    while the same two in parallel HANG -- no exception, no core dump, the
    process simply never finishes loading. An earlier run of the same launch
    instead reported `HAILO_OUT_OF_PHYSICAL_DEVICES` and dropped one model, so
    the race has two faces and one of them looks like a model that failed to
    compile.

    Consequence, stated because it is the whole point: two models CAN be
    resident on one VDevice and take turns -- that is what the block above
    measured at 98.2 Hz -- and this race is what stopped any registry launch
    from getting there.
    """
    global _DEVICE
    if _DEVICE is not None:
        return _DEVICE
    with _DEVICE_LOCK:
        # Re-checked inside: the thread that waited here must see the device
        # the winner made, not make a second one.
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
        # Publish floor. Defaults to `conf`, i.e. OFF -- byte-for-byte the old
        # behaviour until an operator lowers it.
        self._assoc_conf = float(conf)
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
        self._blocking = bool(os.environ.get('MONGLA_HAILO_FORCE_BLOCKING'))
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
                    '[HAILO] MONGLA_HAILO_FORCE_BLOCKING is set: using the '
                    'blocking API, which HOLDS THE GIL for the whole ~10 ms '
                    'inference and stalls the camera pump and every rclpy '
                    'thread in this process. Diagnostics only.')
        else:
            self._model = self._target.create_infer_model(self._path)
            self._model.input().set_format_type(FormatType.UINT8)
            self._configure_outputs()

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
        self._pad_geometry = None
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

        # (cold ms, warm ms) of the warmup, or None: logged so the one-time setup
        # cost is a MEASURED number, not an assumed one.
        self.warmup_ms = None
        if warmup:
            # The first infer pays one-time setup. Paying it here keeps it out
            # of the first mission frame, where it reads as a dropped frame.
            import time as _t
            blank = np.zeros((self._size, self._size, 3), np.uint8)
            try:
                t0 = _t.perf_counter()
                self.infer(blank)
                t1 = _t.perf_counter()
                self.infer(blank)
                t2 = _t.perf_counter()
                self.warmup_ms = ((t1 - t0) * 1e3, (t2 - t1) * 1e3)
                if self._log:
                    self._log.info(
                        f'[HAILO] {Path(self._path).name} warmup: first infer '
                        f'{self.warmup_ms[0]:.1f} ms, then {self.warmup_ms[1]:.1f} ms')
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
        if self._assoc_conf > self._conf:
            # A publish floor above the control floor would silently discard
            # boxes the control loop is willing to use -- the opposite of the
            # intent. Follow `conf` down.
            self._assoc_conf = self._conf
        self._warn_conf(self._conf)

    def update_assoc_conf(self, assoc_conf: float) -> None:
        """Lower the PUBLISH floor below the control floor (BYTE).

        Boxes in [assoc_conf, conf) reach the tracker for association only.
        Clamped to the HEF's baked NMS floor, below which nothing exists to
        publish -- asking for less is not an error, it is simply not available,
        and silently accepting it would make the parameter look effective.
        """
        a = float(assoc_conf)
        if self._baked_conf is not None and a < self._baked_conf:
            if self._log is not None:
                self._log.warn(
                    f'[HAILO] assoc_conf={a:.3f} is below this HEF\'s baked NMS '
                    f'floor {self._baked_conf:.3f}; clamped -- nothing exists '
                    f'below the bake to publish.')
            a = self._baked_conf
        self._assoc_conf = min(a, self._conf)

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
    #  Preprocessing, straight into the bound buffer
    # ------------------------------------------------------------------ #
    def _letterbox_into_bound(self, frame_bgr: np.ndarray):
        """Letterbox `frame_bgr` DIRECTLY into the buffer the chip reads.

        Same pixels as `letterbox()`, three quarters of the work. The shipped
        path allocated a fresh 640x640x3 canvas, memset all 1.23 MB of it to
        114, resized into the middle, then copied the whole thing into the
        bound buffer -- two full-frame passes per inference for a border that
        never changes.

        The bars only depend on the SOURCE FRAME SIZE, so they are painted
        once and repainted only when that changes; `cv2.resize` then writes
        its output into the buffer's interior ROI with no intermediate at all.
        Measured on the vehicle, 810x1080 source:

            full canvas + memset + copy   2.054 ms
            ROI assign into bound buffer  1.656 ms
            cv2.resize(dst=ROI)           1.265 ms

        0.79 ms per frame, on the detection path as well as the seg one. That
        matters because after the quantised-domain decode, preprocessing is
        where the host time actually goes: 9.66 ms end to end against 6.28 ms
        of chip and 0.93 ms of decode.

        ⛔ REPAINTING THE BARS IS NOT OPTIONAL. Only the interior is written
        each frame, so a frame of a different shape would otherwise be
        surrounded by the PREVIOUS geometry's image data instead of grey --
        a border of stale pixels the detector is free to find objects in.
        """
        h, w = frame_bgr.shape[:2]
        geom = self._pad_geometry
        if geom is None or geom[0] != h or geom[1] != w:
            s = min(self._size / h, self._size / w)
            nh, nw = int(round(h * s)), int(round(w * s))
            px, py = (self._size - nw) // 2, (self._size - nh) // 2
            geom = (h, w, s, nw, nh, px, py)
            self._pad_geometry = geom
            self._in_buf[...] = 114
        _h, _w, s, nw, nh, px, py = geom
        import cv2
        cv2.resize(frame_bgr, (nw, nh), dst=self._in_buf[py:py + nh, px:px + nw],
                   interpolation=cv2.INTER_LINEAR)
        return s, px, py

    # ------------------------------------------------------------------ #
    #  Output shape -- the ONE thing a seg HEF does differently
    # ------------------------------------------------------------------ #
    # A detection HEF has a single HAILO_NMS output; a seg HEF has ten raw
    # tensors. Everything else -- the shared VDevice, the configure-once
    # activate-many dance, the eviction lock -- is identical and must stay ONE
    # copy, because that machinery is where every measured failure lived
    # (SIGSEGV on a concurrent evict, SRAM_MEMORY_FULL on a per-swap
    # configure). So the subclass overrides exactly these two.
    def _configure_outputs(self) -> None:
        # Imported HERE, not taken from __init__'s scope. Extracting this hook
        # out of __init__ left the name behind and every detection HEF raised
        # `NameError: FormatType` at construction -- caught on the vehicle, not
        # by any test, because nothing off the chip exercises this line.
        from hailo_platform import FormatType
        self._model.output().set_format_type(FormatType.FLOAT32)
        self._out_shape = tuple(self._model.output().shape)

    def _bind_outputs(self) -> None:
        self._out_buf = np.zeros(self._out_shape, np.float32)
        self._bindings.output().set_buffer(self._out_buf)

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
        # ⛔ THE CEILING IS AT LEAST THREE GROUPS, NOT TWO -- MEASURED
        # 2026-09-11, correcting what this comment used to assert. It said
        # "two configured groups are resident at once here ... and is known to
        # fit", which was true of what we ran and was never the limit. On this
        # Hailo-8, `yolov8n_seg` + `gate_rescue_repair` + `bin_fire_blood` all
        # configure together: three groups, no SRAM error. So a segmentation
        # model can live BESIDE both detectors rather than evicting one, and
        # any plan that assumed a two-group budget was solving a constraint
        # that is not there. The SRAM_MEMORY_FULL failure below is real and
        # was caused by configuring on every swap, which is a leak, not by a
        # two-group ceiling.
        #
        # Not measured, and therefore not claimed: where the ceiling actually
        # is, and what a fourth group or a multi-context model costs. The
        # HEFs differ in that too -- `yolov8n_seg` is Single Context while
        # `yolov11n_seg` needs three contexts for the same task.
        if self._cim is None:
            self._cim = self._model.configure()
            self._cim.__enter__()
            # Buffers allocated ONCE and reused. `set_buffer` binds the array,
            # so a fresh allocation per frame would be 640x640x3 of churn
            # inside the hot loop, plus a rebind.
            self._in_buf = np.zeros((self._size, self._size, 3), np.uint8)
            # A fresh buffer has no bars; force the next frame to paint them.
            self._pad_geometry = None
            self._bindings = self._cim.create_bindings()
            self._bindings.input().set_buffer(self._in_buf)
            self._bind_outputs()
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
            # The diagnostic path keeps the original canvas: it has no bound
            # buffer to write into, and it exists to be byte-comparable with
            # history rather than fast.
            buf, scale, pad_x, pad_y = letterbox(frame_bgr, self._size)
            with _DEVICE_LOCK:
                res = self._acquire_locked().infer(
                    {self._in_name: np.expand_dims(buf, 0)})
            arr = res[self._out_name]
            per_class = arr[0] if len(arr) else []
            return self._boxes_to_detections(per_class, w, h, scale,
                                             pad_x, pad_y)
        with _DEVICE_LOCK:
            cim = self._acquire_locked()
            # Letterboxed straight into the bound buffer -- the binding is set
            # up once in `_acquire_locked` and never rebound.
            scale, pad_x, pad_y = self._letterbox_into_bound(frame_bgr)
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
                # BYTE (ByteTrack): boxes between `assoc_conf` and `conf` are
                # PUBLISHED but are association-only fodder -- the tracker's
                # second stage matches them against motion predictions, and the
                # control path ignores them via `vision.ctrl_conf`. Measured on
                # real footage: presence on hard clips 8.3 -> 34.8 % with no
                # change at all on clips that already work.
                #
                # No flag is needed on the wire because THE SCORE IS THE FLAG --
                # anything below the control floor is by definition low-conf,
                # and a parallel boolean would be a second copy of that fact.
                if score < self._assoc_conf:
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
            # Highest score first, so if the cap bites it is always the
            # weakest association fodder that goes -- never a box the control
            # loop would have steered on.
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


class HailoSegDetector(HailoDetector):
    """A RAW segmentation HEF behind the same `Detector` API.

    Public Model Zoo `*_seg.hef` files carry no NMS and no post-process at
    all: ten uint8 tensors come off the chip and the host does the whole
    decode. `seg_decode` is that decode and holds the reasoning; this class is
    only the wiring -- which output is which, and the un-letterboxing.

    MEASURED ON THE VEHICLE, 2026-09-11 (`yolov8n_seg`, 640x640, Hailo-8):

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

        ⛔ UINT8, NOT FLOAT32, AND THAT IS THE WHOLE OPTIMISATION. Asking
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
        with _DEVICE_LOCK:
            cim = self._acquire_locked()
            scale, pad_x, pad_y = self._letterbox_into_bound(frame_bgr)
            cim.wait_for_async_ready(timeout_ms=_ASYNC_READY_MS)
            cim.run_async([self._bindings]).wait(_ASYNC_WAIT_MS)
            # Decode INSIDE the lock, unlike the detection path, because the
            # output buffers stay bound and a second inference on this
            # detector would overwrite them mid-decode. The chip is idle
            # meanwhile; the alternative is copying 1.6 MB per frame.
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
