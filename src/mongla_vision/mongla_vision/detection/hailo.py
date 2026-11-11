"""Hailo-8 detector backend: a compiled ``.hef`` behind the same `Detector` API.

The vehicle's detector, on the Raspberry Pi 5 + AI HAT+. Round 29
measured this pipeline at **98.0 Hz** on `gate_sharks` against a
`hailortcli --hw-only` benchmark of **97.9 FPS** on the same HEF -- so the host
code is at 100 % of the chip and there is no preprocessing overhead left to
recover. The full campaign, including why the ceiling is the model's CONTEXT
COUNT and why async inference buys exactly nothing here, is in
`.claude/context/perception/hailo-vision.md`.

Four things differ from a `.pt`, and each is silent if missed
---------------------------------------------------------------------
1. **The HEF carries no class names.** A `.pt` has `model.names`; a `.hef` has
   nothing. The `<stem>.yaml` sidecar therefore stops being advisory -- without
   it there is no id->label map, the allowlist matches nothing, and the
   detector returns `[]` every frame while looking perfectly healthy. This
   class REFUSES to construct without one rather than run blind.

2. **The compiled score threshold is a default, and this class lowers it.**
   A HEF carries the NMS score threshold it was compiled with, and a stock
   Model Zoo graph carries 0.200. The NMS of these graphs runs in libhailort
   on the host, and `InferModel.output().set_nms_score_threshold` replaces
   the value when the model is configured. So a graph compiled above
   `EMIT_FLOOR` is configured at `EMIT_FLOOR` here, and `conf` is a runtime
   decision on every model (measured-bars 79: 323 boxes returned below a
   compiled 0.200, every box at or above it unchanged, +0.16 % time). The
   value cannot change after the configure, so `_floor` is what the host
   receives for the life of the detector, and `update_conf()` filters above
   it. Our own graphs are compiled at 0.05, which is the same floor.

3. **Run at conf 0.10-0.15, not the .pt path's 0.45.**

   Measured on real competition footage (2026-09-04), which the round-24
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
import threading
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import numpy as np

from . import hailo_device
from .detector import Detection, Detector
from .hef_file import baked_score_threshold, load_class_index, measured_bars

_INPUT_FALLBACK = 640

# The NMS score threshold every detection graph is configured at, whatever it
# was compiled with. It is the value our own graphs are compiled at, so one
# floor holds for every model; lowering a stock graph from 0.200 to it cost
# +0.16 % time and moved no box at or above 0.200 (measured-bars 79).
EMIT_FLOOR = 0.05

# Async submit/wait budgets. Generous: they are a DEADLOCK guard, not a
# latency knob -- an inference that takes 1 s has already broken the mission,
# and a tight bound here would turn a slow frame into an exception.
_ASYNC_READY_MS = 1000
_ASYNC_WAIT_MS = 1000


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
    # BGR -> RGB. The model was trained on RGB and cv2 hands us BGR. Without
    # this the chip answers on colour-swapped frames and nothing raises: the
    # .pt fed the same swap turns `gate=0.78` into `repair=0.92 @0,0`.
    # ultralytics converts internally, so the .pt path never needed it.
    # The diagnosis, including the two wrong ones first: BUGS B-61.
    out[py:py + nh, px:px + nw] = cv2.cvtColor(
        cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR),
        cv2.COLOR_BGR2RGB)
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
        names = load_class_index(self._path)
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

        self._target = hailo_device.shared_device()
        hailo_device.register(self)

        # The async API, and the reason is the GIL, not speed.
        #
        # The blocking `InferVStreams.infer()` HOLDS THE GIL for its entire
        # ~10 ms. Measured against two controls that separate cleanly
        # (`tools/bench/gil_probe.py`), another Python thread in the process woke:
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
        # is what `tools/bench/hailo_api_equivalence.py` compares against to prove
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

        # Configured on first use, never activated: the scheduler owns
        # activation. `_infer_lock` guards only THIS detector's bound buffers
        # (a second infer on the same detector would overwrite them); it is
        # never shared, so it never makes another model wait.
        self._cim = None            # ConfiguredInferModel, once configured
        self._bindings = None
        self._in_buf = None
        self._out_buf = None
        self._pipe = None
        self._infer_lock = threading.Lock()
        self._pad_geometry = None
        self._nms_classes = len(self._names)
        self._nms_warned = False
        self._ready = True

        self._baked_conf = baked_score_threshold(self._path)
        self._floor = self._lower_floor()

        self._allow_ids: Optional[set] = None
        self.update_allowlist(class_allowlist)

        if self._log:
            baked = ('?' if self._baked_conf is None
                     else f'{self._baked_conf:.3f}')
            floor = '?' if self._floor is None else f'{self._floor:.3f}'
            self._log.info(
                f'[HAILO] {Path(self._path).name} in={self._size}x{self._size} '
                f'classes={len(self._names)} conf={self._conf:.2f} '
                f'baked={baked} floor={floor}')
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
        # Not none of them. If every requested name is unknown, `ids` is
        # EMPTY -- and an empty allowlist filters every detection away, so the
        # detector returns [] on every frame while the graph looks healthy.
        # That is the silent-[] failure this codebase already has a rule
        # against (`CLAUDE.md` §4, the missing-sidecar case), reached from a
        # different direction: one typo'd class name and the whole pipeline
        # goes quiet with a single WARN.
        #
        # Refusing to narrow is the safe direction: the caller gets everything
        # the model found and can filter downstream, instead of getting
        # nothing and believing the water is empty.
        if wanted and not ids:
            if self._log:
                self._log.error(
                    f'[HAILO] NONE of {sorted(wanted)} exist in this model -- '
                    f'ignoring the allowlist rather than filtering every '
                    f'detection away. Fix the class name, or the pipeline is '
                    f'blind while looking healthy.')
            self._allow_ids = None
            return
        self._allow_ids = ids

    def _lower_floor(self) -> Optional[float]:
        """Configure the NMS at `EMIT_FLOOR` when the graph was compiled above it.

        Returns the score below which the host receives nothing: `EMIT_FLOOR`
        when the lowering took, otherwise the compiled value. Called before
        the first configure, because the value is read then and not after.

        The compiled value stays when it cannot be read (a segmentation graph
        has no NMS output and reports none), when it is already at or below
        `EMIT_FLOOR`, and on the blocking diagnostic path, which builds its
        streams another way. A refusal from HailoRT is logged and leaves the
        compiled floor, so the detector reports the floor it really has.
        """
        baked = self._baked_conf
        if self._blocking or baked is None or baked <= EMIT_FLOOR:
            return baked
        try:
            self._model.output().set_nms_score_threshold(EMIT_FLOOR)
        except Exception as exc:                            # noqa: BLE001
            if self._log is not None:
                self._log.warn(
                    f'[HAILO] could not lower the NMS score threshold from '
                    f'the compiled {baked:.3f} to {EMIT_FLOOR:.3f} ({exc!r}); '
                    f'nothing below {baked:.3f} will be returned.')
            return baked
        return EMIT_FLOOR

    def update_conf(self, conf: float) -> None:
        """Runtime confidence. Filters what the NMS returned.

        `_floor` is the lowest score the host receives, fixed when the model
        was configured. Ask for less and you get the floor, so `_warn_conf`
        says so instead of letting a mission believe it lowered the bar.
        """
        self._conf = float(conf)
        if self._assoc_conf > self._conf:
            # A publish floor above the control floor would silently discard
            # boxes the control loop is willing to use -- the opposite of the
            # intent. Follow `conf` down.
            self._assoc_conf = self._conf
        self._warn_conf(self._conf)

    def acting_conf(self) -> float:
        return self._conf

    def update_assoc_conf(self, assoc_conf: float) -> None:
        """Lower the PUBLISH floor below the control floor (BYTE).

        Boxes in [assoc_conf, conf) reach the tracker for association only.
        Clamped to `_floor`, below which nothing exists to publish -- asking
        for less is not an error, it is simply not available, and silently
        accepting it would make the parameter look effective.
        """
        a = float(assoc_conf)
        if self._floor is not None and a < self._floor:
            if self._log is not None:
                self._log.warn(
                    f'[HAILO] assoc_conf={a:.3f} is below the NMS floor '
                    f'{self._floor:.3f} this model is configured at; clamped '
                    f'-- nothing exists below it to publish.')
            a = self._floor
        self._assoc_conf = min(a, self._conf)

    def _warn_conf(self, conf: float) -> None:
        """Say when the runtime threshold cannot do what it was asked to."""
        if self._log is None:
            return
        if self._floor is not None and conf < self._floor - 1e-6:
            self._log.warn(
                f'[HAILO] conf={conf:.3f} is BELOW the NMS floor of '
                f'{self._floor:.3f} this model is configured at -- nothing '
                f'under the floor is returned, so the effective threshold is '
                f'{self._floor:.3f}. Lower EMIT_FLOOR (hailo.py) if a model '
                f'needs less.')
        self._warn_against_measured_bar(conf)

    def _warn_against_measured_bar(self, conf: float) -> None:
        """Compare `conf` with the bar the model's own sidecar measured.

        The bar is where empty water stops reading as the prop, so it belongs
        to the weights and is read from beside them, never from a constant
        here. This replaced a warning that called 0.45 "three times the
        intended operating point" on the strength of an INT8 cost that was
        later retracted (measured-bars section 1: 0.04-0.06, not 0.08) and
        that fired on every launch of the bar gate_sharks measured.

        Below the bar, the detector publishes boxes on empty water. Above it,
        it throws away detections it was right about. Neither is an error --
        a mission may want either -- but each is said once, with the number.
        """
        if self._log is None:
            return
        wanted = None if self._allow_ids is None else {
            self._names[i].lower() for i in self._allow_ids}
        for cls, bar in measured_bars(self._path).items():
            if wanted is not None and cls.lower() not in wanted:
                continue
            if bar is None:
                self._log.warn(
                    f'[HAILO] {cls}: this model was measured and found '
                    f'unable to tell {cls} from empty water at any bar -- '
                    f'its {cls} detections are not evidence.')
            elif conf < bar - 1e-6:
                self._log.warn(
                    f'[HAILO] conf={conf:.3f} is under {cls}\'s measured bar '
                    f'{bar:.2f}; below it this model reports {cls} on empty '
                    f'water. Lower the association floor (assoc_conf) instead, '
                    f'which keeps a target without acquiring a false one.')
            elif conf > bar + 1e-6:
                self._log.warn(
                    f'[HAILO] conf={conf:.3f} is over {cls}\'s measured bar '
                    f'{bar:.2f}; detections between the two are real and are '
                    f'being discarded.')

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

        Repainting the bars is not optional. Only the interior is written
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
        roi = self._in_buf[py:py + nh, px:px + nw]
        cv2.resize(frame_bgr, (nw, nh), dst=roi,
                   interpolation=cv2.INTER_LINEAR)
        # BGR -> RGB, in place (B-61). Same defect as `letterbox`, and this
        # is the path that actually flies -- the async one. Converting with
        # `dst=roi` keeps the zero-copy write the 1.265 ms measurement above
        # depends on, so the fix costs a colour swap on the resized ROI rather
        # than a second full-canvas allocation.
        cv2.cvtColor(roi, cv2.COLOR_BGR2RGB, dst=roi)
        return s, px, py

    # ------------------------------------------------------------------ #
    #  Output shape -- the ONE thing a seg HEF does differently
    # ------------------------------------------------------------------ #
    # A detection HEF has a single HAILO_NMS output; a seg HEF has ten raw
    # tensors. Everything else -- the shared VDevice, configure-once, the
    # buffers -- is identical and must stay ONE copy (SRAM_MEMORY_FULL was a
    # per-swap configure). So the subclass overrides exactly these two.
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
    def _configured(self):
        """This detector's network group, configured on first use. NOT activated.

        Caller holds `self._infer_lock`. `hailo_device.DEVICE_LOCK` is taken for the
        configure only -- configuration touches the device -- and never for an
        inference.

        CONFIGURE ONCE. `configure()` allocates the group into the Hailo-8's
        on-chip SRAM; doing it per frame filled the chip
        (CONTEXT_SWITCH_STATUS_SRAM_MEMORY_FULL, HAILO_OUT_OF_FW_MEMORY(71),
        then every inference failing forever at 98 % CPU). Four groups -- two
        detectors and two XFeat anchors -- were measured resident together.

        NO `activate()`. With the scheduler enabled it is invalid, and the
        scheduler switches groups itself.
        """
        if self._blocking:
            if self._pipe is None:
                with hailo_device.DEVICE_LOCK:
                    self._pipe = self._InferVStreams(self._ng, self._ivp,
                                                     self._ovp)
                    self._pipe.__enter__()
            return self._pipe
        if self._cim is None:
            with hailo_device.DEVICE_LOCK:
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
        return self._cim

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        if not self._ready or frame_bgr is None:
            return []
        h, w = frame_bgr.shape[:2]
        # `job.wait()` releases the GIL: the camera pump and the executor run
        # during the ~10 ms this waits, where the blocking API froze them.
        # And no lock but this detector's own is held, so another model's
        # inference proceeds on the chip whatever this thread is doing.
        if self._blocking:
            # The diagnostic path keeps the original canvas: it has no bound
            # buffer to write into, and it exists to be byte-comparable with
            # history rather than fast.
            buf, scale, pad_x, pad_y = letterbox(frame_bgr, self._size)
            with self._infer_lock:
                res = self._configured().infer(
                    {self._in_name: np.expand_dims(buf, 0)})
            arr = res[self._out_name]
            per_class = arr[0] if len(arr) else []
            return self._boxes_to_detections(per_class, w, h, scale,
                                             pad_x, pad_y)
        with self._infer_lock:
            cim = self._configured()
            # Letterboxed straight into the bound buffer -- the binding is set
            # up once in `_configured` and never rebound.
            scale, pad_x, pad_y = self._letterbox_into_bound(frame_bgr)
            cim.wait_for_async_ready(timeout_ms=_ASYNC_READY_MS)
            job = cim.run_async([self._bindings])
            job.wait(_ASYNC_WAIT_MS)
            # The async path hands back a flat buffer, and this is the one
            # Place the two APIs GENUINELY DIFFER.
            #
            # `InferVStreams` returned a dict keyed by vstream name holding a
            # ragged per-class object array. `InferModel` writes into the
            # buffer we bound, and for a HAILO_NMS output that buffer is FLAT
            # float32:
            #
            #     [ count_0, (y1 x1 y2 x2 score) * MAX, count_1, ... ]
            #
            # with a FIXED per-class stride (measured (1503,) for a 3-class
            # model = 3 * (1 + 100 * 5)). Getting this wrong does not raise --
            # it reads scores as coordinates. `_decode_nms` VALIDATES it.
            # Decoded under this detector's lock: the buffer stays bound, and
            # the next infer on this detector would overwrite it mid-read.
            per_class = self._decode_nms(self._out_buf)
        return self._boxes_to_detections(per_class, w, h, scale, pad_x, pad_y)

    def _boxes_to_detections(self, per_class, w, h, scale, pad_x, pad_y):
        """Per-class boxes -> Detections. ONE copy, shared by both APIs.

        The blocking and async paths differ ONLY in how the buffer arrives.
        Two copies of this arithmetic is how they would come to disagree about
        where a target is, and `tools/bench/hailo_api_equivalence.py` would then be
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
                # association-only. The node publishes them on `candidates`
                # (the tracker and the lock ladder) and never on `detections`,
                # so a consumer that acts on `detections` cannot steer on one
                # whatever its own floor is (`acting_conf`). Measured on real
                # footage: presence on hard clips 8.3 -> 34.8 % with no change
                # at all on clips that already work.
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

        The layout is packed, not fixed-stride, and the difference is silent.

            [ count_0, (y1 x1 y2 x2 score) * count_0,
              count_1, (y1 x1 y2 x2 score) * count_1, ... ]

        Each class's boxes follow its own count immediately; the next count
        sits right after them. The buffer is SIZED for the worst case --
        measured (1503,) for a 3-class model, i.e. 3 * (1 + 100 * 5) -- which
        makes a fixed per-class stride look plausible and arithmetically
        perfect. It is wrong: reading class 2 at a constant offset lands in
        the tail padding, finds a count of 0, and silently returns NOTHING for
        that class.

        Caught only because `tools/bench/hailo_api_equivalence.py` ran both APIs
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
        """Hand this detector's network group back. The DEVICE stays.

        Releasing the shared VDevice here would pull it out from under a second
        model that still has a group configured on it. `close_all` releases it
        at exit, after every group.
        """
        self._ready = False
        with self._infer_lock:
            for obj in (self._pipe, self._cim):
                if obj is not None:
                    try:
                        obj.__exit__(None, None, None)
                    except Exception:                    # noqa: BLE001
                        pass
            self._pipe = self._cim = self._bindings = None
            self._in_buf = self._out_buf = None

    def __repr__(self) -> str:
        return (f'<HailoDetector {Path(self._path).name} '
                f'{self._size}x{self._size} classes={len(self._names)}>')
