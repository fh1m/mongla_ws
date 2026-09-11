"""Pick a detector backend from the resolved model path. One seam, three callers.

Modelled on `cameras/factory.py`, which already solves the same problem for
camera sources.

WHY THE EXTENSION DECIDES, NOT A `backend=` PARAMETER
-----------------------------------------------------
Model identity in this stack is the **stem** -- `gate_rescue_repair`, not
`gate_rescue_repair.pt`. That is load-bearing: `duburi.use('gate_rescue_repair')`,
a `ClassRef`, `model:=` and `models:=` all pass stems, and the `<stem>.yaml`
sidecar is found by suffix-swap. So the same mission runs on the Jetson (`.pt`
or `.engine`) and on the Pi (`.hef`) with no mission change, because the only
thing that differs is which file the resolver found on that machine.

Adding a `backend=` argument would break that: every call site would have to
know what hardware it is on, and a wrong value fails at load rather than
falling back. The extension is already the ground truth about what was
compiled for this device.

PRIORITY, and it is deliberate
------------------------------
`.hef` > `.engine` > `.pt`. Each is a compiled artifact for THIS machine and is
faster than the generic weights: measured 82.3 Hz (Hailo) vs 20-30 Hz (TensorRT)
vs 3-4 Hz (raw PyTorch). A device only ever has one of them, so in practice the
order just means "prefer the compiled thing you found".
"""
from __future__ import annotations

from pathlib import Path

from .detector import Detector

# Extensions we know how to load, most-preferred first. `yolo._resolve_model_path`
# walks this same order when turning a stem into a path -- keep them in sync or a
# machine will resolve one artifact and the factory will try to load another.
KNOWN_EXTENSIONS = ('.hef', '.engine', '.pt')


def backend_for(path: str) -> str:
    ext = Path(str(path)).suffix.lower()
    if ext == '.hef':
        return 'hailo'
    return 'yolo'          # .pt and .engine both go through ultralytics


def make_detector(*, model_path: str, logger=None, **kwargs) -> Detector:
    """Construct the right backend for `model_path` (a stem or a full path).

    Every kwarg the YOLO backend takes is accepted; the Hailo backend ignores
    the ones that describe a graph compiled hours earlier on another machine
    (`device`, `iou`, `half`, `imgsz`). It does NOT ignore `conf` -- that one
    still filters at runtime, it just cannot go below the baked-in threshold.
    """
    from .yolo import _resolve_model_path      # local: pulls in ultralytics
    resolved = _resolve_model_path(model_path)
    kind = backend_for(resolved)

    if kind == 'hailo':
        # Imported lazily so a dev box without hailo_platform can still import
        # this module -- the same reason yolo.py does not import torch at module
        # scope. A missing runtime must fail when you ask for a .hef, not when
        # you import the package.
        from .hailo import HailoDetector, HailoSegDetector, emits_raw_heads
        cls = HailoSegDetector if emits_raw_heads(resolved) else HailoDetector
        return cls(model_path=resolved, logger=logger, **kwargs)

    from .yolo import YoloDetector
    return YoloDetector(model_path=resolved, logger=logger, **kwargs)
