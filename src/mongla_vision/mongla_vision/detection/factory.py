"""Pick a detector backend from the resolved model path. One seam, three callers.

Modelled on `cameras/factory.py`, which already solves the same problem for
camera sources.

Why the extension decides, not a `backend=` parameter
-----------------------------------------------------
Model identity in this stack is the **stem** -- `gate_sharks`, not
`gate_sharks.pt`. That is load-bearing: `mongla.use('gate_sharks')`,
a `ClassRef`, `model:=` and `models:=` all pass stems, and the `<stem>.yaml`
sidecar is found by suffix-swap. So the same mission runs on a dev box (`.pt`,
replaying footage) and on the Pi (`.hef`) with no mission change, because the
only thing that differs is which file the resolver found on that machine.

Adding a `backend=` argument would break that: every call site would have to
know what hardware it is on, and a wrong value fails at load rather than
falling back. The extension is already the ground truth about what was
compiled for this device.

PRIORITY, and it is deliberate
------------------------------
`.hef` > `.pt`. The `.hef` is compiled for THIS machine and is far faster than
the generic weights: measured 82.3 Hz (Hailo) vs 3-4 Hz (raw PyTorch on CPU).
So the order means "prefer the compiled thing you found".
"""
from __future__ import annotations

from pathlib import Path

from .detector import Detector

# Extensions we know how to load, most-preferred first. `yolo._resolve_model_path`
# walks this same order when turning a stem into a path -- keep them in sync or a
# machine will resolve one artifact and the factory will try to load another.
KNOWN_EXTENSIONS = ('.hef', '.pt')


def backend_for(path: str) -> str:
    ext = Path(str(path)).suffix.lower()
    if ext == '.hef':
        return 'hailo'
    return 'yolo'          # a .pt goes through ultralytics


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
        from .hailo import HailoDetector
        from .hef_file import emits_raw_heads
        from .hailo_seg import HailoSegDetector
        cls = HailoSegDetector if emits_raw_heads(resolved) else HailoDetector
        return cls(model_path=resolved, logger=logger, **kwargs)

    # Name what is missing (2026-10-03, on the vehicle). A Pi has no torch;
    # when a model has no .hef there, the resolver falls
    # back to the .pt and the failure read "No module named 'torch'" -- true,
    # and useless to an operator on a pool deck. The missing thing is the
    # compiled engine, so say that.
    import importlib.util
    if importlib.util.find_spec('torch') is None:
        stem = Path(str(resolved)).stem
        raise FileNotFoundError(
            f'{stem}: no compiled .hef for this machine (resolved {resolved}), '
            f'and PyTorch is not installed to run the .pt. Compile it: '
            f'tools/hailo_compile.sh {stem}, and ship {stem}.hef beside '
            f'{stem}.yaml.')
    from .yolo import YoloDetector
    return YoloDetector(model_path=resolved, logger=logger, **kwargs)
