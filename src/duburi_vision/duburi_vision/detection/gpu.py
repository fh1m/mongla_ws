"""select_device — single decision point for laptop / docker / Jetson.

Called once at YoloDetector construction. Fail-fast on missing CUDA so a
broken docker/Jetson is loud at startup, not slow during a mission. The
log line is the canary you grep for during deployment ("am I really on
GPU?"). CPU is opt-in via `device='cpu'` (or 'auto' to pick whatever's
available).
"""

from __future__ import annotations


_DEFAULT = 'cuda:0'


def select_device(requested: str = _DEFAULT, *, logger=None) -> str:
    """Resolve a device string for ultralytics / torch.

    Parameters
    ----------
    requested : 'cuda:N' | 'cuda' | 'cpu' | 'auto'
        - 'cuda*' : require CUDA, raise if unavailable
        - 'cpu'   : explicit CPU
        - 'auto'  : prefer CUDA, fall back to CPU silently (tests, CI)

    Returns
    -------
    str
        A canonical device string ready for `model.to(device)`.
    """
    req = (requested or _DEFAULT).strip().lower()
    if req == '':
        req = _DEFAULT

    try:
        import torch
    except ImportError as exc:
        raise RuntimeError(
            "torch is required for inference. Install the matching "
            "torch+cu128 wheel on this machine.") from exc

    cuda_ok = bool(torch.cuda.is_available())

    def _emit(line):
        if logger is not None and hasattr(logger, 'info'):
            logger.info(line)
        else:
            print(line)

    if req == 'auto':
        if cuda_ok:
            # Parse the device index from _DEFAULT (e.g. 'cuda:0' -> 0). NOTE: split
            # _DEFAULT, not req -- req is 'auto' here (no colon), so splitting it
            # crashed with IndexError on any CUDA host whenever _DEFAULT carried an
            # index (which it always does: 'cuda:0'). That made `device: auto` an
            # instant detector crash on the Jetson -- the exact competition host.
            idx = int(_DEFAULT.split(':', 1)[1]) if ':' in _DEFAULT else 0
            _emit(f"[VIS  ] using {_DEFAULT} ({torch.cuda.get_device_name(idx)})  "
                  f"torch={torch.__version__}  cuda={torch.version.cuda}")
            return _DEFAULT
        _emit(f"[VIS  ] using cpu (auto: cuda unavailable)  torch={torch.__version__}")
        return 'cpu'

    if req == 'cpu':
        _emit(f"[VIS  ] using cpu (requested)  torch={torch.__version__}")
        return 'cpu'

    if req.startswith('cuda'):
        if not cuda_ok:
            raise RuntimeError(
                f"device={requested!r} requested but CUDA is not available. "
                f"torch={torch.__version__} torch.cuda.is_available()=False. "
                f"Either fix CUDA install (driver / cuda toolkit / matching torch wheel) "
                f"or set device='cpu' in detector.yaml to run on CPU.")
        idx = 0
        if ':' in req:
            try:
                idx = int(req.split(':', 1)[1])
            except ValueError:
                pass
        _emit(f"[VIS  ] using {req} ({torch.cuda.get_device_name(idx)})  "
              f"torch={torch.__version__}  cuda={torch.version.cuda}")
        return req

    raise ValueError(
        f"unknown device {requested!r}. use 'cuda', 'cuda:N', 'cpu', or 'auto'.")
