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

    if req == 'auto':
        if cuda_ok:
            return _log_gpu(torch, _DEFAULT, logger)
        return _log_cpu(torch, logger, note='auto: cuda unavailable')

    if req == 'cpu':
        return _log_cpu(torch, logger, note='requested')

    if req.startswith('cuda'):
        if not cuda_ok:
            raise RuntimeError(
                f"device={requested!r} requested but CUDA is not available. "
                f"torch={torch.__version__} torch.cuda.is_available()=False. "
                f"Either fix CUDA install (driver / cuda toolkit / matching torch wheel) "
                f"or set device='cpu' in detector.yaml to run on CPU.")
        return _log_gpu(torch, req, logger)

    raise ValueError(
        f"unknown device {requested!r}. use 'cuda', 'cuda:N', 'cpu', or 'auto'.")


def _log_gpu(torch, device, logger):
    idx = 0
    if ':' in device:
        try:
            idx = int(device.split(':', 1)[1])
        except ValueError:
            idx = 0
    name = torch.cuda.get_device_name(idx)
    line = (
        f"[VIS  ] using {device} ({name})  torch={torch.__version__}  "
        f"cuda={torch.version.cuda}")
    _emit(line, logger)
    return device


def _log_cpu(torch, logger, *, note):
    line = f"[VIS  ] using cpu ({note})  torch={torch.__version__}"
    _emit(line, logger)
    return 'cpu'


def _emit(line, logger):
    if logger is not None and hasattr(logger, 'info'):
        logger.info(line)
    else:
        print(line)
