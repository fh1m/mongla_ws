"""Every long-running node dies with its parent.

⛔ WHY (2026-10-03, on the vehicle). `ros2 launch` died on SIGTERM and left
every child running. An orphaned detector keeps the camera open and the
Hailo VDevice -- the chip allows ONE per process -- so the next launch cannot
open either, and fails as the "previous process still holding the device"
race -- and an orphaned lock or tracker node sits beside the next launch's
copy, publishing the same topics. Asking the kernel to SIGTERM us when the
parent exits makes the orphan impossible; rclpy turns SIGTERM into an ordinary clean shutdown, which closes
the camera and releases the chip.

Lives here because mongla_localization is the one Python package that both
the vision and localization nodes depend on. The manager does NOT use this: it must disarm on the way out, and guards
against the signal arriving while its parent is still alive
(`auv_manager_node._orphan_guard`).
"""
from __future__ import annotations

import signal

_PR_SET_PDEATHSIG = 1


def die_with_parent(sig: int = signal.SIGTERM) -> bool:
    """SIGTERM this process when its parent exits. False where unsupported."""
    try:
        import ctypes
        libc = ctypes.CDLL(None, use_errno=True)
        return libc.prctl(_PR_SET_PDEATHSIG, int(sig), 0, 0, 0) == 0
    except Exception:                       # noqa: BLE001 -- non-Linux: no-op
        return False
