"""The process's one Hailo VDevice, shared by every detector and XFeat anchor.

A second VDevice in the same process, or in another one, fails; so every
network group on the chip is configured on this one, under HailoRT's
ROUND_ROBIN scheduler, and handed back at exit before the device is.
`hailo.py` (the detectors) and `anchor/xfeat_hailo.py` (the anchors) both
use `shared_device()`, `register()` and `DEVICE_LOCK`.
"""
from __future__ import annotations

import atexit
import threading
import weakref

# --------------------------------------------------------------------------- #
#  One device per process, and hailort's scheduler shares it                  #
# --------------------------------------------------------------------------- #
# Measured on this hardware:
#
#   two PROCESSES, a VDevice each   -> HAILO_OUT_OF_PHYSICAL_DEVICES (74) on the
#                                      second. Sharing happens in ONE process.
#   multi_process_service + scheduler -> HailoRTInvalidOperationException from
#                                      the installed library, with or without
#                                      a service running: it was built without
#                                      the service client. Built from source
#                                      with HAILO_BUILD_SERVICE, two processes
#                                      share the chip at 47.8 Hz each and one
#                                      inference costs +0.012 ms
#                                      (measured-bars 80). Not installed: the
#                                      stack is one process by design.
#   ONE process, activation handed back and forth by hand (what shipped until
#   2026-10-03)                      -> works: 73.8 Hz across two detectors
#                                      swapping every frame, 4.15 ms a swap.
#   ONE process, ROUND_ROBIN scheduler (now) -> 46.1 + 46.1 = 92.2 Hz for the
#                                      same two detectors, and with both XFeat
#                                      anchors added at a realistic duty
#                                      (~5 Hz each) the detectors keep 44.4 Hz
#                                      each. HailoRT switches network groups
#                                      itself, in C++, with NO Python lock held
#                                      around an inference.
#
# RETRACTED: "ROUND_ROBIN scheduler, one process -> SIGSEGV". Re-measured
# 2026-10-03 on HailoRT 4.24: four network groups, four threads, a group
# configured while the others were mid-inference and one closed mid-run --
# zero errors, exit 0, three runs out of three. What DOES segfault is
# interpreter teardown with groups still configured (see `close_all`), which
# every bench of that era hit on the way out because none released the device.
# The crash was real; the attribution was wrong.
#
# Why the hand-off had to go, measured live (measured-bars §28.6): the
# hand-off held `DEVICE_LOCK` across Python code -- letterbox, the
# `run_async` wrapper, the activation swap -- so whenever the holder lost the
# GIL to another thread the CHIP sat idle and the other detector waited for
# a lock whose owner was not even running. With the lock ladders composed
# into the same process, two detectors fell from 69 to 47 Hz combined at an
# unchanged ~10.5 ms of chip time per call. A scheduler holds no Python lock,
# so a thread starved of the GIL delays only itself.
_DEVICE = None
# Guards device CREATION and every network-group CONFIGURE. Never held
# around an inference: the scheduler arbitrates the chip.
DEVICE_LOCK = threading.Lock()
# Everything that configured a network group on `_DEVICE`: the detectors AND
# the XFeat anchors. Weak, so a dropped object is not kept alive by this.
_CLAIMANTS: 'weakref.WeakSet' = weakref.WeakSet()


def register(claimant) -> None:
    """Record a network-group owner so `close_all` can hand it back."""
    _CLAIMANTS.add(claimant)


def close_all() -> None:
    """Release every configured network group, THEN the VDevice. At exit.

    Measured on the vehicle, 2026-10-03: two detectors and two XFeat
    anchors in one process, left to interpreter teardown, die with SIGSEGV
    (rc 139, sometimes "corrupted double-linked list") or hang there -- the
    garbage collector frees the VDevice and the ConfiguredInferModels in
    whatever order it likes. The same process closing each group and then
    releasing the device exits 0 in 1.2 s. No node called `close()`, so every
    Hailo process on the vehicle crashed on the way out, and a SIGINT'd
    detector that hangs instead is a process holding the chip that the next
    launch cannot open.

    Registered with `atexit`, so no node has to remember it. Safe to call
    twice. A worker thread still inside an infer holds `DEVICE_LOCK`, and
    `close()` waits for it rather than pulling the stream out from under it.
    """
    global _DEVICE
    for c in list(_CLAIMANTS):
        try:
            c.close()
        except Exception:                                    # noqa: BLE001
            pass
    with DEVICE_LOCK:
        dev, _DEVICE = _DEVICE, None
    if dev is not None:
        try:
            dev.release()
        except Exception:                                    # noqa: BLE001
            pass


atexit.register(close_all)


def shared_device():
    """The process's single VDevice. Created on first use, never released.

    Not released on close() either: releasing it while a second detector still
    holds a network group configured on it is how you turn a clean shutdown
    into a segfault, and the process is about to exit anyway.

    The creation is locked, and that is not defensive. `detector_node` builds
    a registry in a `ThreadPoolExecutor`, so two detectors are constructed
    Genuinely concurrently. Unlocked, both threads saw `_DEVICE is None` and
    both called `VDevice()` -- and the note above this function already records
    what a second VDevice does. Measured on the vehicle: sequential
    construction of `gate_sharks` and `bin_fire_blood` gives OK/OK,
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
    with DEVICE_LOCK:
        # Re-checked inside: the thread that waited here must see the device
        # the winner made, not make a second one.
        if _DEVICE is None:
            from hailo_platform import HailoSchedulingAlgorithm, VDevice
            params = VDevice.create_params()
            params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
            _DEVICE = VDevice(params)
    return _DEVICE
