"""The process's one Hailo VDevice, shared by every detector and XFeat anchor.

Opened through the HailoRT service when it runs, so several processes share
the chip; otherwise directly, and then a second VDevice in another process
fails (`_open` has the order). Within a process every network group on the
chip is configured on this one, under HailoRT's ROUND_ROBIN scheduler, and
handed back at exit before the device is.
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
#                                      the packaged library, which was built
#                                      without the service client. Built with
#                                      HAILO_BUILD_SERVICE (kutum/setup.sh),
#                                      processes share the chip: +0.012 ms a
#                                      frame, two at 47.8 Hz each
#                                      (measured-bars 80). `shared_device`
#                                      opens through it when it runs; a killed
#                                      service is back in 0.31 s (83).
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
            _DEVICE = _open()
    return _DEVICE


# Where the service is, and how long a starting process waits for it.
SERVICE_BIN = '/usr/local/bin/hailort_service'
SERVICE_PID = '/run/hailo/hailort_service.pid'
GROUP_ID = 'mongla'
SERVICE_WAIT_S = 3.0
# How this process falls back when the service is not there:
# 'direct' opens the chip itself (only one process can), 'none' refuses and
# the caller runs without the chip. The forward vision process is 'direct',
# the downward one 'none': a lost service costs the downward detector, never
# the forward one.
FALLBACK_ENV = 'MONGLA_HAILO_FALLBACK'

# What the last open did, for the start-up log and the health board:
# 'service', 'direct', or '' before the first open.
opened_via = ''


def _open(*, installed=None, running=None, wait_s=None, sleep=None,
          make=None, fallback=None):
    """The VDevice this process uses, in the order the owner chose.

    1. Through the HailoRT service, when it is installed. A process started
       while the service is restarting waits up to `SERVICE_WAIT_S` for it:
       systemd brings it back 200 ms after a crash (hailort.service).
    2. Otherwise, by `MONGLA_HAILO_FALLBACK`: 'direct' (the default, and what
       a bench or a single-process launch gets) opens the chip in this
       process; 'none' raises, and the caller runs without the chip.

    A process that loses the service mid-run fails its inferences, exits
    after its rebuild also fails (`DetectorNode._on_infer_failure`) and is
    respawned by its launch, which brings it back here. The keyword
    arguments are for the tests.
    """
    import os
    import time
    global opened_via
    installed = os.path.exists(SERVICE_BIN) if installed is None else installed
    running = running or (lambda: os.path.exists(SERVICE_PID))
    wait_s = SERVICE_WAIT_S if wait_s is None else wait_s
    sleep = sleep or time.sleep
    make = make or _make
    fallback = fallback or os.environ.get(FALLBACK_ENV, 'direct')
    if fallback not in ('direct', 'none'):
        raise ValueError(f'{FALLBACK_ENV}={fallback!r}: expected direct or none')
    service_error = None
    if installed:
        waited = 0.0
        while not running() and waited < wait_s:
            sleep(0.1)
            waited += 0.1
        if running():
            try:
                dev = make(service=True)
                opened_via = 'service'
                return dev
            except Exception as exc:                         # noqa: BLE001
                service_error = exc
    if fallback == 'none':
        raise RuntimeError(
            'the HailoRT service is not available and this process does not '
            'open the chip itself (' + FALLBACK_ENV + '=none)'
            + (f': {service_error!r}' if service_error else ''))
    dev = make(service=False)
    opened_via = 'direct'
    return dev


def _make(*, service: bool):
    from hailo_platform import HailoSchedulingAlgorithm, VDevice
    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    if service:
        params.multi_process_service = True
        params.group_id = GROUP_ID
    return VDevice(params)
