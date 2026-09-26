"""Vision preflight -- block until the perception pipeline is alive.

Called by `auv_manager_node`, once per process, before the first vision verb
of a mission dispatches: `wait_vision_state_ready` polls the `VisionState` the
manager already owns.

⛔ IT POLLS AN EXISTING VisionState AND SUBSCRIBES TO NOTHING. That is the
whole design. VisionState is owned by the rclpy executor thread, so a preflight
that opened its own subscriptions would have to spin rclpy from inside an
action callback -- re-entering the executor from a worker, with the heartbeat
running. Sleep-and-check from the outside cannot.

⛔ THE READINESS SIGNAL IS DETECTIONS, NOT FRAMES, and the reason is a bug that
cost 10 s on the first vision verb of every mission. See the note in
`wait_vision_state_ready`.

⚠ A SECOND, NODE-BASED PREFLIGHT USED TO LIVE HERE. `assert_vision_ready`
opened its own three subscriptions, counted a window and raised with the
failing stage named -- 99 lines, exported from `mongla_vision/__init__.py`,
and called by NOTHING for its whole life. Its docstring named two consumers,
`auv_manager_node` and the `vision_check` CLI, and neither ever called it: the
manager uses the state-based function above, and `utils/check_pipeline.py`
measures the same three topics itself. `check_pipeline` is also the RIGHT tool
for that job -- it reports a fixed window and a class histogram, where a
preflight exits on the first good window and caches, so routing one through
the other would cost the report its window. Deleted rather than wired, which
also took `rclpy.node`, `sensor_msgs`, `vision_msgs` and `qos` out of
`import mongla_vision` -- an import that `calibration/solver.py` and
`calibration/guide.py` both carry workarounds for.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

class VisionNotReadyError(RuntimeError):
    """Raised when the preflight times out. The message names the failing
    stage so the operator can fix it without grepping logs."""


@dataclass
class VisionStatus:
    image_hz:      float
    detection_hz:  float
    image_size:    tuple   # (W, H) from CameraInfo, or (0, 0) if not seen
    info_seen:     bool
    elapsed:       float


# Cache so missions don't pay the wait twice. Key = (process id, camera).
_PREFLIGHT_CACHE: dict = {}


# Enough messages to prove the producer is running rather than having emitted
# one startup frame. An empty Detection2DArray counts: "the detector is
# alive and looking" is the question here, not "it can see the target" --
# that is what `require_detection` asks.
_MIN_DET_MSGS = 5


def wait_vision_state_ready(vision_state, *,
                            timeout: float = 10.0,
                            stale_after: float = 0.8,
                            require_detection: bool = False,
                            log=None) -> VisionStatus:
    """Poll an EXISTING VisionState until it's healthy.

    Used by the manager: VisionState is owned by the rclpy executor
    thread, so we only need to sleep-and-check from inside an action
    callback -- no spin_once, no second subscription set, no risk of
    re-entering rclpy from a worker.

    `require_detection=True` waits for a fresh Sample (any class) to
    appear; otherwise we settle for "detections flowing + camera_info".

    THE READINESS SIGNAL IS DETECTIONS, NOT FRAMES. It used to be
    `diagnostics()['image_frames'] >= 5`, which could never become true:
    `VisionState` subscribed to `image_raw` RELIABLE while `camera_node`
    publishes it BEST_EFFORT, so the counter stayed at 0 and this function
    ALWAYS burned its full timeout -- 10 s added to the first vision verb of
    every mission, reported as "did not pass within 10s", which reads as a
    slow pipeline rather than a QoS mismatch. The same hazard is called out
    by name in `assert_vision_ready` above and in `check_pipeline`, and was
    fixed in five subscribers and missed in the one the control loop uses.

    Detections are also the better gate on the merits: they cannot flow
    without frames, and they are what the loop actually steers on.
    """
    cache_key = f'state::{id(vision_state)}'
    cached = _PREFLIGHT_CACHE.get(cache_key)
    if cached is not None and not require_detection:
        return cached

    deadline = time.monotonic() + max(timeout, 0.1)
    started  = time.monotonic()
    while True:
        diag = vision_state.diagnostics()
        info_ok    = bool(diag['info_seen'])
        frames_ok  = int(diag['det_msgs']) >= _MIN_DET_MSGS
        if require_detection:
            det_ok = vision_state.is_fresh(stale_after)
        else:
            det_ok = True

        if info_ok and frames_ok and det_ok:
            elapsed = time.monotonic() - started
            status = VisionStatus(
                image_hz=float('nan'), detection_hz=float('nan'),
                image_size=tuple(diag['image_size']),
                info_seen=info_ok, elapsed=elapsed)
            _PREFLIGHT_CACHE[cache_key] = status
            if log is not None:
                W, H = status.image_size
                log.info(
                    f"[VPRE ] state preflight READY  size={W}x{H}  "
                    f"det_msgs={diag['det_msgs']}  "
                    f"after {elapsed:.1f}s")
            return status

        if time.monotonic() >= deadline:
            missing = []
            if not info_ok:    missing.append('camera_info')
            if not frames_ok:
                missing.append(f'detection messages (<{_MIN_DET_MSGS})')
            if require_detection and not det_ok:
                missing.append(f'fresh detection (<{stale_after:.2f}s)')
            raise VisionNotReadyError(
                f"vision state preflight failed: missing {', '.join(missing)} "
                f"after {timeout:.1f}s -- start the vision pipeline "
                f"(e.g. `ros2 launch mongla_vision vision.launch.py camera:=forward`)")

        time.sleep(0.05)
