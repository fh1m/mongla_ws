"""Guided camera calibration as a node; Batighor is its page.

    ros2 launch mongla_batighor calibrate.launch.py \\
        device_path:=/dev/mongla_cam_downward applies_to:=pi_downward

The launch starts a camera node named `calibration` at the calibration
resolution (the vision launch must be stopped: it holds the cameras), this
node, and Batighor. Everything here is under `/mongla/vision/calibration/`,
so one layout serves any camera:

  image_raw                  the camera's frames, read here
  state                      latched JSON, at most 4 Hz and only on change:
                             the step and what it asks for, the live pose,
                             the board's corners, the target box, the
                             least-certain pixel, the fit so far, the solve
                             and the calibration library
  undistorted/compressed     the frame corrected by the latest fit, 2 Hz, only
                             while someone watches: straight edges must come
                             out straight, the one check made with the eyes
  op                         `mongla_interfaces/srv/Helm`, `op` and a JSON
                             `fields_json`. Batighor's helm forwards an
                             operator's request here; a terminal may call it

The ops: `skip` a pose; `undo` the last capture; `reset` the capture set;
`solve`, which runs `solver.py` and installs the calibration; `apply` an
existing calibration to this camera ({"file": ...}); and `configure` a new
capture ({"applies_to", "medium", "grid", "square"}). A capture taken back is
moved aside into `set-aside-<time>/` beside it, never deleted.

The pose script, the detection and the live assessment are `guide_capture`'s,
unchanged; the maths is `solver`'s. Drawing is the browser's:
`mongla_batighor/overlay.py` turns `state` into annotations over the camera's
own JPEG.
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

from mongla_vision.calibration import guide_capture as capture
from mongla_vision.calibration import guide_files as files
from mongla_vision.calibration import solver

NS = '/mongla/vision/calibration'
CAMERA = 'calibration'
OPS = ('skip', 'undo', 'reset', 'solve', 'apply', 'configure')


def state_dict(st, applies_to: str, size, result: str, library) -> dict:
    """Everything the page shows, from one `guide_capture.State`. No ROS."""
    with st.lock:
        i, n = st.i, len(st.steps)
        ty, tx, tb = st.steps[min(i, n - 1)]
        cell, band = st.cell, st.band
        target = None
        if size and i < n:
            w, h = size
            g = capture.GRID
            target = [tx * w / g, ty * h / g, (tx + 1) * w / g, (ty + 1) * h / g]
        corners = (np.asarray(st.corners).reshape(-1, 2).round(1).tolist()
                   if st.corners is not None else None)
        return {
            'applies_to': applies_to, 'medium': st.medium,
            'grid': f'{st.cols}x{st.rows}', 'square': st.square,
            'size': list(size) if size else None,
            'i': i, 'n': n, 'done': i >= n,
            'want_cell': capture.CELL_NAME[ty][tx], 'want_tilt': capture.TILT_NAME[tb],
            'cell': capture.CELL_NAME[cell[0]][cell[1]] if cell else None,
            'band': capture.TILT_NAME[band] if band is not None else None,
            'hold': round(st.hold, 2), 'msg': st.msg, 'err': st.err,
            'sharp': st.sharp, 'sharp_max': capture.MAX_SHARPNESS_PX,
            'corners': corners, 'target': target, 'worst_px': st.worst_px,
            'ere': st.ere, 'ere_pred': st.ere_pred, 'ere_bar': capture.ERE_BAR_PX,
            'n_boards': st.n_boards, 'assess_err': st.assess_err,
            'fx': st.fx, 'fy': st.fy, 'cx': st.cx, 'cy': st.cy, 'fx_sd': st.fx_sd,
            'hfov_air': st.hfov_air, 'hfov_water': st.hfov_water,
            'suggest': (f'{capture.CELL_NAME[st.suggest[0][0]][st.suggest[0][1]]}'
                        f' at ~{st.suggest[1]} deg' if st.suggest else None),
            'det_ms': round(st.det_ms, 1),
            'solve': st.solve, 'solve_out': st.solve_out[-1500:],
            'result': result if os.path.exists(result) else '',
            'library': library,
        }


def next_capture(current: dict, args: dict):
    """(the settings of the capture `args` asks for, '') or (None, why not).

    The capture folder always follows the camera AND the medium: a water
    capture that resumed the air one's frames would install a `_water` file
    made of air pixels, a correct-looking artifact from the wrong input.
    """
    unknown = sorted(set(args) - {'applies_to', 'medium', 'grid', 'square'})
    if unknown:
        return None, f'unknown setting {", ".join(unknown)}'
    applies_to = str(args.get('applies_to', current['applies_to']))
    medium = str(args.get('medium', current['medium']))
    grid = str(args.get('grid', current['grid']))
    try:
        square = float(args.get('square', current['square']))
        out = files.capture_dir(applies_to, medium)      # refuses a bad pair
        cols, rows = (int(x) for x in grid.lower().split('x'))
    except (ValueError, TypeError) as exc:
        return None, str(exc)
    if cols < 2 or rows < 2 or not square > 0:
        return None, f'{grid} at {square} m is not a board'
    return {'applies_to': applies_to, 'medium': medium, 'grid': grid,
            'square': square, 'out': out}, ''


def set_aside(outdir: str, paths) -> str:
    """Move captures into a dated folder beside them; never delete one."""
    dest = os.path.join(outdir, 'set-aside-' + time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(dest, exist_ok=True)
    for p in paths:
        shutil.move(p, os.path.join(dest, os.path.basename(p)))
    return dest


class Capture:
    """One capture set: a State, its two threads, and the ops on it. No ROS."""

    def __init__(self, applies_to, medium, grid, square, install, out=None,
                 detect_width=480):
        cols, rows = (int(x) for x in grid.lower().split('x'))
        self.applies_to, self.grid, self.install = applies_to, grid, install
        self.outdir = out or files.capture_dir(applies_to, medium)
        os.makedirs(self.outdir, exist_ok=True)
        st = capture.State(cols, rows, self.outdir, capture.build_steps())
        st.square, st.medium = square, medium
        capture.resume(st)
        self.st = st
        self.size = None
        self.detect_width = detect_width
        self._assess = None
        threading.Thread(target=capture.detect_loop, args=(st, detect_width),
                         daemon=True).start()

    def frame(self, img) -> None:
        """A new camera frame. The assessment starts once the size is known."""
        size = (int(img.shape[1]), int(img.shape[0]))
        with self.st.lock:
            self.st.raw = img
        if self.size is None:
            self.size = size
            self._assess = threading.Thread(
                target=capture.assess,
                args=(self.st, self.st.cols, self.st.rows, self.st.square, size, solver),
                daemon=True)
            self._assess.start()
        elif size != self.size:
            with self.st.lock:
                self.st.err = (f'the camera changed size {self.size} -> {size}; '
                               f'configure a new capture')

    def stop(self) -> None:
        self.st.stopped = True

    def result_path(self) -> str:
        if not self.size:
            return ''
        return os.path.join(self.install, solver.calibration_filename(
            self.applies_to, self.size[0], self.size[1], self.st.medium))

    def captures(self):
        return sorted(glob.glob(os.path.join(self.outdir, 'cal_*.png')))

    def skip(self) -> str:
        with self.st.lock:
            if self.st.i >= len(self.st.steps):
                return 'every pose is done'
            self.st.i += 1
            self.st.msg = 'skipped -- that pose is missing from the set'
        return 'skipped'

    def undo(self) -> str:
        # A view you know was bad -- a hand in frame, a wobble -- is worth
        # taking back on the spot; leaving it in and hoping the fit eats it is
        # how a quiet bias gets in.
        with self.st.lock:
            caps = self.captures()
            if not caps:
                return 'nothing captured yet'
            dest = set_aside(self.outdir, caps[-1:])
            self.st.i = max(0, self.st.i - 1)
            self.st.assessed_n = -1           # force a refit
            self.st.msg = f'took back {os.path.basename(caps[-1])}'
        return f'took back {os.path.basename(caps[-1])} (kept in {dest})'

    def reset(self) -> str:
        with self.st.lock:
            caps = self.captures()
            dest = set_aside(self.outdir, caps) if caps else ''
            self.st.i = 0
            self.st.ere = self.st.ere_pred = self.st.K = self.st.D = None
            self.st.worst_px = None
            self.st.assessed_n = -1
            self.st.n_boards = 0
            self.st.msg = 'reset'
        return f'reset; {len(caps)} captures kept in {dest}' if caps else 'reset'

    def solve_argv(self, device: str):
        return [sys.executable, solver.__file__, self.outdir,
                '--grid', self.grid, '--square', str(self.st.square),
                '--applies-to', self.applies_to, '--install', self.install,
                '--medium', self.st.medium,
                '--identity', json.dumps(files.camera_identity(device))]

    def solve(self, device: str) -> str:
        with self.st.lock:
            if self.st.solve == 'running':
                return 'a solve is already running'
            self.st.solve, self.st.solve_out = 'running', 'solving...'
        dest = self.result_path()
        if not dest:
            with self.st.lock:
                self.st.solve = 'idle'
            return 'no frame from the camera yet'
        argv = self.solve_argv(device)

        def run():
            try:
                r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
                ok = r.returncode == 0 and os.path.exists(dest)
                out = (r.stdout or '') + (r.stderr or '')
            except Exception as exc:              # noqa: BLE001 -- reported, not raised
                ok, out = False, repr(exc)
            with self.st.lock:
                self.st.solve = 'done' if ok else 'failed'
                self.st.solve_out = out[-4000:]

        threading.Thread(target=run, daemon=True).start()
        return f'solving {len(self.captures())} captures -> {os.path.basename(dest)}'


class CalibrateNode:
    """The ROS half, built by `main` so the module imports without rclpy."""

    @staticmethod
    def build():
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import CompressedImage, Image
        from std_msgs.msg import String

        from mongla_interfaces.srv import Helm
        from mongla_vision import qos
        from mongla_vision.image_msg import to_array

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        class Node_(Node):
            def __init__(self):
                super().__init__('mongla_calibrate')
                p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
                self.device = p('device_path', '')
                self.install = os.path.abspath(
                    p('install', '') or files.default_calibration_dir())
                os.makedirs(self.install, exist_ok=True)
                exposure, brightness = p('exposure', 50), p('brightness', 150)
                self.cap = Capture(p('applies_to', 'pi_downward'), p('medium', 'air'),
                                   p('grid', '8x6'), p('square', 0.025), self.install,
                                   p('out', '') or None, p('detect_width', 480))
                if self.device and exposure:
                    # The motion-blur fix (guide_capture.set_camera): a short
                    # manual shutter, set on the device the camera node streams.
                    if not capture.set_camera(self.device, exposure, brightness):
                        self.get_logger().warning(
                            f'could not set the exposure on {self.device}; '
                            f'auto exposure smears a moving board')
                self._last = ''
                # Which cameras are plugged in, and whether each one's
                # calibration was taken on that unit: the competition-day
                # question. A v4l2-ctl per device, so once at start and after
                # each configure or apply, never at the state's rate.
                self.cameras = []
                self._survey()
                self.pub_state = self.create_publisher(String, f'{NS}/state', latched)
                self.pub_undist = self.create_publisher(
                    CompressedImage, f'{NS}/undistorted/compressed', qos.IMAGE)
                self.create_subscription(
                    Image, f'{NS}/image_raw',
                    lambda m: self.cap.frame(to_array(m, 'bgr8')), qos.IMAGE)
                self.create_service(Helm, f'{NS}/op', self._on_op)
                self.create_timer(0.25, self._publish)
                self.create_timer(0.5, self._undistorted)

            def _library(self):
                return files.library(self.install)

            def _survey(self):
                def run():
                    self.cameras = files.list_cameras(self.install)
                threading.Thread(target=run, daemon=True).start()

            def _publish(self, force=False):
                d = state_dict(self.cap.st, self.cap.applies_to, self.cap.size,
                               self.cap.result_path(), self._library())
                d['cameras'] = self.cameras
                s = json.dumps(d, sort_keys=True)
                if force or s != self._last:
                    self._last = s
                    self.pub_state.publish(String(data=s))

            def _undistorted(self):
                if self.pub_undist.get_subscription_count() == 0:
                    return
                with self.cap.st.lock:
                    raw, K, D = self.cap.st.raw, self.cap.st.K, self.cap.st.D
                if raw is None or K is None:
                    return
                img = cv2.undistort(raw, K, D)
                ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    m = CompressedImage(format='jpeg', data=jpg.tobytes())
                    m.header.stamp = self.get_clock().now().to_msg()
                    self.pub_undist.publish(m)

            def _on_op(self, req, res):
                res.ok, res.message = self.op(req.op, req.fields_json)
                if req.op in ('apply', 'configure'):
                    self._survey()
                self.get_logger().info(f'[CAL  ] {req.who or "terminal"}: {req.op} '
                                       f'-> {res.message}')
                self._publish(force=True)
                return res

            def op(self, op, fields_json):
                try:
                    args = json.loads(fields_json) if (fields_json or '').strip() else {}
                except ValueError:
                    return False, 'fields are not JSON'
                if not isinstance(args, dict):
                    return False, 'fields must be a JSON object'
                if op == 'skip':
                    return True, self.cap.skip()
                if op == 'undo':
                    return True, self.cap.undo()
                if op == 'reset':
                    return True, self.cap.reset()
                if op == 'solve':
                    return True, self.cap.solve(self.device)
                if op == 'apply':
                    if not self.device:
                        return False, 'no device_path: cannot read whose camera this is'
                    name, err = files.apply_calibration(
                        self.install, str(args.get('file', '')), self.device,
                        self.cap.applies_to)
                    return (True, f'applied -> {name}; relaunch the vision stack '
                                  f'to use it') if name else (False, err)
                if op == 'configure':
                    return self.configure(args)
                return False, f'unknown op {op!r}; ops: {", ".join(OPS)}'

            def configure(self, args):
                old = self.cap
                cfg, why = next_capture(
                    {'applies_to': old.applies_to, 'medium': old.st.medium,
                     'grid': old.grid, 'square': old.st.square}, args)
                if cfg is None:
                    return False, why
                old.stop()
                self.cap = Capture(install=self.install,
                                   detect_width=old.detect_width, **cfg)
                return True, (f'{cfg["applies_to"]} in {cfg["medium"]}, board '
                              f'{cfg["grid"]} at {cfg["square"]} m -> '
                              f'{self.cap.outdir} (step {self.cap.st.i + 1})')

        return Node_


def main(args=None):
    import rclpy
    from mongla_localization.params_file import refuse_unknown
    from mongla_localization.spin import spin
    rclpy.init(args=args)
    node = CalibrateNode.build()()
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    node._publish(force=True)
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
