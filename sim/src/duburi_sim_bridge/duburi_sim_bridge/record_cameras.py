#!/usr/bin/env python3

"""Record bridged front/bottom camera streams for dataset building.

    datasets/<label>_<stamp>/
      front.mp4 | bottom.mp4
      meta.json
      classes.txt
      frames/{front,bottom}/######.png     (--frames)
      labels/{front,bottom}/######.txt    (--labels)

MP4 duration matches wall-clock time: frames are buffered and encoded at
fps_actual = count / duration_s (not a fixed nominal fps).

Usage:
    ros2 run duburi_sim_bridge record_cameras --duration 60
    ros2 run duburi_sim_bridge record_cameras --fx --frames --labels --label gate_approach
"""

from __future__ import annotations

import argparse
import json
import queue
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

from duburi_sim_bridge.paths import sim_ws_root
from duburi_sim_bridge.underwater_fx import apply_underwater_fx
from duburi_sim_bridge.box_labels import (
    CLASSES,
    boxes_to_yolo,
    format_yolo_line,
    write_classes_file,
)

FRONT_RAW = '/duburi/sim/front_camera/image_raw'
BOTTOM_RAW = '/duburi/sim/bottom_camera/image_raw'
FRONT_BOXES = '/duburi/sim/front_camera/boxes'
BOTTOM_BOXES = '/duburi/sim/bottom_camera/boxes'
FRONT_FX = '/duburi/sim/front_camera/image_fx'
BOTTOM_FX = '/duburi/sim/bottom_camera/image_fx'
FRONT_INFO = '/duburi/sim/front_camera/camera_info'
BOTTOM_INFO = '/duburi/sim/bottom_camera/camera_info'
GROUND_TRUTH = '/duburi/sim/ground_truth'

ENCODING_TO_CHANNELS = {
    'rgb8': 3,
    'bgr8': 3,
    'rgba8': 4,
    'bgra8': 4,
    'mono8': 1,
}

_REC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _workspace_root() -> Path:
    # One implementation for both workspaces; see paths.py for why the old
    # name-match + cwd fallback was deleted (it wrote datasets/ to a random dir).
    return sim_ws_root()


def _course_yaml(course: str) -> Path:
    try:
        share = Path(get_package_share_directory('duburi_sim_worlds'))
        candidate = share / 'courses' / f'{course}.yaml'
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    return _workspace_root() / 'src' / 'duburi_sim_worlds' / 'courses' / f'{course}.yaml'


def _image_to_bgr(msg: Image) -> np.ndarray:
    channels = ENCODING_TO_CHANNELS.get(msg.encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding: {msg.encoding}')
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    if raw.size < expected:
        raise ValueError('truncated image buffer')
    img = raw[:expected].reshape((msg.height, msg.step))[:, : msg.width * channels]
    img = img.reshape((msg.height, msg.width, channels))
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _fx_params_for(course: str, lighting: str = '') -> dict:
    """The course's own turbidity, from the <course>.fx.yaml sidecar.

    Recording has to use the SAME water the live sim shows, or a dataset
    captured from `lighting: murky` would quietly be clear. The sidecar is the
    one source of truth for that (gz-sim ignores the world's <scene><fog>
    entirely -- see gen_world.py), and it is what bridge.launch.py feeds the
    live node, so reading it here keeps the two paths from drifting.

    Falls back to the package defaults if the sidecar is missing, exactly as
    bridge.launch.py does, so a hand-written world still records something sane.
    """
    keys = ('turbidity', 'backscatter', 'blur_sigma', 'noise', 'vignette',
            'atten_scale')
    params = {}
    try:
        from ament_index_python.packages import get_package_share_directory
        import yaml
        share = Path(get_package_share_directory('duburi_sim_worlds'))
        presets = share / 'worlds' / 'lighting_presets.yaml'
        if lighting and presets.is_file():
            # --lighting overrides the course, so a murky/clear PAIR of the same
            # scene comes out of one sim launch instead of two courses that
            # would also differ in prop placement.
            table = yaml.safe_load(presets.read_text())
            if lighting not in table:
                raise KeyError(
                    f'unknown lighting {lighting!r}; known: '
                    f"{', '.join(sorted(table))}")
            params = table[lighting]
        elif (share / 'worlds' / f'{course}.fx.yaml').is_file():
            params = yaml.safe_load(
                (share / 'worlds' / f'{course}.fx.yaml').read_text()
            )['/**']['ros__parameters']
        else:
            share = Path(get_package_share_directory('duburi_sim_bridge'))
            params = yaml.safe_load(
                (share / 'config' / 'underwater_fx.yaml').read_text()
            )['/**']['ros__parameters']
    except Exception:
        params = {}
    defaults = {'turbidity': 0.45, 'backscatter': 0.55, 'blur_sigma': 0.8,
                'noise': 0.012, 'vignette': 0.25, 'atten_scale': 1.0}
    return {k: float(params.get(k, defaults[k])) for k in keys}


class CameraRecorder(Node):
    def __init__(
        self,
        out_dir: Path,
        cameras: list[str],
        dump_frames: bool,
        write_labels: bool,
        use_fx: bool,
        fps: float,
        course: str,
        label: str,
        lighting: str = '',
    ) -> None:
        super().__init__('record_cameras')
        self._out_dir = out_dir
        self._cameras = cameras
        self._dump_frames = dump_frames
        self._write_labels = write_labels
        self._fps_hint = fps
        self._course = course
        self._label = label
        self._depth = -0.8       # until the first ground-truth odom arrives
        self._lock = threading.Lock()
        self._frames: dict[str, list[np.ndarray]] = {c: [] for c in cameras}
        self._counts = {c: 0 for c in cameras}
        self._sizes: dict[str, tuple[int, int]] = {}
        self._encodings: dict[str, str] = {}
        self._stop = False
        self._pose = None
        self._gt_start = None
        self._gt_end = None
        self._trajectory: list[dict] = []
        self._last_traj_t = 0.0
        self._info: dict[str, CameraInfo] = {}
        self._disk_q: queue.Queue = queue.Queue(maxsize=256)
        self._disk_thread = threading.Thread(target=self._disk_worker, daemon=True)
        self._disk_thread.start()

        # Labels come from Gazebo's boundingbox_camera, not from projecting
        # prop AABBs ourselves. The GPU already knows what is visible, so the
        # boxes are occlusion- and truncation-correct for free -- the projector
        # gave a hidden crate a full box, and it had no idea a prop was behind
        # another one. It also means the course YAML is no longer consulted:
        # anything spawned at RUNTIME is labelled too, which the projector
        # could never do.
        self._boxes = {c: None for c in cameras}
        if write_labels:
            write_classes_file(out_dir / 'classes.txt')
            self.get_logger().info(
                f'GT labels from Gazebo bounding-box cameras '
                f'({len(CLASSES)} classes)')

        for cam in cameras:
            if dump_frames:
                (out_dir / 'frames' / cam).mkdir(parents=True, exist_ok=True)
            if self._write_labels:
                (out_dir / 'labels' / cam).mkdir(parents=True, exist_ok=True)

        # --fx reads image_RAW and filters in-process; it does NOT subscribe to
        # image_fx.
        #
        # Measured 2026-08-28: the underwater_fx node burns a full CPU core even
        # with `enabled: false` -- i.e. as a pure subscribe-and-republish, doing
        # no image work at all. The cost is rclpy moving ~1 MB Image messages,
        # four crossings per frame per camera, and it drags the cameras from
        # 12 Hz down to 5 Hz and the recording to 2.4 fps. The filter itself is
        # 4.4 ms and irrelevant next to that.
        #
        # So the frame is already here, in memory, and applying the filter costs
        # 4.4 ms while subscribing to a filtered copy of it costs most of a core.
        # Same pixels, recorded at the sim's real frame rate.
        self._fx_params = _fx_params_for(course, lighting) if use_fx else None
        topics = {'front': FRONT_RAW, 'bottom': BOTTOM_RAW}
        self._topics = {c: topics[c] for c in cameras}
        for cam in cameras:
            self.create_subscription(
                Image, self._topics[cam], lambda m, c=cam: self._on_image(c, m), _REC_QOS
            )
            if self._write_labels:
                self.create_subscription(
                    Detection2DArray,
                    FRONT_BOXES if cam == 'front' else BOTTOM_BOXES,
                    lambda m, c=cam: self._on_boxes(c, m), _REC_QOS)
            info_topic = FRONT_INFO if cam == 'front' else BOTTOM_INFO
            self.create_subscription(
                CameraInfo, info_topic, lambda m, c=cam: self._on_info(c, m), _REC_QOS
            )
        self.create_subscription(Odometry, GROUND_TRUTH, self._on_odom, 10)
        self.get_logger().info(
            f'recording {cameras} → {out_dir}  fx={use_fx} frames={dump_frames} '
            f'labels={self._write_labels}'
        )

    def request_stop(self) -> None:
        self._stop = True

    def _disk_worker(self) -> None:
        while True:
            item = self._disk_q.get()
            if item is None:
                self._disk_q.task_done()
                break
            try:
                kind = item[0]
                if kind == 'png':
                    _, path, bgr = item
                    cv2.imwrite(path, bgr)
                elif kind == 'label':
                    _, path, text = item
                    Path(path).write_text(text)
            except Exception as exc:
                self.get_logger().warn(f'disk worker: {exc}')
            finally:
                self._disk_q.task_done()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        xyz = (p.x, p.y, p.z)
        quat = (q.x, q.y, q.z, q.w)
        self._pose = (xyz, quat)
        self._depth = p.z
        sample = {'x': p.x, 'y': p.y, 'z': p.z}
        if self._gt_start is None:
            self._gt_start = sample
        self._gt_end = sample
        now = time.monotonic()
        if now - self._last_traj_t >= 0.5:
            self._last_traj_t = now
            self._trajectory.append(sample)

    def _on_info(self, cam: str, msg: CameraInfo) -> None:
        self._info[cam] = msg

    def _on_boxes(self, cam: str, msg) -> None:
        with self._lock:
            self._boxes[cam] = msg

    def _label_rows(self, cam: str, w: int, h: int):
        if not self._write_labels:
            return []
        with self._lock:
            msg = self._boxes.get(cam)
        # No boxes yet is NOT the same as no props visible: an empty label file
        # is a legitimate negative and a missing one is a broken frame, so a
        # frame recorded before the first box message must still be a real
        # empty rather than a silently absent label.
        return boxes_to_yolo(msg, w, h) if msg is not None else []

    def _on_image(self, name: str, msg: Image) -> None:
        if self._stop:
            return
        try:
            bgr = _image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(f'{name}: {exc}')
            return

        if self._fx_params is not None:
            bgr = apply_underwater_fx(bgr, self._depth, **self._fx_params)

        h, w = bgr.shape[:2]
        frame = bgr.copy()
        with self._lock:
            if name not in self._sizes:
                self._sizes[name] = (w, h)
                self._encodings[name] = msg.encoding
                self.get_logger().info(
                    f'{name}: {w}x{h} {msg.encoding} ← {self._topics[name]}'
                )
            self._frames[name].append(frame)
            idx = self._counts[name]
            self._counts[name] = idx + 1

        if self._dump_frames:
            path = str(self._out_dir / 'frames' / name / f'{idx:06d}.png')
            try:
                self._disk_q.put_nowait(('png', path, frame))
            except queue.Full:
                pass

        if self._write_labels:
            rows = self._label_rows(name, w, h)
            text = '\n'.join(format_yolo_line(r) for r in rows) + ('\n' if rows else '')
            path = str(self._out_dir / 'labels' / name / f'{idx:06d}.txt')
            try:
                self._disk_q.put_nowait(('label', path, text))
            except queue.Full:
                pass

    def close(self, elapsed_s: float) -> dict:
        self._stop = True
        self._disk_q.join()
        self._disk_q.put(None)
        self._disk_thread.join(timeout=30)

        fps_actual: dict[str, float] = {}
        with self._lock:
            for name, frames in self._frames.items():
                n = len(frames)
                if n == 0:
                    fps_actual[name] = 0.0
                    continue
                fps = n / max(elapsed_s, 1e-3)
                fps = float(max(1.0, min(60.0, fps)))
                fps_actual[name] = round(fps, 3)
                w, h = self._sizes.get(name, (frames[0].shape[1], frames[0].shape[0]))
                path = str(self._out_dir / f'{name}.mp4')
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(path, fourcc, fps, (w, h))
                if not writer.isOpened():
                    self.get_logger().error(f'failed to open VideoWriter for {path}')
                    continue
                for fr in frames:
                    writer.write(fr)
                writer.release()
                self.get_logger().info(f'{name}: wrote {n} frames @ {fps:.2f} fps → {path}')
            self._frames = {c: [] for c in self._cameras}
            return {
                'counts': dict(self._counts),
                'sizes': {k: list(v) for k, v in self._sizes.items()},
                'encodings': dict(self._encodings),
                'topics': dict(self._topics),
                'fps_actual': fps_actual,
                'gt_start': self._gt_start,
                'gt_end': self._gt_end,
                'trajectory': list(self._trajectory),
            }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument(
        '--fps',
        type=float,
        default=20.0,
        help='Hint only; MP4 uses measured fps_actual = count/duration.',
    )
    parser.add_argument('--frames', action='store_true')
    parser.add_argument('--labels', action='store_true', help='Write YOLO GT labels.')
    parser.add_argument('--fx', action='store_true',
                        help="Apply the course's underwater turbidity to the "
                             'recorded frames (in-process; does NOT subscribe '
                             'to image_fx -- see _fx_params_for).')
    parser.add_argument('--cameras', default='front,bottom', help='Comma list: front,bottom.')
    parser.add_argument('--course', default='sauvc26_qualification')
    parser.add_argument(
        '--lighting', default='',
        help="Override the course's water with a named preset "
             '(clear|competition|murky). Use it to capture a murky/clear PAIR '
             'of the same scene from ONE sim launch. Implies --fx.')
    parser.add_argument('--label', default='', help='Run tag prefix (default: course name).')
    parser.add_argument('--outdir', default='')
    parser.add_argument('--script-id', default='', dest='script_id')
    args = parser.parse_args(argv)

    if args.lighting:
        args.fx = True
    cameras = [c.strip() for c in args.cameras.split(',') if c.strip()]
    for c in cameras:
        if c not in ('front', 'bottom'):
            print(f'unknown camera: {c}', file=sys.stderr)
            return 2

    tag = args.label or args.course
    parent = Path(args.outdir) if args.outdir else _workspace_root() / 'datasets'
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    out_dir = parent / f'{tag}_{stamp}'
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f'recording {out_dir}', flush=True)

    rclpy.init()
    node = CameraRecorder(
        out_dir,
        cameras,
        args.frames,
        args.labels,
        args.fx,
        args.fps,
        args.course,
        tag,
        args.lighting,
    )

    stop = {'flag': False}

    def _handle_sig(_signum, _frame):
        stop['flag'] = True
        node.request_stop()

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)
    (out_dir / '.ready').write_text('1\n')
    print(f'ready {out_dir}', flush=True)

    t0 = time.monotonic()
    wall_start = datetime.now(timezone.utc)
    try:
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.duration > 0 and (time.monotonic() - t0) >= args.duration:
                break
    finally:
        (out_dir / '.ready').unlink(missing_ok=True)
        elapsed = time.monotonic() - t0
        stats = node.close(elapsed)
        meta = {
            'label': tag,
            'course': args.course,
            'lighting': args.lighting or None,
            'script_id': args.script_id or None,
            'duration_s': round(elapsed, 3),
            'fps_requested': args.fps,
            'fps_actual': stats.get('fps_actual', {}),
            'frames_dumped': args.frames,
            'labels': args.labels,
            'use_fx': args.fx,
            'cameras': cameras,
            'classes': CLASSES if args.labels else [],
            'utc_start': wall_start.isoformat(),
            'counts': stats['counts'],
            'sizes': stats['sizes'],
            'encodings': stats['encodings'],
            'topics': stats['topics'],
            'gt_start': stats['gt_start'],
            'gt_end': stats['gt_end'],
            'trajectory': stats['trajectory'],
        }
        (out_dir / 'meta.json').write_text(json.dumps(meta, indent=2) + '\n')
        node.destroy_node()
        # Guarded: `ros2 launch` SIGINTs the whole group, and rclpy may have
        # already torn the context down. An unguarded call then raises
        # "rcl_shutdown already called" and the node exits 1 -- a clean
        # ctrl-c reports three processes as DIED.
        if rclpy.ok():
            rclpy.shutdown()
        print(f'wrote {out_dir}')
        print(json.dumps(meta, indent=2))
        if sum(stats['counts'].values()) == 0:
            print(
                'WARNING: no frames received — is the sim + bridge running?',
                file=sys.stderr,
            )
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
