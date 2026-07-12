"""mission_web -- browser mission-control console for the Duburi vision stack.

One ROS2 node that:
  * subscribes the per-camera detection/data topics + /duburi/state + the latched
    /duburi/vision/active_camera, and polls the topic-less detector params
    (active_model / conf / models / paused) at 1 Hz,
  * serves a self-contained single-page console (video via web_video_server) with
    a Server-Sent-Events data feed, and
  * writes vision control through the SAME surface the mission DSL writes --
    SetParameters on /duburi_detector_<cam> + the latched active_camera publish +
    pause-others/resume-target exclusivity -- so a switch from the UI and a switch
    from a running DSL mission are the same operation and stay in sync for free.

Start the whole thing (cameras + detectors + web_video_server + this node, browser
auto-opens) with:  ros2 launch duburi_vision mission_web.launch.py
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from duburi_interfaces.msg import DuburiState

from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from .dashboard_state import (build_snapshot, param_value_for,
                              active_camera_targets)
from ..detection.messages import _msg_bbox_center as bbox_center


_KNOWN_CAMERAS = ('forward', 'downward')      # matches DSL _KNOWN_CAMERAS
_FPS_EMA = 0.3                                 # detection-rate smoothing
_POLL_S = 1.0                                  # param/graph poll period
_SVC_TIMEOUT_S = 2.0                           # per SetParameters/GetParameters call


def _resolve_static_dir() -> Path:
    """Locate the SPA assets across build styles.

    Regular colcon build installs the static files to the package SHARE dir (via
    setup.py data_files); a symlink-install / running-from-source has them next to
    this module. Prefer whichever actually holds index.html.
    """
    candidates = [Path(__file__).parent / 'static']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory('duburi_vision')) / 'web' / 'static')
    except Exception:                             # noqa: BLE001 -- not installed yet
        pass
    for c in candidates:
        if (c / 'index.html').is_file():
            return c
    return candidates[0]


_STATIC_DIR = _resolve_static_dir()


def _param_value(ros_type: str, value: Any) -> ParameterValue:
    """Build an rcl_interfaces ParameterValue from a (type, value) pair."""
    pv = ParameterValue()
    if ros_type == 'double':
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
    elif ros_type == 'integer':
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)
    elif ros_type == 'bool':
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(value)
    else:
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(value)
    return pv


class MissionWebNode(Node):
    def __init__(self):
        super().__init__('duburi_mission_web')
        self.declare_parameter('cameras', ','.join(_KNOWN_CAMERAS))
        self.declare_parameter('web_port', 8090)
        self.declare_parameter('video_port', 8080)

        cams = str(self.get_parameter('cameras').value).strip()
        self.cameras: List[str] = [c.strip() for c in cams.split(',') if c.strip()]
        self.web_port = int(self.get_parameter('web_port').value)
        self.video_port = int(self.get_parameter('video_port').value)

        self._cbg = ReentrantCallbackGroup()   # lets service calls complete while spinning
        self._lock = threading.Lock()
        self._store: Dict[str, Any] = {
            'cameras': {c: self._blank_cam() for c in self.cameras},
            'active_camera': None,
            'state': {},
            'ts': 0.0,
        }
        self._param_clients: Dict[str, Any] = {}
        self._active_cam_pub = None

        best = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        for cam in self.cameras:
            ns = f'/duburi/vision/{cam}'
            self.create_subscription(Detection2DArray, f'{ns}/detections',
                                     self._mk_det_cb(cam), best,
                                     callback_group=self._cbg)
            self.create_subscription(String, f'{ns}/classes_filter',
                                     self._mk_classes_cb(cam), latched,
                                     callback_group=self._cbg)
            self.create_subscription(Float32MultiArray, f'{ns}/vis_range',
                                     self._mk_vis_cb(cam), best,
                                     callback_group=self._cbg)
            self.create_subscription(CameraInfo, f'{ns}/camera_info',
                                     self._mk_info_cb(cam), best,
                                     callback_group=self._cbg)

        self.create_subscription(String, '/duburi/vision/active_camera',
                                 self._active_cam_cb, latched,
                                 callback_group=self._cbg)
        self.create_subscription(DuburiState, '/duburi/state',
                                 self._state_cb, 10, callback_group=self._cbg)

        # Param/graph poll runs on its own thread (call_async + wait), so a slow or
        # paused detector can't stall a ROS timer/callback.
        self._stop = threading.Event()
        threading.Thread(target=self._poll_loop, daemon=True).start()
        self.get_logger().info(
            f"[WEB  ] mission console: cameras={self.cameras} "
            f"web:{self.web_port} video:{self.video_port}")

    # ---- store scaffolding -------------------------------------------------
    @staticmethod
    def _blank_cam() -> Dict[str, Any]:
        return {'present': False, 'paused': None, 'active_model': '', 'models': [],
                'conf': None, 'classes': [], 'fps': 0.0, 'frame': None,
                'dets': [], '_last_det': 0.0, 'vis': []}

    # ---- subscription callbacks -------------------------------------------
    def _mk_det_cb(self, cam: str):
        def cb(msg: Detection2DArray):
            now = time.monotonic()
            dets = []
            with self._lock:
                c = self._store['cameras'][cam]
                vis = c.get('vis', [])
                for i, d in enumerate(msg.detections):
                    if not d.results:
                        continue
                    hyp = d.results[0]
                    if hasattr(hyp, 'hypothesis'):
                        cls, score = str(hyp.hypothesis.class_id), float(hyp.hypothesis.score)
                    else:
                        cls, score = str(getattr(hyp, 'id', '')), float(getattr(hyp, 'score', 0.0))
                    cx, cy = bbox_center(d.bbox)
                    dets.append({'cls': cls, 'conf': score, 'cx': cx, 'cy': cy,
                                 'w': float(d.bbox.size_x), 'h': float(d.bbox.size_y),
                                 'vis': vis[i] if i < len(vis) else None,
                                 'id': str(getattr(d, 'id', '') or '')})
                dt = now - c['_last_det']
                if 0.0 < dt < 5.0:
                    inst = 1.0 / dt
                    c['fps'] = inst if c['fps'] <= 0 else (1 - _FPS_EMA) * c['fps'] + _FPS_EMA * inst
                c['_last_det'] = now
                c['dets'] = dets
        return cb

    def _mk_classes_cb(self, cam: str):
        def cb(msg: String):
            classes = [s.strip() for s in str(msg.data).split(',') if s.strip()]
            with self._lock:
                self._store['cameras'][cam]['classes'] = classes
        return cb

    def _mk_vis_cb(self, cam: str):
        def cb(msg: Float32MultiArray):
            with self._lock:
                self._store['cameras'][cam]['vis'] = list(msg.data)
        return cb

    def _mk_info_cb(self, cam: str):
        def cb(msg: CameraInfo):
            with self._lock:
                self._store['cameras'][cam]['frame'] = (int(msg.width), int(msg.height))
        return cb

    def _active_cam_cb(self, msg: String):
        with self._lock:
            self._store['active_camera'] = str(msg.data).strip()

    def _state_cb(self, msg: DuburiState):
        import math
        def _n(v):  # NaN -> None so the UI shows "--" not "NaN"
            return None if isinstance(v, float) and math.isnan(v) else v
        with self._lock:
            self._store['state'] = {
                'armed': bool(msg.armed), 'mode': str(msg.mode),
                'yaw': _n(float(msg.yaw_deg)), 'depth': _n(float(msg.depth_m)),
                'batt': _n(float(msg.battery_voltage)),
            }

    # ---- param / graph poll ------------------------------------------------
    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            try:
                names = set(self.get_node_names())
            except Exception:            # noqa: BLE001 -- graph read best-effort
                names = set()
            for cam in self.cameras:
                present = f'duburi_detector_{cam}' in names
                got = self._get_detector_params(cam) if present else {}
                with self._lock:
                    c = self._store['cameras'][cam]
                    c['present'] = present
                    if present:
                        c['active_model'] = got.get('active_model', c['active_model'])
                        c['conf'] = got.get('conf', c['conf'])
                        c['models'] = got.get('models', c['models'])
                        c['paused'] = got.get('paused', c['paused'])
                    else:
                        c['fps'] = 0.0
            self._stop.wait(_POLL_S)

    def _get_detector_params(self, cam: str) -> Dict[str, Any]:
        """Read the topic-less detector params (active_model/conf/models/paused)."""
        node = f'/duburi_detector_{cam}'
        cli = self._client(GetParameters, f'{node}/get_parameters')
        if cli is None or not cli.wait_for_service(timeout_sec=0.3):
            return {}
        req = GetParameters.Request(names=['active_model', 'conf', 'models', 'paused'])
        resp = self._call(cli, req)
        if resp is None or len(resp.values) < 4:
            return {}
        am, conf, models, paused = resp.values[0], resp.values[1], resp.values[2], resp.values[3]
        out: Dict[str, Any] = {}
        if am.type == ParameterType.PARAMETER_STRING:
            out['active_model'] = am.string_value
        if conf.type == ParameterType.PARAMETER_DOUBLE:
            out['conf'] = conf.double_value
        if models.type == ParameterType.PARAMETER_STRING:
            # "name=stem,name2=stem2" (or bare stems) -> the switchable keys.
            keys = []
            for part in str(models.string_value).split(','):
                part = part.strip()
                if part:
                    keys.append(part.split('=', 1)[0].strip())
            out['models'] = keys
        if paused.type == ParameterType.PARAMETER_BOOL:
            out['paused'] = paused.bool_value
        return out

    # ---- control (mirrors the DSL surface) --------------------------------
    # Whitelist of live-settable detector params -- refuse anything else so a
    # bogus/typo'd param name can't reach SetParameters (declared-only params
    # would be rejected by the node anyway, but this fails fast + clearly).
    _SETTABLE = ('conf', 'model_conf', 'active_model', 'classes', 'max_det', 'paused')

    def set_detector_param(self, cam: str, name: str, value: Any) -> Dict[str, Any]:
        """Never raises -- returns {ok, reason}. A control write on pool day must
        fail loud-but-safe, never crash the handler thread or drop silently."""
        try:
            if cam not in self.cameras:
                return {'ok': False, 'reason': f'unknown camera {cam!r}'}
            if name not in self._SETTABLE:
                return {'ok': False, 'reason': f'param {name!r} not live-settable '
                                               f'({", ".join(self._SETTABLE)})'}
            # Coerce+validate BEFORE the service wait so a bad value fails instantly.
            try:
                ros_type, coerced = param_value_for(name, value)
            except ValueError as exc:
                return {'ok': False, 'reason': str(exc)}

            node = f'/duburi_detector_{cam}'
            cli = self._client(SetParameters, f'{node}/set_parameters')
            if cli is None or not cli.wait_for_service(timeout_sec=1.0):
                return {'ok': False, 'reason': f'{node}/set_parameters unavailable'}
            req = SetParameters.Request(parameters=[
                Parameter(name=name, value=_param_value(ros_type, coerced))])
            resp = self._call(cli, req)
            if resp is None or not resp.results:
                return {'ok': False, 'reason': 'timed out'}
            res = resp.results[0]
            self.get_logger().info(
                f"[WEB  ] {node} {name} → {coerced!r} "
                f"({'ok' if res.successful else 'REJECT: ' + res.reason})")
            return {'ok': bool(res.successful), 'reason': res.reason}
        except Exception as exc:                  # noqa: BLE001 -- last-resort guard
            self.get_logger().warning(f"[WEB  ] set_detector_param crashed: {exc!r}")
            return {'ok': False, 'reason': f'internal error: {exc}'}

    def switch_active_camera(self, target: str) -> Dict[str, Any]:
        """Latched publish + pause-others/resume-target -- identical to DSL use_camera.
        Never raises; the HUD-follow publish is best-effort (mirrors the DSL)."""
        try:
            if target not in self.cameras:
                return {'ok': False, 'reason': f'unknown camera {target!r}'}
            self._publish_active_camera(target)
            try:
                names = set(self.get_node_names())
            except Exception:                # noqa: BLE001
                names = set()
            if f'duburi_detector_{target}' not in names:
                # Indicator follows (latched), but be honest that nothing resumed.
                return {'ok': False, 'reason': f'{target} detector not running '
                                               f'(indicator set; no stream to resume)'}
            for other in active_camera_targets(target, _KNOWN_CAMERAS):
                if f'duburi_detector_{other}' in names:
                    self.set_detector_param(other, 'paused', True)
            result = self.set_detector_param(target, 'paused', False)
            self.get_logger().info(f"[WEB  ] active_camera → {target!r}")
            return result
        except Exception as exc:                  # noqa: BLE001 -- last-resort guard
            self.get_logger().warning(f"[WEB  ] switch_active_camera crashed: {exc!r}")
            return {'ok': False, 'reason': f'internal error: {exc}'}

    def _publish_active_camera(self, name: str) -> None:
        if self._active_cam_pub is None:
            qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._active_cam_pub = self.create_publisher(
                String, '/duburi/vision/active_camera', qos)
        self._active_cam_pub.publish(String(data=str(name)))

    # ---- service plumbing (call_async + wait; safe under MultiThreadedExecutor) --
    def _client(self, srv_type, path: str):
        cli = self._param_clients.get(path)
        if cli is None:
            cli = self.create_client(srv_type, path, callback_group=self._cbg)
            self._param_clients[path] = cli
        return cli

    def _call(self, cli, req):
        fut = cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(_SVC_TIMEOUT_S):
            return None
        return fut.result()

    # ---- snapshot for SSE --------------------------------------------------
    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            self._store['ts'] = time.time()
            # deep-ish copy of the mutable bits the builder reads
            store = {
                'cameras': {n: dict(c) for n, c in self._store['cameras'].items()},
                'active_camera': self._store['active_camera'],
                'state': dict(self._store['state']),
                'ts': self._store['ts'],
            }
        return build_snapshot(store, self.video_port)

    def stop(self) -> None:
        self._stop.set()


# ---- HTTP layer -----------------------------------------------------------
def _make_handler(node: MissionWebNode):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *a):           # silence stdlib access log
            return

        def _send(self, code: int, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/' or path == '/index.html':
                return self._serve_static('index.html', 'text/html')
            if path.startswith('/static/'):
                return self._serve_static(path[len('/static/'):], None)
            if path == '/events':
                return self._serve_events()
            self._send(404, b'not found', 'text/plain')

        def _serve_static(self, rel: str, ctype: Optional[str]):
            # Path-traversal guard: resolve and confirm it stays under _STATIC_DIR.
            target = (_STATIC_DIR / rel).resolve()
            if not str(target).startswith(str(_STATIC_DIR.resolve())) or not target.is_file():
                return self._send(404, b'not found', 'text/plain')
            if ctype is None:
                ctype = {'.js': 'text/javascript', '.css': 'text/css',
                         '.html': 'text/html'}.get(target.suffix, 'application/octet-stream')
            self._send(200, target.read_bytes(), ctype)

        def _serve_events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while not node._stop.is_set():
                    try:
                        payload = json.dumps(node.snapshot()).encode()
                    except Exception:         # noqa: BLE001 -- skip a bad frame, keep the stream
                        time.sleep(1.0 / 12.0)
                        continue
                    self.wfile.write(b'data: ' + payload + b'\n\n')
                    self.wfile.flush()
                    time.sleep(1.0 / 12.0)
            except (BrokenPipeError, ConnectionResetError):
                pass                          # tab closed -- normal

        def do_POST(self):
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except (ValueError, json.JSONDecodeError):
                return self._send(400, b'{"ok":false,"reason":"bad json"}', 'application/json')
            if not isinstance(body, dict):     # JSON 5 / "x" / [..] are not control bodies
                return self._send(400, b'{"ok":false,"reason":"body must be a JSON object"}',
                                  'application/json')
            path = self.path.split('?', 1)[0]
            try:
                result = self._dispatch(path, body)
            except Exception as exc:           # noqa: BLE001 -- no request may wedge the thread
                node.get_logger().warning(f"[WEB  ] POST {path} crashed: {exc!r}")
                return self._send(500, b'{"ok":false,"reason":"internal error"}', 'application/json')
            if result is None:
                return self._send(404, b'{"ok":false,"reason":"no route"}', 'application/json')
            self._send(200, json.dumps(result).encode(), 'application/json')

        def _dispatch(self, path: str, body: dict):
            parts = [p for p in path.split('/') if p]        # ['api','detector',cam,'param']
            if parts[:2] == ['api', 'detector'] and len(parts) == 4 and parts[3] == 'param':
                if 'name' not in body or 'value' not in body:
                    return {'ok': False, 'reason': 'missing name/value'}
                return node.set_detector_param(parts[2], str(body['name']), body['value'])
            if parts == ['api', 'active_camera']:
                if 'camera' not in body:
                    return {'ok': False, 'reason': 'missing camera'}
                return node.switch_active_camera(str(body['camera']))
            return None

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(prog='mission_web')
    ap.add_argument('--no-browser', action='store_true',
                    help="don't auto-open the console in a browser")
    ns, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = MissionWebNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    server = ThreadingHTTPServer(('0.0.0.0', node.web_port), _make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()

    url = f'http://localhost:{node.web_port}'
    node.get_logger().info(f"[WEB  ] console live → {url}  (video on :{node.video_port})")
    if not ns.no_browser and not os.environ.get('MISSION_WEB_NO_BROWSER'):
        try:
            webbrowser.open(url)
        except Exception:                     # noqa: BLE001 -- headless is fine, URL printed
            pass

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
