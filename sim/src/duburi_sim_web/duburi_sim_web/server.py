#!/usr/bin/env python3

"""FastAPI operator lab for the Duburi simulator."""

from __future__ import annotations

import asyncio
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import zipfile
from pathlib import Path
from typing import List, Optional

import yaml
from fastapi import BackgroundTasks, FastAPI, File, HTTPException, Request, UploadFile
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from duburi_sim_web.ros_bridge import RosWorker
from duburi_sim_web.script_runner import ScriptRunner
from duburi_sim_web.joystick import JoystickReader
from duburi_sim_web.teleop import TeleopStreamer

HERE = Path(__file__).resolve().parent
PKG = HERE.parent


def _workspace_root() -> Path:
    # Shared with record_cameras via duburi_sim_bridge.paths -- see that module
    # for why the old name-match + cwd fallback was deleted. duburi_sim_web
    # already exec_depends on duburi_sim_bridge, so this is not a new edge.
    from duburi_sim_bridge.paths import sim_ws_root

    return sim_ws_root()


def _rt(name: str) -> Path:
    """A side-channel file under /tmp/duburi-$USER/ (see paths.runtime_dir)."""
    from duburi_sim_bridge.paths import runtime_dir

    return runtime_dir() / name


def _scripts_dir() -> Path:
    env = os.environ.get('DUBURI_SIM_SCRIPTS')
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('duburi_sim_web')) / 'scripts'
    except Exception:
        return PKG / 'scripts'


def _static_dir() -> Optional[Path]:
    candidates = [
        PKG / 'static',
        HERE / 'static',
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.insert(0, Path(get_package_share_directory('duburi_sim_web')) / 'static')
    except Exception:
        pass
    for c in candidates:
        if (c / 'index.html').is_file():
            return c
    return None


def _courses_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('duburi_sim_worlds')) / 'courses'
    except Exception:
        return _workspace_root() / 'src' / 'duburi_sim_worlds' / 'courses'


worker = RosWorker()
runner = ScriptRunner(_scripts_dir(), _workspace_root() / 'datasets')
teleop = TeleopStreamer()


def _joystick_button(action: str) -> None:
    """Arm/disarm from a pad button.

    Deliberately routed through the same handlers the UI buttons use, so a
    stick cannot reach the vehicle by a path the operator's own clicks do not.
    """
    try:
        if action == 'arm':
            vehicle_arm()
        elif action == 'disarm':
            vehicle_disarm()
    except Exception:
        pass


# A gamepad plugged into the machine running the lab. The other supported
# place for one is the machine running the BROWSER, which the UI reads with
# the Gamepad API and POSTs to /api/vehicle/teleop -- the QGC arrangement,
# where the ground station reads the stick. Both drive the same TeleopStreamer,
# so there is never a second RC writer.
joystick = JoystickReader(
    teleop,
    device=os.environ.get('DUBURI_JOYSTICK') or None,
    on_button=_joystick_button,
    logger=lambda m: print(m, flush=True),
)
_record_proc: Optional[subprocess.Popen] = None
_record_lock = threading.Lock()
_record_meta: dict = {}
_sim_lock = threading.Lock()
_sim_job: dict = {
    'running': False,
    'phase': 'idle',
    'course': 'sauvc26_qualification',
    'active_course': 'sauvc26_qualification',
    'error': None,
    'log': '',
    'gui': True,
    'stack': True,
}
_prop_manager_proc: Optional[subprocess.Popen] = None

app = FastAPI(title='Duburi Sim Lab', version='0.3.0')


class FxBody(BaseModel):
    turbidity: Optional[float] = None
    backscatter: Optional[float] = None
    blur_sigma: Optional[float] = None
    noise: Optional[float] = None
    vignette: Optional[float] = None
    enabled: Optional[bool] = None
    use_fx_feed: Optional[bool] = None


# ArduSub modes this lab may select. An allowlist, not a passthrough:
# `target_name` lands in argv. SURFACE is deliberately absent (it is an
# emergency action with its own verb) and so is POSHOLD (it needs DVL/EKF
# params the sim does not set).
ALLOWED_MODES = ('MANUAL', 'STABILIZE', 'ALT_HOLD', 'ACRO', 'DEPTH_HOLD')


class MoveBody(BaseModel):
    cmd: str
    duration: Optional[float] = None
    gain: Optional[float] = None
    target: Optional[float] = None
    timeout: Optional[float] = None
    # The only STRING argument the lab passes through. `set_mode` needs
    # --target_name, and every other field here is numeric -- which is exactly
    # why mode changes were unreachable from the browser: the request reached
    # argparse with a required argument missing and died with exit 2.
    target_name: Optional[str] = None


class TeleopBody(BaseModel):
    fwd: float = 0.0
    lat: float = 0.0
    up: float = 0.0
    yaw: float = 0.0
    gain: Optional[float] = None


class RecordBody(BaseModel):
    cameras: List[str] = Field(default_factory=lambda: ['front', 'bottom'])
    duration: float = 0.0  # 0 = until stop
    label: str = 'lab'
    name: Optional[str] = None
    course: str = ''
    frames: bool = True
    labels: bool = True
    fx: bool = True


class ScriptBody(BaseModel):
    script_id: str
    label: str = ''
    use_fx: bool = True
    frames: bool = True
    labels: bool = True


class PropBody(BaseModel):
    model: str
    name: str
    x: float
    y: float
    z: Optional[float] = None
    yaw: float = 0.0


class PropMoveBody(BaseModel):
    name: str
    x: float
    y: float
    z: Optional[float] = None
    yaw: float = 0.0


_prop_instances: dict = {}  # name -> {model, x, y, z, yaw}
_prop_instances_lock = threading.Lock()


class SimBody(BaseModel):
    course: str = 'sauvc26_qualification'
    gui: bool = True
    stack: bool = True


def _gz_up() -> bool:
    return subprocess.run(['pgrep', '-f', 'gz sim'], capture_output=True).returncode == 0


def _ardusub_up() -> bool:
    return subprocess.run(['pgrep', '-x', 'ardusub'], capture_output=True).returncode == 0


def _manager_up() -> bool:
    return (
        subprocess.run(['pgrep', '-f', 'lib/duburi_manager/start'], capture_output=True).returncode
        == 0
    )


def _set_job(**kwargs) -> None:
    with _sim_lock:
        _sim_job.update(kwargs)
        course = _sim_job.get('active_course') or _sim_job.get('course')
    if course:
        try:
            _rt('lab_active_course.txt').write_text(str(course) + '\n', encoding='utf-8')
        except OSError:
            pass


def _job_snapshot() -> dict:
    with _sim_lock:
        return {
            'running': _sim_job['running'],
            'phase': _sim_job['phase'],
            'course': _sim_job['course'],
            'active_course': _sim_job['active_course'],
            'error': _sim_job['error'],
            'log_tail': (_sim_job['log'] or '')[-2000:],
            'gui': _sim_job['gui'],
            'stack': _sim_job['stack'],
        }


def _ensure_prop_manager(course: str) -> None:
    """Restart prop_manager pinned to the active Gazebo world name."""
    global _prop_manager_proc
    # Kill prior prop_manager processes started by the lab.
    #
    # NOT `bash -lc`. A LOGIN shell sources the system profile, and on a container
    # whose profile prompts interactively (this one runs a first-time `passwd`
    # setup) that prompt blocks on stdin forever. This call runs inside the lab's
    # startup handler, so the whole API hung with the port bound and connections
    # piling up in the accept queue -- it looked like a wedged event loop, not a
    # shell waiting for a password. Nothing here needs a login shell; there is no
    # shell syntax left to justify one either.
    subprocess.run(
        ['pkill', '-f', 'lib/duburi_sim_scenarios/prop_manager'],
        check=False, stdin=subprocess.DEVNULL,
    )
    time.sleep(0.5)
    log = open(_rt('prop_manager.log'), 'a')
    _prop_manager_proc = subprocess.Popen(
        [
            'ros2',
            'run',
            'duburi_sim_scenarios',
            'prop_manager',
            '--ros-args',
            '-p',
            f'world:={course}',
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _wait_sim_ready(timeout: float = 90.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _gz_up() and _ardusub_up():
            return True
        time.sleep(1.0)
    return False


def _validate_course(course: str) -> None:
    courses = {p.stem for p in _courses_dir().glob('*.yaml')}
    if course not in courses:
        raise HTTPException(400, f'unknown course: {course}')


def _run_sim_bringup(course: str, gui: bool, with_stack: bool, do_stop: bool) -> None:
    chunks: list[str] = []
    try:
        teleop.set_axes(0, 0, 0, 0)
        if do_stop:
            _set_job(phase='stopping', error=None)
            stop = subprocess.run(
                ['ros2', 'run', 'duburi_sim_bringup', 'duburi_sim', 'stop'],
                capture_output=True,
                text=True,
                timeout=60,
            )
            chunks.append(stop.stdout or '')
            chunks.append(stop.stderr or '')
            time.sleep(1.5)

        _set_job(phase='starting_sim', course=course, gui=gui, stack=with_stack)
        sim_args = ['ros2', 'run', 'duburi_sim_bringup', 'duburi_sim', 'sim', f'course:={course}']
        if not gui:
            sim_args.append('--headless')
        log_path = _rt('lab_sim_restart.log')
        log_f = open(log_path, 'w')
        subprocess.Popen(
            sim_args,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        chunks.append(f'sim launched course={course} gui={gui} log={log_path}')

        _set_job(phase='waiting_ready', log='\n'.join(chunks))
        if not _wait_sim_ready(90.0):
            raise RuntimeError(f'sim not ready (gz/ardusub) within 90s — see {log_path}')

        _set_job(phase='prop_manager')
        try:
            _ensure_prop_manager(course)
            chunks.append(f'prop_manager world:={course}')
        except Exception as exc:
            chunks.append(f'prop_manager warn: {exc}')

        if with_stack:
            _set_job(phase='starting_stack', log='\n'.join(chunks))
            # Brief settle after ardusub heartbeat appears.
            time.sleep(3.0)
            stack_log = _rt('lab_stack_restart.log')
            stack_f = open(stack_log, 'w')
            subprocess.Popen(
                ['ros2', 'run', 'duburi_sim_bringup', 'duburi_sim', 'stack', '--no-vision'],
                stdout=stack_f,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            chunks.append(f'stack launched log={stack_log}')

        _set_job(
            running=False,
            phase='ready',
            active_course=course,
            course=course,
            error=None,
            log='\n'.join(chunks),
        )
    except Exception as exc:
        _set_job(
            running=False,
            phase='error',
            error=str(exc),
            log='\n'.join(chunks),
        )


def _start_sim_job(body: SimBody, do_stop: bool) -> dict:
    if _sim_job.get('running'):
        raise HTTPException(409, 'sim job already in progress')
    _validate_course(body.course)
    if not do_stop and _gz_up():
        raise HTTPException(409, 'simulator already running — use restart to change course')
    _set_job(
        running=True,
        phase='queued',
        course=body.course,
        error=None,
        gui=body.gui,
        stack=body.stack,
        log='',
    )
    threading.Thread(
        target=_run_sim_bringup,
        args=(body.course, body.gui, body.stack, do_stop),
        daemon=True,
    ).start()
    return {
        'ok': True,
        'course': body.course,
        'gui': body.gui,
        'stack': body.stack,
        'mode': 'restart' if do_stop else 'start',
    }


@app.on_event('startup')
def _startup():
    worker.start()
    teleop.start()
    joystick.start()
    # Best-effort prop_manager for the default course.
    try:
        if not _manager_up():
            pass
        _ensure_prop_manager(_sim_job['active_course'])
    except Exception:
        pass


@app.on_event('shutdown')
def _shutdown():
    joystick.stop()
    teleop.set_axes(0, 0, 0, 0)
    teleop.stop()


@app.get('/api/health')
def health():
    return {'ok': True, 'teleop': teleop.snapshot(), 'sim': _job_snapshot()}


@app.get('/api/sim/status')
def sim_status():
    snap = worker.node.snapshot() if worker.node else {}
    cams = snap.get('cameras') or {}
    return {
        'gz': _gz_up(),
        'ardusub': _ardusub_up(),
        'manager': _manager_up(),
        'lab_ros': worker.node is not None,
        'teleop': teleop.snapshot(),
        'joystick': joystick.snapshot(),
        'restart': _job_snapshot(),  # alias for older UI
        'sim': _job_snapshot(),
        'active_course': _job_snapshot()['active_course'],
        'link': {
            'gz': _gz_up(),
            'sitl': _ardusub_up(),
            'mav': bool((snap.get('state') or {}).get('have_state')),
            'cams': bool(cams.get('front') or cams.get('bottom')),
            'teleop': bool(teleop.snapshot().get('connected')),
        },
        **snap,
    }


@app.post('/api/sim/stop')
def sim_stop():
    teleop.set_axes(0, 0, 0, 0)
    _set_job(phase='stopping')
    subprocess.run(['ros2', 'run', 'duburi_sim_bringup', 'duburi_sim', 'stop'], check=False)
    _set_job(phase='stopped', running=False, error=None)
    return {'ok': True}


@app.post('/api/sim/start')
def sim_start(body: SimBody):
    return _start_sim_job(body, do_stop=False)


@app.post('/api/sim/restart')
def sim_restart(body: SimBody):
    return _start_sim_job(body, do_stop=True)


@app.get('/api/course')
def list_courses():
    courses = []
    for path in sorted(_courses_dir().glob('*.yaml')):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        courses.append(
            {
                'id': path.stem,
                'name': data.get('name', path.stem),
                'description': (data.get('description') or '').strip().split('\n')[0],
            }
        )
    return {
        'courses': courses,
        'active_course': _job_snapshot()['active_course'],
        'note': 'POST /api/sim/restart to switch course (stop+start).',
    }


@app.get('/api/vehicle/state')
def vehicle_state():
    if worker.node is None:
        raise HTTPException(503, 'ROS lab node not ready')
    snap = worker.node.snapshot()
    snap['teleop'] = teleop.snapshot()
    return snap


_VERB_RE = re.compile(r'^[a-z][a-z0-9_]*$')


def _duburi_cmd(verb: str, *extra: str) -> list[str]:
    """Build a shell invocation that sources duburi_ws then runs the planner CLI.

    The lab binds a socket and takes `cmd` straight off the wire, so the verb is
    attacker-controlled. It is passed to bash as a POSITIONAL ARGUMENT and expanded
    with "$@" -- never concatenated into the script text. bash parses the script
    once, before $1.. are substituted, so `arm; rm -rf ~` arrives at the CLI as one
    argv entry and dies in argparse instead of running.

    Deliberately NOT validated against duburi_control.commands.COMMANDS: that import
    is only available when duburi_ws happens to be on PYTHONPATH, so the check would
    silently evaporate in exactly the deployments that need it most, while the argv
    boundary above cannot fail. The regex is a fast 400 for junk, not the defence.
    """
    from duburi_sim_bridge.paths import duburi_ws_root

    if not _VERB_RE.match(verb):
        raise HTTPException(400, f'invalid verb {verb!r}')

    duburi_ws = str(duburi_ws_root())
    script = (
        'source /opt/ros/humble/setup.bash && '
        'test -f "$0/install/setup.bash" && source "$0/install/setup.bash"; '
        'exec ros2 run duburi_planner duburi "$@"'
    )
    return ['bash', '-c', script, duburi_ws, verb, *extra]


@app.post('/api/vehicle/cmd')
def vehicle_cmd(body: MoveBody):
    extra = []
    if body.duration is not None:
        extra += ['--duration', str(body.duration)]
    if body.gain is not None:
        extra += ['--gain', str(body.gain)]
    if body.target is not None:
        extra += ['--target', str(body.target)]
    if body.timeout is not None:
        extra += ['--timeout', str(body.timeout)]
    if body.target_name is not None:
        name = body.target_name.strip().upper()
        if name not in ALLOWED_MODES:
            raise HTTPException(
                400, f'mode {body.target_name!r} not allowed; '
                     f'choose one of {", ".join(ALLOWED_MODES)}')
        extra += ['--target_name', name]
    args = _duburi_cmd(body.cmd, *extra)
    # stdin=DEVNULL: this runs in the request path with a 180 s timeout. Anything
    # downstream that decides to prompt would otherwise hold a worker for the
    # full timeout instead of failing immediately.
    proc = subprocess.run(args, capture_output=True, text=True, timeout=180,
                          stdin=subprocess.DEVNULL)
    if proc.returncode != 0:
        raise HTTPException(400, proc.stderr or proc.stdout or 'command failed')
    return {'ok': True, 'stdout': proc.stdout}


@app.get('/api/vehicle/modes')
def vehicle_modes():
    return {'modes': list(ALLOWED_MODES)}


@app.post('/api/vehicle/mode')
def vehicle_mode(body: MoveBody):
    """Set the ArduSub flight mode. `target_name` is the mode; `cmd` is ignored."""
    if not body.target_name:
        raise HTTPException(400, 'target_name is required')
    return vehicle_cmd(MoveBody(cmd='set_mode', target_name=body.target_name))


@app.post('/api/vehicle/arm')
def vehicle_arm():
    return vehicle_cmd(MoveBody(cmd='arm'))


@app.post('/api/vehicle/disarm')
def vehicle_disarm():
    teleop.set_axes(0, 0, 0, 0)
    return vehicle_cmd(MoveBody(cmd='disarm'))


@app.post('/api/vehicle/teleop')
def vehicle_teleop(body: TeleopBody):
    snap = teleop.set_axes(body.fwd, body.lat, body.up, body.yaw, body.gain)
    return {'ok': True, **snap}


@app.get('/api/vehicle/teleop')
def vehicle_teleop_status():
    return teleop.snapshot()


@app.get('/api/vehicle/joystick')
def vehicle_joystick():
    """State of a gamepad attached to the LAB machine.

    A pad attached to the browser's machine does not appear here -- the UI
    reads that one itself and reports it separately. Raw axis and button
    indices are included because js numbering depends on which controls a pad
    declares (a pad without triggers shifts every later index), which is the
    same reason QGC ships a joystick calibration page.
    """
    return joystick.snapshot()


@app.get('/api/fx')
def get_fx():
    if worker.node is None:
        raise HTTPException(503, 'ROS lab node not ready')
    return worker.node.snapshot()['fx'] | {'use_fx_feed': worker.node._use_fx}


@app.post('/api/fx')
def set_fx(body: FxBody):
    if worker.node is None:
        raise HTTPException(503, 'ROS lab node not ready')
    data = body.model_dump(exclude_none=True)
    use_feed = data.pop('use_fx_feed', None)
    if use_feed is not None:
        worker.node._rewire_cameras(bool(use_feed))
    if data:
        result = worker.node.set_fx_params(data)
        if isinstance(result, tuple):
            params, fut = result
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 2.0:
                time.sleep(0.05)
        else:
            params = result
    else:
        params = worker.node._fx_params
    return params | {'use_fx_feed': worker.node._use_fx}


@app.get('/api/cameras/{cam}/mjpeg')
async def mjpeg(cam: str, request: Request):
    """Camera stream.

    MUST stay `async`. Starlette runs a plain `def` route in a 40-slot thread pool,
    and this handler is an unbounded `while True`, so every open stream used to pin
    one thread for the life of the process -- with no disconnect check, closing the
    tab did not give it back. The UI opens two streams; ~20 page reloads wedged the
    whole API, including `disarm`. As a coroutine it costs no thread at all, and
    `is_disconnected()` ends it when the browser goes away.
    """
    if cam not in ('front', 'bottom'):
        raise HTTPException(404, 'cam must be front or bottom')

    async def gen():
        last_seq = -1
        idle = 0
        while True:
            if await request.is_disconnected():
                return
            jpeg = None
            seq = -1
            if worker.node is not None:
                with worker.node._lock:
                    jpeg = worker.node._jpeg.get(cam)
                    seq = worker.node._jpeg_seq.get(cam, 0)
            if jpeg and seq != last_seq:
                last_seq = seq
                idle = 0
                yield (
                    b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
                )
            else:
                idle += 1
            await asyncio.sleep(0.033 if idle < 5 else 0.05)

    return StreamingResponse(gen(), media_type='multipart/x-mixed-replace; boundary=frame')


@app.get('/api/cameras/{cam}/jpeg')
def jpeg_once(cam: str):
    if cam not in ('front', 'bottom'):
        raise HTTPException(404)
    jpeg = worker.node.latest_jpeg(cam) if worker.node else None
    if not jpeg:
        raise HTTPException(404, 'no frame yet')
    return Response(content=jpeg, media_type='image/jpeg')


@app.post('/api/record/start')
def record_start(body: RecordBody):
    global _record_proc, _record_meta
    with _record_lock:
        if _record_proc is not None and _record_proc.poll() is None:
            raise HTTPException(409, 'recording already in progress')
        label = (body.name or body.label or 'lab').strip().replace(' ', '_')
        course = (body.course or '').strip() or _job_snapshot()['active_course']
        args = [
            'ros2', 'run', 'duburi_sim_bridge', 'record_cameras',
            '--course', course,
            '--label', label,
            '--cameras', ','.join(body.cameras),
        ]
        if body.duration and body.duration > 0:
            args += ['--duration', str(body.duration)]
        if body.fx:
            args.append('--fx')
        if body.frames:
            args.append('--frames')
        if body.labels:
            args.append('--labels')
        # New session so stop can SIGINT the whole ros2-run process group
        # (plain SIGINT on the parent often leaves record_cameras orphaned).
        _record_proc = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        _record_meta = {
            'args': args,
            'started': time.time(),
            'duration': body.duration,
            'label': label,
            'course': course,
            'cameras': body.cameras,
        }
    return {'ok': True, 'args': args, 'label': label, 'course': course}


def _datasets_matching(label: str) -> List[Path]:
    root = _workspace_root() / 'datasets'
    if not root.is_dir() or not label:
        return []
    prefix = f'{label}_'
    return [p for p in root.iterdir() if p.is_dir() and p.name.startswith(prefix)]


def _parse_record_dir(out: str, label: str = '') -> Optional[str]:
    recording = None
    for line in (out or '').splitlines():
        if line.startswith('wrote '):
            return line[6:].strip()
        if line.startswith('recording ') and recording is None:
            recording = line[len('recording '):].strip() or None
    if recording:
        return recording
    candidates = _datasets_matching(label)
    if not candidates:
        return None
    newest = max(candidates, key=lambda p: p.stat().st_mtime)
    return str(newest)


@app.post('/api/record/stop')
def record_stop():
    global _record_proc
    import signal as _signal

    with _record_lock:
        if _record_proc is None:
            return {'ok': True, 'message': 'not recording'}
        proc = _record_proc
        label = str(_record_meta.get('label') or '')
        # Wait until recorder installed SIGINT handlers (.ready marker).
        ready_deadline = time.time() + 45.0
        while time.time() < ready_deadline:
            dirs = _datasets_matching(label)
            if any((d / '.ready').is_file() for d in dirs):
                break
            if proc.poll() is not None:
                break
            time.sleep(0.25)
        try:
            os.killpg(proc.pid, _signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            try:
                proc.send_signal(_signal.SIGINT)
            except ProcessLookupError:
                pass
        out = ''
        try:
            out, _ = proc.communicate(timeout=30)
        except Exception:
            try:
                os.killpg(proc.pid, _signal.SIGKILL)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
            try:
                out, _ = proc.communicate(timeout=5)
            except Exception:
                out = out or ''
        code = proc.returncode
        _record_proc = None
    record_dir = _parse_record_dir(out, label)
    # Wait briefly for meta.json if the process flushed the dir but not stdout.
    if record_dir:
        meta = Path(record_dir) / 'meta.json'
        for _ in range(40):
            if meta.is_file():
                break
            time.sleep(0.25)
    meta_obj = None
    if record_dir:
        meta_path = Path(record_dir) / 'meta.json'
        if meta_path.is_file():
            try:
                meta_obj = json.loads(meta_path.read_text(encoding='utf-8'))
            except Exception:
                meta_obj = None
    ok = record_dir is not None and meta_obj is not None
    return {
        'ok': ok,
        'record_dir': record_dir,
        'stdout': out,
        'code': code,
        'meta': meta_obj,
    }


@app.get('/api/record/status')
def record_status():
    with _record_lock:
        running = _record_proc is not None and _record_proc.poll() is None
        return {'running': running, **_record_meta}


@app.get('/api/scripts')
def list_scripts():
    return {'scripts': runner.list_scripts()}


@app.post('/api/scripts/run')
def run_script(body: ScriptBody):
    try:
        st = runner.start(body.script_id, body.label, body.use_fx, body.frames, body.labels)
    except FileNotFoundError:
        raise HTTPException(404, 'unknown script')
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    return {'ok': True, 'status': st.__dict__}


@app.get('/api/scripts/status')
def script_status():
    return runner.status.__dict__


@app.get('/api/props/catalog')
def props_catalog():
    try:
        from duburi_sim_scenarios.prop_catalog import load_prop_library, prop_anchor, prop_names

        library = load_prop_library()
        names = prop_names(library)
        models = [
            {
                'id': name,
                'anchor': prop_anchor(library, name) or 'floor',
                'source': 'library',
            }
            for name in names
        ]
        seen = set(names)
        for root in _models_dirs():
            for child in sorted(root.iterdir()):
                if not child.is_dir() or child.name in seen:
                    continue
                if (child / 'model.sdf').is_file():
                    models.append({'id': child.name, 'anchor': 'floor', 'source': 'custom'})
                    seen.add(child.name)
        return {'models': models, 'active_course': _job_snapshot()['active_course']}
    except Exception as exc:
        return {'models': [], 'error': str(exc)}


@app.get('/api/props/list')
def props_list():
    """Catalog listing (legacy). Prefer /api/props/catalog + /api/props/instances."""
    proc = subprocess.run(
        ['ros2', 'run', 'duburi_sim_scenarios', 'props', 'list'],
        capture_output=True,
        text=True,
        timeout=30,
    )
    return {'ok': proc.returncode == 0, 'stdout': proc.stdout, 'stderr': proc.stderr}


@app.get('/api/props/instances')
def props_instances():
    with _prop_instances_lock:
        items = [
            {'name': n, **meta}
            for n, meta in sorted(_prop_instances.items())
        ]
    return {'instances': items, 'active_course': _job_snapshot()['active_course']}


@app.post('/api/props/spawn')
def props_spawn(body: PropBody):
    args = [
        'ros2', 'run', 'duburi_sim_scenarios', 'props', 'add',
        body.model, body.name, str(body.x), str(body.y),
    ]
    if body.z is not None:
        args += ['--z', str(body.z)]
    if body.yaw:
        args += ['--yaw', str(body.yaw)]
    proc = subprocess.run(args, capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        raise HTTPException(400, proc.stderr or proc.stdout)
    with _prop_instances_lock:
        _prop_instances[body.name] = {
            'model': body.model,
            'x': body.x,
            'y': body.y,
            'z': body.z,
            'yaw': body.yaw,
        }
    return {'ok': True, 'stdout': proc.stdout}


@app.post('/api/props/move')
def props_move(body: PropMoveBody):
    args = [
        'ros2', 'run', 'duburi_sim_scenarios', 'props', 'move',
        body.name, str(body.x), str(body.y),
    ]
    if body.z is not None:
        args += ['--z', str(body.z)]
    args += ['--yaw', str(body.yaw)]
    proc = subprocess.run(args, capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        raise HTTPException(400, proc.stderr or proc.stdout)
    with _prop_instances_lock:
        cur = _prop_instances.get(body.name, {'model': '?'})
        cur.update({'x': body.x, 'y': body.y, 'z': body.z, 'yaw': body.yaw})
        _prop_instances[body.name] = cur
    return {'ok': True, 'stdout': proc.stdout}


@app.post('/api/props/remove/{name}')
def props_remove(name: str):
    proc = subprocess.run(
        ['ros2', 'run', 'duburi_sim_scenarios', 'props', 'remove', name],
        capture_output=True,
        text=True,
        timeout=60,
    )
    if proc.returncode != 0:
        raise HTTPException(400, proc.stderr or proc.stdout)
    with _prop_instances_lock:
        _prop_instances.pop(name, None)
    return {'ok': True}


def _models_dirs() -> list[Path]:
    dirs = []
    src = _workspace_root() / 'src' / 'duburi_sim_worlds' / 'models'
    if src.is_dir():
        dirs.append(src)
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('duburi_sim_worlds')) / 'models'
        if share.is_dir() and share.resolve() != src.resolve():
            dirs.append(share)
    except Exception:
        pass
    return dirs


@app.post('/api/assets/upload')
async def assets_upload(file: UploadFile = File(...)):
    """Upload a prop model zip containing model.sdf into duburi_sim_worlds/models."""
    if not file.filename or not file.filename.lower().endswith('.zip'):
        raise HTTPException(400, 'upload a .zip containing model.sdf (+ meshes)')
    dest_root = _workspace_root() / 'src' / 'duburi_sim_worlds' / 'models'
    dest_root.mkdir(parents=True, exist_ok=True)
    raw = await file.read()
    try:
        zf = zipfile.ZipFile(io.BytesIO(raw))
    except zipfile.BadZipFile as exc:
        raise HTTPException(400, f'invalid zip: {exc}') from exc
    # Detect model folder: either zip root is the model, or one top-level dir.
    names = [n for n in zf.namelist() if not n.endswith('/')]
    sdf_members = [n for n in names if n.endswith('model.sdf') or n.endswith('/model.sdf')]
    if not sdf_members:
        raise HTTPException(400, 'zip must contain model.sdf')
    sdf_path = sorted(sdf_members, key=len)[0]
    parts = Path(sdf_path).parts
    if len(parts) == 1:
        model_id = Path(file.filename).stem.replace(' ', '_')
        prefix = ''
    else:
        model_id = parts[0]
        prefix = parts[0] + '/'
    if not model_id or model_id.startswith('.'):
        raise HTTPException(400, 'invalid model id')
    target = dest_root / model_id
    target.mkdir(parents=True, exist_ok=True)
    for member in names:
        if prefix and not member.startswith(prefix):
            continue
        rel = member[len(prefix):] if prefix else member
        # `..` alone is not enough: pathlib DISCARDS the left operand when the
        # right side is absolute, so `target / '/etc/cron.d/x'` escapes silently.
        if not rel or '..' in Path(rel).parts or Path(rel).is_absolute():
            continue
        out = target / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        with zf.open(member) as src, open(out, 'wb') as dst:
            dst.write(src.read())
    # Also copy into install share if present (symlink install often shares src).
    for extra in _models_dirs():
        if extra.resolve() == dest_root.resolve():
            continue
        try:
            shutil.copytree(target, extra / model_id, dirs_exist_ok=True)
        except Exception:
            pass
    return {
        'ok': True,
        'model_id': model_id,
        'path': str(target),
        'note': 'Custom models appear after catalog refresh; first use may need sim restart for gz resource path.',
    }


def _dataset_integrity(path, meta):
    """Do the frames on disk match what meta.json claims?

    record_cameras buffers frames in RAM and DROPS png/label writes when the
    queue fills, without erroring -- so the indices desync silently and the
    directory still looks finished. "It exists" proves nothing; the counts have
    to agree. This is the same check the README documents, run per-clip so a bad
    recording is caught in the UI instead of after training on it.
    """
    counts = (meta or {}).get('counts') or {}
    if not counts:
        return {'state': 'unknown', 'detail': 'no counts in meta.json'}
    if not (meta or {}).get('frames_dumped'):
        return {'state': 'ok', 'detail': 'video only (no frames dumped)'}
    bad = []
    for cam, n in counts.items():
        try:
            frames = sum(1 for _ in (path / 'frames' / cam).iterdir())
        except OSError:
            bad.append(f'{cam}: frames/ missing')
            continue
        if frames != n:
            bad.append(f'{cam}: {frames} frames vs meta {n}')
        if (meta or {}).get('labels'):
            try:
                labels = sum(1 for _ in (path / 'labels' / cam).iterdir())
            except OSError:
                bad.append(f'{cam}: labels/ missing')
                continue
            if labels != n:
                bad.append(f'{cam}: {labels} labels vs meta {n}')
    if bad:
        return {'state': 'mismatch', 'detail': '; '.join(bad)}
    return {'state': 'ok', 'detail': 'frames == labels == meta.counts'}


@app.get('/api/datasets')
def list_datasets():
    import json

    root = _workspace_root() / 'datasets'
    root.mkdir(parents=True, exist_ok=True)
    runs = []
    for path in root.iterdir():
        if not path.is_dir():
            continue
        meta_path = path / 'meta.json'
        if not meta_path.is_file():
            # Skip in-progress / aborted runs (no finalize).
            continue
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            meta = {}
        runs.append(
            {
                'id': path.name,
                'path': str(path),
                'size_mb': round(sum(f.stat().st_size for f in path.rglob('*') if f.is_file()) / 1e6, 2),
                'meta': meta,
                'integrity': _dataset_integrity(path, meta),
                'mtime': path.stat().st_mtime,
            }
        )
    # Newest first (utc_start when present, else directory mtime).
    runs.sort(key=lambda r: (r['meta'].get('utc_start') or '', r['mtime']), reverse=True)
    for r in runs:
        r.pop('mtime', None)
    return {'datasets': runs}


@app.get('/api/datasets/{run_id}/zip')
def download_dataset(run_id: str, background_tasks: BackgroundTasks):
    """Zip on disk then stream file (avoids holding multi-100MB in RAM)."""
    root = _workspace_root() / 'datasets' / run_id
    if not root.is_dir():
        raise HTTPException(404)
    fd, tmp_path = tempfile.mkstemp(prefix=f'{run_id}_', suffix='.zip')
    os.close(fd)
    try:
        with zipfile.ZipFile(tmp_path, 'w', zipfile.ZIP_DEFLATED) as zf:
            for f in root.rglob('*'):
                if f.is_file():
                    zf.write(f, f.relative_to(root))
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise
    background_tasks.add_task(lambda p=tmp_path: os.unlink(p) if os.path.exists(p) else None)
    return FileResponse(
        tmp_path,
        media_type='application/zip',
        filename=f'{run_id}.zip',
        background=None,
    )


# Static UI last so /api wins.
_static = _static_dir()
if _static is not None:
    _static_real = Path(os.path.realpath(_static))
    assets = _static_real / 'assets'
    if assets.is_dir():
        app.mount(
            '/assets',
            StaticFiles(directory=str(assets), follow_symlink=True),
            name='assets',
        )
    # Logo and other public files next to index.
    for name in ('ue-logo.png', 'favicon.ico'):
        pass

    @app.get('/')
    def spa_index():
        return FileResponse(_static_real / 'index.html')

    @app.get('/{spa_path:path}')
    def spa_fallback(spa_path: str):
        if spa_path.startswith('api/'):
            raise HTTPException(404)
        # Containment is checked LEXICALLY (normpath), not with resolve().
        #
        # resolve() follows symlinks, and `colcon build --symlink-install` makes
        # every file under static/ a symlink into build/ -> src/. So resolve()
        # walked the target OUT of the install tree, is_relative_to() said "not
        # contained", and every static file except /assets/* (served by the
        # StaticFiles mount, not this handler) silently fell through to
        # index.html -- /ue-logo.png returned HTML and the header logo and
        # favicon were broken images.
        #
        # normpath collapses `..` textually, which is what actually defeats
        # traversal, and leaves symlinks alone. A symlink inside static/ that
        # points outside would still be served -- acceptable, because static/ is
        # our own build output and colcon deliberately fills it with exactly
        # such symlinks. Attacker-controlled uploads are a different handler and
        # have their own check.
        candidate = Path(os.path.normpath(_static_real / spa_path))
        if spa_path and candidate.is_relative_to(_static_real) and candidate.is_file():
            return FileResponse(candidate)
        return FileResponse(_static_real / 'index.html')
else:

    @app.get('/')
    def index_fallback():
        return JSONResponse(
            {
                'message': 'UI not built yet. API is live.',
                'hint': 'cd frontend && npm install && npm run build',
            }
        )


def _claim_lab_port(host: str, preferred: int):
    """Bind+listen and keep the socket (Electron races if we probe-then-release)."""
    import socket

    candidates = [preferred]
    for offset in range(0, 80):
        p = preferred + offset if preferred >= 28000 else 28765 + offset
        if p not in candidates:
            candidates.append(p)
    # Also try a high fallback away from Cursor port-forwards.
    for p in (28999, 29001, 29111):
        if p not in candidates:
            candidates.append(p)

    for p in candidates:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # host is normalised by the caller; '0.0.0.0' is the ONLY wildcard.
            sock.bind((host, p))
            sock.listen(2048)
            return sock, p
        except OSError:
            try:
                sock.close()
            except OSError:
                pass
    raise RuntimeError(f'no free lab port near {preferred}')


def main(argv=None) -> int:
    import uvicorn

    # Loopback by default -- the lab is unauthenticated and arms thrusters.
    # DUBURI_LAB_HOST=0.0.0.0 is the explicit opt-in for a topside laptop.
    #
    # `or`, NOT os.environ.get(k, default): a SET-BUT-EMPTY var is a real case
    # (`DUBURI_LAB_HOST= duburi_sim lab`, an unset key in docker --env-file, a
    # bare `DUBURI_LAB_HOST=` in .env or systemd Environment=). get() returns ''
    # there because the key exists, and '' used to fall through to the 0.0.0.0
    # wildcard below -- while the shell wrapper's ${VAR:-default} DOES substitute
    # on empty and printed "LOOPBACK ONLY". Same variable, two parsers, opposite
    # answers, and the reassuring one was wrong: the lab bound every interface
    # while saying it had not.
    host = (os.environ.get('DUBURI_LAB_HOST') or '127.0.0.1').strip() or '127.0.0.1'
    preferred = int(os.environ.get('DUBURI_LAB_PORT', '28765'))
    sock, port = _claim_lab_port(host, preferred)
    if port != preferred:
        print(f'[lab_server] port {preferred} busy; using {port}', flush=True)
    print(f'[lab_server] listening on http://{host}:{port}', flush=True)
    os.environ['DUBURI_LAB_PORT'] = str(port)
    try:
        _rt('lab_port.txt').write_text(str(port) + '\n', encoding='utf-8')
    except OSError:
        pass
    # Pass the already-bound listening socket so Cursor/Electron cannot steal it.
    uvicorn.run(app, fd=sock.fileno(), log_level='info')
    return 0


if __name__ == '__main__':
    sys.exit(main())
