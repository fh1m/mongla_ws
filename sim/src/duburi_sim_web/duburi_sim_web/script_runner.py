#!/usr/bin/env python3

"""Run predefined dataset collection move scripts."""

from __future__ import annotations

import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


@dataclass
class ScriptStatus:
    running: bool = False
    script_id: str = ''
    step: str = ''
    message: str = ''
    record_dir: Optional[str] = None
    ok: Optional[bool] = None
    log: list[str] = field(default_factory=list)


class ScriptRunner:
    def __init__(self, scripts_dir: Path, datasets_dir: Path) -> None:
        self.scripts_dir = scripts_dir
        self.datasets_dir = datasets_dir
        self.status = ScriptStatus()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def list_scripts(self) -> list[dict]:
        out = []
        for path in sorted(self.scripts_dir.glob('*.yaml')):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            out.append(
                {
                    'id': path.stem,
                    'name': data.get('name', path.stem),
                    'description': data.get('description', ''),
                    'course': data.get('course', 'sauvc26_qualification'),
                    'cameras': data.get('cameras', ['front', 'bottom']),
                }
            )
        return out

    def start(
        self,
        script_id: str,
        label: str = '',
        use_fx: bool = True,
        frames: bool = True,
        labels: bool = True,
    ) -> ScriptStatus:
        with self._lock:
            if self.status.running:
                raise RuntimeError('a script is already running')
            path = self.scripts_dir / f'{script_id}.yaml'
            if not path.is_file():
                raise FileNotFoundError(script_id)
            self.status = ScriptStatus(running=True, script_id=script_id, step='starting')
            self._thread = threading.Thread(
                target=self._run,
                args=(path, label or script_id, use_fx, frames, labels),
                daemon=True,
            )
            self._thread.start()
            return self.status

    def _log(self, msg: str) -> None:
        with self._lock:
            self.status.log.append(msg)
            self.status.message = msg

    def _run(self, path: Path, label: str, use_fx: bool, frames: bool, labels: bool) -> None:
        log_path = None
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            course = data.get('course', 'sauvc26_qualification')
            cameras = ','.join(data.get('cameras', ['front', 'bottom']))
            steps = data.get('steps') or []
            duration = float(data.get('record_duration', 45))

            rec_cmd = [
                'ros2', 'run', 'duburi_sim_bridge', 'record_cameras',
                '--duration', str(duration),
                '--course', course,
                '--label', label,
                '--cameras', cameras,
                '--script-id', path.stem,
            ]
            if use_fx:
                rec_cmd.append('--fx')
            if frames:
                rec_cmd.append('--frames')
            if labels:
                rec_cmd.append('--labels')

            self._log(f'recording: {" ".join(rec_cmd)}')
            with self._lock:
                self.status.step = 'record'
            log_f = tempfile.NamedTemporaryFile('w+', delete=False, suffix='.log')
            log_path = Path(log_f.name)
            rec = subprocess.Popen(rec_cmd, stdout=log_f, stderr=subprocess.STDOUT, text=True)

            time.sleep(1.5)
            for step in steps:
                cmd = step.get('cmd')
                if not cmd:
                    continue
                with self._lock:
                    self.status.step = cmd
                args = ['ros2', 'run', 'duburi_planner', 'duburi', cmd]
                for k, v in step.items():
                    if k in ('cmd', 'timeout'):
                        continue
                    args += [f'--{k}', str(v)]
                self._log(' '.join(args))
                proc = subprocess.run(
                    args,
                    capture_output=True,
                    text=True,
                    timeout=float(step.get('timeout', 120)),
                )
                if proc.returncode != 0:
                    self._log(f'FAIL {cmd}: {proc.stderr or proc.stdout}')
                    with self._lock:
                        self.status.ok = False
                        self.status.running = False
                    rec.terminate()
                    return
                self._log(f'ok {cmd}')

            try:
                rec.wait(timeout=duration + 30)
            except subprocess.TimeoutExpired:
                rec.terminate()
            log_f.flush()
            out = log_path.read_text()
            record_dir = None
            for line in out.splitlines():
                if line.startswith('wrote '):
                    record_dir = line[6:].strip()
            with self._lock:
                self.status.record_dir = record_dir
                self.status.ok = rec.returncode == 0
                self.status.running = False
                self.status.step = 'done'
            self._log('script complete')
        except Exception as exc:
            with self._lock:
                self.status.ok = False
                self.status.running = False
                self.status.message = str(exc)
                self.status.log.append(str(exc))
        finally:
            if log_path is not None and log_path.is_file():
                try:
                    log_path.unlink()
                except OSError:
                    pass
