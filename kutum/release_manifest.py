#!/usr/bin/env python3
"""What exactly is running on the vehicle: one JSON file per release.

    python3 kutum/release_manifest.py write RELEASE_DIR COMMIT_DESC
    python3 kutum/release_manifest.py gate RELEASE_DIR EXIT_CODE

`kutum/release.sh` writes it when a release is activated, not before (#64),
and `deploy.sh` adds the vehicle gate's verdict after `bringup_check`. A
reviewer starting from it can rebuild the configuration that ran: the commit,
the board firmware floor the code demands, every model and calibration by
hash, the hull geometry, and the system packages and Hailo runtime under it
(#96, and the vehicle half of #95's bill of materials).

Two states are kept apart on purpose. `software` says the tree is built and
active; `vehicle_gate` says whether the vehicle's own gate passed on that
release. Neither is an evidence level for water: that is earned by a
recorded trial, not by a deploy.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

NAME = 'release.json'
STAMP = '.kutum-deployed'

MODEL_GLOBS = ('*.hef', '*.yaml', '*.onnx', '*.pt')
APT_PACKAGES = ('hailort', 'python3-numpy', 'python3-opencv', 'ros-jazzy-rclpy',
                'ros-jazzy-cv-bridge')
PIP_PACKAGES = ('pymavlink', 'filterpy', 'scipy', 'onnxruntime', 'supervision',
                'trackers', 'hailort', 'numpy')


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def hashes(folder: Path, globs=('*',)) -> dict:
    if not folder.is_dir():
        return {}
    files = sorted({p for g in globs for p in folder.glob(g) if p.is_file()})
    return {p.name: sha256(p) for p in files}


def fw_floor(root: Path):
    """The firmware behaviour revision this code refuses to arm below."""
    p = root / 'src/mongla_control/mongla_control/fc/srot_protocol.py'
    m = re.search(r'^FW_BEHAVIOUR_REV_REQUIRED\s*=\s*(\d+)', p.read_text(), re.M) \
        if p.is_file() else None
    return int(m.group(1)) if m else None


def _run(cmd) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return r.stdout.strip() if r.returncode == 0 else ''
    except (OSError, subprocess.SubprocessError):
        return ''


def apt_versions() -> dict:
    out = {}
    for pkg in APT_PACKAGES:
        v = _run(['dpkg-query', '-W', '-f=${Version}', pkg])
        out[pkg] = v or None
    return out


def pip_versions() -> dict:
    from importlib import metadata
    out = {}
    for pkg in PIP_PACKAGES:
        try:
            out[pkg] = metadata.version(pkg)
        except metadata.PackageNotFoundError:
            out[pkg] = None
    return out


def manifest(root: Path, commit_desc: str) -> dict:
    home = Path.home()
    return {
        'release': root.name,
        'commit': commit_desc,
        'activated_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'host': socket.gethostname(),
        'software': 'built and active',
        'vehicle_gate': 'not run',
        'board_firmware_rev_required': fw_floor(root),
        'models': hashes(root / 'src/mongla_vision/models', MODEL_GLOBS),
        'models_outside_tree': hashes(home / 'hailo_models'),
        'calibration': hashes(root / 'src/mongla_vision/config/calibration', ('*.json',)),
        'hull_geometry': hashes(root / 'src/mongla_control/config', ('hull_geometry.yaml',)),
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'hailortcli': _run(['hailortcli', '--version']) or None,
        'apt': apt_versions(),
        'pip': pip_versions(),
    }


def write(root: Path, commit_desc: str) -> dict:
    doc = manifest(root, commit_desc)
    (root / NAME).write_text(json.dumps(doc, indent=2) + '\n')
    (root / STAMP).write_text(f'{commit_desc} {doc["activated_utc"]} release {root.name}\n')
    return doc


def record_gate(root: Path, code: int) -> dict:
    p = root / NAME
    doc = json.loads(p.read_text())
    doc['vehicle_gate'] = 'pass' if code == 0 else f'fail (bringup_check exit {code})'
    doc['vehicle_gate_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    p.write_text(json.dumps(doc, indent=2) + '\n')
    return doc


def main(argv) -> int:
    if len(argv) == 4 and argv[1] == 'write':
        doc = write(Path(argv[2]).resolve(), argv[3])
        print(f'ok manifest: {len(doc["models"])} model files, '
              f'{len(doc["calibration"])} calibrations, firmware floor '
              f'{doc["board_firmware_rev_required"]}')
        return 0
    if len(argv) == 4 and argv[1] == 'gate':
        doc = record_gate(Path(argv[2]).resolve(), int(argv[3]))
        print(f'ok vehicle gate recorded: {doc["vehicle_gate"]}')
        return 0
    print('usage: release_manifest.py write DIR COMMIT | gate DIR CODE', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
