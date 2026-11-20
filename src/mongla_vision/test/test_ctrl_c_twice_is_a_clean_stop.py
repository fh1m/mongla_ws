"""A Ctrl-C that arrives twice stops a node cleanly.

A terminal Ctrl-C reaches every process of a launch twice: the terminal
signals the process group and `ros2 launch` forwards its own. On Kutum
(2026-11-19) the second landed inside `destroy_node()`: every Python node of
`vision_pi` printed a KeyboardInterrupt traceback and exited -2, and the
detector was interrupted inside the Hailo VDevice's `__exit__`. The teardown
now starts with `spin.deaf_to_ctrl_c()`. These tests hold the helper's
behaviour and that every node `vision_pi` starts calls it before tearing down.
"""
import os
import re
import signal
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'mongla_localization'))

# Every node `vision_pi.launch.py` starts, and the vision nodes run alone.
NODES = [
    'mongla_vision/mongla_vision/camera_node.py',
    'mongla_vision/mongla_vision/detector_node.py',
    'mongla_vision/mongla_vision/detector_dual_node.py',
    'mongla_vision/mongla_vision/tracker_node.py',
    'mongla_vision/mongla_vision/lock_node.py',
    'mongla_vision/mongla_vision/flow/flow_node.py',
    'mongla_vision/mongla_vision/calibration/calibrate_node.py',
    'mongla_localization/mongla_localization/pnp_node.py',
    'mongla_localization/mongla_localization/pose_fuse_node.py',
    'mongla_localization/mongla_localization/localization_node.py',
]


def test_after_the_call_a_ctrl_c_does_not_raise():
    from mongla_localization.spin import deaf_to_ctrl_c
    before = signal.getsignal(signal.SIGINT)
    try:
        deaf_to_ctrl_c()
        os.kill(os.getpid(), signal.SIGINT)      # would raise KeyboardInterrupt
    finally:
        signal.signal(signal.SIGINT, before)


def test_every_node_goes_deaf_before_it_tears_down():
    for rel in NODES:
        src = (SRC / rel).read_text()
        main = src[src.index('\ndef main'):]
        m = re.search(r'\n(\s+)finally:\n((?:\1\s+.*\n|\s*\n)+)', main)
        assert m, f'{rel}: main has no finally'
        first = [ln.strip() for ln in m.group(2).splitlines() if ln.strip()
                 and not ln.strip().startswith(('#', 'from ', 'import '))][0]
        assert first.startswith('deaf_to_ctrl_c()'), (
            f'{rel}: the teardown starts with {first!r}; a second Ctrl-C would land inside it')
