#!/usr/bin/env python3
"""How long the vehicle is blind while its vision process starts: frame and detection.

The restart policy (`mongla_localization/launch_respawn.py`, measured-bars
"The restart policy") claims the mission keeps running after a vision process
dies. The cost is the blind time: the restart delay plus how long the process
takes from exec to its first frame and first detection. This measures the
second term, which is what a restart pays on top of the delay.

Each cycle starts the launch in its own process group, records the first
message on the camera's `image_raw` and `detections` topics, then stops the
group with SIGINT and waits for it to exit. Times are from the `Popen` call.

    # on Kutum, board disarmed, a camera attached:
    python3 restart_blind_time.py --cycles 5 -- \\
        ros2 launch mongla_vision vision_pi.launch.py paused:=false

    python3 restart_blind_time.py --self-test   # proves the scorer first

What would make the number wrong: a subscriber that joins late (it is created
before the launch starts, and discovery is part of what is measured, as it is
for a real consumer), or a previous cycle still alive (each cycle waits for its
group to exit and the next refuses while a matching process remains).
"""
from __future__ import annotations

import argparse
import os
import signal
import statistics
import subprocess
import sys
import time

FWD = '/mongla/vision/{camera}/{what}'


def _measure(cmd: list[str], camera: str, timeout_s: float, node, spin) -> dict:
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray
    from mongla_vision import qos

    first: dict[str, float] = {}
    boxes = [0]
    t0 = time.monotonic()

    def mark(name):
        def cb(msg):
            if name not in first:
                first[name] = time.monotonic() - t0
            if name == 'detections' and msg.detections:
                boxes[0] += 1
        return cb

    subs = [node.create_subscription(Image, FWD.format(camera=camera, what='image_raw'),
                                     mark('image_raw'), qos.IMAGE),
            node.create_subscription(Detection2DArray,
                                     FWD.format(camera=camera, what='detections'),
                                     mark('detections'), qos.DETECTIONS)]
    t0 = time.monotonic()
    proc = subprocess.Popen(cmd, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        while time.monotonic() - t0 < timeout_s and len(first) < 2:
            if proc.poll() is not None:
                break
            spin(0.05)
    finally:
        exit_early = proc.poll()
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        for s in subs:
            node.destroy_subscription(s)
    return {'image_raw_s': first.get('image_raw'), 'detections_s': first.get('detections'),
            'boxes': boxes[0], 'exited_early': exit_early}


def _still_running(pattern: str) -> bool:
    return subprocess.run(['pgrep', '-f', pattern], capture_output=True).returncode == 0


def run(cmd: list[str], cycles: int, camera: str, timeout_s: float,
        settle_s: float, guard: str | None) -> list[dict]:
    import rclpy
    rclpy.init(args=[])     # the launch command's `name:=value` is not ours
    node = rclpy.create_node('restart_blind_time')

    def spin(t):
        rclpy.spin_once(node, timeout_sec=t)
    out = []
    try:
        for i in range(cycles):
            if guard and _still_running(guard):
                raise SystemExit(f'ERROR: a {guard!r} process is still running; '
                                 f'the cycle would measure a warm start')
            r = _measure(cmd, camera, timeout_s, node, spin)
            out.append(r)
            print(f'cycle {i + 1}: first frame {_fmt(r["image_raw_s"])}  first '
                  f'detections {_fmt(r["detections_s"])}  boxes {r["boxes"]}'
                  + (f'  EXITED EARLY ({r["exited_early"]})'
                     if r['exited_early'] is not None else ''), flush=True)
            time.sleep(settle_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return out


def _fmt(v):
    return '--' if v is None else f'{v:6.2f} s'


def summary(results: list[dict]) -> str:
    lines = []
    for key in ('image_raw_s', 'detections_s'):
        vals = [r[key] for r in results if r[key] is not None]
        missing = len(results) - len(vals)
        if vals:
            lines.append(f'{key}: worst {max(vals):.2f} s  median '
                         f'{statistics.median(vals):.2f} s  best {min(vals):.2f} s'
                         f'  (n={len(vals)}' + (f', {missing} never arrived)' if missing else ')'))
        else:
            lines.append(f'{key}: never arrived in {len(results)} cycles')
    return '\n'.join(lines)


_FAKE = r'''
import sys, time, rclpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from mongla_vision import qos
delay = float(sys.argv[1])
rclpy.init(); n = rclpy.create_node('fake_vision')
time.sleep(delay)
pi = n.create_publisher(Image, '/mongla/vision/forward/image_raw', qos.IMAGE)
pd = n.create_publisher(Detection2DArray, '/mongla/vision/forward/detections', qos.DETECTIONS)
while rclpy.ok():
    pi.publish(Image()); pd.publish(Detection2DArray()); time.sleep(0.03)
'''


def self_test() -> int:
    """The scorer against a fake launch whose delay was chosen: it must report it."""
    ok = True
    for delay in (0.5, 2.5):
        r = run([sys.executable, '-c', _FAKE, str(delay)], cycles=2,
                camera='forward', timeout_s=15, settle_s=0.5, guard=None)
        for c in r:
            for key in ('image_raw_s', 'detections_s'):
                v = c[key]
                # Discovery adds time, never removes it: never early, and
                # within 1.5 s of the chosen delay on a quiet machine.
                good = v is not None and delay <= v <= delay + 1.5
                ok &= good
                print(f'self-test delay {delay}: {key} {_fmt(v)} '
                      f'{"ok" if good else "FAIL"}')
    print('self-test', 'ok' if ok else 'FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--cycles', type=int, default=5)
    ap.add_argument('--camera', default='forward')
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--settle', type=float, default=5.0,
                    help='seconds between cycles, so devices are released')
    ap.add_argument('--guard', default='detector_dual_node',
                    help='refuse a cycle while a process matching this runs')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('cmd', nargs=argparse.REMAINDER)
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    cmd = a.cmd[1:] if a.cmd[:1] == ['--'] else a.cmd
    if not cmd:
        ap.error('give the launch command after --')
    results = run(cmd, a.cycles, a.camera, a.timeout, a.settle, a.guard)
    print(summary(results))
    return 0


if __name__ == '__main__':
    sys.exit(main())
