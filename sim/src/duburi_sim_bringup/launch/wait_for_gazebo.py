#!/usr/bin/env python3

"""Block until Gazebo's IMU topic is publishing, then exit.

Startup order is not cosmetic here. The ArduPilot Gazebo plugin opens a UDP
socket and waits; ArduSub connects out to it. If ArduSub starts first it finds
nothing, and rather than failing cleanly it retries against a dead socket while
the EKF initialises on garbage - which surfaces much later as an attitude
solution that never converges.

Waiting on the IMU topic rather than on a fixed sleep is what makes this
reliable: the IMU is the last thing to come up in the vehicle model, and it is
the exact signal ArduSub is about to consume.

Usage:
    wait_for_gazebo.py --world pool_empty --model duburi [--timeout 60]
"""

import argparse
import subprocess
import sys
import time


def imu_topic(world: str, model: str) -> str:
    return (
        f'/world/{world}/model/{model}/link/base_link/sensor/imu_sensor/imu'
    )


def topic_exists(topic: str) -> bool:
    try:
        result = subprocess.run(
            ['gz', 'topic', '-l'],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return topic in result.stdout.split()


def has_data(topic: str, timeout: float) -> bool:
    """Confirm the topic is actually publishing, not merely advertised.

    `gz topic -e -n 1` does terminate on its own, but only after gz-transport
    discovery settles, which takes the better part of ten seconds on a cold
    start. Read whatever it produced and kill it rather than trusting it to
    exit inside any particular window - a short timeout here reads as "Gazebo
    is not up" and stalls the whole launch.
    """
    proc = subprocess.Popen(
        ['gz', 'topic', '-e', '-t', topic, '-n', '1'],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
    return bool(out and out.strip())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True)
    parser.add_argument('--model', default='duburi')
    parser.add_argument('--timeout', type=float, default=90.0)
    args = parser.parse_args()

    topic = imu_topic(args.world, args.model)
    print(f'[wait_for_gazebo] waiting for {topic}', flush=True)

    deadline = time.time() + args.timeout
    while time.time() < deadline:
        if topic_exists(topic) and has_data(topic, timeout=20.0):
            print('[wait_for_gazebo] Gazebo is up, starting ArduSub', flush=True)
            return 0
        time.sleep(1.0)

    print(
        f'[wait_for_gazebo] timed out after {args.timeout:.0f}s. '
        'Gazebo did not publish the vehicle IMU. Check that the world loaded, '
        'that gz-sim-imu-system is in the world, and that the vehicle instance '
        f'is really named "{args.model}".',
        file=sys.stderr,
        flush=True,
    )
    return 1


if __name__ == '__main__':
    sys.exit(main())
