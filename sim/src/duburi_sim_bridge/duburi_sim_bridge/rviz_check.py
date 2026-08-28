#!/usr/bin/env python3
"""Assert every topic an RViz config references is actually SUBSCRIBED.

An RViz display whose `Topic` is mis-specified does not error. It renders
nothing, sits in the tree looking normal, and the only outward sign is an
unticked checkbox that also happens to be what a user-disabled display looks
like. That is how two camera displays and the ground-truth track shipped dead.

Subscriber count is the signal that separates "wired" from "looks wired": a
display that parsed its topic subscribes to it, and one that did not does not.
So this reads the config, pulls out every topic it names, and checks each one
has at least one subscriber while RViz is running.

    ros2 run duburi_sim_bridge rviz_check                      # installed config
    ros2 run duburi_sim_bridge rviz_check --config path.rviz

Exit 0 only when every referenced topic is subscribed.
"""
import argparse
import subprocess
import sys

import yaml


def topics_in(cfg: dict):
    """(display name, topic) for every display that names one.

    RViz accepts `Topic` BOTH as a plain string and as a mapping with a `Value`
    key, and which one a display wants varies by class -- installed configs
    shipped with Humble use both forms. Rather than guess, accept either here
    and let the subscriber check be the arbiter of whether RViz agreed.
    """
    out = []
    vm = cfg.get('Visualization Manager') or {}
    for d in vm.get('Displays') or []:
        name = d.get('Name', d.get('Class', '?'))
        for key in ('Topic', 'Description Topic'):
            t = d.get(key)
            if isinstance(t, dict):
                t = t.get('Value')
            if isinstance(t, str) and t.startswith('/'):
                out.append((name, t))
    return out


def subscriber_count(topic: str) -> int:
    try:
        out = subprocess.run(['ros2', 'topic', 'info', topic],
                             capture_output=True, text=True, timeout=20).stdout
    except Exception:
        return -1
    for line in out.splitlines():
        if 'Subscription count' in line:
            try:
                return int(line.split(':')[1])
            except ValueError:
                return -1
    return -1


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='')
    args = ap.parse_args(argv)

    path = args.config
    if not path:
        from ament_index_python.packages import get_package_share_directory
        import os
        path = os.path.join(get_package_share_directory('duburi_sim_bringup'),
                            'config', 'duburi.rviz')

    with open(path) as fh:
        cfg = yaml.safe_load(fh)

    pairs = topics_in(cfg)
    if not pairs:
        print(f'no topics referenced by {path}')
        return 1

    print(f'checking {len(pairs)} topics from {path}\n')
    dead = []
    for name, topic in pairs:
        n = subscriber_count(topic)
        # >1 is normal and fine: the lab, the HUD and a bag recorder all
        # subscribe too. Only zero means RViz did not take the topic.
        ok = n > 0
        print(f'  {"OK  " if ok else "DEAD"} {name:26s} {topic:44s} subs={n}')
        if not ok:
            dead.append((name, topic))

    print()
    if dead:
        print(f'{len(dead)} display(s) reference a topic nobody subscribes to.')
        print('If the publisher exists, RViz did not accept the Topic entry --')
        print('check the `Topic` form for that display class (string vs mapping).')
        for name, topic in dead:
            print(f'  {name}: {topic}')
        return 2
    print(f'all {len(pairs)} displays are subscribed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
