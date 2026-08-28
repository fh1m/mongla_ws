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
import re
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


def rviz_subscribes(topic: str):
    """(rviz_is_subscribed, total_subs, other_subscriber_names).

    Counting subscribers is NOT enough, and this function exists because the
    first version of it gave a FALSE PASS: `underwater_fx` also subscribes to
    both camera image_raw topics, so the count was >=1 whether or not RViz had
    the display switched on. The camera displays were in fact disabled and this
    tool said they were fine.

    `ros2 topic info -v` names the subscribing nodes, so ask for RViz by name.
    """
    try:
        out = subprocess.run(['ros2', 'topic', 'info', '-v', topic],
                             capture_output=True, text=True, timeout=25).stdout
    except Exception:
        return False, -1, []

    total = -1
    for line in out.splitlines():
        if 'Subscription count' in line:
            try:
                total = int(line.split(':')[1])
            except ValueError:
                pass

    # The subscriber block lists "Node name: <name>" entries.
    names = re.findall(r'Node name:\s*(\S+)', out)
    # Publishers are listed first; without parsing sections precisely, matching
    # on the rviz node name is what actually answers the question.
    rviz = any('rviz' in n.lower() for n in names)
    others = sorted({n for n in names if 'rviz' not in n.lower()})
    return rviz, total, others


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
        rviz, n, others = rviz_subscribes(topic)
        tag = 'OK  ' if rviz else 'DEAD'
        extra = f'  (other subs: {", ".join(others)})' if others and not rviz else ''
        print(f'  {tag}{name:26s} {topic:44s} rviz={rviz} total={n}{extra}')
        if not rviz:
            dead.append((name, topic))

    print()
    if dead:
        print(f'{len(dead)} display(s) that RViz is NOT subscribed to.')
        print('Either the display is switched off in the saved config (check')
        print('`Enabled:`), or RViz did not accept the Topic entry -- check the')
        print('`Topic` form for that display class (string vs mapping).')
        for name, topic in dead:
            print(f'  {name}: {topic}')
        return 2
    print(f'all {len(pairs)} displays are subscribed BY RVIZ.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
