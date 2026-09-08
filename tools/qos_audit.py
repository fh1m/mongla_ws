#!/usr/bin/env python3
"""Every ROS 2 pub/sub in the stack, with its QoS -- looking for silent mismatches.

The failure this hunts is ROS 2's worst: a BEST_EFFORT publisher and a RELIABLE
subscriber are INCOMPATIBLE. rclpy logs one warning and then delivers nothing
for ever. Clean launch, healthy nodes, zero messages. It has bitten this project
before (`RosTopicCamera` subscribed RELIABLE while every camera publisher is
BEST_EFFORT).

Second failure: a VOLATILE publisher with a TRANSIENT_LOCAL subscriber means a
late joiner never sees the latched value.
"""
import ast, pathlib, collections, re

pubs = collections.defaultdict(list)
subs = collections.defaultdict(list)

def qos_of(node, src):
    """Best-effort QoS description from the call's 3rd arg / qos_profile kwarg."""
    txt = ast.get_source_segment(src, node) or ''
    if 'qos_profile_sensor_data' in txt: return 'SENSOR_DATA(best_effort)'
    if 'BEST_EFFORT' in txt: return 'BEST_EFFORT'
    if 'TRANSIENT_LOCAL' in txt: return 'TRANSIENT_LOCAL(+reliable)'
    if 'RELIABLE' in txt: return 'RELIABLE'
    m = re.search(r',\s*(\d+)\s*\)', txt)
    if m: return f'depth={m.group(1)} (default RELIABLE)'
    m2 = re.search(r',\s*([A-Za-z_][\w\.]*)\s*\)', txt)
    if m2: return f'var:{m2.group(1)}'
    return 'unknown'

def topic_of(node, src):
    if len(node.args) >= 2:
        a = node.args[1]
        if isinstance(a, ast.Constant): return str(a.value)
        seg = ast.get_source_segment(src, a) or '?'
        return seg[:44]
    return '?'

for f in sorted(pathlib.Path('src').rglob('*.py')):
    p = str(f)
    if '/test' in p or '__pycache__' in p: continue
    try: src = f.read_text(encoding='utf-8'); tree = ast.parse(src)
    except SyntaxError: continue
    for n in ast.walk(tree):
        if not isinstance(n, ast.Call) or not isinstance(n.func, ast.Attribute):
            continue
        if n.func.attr == 'create_publisher':
            pubs[topic_of(n, src)].append((p.replace('src/',''), n.lineno, qos_of(n, src)))
        elif n.func.attr == 'create_subscription':
            subs[topic_of(n, src)].append((p.replace('src/',''), n.lineno, qos_of(n, src)))

BEST = ('BEST_EFFORT', 'SENSOR_DATA(best_effort)')
print(f'publishers: {sum(len(v) for v in pubs.values())}   subscribers: {sum(len(v) for v in subs.values())}\n')
print('=== TOPICS WITH BOTH ENDS IN-TREE (the only ones we can check) ===')
bad = 0
for t in sorted(set(pubs) & set(subs)):
    pq = {q for _,_,q in pubs[t]}
    sq = {q for _,_,q in subs[t]}
    incompatible = any(p in BEST for p in pq) and any('RELIABLE' in s and 'best_effort' not in s for s in sq)
    flag = '  <-- INCOMPATIBLE: pub BEST_EFFORT / sub RELIABLE = NO DATA' if incompatible else ''
    if incompatible: bad += 1
    print(f'{t:44s} pub={sorted(pq)}  sub={sorted(sq)}{flag}')
print(f'\nincompatible pairs found: {bad}')
