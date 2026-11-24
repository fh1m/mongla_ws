"""The ROS graph grouped by stage, for a graph a person can read.

Lichtblick's own topic graph draws every node and topic in one band, and its
topic filter lives in panel state no layout can set (1.27: `useState("all")`),
so with ~25 nodes and ~90 topics it is a line of hairline (bead tci). Here
each node goes into the column of the stage it belongs to, in the order the
chain runs, and the topics between two nodes become one edge that names them.

`group` is pure: the chain node (chain.py) feeds it the graph it reads from
rclpy while someone watches, and publishes the result on `TOPIC`.
"""
from __future__ import annotations

TOPIC = '/mongla/batighor/graph'
PERIOD_S = 2.0

# Column order is the chain's: photons to thrust, then who watches. A node is
# placed by its name's prefix; one that matches nothing is drawn under
# `other`, so a new node is never hidden, only not yet placed.
STAGES = (
    ('camera', ('mongla_camera_',)),
    ('detector', ('mongla_detector_',)),
    ('tracker', ('mongla_tracker_',)),
    ('lock', ('mongla_lock_', 'mongla_anchor')),
    ('pose', ('mongla_pnp_', 'mongla_pose_fuse_')),
    ('navigation', ('mongla_flow', 'mongla_localization', 'mongla_course')),
    ('mission', ('mongla_mission', 'mongla_planner', 'mongla_run_plan', 'mongla_cli')),
    ('manager', ('mongla_manager',)),
    ('batighor', ('batighor_', 'mongla_chain', 'foxglove_bridge')),
)

# Every node has these; drawn, they would join everything to everything.
EVERYWHERE = frozenset({'/rosout', '/parameter_events'})


def stage_of(name: str) -> str:
    bare = name.rsplit('/', 1)[-1]
    for stage, prefixes in STAGES:
        if bare.startswith(prefixes):
            return stage
    return 'other'


def group(nodes) -> dict:
    """`nodes`: (full name, published topics, subscribed topics) per node.

    Returns the columns (each stage with its nodes, sorted) and the edges:
    one per (publisher, subscriber) pair with every topic that joins them.
    A node's own topics never make an edge to itself.
    """
    pubs, subs, cols = {}, {}, {}
    for name, p, s in nodes:
        pubs[name] = set(p) - EVERYWHERE
        subs[name] = set(s) - EVERYWHERE
        cols.setdefault(stage_of(name), []).append(name)
    joined = {}
    for a, out in pubs.items():
        for b, inp in subs.items():
            if a != b and out & inp:
                joined[(a, b)] = sorted(out & inp)
    order = [s for s, _ in STAGES] + ['other']
    return {
        'columns': [{'stage': s, 'nodes': sorted(cols[s])} for s in order if s in cols],
        'edges': [{'from': a, 'to': b, 'topics': t} for (a, b), t in sorted(joined.items())],
        'nodes': len(pubs),
        # A topic only one side of the graph holds: published and unread, or
        # read and unfed. Shown, because an operator debugging asks exactly this.
        'unread': sorted(set().union(*pubs.values()) - set().union(*subs.values())) if pubs else [],
        'unfed': sorted(set().union(*subs.values()) - set().union(*pubs.values())) if subs else [],
    }


def read(node) -> list:
    """The live graph as `group` takes it, from an rclpy node."""
    out = []
    for name, ns in node.get_node_names_and_namespaces():
        full = (ns.rstrip('/') + '/' + name) if ns else '/' + name
        try:
            p = [t for t, _ in node.get_publisher_names_and_types_by_node(name, ns)]
            s = [t for t, _ in node.get_subscriber_names_and_types_by_node(name, ns)]
        except Exception:                                   # noqa: BLE001
            continue        # it left between the two calls
        out.append((full, p, s))
    return out
