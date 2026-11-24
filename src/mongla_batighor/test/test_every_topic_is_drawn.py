"""Everything Mongla publishes is drawn by some view, or says why not.

Read against a recorded graph (kutum_graph.txt), not against the code's own
idea of what runs: the 2026-11-25 census found target_pose_near, the
downward poses, the forward anchor, the model's classes and flow's
de-rotation rates published and drawn nowhere.
"""
import pathlib
import re

from mongla_batighor import views

PKG = pathlib.Path(__file__).resolve().parents[1]

# The graph has two topics the source scan in test_views never sees; every
# other topic not drawn is in views.NOT_SHOWN, by its last segment, with why.
GRAPH_ONLY = {
    '/mongla/move/_action/status': 'the action server goal states; its feedback and the verb are drawn',
    '/tf_static': 'Lichtblick reads it itself for every 3D panel',
}


def _graph():
    lines = (PKG / 'test' / 'kutum_graph.txt').read_text().splitlines()
    return [ln.split()[0] for ln in lines if ln.startswith('/')]


def _drawn(topic, layouts, code):
    # Whole names only: /x/target_pose is not drawn by /x/target_pose_near.
    whole = re.escape(topic) + r'(?![\w/])'
    if re.search(whole, layouts) or re.search(whole, code):
        return True
    parts = topic.split('/')
    # A camera's topics are built from its name: `${V}/${c}/lock`, f'{c}/lock'.
    if topic.startswith('/mongla/vision/') and len(parts) > 4 and parts[3] in ('forward', 'downward'):
        leaf = '/'.join(parts[4:])
        return bool(re.search(r'\}/' + re.escape(leaf) + r'(?![\w/])', code))
    return False


def test_every_published_topic_is_drawn_or_says_why_not():
    layouts = ''.join(p.read_text() for p in (PKG / 'layouts').glob('*.json'))
    # views.py is read as the layouts it writes: its own text names every
    # topic a view once drew, and a passing test would hide a dropped one.
    code = ''.join(p.read_text() for p in [*(PKG / 'panels').rglob('*.js'),
                                            *(PKG / 'mongla_batighor').rglob('*.py')]
                   if p.name != 'views.py')
    graph = _graph()
    assert len(graph) > 50, 'the recorded graph is missing or cut'
    excused = lambda t: t in GRAPH_ONLY or t.rsplit('/', 1)[-1] in views.NOT_SHOWN  # noqa: E731
    unread = [t for t in graph if not excused(t) and not _drawn(t, layouts, code)]
    assert not unread, f'published and drawn by no view (draw it, or say why in views.NOT_SHOWN): {unread}'
    stale = [t for t in GRAPH_ONLY if t not in graph]
    assert not stale, f'GRAPH_ONLY names topics the graph does not have: {stale}'
