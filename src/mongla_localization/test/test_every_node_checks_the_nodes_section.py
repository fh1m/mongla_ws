"""Every node refuses a misspelt `nodes:` key in bringup.yaml.

The check (`params_file.refuse_unknown`) only protects the nodes that call it,
and a new node that forgets is exactly the silent case it exists to stop: ROS
ignores the typo and the node runs on its default. So: every module that
defines a ROS node class must call it.
"""
import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]


def _node_modules():
    for p in sorted(SRC.glob('mongla_*/mongla_*/**/*.py')):
        if '/test/' in str(p):
            continue
        tree = ast.parse(p.read_text())
        if any(isinstance(n, ast.ClassDef) and any(getattr(b, 'id', '') == 'Node' for b in n.bases)
               for n in ast.walk(tree)):
            yield p


def test_there_are_nodes_to_check():
    assert len(list(_node_modules())) >= 10


def test_every_node_module_calls_refuse_unknown():
    missing = [str(p.relative_to(SRC)) for p in _node_modules()
               if 'refuse_unknown(' not in p.read_text()]
    assert not missing, ('these nodes ignore a misspelt bringup.yaml `nodes:` key; '
                         f'call params_file.refuse_unknown(node) in main(): {missing}')
