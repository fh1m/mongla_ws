"""A variant estimator must publish to its OWN topic.

Redundancy is only useful if a consumer can tell the estimators apart. If a
`_near` solve arrived on the same topic as the tight one, a disagreement
between them -- which is the evidence the pair exists to produce -- would be
indistinguishable from one estimator being noisy.

Structural: the launch and the node agree on the naming rule, checked without
starting either. The alternative is discovering at the pool that both nodes
took the same topic and one of them silently won.
"""
from __future__ import annotations

import ast
import pathlib

_VISION = pathlib.Path(__file__).resolve().parents[1]
_NODE = _VISION / 'duburi_vision' / 'pnp_node.py'
_LAUNCH = _VISION / 'launch' / 'vision_pi.launch.py'


def test_the_node_suffixes_its_topic_with_the_variant():
    src = _NODE.read_text()
    assert "f'{ns}/target_pose' + (f'_{variant}' if variant else '')" in src, \
        'the topic must carry the variant, or two solvers share one topic'


def test_an_empty_variant_keeps_the_plain_topic():
    # The default solver must not become `/target_pose_` -- every existing
    # consumer subscribes to the plain name.
    src = _NODE.read_text()
    assert "if variant else ''" in src


def _launch_variants() -> list[str]:
    """Every `variant` value the launch passes to a pnp_node."""
    out = []
    for node in ast.walk(ast.parse(_LAUNCH.read_text())):
        if not (isinstance(node, ast.Call)
                and getattr(node.func, 'id', '') == 'Node'):
            continue
        kw = {k.arg: k.value for k in node.keywords}
        exe = kw.get('executable')
        if not (isinstance(exe, ast.Constant) and exe.value == 'pnp_node'):
            continue
        params = kw.get('parameters')
        found = ''
        for sub in ast.walk(params) if params is not None else []:
            if isinstance(sub, ast.Dict):
                for k, v in zip(sub.keys, sub.values):
                    if (isinstance(k, ast.Constant) and k.value == 'variant'
                            and isinstance(v, ast.Constant)):
                        found = v.value
        out.append(found)
    return out


def test_the_launch_defines_one_tight_and_one_near_solver():
    # The solvers are factory functions called per camera, so the AST sees one
    # Node definition each; the call sites below are what make it two cameras.
    variants = _launch_variants()
    assert sorted(variants) == ['', 'near'], (
        f'expected exactly a tight and a near solver, got {variants}')


def test_both_solvers_are_started_for_both_cameras():
    src = _LAUNCH.read_text()
    for call in ("solver('forward')", "solver('downward')",
                 "solver_near('forward')", "solver_near('downward')"):
        assert call in src, f'{call} is defined but never started'


def test_the_near_solver_actually_opens_the_gate():
    # A "redundant" estimator with the SAME threshold is not redundancy, it is
    # a second copy of the first answer at twice the cost.
    src = _LAUNCH.read_text()
    i = src.index('def solver_near')
    body = src[i:i + 1600]
    assert "'variant':       'near'" in body
    assert "'max_reproj_px': 100.0" in body


def test_no_two_pnp_nodes_share_a_name():
    src = _LAUNCH.read_text()
    names = [line for line in src.splitlines() if 'duburi_pnp' in line and 'name=' in line]
    assert len(set(names)) == len(names), f'duplicate pnp node names: {names}'
