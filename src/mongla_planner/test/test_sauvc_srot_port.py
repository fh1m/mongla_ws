"""What the SAUVC missions ask the board for -- frozen, so a change is a diff.

Every verb that enters AUTO closes the depth loop, which has never run closed:
with the two bench checks not passed, the board denies them. The list below is
which of those each mission calls.
"""


# ── Verbs the BOARD can deny ─────────────────────────────────────────────────
# These are dispatched as SROT_MOVE, which enters AUTO, and AUTO closes the
# depth loop that has never run closed. With the two
# bench checks not passed, the board denies them -- and the vision verbs' creep
# fallback, the blind gate transit and the flare push are all among them.
# Frozen, so a new depth-gated call is a visible diff, not a pool-day surprise.
_DEPTH_GATED_ON_SROT = {
    'sauvc_full':               {'move_back', 'move_forward', 'pause', 'set_depth'},
    'sauvc_navigation':         {'move_forward', 'pause', 'set_depth'},
    'sauvc_qualification':      {'move_forward', 'pause', 'set_depth', 'yaw_right'},
    'sauvc_target_acquisition': {'move_forward', 'pause', 'set_depth'},
}


def test_the_depth_gated_calls_per_mission_are_the_frozen_list():
    import mission_ast as MA
    from mongla_control.fc.srot_fc import MOVE_VERBS
    found = {}
    for m in MA.sauvc_mission_names():
        tree = MA.tree(m)
        found[m] = {v for v in MOVE_VERBS if MA.calls(tree, v)}
    assert found == _DEPTH_GATED_ON_SROT


def test_nothing_moves_the_board_after_surface():
    """`surface()` puts the board in SURFACE. Every MOVE verb -- `stop`
    included -- is a SROT_MOVE, which enters AUTO, so a `stop()` after
    `surface()` takes the hull straight back out of SURFACE and holds it at
    depth. The finally blocks used to end `surface(); stop(); disarm()`.
    """
    import ast
    import mission_ast as MA
    from mongla_control.fc.srot_fc import MOVE_VERBS

    def _is_call(stmt, names):
        return any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                   and n.func.attr in names for n in ast.walk(stmt))

    bad = []
    for m in MA.sauvc_mission_names():
        for node in ast.walk(MA.tree(m)):
            for field in ('body', 'orelse', 'finalbody'):
                block = getattr(node, field, None)
                if not isinstance(block, list):
                    continue
                for i, stmt in enumerate(block):
                    # the statement IS the surface() call -- not a def or a
                    # with-block that contains one somewhere inside
                    if (isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call)
                            and isinstance(stmt.value.func, ast.Attribute)
                            and stmt.value.func.attr == 'surface'):
                        for later in block[i + 1:]:
                            if _is_call(later, MOVE_VERBS):
                                bad.append(f'{m}:{later.lineno}')
    assert not bad, f'a board MOVE follows surface(): {bad}'
