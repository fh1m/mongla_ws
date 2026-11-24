"""Which of a node's parameters change while it runs, and a refusal for the rest.

`ros2 param set` and the DSL's `set_node` report success for ANY declared
parameter. A node that read the value once at startup carries on unchanged,
so the success is a lie the mission acts on. Every node a mission can reach
registers here: the names it can apply live, each with the function that
applies it, and a reasoned refusal for every other name.
"""
from rcl_interfaces.msg import SetParametersResult
from mongla_runtime.tether import changes


def apply_live(node, live: dict, *, restart_hint='set it under `nodes:` in bringup.yaml and restart'):
    """Register the callback. `live` maps a parameter name to a function of
    the new value; a function that raises ValueError refuses with its message.
    Sets `node.LIVE_PARAMS` so a test can compare a claim against it."""
    node.LIVE_PARAMS = tuple(sorted(live))

    def on_set(params):
        params = changes(node, params)     # a declaration is not a change (B-145)
        if not params:
            return SetParametersResult(successful=True)
        fixed = [p.name for p in params if p.name not in live]
        if fixed:
            return SetParametersResult(successful=False, reason=(
                f'{", ".join(fixed)}: read once at startup -- {restart_hint}. '
                f'Live here: {", ".join(node.LIVE_PARAMS) or "none"}'))
        done = []
        try:
            for p in params:
                live[p.name](p.value)
                done.append(p.name)
        except ValueError as exc:
            # The reply says nothing changed, so nothing may have: put back
            # what this batch already applied, from the values still stored.
            for name in reversed(done):
                try:
                    live[name](node.get_parameter(name).value)
                except ValueError:
                    pass
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(on_set)
