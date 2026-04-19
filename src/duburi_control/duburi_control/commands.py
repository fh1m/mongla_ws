"""Single source of truth for every /duburi/move command.

This is the ONE place that knows what commands exist, what fields each
one reads from `Move.Goal`, what the defaults are, and what to print in
`--help`. The action server, the Python client, and the `duburi` CLI
all read from here — adding a new command is two edits, no more:

    1. Add a row to COMMANDS below.
    2. Add a method on `Duburi` whose name and parameter names match
       the row's `cmd` and `fields`.

Field names map directly to fields on `duburi_interfaces.action.Move`
(`duration`, `gain`, `target`, `target_name`, `timeout`). The dispatch
just does `getattr(duburi, cmd)(**kwargs_built_from_fields)`.

`defaults` covers the case where the operator omits a field. Any field
NOT listed in `defaults` is required — the CLI marks it `required=True`,
and the action server uses whatever the caller put on the goal (which
rosidl zero-initialises, so the operator must supply it explicitly).
"""


COMMANDS = {
    'arm': {
        'help':     'Arm the vehicle (motors hot).',
        'fields':   ['timeout'],
        'defaults': {'timeout': 15.0},
    },
    'disarm': {
        'help':     'Disarm safely (mode -> MANUAL, neutral, then disarm).',
        'fields':   ['timeout'],
        'defaults': {'timeout': 20.0},
    },
    'set_mode': {
        'help':     'Switch ArduSub flight mode (MANUAL, ALT_HOLD, STABILIZE, ...).',
        'fields':   ['target_name', 'timeout'],
        'defaults': {'timeout': 8.0},
    },
    'stop': {
        'help':     'Active hold: send neutral 1500 PWM to all six channels.',
        'fields':   [],
        'defaults': {},
    },
    'pause': {
        'help':     'Release RC override for N seconds (autopilot takes over).',
        'fields':   ['duration'],
        'defaults': {'duration': 2.0},
    },
    'move_forward': {
        'help':     'Drive forward for `duration` seconds at `gain` percent thrust.',
        'fields':   ['duration', 'gain'],
        'defaults': {'gain': 80.0},
    },
    'move_back': {
        'help':     'Drive backward for `duration` seconds at `gain` percent thrust.',
        'fields':   ['duration', 'gain'],
        'defaults': {'gain': 80.0},
    },
    'move_left': {
        'help':     'Strafe left for `duration` seconds at `gain` percent thrust.',
        'fields':   ['duration', 'gain'],
        'defaults': {'gain': 80.0},
    },
    'move_right': {
        'help':     'Strafe right for `duration` seconds at `gain` percent thrust.',
        'fields':   ['duration', 'gain'],
        'defaults': {'gain': 80.0},
    },
    'yaw_left': {
        'help':     'Turn left by `target` degrees within `timeout` seconds.',
        'fields':   ['target', 'timeout'],
        'defaults': {'timeout': 30.0},
    },
    'yaw_right': {
        'help':     'Turn right by `target` degrees within `timeout` seconds.',
        'fields':   ['target', 'timeout'],
        'defaults': {'timeout': 30.0},
    },
    'set_depth': {
        'help':     'Hold absolute depth (`target` metres, negative below surface).',
        'fields':   ['target', 'timeout'],
        'defaults': {'timeout': 30.0},
    },
}


# Field names that carry a string instead of a float (everything else is float).
STRING_FIELDS = ('target_name',)


def fields_for(cmd, request):
    """Pull the kwargs for `cmd` out of a Move.Goal `request`.

    For each field the spec lists, take the value from the request. If
    the value is the rosidl "unset" default (0.0 for floats, '' for
    strings) AND the spec provides a default, substitute the default.

    The returned dict can be passed straight to the matching `Duburi`
    method as `**kwargs` — the dispatch never has to know what each
    command is doing.
    """
    spec = COMMANDS[cmd]
    defaults = spec['defaults']
    kwargs = {}
    for field in spec['fields']:
        value = getattr(request, field)
        unset = (value == '' if field in STRING_FIELDS else value == 0.0)
        if unset and field in defaults:
            value = defaults[field]
        kwargs[field] = value
    return kwargs
