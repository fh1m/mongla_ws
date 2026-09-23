"""Let the node tests run without a built `mongla_interfaces`.

⛔ WHY. `localization_node` subscribes to `/mongla/state`, so it imports
`MonglaState` at module scope. On a dev box `mongla_interfaces` is not built,
so `test_localization_node.py` could not even be COLLECTED -- roughly fifty node
tests that only ever ran on the vehicle, which is the opposite of what a unit
test is for.

⚠ THE STUB IS INSTALLED ONLY IF THE REAL PACKAGE IS ABSENT. On the vehicle, and
in any built workspace, the generated message wins and nothing here runs.

⛔ AND IT IS PARSED FROM THE `.msg` / `.action` FILES, NOT HAND-WRITTEN. The
first version was a bare attribute bag, and it broke a `mongla_control` test
that legitimately introspects `Move.Goal.get_fields_and_field_types()` -- turning
a clean "module not found" into a confusing AttributeError, which is worse than
what it replaced. `sys.modules` is global, so no amount of conftest scoping
contains that.

Reading the interface definitions instead means the stub cannot drift from them:
it IS them. A field added to `Move.action` appears here on the next run with no
edit, and a test that asserts the field list keeps working on a dev box.
"""
import sys
import types
from pathlib import Path

_IFACE = Path(__file__).resolve().parents[2] / 'mongla_interfaces'


def _parse_fields(text: str) -> dict:
    """`name -> type` for one interface section. Comments and blanks dropped."""
    out = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            out[parts[1]] = parts[0]
    return out


def _sections(path: Path) -> list:
    """An .action is Goal---Result---Feedback; a .msg is one section."""
    if not path.exists():
        return [{}]
    return [_parse_fields(chunk) for chunk in path.read_text().split('---')]


def _make(name: str, fields: dict):
    """A message double that answers the introspection rosidl provides."""
    def get_fields_and_field_types(self=None, _f=dict(fields)):
        return dict(_f)

    ns = {'_IS_TEST_STUB': True,
          'get_fields_and_field_types': staticmethod(get_fields_and_field_types)}
    for fname in fields:
        ns[fname] = None                      # so `Msg().field` does not raise
    return type(name, (), ns)


def _install_stub() -> None:
    try:
        import mongla_interfaces.msg  # noqa: F401
        return                        # the real thing is built -- use it
    except Exception:                 # noqa: BLE001
        pass

    pkg = types.ModuleType('mongla_interfaces')
    # ⚠ `__path__` MATTERS. Without it Python refuses `mongla_interfaces.action`
    # with "not a package", which breaks collection of every sibling test that
    # reaches the action -- a stub that fixes one import and breaks another.
    pkg.__path__ = []
    msg = types.ModuleType('mongla_interfaces.msg')
    action = types.ModuleType('mongla_interfaces.action')

    for f in sorted((_IFACE / 'msg').glob('*.msg')):
        setattr(msg, f.stem, _make(f.stem, _sections(f)[0]))

    for f in sorted((_IFACE / 'action').glob('*.action')):
        parts = _sections(f)
        top = _make(f.stem, {})
        for label, fields in zip(('Goal', 'Result', 'Feedback'), parts):
            setattr(top, label, _make(f'{f.stem}_{label}', fields))
        setattr(action, f.stem, top)

    pkg.msg = msg
    pkg.action = action
    sys.modules.setdefault('mongla_interfaces', pkg)
    sys.modules.setdefault('mongla_interfaces.msg', msg)
    sys.modules.setdefault('mongla_interfaces.action', action)


_install_stub()
