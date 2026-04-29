"""model_context -- typed model/class handles for the mission DSL.

Usage in a mission::

    def run(duburi, log):
        m = duburi.models(
            gate    = ('gate_flare_medium_100ep', ['gate', 'flare']),
            slalom  = ('slalom_combined',          ['slalom_red', 'slalom_white']),
        )

        # Access classes by name attribute:
        duburi.vision.find(target=m.gate.gate,   move='forward', gain=35)
        duburi.vision.home(target=m.gate.gate,   yaw=True, lat=True)

        # Access classes by index:
        duburi.vision.turn(target=m.slalom[0])   # slalom_red
        duburi.vision.turn(target=m.slalom[1])   # slalom_white

When a `ClassRef` is passed to any vision verb, the DSL automatically calls
`set_model(model_name)` and `set_classes(class_name)` before sending the
goal — no manual `duburi.use(...)` needed per verb.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ClassRef:
    """Reference to one YOLO class within a named model registry entry.

    Returned by `ModelHandle.class_name` attribute access or index access.
    Passed to any vision verb as `target=` to auto-switch the detector.
    """
    model_name: str   # registry key used in models:="name=stem,..." launch arg
    class_name: str   # YOLO class label string

    def __str__(self) -> str:
        return self.class_name

    def __repr__(self) -> str:
        return f'ClassRef({self.model_name!r}/{self.class_name!r})'


class ModelHandle:
    """Handle for one entry in the detector registry.

    Attribute access returns a ClassRef by class name.
    Index access returns a ClassRef by position in the classes list.

    This uses `object.__setattr__` / `object.__getattribute__` throughout
    so that any class name can be used as an attribute without clashing with
    Python dunder infrastructure.
    """

    def __init__(self, name: str, stem: str, classes: list[str]):
        object.__setattr__(self, '_name',    name)
        object.__setattr__(self, '_stem',    stem)
        object.__setattr__(self, '_classes', list(classes))

    def __getattr__(self, item: str) -> ClassRef:
        classes = object.__getattribute__(self, '_classes')
        name    = object.__getattribute__(self, '_name')
        if item in classes:
            return ClassRef(name, item)
        raise AttributeError(
            f"Model {name!r} has no class {item!r}. "
            f"Available: {classes}")

    def __getitem__(self, idx: int) -> ClassRef:
        classes = object.__getattribute__(self, '_classes')
        name    = object.__getattribute__(self, '_name')
        try:
            return ClassRef(name, classes[idx])
        except IndexError:
            raise IndexError(
                f"Model {name!r} has {len(classes)} classes; "
                f"index {idx} is out of range. "
                f"Classes: {classes}")

    def __repr__(self) -> str:
        classes = object.__getattribute__(self, '_classes')
        name    = object.__getattribute__(self, '_name')
        stem    = object.__getattribute__(self, '_stem')
        return f'ModelHandle({name!r}, stem={stem!r}, classes={classes})'


class ModelRegistry:
    """Collection of ModelHandle objects, indexed by alias.

    Returned by `DuburiMission.models(**kwargs)`.
    Access handles as attributes: ``m.gate``, ``m.slalom``.
    """

    def __init__(self, handles: dict[str, ModelHandle]):
        for k, v in handles.items():
            object.__setattr__(self, k, v)

    def __repr__(self) -> str:
        keys = list(object.__getattribute__(self, '__dict__').keys())
        return f'ModelRegistry({keys})'
