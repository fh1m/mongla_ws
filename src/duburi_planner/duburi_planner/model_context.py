"""model_context -- typed model/class handles for the mission DSL.

Usage in a mission::

    def run(duburi, log):
        duburi.models(
            gate   = 'gate_flare_medium_100ep',
            slalom = 'slalom_combined',
        )

        # Access classes by name — any name is valid (validated at detector):
        duburi.vision.find(target=duburi.models.gate.gate,  move='forward', gain=35)
        duburi.vision.home(target=duburi.models.gate.flare, yaw=True, depth=True)
        duburi.vision.turn(target=duburi.models.slalom.slalom_red)

        # Explicit class list validates at handle creation (optional):
        duburi.models(strict=('slalom_combined', ['slalom_red', 'slalom_white']))
        duburi.vision.turn(target=duburi.models.strict.slalom_red)   # validated
        # duburi.vision.turn(target=duburi.models.strict.typo)        # → AttributeError

When a `ClassRef` is passed to any vision verb, the DSL automatically calls
`set_model(model_name)` and `set_classes(class_name)` before sending the
goal — no manual `duburi.use(...)` or `duburi.set_classes(...)` needed.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ClassRef:
    """Reference to one YOLO class within a named model registry entry.

    Returned by `ModelHandle.class_name` attribute access or index access.
    Passed to any vision verb as `target=` to auto-switch the detector.
    """
    model_name: str   # registry key used when registering with duburi.models()
    class_name: str   # YOLO class label string

    def __str__(self) -> str:
        return self.class_name

    def __repr__(self) -> str:
        return f'ClassRef({self.model_name!r}/{self.class_name!r})'


class ModelHandle:
    """Handle for one entry in the detector registry.

    Attribute access returns a ClassRef by class name.
    If the handle was created with an explicit classes list, access is
    validated against that list. Otherwise any name is accepted (validation
    is deferred to the detector node).

    Index access (``handle[0]``) is only supported when an explicit classes
    list was provided at registration time.
    """

    def __init__(self, name: str, stem: str, classes: list[str]):
        object.__setattr__(self, '_name',    name)
        object.__setattr__(self, '_stem',    stem)
        object.__setattr__(self, '_classes', list(classes))

    def __getattr__(self, item: str) -> ClassRef:
        classes = object.__getattribute__(self, '_classes')
        name    = object.__getattribute__(self, '_name')
        if not classes or item in classes:
            return ClassRef(name, item)
        raise AttributeError(
            f"Model {name!r} has no class {item!r}. "
            f"Available: {classes}  "
            f"(register without a classes list to skip validation)")

    def __getitem__(self, idx: int) -> ClassRef:
        classes = object.__getattribute__(self, '_classes')
        name    = object.__getattribute__(self, '_name')
        if not classes:
            raise IndexError(
                f"Model {name!r} was registered without a classes list; "
                f"use attribute access: duburi.models.{name}.class_name")
        try:
            return ClassRef(name, classes[idx])
        except IndexError:
            raise IndexError(
                f"Model {name!r} has {len(classes)} classes; "
                f"index {idx} is out of range. Classes: {classes}")

    def __repr__(self) -> str:
        classes = object.__getattribute__(self, '_classes')
        name    = object.__getattribute__(self, '_name')
        stem    = object.__getattribute__(self, '_stem')
        return f'ModelHandle({name!r}, stem={stem!r}, classes={classes or "any"})'


class ModelRegistry:
    """Callable singleton attached to ``DuburiMission`` as ``duburi.models``.

    Register models by calling it; access handles as attributes.

    Registration::

        duburi.models(gate='gate_flare_medium_100ep')
        duburi.models(gate='gate_flare_medium_100ep',
                      slalom='slalom_combined')

    Access::

        duburi.models.gate.gate     # ClassRef('gate', 'gate')
        duburi.models.gate.flare    # ClassRef('gate', 'flare')

    Optional strict class list (validates at access time)::

        duburi.models(gate=('gate_flare_medium_100ep', ['gate', 'flare']))
        # duburi.models.gate.typo  → AttributeError

    Multiple calls accumulate (later registrations overwrite same alias).
    Returns self so ``m = duburi.models(...)`` still works if preferred.
    """

    def __call__(self, **kwargs) -> 'ModelRegistry':
        """Register one or more model aliases into this registry.

        Parameters
        ----------
        **kwargs : alias = spec
            spec may be:

            str
                Model stem only.  Any class name is accepted at access time;
                validation is deferred to the detector node.
            (stem, [class, ...])
                Stem plus explicit class list.  Attribute access is validated
                against the list immediately (``AttributeError`` on unknown name).

        Returns self so the call can be chained or assigned::

            duburi.models(gate='stem')          # register in-place
            m = duburi.models(gate='stem')      # m is duburi.models (same object)
        """
        for alias, spec in kwargs.items():
            if isinstance(spec, (list, tuple)) and len(spec) == 2:
                stem, classes = spec
            else:
                stem    = str(spec)
                classes = []
            handle = ModelHandle(alias, str(stem), list(classes))
            object.__setattr__(self, alias, handle)
        return self

    def __getattr__(self, item: str):
        raise AttributeError(
            f"No model registered under alias {item!r}. "
            f"Call duburi.models({item}='model_stem') first.")

    def __repr__(self) -> str:
        keys = [k for k in object.__getattribute__(self, '__dict__')
                if not k.startswith('_')]
        return f'ModelRegistry({keys})'
