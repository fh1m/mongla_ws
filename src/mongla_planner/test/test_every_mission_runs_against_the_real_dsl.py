"""Run every mission end to end against the REAL DSL's method signatures.

The other mission tests read a mission's AST or run it against a recorder that
accepts any call. Neither notices `mongla.lock_heading(target=0.0)` when the
method is `lock_heading(self, degrees=0.0, *, timeout=...)`: the mission
imports, registers, lists -- and dies with a TypeError at that line, in the
water, on the day.

So here every call a mission makes is bound against the signature it would
really reach:

  * a `MonglaMission` method or property         -> that method's signature
  * a verb that falls through to the client      -> keyword-only, and every
    (`mongla.unlock_heading()` and friends)         keyword a real Move.Goal field
  * `mongla.vision.*`                             -> `_VisionDSL`'s signatures
  * anything else                                 -> AttributeError, as for real

Each mission is run twice: once where every detection, verb and query
succeeds, once where nothing is ever seen. Missions loop on time, so a call
budget ends a run that is still going; reaching it is a pass. Any other
exception is a defect in the mission.

What this does NOT prove: that a mission flies. It proves the mission can be
executed to its end by the code it will call.
"""
from __future__ import annotations

import inspect
import time

import pytest

from mongla_control import COMMANDS
from mongla_planner.missions import discover
from mongla_planner import mongla_dsl, vision_dsl

BUDGET = 600


class _Budget(BaseException):
    """BaseException so a mission's `except Exception` cannot swallow it."""


class _Any:
    """A return value that behaves like whatever the mission expects of it:
    a truthy/falsy result, a number, a detection, an object with fields."""

    def __init__(self, truth: bool):
        self._truth = truth

    def __bool__(self):
        return self._truth

    def __float__(self):
        return 0.0

    def __int__(self):
        return 0

    __index__ = __int__

    def __format__(self, spec):
        return format(0.0, spec) if spec else repr(self)

    def __repr__(self):
        return f'<any {self._truth}>'

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        if name == 'success':
            return self._truth
        if name in ('message', 'reason', 'class_name', 'name'):
            return ''
        return _Any(self._truth)

    def __call__(self, *a, **k):
        return _Any(self._truth)

    def __iter__(self):
        return iter(())

    def __enter__(self):                     # `with mongla.task(...):`
        return self

    def __exit__(self, *exc):
        return False

    def __len__(self):
        return 0

    def __getitem__(self, key):
        return _Any(self._truth)

    def __contains__(self, item):
        return self._truth

    def _num(self, *a):
        return 0.0

    __add__ = __radd__ = __sub__ = __rsub__ = __mul__ = __rmul__ = _num
    __truediv__ = __rtruediv__ = __neg__ = __abs__ = __round__ = _num

    def __lt__(self, o):
        return False

    __le__ = __gt__ = __ge__ = __lt__

    def __eq__(self, o):
        return o is self

    __hash__ = object.__hash__


def _goal_fields() -> set[str] | None:
    try:
        from mongla_interfaces.action import Move
    except Exception:                                  # noqa: BLE001
        return None
    return set(Move.Goal.get_fields_and_field_types())


class _Strict:
    """Proxy for an object of class `cls`: each call is bound against the real
    signature before anything is returned."""

    def __init__(self, cls, run, label):
        self._cls, self._run, self._label = cls, run, label

    def __getattr__(self, name):
        member = inspect.getattr_static(self._cls, name, None)
        if member is None:
            raise AttributeError(f'{self._label} has no attribute {name!r}')
        if isinstance(member, property):
            self._run.tick(f'{self._label}.{name}')
            return _Any(self._run.truth)
        fn = getattr(self._cls, name)
        if not callable(fn):
            return fn
        sig = inspect.signature(fn)

        def call(*a, **k):
            self._run.tick(f'{self._label}.{name}')
            sig.bind(None, *a, **k)                    # TypeError on a bad call
            return _Any(self._run.truth)
        return call


class _Mission(_Strict):
    """Stands in for MonglaMission."""

    def __init__(self, run, backend):
        super().__init__(mongla_dsl.MonglaMission, run, 'mongla')
        self.backend = backend
        self.vision = _Strict(vision_dsl._VisionDSL, run, 'mongla.vision')
        self.models = _Any(True)
        self.client = _Any(True)
        self.log = _Any(True)
        self.camera = 'forward'
        self.target = ''

    def can(self, verb):
        self._run.tick('mongla.can')
        from mongla_control.fc.srot_fc import UNSUPPORTED_VERBS
        return self.backend != 'srot' or verb not in UNSUPPORTED_VERBS

    def __getattr__(self, name):
        if inspect.getattr_static(self._cls, name, None) is not None:
            return super().__getattr__(name)
        if name in COMMANDS:                           # falls through to client
            fields = self._run.goal_fields

            def verb(*a, **k):
                self._run.tick(f'mongla.{name}')
                if a:
                    raise TypeError(f'{name}() falls through to the client, '
                                    f'which takes keywords only; got {a!r}')
                bad = set(k) - fields if fields is not None else set()
                if bad:
                    raise TypeError(f'{name}(): {sorted(bad)} is not a Move.Goal field')
                return _Any(self._run.truth)
            return verb
        raise AttributeError(f'MonglaMission has no attribute {name!r}')


class _Run:
    def __init__(self, truth):
        self.truth = truth
        self.calls = 0
        self.goal_fields = _goal_fields()

    def tick(self, what):
        self.calls += 1
        if self.calls == BUDGET:
            raise _Budget(what)


MISSIONS = sorted(discover().items())


@pytest.mark.parametrize('backend', ['srot', 'pixhawk'])
@pytest.mark.parametrize('truth', [True, False], ids=['all_seen', 'nothing_seen'])
@pytest.mark.parametrize('name,run', MISSIONS, ids=[n for n, _ in MISSIONS])
def test_mission_runs_to_its_end_against_real_signatures(name, run, truth, backend,
                                                         monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda s: None)
    r = _Run(truth)
    mongla = _Mission(r, backend)
    try:
        run(mongla, _Any(True))
    except _Budget:
        pass
    except (TypeError, AttributeError, NameError, KeyError) as exc:
        pytest.fail(f'{name} ({backend}, {"all seen" if truth else "nothing seen"}): '
                    f'{type(exc).__name__}: {exc}')
    except Exception as exc:                           # noqa: BLE001
        # A mission is allowed to refuse ITSELF (MissionRefused, a deliberate
        # RuntimeError on a missing prerequisite) -- that is "refuse loudly".
        # Anything else is reported with its type so it can be judged.
        # (AssertionError is how task_torpedo refuses an unmeasured depth; it
        # works because ros2 run never passes -O, which would strip it.)
        if type(exc).__name__ not in ('MissionRefused', 'TaskAbandoned',
                                      'RuntimeError', 'ValueError',
                                      'AssertionError'):
            pytest.fail(f'{name}: {type(exc).__name__}: {exc}')


def test_the_stand_in_catches_the_defect_it_exists_for():
    """Injection: a call the real DSL would reject must fail here."""
    r = _Run(True)
    m = _Mission(r, 'srot')
    with pytest.raises(TypeError):
        m.lock_heading(target=0.0, timeout=1)          # real kwarg is `degrees`
    with pytest.raises(AttributeError):
        m.no_such_verb()
    with pytest.raises(TypeError):
        m.vision.align('gate', no_such_kw=1)
