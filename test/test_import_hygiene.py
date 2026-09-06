"""The suite must be green ABOUT THE FILES IN THIS CHECKOUT.

Sourcing a colcon workspace puts its `install/` tree on PYTHONPATH, so
`import duburi_vision` resolves there rather than to the source beside the
test. That has cost this project several rounds, always silently:

  * round 34 chased a "drift bug" that was a test reading 0.4 from an
    installed copy while its source said 0.20;
  * a method added to `flow_math` was reported ABSENT by pytest while grep
    found it in the file -- in a worktree with no install tree of its own, so
    the import came from a DIFFERENT workspace entirely;
  * on the Pi, `colcon build --symlink-install` copies rather than symlinks
    for this layout, so the vehicle's suite validates code the vehicle is not
    running.

The root `conftest.py` fixes it. These assert the fix is in force, because a
path fix that stops working fails exactly the way the original bug did: green,
and about the wrong file.
"""
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'

SOURCE_PACKAGES = sorted(
    d.name for d in SRC.iterdir()
    if d.is_dir() and (d / d.name).is_dir()
) if SRC.is_dir() else []


def test_there_are_source_packages_to_check():
    """Without this the parametrised tests below vacuously pass."""
    assert SOURCE_PACKAGES, f'no python source packages under {SRC}'


@pytest.mark.parametrize('name', SOURCE_PACKAGES)
def test_the_package_imports_from_THIS_checkout(name):
    mod = __import__(name)
    where = pathlib.Path(getattr(mod, '__file__', '') or '').resolve()
    assert ROOT in where.parents, (
        f'{name} imported from {where}, which is OUTSIDE this checkout '
        f'({ROOT}). A colcon install tree is shadowing the source -- the '
        f'suite would be green about the wrong file. See conftest.py.')


@pytest.mark.parametrize('name', SOURCE_PACKAGES)
def test_the_package_is_not_served_from_an_install_tree(name):
    """Stricter and separately useful: even inside this checkout, `install/`
    and `build/` are colcon artefacts and go stale the moment source moves."""
    mod = __import__(name)
    where = str(pathlib.Path(getattr(mod, '__file__', '') or '').resolve())
    assert '/install/' not in where and '/build/' not in where, where


def test_duburi_interfaces_is_still_reachable():
    """The exemption that makes the rest safe. `duburi_interfaces` is
    GENERATED -- there is no `src/duburi_interfaces/duburi_interfaces/` -- so
    it must keep coming from the install tree. A conftest that purged it
    would break `from duburi_interfaces.msg import DuburiState` in every node
    test, which is why the rule is 'shadow only what we have source for'."""
    pytest.importorskip(
        'duburi_interfaces',
        reason='workspace not built here; nothing to shadow either')
    from duburi_interfaces.msg import DuburiState  # noqa: F401


def test_no_install_shadow_survives_on_sys_path():
    """The eviction itself, asserted directly. A source dir arriving at the
    front of sys.path is not enough if a shadow is also present: an already
    imported module keeps its binding, which is the case conftest's third
    step handles."""
    bad = [p for p in sys.path
           if ('/install/' in str(p) or '/build/' in str(p))
           and any(f'/{n}/' in str(p) for n in SOURCE_PACKAGES)]
    assert not bad, 'colcon shadows still on sys.path:\n  ' + '\n  '.join(bad)
